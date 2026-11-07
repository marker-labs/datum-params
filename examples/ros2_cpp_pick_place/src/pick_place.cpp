#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "parameters.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using Action = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
namespace params = datum::robot::main;
using namespace std::chrono_literals;

class PickPlace : public rclcpp::Node {
 public:
  PickPlace() : Node("dparams_cpp_pick_place") {
    server_timeout_seconds_ = declare_parameter<double>("action_server_timeout_sec", 10.0);
    if (!std::isfinite(server_timeout_seconds_) || server_timeout_seconds_ <= 0.0)
      throw std::invalid_argument("action_server_timeout_sec must be positive and finite");
    client_ = rclcpp_action::create_client<Action>(this, "/arm_controller/follow_joint_trajectory");
    target_ = create_subscription<geometry_msgs::msg::PoseStamped>("/pick_target", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { on_target(*msg); });
    joints_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        for (size_t i=0; i<msg->name.size() && i<msg->position.size(); ++i) {
          if (msg->name[i]=="left_finger") left_=msg->position[i];
          if (msg->name[i]=="right_finger") right_=msg->position[i];
        }
      });
    timer_ = create_wall_timer(200ms, [this] { tick(); });
    RCLCPP_INFO(get_logger(), "Generated tool length %.4f m; waiting for /pick_target in base_link", tool());
  }
 private:
  static double tool() { return params::forearm_length_m + params::gripper_palm_length_m + params::finger_length_m / 2; }
  static std::vector<double> ik(double x,double y,double z,double grip) {
    const double dz=z-params::shoulder_height_m, radius=std::hypot(x,y), l1=params::upper_arm_length_m, l2=tool();
    if (!(std::isfinite(l1) && std::isfinite(l2) && l1 > 0 && l2 > 0)) throw std::runtime_error("invalid generated link dimensions");
    const double c=(radius*radius+dz*dz-l1*l1-l2*l2)/(2*l1*l2);
    if (!std::isfinite(c) || c < -1 || c > 1) throw std::runtime_error("target outside reach");
    const double elbow=-std::acos(c), shoulder=std::atan2(dz,radius)-std::atan2(l2*std::sin(elbow),l1+l2*std::cos(elbow));
    std::vector<double> q={std::atan2(y,x),-shoulder,-elbow,0,grip,grip};
    constexpr std::array<double,6> lo={-3.14,-1.57,-2.,-1.57,0.,0.}, hi={3.14,1.57,2.,1.57,.02,.02};
    for(size_t i=0;i<q.size();++i) if(!std::isfinite(q[i])||q[i]<lo[i]||q[i]>hi[i]) throw std::runtime_error("joint limit exceeded");
    return q;
  }
  void on_target(const geometry_msgs::msg::PoseStamped& msg) {
    if (busy_ || done_) return;
    if (msg.header.frame_id != "base_link") { RCLCPP_ERROR(get_logger(),"Target must use base_link; TF conversion is not provided by this example"); return; }
    try {
      const auto& p=msg.pose.position; const double z=p.z-.015, open=.018;
      std::deque<std::pair<std::string,std::vector<double>>> proposed;
      proposed.emplace_back("APPROACH",ik(p.x,p.y,z+.05,open));
      proposed.emplace_back("DESCEND",ik(p.x,p.y,z,open));
      proposed.emplace_back("GRASP",ik(p.x,p.y,z,0));
      proposed.emplace_back("LIFT",ik(p.x,p.y,z+.05,0));
      proposed.emplace_back("TRANSFER",ik(.24,-.12,.185,0));
      proposed.emplace_back("RELEASE",ik(.24,-.12,.185,open));
      steps_=std::move(proposed); busy_=true;
      server_wait_started_=std::chrono::steady_clock::now();
      if (!client_->action_server_is_ready())
        RCLCPP_INFO(get_logger(), "Waiting up to %.1f seconds for /arm_controller/follow_joint_trajectory", server_timeout_seconds_);
    } catch(const std::exception& e) { RCLCPP_ERROR(get_logger(),"Target rejected: %s",e.what()); }
  }
  void tick() {
    if(!busy_ || in_flight_) return;
    if(steps_.empty()) { busy_=false; done_=true; RCLCPP_INFO(get_logger(),"Pick and place sequence complete"); return; }
    if(!client_->action_server_is_ready()) {
      const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-server_wait_started_).count();
      if(elapsed >= server_timeout_seconds_) {
        RCLCPP_ERROR(get_logger(), "Action server unavailable after %.1f seconds; target cancelled. Publish a new target to retry.", server_timeout_seconds_);
        steps_.clear();busy_=false;
      }
      return;
    }
    auto step=steps_.front();steps_.pop_front();in_flight_=true;
    Action::Goal goal; goal.trajectory.joint_names={"base_yaw","shoulder","elbow","wrist","left_finger","right_finger"};
    trajectory_msgs::msg::JointTrajectoryPoint point;point.positions=step.second;point.time_from_start.sec=2;goal.trajectory.points.push_back(point);
    RCLCPP_INFO(get_logger(),"State: %s",step.first.c_str());
    rclcpp_action::Client<Action>::SendGoalOptions options;
    options.goal_response_callback=[this](GoalHandle::SharedPtr handle) { if(!handle){RCLCPP_ERROR(get_logger(),"Trajectory rejected");steps_.clear();busy_=false;done_=true;in_flight_=false;} };
    options.result_callback=[this,label=step.first](const GoalHandle::WrappedResult& result) {
      in_flight_=false;
      server_wait_started_=std::chrono::steady_clock::now();
      if(result.code!=rclcpp_action::ResultCode::SUCCEEDED || !result.result || result.result->error_code!=Action::Result::SUCCESSFUL) {
        RCLCPP_ERROR(get_logger(),"Trajectory failed at %s",label.c_str());steps_.clear();busy_=false;done_=true;return;
      }
      if(label=="GRASP" && (!std::isfinite(left_) || !std::isfinite(right_) || left_>.014 || right_>.014)) {
        RCLCPP_ERROR(get_logger(),"Gripper feedback did not confirm closure; refusing to lift");steps_.clear();busy_=false;done_=true;
      }
    };
    client_->async_send_goal(goal,options);
  }
  rclcpp_action::Client<Action>::SharedPtr client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::deque<std::pair<std::string,std::vector<double>>> steps_;
  std::chrono::steady_clock::time_point server_wait_started_;
  double server_timeout_seconds_=10.0;
  double left_=NAN,right_=NAN;
  bool busy_=false,done_=false,in_flight_=false;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<PickPlace>());rclcpp::shutdown();}
