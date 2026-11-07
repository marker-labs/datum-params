#include <algorithm>
// Simple ROS action server: interpolated joints only, with no contact physics.
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using Action = control_msgs::action::FollowJointTrajectory;
using Handle = rclcpp_action::ServerGoalHandle<Action>;
using namespace std::chrono_literals;

class MotorSim : public rclcpp::Node {
 public:
  MotorSim() : Node("dparams_cpp_motor_sim") {
    publisher_=create_publisher<sensor_msgs::msg::JointState>("/joint_states",10);
    timer_=create_wall_timer(50ms,[this]{publish();});
    server_=rclcpp_action::create_server<Action>(this,"/arm_controller/follow_joint_trajectory",
      [this](const rclcpp_action::GoalUUID&,std::shared_ptr<const Action::Goal> goal){
        if(busy_.exchange(true)) return rclcpp_action::GoalResponse::REJECT;
        const auto& t=goal->trajectory;
        if(t.joint_names!=names_||t.points.size()!=1||t.points[0].positions.size()!=names_.size()) {busy_=false;return rclcpp_action::GoalResponse::REJECT;}
        const auto& point=t.points[0]; const double duration=point.time_from_start.sec+point.time_from_start.nanosec/1e9;
        std::lock_guard<std::mutex> guard(mutex_);
        constexpr std::array<double,6> lo={-3.14,-1.57,-2.,-1.57,0.,0.},hi={3.14,1.57,2.,1.57,.02,.02},vel={1.,1.,1.,1.,.03,.03};
        bool valid=duration>0 && std::isfinite(duration) && std::abs(point.positions[4]-point.positions[5])<1e-9;
        for(size_t i=0;i<names_.size();++i) valid=valid&&std::isfinite(point.positions[i])&&point.positions[i]>=lo[i]&&point.positions[i]<=hi[i]&&std::abs(point.positions[i]-positions_[i])<=vel[i]*duration+1e-9;
        if(!valid){busy_=false;return rclcpp_action::GoalResponse::REJECT;}
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<Handle>){return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const std::shared_ptr<Handle> handle){auto self=std::static_pointer_cast<MotorSim>(shared_from_this());std::thread([self,handle]{self->execute(handle);}).detach();});
  }
 private:
  void publish() {
    sensor_msgs::msg::JointState msg;msg.header.stamp=now();msg.name=names_;
    {std::lock_guard<std::mutex> guard(mutex_);msg.position=positions_;}
    publisher_->publish(msg);
  }
  void execute(const std::shared_ptr<Handle>& handle) {
    const auto& p=handle->get_goal()->trajectory.points[0];const double duration=p.time_from_start.sec+p.time_from_start.nanosec/1e9;
    std::vector<double> start;{std::lock_guard<std::mutex> guard(mutex_);start=positions_;}
    const auto begin=std::chrono::steady_clock::now();
    while(rclcpp::ok()){
      if(handle->is_canceling()){handle->canceled(std::make_shared<Action::Result>());busy_=false;return;}
      const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();const double fraction=std::min(1.,elapsed/duration);
      {std::lock_guard<std::mutex> guard(mutex_);for(size_t i=0;i<positions_.size();++i)positions_[i]=start[i]+fraction*(p.positions[i]-start[i]);}
      if(fraction>=1.)break;
      std::this_thread::sleep_for(20ms);
    }
    if (!rclcpp::ok()) { busy_=false; return; }
    auto result=std::make_shared<Action::Result>();result->error_code=Action::Result::SUCCESSFUL;handle->succeed(result);busy_=false;
  }
  const std::vector<std::string> names_={"base_yaw","shoulder","elbow","wrist","left_finger","right_finger"};
  std::vector<double> positions_=std::vector<double>(6,0.0);std::mutex mutex_;std::atomic<bool> busy_{false};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;rclcpp_action::Server<Action>::SharedPtr server_;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<MotorSim>());rclcpp::shutdown();}
