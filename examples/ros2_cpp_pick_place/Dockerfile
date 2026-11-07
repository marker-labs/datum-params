FROM ros:jazzy-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends ros-jazzy-control-msgs \
    && rm -rf /var/lib/apt/lists/*
SHELL ["/bin/bash", "-c"]
WORKDIR /workspace
