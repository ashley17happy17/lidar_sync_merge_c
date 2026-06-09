# 使用官方 ROS Noetic 包含桌面環境與 RViz 的基礎映像檔
FROM osrf/ros:noetic-desktop-full

# 避免安裝過程中的互動式時區/鍵盤提示卡住
ENV DEBIAN_FRONTEND=noninteractive

# 安裝編譯工具、Catkin 工具以及 PCL/Eigen 開發套件
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    vim \
    tmux \
    python3-catkin-tools \
    python3-osrf-pycommon \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    libpcl-dev \
    libeigen3-dev \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# 設定預設工作目錄
WORKDIR /root/catkin_ws

# 將 ROS 環境變數自動加入 bashrc，這樣每次進 container 都不用手動 source
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /root/catkin_ws/devel/setup.bash ]; then source /root/catkin_ws/devel/setup.bash; fi" >> ~/.bashrc

# 預設啟動 bash
CMD ["/bin/bash"]