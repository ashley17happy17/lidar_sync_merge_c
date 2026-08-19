# 使用官方 ROS Noetic 包含桌面環境與 RViz 的基礎映像檔
FROM osrf/ros:noetic-desktop-full

# 避免安裝過程中的互動式時區/鍵盤提示卡住
ENV DEBIAN_FRONTEND=noninteractive

# 安裝編譯工具以及 PCL/Eigen 開發套件
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    vim \
    tmux \
    libpcl-dev \
    libeigen3-dev \
    libyaml-cpp-dev \
    libpdal-dev \
    pdal \
    && rm -rf /var/lib/apt/lists/*

# 設定預設工作目錄 (專案原始碼掛載於此)
WORKDIR /root/ws/src

# 基礎映像檔的 ROS 環境僅用於 rviz / pcl 等視覺化工具，建置本專案不需要 ROS
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# 預設啟動 bash
CMD ["/bin/bash"]