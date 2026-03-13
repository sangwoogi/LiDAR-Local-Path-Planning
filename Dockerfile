# 1. Base Image: Ubuntu 22.04 + CUDA 12.4.1 (sm_120 완벽 지원)
FROM nvidia/cuda:12.4.1-devel-ubuntu22.04

# 2. 환경 변수 설정
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# 3. apt 네트워크 파이프라인 병목 현상 및 캐시 충돌 방지 설정
RUN echo 'Acquire::http::Pipeline-Depth "0";' > /etc/apt/apt.conf.d/99custom && \
    echo 'Acquire::http::No-Cache "true";' >> /etc/apt/apt.conf.d/99custom && \
    echo 'Acquire::BrokenProxy "true";' >> /etc/apt/apt.conf.d/99custom

# 4. 기본 패키지 및 Universe 저장소 추가
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release build-essential cmake git wget \
    python3-pip python3-dev libgl1-mesa-glx libglib2.0-0 libpcl-dev \
    software-properties-common && \
    add-apt-repository universe && \
    rm -rf /var/lib/apt/lists/*

# 5. ROS2 Humble 설치
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-pcl-conversions \
    ros-humble-nav2-msgs \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 6. Python 딥러닝 패키지 설치 (blinker, sympy, mpmath 충돌 무시 옵션 포함)
RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir --ignore-installed sympy mpmath blinker && \
    pip3 install --no-cache-dir torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu124 && \
    pip3 install --no-cache-dir spconv-cu124 && \
    pip3 install --no-cache-dir open3d numpy scipy matplotlib opencv-python

# 7. 작업 공간 설정
WORKDIR /workspace

# 8. ROS2 환경 변수 자동 로드
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]