FROM ros:humble-ros-base

# System deps for PySide6/QtWebEngine + cv_bridge + build tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3-pip \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-ament-cmake-python \
    # PySide6 / QtWebEngine deps
    libxcb-cursor0 \
    libgbm1 \
    libnss3 \
    libasound2 \
    libxcomposite1 \
    libxdamage1 \
    libxrandr2 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-render-util0 \
    libgl1 \
    libegl1 \
    && rm -rf /var/lib/apt/lists/*

# Install Python deps
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Copy and build the package
COPY . /ros2_ws/src/bob_av_tools
WORKDIR /ros2_ws

RUN bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select bob_av_tools"

ENV QT_QPA_PLATFORM=offscreen
ENV QTWEBENGINE_CHROMIUM_FLAGS="--disable-gpu --no-sandbox --disable-software-rasterizer"

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["ros2 run bob_av_tools webscreen"]
