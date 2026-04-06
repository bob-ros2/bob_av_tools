FROM ros:humble-ros-base

# System deps for PySide6/QtWebEngine + cv_bridge + build tools
# We install EVERYTHING needed for a headless Qt6 X11 environment
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3-pip \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-ament-cmake-python \
    # PySide6 / QtWebEngine mandatory deps
    libxcb-cursor0 \
    libxcb-xinerama0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-render-util0 \
    libxcb-xkb1 \
    libxcb-randr0 \
    libxcb-util1 \
    libxcb-shape0 \
    libxcb-xfixes0 \
    libxcb-sync1 \
    libx11-xcb1 \
    libsm6 \
    libice6 \
    libgbm1 \
    libnss3 \
    libasound2 \
    libxcomposite1 \
    libxdamage1 \
    libxrandr2 \
    libxtst6 \
    libxshmfence1 \
    libgl1 \
    libegl1 \
    libgl1-mesa-dri \
    libosmesa6 \
    xvfb \
    libxkbfile1 \
    libxkbcommon-x11-0 \
    libdbus-1-3 \
    && rm -rf /var/lib/apt/lists/*

# Install Python deps
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Copy and build the package
COPY . /ros2_ws/src/bob_av_tools
WORKDIR /ros2_ws

RUN bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select bob_av_tools"

ENV QT_QPA_PLATFORM=xcb
ENV QTWEBENGINE_CHROMIUM_FLAGS="--disable-gpu --no-sandbox"
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV GALLIUM_DRIVER=llvmpipe
ENV DISPLAY=:99

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && rm -f /tmp/.X99-lock && (Xvfb :99 -screen 0 1280x1024x24 -nolisten tcp &) && sleep 1 && exec \"$@\"", "--"]
CMD ["ros2 run bob_av_tools webscreen"]
