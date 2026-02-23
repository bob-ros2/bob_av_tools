# bob_av_tools

[![CI – Build & Test](https://github.com/bob-ros2/bob_av_tools/actions/workflows/ci.yml/badge.svg)](https://github.com/bob-ros2/bob_av_tools/actions/workflows/ci.yml)
[![Docker – Build & Push](https://github.com/bob-ros2/bob_av_tools/actions/workflows/docker.yml/badge.svg)](https://github.com/bob-ros2/bob_av_tools/actions/workflows/docker.yml)

A collection of audio-visual utilities for the Bob ROS 2 ecosystem. This package provides high-fidelity web-based video rendering, interactive terminal overlays, and robust FIFO stream orchestration.

## 🚀 Key Features

- **`webvideo` Node**: Renders an offscreen browser overlay with auto-reconnecting FIFO output (BGRA raw) or ROS Image topics.
- **`webview` Node**: Interactive sibling of `webvideo` that opens a GUI window, supporting interactive chat and live stream display.
- **`webscreen` Node**: Renders **any URL** offscreen and streams it to FIFO/ROS Image.
- **`write_fifo.sh`**: Helper script to pipe any stdin stream (e.g., from FFmpeg) into a managed FIFO.
- **Robust FIFO Reconnect**: Advanced producer/consumer handling (`O_NONBLOCK` + `fcntl`) allowing seamless connection/disconnection.
- **Offline Aesthetic**: Pre-bundled Markdown rendering and "Nexus Style" design.

---

## 🛠️ Usage & ROS API

### 1. Web Video Renderer (`webvideo`)
Renders a localized HTML overlay for LLM streams.

#### ROS API
- **Subscribed Topics**:
  - `llm_stream` (`std_msgs/msg/String`): Incoming text/markdown chunks to display.
- **Published Topics**:
  - `web_image` (`sensor_msgs/msg/Image`): Rendered frames (if `cv_bridge` is installed).
- **Parameters**:
  - `width` (int, default: 854): Rendering width.
  - `height` (int, default: 480): Rendering height.
  - `fps` (float, default: 30.0): Capture rate.
  - `fifo_path` (string, default: `/tmp/web_fifo`): Target FIFO path.
  - `override_css` (string, default: `''`): Path to custom CSS file.

---

### 2. Webview Terminal (`webview`)
Interactive window for human-in-the-loop chat and terminal output.

#### ROS API
- **Subscribed Topics**:
  - `llm_stream` (`std_msgs/msg/String`): Feed for the terminal area.
- **Published Topics**:
  - `chat_out` (`std_msgs/msg/String`): User input from the chat box.
- **Parameters**:
  - `width` (int, default: 1024): Window width.
  - `height` (int, default: 768): Window height.
  - `enable_chat` (bool, default: false): Show/hide chat interface.
  - `override_css` (string, default: `''`): Path to custom CSS file.

---

### 3. URL Screen Capture (`webscreen`)
Offscreen render of any URL (website or local file).

#### ROS API
- **Published Topics**:
  - `webscreen_image` (`sensor_msgs/msg/Image`): Rendered frames.
- **Parameters**:
  - `url` (string, required): The URL to render.
  - `width` (int, default: 1280): Viewport width.
  - `height` (int, default: 720): Viewport height.
  - `fps` (float, default: 30.0): Capture rate.
  - `fifo_path` (string, default: `/tmp/webscreen_fifo`): Target FIFO path.
  - `cookies_file` (string, default: `''`): Path to a JSON cookies file.
  - `pre_script` (string, default: `''`): Path to a JS script to run on page load.
  - `scroll_x` / `scroll_y` (int, default: 0): Initial scroll offset.

---

## 📂 Configuration Examples

### Cookies JSON (`cookies_file`)
Compatible with common browser export formats (e.g., EditThisCookie).
```json
[
  {
    "name": "session_id",
    "value": "your_secure_token_here",
    "domain": ".example.com",
    "path": "/",
    "secure": true,
    "httpOnly": true
  }
]
```

### Pre-Script JS (`pre_script`)
Executed at `DeferredLoad` (DOM ready) to automate the page.
```javascript
// Example: Hide a cookie banner and click a button
document.querySelector('.banner-close').click();
console.log('Banner dismissed');
```

---

## 📜 Helper Scripts

### `write_fifo.sh`
Used to pipe standard output into a FIFO that `bob_sdlviz` or other nodes can read from.

**Example: Pipe FFmpeg into a FIFO**
```bash
ffmpeg -i /dev/video0 -f rawvideo - | \
  ros2 run bob_av_tools write_fifo.sh --path /tmp/camera_fifo
```

---

## 📦 Installation
```bash
# Python dependencies
pip install PySide6 numpy

# System requirements (QtWebEngine)
sudo apt update
sudo apt install libxcb-cursor0 libgbm1 libnss3 libasound2 libxcomposite1 libxdamage1 libxrandr2 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-render-util0 libgl1 libegl1
```

---

## ⚖️ License
Apache-2.0
