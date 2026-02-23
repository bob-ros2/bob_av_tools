# bob_av_tools

A collection of audio-visual utilities for the Bob ROS 2 ecosystem. This package provides high-fidelity web-based video rendering, interactive terminal overlays, and robust FIFO stream orchestration.

## 🚀 Key Features

- **`webvideo` Node**: Renders an offscreen browser overlay with auto-reconnecting FIFO output (BGRA raw) or ROS Image topics. Optimized for "Nexus/Matrix" aesthetics.
- **`webview` Node**: Interactive sibling of `webvideo` that opens a GUI window, supporting interactive chat ("Uplink Echo") and live stream display.
- **Robust FIFO Reconnect**: Advanced producer/consumer handling (`O_NONBLOCK` + `fcntl`) that allows ffmpeg, ffplay, or `bob_sdlviz` to connect and disconnect without breaking the pipeline.
- **Offline Aesthetic**: Pre-bundled `marked.min.js` for Markdown rendering and Vanilla CSS "Nexus Style" design—no internet or heavy frameworks required.
- **Custom CSS Overrides**: Inject your own styles at runtime using ROS parameters.

---

## 📦 Installation

### Dependencies
Ensure you have the following Python libraries:
```bash
pip install PySide6 numpy
```

System dependencies for QtWebEngine:
```bash
sudo apt update
sudo apt install libxcb-cursor0 libgbm1 libnss3 libasound2
```

---

## 🛠️ Usage

### Web Video Renderer (`webvideo`)
Renders an offscreen browser and sends frames to a video pipe or ROS topic.

```bash
# Basic run (default FIFO: /tmp/web_fifo)
ros2 run bob_av_tools webvideo

# Advanced: Custom CSS and remapping
ros2 run bob_av_tools webvideo --ros-args \
  --remap llm_stream:=/bob/llm_stream \
  -p override_css:=/path/to/my_style.css
```

### Webview Terminal (`webview`)
Opens a visible interactive window.

```bash
# Run with interactive chat enabled
ros2 run bob_av_tools webview --ros-args -p enable_chat:=true
```

---

## ⚙️ Configuration

### Parameters
All parameters support environment variable overrides (prefix `WEBVIEW_` for webview, `WEBVIDEO_` for webvideo).

| Parameter | Default | Description |
|-----------|---------|-------------|
| `width` | `854` (video) / `1024` (view) | Canvas/Window width. |
| `height` | `480` (video) / `768` (view) | Canvas/Window height. |
| `fps` | `30.0` (video only) | Frames per second for capture. |
| `fifo_path` | `/tmp/web_fifo` (video only) | Path to the raw video FIFO. |
| `enable_chat` | `false` (webview only) | Enables the interactive chat area. |
| `override_css`| `''` | Path to a `.css` file for custom styling. |

### Topics (Remappable)
| Topic Name | Type | Description |
|------------|------|-------------|
| `llm_stream` | `std_msgs/msg/String` | Input for text/markdown content. |
| `web_image`  | `sensor_msgs/msg/Image` (video only) | Rendered frames. |

---

## 🎨 Nexus Aesthetic & Customization

The package uses a "Nexus Style" aesthetic: cyan accents, terminal prefixes (`UPLINK >`), and typewriter animations.

### Custom CSS
You can override any style by providing a CSS file via the `override_css` parameter.
Example `my_style.css`:
```css
:root {
    --accent-color: #00ff41; /* Matrix Green */
    --font-size: 24px;
}
```

### FIFO Reconnection
The `webvideo` node is designed to be "sticky". You can start and stop your consumer (e.g., `ffplay`) as many times as you like; the node will automatically detect the connection and resume frame delivery within 1 second.

---

## ⚖️ License
Apache-2.0
