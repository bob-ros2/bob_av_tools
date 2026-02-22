# bob_av_tools

A collection of audio-visual utilities for the Bob ROS 2 ecosystem. This package provides high-fidelity web-based video rendering, FIFO management, and stream orchestration tools.

## 🚀 Key Features

- **`webvideo` Node**: Renders a premium HTML/JS overlay using QtWebEngine and outputs it to a FIFO pipe or a ROS Image topic.
- **`write_fifo.sh`**: A robust utility for feeding external data into FIFO pipes with proper buffer handling.
- **Premium Offline Rendering**: Integrated Markdown engine (`markdown-it`) and Tailwind CSS for high-quality visuals without internet dependency.
- **Duale Video-Ausgabe**: Gleichzeitige Ausgabe via FIFO (für FFmpeg/sdlviz) und ROS Image Topic (`cv_bridge`).

---

## 📦 Installation

### Dependencies
Ensure you have the following Python libraries:
```bash
pip install PySide6 numpy
```

System dependencies for QtWebEngine (Headless):
```bash
sudo apt update
sudo apt install libxcb-cursor0 libgbm1 libnss3 libasound2
```

---

## 🛠️ Usage

### Web Video Renderer (`webvideo`)
Renders an offscreen browser and sends frames to a video pipe or ROS topic.

```bash
ros2 run bob_av_tools webvideo --ros-args \
  -p fifo_path:=/tmp/overlay_video \
  -p pub_topic:=/bob/overlay_stream \
  -p sub_topic:=/bob/llm_stream
```

#### Parameters:
| Parameter | Default | Description |
|-----------|---------|-------------|
| `fifo_path` | `/tmp/overlay_video` | Path to the output FIFO pipe. |
| `pub_topic` | `/bob/overlay_stream`| ROS Image topic for the rendered output. |
| `sub_topic` | `/bob/llm_stream` | ROS String topic for incoming text/markdown. |
| `width` | `854` | Render width. |
| `height` | `480` | Render height. |
| `fps` | `30.0` | Frames per second. |

### FIFO Helper (`write_fifo.sh`)
Pipes data into a FIFO, creating it if necessary.

```bash
cat my_video.bgra | ros2 run bob_av_tools write_fifo.sh --path /tmp/video_pipe
```

---

## 🎨 Premium Overlay Customization

The renderer uses an `overlay.html` file with an integrated Markdown parser. It supports:
- **Glassmorphism**: Advanced blur and glowing accents.
- **Markdown**: Headers, bold/italic, lists, tables, and code blocks.
- **Animations**: Slide-in transitions for new content.

To customize the design, you can modify `overlay.html` in the package source or override it via parameters (future).

---

## ⚖️ License
Apache-2.0
