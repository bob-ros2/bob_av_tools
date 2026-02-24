# bob_av_tools

[![CI – Build & Test](https://github.com/bob-ros2/bob_av_tools/actions/workflows/ci.yml/badge.svg)](https://github.com/bob-ros2/bob_av_tools/actions/workflows/ci.yml)
[![Docker – Build & Push](https://github.com/bob-ros2/bob_av_tools/actions/workflows/docker.yml/badge.svg)](https://github.com/bob-ros2/bob_av_tools/actions/workflows/docker.yml)

A collection of audio-visual utilities for the Bob ROS 2 ecosystem. This package provides high-fidelity web-based video rendering, interactive terminal overlays, and robust FIFO stream orchestration.

---

## 🚀 Key Features

- **`webvideo`**: Offscreen browser renderer for overlays. Output: FIFO (raw BGRA) or ROS Image.
- **`webview`**: Interactive GUI terminal sibling with chat support ("Uplink Echo").
- **`webscreen`**: Capture **any URL** (website/local) offscreen. Supports authentications & automation.
- **`write_fifo.sh`**: Helper script to pipe external streams (FFmpeg, etc.) into a managed FIFO.
- **Robust FIFO Reconnect**: Advanced producer/consumer handling (`O_NONBLOCK` + `fcntl`) for seamless hot-swapping of readers.
- **Nexus Aesthetic**: Built-in "Nexus Style" UI: Cyberspace/Terminal design with typewriter effects.

---

## 📦 Installation

### 1. Python Dependencies
```bash
pip install PySide6 numpy
```

### 2. System Requirements (QtWebEngine/Chromium)
The renderer requires several X11 and GL libraries for offscreen rendering:
```bash
sudo apt update && sudo apt install -y \
    libxcb-cursor0 libgbm1 libnss3 libasound2 libxcomposite1 \
    libxdamage1 libxrandr2 libxcb-icccm4 libxcb-image0 \
    libxcb-keysyms1 libxcb-render-util0 libgl1 libegl1
```

---

## 🛠️ Node Documentation

### 1. Web Video Renderer (`webvideo`)
Renders a localized HTML overlay (`overlay.html`) for LLM text streams. Optimized for embedding in video mixers.

#### ROS API
- **Subscribed Topics**:
  - `llm_stream` (`std_msgs/msg/String`): Incoming text chunks.
- **Published Topics**:
  - `web_image` (`sensor_msgs/msg/Image`): Rendered frames (Requires `cv_bridge`).

#### Configuration (Parameters & Env Vars)
| Parameter | Env Var Equivalent | Default | Description |
|-----------|--------------------|---------|-------------|
| `width` | `WEBVIDEO_WIDTH` | `854` | Rendering width (px). |
| `height` | `WEBVIDEO_HEIGHT` | `480` | Rendering height (px). |
| `fps` | `WEBVIDEO_FPS` | `30.0` | Frames per second. |
| `fifo_path` | `WEBVIDEO_FIFO_PATH` | `/tmp/web_fifo` | Path to output raw BGRA pipe. |
| `queue_length`| `WEBVIDEO_QUEUE_LENGTH`| `1000` | Subscription queue size. |
| `override_css`| `WEBVIDEO_OVERRIDE_CSS`| `''` | Path to a custom .css file. |

---

### 2. Webview Terminal (`webview`)
Interactive window for human-in-the-loop interaction. Opens a GUI window on the primary display.

#### ROS API
- **Subscribed Topics**:
  - `llm_stream` (`std_msgs/msg/String`): Feed for the terminal display.
- **Published Topics**:
  - `chat_out` (`std_msgs/msg/String`): Published when a user sends a message in the UI.

#### Configuration (Parameters & Env Vars)
| Parameter | Env Var Equivalent | Default | Description |
|-----------|--------------------|---------|-------------|
| `width` | `WEBVIEW_WIDTH` | `1024` | Window width (px). |
| `height` | `WEBVIEW_HEIGHT` | `768` | Window height (px). |
| `enable_chat` | `WEBVIEW_ENABLE_CHAT` | `false` | Enable/Disable chat input area. |
| `queue_length`| `WEBVIEW_QUEUE_LENGTH`| `1000` | Subscription queue size. |
| `override_css`| `WEBVIEW_OVERRIDE_CSS`| `''` | Path to a custom .css file. |

---

### 3. URL Screen Capture (`webscreen`)
Renders any external URL or local file offscreen. Ideal for capturing Twitch chats, dashboards, or static web pages.

#### ROS API
- **Published Topics**:
  - `webscreen_image` (`sensor_msgs/msg/Image`): Captures views as ROS messages.

#### Configuration (Parameters & Env Vars)
| Parameter | Env Var Equivalent | Default | Description |
|-----------|--------------------|---------|-------------|
| `url` | `WEBSCREEN_URL` | `''` | **Required.** URL or `file://` path. |
| `width` | `WEBSCREEN_WIDTH` | `1280` | Viewport width. |
| `height` | `WEBSCREEN_HEIGHT` | `720` | Viewport height. |
| `fps` | `WEBSCREEN_FPS` | `30.0` | Capture rate. |
| `fifo_path` | `WEBSCREEN_FIFO_PATH`| `/tmp/webscreen_fifo` | Path to raw BGRA pipe. |
| `cookies_file`| `WEBSCREEN_COOKIES_FILE`| `''` | Path to JSON cookies for auth. |
| `pre_script` | `WEBSCREEN_PRE_SCRIPT` | `''` | Path to JS automation script. |
| `scroll_x` | `WEBSCREEN_SCROLL_X` | `0` | Initial horizontal scroll. |
| `scroll_y` | `WEBSCREEN_SCROLL_Y` | `0` | Initial vertical scroll. |

---

## 📂 Configuration Examples

### JSON Cookies (`cookies_file`)
Required for websites with login (e.g., Twitch, Matrix). Use a browser extension to export cookies as JSON.
```json
[
  {
    "name": "session",
    "value": "xyz123...",
    "domain": ".twitch.tv",
    "path": "/",
    "secure": true,
    "httpOnly": true,
    "expirationDate": 1771890000.5
  }
]
```

### Pre-Script JS (`pre_script`)
Automate page actions (dismiss banners, focus elements, click buttons) before capture begins.
```javascript
// Hide persistent cookie banners or ads
const banner = document.querySelector('.cookie-consent');
if (banner) banner.style.display = 'none';

// Force dark mode if supported
document.documentElement.setAttribute('data-theme', 'dark');
console.log('Page automation complete.');
```

### Custom Styling (`override_css`)
Override the default "Nexus" look for `webvideo` and `webview`.

#### Layout Variables
You can override these in your `.css` file for precise layout control without worrying about CSS specificity of IDs:
- `--root-padding`: Padding of the outer container (default: `40px` for overlay, `20px` for webview).
- `--content-padding`: Padding inside the terminal/content area (default: `20px`).
- `--content-bg`: Background color/alpha of the content area.
- `--content-border-left`: Style of the left accent border.
- `--chat-bg`: (Webview only) Background of the chat input area.

**Example:**
```css
:root {
    --accent-color: #ff00ff; /* Cyberpink */
    --root-padding: 0px;      /* Full screen capture */
    --content-bg: transparent;
    --content-border-left: none;
}
```

---

## 📜 Utility Scripts

### `write_fifo.sh`
Orchestrates streams from external processes (like FFmpeg) into managed FIFOs.
```bash
# Capture webcam and pipe to FIFO for sdlviz
ffmpeg -i /dev/video0 -f rawvideo -pixel_format bgr24 - | \
  ros2 run bob_av_tools write_fifo.sh --path /tmp/cam_fifo
```

---

## ⚙️ Advanced Configuration (Chromium Flags)
The nodes automatically set `QTWEBENGINE_CHROMIUM_FLAGS` for headless compatibility. You can override these via the environment if needed:
```bash
export QTWEBENGINE_CHROMIUM_FLAGS="--no-sandbox --disable-setuid-sandbox --disable-gpu"
```

## ⚖️ License
Apache-2.0
