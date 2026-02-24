# Copyright 2026 Bob Ros
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import fcntl
import os
import signal
import sys
import threading
from pathlib import Path

import numpy as np

# ROS 2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor
from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError
)

# CV Bridge support
try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False

# Qt imports
try:
    from PySide6.QtWidgets import QApplication
    from PySide6.QtWebEngineCore import QWebEnginePage
    from PySide6.QtWebEngineWidgets import QWebEngineView
    from PySide6.QtCore import QUrl, QTimer, QPoint
    from PySide6.QtGui import QImage, QPainter
except ImportError as e:
    print(f"Error: {e}. Please install PySide6 with: pip install PySide6")
    sys.exit(1)


class CustomPage(QWebEnginePage):
    def __init__(self, node, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.node = node

    def javaScriptConsoleMessage(self, level, message, line_id, source_id):
        self.node.get_logger().info(f"[JS] {message} (line {line_id})")


class WebRenderer(Node):
    def __init__(self):
        super().__init__('webvideo_node')

        # Log ROS Domain for transparency
        domain_id = os.environ.get('ROS_DOMAIN_ID', '0 (default)')
        self.get_logger().info(
            f"Web Video Node starting on ROS_DOMAIN_ID: {domain_id}"
        )

        # Parameters with Env-Var defaults and Descriptors
        self.declare_parameter(
            'width',
            int(os.environ.get('WEBVIDEO_WIDTH', 854)),
            ParameterDescriptor(description='Rendering width in pixels')
        )
        self.declare_parameter(
            'height',
            int(os.environ.get('WEBVIDEO_HEIGHT', 480)),
            ParameterDescriptor(description='Rendering height in pixels')
        )
        self.declare_parameter(
            'fps',
            float(os.environ.get('WEBVIDEO_FPS', 30.0)),
            ParameterDescriptor(description='Frames per second for capture')
        )
        self.declare_parameter(
            'fifo_path',
            os.environ.get('WEBVIDEO_FIFO_PATH', '/tmp/web_fifo'),
            ParameterDescriptor(description='File path for the raw video FIFO')
        )
        self.declare_parameter(
            'queue_length',
            int(os.environ.get('WEBVIDEO_QUEUE_LENGTH', 1000)),
            ParameterDescriptor(description='ROS subscription queue size')
        )
        self.declare_parameter(
            'override_css',
            os.environ.get('WEBVIDEO_OVERRIDE_CSS', ''),
            ParameterDescriptor(description='Path to a custom .css file')
        )
        self.declare_parameter(
            'ui_path',
            os.environ.get('WEBVIDEO_UI_PATH', ''),
            ParameterDescriptor(description='Path to a custom .html file')
        )

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.fifo_path = self.get_parameter('fifo_path').value
        self.queue_length = self.get_parameter('queue_length').value
        self.override_css_path = self.get_parameter('override_css').value

        # ROS Publishers & Subscriptions
        # Fixed topic name 'web_image', can be remapped
        self.image_pub = None
        if HAS_CV_BRIDGE:
            self.image_pub = self.create_publisher(Image, 'web_image', 10)
            self.bridge = CvBridge()
            self.get_logger().info("Publishing frames to: web_image")
        else:
            self.get_logger().warn("CvBridge not found. No Image publisher.")

        self.subscription = self.create_subscription(
            String,
            'llm_stream',
            self.listener_callback,
            self.queue_length
        )

        # FIFO Setup (non-blocking, auto-reconnect)
        self.fifo_fd = None
        if self.fifo_path:
            if not os.path.exists(self.fifo_path):
                os.mkfifo(self.fifo_path)
            self.get_logger().info(
                f"FIFO ready: {self.fifo_path} (waiting for reader...)"
            )

        # Shared state
        self.current_content = ""
        self.lock = threading.Lock()

        # Initialize Qt Application
        self.qt_app = QApplication.instance() or QApplication(sys.argv)

        # Create WebEngineView
        self.view = QWebEngineView()
        self.view.resize(self.width, self.height)
        self.view.show()

        # Custom page for console logging
        self.page = CustomPage(self)
        self.view.setPage(self.page)

        # Load local HTML
        self.ui_path = self.get_parameter('ui_path').value
        if not self.ui_path:
            try:
                package_share_dir = get_package_share_directory('bob_av_tools')
                html_path = Path(package_share_dir) / "webvideo.html"
            except (PackageNotFoundError, Exception):
                # Fallback for local development/non-installed runs
                html_path = Path(__file__).parent / "webvideo.html"
                if not html_path.exists():
                    html_path = (
                        Path(os.getcwd()) / "bob_av_tools" / "webvideo.html"
                    )
            self.ui_path = str(html_path.absolute())
        else:
            self.ui_path = str(Path(self.ui_path).absolute())

        self.get_logger().info(f"Loading UI from: {self.ui_path}")
        self.page.load(QUrl.fromLocalFile(self.ui_path))

        # Connect injection once loaded
        self.page.loadFinished.connect(self._on_load_finished)

        # Timer for frame capture
        self.timer = QTimer()
        self.timer.timeout.connect(self.capture_frame)
        self.timer.start(int(1000 / self.fps))

        # FIFO reconnect timer (must be created AFTER QApplication)
        if self.fifo_path:
            self.reconnect_timer = QTimer()
            self.reconnect_timer.timeout.connect(self._try_reconnect_fifo)
            self.reconnect_timer.start(1000)

    def _try_reconnect_fifo(self):
        """Try to open the FIFO in non-blocking write mode."""
        if self.fifo_fd is not None:
            return  # Already connected
        try:
            fd = os.open(self.fifo_path, os.O_WRONLY | os.O_NONBLOCK)
            # Switch back to blocking for reliable full-frame writes
            flags = fcntl.fcntl(fd, fcntl.F_GETFL)
            fcntl.fcntl(fd, fcntl.F_SETFL, flags & ~os.O_NONBLOCK)
            self.fifo_fd = fd
            self.get_logger().info("FIFO connected to reader.")
        except OSError:
            pass  # No reader yet, will retry on next timer tick

    def _on_load_finished(self, success):
        if success:
            css_exists = (
                self.override_css_path and
                os.path.exists(self.override_css_path)
            )
            if css_exists:
                try:
                    with open(self.override_css_path, 'r') as f:
                        css_content = f.read()
                    js_inject = (
                        "const style = document.createElement('style');"
                        f"style.textContent = {repr(css_content)}; "
                        "document.head.insertAdjacentElement('beforeend', style);"
                    )
                    self.page.runJavaScript(js_inject)
                    self.get_logger().info(
                        f"Injected custom CSS from: {self.override_css_path}"
                    )
                except Exception as e:
                    self.get_logger().error(f"Failed to load CSS: {e}")
            elif self.override_css_path:
                self.get_logger().warning(
                    f"CSS file not found: {self.override_css_path}"
                )

    def listener_callback(self, msg):
        with self.lock:
            self.current_content += msg.data
            js_code = (
                "if(window.updateContent) "
                "window.updateContent("
                f"{repr(self.current_content)}"
                ");"
            )
            self.page.runJavaScript(js_code)

    def capture_frame(self):
        image = QImage(self.width, self.height, QImage.Format_ARGB32)
        image.fill(0)

        painter = QPainter(image)
        self.view.render(painter, QPoint(0, 0))
        painter.end()

        # 1. Publish to ROS Topic if enabled
        if self.image_pub:
            # Convert QImage to numpy (BGRA)
            buf = image.bits()
            arr = np.frombuffer(buf, dtype=np.uint8).reshape(
                self.height, self.width, 4
            )

            # Publish as Image message
            try:
                # Use "bgra8" for image.Format_ARGB32
                # (actually BGRA in memory on little-endian)
                msg = self.bridge.cv2_to_imgmsg(arr, "bgra8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "overlay_frame"
                self.image_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish image: {e}")

        # 2. Write to FIFO if enabled
        if self.fifo_fd is not None:
            data = image.constBits().tobytes()
            try:
                total_sent = 0
                while total_sent < len(data):
                    sent = os.write(self.fifo_fd, data[total_sent:])
                    if sent == 0:
                        break
                    total_sent += sent
            except OSError as e:
                # 32=EPIPE (broken pipe), 11=EAGAIN (non-blocking would block)
                if e.errno == 32:  # Broken pipe - reader disconnected
                    self.get_logger().warn("Reader disconnected, waiting...")
                    try:
                        os.close(self.fifo_fd)
                    except OSError:
                        pass
                    self.fifo_fd = None
                elif e.errno != 11:  # Ignore EAGAIN, log others
                    self.get_logger().error(f"FIFO write failed: {e}")
                    try:
                        os.close(self.fifo_fd)
                    except OSError:
                        pass
                    self.fifo_fd = None

    def run(self):
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        timer = QTimer()
        timer.timeout.connect(self._ros_spin_once)
        timer.start(10)  # 100Hz
        return self.qt_app.exec()

    def _ros_spin_once(self):
        if rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0)
            except Exception:
                pass


def main(args=None):
    os.environ["QT_QPA_PLATFORM"] = "offscreen"
    # Set Chromium flags via env var - reliable for both run and launch
    flags = (
        "--disable-gpu --no-sandbox --disable-software-rasterizer"
    )
    os.environ["QTWEBENGINE_CHROMIUM_FLAGS"] = (
        os.environ.get("QTWEBENGINE_CHROMIUM_FLAGS", "") + " " + flags
    ).strip()
    rclpy.init(args=args)

    renderer = WebRenderer()
    exit_code = renderer.run()

    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
