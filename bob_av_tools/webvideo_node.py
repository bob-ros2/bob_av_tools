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

        # Parameters
        self.declare_parameter('fifo_path', '/tmp/overlay_video')
        self.declare_parameter('pub_topic', '/bob/overlay_stream')
        self.declare_parameter('width', 854)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('sub_topic', '/bob/llm_stream')

        self.fifo_path = self.get_parameter('fifo_path').value
        self.pub_topic = self.get_parameter('pub_topic').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value

        # ROS Publishers & Subscriptions
        self.image_pub = None
        if self.pub_topic and HAS_CV_BRIDGE:
            self.image_pub = self.create_publisher(Image, self.pub_topic, 10)
            self.bridge = CvBridge()
            self.get_logger().info(f"Publishing frames to: {self.pub_topic}")
        elif self.pub_topic and not HAS_CV_BRIDGE:
            self.get_logger().warn(
                "CvBridge not found. Disabling Image publisher."
            )

        self.subscription = self.create_subscription(
            String,
            self.get_parameter('sub_topic').value,
            self.listener_callback,
            20
        )

        # FIFO Setup
        self.fifo_fd = None
        if self.fifo_path:
            if not os.path.exists(self.fifo_path):
                os.mkfifo(self.fifo_path)
            self.get_logger().info(
                f"Opening FIFO: {self.fifo_path} (Waiting for reader...)"
            )
            self.fifo_fd = os.open(self.fifo_path, os.O_WRONLY)
            self.get_logger().info("FIFO opened for writing.")

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
        try:
            package_share_dir = get_package_share_directory('bob_av_tools')
            html_path = Path(package_share_dir) / "overlay.html"
        except (PackageNotFoundError, Exception):
            # Fallback for local development/non-installed runs
            html_path = Path(__file__).parent / "overlay.html"
            if not html_path.exists():
                html_path = Path(os.getcwd()) / "bob_av_tools" / "overlay.html"

        self.get_logger().info(f"Loading UI from: {html_path}")
        self.page.load(QUrl.fromLocalFile(str(html_path.absolute())))

        # Timer for frame capture
        self.timer = QTimer()
        self.timer.timeout.connect(self.capture_frame)
        self.timer.start(int(1000 / self.fps))

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
        if self.fifo_fd:
            data = image.constBits().tobytes()
            try:
                total_sent = 0
                while total_sent < len(data):
                    sent = os.write(self.fifo_fd, data[total_sent:])
                    if sent == 0:
                        break
                    total_sent += sent
            except OSError as e:
                if e.errno == 32:  # Broken pipe
                    self.get_logger().warn("Reader disconnected. Exit.")
                    os.close(self.fifo_fd)
                    self.fifo_fd = None
                else:
                    self.get_logger().error(f"FIFO write failed: {e}")

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
    rclpy.init(args=args)

    # Chromium flags
    sys.argv.append("--disable-gpu")
    sys.argv.append("--no-sandbox")
    sys.argv.append("--disable-software-rasterizer")
    sys.argv.append("--single-process")

    renderer = WebRenderer()
    exit_code = renderer.run()

    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
