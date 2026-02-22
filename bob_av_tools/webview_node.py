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

# ROS 2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor
from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError
)

# Qt imports
try:
    from PySide6.QtWidgets import QApplication, QMainWindow
    from PySide6.QtWebEngineCore import QWebEnginePage
    from PySide6.QtWebEngineWidgets import QWebEngineView
    from PySide6.QtCore import QUrl, QTimer
except ImportError as e:
    print(f"Error: {e}. Please install PySide6 with: pip install PySide6")
    sys.exit(1)


class CustomPage(QWebEnginePage):
    def __init__(self, node, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.node = node

    def javaScriptConsoleMessage(self, level, message, line_id, source_id):
        self.node.get_logger().info(f"[JS] {message} (line {line_id})")


class WebviewNode(Node):
    def __init__(self):
        super().__init__('webview_node')

        # Parameters with Env-Var defaults and Descriptors
        self.declare_parameter(
            'width',
            int(os.environ.get('WEBVIEW_WIDTH', 1024)),
            ParameterDescriptor(description='Window width in pixels')
        )
        self.declare_parameter(
            'height',
            int(os.environ.get('WEBVIEW_HEIGHT', 768)),
            ParameterDescriptor(description='Window height in pixels')
        )
        self.declare_parameter(
            'queue_length',
            int(os.environ.get('WEBVIEW_QUEUE_LENGTH', 1000)),
            ParameterDescriptor(description='ROS subscription queue size')
        )

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.queue_length = self.get_parameter('queue_length').value

        # Shared state
        self.current_content = ""
        self.lock = threading.Lock()

        # Initialize Qt Application
        self.qt_app = QApplication.instance() or QApplication(sys.argv)

        # Create Main Window
        self.main_window = QMainWindow()
        self.main_window.setWindowTitle("BOB NEXUS | INTERACTIVE TERMINAL")
        self.main_window.resize(self.width, self.height)

        # Create WebEngineView
        self.view = QWebEngineView()
        self.main_window.setCentralWidget(self.view)

        # Custom page for console logging
        self.page = CustomPage(self)
        self.view.setPage(self.page)

        # Load local HTML
        try:
            package_share_dir = get_package_share_directory('bob_av_tools')
            html_path = Path(package_share_dir) / "index.html"
        except (PackageNotFoundError, Exception):
            html_path = Path(__file__).parent / "index.html"
            if not html_path.exists():
                html_path = Path(os.getcwd()) / "bob_av_tools" / "index.html"

        self.get_logger().info(f"Loading UI from: {html_path}")
        self.page.load(QUrl.fromLocalFile(str(html_path.absolute())))

        # ROS Subscriptions
        self.subscription = self.create_subscription(
            String,
            'llm_stream',
            self.listener_callback,
            self.queue_length
        )

        self.main_window.show()

    def listener_callback(self, msg):
        with self.lock:
            self.current_content += msg.data
            # We don't need complex repair here as index.html uses marked.js directly
            js_code = (
                "if(window.updateContent) "
                "window.updateContent("
                f"{repr(self.current_content)}"
                ");"
            )
            self.page.runJavaScript(js_code)

    def run(self):
        # Handle Ctrl+C properly in Qt
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
    # For interactive webview, we want standard windowing, not offscreen
    rclpy.init(args=args)

    renderer = WebviewNode()
    exit_code = renderer.run()

    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
