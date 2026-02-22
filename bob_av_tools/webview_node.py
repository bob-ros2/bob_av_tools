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
    from PySide6.QtCore import QUrl, QTimer, Slot, QObject
except ImportError as e:
    print(f"Error: {e}. Please install PySide6 with: pip install PySide6")
    sys.exit(1)


class Bridge(QObject):
    def __init__(self, node):
        super().__init__()
        self.node = node

    @Slot(str)
    def sendMessage(self, text):
        msg = String()
        msg.data = text
        self.node.chat_pub.publish(msg)
        self.node.get_logger().info(
            f"Published chat message: '{text[:20]}...'"
        )


class CustomPage(QWebEnginePage):
    def __init__(self, node, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.node = node

    def javaScriptConsoleMessage(self, level, message, line_id, source_id):
        if message.startswith("BRIDGE_CALL:"):
            text = message[len("BRIDGE_CALL:"):]
            self.node.bridge.sendMessage(text)
        else:
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
        self.declare_parameter(
            'enable_chat',
            False,
            ParameterDescriptor(description='Enable interactive chat area')
        )
        self.declare_parameter(
            'override_css',
            '',
            ParameterDescriptor(description='Path to a custom .css file')
        )

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.queue_length = self.get_parameter('queue_length').value
        self.enable_chat = self.get_parameter('enable_chat').value
        self.override_css_path = self.get_parameter('override_css').value

        # Shared state
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

        # Custom page for console logging and bridge calls
        self.page = CustomPage(self)
        self.view.setPage(self.page)

        # Bridge object
        self.bridge = Bridge(self)

        # Load local HTML
        try:
            package_share_dir = get_package_share_directory('bob_av_tools')
            html_path = Path(package_share_dir) / "index.html"
        except (PackageNotFoundError, Exception):
            html_path = Path(__file__).parent / "index.html"
            if not html_path.exists():
                html_path = Path(os.getcwd()) / "bob_av_tools" / "index.html"

        # Subscription
        self.subscription = self.create_subscription(
            String,
            'llm_stream',
            self.listener_callback,
            self.queue_length
        )

        # Publisher for Chat
        self.chat_pub = self.create_publisher(String, 'chat_out', 10)

        # Load file with query param for chat state
        url = QUrl.fromLocalFile(str(html_path.absolute()))
        query_params = []
        if self.enable_chat:
            query_params.append("chat=true")
        if query_params:
            url.setQuery("&".join(query_params))

        self.get_logger().info(
            f"Loading UI from: {html_path} (Chat: {self.enable_chat})"
        )
        self.page.load(url)

        # Connect bridge and inject CSS once loaded
        self.page.loadFinished.connect(self._on_load_finished)

        self.main_window.show()

    def _on_load_finished(self, success):
        if success:
            # 1. Inject Bridge
            self.page.runJavaScript(
                "window.pythonBridge = { sendMessage: (text) => "
                "{ console.log('BRIDGE_CALL:' + text); } };"
            )
            # 2. Inject Custom CSS if provided
            if self.override_css_path and os.path.exists(self.override_css_path):
                try:
                    with open(self.override_css_path, 'r') as f:
                        css_content = f.read()
                    # Use a style tag creation logic in JS
                    js_inject = (
                        "const style = document.createElement('style');"
                        f"style.textContent = {repr(css_content)}; "
                        "document.head.appendChild(style);"
                    )
                    self.page.runJavaScript(js_inject)
                    self.get_logger().info(
                        f"Injected custom CSS from: {self.override_css_path}"
                    )
                except Exception as e:
                    self.get_logger().error(f"Failed to load CSS: {e}")

    def listener_callback(self, msg):
        # We now send data chunks. The JS side will handle the accumulation
        # into a persistent AI block.
        js_code = f"if(window.appendStream) window.appendStream({repr(msg.data)});"
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
    rclpy.init(args=args)
    renderer = WebviewNode()
    exit_code = renderer.run()

    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
