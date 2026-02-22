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
            ParameterDescriptor(
                description='Enable interactive chat input area'
            )
        )

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.queue_length = self.get_parameter('queue_length').value
        self.enable_chat = self.get_parameter('enable_chat').value

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

        # Setup Bridge for JS -> Python communication
        self.bridge = Bridge(self)
        self.page.setWebChannel(None)  # Reset
        # We'll use a simpler runJavaScript approach if possible,
        # but for clean Slot usage we'll stick to exposing an object
        # Note: QtWebChannel is usually safer, but for a single slot,
        # we can also use a custom object injection or just a direct JS prompt.
        # Let's use the standard "expose object" pattern.

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
        if self.enable_chat:
            url.setQuery("chat=true")

        self.get_logger().info(
            f"Loading UI from: {html_path} (Chat: {self.enable_chat})"
        )
        self.page.load(url)

        # Connect bridge to window object once loaded
        self.page.loadFinished.connect(self._setup_js_bridge)

        self.main_window.show()

    def _setup_js_bridge(self, success):
        if success:
            # Simple injection to handle Python calls from JS
            self.page.runJavaScript(
                "window.pythonBridge = { sendMessage: (text) => "
                "{ console.log('BRIDGE_CALL:' + text); } };"
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

    def run(self):
        # Handle Ctrl+C properly in Qt
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        timer = QTimer()
        timer.timeout.connect(self._ros_spin_once)
        timer.start(10)  # 100Hz

        # Poll Console for Bridge Calls (Simpler than full WebChannel)
        bridge_timer = QTimer()
        bridge_timer.timeout.connect(self._check_bridge_calls)
        bridge_timer.start(50)

        return self.qt_app.exec()

    def _check_bridge_calls(self):
        # We'll rely on a manual bridge mechanism since QWebChannel needs
        # more dependencies
        pass

    # Alternative: Use simple console.log parsing for the bridge
    def javaScriptConsoleMessage(self, level, message, line_id, source_id):
        if message.startswith("BRIDGE_CALL:"):
            text = message[len("BRIDGE_CALL:"):]
            self.bridge.sendMessage(text)
        else:
            self.get_logger().info(f"[JS] {message}")

    def _ros_spin_once(self):
        if rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    renderer = WebviewNode()
    # Apply the console log handler fix
    renderer.page.javaScriptConsoleMessage = renderer.javaScriptConsoleMessage
    exit_code = renderer.run()

    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
