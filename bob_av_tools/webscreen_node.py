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

"""
Render any URL offscreen and stream it to a FIFO pipe and/or ROS topic.

Supports cookie injection (JSON file), JavaScript pre-script injection at
DeferredLoad time, non-blocking FIFO auto-reconnect, and ROS Image publishing.
"""

import fcntl
import json
import os
import signal
import sys
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rcl_interfaces.msg import ParameterDescriptor

try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False

try:
    from PySide6.QtWidgets import QApplication
    from PySide6.QtWebEngineCore import (
        QWebEnginePage,
        QWebEngineProfile,
        QWebEngineScript,
    )
    from PySide6.QtWebEngineWidgets import QWebEngineView
    from PySide6.QtNetwork import QNetworkCookie
    from PySide6.QtCore import QUrl, QTimer, QByteArray
    from PySide6.QtGui import QImage, QPainter
except ImportError as e:
    print(f"Error: {e}. Please install PySide6 with: pip install PySide6")
    sys.exit(1)


class CustomPage(QWebEnginePage):
    """QWebEnginePage subclass forwarding console output to ROS logger."""

    def __init__(self, node):
        """Initialise with a reference to the owning ROS node."""
        super().__init__()
        self.node = node

    def javaScriptConsoleMessage(self, level, message, line, source):
        """Forward browser console messages to ROS logger at DEBUG level."""
        self.node.get_logger().debug(f"[JS] {source}:{line}: {message}")


class WebScreenNode(Node):
    """
    ROS 2 node that renders any URL offscreen and streams frames.

    Renders a URL or local file using QtWebEngine (Chromium) in offscreen mode.
    Frames are captured at the configured FPS, published as ROS Image, and/or
    written to a named FIFO pipe (ffplay, bob_sdlviz, FFmpeg).
    Supports cookie injection and JS pre-script injection for automation.
    """

    def __init__(self):
        """Declare parameters, initialise Qt application, load the URL."""
        super().__init__('webscreen_node')
        self.get_logger().info(
            f"Web Screen Node starting on "
            f"ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', '0')}"
        )

        # ── Parameters ───────────────────────────────────────────────────────
        def _p(key, default, desc):
            d = ParameterDescriptor(description=desc)
            self.declare_parameter(key, default, d)

        _p('width', int(os.environ.get('WEBSCREEN_WIDTH', 1280)),
           'Render width in pixels (Env: WEBSCREEN_WIDTH).')
        _p('height', int(os.environ.get('WEBSCREEN_HEIGHT', 720)),
           'Render height in pixels (Env: WEBSCREEN_HEIGHT).')
        _p('fps', float(os.environ.get('WEBSCREEN_FPS', 30.0)),
           'Frames per second for capture (Env: WEBSCREEN_FPS).')
        _p('url', os.environ.get('WEBSCREEN_URL', ''),
           'URL or local file:// path to load (Env: WEBSCREEN_URL).')
        _p('fifo_path',
           os.environ.get('WEBSCREEN_FIFO_PATH', '/tmp/webscreen_fifo'),
           'FIFO path for raw BGRA output (Env: WEBSCREEN_FIFO_PATH).')
        _p('queue_length', int(os.environ.get('WEBSCREEN_QUEUE_LENGTH', 10)),
           'ROS publisher queue depth (Env: WEBSCREEN_QUEUE_LENGTH).')
        _p('cookies_file', os.environ.get('WEBSCREEN_COOKIES_FILE', ''),
           'Path to a JSON cookie file for authentication '
           '(Env: WEBSCREEN_COOKIES_FILE).')
        _p('pre_script', os.environ.get('WEBSCREEN_PRE_SCRIPT', ''),
           'Path to a .js file injected at DeferredLoad time '
           '(Env: WEBSCREEN_PRE_SCRIPT).')
        _p('scroll_x', int(os.environ.get('WEBSCREEN_SCROLL_X', 0)),
           'Horizontal scroll offset in pixels (Env: WEBSCREEN_SCROLL_X).')
        _p('scroll_y', int(os.environ.get('WEBSCREEN_SCROLL_Y', 0)),
           'Vertical scroll offset in pixels (Env: WEBSCREEN_SCROLL_Y).')

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.url = self.get_parameter('url').value or ''
        self.fifo_path = self.get_parameter('fifo_path').value or ''
        self.queue_length = self.get_parameter('queue_length').value
        self.cookies_file = self.get_parameter('cookies_file').value or ''
        self.pre_script = self.get_parameter('pre_script').value or ''
        self.scroll_x = self.get_parameter('scroll_x').value
        self.scroll_y = self.get_parameter('scroll_y').value

        # ── ROS publisher ────────────────────────────────────────────────────
        self.publisher = self.create_publisher(
            Image, 'webscreen_image', self.queue_length)
        self.get_logger().info('Publishing frames to: webscreen_image')
        if HAS_CV_BRIDGE:
            self.bridge = CvBridge()

        # ── FIFO setup (non-blocking, auto-reconnect after QApp) ─────────────
        self.fifo_fd = None
        if self.fifo_path:
            if not os.path.exists(self.fifo_path):
                os.mkfifo(self.fifo_path)
            self.get_logger().info(
                f"FIFO ready: {self.fifo_path} (waiting for reader...)")

        # ── Qt application ───────────────────────────────────────────────────
        self.qt_app = QApplication.instance() or QApplication(sys.argv)

        # ── WebEngine profile (for cookies / persistent storage) ─────────────
        self.profile = QWebEngineProfile('webscreen', self.qt_app)
        self.cookie_store = self.profile.cookieStore()

        if self.cookies_file:
            self._inject_cookies(self.cookies_file)

        if self.pre_script:
            self._inject_pre_script(self.pre_script)

        # ── WebEngineView ────────────────────────────────────────────────────
        self.page = CustomPage(self)
        self.page.setBackgroundColor(
            self.qt_app.palette().color(
                self.qt_app.palette().ColorRole.Base))
        self.view = QWebEngineView()
        self.view.setPage(self.page)
        self.view.resize(self.width, self.height)
        self.view.show()
        self.page.loadFinished.connect(self._on_load_finished)

        if self.url:
            self.get_logger().info(f"Loading URL: {self.url}")
            self.page.load(QUrl(self.url))
        else:
            self.get_logger().warn(
                "No URL configured. Set the 'url' parameter or "
                "WEBSCREEN_URL env var.")
            self.page.setHtml(
                '<html><body style="background:#1e1e1e;color:#ccc;'
                'font-family:monospace;padding:2rem">'
                '<h2>webscreen</h2><p>No URL configured.</p></body></html>')

        # ── Timers (must be created AFTER QApplication) ──────────────────────
        self.timer = QTimer()
        self.timer.timeout.connect(self.capture_frame)
        self.timer.start(int(1000 / self.fps))

        if self.fifo_path:
            self.reconnect_timer = QTimer()
            self.reconnect_timer.timeout.connect(self._try_reconnect_fifo)
            self.reconnect_timer.start(1000)

    def _inject_cookies(self, path):
        """Load cookies from a JSON file and inject them into the browser."""
        try:
            with open(path) as f:
                cookies = json.load(f)
            for c in cookies:
                cookie = QNetworkCookie(
                    QByteArray(c['name'].encode()),
                    QByteArray(c.get('value', '').encode()),
                )
                if 'domain' in c:
                    cookie.setDomain(c['domain'])
                cookie.setPath(c.get('path', '/'))
                cookie.setSecure(c.get('secure', False))
                cookie.setHttpOnly(c.get('httpOnly', False))
                self.cookie_store.setCookie(cookie)
            self.get_logger().info(
                f"Injected {len(cookies)} cookies from {path}")
        except Exception as e:
            self.get_logger().error(
                f"Failed to load cookies from {path}: {e}")

    def _inject_pre_script(self, path):
        """Register a JS file to be injected at DeferredLoad into each page."""
        try:
            js_code = Path(path).read_text()
            script = QWebEngineScript()
            script.setName('webscreen_pre_script')
            script.setSourceCode(js_code)
            injection = QWebEngineScript.InjectionPoint.DeferredLoad
            script.setInjectionPoint(injection)
            script.setWorldId(QWebEngineScript.ScriptWorldId.MainWorld)
            script.setRunsOnSubFrames(False)
            self.page.scripts().insert(script)
            self.get_logger().info(f"Pre-script registered from: {path}")
        except Exception as e:
            self.get_logger().error(
                f"Failed to load pre_script from {path}: {e}")

    def _on_load_finished(self, success):
        """Log the result of a page load attempt."""
        if success:
            self.get_logger().info(f"Page loaded: {self.url}")
        else:
            self.get_logger().warn(
                f"Page load failed or partial: {self.url}")

    def _try_reconnect_fifo(self):
        """Try to open the FIFO non-blockingly and then switch to blocking."""
        if self.fifo_fd is not None:
            return
        try:
            fd = os.open(self.fifo_path, os.O_WRONLY | os.O_NONBLOCK)
            flags = fcntl.fcntl(fd, fcntl.F_GETFL)
            fcntl.fcntl(fd, fcntl.F_SETFL, flags & ~os.O_NONBLOCK)
            self.fifo_fd = fd
            self.get_logger().info("FIFO connected to reader.")
        except OSError:
            pass

    def capture_frame(self):
        """Capture the current render, publish to ROS, and write to FIFO."""
        image = QImage(self.width, self.height, QImage.Format.Format_ARGB32)
        painter = QPainter(image)
        self.view.render(painter, sourceRegion=None)
        painter.end()

        if HAS_CV_BRIDGE:
            try:
                ptr = image.bits()
                arr = np.frombuffer(ptr, dtype=np.uint8).reshape(
                    (self.height, self.width, 4))
                bgr = arr[:, :, :3]
                msg = self.bridge.cv2_to_imgmsg(
                    np.ascontiguousarray(bgr), encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                self.publisher.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish image: {e}")

        if self.fifo_fd is not None:
            data = image.constBits().tobytes()
            try:
                total = 0
                while total < len(data):
                    sent = os.write(self.fifo_fd, data[total:])
                    if sent == 0:
                        break
                    total += sent
            except OSError as e:
                if e.errno == 32:  # EPIPE – reader disconnected
                    self.get_logger().warn("Reader disconnected, waiting...")
                    try:
                        os.close(self.fifo_fd)
                    except OSError:
                        pass
                    self.fifo_fd = None
                elif e.errno != 11:  # ignore EAGAIN
                    self.get_logger().error(f"FIFO write failed: {e}")
                    try:
                        os.close(self.fifo_fd)
                    except OSError:
                        pass
                    self.fifo_fd = None

    def run(self):
        """Run the Qt + ROS event loop and return the exit code."""
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        import threading

        def _ros_spin():
            rclpy.spin(self)

        spin_thread = threading.Thread(target=_ros_spin, daemon=True)
        spin_thread.start()
        self.qt_app.exec()
        return 0


def main(args=None):
    """Entry point for the webscreen node."""
    os.environ['QT_QPA_PLATFORM'] = 'offscreen'
    flags = (
        '--disable-gpu --no-sandbox '
        '--disable-software-rasterizer --single-process'
    )
    os.environ['QTWEBENGINE_CHROMIUM_FLAGS'] = (
        os.environ.get('QTWEBENGINE_CHROMIUM_FLAGS', '') + ' ' + flags
    ).strip()

    rclpy.init(args=args)
    node = WebScreenNode()
    exit_code = node.run()

    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
