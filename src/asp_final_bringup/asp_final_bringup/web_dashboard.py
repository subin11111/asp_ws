import argparse
import errno
import os
import socketserver
from http.server import SimpleHTTPRequestHandler
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


class DashboardRequestHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()


class DashboardTCPServer(socketserver.TCPServer):
    allow_reuse_address = True


def dashboard_root():
    return Path(get_package_share_directory("asp_final_bringup")) / "web"


def main(args=None):
    parser = argparse.ArgumentParser(description="Serve the ASP final mission dashboard.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8088, type=int)
    parsed, _ = parser.parse_known_args(args)

    os.chdir(dashboard_root())
    try:
        with DashboardTCPServer((parsed.host, parsed.port), DashboardRequestHandler) as httpd:
            print(f"ASP dashboard: http://{parsed.host}:{parsed.port}/", flush=True)
            try:
                httpd.serve_forever()
            except KeyboardInterrupt:
                print("ASP dashboard stopped.", flush=True)
    except OSError as exc:
        if exc.errno == errno.EADDRINUSE:
            print(
                f"ASP dashboard port is already in use: http://{parsed.host}:{parsed.port}/",
                flush=True,
            )
            print("Use another --port value, or stop the existing dashboard process.", flush=True)
            return
        raise
