#!/usr/bin/env python3
import json
import os
import sys
import time
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer


CMD_FILE = "lidar_command.json"


class LidarHandler(SimpleHTTPRequestHandler):
    def _send_json(self, status: int, payload: dict):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if self.path != "/cmd":
            self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "unknown endpoint"})
            return

        content_length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(content_length)

        try:
            payload = json.loads(raw.decode("utf-8"))
            x = float(payload["x"])
            y = float(payload["y"])
            cmd_id = int(payload.get("id", int(time.time() * 1000)))
        except Exception as exc:
            self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": f"invalid payload: {exc}"})
            return

        cmd = {"id": cmd_id, "x": x, "y": y, "type": "goto"}
        tmp_path = CMD_FILE + ".tmp"

        try:
            with open(tmp_path, "w", encoding="utf-8") as tmp:
                json.dump(cmd, tmp)
            os.replace(tmp_path, CMD_FILE)
        except Exception as exc:
            self._send_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"ok": False, "error": f"write failed: {exc}"})
            return

        self._send_json(HTTPStatus.OK, {"ok": True, "id": cmd_id})


def main():
    port = 8089
    if len(sys.argv) >= 2:
        port = int(sys.argv[1])

    server = ThreadingHTTPServer(("", port), LidarHandler)
    print(f"🌐 Serveur lidar_visual sur http://localhost:{port}/lidar_viewer.html")
    print("   Endpoint commande: POST /cmd")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
