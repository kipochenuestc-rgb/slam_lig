from pathlib import Path
import argparse
import webbrowser
import socket
import threading
import time

import uvicorn
from studio_api import app


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=0)
    parser.add_argument("--web", action="store_true", help="仅启动 Web 服务")
    args = parser.parse_args()
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", args.port))
        url = f"http://127.0.0.1:{sock.getsockname()[1]}/"
        server = uvicorn.Server(uvicorn.Config(app, access_log=False))
        print(url, flush=True)
        if args.web:
            server.run(sockets=[sock])
            return
        import webview
        thread = threading.Thread(target=server.run, kwargs={"sockets":[sock]}, daemon=True)
        thread.start()
        try:
            while thread.is_alive() and not server.started:
                time.sleep(.05)
            if not server.started:
                raise RuntimeError("FastAPI 服务启动失败")
            class MapPicker:
                def open_vector_editor(self):
                    return webbrowser.open("https://tools.tier4.jp/vector_map_builder_ll2/", new=2)

                def choose_directory(self):
                    folders = window.create_file_dialog(webview.FileDialog.FOLDER)
                    return folders[0] if folders else None

            window = webview.create_window("ADToolBox", url, js_api=MapPicker(), width=1380, height=900, min_size=(800,600))
            webview.start(icon=str(Path(__file__).resolve().parent / "public" / "toolbox.ico"))
        finally:
            server.should_exit = True
            thread.join(5)


if __name__ == "__main__":
    main()
