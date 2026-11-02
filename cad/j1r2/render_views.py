"""Render (concept, view) PNGs with three.js in headless Chromium.  usage: render_views.py out_dir CID[,CID..] view[,view..]"""
import sys, os, subprocess, time, threading, http.server, socketserver, functools
from playwright.sync_api import sync_playwright
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
class Q(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *a): pass
def main(out, cids, views, port=8765):
    os.makedirs(out, exist_ok=True)
    H = functools.partial(Q, directory=ROOT); socketserver.TCPServer.allow_reuse_address = True
    srv = socketserver.ThreadingTCPServer(("127.0.0.1", port), H); threading.Thread(target=srv.serve_forever, daemon=True).start()
    with sync_playwright() as p:
        b = p.chromium.launch(args=["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist"])
        pg = b.new_page(viewport={"width": 900, "height": 700}); pg.on("console", lambda m: print("console:", m.text) if m.type == "error" else None)
        pg.goto(f"http://127.0.0.1:{port}/render/index.html"); pg.wait_for_function("window.ready === true", timeout=30000)
        for c in cids:
            for v in views:
                vv, gh = (v[:-1], "true") if v.endswith("G") else (v, "false")
                pg.evaluate(f"window.show({repr(c) if c != 'none' else 'null'}, '{vv}', {gh})"); pg.wait_for_timeout(150)
                pg.locator("#c").screenshot(path=os.path.join(out, f"{c}_{v}.png"))
        b.close()
    srv.shutdown()
if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2].split(","), sys.argv[3].split(","))
