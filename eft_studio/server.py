import os
import sys
import base64
import json
import time
from http.server import HTTPServer, SimpleHTTPRequestHandler
import socketserver

PORT = 8090
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
RENDERS_DIR = os.path.join(os.path.dirname(BASE_DIR), 'renders')
os.makedirs(RENDERS_DIR, exist_ok=True)

class StudioHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=BASE_DIR, **kwargs)

    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()

    def do_GET(self):
        if self.path == '/api/list_photos':
            photos = []
            for f in sorted(os.listdir(RENDERS_DIR), reverse=True):
                if f.lower().endswith(('.png', '.jpg', '.jpeg', '.webp')):
                    full_path = os.path.join(RENDERS_DIR, f)
                    photos.append({
                        'filename': f,
                        'size': os.path.getsize(full_path),
                        'modified': os.path.getmtime(full_path),
                        'url': f'/renders/{f}'
                    })
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(photos).encode('utf-8'))
            return
        elif self.path.startswith('/renders/'):
            filename = self.path[len('/renders/'):]
            filepath = os.path.join(RENDERS_DIR, filename)
            if os.path.exists(filepath) and os.path.isfile(filepath):
                self.send_response(200)
                if filename.endswith('.png'):
                    self.send_header('Content-Type', 'image/png')
                elif filename.endswith(('.jpg', '.jpeg')):
                    self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(os.path.getsize(filepath)))
                self.end_headers()
                with open(filepath, 'rb') as f:
                    self.wfile.write(f.read())
                return
            else:
                self.send_error(404, 'File not found')
                return

        super().do_GET()

    def do_POST(self):
        if self.path == '/api/save_photo':
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length)
            try:
                data = json.loads(body.decode('utf-8'))
                img_b64 = data.get('image', '')
                if ',' in img_b64:
                    img_b64 = img_b64.split(',', 1)[1]
                
                img_data = base64.b64decode(img_b64)
                custom_name = data.get('filename', f'eft_e610p_{int(time.time())}.png')
                if not custom_name.endswith('.png'):
                    custom_name += '.png'
                
                save_path = os.path.join(RENDERS_DIR, custom_name)
                with open(save_path, 'wb') as f:
                    f.write(img_data)
                
                response = {
                    'status': 'success',
                    'filename': custom_name,
                    'path': save_path,
                    'size': len(img_data),
                    'url': f'/renders/{custom_name}'
                }
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(response).encode('utf-8'))
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'error', 'message': str(e)}).encode('utf-8'))
            return

        self.send_error(404, 'Endpoint not found')

def run(port=PORT):
    for p in range(port, port + 10):
        try:
            socketserver.TCPServer.allow_reuse_address = True
            httpd = socketserver.TCPServer(('0.0.0.0', p), StudioHandler)
            print(f'EFT E610P Digital Twin Studio running on http://localhost:{p}')
            httpd.serve_forever()
            break
        except OSError:
            print(f'Port {p} in use, trying next port...')

if __name__ == '__main__':
    run()
