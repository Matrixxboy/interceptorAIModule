import json
import webbrowser
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
import urllib.parse

CONFIG_PATH = Path("config.json")

HTML_CONTENT = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>JS aerial - Config Builder</title>
    <style>
        :root {
            --bg: #0a0a0a;
            --surface: #1a1a1a;
            --accent: #9d3bf6;
            --text: #ffffff;
            --text-muted: #888888;
        }
        body {
            background-color: var(--bg);
            color: var(--text);
            font-family: 'Segoe UI', Roboto, Helvetica, Arial, sans-serif;
            margin: 0;
            padding: 0;
            display: flex;
            flex-direction: column;
            align-items: center;
        }
        header {
            width: 100%;
            padding: 40px 20px;
            text-align: center;
            background: linear-gradient(180deg, #111 0%, #0a0a0a 100%);
            border-bottom: 1px solid #333;
        }
        .logo-container {
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 10px;
            font-size: 32px;
            font-weight: bold;
            letter-spacing: 1px;
        }
        .logo-dot {
            width: 35px;
            height: 20px;
            background-color: var(--accent);
            border-radius: 10px;
            display: inline-block;
        }
        .subtitle {
            margin-top: 15px;
            font-size: 11px;
            color: var(--text-muted);
            letter-spacing: 2px;
            text-transform: uppercase;
        }
        .container {
            max-width: 800px;
            width: 90%;
            margin: 40px auto;
            background-color: var(--surface);
            padding: 30px;
            border-radius: 12px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.5);
            border: 1px solid #333;
        }
        .section-title {
            color: var(--accent);
            border-bottom: 1px solid #333;
            padding-bottom: 10px;
            margin-bottom: 20px;
            font-size: 18px;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        .grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
            margin-bottom: 30px;
        }
        .form-group {
            display: flex;
            flex-direction: column;
        }
        label {
            font-size: 13px;
            color: #ccc;
            margin-bottom: 8px;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }
        input {
            background-color: #2a2a2a;
            border: 1px solid #444;
            color: white;
            padding: 12px;
            border-radius: 6px;
            font-size: 14px;
            transition: border-color 0.2s;
        }
        input:focus {
            outline: none;
            border-color: var(--accent);
        }
        .actions {
            display: flex;
            justify-content: flex-end;
            gap: 15px;
            margin-top: 20px;
            border-top: 1px solid #333;
            padding-top: 20px;
        }
        button {
            background-color: #333;
            color: white;
            border: none;
            padding: 12px 24px;
            border-radius: 6px;
            cursor: pointer;
            font-weight: bold;
            letter-spacing: 1px;
            transition: all 0.2s;
        }
        button:hover {
            background-color: #444;
        }
        .btn-primary {
            background-color: var(--accent);
        }
        .btn-primary:hover {
            background-color: #b057ff;
            box-shadow: 0 0 15px rgba(157, 59, 246, 0.4);
        }
        #toast {
            position: fixed;
            bottom: 20px;
            right: 20px;
            background-color: var(--accent);
            color: white;
            padding: 15px 25px;
            border-radius: 6px;
            box-shadow: 0 4px 12px rgba(0,0,0,0.3);
            display: none;
            font-weight: bold;
        }
    </style>
</head>
<body>
    <header>
        <div class="logo-container">
            <span class="logo-dot"></span>
            <span>aerial <span style="font-size: 14px; font-weight: normal; color: #888;">SOLUTIONS PVT LTD.</span></span>
        </div>
        <div class="subtitle">DRONE LIGHT SHOW | CUSTOMIZED DRONE BUILD | DRONE SERVICES</div>
    </header>

    <div class="container">
        <div class="section-title">RC Channel Mappings</div>
        <div class="grid">
            <div class="form-group">
                <label>Roll Channel Index</label>
                <input type="number" id="roll_channel" value="0">
            </div>
            <div class="form-group">
                <label>Pitch Channel Index</label>
                <input type="number" id="pitch_channel" value="1">
            </div>
            <div class="form-group">
                <label>Throttle Channel Index</label>
                <input type="number" id="throttle_channel" value="2">
            </div>
            <div class="form-group">
                <label>Yaw Channel Index</label>
                <input type="number" id="yaw_channel" value="3">
            </div>
            <div class="form-group">
                <label>Lock Switch Channel</label>
                <input type="number" id="lock_channel" value="6">
            </div>
            <div class="form-group">
                <label>Follow Switch Channel</label>
                <input type="number" id="follow_channel" value="5">
            </div>
        </div>

        <div class="section-title">Calibration & Endpoints</div>
        <div class="grid">
            <div class="form-group">
                <label>RC Mid (us)</label>
                <input type="number" id="rc_mid" value="1500">
            </div>
            <div class="form-group">
                <label>RC Min (us)</label>
                <input type="number" id="rc_min" value="1000">
            </div>
            <div class="form-group">
                <label>RC Max (us)</label>
                <input type="number" id="rc_max" value="2000">
            </div>
            <div class="form-group">
                <label>Expo (Smoothing)</label>
                <input type="number" step="0.01" id="expo" value="0.85">
            </div>
        </div>
        
        <div class="section-title">Axis Inversions (1.0 or -1.0)</div>
        <div class="grid">
            <div class="form-group">
                <label>Pitch Direction</label>
                <input type="number" step="0.1" id="pitch_dir" value="-1.0">
            </div>
            <div class="form-group">
                <label>Yaw Direction</label>
                <input type="number" step="0.1" id="yaw_dir" value="1.0">
            </div>
        </div>

        <div class="actions">
            <button onclick="loadConfig()">Reload</button>
            <button class="btn-primary" onclick="saveConfig()">Save Configuration</button>
        </div>
    </div>
    
    <div id="toast">Configuration Saved Successfully!</div>

    <script>
        let currentConfig = {};

        async function loadConfig() {
            try {
                const res = await fetch('/api/config');
                const data = await res.json();
                currentConfig = data;
                
                if (data.rc_control) {
                    const rc = data.rc_control;
                    document.getElementById('roll_channel').value = rc.roll_channel;
                    document.getElementById('pitch_channel').value = rc.pitch_channel;
                    document.getElementById('throttle_channel').value = rc.throttle_channel;
                    document.getElementById('yaw_channel').value = rc.yaw_channel;
                    document.getElementById('lock_channel').value = rc.lock_channel;
                    document.getElementById('follow_channel').value = rc.follow_channel;
                    
                    document.getElementById('rc_mid').value = rc.rc_mid;
                    document.getElementById('rc_min').value = rc.rc_min;
                    document.getElementById('rc_max').value = rc.rc_max;
                    document.getElementById('expo').value = rc.expo;
                    document.getElementById('pitch_dir').value = rc.pitch_dir;
                    document.getElementById('yaw_dir').value = rc.yaw_dir;
                }
            } catch (e) {
                console.error("Failed to load config", e);
            }
        }

        async function saveConfig() {
            if (!currentConfig.rc_control) {
                currentConfig.rc_control = {};
            }
            
            const rc = currentConfig.rc_control;
            rc.roll_channel = parseInt(document.getElementById('roll_channel').value);
            rc.pitch_channel = parseInt(document.getElementById('pitch_channel').value);
            rc.throttle_channel = parseInt(document.getElementById('throttle_channel').value);
            rc.yaw_channel = parseInt(document.getElementById('yaw_channel').value);
            rc.lock_channel = parseInt(document.getElementById('lock_channel').value);
            rc.follow_channel = parseInt(document.getElementById('follow_channel').value);
            
            rc.rc_mid = parseInt(document.getElementById('rc_mid').value);
            rc.rc_min = parseInt(document.getElementById('rc_min').value);
            rc.rc_max = parseInt(document.getElementById('rc_max').value);
            rc.expo = parseFloat(document.getElementById('expo').value);
            rc.pitch_dir = parseFloat(document.getElementById('pitch_dir').value);
            rc.yaw_dir = parseFloat(document.getElementById('yaw_dir').value);

            try {
                const res = await fetch('/api/config', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(currentConfig)
                });
                
                if (res.ok) {
                    const toast = document.getElementById('toast');
                    toast.style.display = 'block';
                    setTimeout(() => { toast.style.display = 'none'; }, 3000);
                }
            } catch (e) {
                console.error("Failed to save config", e);
                alert("Error saving configuration!");
            }
        }

        // Load on startup
        window.onload = loadConfig;
    </script>
</body>
</html>
"""

class ConfigHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass # Suppress logs
        
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            self.wfile.write(HTML_CONTENT.encode("utf-8"))
            
        elif self.path == '/api/config':
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.end_headers()
            
            if CONFIG_PATH.exists():
                with open(CONFIG_PATH, "r") as f:
                    data = f.read()
                self.wfile.write(data.encode("utf-8"))
            else:
                # Return empty/default if not found
                from config import SystemConfig
                cfg = SystemConfig()
                self.wfile.write(json.dumps(cfg.to_dict()).encode("utf-8"))
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path == '/api/config':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            
            try:
                data = json.loads(post_data.decode('utf-8'))
                with open(CONFIG_PATH, "w") as f:
                    json.dump(data, f, indent=2)
                    
                self.send_response(200)
                self.send_header("Content-type", "application/json")
                self.end_headers()
                self.wfile.write(b'{"status": "success"}')
            except Exception as e:
                self.send_response(500)
                self.end_headers()
                self.wfile.write(f'{{"error": "{str(e)}" }}'.encode("utf-8"))

def start_server():
    server = HTTPServer(('127.0.0.1', 8080), ConfigHandler)
    print("JS aerial Config Builder running at http://127.0.0.1:8080")
    print("Press Ctrl+C to exit.")
    server.serve_forever()

if __name__ == "__main__":
    # Start server in a daemon thread so it exits when main exits (if we wanted to build a UI)
    # But here we just block and run the server.
    threading.Thread(target=lambda: webbrowser.open("http://127.0.0.1:8080"), daemon=True).start()
    try:
        start_server()
    except KeyboardInterrupt:
        print("\nShutting down config builder.")
