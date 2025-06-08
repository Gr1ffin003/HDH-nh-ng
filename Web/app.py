import paho.mqtt.client as mqtt
import json
from flask import Flask, jsonify, render_template
import threading

app = Flask(__name__)

latest_light = None  # lưu giá trị ánh sáng mới nhất

# Callback khi kết nối MQTT thành công
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("✅ Connected to MQTT Broker!")
        client.subscribe("sensor")
    else:
        print(f"❌ Connection failed with code {rc}")

# Callback khi nhận được tin nhắn MQTT
def on_message(client, userdata, msg):
    global latest_light
    try:
        payload = msg.payload.decode().strip()
        print(f"📥 Raw payload: {payload}")
        data = json.loads(payload)

        if isinstance(data, dict):
            latest_light = data.get("light", None)
        elif isinstance(data, (int, float)):
            latest_light = data  # nếu chỉ nhận giá trị số đơn giản
        else:
            print(f"⚠️ Không rõ định dạng data: {type(data)}")

        print(f"💡 Updated light intensity: {latest_light}")
    except Exception as e:
        print(f"❗ Error processing message: {e}")

# Khởi tạo client MQTT
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect("192.168.137.1", 1883, 60)  # đổi IP/port broker nếu cần

# Chạy MQTT loop trong thread riêng để không block Flask
def mqtt_loop():
    client.loop_forever()

threading.Thread(target=mqtt_loop, daemon=True).start()

# API trả dữ liệu ánh sáng
@app.route("/data")
def get_data():
    return jsonify({"sensor": latest_light})
@app.route("/")
def index():
    return render_template("index.html")

if __name__ == "__main__":
    # Chạy Flask trên tất cả IP, port 5000
    app.run(host="0.0.0.0", port=5000)
