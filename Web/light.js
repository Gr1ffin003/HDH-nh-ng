let myChart1; 
document.addEventListener('DOMContentLoaded', function () {
  const mqttClient = new Paho.MQTT.Client('192.168.137.1', 8080, 'clientId' + new Date().getTime());

  mqttClient.connect({
    onSuccess: function () {
      console.log("✅ Đã kết nối MQTT Broker");
    },
    onFailure: function (responseObject) {
      console.error("❌ Kết nối thất bại: " + responseObject.errorMessage);
    }
  });

  function publishMessage(value) {
    const message = new Paho.MQTT.Message(String(value)); // Gửi giá trị 0 hoặc 1
    message.destinationName = "led"; // topic MQTT
    mqttClient.send(message);
    console.log("📤 Gửi đến topic 'led':", value);
  }

  const toggleSwitch = document.querySelector('.toggle-light');
  const switchStatus = document.querySelector('.switch-status');
  const lightBulb = document.querySelector('.light-bulb');

  if (!toggleSwitch || !switchStatus || !lightBulb) {
    console.error("❗ Không tìm thấy phần tử công tắc LED, status hoặc hình ảnh.");
    return;
  }

  toggleSwitch.addEventListener('change', function () {
    const isOn = this.checked;
    switchStatus.textContent = isOn ? 'ON' : 'OFF';
    lightBulb.src = isOn
      ? 'https://img.icons8.com/ios-filled/50/000000/light-on.png'
      : 'https://img.icons8.com/ios-filled/50/000000/light-off.png';

    publishMessage(isOn ? 1 : 0);
  });

    const style = document.createElement('style');
    style.innerHTML = `
        @keyframes spin {
            0% { transform: rotate(0deg); }
            100% { transform: rotate(360deg); }
        }
    `;
    document.head.appendChild(style);
});

async function fetchLight() {
    try {
      const response = await fetch('/data');
      const data = await response.json();
      document.getElementById('light-value').innerText = data.sensor ?? "No data";
    } catch (err) {
      document.getElementById('light-value').innerText = "Error fetching data";
    }
  }
  
  // Gọi hàm fetchLight mỗi 1 giây
  setInterval(fetchLight, 10000);
  
  // Gọi ngay lúc tải trang
  fetchLight();