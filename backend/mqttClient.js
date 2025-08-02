const mqtt = require('mqtt');

// Connect to your broker (Mosquitto running on localhost)
//const client = mqtt.connect('mqtt://192.168.25.103:1883');
const client = mqtt.connect('mqtt://broker.hivemq.com');


client.on('connect', () => {
  console.log('✅ MQTT connected');
});

client.on('error', (err) => {
  console.error('❌ MQTT connection error:', err);
});


function publishTransfer(source, destination) {
  const payload = JSON.stringify({ source, destination, timestamp: new Date() });
  client.publish('transfer/documents', payload);
  console.log('📤 Published to MQTT:', payload);
}

function receiveLocation() {
  return new Promise((resolve, reject) => {
    // Subscribe nếu chưa có
    client.subscribe('delivery/location', (err) => {
      if (err) {
        return reject('❌ Subscribe failed: ' + err);
      }

      // Chờ nhận 1 message đầu tiên
      const onMessage = (receivedTopic, message) => {
        if (receivedTopic === 'delivery/location') {
          client.removeListener('message', onMessage); // chỉ nhận 1 lần
          try {
            const data = JSON.parse(message.toString());
            resolve(data); // trả kết quả
          } catch (parseErr) {
            reject('❌ Parse error: ' + parseErr);
          }
        }
      };

      client.on('message', onMessage);
    });
  });
}

function initLocationSocket(io) {
  client.subscribe('delivery/location', (err) => {
    if (err) return console.error('❌ Subscribe error:', err);
    console.log('📡 Subscribed to delivery/location for socket updates');
  });

  client.on('message', (topic, message) => {
    if (topic === 'delivery/location') {
      try {
        const data = JSON.parse(message.toString());
        if (data.currentLocation) {
          io.emit('location-update', data.currentLocation); // ✅ Dùng io đúng cách
          console.log('📡 Sent via socket:', data.currentLocation);
        }
      } catch (e) {
        console.error('❌ MQTT parse error:', e);
      }
    }
  });
}

module.exports = { publishTransfer, receiveLocation, initLocationSocket };
