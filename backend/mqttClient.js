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

module.exports = { publishTransfer };
