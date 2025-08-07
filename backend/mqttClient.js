const mqtt = require('mqtt');
const { Station, Room, TransferLog } = require('./Model/models');


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
  client.on('message', (topic, message) => {
  if (topic === 'delivery/location') {
    try {
      const data = JSON.parse(message.toString());
      if (data.name) {
        console.log('📝 Logged location to DB:', data.name);
        const log = new TransferLog({ source: 'robot', destination: data.name });
        log.save();
        io.emit('location', data.name);
        console.log('📡 Sent via socket:', data.name);
      }
    } catch (err) {
      console.error('❌ Parse error:', err);
    }
  }
});
}

// function initLocationSocket(io) {
//   // Theo dõi kết nối socket nếu cần kiểm soát đăng nhập nâng cao
//   // const userSockets = new Map();

//   client.subscribe('delivery/location', (err) => {
//     if (err) return console.error('❌ Subscribe error:', err);
//     console.log('📡 Subscribed to delivery/location for socket updates');
//   });

//   client.on('message', async (topic, message) => {
//     if (topic === 'delivery/location') {
//       try {
//         const data = JSON.parse(message.toString());

//         if (data.currentLocation) {
//           // ✅ Ghi log vào MongoDB
//           const log = new TransferLog({
//             source: 'robot',
//             destination: `station ${data.currentLocation}`,
//             timestamp: new Date()
//           });
//           await log.save();
//           console.log('📝 Logged location to DB:', data.currentLocation);

//           io.emit('new-log', log);

//           console.log('📡 Sent via socket:', data.currentLocation);
//         }
//       } catch (e) {
//         console.error('❌ MQTT parse error:', e);
//       }
//     }
//   });
// }

function initLocationSocket(io) {
  client.subscribe('delivery/location', (err) => {
    if (err) return console.error('❌ Subscribe error:', err);
    console.log('📡 Subscribed to delivery/location for socket updates');
  });

  client.on('message', async (topic, message) => {
    if (topic === 'delivery/location') {
      try {
        const data = JSON.parse(message.toString());

        if (data.name) {
          let locationLabel = `${data.name}`; // default fallback

          // ✅ Save to TransferLog
          const log = new TransferLog({
            source: 'robot',
            destination: `${locationLabel}`,
            timestamp: new Date()
          });

          await log.save();

          // ✅ Emit to frontend
          io.emit('new-log', log);
          console.log('📡 Current Location:', locationLabel);
        }
      } catch (e) {
        console.error('❌ MQTT parse error:', e);
      }
    }
  });
}


module.exports = { publishTransfer, receiveLocation, initLocationSocket };
