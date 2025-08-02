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
            if (data.currentLocation) {
              const log = new TransferLog({ source: 'robot', destination: `station ${data.currentLocation}` });
              log.save();
            }
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

        if (data.currentLocation) {
          let locationLabel = `station ${data.currentLocation}`; // default fallback

          // Try to resolve location name from DB
          const station = await Station.findOne({ name: `Station ${data.currentLocation}` });
          const room = await Room.findOne({ name: `${data.currentLocation}` });

          if (station) {
            locationLabel = `station: ${data.currentLocation}`;
          } else if (room) {
            locationLabel = `room: ${data.currentLocation}`;
          }

          // ✅ Save to TransferLog
          const log = new TransferLog({
            source: 'robot',
            destination: `${locationLabel}`,
            timestamp: new Date()
          });

          await log.save();
          console.log('📝 Logged location to DB:', locationLabel);

          // ✅ Emit to frontend
          io.emit('new-log', log);
          console.log('📡 Sent via socket:', locationLabel);
        }
      } catch (e) {
        console.error('❌ MQTT parse error:', e);
      }
    }
  });
}


module.exports = { publishTransfer, receiveLocation, initLocationSocket };
