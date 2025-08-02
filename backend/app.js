// backend/app.js
require('dotenv').config();
const express = require('express');
const cors = require('cors');
const path = require('path');
const http = require('http');
const connectDB = require('./db');
const { Server } = require('socket.io');
const apiRoutes = require('./routes/api');
const { initLocationSocket } = require('./mqttClient');

connectDB(); // ✅ Kết nối MongoDB

const app = express();
const port = process.env.PORT || 3000;

const server = http.createServer(app);
const io = new Server(server); 

initLocationSocket(io)
app.use(cors());
app.use(express.json());

// ✅ Serve static frontend files
const frontendPath = path.join(__dirname, '../frontend/public');
app.use(express.static(frontendPath));

app.get('/', (req, res) => {
  res.sendFile(path.join(frontendPath, 'index.html'));
});

// ✅ Mount API routes
app.use('/api', apiRoutes);

// ✅ API thông báo vị trí
// app.get('/api/location', async (req, res) => {
//   try {
//     if()
//     const data = await receiveLocation(); // ví dụ: { currentLocation: 4 }
//     res.send(`✅ Đã đến vị trí số ${data.currentLocation}`);
//   } catch (err) {
//     res.status(500).send('❌ Không thể nhận dữ liệu từ robot');
//   }
// });

// Route chính phục vụ index.html
app.use((req, res) => {
  const filePath = path.join(__dirname, '../frontend/public/index.html');
  console.log('Phục vụ file:', filePath);
  res.sendFile(filePath);
});

app.use((req, res, next) => {
  console.log(`Request: ${req.method} ${req.url}`);
  next();
});

server.listen(port, () => {
  console.log(`🌐 Server is running at http://localhost:${port}`);
});
