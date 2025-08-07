const express = require('express');
const router = express.Router();
const { publishTransfer, receiveLocation} = require('../mqttClient');
const { User, Station, Room, TransferLog } = require('../Model/models');

// ===== LOGIN =====
router.post('/login', async (req, res) => {
  const { username, password } = req.body;
  const user = await User.findOne({ username, password });
  if (user) {
    user.isLoggedIn = true; // ✅ Ghi nhận đã login
    await user.save();
    res.json({ message: 'Login success' });
  } else {
    res.status(401).json({ message: 'Invalid username or password' });
  }
});

// ===== REGISTER =====
router.post('/register', async (req, res) => {
  const { username, email, password } = req.body;
  const exists = await User.findOne({ username });
  if (exists) {
    return res.status(400).json({ message: 'User already exists' });
  }
  const user = new User({ username, email, password });
  await user.save();
  res.json({ message: 'User registered successfully' });
});

// ===== FORGOT PASSWORD =====
router.post('/forgot-password', async (req, res) => {
  const { username, email, password } = req.body;
  const user = await User.findOne({ username, email });
  if (!user) return res.status(404).json({ message: 'User not found' });

  user.password = password;
  await user.save();
  res.json({ message: 'Password updated successfully' });
});

// ===== GET STATIONS =====
router.get('/stations', async (req, res) => {
  const stations = await Station.find();
  res.json(stations);
});

// ===== GET ROOMS =====
router.get('/rooms', async (req, res) => {
  const rooms = await Room.find();
  res.json(rooms.map(r => r.name));
});

// ===== POST TRANSFER LOG =====
router.post('/transfer', async (req, res) => {
  let { source, destination } = req.body;

  const log = new TransferLog({ source, destination });

  if (source == 'Room 1') {
    source = '2';
  } else if (source == 'Room 2') {
    source = '5';
  } else if (source == 'Room 3') {
    source = '6';
  } else if (source == 'Room 4') {
    source = '9';
  }

  if (destination == 'Room 1') {
    destination = '2';
  } else if (destination == 'Room 2') {
    destination = '5';
  } else if (destination == 'Room 3') {
    destination = '6';
  } else if (destination == 'Room 4') {
    destination = '9';
  }

  try {
    publishTransfer(source, destination);
    await log.save();

    const io = req.app.get('io');
    if (!io) {
      console.warn('[TRANSFER] Socket.IO instance is undefined; cannot emit new-log');
    } else {
      console.log('[TRANSFER] Emitting new-log with:', log);
      io.emit('new-log', log);
    }

    res.json({ message: 'Transfer logged', log });
  } catch (err) {
    console.error('Error in /transfer:', err);
    res.status(500).json({ message: 'Internal server error' });
  }
});

// ===== GET TRANSFER LOGS =====
router.get('/logs', async (req, res) => {
  const logs = await TransferLog.find().sort({ timestamp: -1 });
  res.json(logs);
});

// ===== GET CURRENT LOCATION =====
router.get('/location', async (req, res) => {
  try {
    const data = await receiveLocation(); // { currentLocation: ... }
    res.send(`✅ Đã đến vị trí số ${data.currentLocation}`);
  } catch (err) {
    console.error('MQTT error:', err);
    res.status(500).send('❌ Không thể nhận dữ liệu từ MQTT');
  }
});

module.exports = router;
