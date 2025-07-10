const express = require('express');
const router = express.Router();
const { User, Station, Room, TransferLog } = require('../Model/models');

// ===== LOGIN =====
router.post('/login', async (req, res) => {
  const { username, password } = req.body;
  const user = await User.findOne({ username, password });
  if (user) {
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
  const { source, destination } = req.body;
  const log = new TransferLog({ source, destination });
  await log.save();
  res.json({ message: 'Transfer logged', log });
});

// ===== GET TRANSFER LOGS =====
router.get('/logs', async (req, res) => {
  const logs = await TransferLog.find().sort({ timestamp: -1 });
  res.json(logs);
});

module.exports = router;
