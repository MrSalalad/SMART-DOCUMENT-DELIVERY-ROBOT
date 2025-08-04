const mongoose = require('mongoose');

const userSchema = new mongoose.Schema({
  username: String,
  email: String,
  password: String,
  isLoggedIn: { type: Boolean, default: false }
});

const stationSchema = new mongoose.Schema({
  name: String,
  max_capacity: Number,
  current_occupation: Number,
});

const roomSchema = new mongoose.Schema({
  name: String,
});

const transferLogSchema = new mongoose.Schema({
  source: String,
  destination: String,
  timestamp: { type: Date, default: Date.now }
});

const User = mongoose.model('User', userSchema);
const Station = mongoose.model('Station', stationSchema);
const Room = mongoose.model('Room', roomSchema);
const TransferLog = mongoose.model('TransferLogs', transferLogSchema);

module.exports = {
  User,
  Station,
  Room,
  TransferLog
};
