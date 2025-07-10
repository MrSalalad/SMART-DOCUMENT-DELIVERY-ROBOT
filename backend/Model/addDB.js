// backend/User/addUser.js
require('dotenv').config();
const mongoose = require('mongoose');
const connectDB = require('../db');

// Định nghĩa schema cho collection "users"
const userSchema = new mongoose.Schema({
  _id: mongoose.Schema.Types.String, 
  name: String,
  password: String,
  is_admin: Boolean
}, { versionKey: false });

// Hàm thêm user mới
const addUserDB = async (_id, name, password, is_admin) => {
  try {
    // Tạo model - tham số thứ 3 là collection name "users"
    const User = mongoose.model('User', userSchema, 'users');

    const newUser = new User({ _id, name, password, is_admin });
    await newUser.save();
    
    console.log('✅ User added to DB.users:', newUser);
    return newUser;
  } catch (err) {
    console.error('❌ Error:', err.message);
    throw err;
  }
};

// Định nghĩa schema cho collection "statuses"
const statusSchema = new mongoose.Schema({
  _id: mongoose.Schema.Types.Number,
  name: String,
  type: Number
}, { versionKey: false });

// Hàm thêm status mới
const addStatusDB = async (_id, name, type) => {
  try {
    // Tạo model - tham số thứ 3 là collection name "statuses"
    const Status = mongoose.model('Status', statusSchema, 'statuses');

    const newStatus = new Status({ _id, name, type });
    await newStatus.save();

    console.log('✅ Status added to DB.statuses:', newStatus);
    return newStatus;
  } catch (err) {
    console.error('❌ Error:', err.message);
    throw err;
  }
};

// Định nghĩa schema cho collection "station"
const stationSchema = new mongoose.Schema({
  _id: mongoose.Schema.Types.String,
  max_capacity: Number,
  current_occupation: Number
}, { versionKey: false });

// Hàm thêm station mới
const addStationDB = async (_id, max_capacity, current_occupation) => {
  try {
    // Tạo model - tham số thứ 3 là collection name "stations"
    const Station = mongoose.model('Station', stationSchema, 'stations');

    const newStation = new Station({ _id, max_capacity, current_occupation });
    await newStation.save();

    console.log('✅ Station added to DB.stations:', newStation);
    return newStation;
  } catch (err) {
    console.error('❌ Error:', err.message);
    throw err;
  }
};

// Định nghĩa schema cho collection "room"
const roomSchema = new mongoose.Schema({
  _id: mongoose.Schema.Types.String
}, { versionKey: false });

// Hàm thêm room mới
const addRoomDB = async (_id) => {
  try {
    // Tạo model - tham số thứ 3 là collection name "stations"
    const Room = mongoose.model('Room', roomSchema, 'rooms');

    const newRoom = new Room({ _id });
    await newRoom.save();

    console.log('✅ Room added to DB.rooms:', newRoom);
    return newRoom;
  } catch (err) {
    console.error('❌ Error:', err.message);
    throw err;
  }
};

// Định nghĩa schema cho collection "robot"
const robotSchema = new mongoose.Schema({
  _id: mongoose.Schema.Types.String,
  current_station: { type: mongoose.Schema.Types.String, ref: 'stations' },
  current_room: { type: mongoose.Schema.Types.String, ref: 'rooms' },
  status_id: { type: mongoose.Schema.Types.Number, ref: 'statuses' },
}, { versionKey: false });

// Hàm thêm robot mới
const addRobotDB = async (_id, current_station, current_room, status_id) => {
  try {
    // Tạo model - tham số thứ 3 là collection name "robots"
    const Robot = mongoose.model('Robot', robotSchema, 'robots');

    const newRobot = new Robot({ _id, current_station, current_room, status_id });
    await newRobot.save();

    console.log('✅ Robot added to DB.robots:', newRobot);
    return newRobot;
  } catch (err) {
    console.error('❌ Error:', err.message);
    throw err;
  }
};

const now = new Date();  // Lấy thời gian hiện tại
// Định nghĩa schema cho collection "commands"
const commandSchema = new mongoose.Schema({
  _id: mongoose.Schema.Types.String,  // Sử dụng ObjectId cho _id
  assigned_robot: { type: mongoose.Schema.Types.String, ref: 'robots' },
  started_at: { type: mongoose.Schema.Types.Date, default: Date.now },
  completed_at: { type: mongoose.Schema.Types.Date, default: Date.now },
  source_room: { type: mongoose.Schema.Types.String, ref: 'rooms' },
  destination_room: { type: mongoose.Schema.Types.String, ref: 'rooms' },
  user: { type: mongoose.Schema.Types.String, ref: 'users' },
  status_id: { type: mongoose.Schema.Types.Number, ref: 'statuses' }
}, { versionKey: false });

// Hàm thêm command mới
const addCommandDB = async (_id, assigned_robot, source_room, destination_room, user, status_id) => {
  try {
    // Tạo model - tham số thứ 3 là collection name "commands"
    const Command = mongoose.model('Command', commandSchema, 'commands');

    const newCommand = new Command({ _id, assigned_robot, source_room, destination_room, user, status_id, started_at: now, completed_at: now });
    await newCommand.save();

    console.log('✅ Command added to DB.commands:', newCommand);
    return newCommand;
  } catch (err) {
    console.error('❌ Error:', err.message);
    throw err;
  }
};

const now1 = new Date();  // Lấy thời gian hiện tại
// Định nghĩa schema cho collection "commandLogs"
const commandLogSchema = new mongoose.Schema({
  command_id: { type: mongoose.Schema.Types.String, ref: 'commands' },
  timestamp: Date,
  old_status_id: { type: mongoose.Schema.Types.Number, ref: 'statuses' },
  new_status_id: { type: mongoose.Schema.Types.Number, ref: 'statuses' },
}, { versionKey: false });

// Hàm thêm command log mới
const addCommandLogDB = async (command_id, old_status_id, new_status_id) => {
  try {
    // Tạo model - tham số thứ 3 là collection name "command_logs"
    const CommandLog = mongoose.model('CommandLog', commandLogSchema, 'command_logs');

    const newCommandLog = new CommandLog({ command_id, timestamp: now1, old_status_id, new_status_id });
    await newCommandLog.save();

    console.log('✅ Command Log added to DB.command_logs:', newCommandLog);
    return newCommandLog;
  } catch (err) {
    console.error('❌ Error:', err.message);
    throw err;
  }
};

module.exports = { addUserDB, addStatusDB, addRoomDB, addStationDB, addRobotDB, addCommandDB, addCommandLogDB };  // Export hàm đúng cách