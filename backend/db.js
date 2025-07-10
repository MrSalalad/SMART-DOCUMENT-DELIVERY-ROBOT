// backend/db.js
require('dotenv').config();
const mongoose = require('mongoose');

const connectDB = async (retries = 5) => {
  while (retries) {
    try {
      if (!process.env.MONGO_URI) {
      console.error('MONGO_URI not found in environment variables');
      process.exit(1);
      } 

      await mongoose.connect(process.env.MONGO_URI, {
        dbName: 'DB'
      });

      console.log('MongoDB connected');
      break;
    } catch (err) {
      console.error('MongoDB connection failed:', err.message);
      retries--;
      console.log(`Retrying... (${retries} retries left)`);
      await new Promise(res => setTimeout(res, 5000)); // đợi 5 giây
    }
  }

  if (!retries) {
    process.exit(1);
  }
};


module.exports = connectDB;