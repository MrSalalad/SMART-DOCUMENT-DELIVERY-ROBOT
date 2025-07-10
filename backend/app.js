// backend/app.js
require('dotenv').config();
const express = require('express');
const cors = require('cors');
const path = require('path');
const connectDB = require('./db');

const { User, Station, Room, TransferLog } = require('./Model/models');

const apiRoutes = require('./routes/api'); // ✅ Import route
connectDB(); // ✅ Kết nối MongoDB

const app = express();
const port = process.env.PORT || 3000;

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


app.listen(port, () => {
  console.log(`🌐 Server is running at http://localhost:${port}`);
});
