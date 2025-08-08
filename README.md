# SMART DOCUMENT DELIVERY ROBOT
A smart robot system designed to automatically deliver documents between rooms using an optimized pathfinding algorithm (A*) and an interactive web interface. Built with ESP32, Express.js backend, and a simple HTML frontend, this project allows users to initiate deliveries through buttons or voice commands.

## Features:
Document delivery from room to room
A* pathfinding for optimal navigation
Voice command support for room selection
Web interface to control delivery
Communication between ESP32 and backend (via MQTT)

## Technologies Used:
- Microcontroller: ESP32
- Frontend: HTML, JavaScript
- Backend: Node.js, Express.js
- Pathfinding: A* algorithm
- Communication: MQTT
- Voice Recognition: MFE combined with MobileNeyV1 0.1

## Project Structure:
- backend/: Express.js backend
- frontend/: HTML UI
- esp32/: ESP32 firmware

## Setup Instructions:
- Clone the Repository:
```
git clone https://github.com/MrSalalad/SMART-DOCUMENT-DELIVERY-ROBOT.git
cd SMART-DOCUMENT-DELIVERY-ROBOT
```
- Backend Setup:
```
cd backend
npm install
node app.js
```
(Backend will run at http://localhost:3000)
- Frontend Setup:
```cd ../frontend```
- Open index.html in browser (double-click or use open/start command depending on OS)

## ESP32 Setup:
- Open main.ino inside esp32/ using Arduino IDE or PlatformIO
- Set your WiFi SSID and password
- Connect ESP32 and upload code
- Ensure ESP32 can reach backend (same network, correct IP/port)

##Notes:
- Ensure backend and ESP32 are on same network
- For remote use, expose backend using ngrok or similar tools
- Allow microphone access in browser for voice commands


License: For educational use only. Please credit the github when reusing code.
