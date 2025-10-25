🛰️ TWO-DRONE GROUND CONTROL STATION (GCS)

A web-based Ground Control Station for managing and monitoring two Pixhawk-based drones at once.  
You can control, monitor, and upload missions to both drones from a single dashboard.

------------------------------------------------------------
FEATURES
------------------------------------------------------------
- Dual Drone Control: Separate controls for Scanning Drone and Delivery Drone.
- Real-Time Telemetry: Altitude, Speed, GPS position live updates.
- Map Integration: Leaflet maps for drone tracking.
- Manual Control: Arm, Disarm, Takeoff, RTL, and Flight Mode switching.
- Mission Upload: Upload .waypoints or KML files.
- MAVLink Backend: Flask + pymavlink communication with Pixhawks
- CORS-enabled Flask backend for frontend JS requests.

------------------------------------------------------------
SYSTEM ARCHITECTURE
------------------------------------------------------------
Web Dashboard (Flask UI - port 5500)
        |
        +-- Drone Server 1 (Flask - port 5000)
        |     -> Communicates with Pixhawk 1 (/dev/pixhawk1)
        |
        +-- Drone Server 2 (Flask - port 5001)
              -> Communicates with Pixhawk 2 (/dev/pixhawk2)

------------------------------------------------------------
PROJECT STRUCTURE
------------------------------------------------------------
two-drone-gcs/
│
├── static/
│   ├── dashboard.js        (Frontend logic)
│   └── styles.css          (UI styling)
│
├── templates/
│   └── index.html          (Dashboard UI)
│
├── drone1/
|   drone1_server.py        (Flask backend for Drone 1 - port 5000)
├── drone2/
|   drone2_server.py        (Flask backend for Drone 2 - port 5001)
├── main_dashboard.py       (Main Flask dashboard - port 5500)
└── README.md

------------------------------------------------------------
SETUP INSTRUCTIONS
------------------------------------------------------------
1. Install dependencies:
   pip install flask flask-cors pymavlink


2. Start the servers:
   python3 drone1_server.py
   python3 drone2_server.py
   python3 main_dashboard.py

3. Open in your browser:
   http://localhost:5500

------------------------------------------------------------
API ENDPOINTS
------------------------------------------------------------
Each drone server supports:

  GET  /telemetry              - Get current telemetry
  POST /command/<cmd>          - Send command (arm, disarm, rtl, takeoff)
  POST /set_throttle           - Set manual throttle
  POST /upload_file            - Upload mission file

Example:
  curl -X POST http://localhost:5000/command/arm

------------------------------------------------------------
TROUBLESHOOTING
------------------------------------------------------------
- 405 Error: Frontend sending GET instead of POST. Fix fetch() method.
- No telemetry: Check Pixhawk connections and device path.
- Dashboard not updating: Press Ctrl+Shift+R or disable browser cache.

------------------------------------------------------------
LICENSE
------------------------------------------------------------
MIT License - free for modification and use.

------------------------------------------------------------
AUTHOR
------------------------------------------------------------
Developed by Krishiv Nair
