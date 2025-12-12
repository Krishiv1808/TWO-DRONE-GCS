from flask import Flask, jsonify, request
from flask_cors import CORS
from pymavlink import mavutil
import time
import os
import glob

app = Flask(__name__)
CORS(app)
CORS(app, resources={r"/*": {"origins": "*"}})


# Connect to Pixhawk
#master = mavutil.mavlink_connection('/dev/ttyACM2', baud=115200)
#master.wait_heartbeat()
def find_pixhawk(baud=57600, timeout=5):
    # Get all possible ACM devices
    ports = glob.glob('/dev/ttyACM*')
    if not ports:
        print("No ACM devices found.",flush=True)
        return None

    for port in ports:
        print(f"Trying {port}...")
        try:
            master = mavutil.mavlink_connection(port, baud=baud)
            # wait for heartbeat for up to `timeout` seconds
            master.wait_heartbeat(timeout=timeout)
            print(f"Connected to Pixhawk on {port}!")
            master.mav.request_data_stream_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                1,  # Rate (Hz)
                1    # Start streaming
            )
            return master
        except Exception as e:
            print(f"Failed to connect on {port}: {e}")
            continue

    print("Could not find Pixhawk on any ACM port.")
    return None
master = find_pixhawk()

#----------------------drone location---------------------------------------------------
@app.route('/telemetry', methods=['GET'])
"""
def telemetry():
    try:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)

        if msg is None:
            # No message received in 2 seconds
            return jsonify({"error": "No telemetry data received"}), 504

        # Some autopilots may send partial messages — check existence
        if not hasattr(msg, 'lat') or not hasattr(msg, 'lon') or not hasattr(msg, 'relative_alt'):
            return jsonify({"error": "Incomplete telemetry data"}), 500

        lat = msg.lat / 1e7  # degrees
        lon = msg.lon / 1e7
        alt = msg.relative_alt / 1000.0  # meters

        return jsonify({
            "lat": lat,
            "lon": lon,
            "alt": alt
        })

    except Exception as e:
        return jsonify({"error": str(e)}), 500
"""
def telemetry():
    global latest_status
    try:
        msg = master.recv_match(blocking=True, timeout=5)

        if msg is None:
            return jsonify({"error": "No data received"}), 504

        # Update status if received
        if msg.get_type() == "STATUSTEXT":
            latest_status = msg.text

        if msg.get_type() == "GLOBAL_POSITION_INT":
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0

            return jsonify({
                "lat": lat,
                "lon": lon,
                "alt": alt,
                "status": latest_status
            })

        # Ignore other messages
        #return jsonify({"ignored": msg.get_type()})

    except Exception as e:
        return jsonify({"error": str(e)}), 500
#===================== ARM / DISARM =============
@app.route('/command/<cmd>', methods=['POST'])
def command(cmd):
    if cmd == "arm":
        master.mav.command_long_send(master.target_system, master.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        master.motors_armed_wait()
        return jsonify({"message": "Drone armed"})
    elif cmd == "disarm":
        master.mav.command_long_send(master.target_system, master.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
        master.motors_disarmed_wait()
        return jsonify({"message": "Drone disarmed"})
    elif cmd == "takeoff":
        master.mav.command_long_send(master.target_system, master.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
        return jsonify({"message": "Takeoff command sent"})
    elif cmd in ["rtl","stabilize", "guided", "auto"]:
        mode_id = master.mode_mapping()[cmd.upper()]
        master.mav.set_mode_send(master.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_id)
        return jsonify({"message": f"Mode set to {cmd.upper()}"})

    else:
        return jsonify({"message": "Invalid command"})
#takeoff---------------------------------------------------------------------------------------------------------------
@app.route('/takeoff', methods=['POST'])
def takeoff():
    data = request.json or {}
    alt = data.get("altitude", 10)  # default 10m

    mode_id = master.mode_mapping()["GUIDED"]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    # Send takeoff command
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt
    )

    return jsonify({"message": f"Takeoff command sent to {alt}m"})


# === MANUAL THROTTLE ===
@app.route('/set_throttle', methods=['POST'])
def set_throttle():
    data = request.json

    # If 'channels' provided, send full RC override
    if 'channels' in data:
        channels = data['channels'] + [0] * (8 - len(data['channels']))  # pad to 8
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            *channels
        )
        return jsonify({"message": f"Full override sent: {channels}"})
    
    # Else just set throttle
    pwm = int(data.get('pwm', 1500))
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1500, 1500, pwm, 1500, 1500, 1500, 1500, 1500
    )
    return jsonify({"message": f"Throttle override sent: {pwm}"})
# ===== HELPER FUNCTIONS =====
def upload_mission(waypoints):
    """
    Uploads a list of waypoints directly to the autopilot
    using pymavlink.
    """
    if not waypoints or len(waypoints) == 0:
        raise ValueError("Waypoint list is empty")

    # Clear any previous mission
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    time.sleep(1)

    # Tell autopilot how many waypoints are coming
    master.mav.mission_count_send(
        master.target_system,
        master.target_component,
        len(waypoints)
    )

    # Loop through each mission item as requested by the autopilot
    for i in range(len(waypoints)):
        msg = master.recv_match(type=['MISSION_REQUEST'], blocking=True, timeout=10)
        if msg is None:
            raise Exception("Timeout: Pixhawk did not request next waypoint")
        seq = msg.seq
        wp = waypoints[seq]

        master.mav.mission_item_send(
            master.target_system,
            master.target_component,
            wp['seq'], wp['frame'], wp['command'],
            wp['current'], wp['autocontinue'],
            wp['param1'], wp['param2'], wp['param3'], wp['param4'],
            wp['x'], wp['y'], wp['z']
        )

    ack = master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=10)
    if ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
        print(f"Successfully uploaded {len(waypoints)} waypoints")
    else:
        raise Exception(f"Mission upload failed: {ack}")
    return True
    
def load_waypoints(filepath):
        """Load waypoints from QGC waypoint file"""
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Mission file not found: {filepath}")

        print(f"Loading mission from: {filepath}")
        waypoints = []

        with open(filepath, 'r') as f:
            first_line = f.readline().strip()
            if not first_line.startswith("QGC WPL"):
                raise ValueError("Invalid mission file format. Must be QGC waypoint file.")

            for line in f:
                parts = line.strip().split('\t')
                if len(parts) == 12:
                    waypoint = {
                        'seq': int(parts[0]),
                        'current': int(parts[1]),
                        'frame': int(parts[2]),
                        'command': int(parts[3]),
                        'param1': float(parts[4]),
                        'param2': float(parts[5]),
                        'param3': float(parts[6]),
                        'param4': float(parts[7]),
                        'x': float(parts[8]),
                        'y': float(parts[9]),
                        'z': float(parts[10]),
                        'autocontinue': int(parts[11])
                    }
                    waypoints.append(waypoint)

        print(f"Loaded {len(waypoints)} waypoints")
        return waypoints
#-----------------------------------------------------------------------------------------

@app.route('/upload_file', methods=['POST'])
def upload_file():
    if 'mission_file' not in request.files:
        return jsonify({"message": "No file uploaded"}), 400

    file = request.files['mission_file']
    if file.filename == '':
        return jsonify({"message": "Empty filename"}), 400

    # Save temporarily
    filepath = os.path.join("/tmp", file.filename)
    file.save(filepath)

    try:
        waypoints = load_waypoints(filepath)
        upload_mission(waypoints)
                                                
    except Exception as e:
        return jsonify({"message": f"Error parsing mission: {str(e)}"}), 400

   
    return jsonify({"message": f"Loaded {len(waypoints)} waypoints from {file.filename}",
                    "waypoints": waypoints})
@app.route('/start_mission', methods=['POST'])
def start_mission():
    
    try:
        mode_id = master.mode_mapping()["GUIDED"]
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
    except Exception as e:
        return jsonify({"message": f"Failed to switch mode to GUIDED: {str(e)}"})

    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,   # confirmation
            1,   # arm
            0, 0, 0, 0, 0, 0
        )
    except Exception as e:
        return jsonify({"message": f"Failed to arm: {str(e)}"})

    
    try:
        mode_id = master.mode_mapping()["AUTO"]
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
    except Exception as e:
        return jsonify({"message": f"Failed to switch mode to AUTO: {str(e)}"})

 
    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0,   # confirmation
            0,   # first mission item index
            0,   # last mission item index (0 = all)
            0, 0, 0, 0, 0
        )
    except Exception as e:
        return jsonify({"message": f"Failed to start mission: {str(e)}"})

    return jsonify({"message": "Mission started successfully"})
#---------------------------------------------------------------------------------------------------------------------------------------
@app.route('/save_waypoints', methods=['POST'])
def save_waypoints():
    data = request.json
    if not data or 'waypoints' not in data:
        return jsonify({"message": "No waypoints provided"}), 400

    waypoints = data['waypoints']
    filepath = "/tmp/generated_mission.waypoints"

    with open(filepath, 'w') as f:
        f.write("QGC WPL 110\n")
        for i, wp in enumerate(waypoints):
            f.write(f"{i}\t0\t3\t16\t0\t0\t0\t0\t{wp['lat']}\t{wp['lon']}\t{wp['alt']}\t1\n")

    print(f"Saved {len(waypoints)} waypoints to {filepath}")
    return jsonify({"message": "Mission file saved", "path": filepath})
#---------------------------------------------------------------------------------------------------------------------------------------
@app.route('/tryconnect')
def tryconnect():
    global master
    master = find_pixhawk()
    if master:
        return jsonify({"status": "connected"})
    else:
        return jsonify({"status": "not found"})
    



if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001)  # Drone 2 del

