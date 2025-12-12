"""@app.route('/telemetry', methods=['GET'])
def telemetry():
    global latest_status, master

    # --- 1. Handle case where Pixhawk is not connected ---
    if master is None:
        return jsonify({"error": "Pixhawk not connected"}), 503

    try:
        msg = master.recv_match(blocking=True, timeout=3)
    except Exception as e:
        return jsonify({"error": f"Connection error: {str(e)}"}), 500

    # --- 2. Handle NO message received ---
    if msg is None:
        return jsonify({"error": "No telemetry received"}), 504

    mtype = msg.get_type()

    # --- 3. STATUSTEXT message ---
    if mtype == "STATUSTEXT":
        txt = getattr(msg, "text", None)
        if txt:
            latest_status = txt
        return jsonify({"status": latest_status})

    # --- 4. GLOBAL_POSITION_INT message ---
    if mtype == "GLOBAL_POSITION_INT":

        # Gracefully read fields or default them
        lat = getattr(msg, "lat", None)
        lon = getattr(msg, "lon", None)
        alt = getattr(msg, "relative_alt", None)

        # Handle missing lat/lon/alt
        if lat is None or lon is None or alt is None:
            return jsonify({"error": f"Incomplete GPS data: lat={lat}, lon={lon}, alt={alt}"}), 500

        # Convert safely
        try:
            lat = lat / 1e7
            lon = lon / 1e7
            alt = alt / 1000.0
        except:
            return jsonify({"error": "Invalid GPS values"}), 500

        mode = getattr(master, "flightmode", "UNKNOWN")

        return jsonify(latest_telemetry)

    # --- 5. Unknown message types ---
    print("
    return jsonify({"ignored": mtype})
"""
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
"""
def telemetry():
    global latest_status, master
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
            mode = getattr(master, "flightmode", "UNKNOWN")

            return jsonify({
                "lat": lat,
                "lon": lon,
                "alt": alt,
                "status": latest_status,
                "mode": mode
            })

        # Ignore other messages
        return jsonify({"ignored": msg.get_type()})

    except Exception as e:
        return jsonify({"error": str(e)}), 500
"""
