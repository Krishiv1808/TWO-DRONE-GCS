// Change these IPs to your two Raspberry Pis
const droneIPs = {
  scan: "http://127.0.0.1:5000", // Drone 1 RPi scan
  del:  "http://127.0.0.1:5001"  // Drone 2 RPi del
};

let scanMap = L.map('scan_map').setView([18.52, 73.85], 8);
let delMap  = L.map('del_map').setView([18.52, 73.85], 8);

L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  attribution: '&copy; OpenStreetMap contributors'
}).addTo(scanMap);

L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  attribution: '&copy; OpenStreetMap contributors'
}).addTo(delMap);

let markers = { scan: null, del: null };

async function updateDronePosition(drone) {
  try {
    const res = await fetch(`${droneIPs[drone]}/telemetry`);
    const data = await res.json();

    //if (!data.lat || !data.lon) return;

    const pos = [data.lat / 1.0, data.lon / 1.0];
    const map = drone === 'scan' ? scanMap : delMap;

    if (!markers[drone]) {
      markers[drone] = L.marker(pos).addTo(map).bindPopup(`${drone} Drone`);
      map.setView(pos, 16);
    } else {
      markers[drone].setLatLng(pos);
    }
  } catch (err) {
    console.error(`${drone} telemetry error:`, err);
  }
   
}

 async function tryconnect(drone) {
      try {
        // call your Flask backend API
        const res = await fetch(`${droneIPs[drone]}/tryconnect`);
        const data = await res.json();
        console.log('Response from Flask:', data);
      } catch (err) {
        console.error('Connection failed:', err);
      }
    }


// --- Commands ---
// --- Drone IP Configuration ---


// --- Utility Function for Status Updates ---
function updateStatus(drone, message) {
    document.getElementById(`${drone}_status`).innerText = message;
}

// --- Load Mission File ---
async function uploadMission(drone) {
    const fileInput = document.getElementById(`${drone}_fileInput`);
    const file = fileInput.files[0];
    if (!file) return alert("Please select a mission file first");

    const formData = new FormData();
    formData.append("mission_file", file);

    try {
        const res = await fetch(`${droneIPs[drone]}/upload_file`, { method: "POST", body: formData });
        const data = await res.json();
        updateStatus(drone, data.message);
    } catch (err) {
        console.error(err);
        updateStatus(drone, "Mission upload failed!");
    }
}

// --- Send Throttle ---
async function sendThrottle(drone, kill = false) {
    let bodyData;

    if (kill) {
        // Full motor stop
        bodyData = { channels: [1100, 1100, 1100, 1100, 1100, 1100, 1100, 1100] };
    } else {
        const value = document.getElementById(`${drone}_throttleInput`).value;
        if (!value) return alert("Enter throttle value first!");
        bodyData = { pwm: parseInt(value) };
    }

    try {
        const res = await fetch(`${droneIPs[drone]}/set_throttle`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(bodyData)
        });
        const data = await res.json();
        updateStatus(drone, data.message);
    } catch (err) {
        console.error(err);
        updateStatus(drone, "Throttle command failed!");
    }
}

// --- Send Mode or Arm/Disarm Command ---
async function sendCommand(drone, cmd) {
    try {
        const res = await fetch(`${droneIPs[drone]}/command/${cmd}`, { method: "POST" });
        const data = await res.json();
        updateStatus(drone, data.message);
    } catch (err) {
        console.error(err);
        updateStatus(drone, "Command failed!");
    }
}

// --- Takeoff Command ---
async function sendTakeoff(drone) {
    const alt = document.getElementById(`${drone}_takeoff_height`).value || 10;

    try {
        const res = await fetch(`${droneIPs[drone]}/takeoff`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ altitude: parseFloat(alt) })
        });
        const data = await res.json();
        updateStatus(drone, data.message);
    } catch (err) {
        console.error(err);
        updateStatus(drone, "Takeoff failed!");
    }
}

// --- Start Mission ---
async function startMission(drone) {
    try {
        const res = await fetch(`${droneIPs[drone]}/start_mission`, { method: "POST" });
        const data = await res.json();
        updateStatus(drone, data.message);
    } catch (err) {
        console.error(err);
        updateStatus(drone, "Mission start failed!");
    }
}

// --- Telemetry Fetch Loop ---
async function getTelemetry(drone) {
    try {
        const res = await fetch(`${droneIPs[drone]}/telemetry`);
        const data = await res.json();

        const section = document.querySelector(`#${drone}_map`).parentElement;
        section.querySelector(".status").innerText = data.status;
        section.querySelector(".altitude").innerText = data.alt;
        section.querySelector(".speed").innerText = data.speed ;
        section.querySelector(".lat").innerText = data.lat;
        section.querySelector(".lon").innerText = data.lon;

    } catch (err) {
        console.error(`Telemetry fetch failed for ${drone}`);
    }
}

// --- Periodic Telemetry Update ---
/*setInterval(() => {
    getTelemetry("scan");
    getTelemetry("del");
}, 2000);*/

