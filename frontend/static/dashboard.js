// Change these IPs to your two Raspberry Pis
const droneIPs = {
  scan: "http://127.0.0.1:5000", // Drone 1 RPi scan //arsc2//"http://100.110.74.112:5000" jetson ip -http://100.78.63.89:5000
  del:  "http://100.80.4.28:5001"  // Drone 2 RPi del //arsc1 //"http://100.80.4.28:5001" //
};

let scanMap = L.map('scan_map').setView([18.52, 73.85], 8);
let delMap  = L.map('del_map').setView([18.52, 73.85], 8);

L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  attribution: '&copy; OpenStreetMap contributors'
}).addTo(scanMap);

let waypoints = [];
let seq = 0;
let tempMarker = null;

scanMap.on("click", function(e) {

    if (tempMarker) scanMap.removeLayer(tempMarker);

    tempMarker = L.marker([e.latlng.lat, e.latlng.lng]).addTo(scanMap);

    document.getElementById("wpLat").value = e.latlng.lat.toFixed(7);
    document.getElementById("wpLng").value = e.latlng.lng.toFixed(7);

    document.getElementById("wpCurrent").checked = (seq === 0);
    document.getElementById("wpBar").style.display = "flex";

    document.getElementById("saveWpBtn").onclick = function () {

        let command = parseInt(document.getElementById("wpCommand").value);
        let height = parseFloat(document.getElementById("wpHeight").value);
        let delay = parseFloat(document.getElementById("wpDelay").value);
        let auto = document.getElementById("wpAuto").checked ? 1 : 0;
        let current = document.getElementById("wpCurrent").checked ? 1 : 0;

        let lat = parseFloat(document.getElementById("wpLat").value);
        let lng = parseFloat(document.getElementById("wpLng").value);

        let wp = {
            seq: seq,
            frame: 3,
            command: command,
            current: current,
            param1: delay,
            param2: 0,
            param3: 0,
            param4: 0,
            x: lat,
            y: lng,
            z: height,
            autocontinue: auto,
        };

        waypoints.push(wp);
        seq++;

        console.log("Added waypoint:", wp);
        console.log("All:", waypoints);

        document.getElementById("wpBar").style.display = "none";
    };

});

L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  attribution: '&copy; OpenStreetMap contributors'
}).addTo(delMap);

delMap.on('click', function(e) {
    const m = L.marker([e.latlng.lat, e.latlng.lng]).addTo(delMap);

    // remove marker on right click
    m.on('contextmenu', function() {
        delMap.removeLayer(m);
    });

    m.bindPopup("Delivery Location:<br>" + e.latlng.lat + ", " + e.latlng.lng).openPopup();
});

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
    console.log(`${drone}_fileInput`)
    const file = fileInput.files[0];
    if (!file) {
        console.log("file daalo ji")
        return alert(`Please select a mission file first ${drone}_fileInput`);  
    }

    const formData = new FormData();
    formData.append("mission_file", file);

    try {
        let res = await fetch(`${droneIPs[drone]}/upload_file`, { method: "POST", body: formData });
        let data = await res.json();
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
        //section.querySelector(".speed").innerText = data.speed ;
        section.querySelector(".lat").innerText = data.lat;
        section.querySelector(".lon").innerText = data.lon;
        section.querySelector(".altitude").innerText = data.alt;
        section.querySelector(".status").innerText = data.status;
        section.querySelector(".mode").innerText = data.mode;
        section.querySelector(".groundspeed").innerText = data.groundspeed;
        section.querySelector(".lidar").innerText = data.lidar_reading;

    } catch (err) {
        console.error(`Telemetry fetch failed for ${drone}`);
    }
}

// --- Periodic Telemetry Update ---
setInterval(() => {
    getTelemetry("scan");
    getTelemetry("del");
}, 2000);
//-------------------------------------------------------testing--------------------
function waypoint_adder()
{
    console.log("hi i was clicked");
    const newTodo = document.getElementById('new-todo').value;
    if (newTodo.trim() === '') return;
    const li = document.createElement('li');
    li.textContent = newTodo;
    document.getElementById('todo-list').appendChild(li);
    document.getElementById('new-todo').value = '';
}

function exportWP() {

    if (waypoints.length === 0) {
        alert("No waypoints to export.");
        return;
    }

    let text = "";

    text += "QGC WPL 110\n";

    for (let i = 0; i < waypoints.length; i++) {
        let wp = waypoints[i];

        text += `${wp.seq}\t${wp.current}\t${wp.frame}\t${wp.command}\t${wp.param1}\t${wp.param2}\t${wp.param3}\t${wp.param4}\t${wp.x}\t${wp.y}\t${wp.z}\t${wp.autocontinue}\n`;
    }

    // Convert text → downloadable file
    let blob = new Blob([text], { type: "text/plain" });
    let url = URL.createObjectURL(blob);

    let a = document.createElement("a");
    a.href = url;
    a.download = "mission.waypoints";
    a.click();

    URL.revokeObjectURL(url);
};
//-----------------------------------------apangire------------------------------------------------
async function generateMissionFromKML(drone) {

        const missionRes = await fetch(`${droneIPs[drone]}/gen_kml_mission`, {
            method: "POST",
        });


        //Get the mission file as a Blob
        const blob = await missionRes.blob();

        //Create a temporary download link and trigger download
        const url = window.URL.createObjectURL(blob);
        const a = document.createElement("a");
        a.href = url;
        a.download = "mission.waypoints";
        document.body.appendChild(a);
        a.click();

        //Cleanup
        a.remove();
        window.URL.revokeObjectURL(url);

        console.log("Mission downloaded successfully.");
}
async function lawnmower(drone) {
    const fileInput = document.getElementById(`${drone}kml_fileInput`);
    if (!fileInput.files.length) {
        alert("Select a KML file");
        return;
    }
    
    const formData = new FormData();
    formData.append("kml", fileInput.files[0]);

    fetch(`${droneIPs[drone]}/gen_lawnmower`, {
        method: "POST",
        body: formData
    })
    .then(res => {
        if (!res.ok) throw new Error("Generation failed");
        return res.blob();
    })
    .then(blob => {
        const url = window.URL.createObjectURL(blob);
        const a = document.createElement("a");
        a.href = url;
        a.download = "lawnmower.waypoints";
        document.body.appendChild(a);
        a.click();
        a.remove();
    })
    .catch(err => alert(err.message));
}



