// =====================================
// SmartSteps Dashboard - script.js
// Works with ESP32 MMA7660 Sketch
// Uses Paho MQTT over WebSockets
// =====================================

// -------------------------------
// MQTT CONFIG
// -------------------------------
const MQTT_BROKER = "wss://broker.emqx.io:8084/mqtt";
const TOPIC_STEPS = "smartsteps/steps";
const TOPIC_CONFIG = "smartsteps/config";

// -------------------------------
// UI ELEMENTS
// -------------------------------
const ui = {
    status: document.getElementById("status"),
    steps: document.getElementById("stepCount"),
    goalValue: document.getElementById("goalValue"),
    calories: document.getElementById("calories"),
    distance: document.getElementById("distance"),
    pace: document.getElementById("pace"),
    goalRing: document.getElementById("goalRing"),
    weeklyChart: document.getElementById("weeklyChart"),
    goalInput: document.getElementById("goalInput"),
    sensInput: document.getElementById("sensInput"),
    resetBtn: document.getElementById("resetBtn"),
    applyBtn: document.getElementById("applyBtn")
};

// -------------------------------
// MQTT CLIENT
// -------------------------------
console.log("[MQTT] Connecting...");
const client = new Paho.MQTT.Client(
    "broker.emqx.io",
    8084,
    "/mqtt",
    "WebDashboard_" + Math.random().toString(16).substr(2, 8)
);

client.connect({
    useSSL: true,
    timeout: 5,
    onSuccess: () => {
        console.log("[MQTT] Connected!");
        ui.status.textContent = "Connected";
        ui.status.style.color = "green";
        client.subscribe(TOPIC_STEPS);
    },
    onFailure: () => {
        ui.status.textContent = "Connection Failed";
        ui.status.style.color = "red";
    }
});

client.onConnectionLost = () => {
    ui.status.textContent = "Disconnected";
    ui.status.style.color = "red";
};

client.onMessageArrived = (msg) => {
    console.log("[MQTT] Message:", msg.payloadString);
    handleIncoming(JSON.parse(msg.payloadString));
};

// -------------------------------
// CALCULATIONS
// -------------------------------

// ✅ Calories burned (approx): 0.04 kcal per step
function calcCalories(steps) {
    return (steps * 0.04).toFixed(1);
}

// ✅ Distance (km): average step length 0.78m
function calcDistance(steps) {
    return (steps * 0.00078).toFixed(2);
}

// ✅ Pace (steps/min): based on last 60 seconds
let paceHistory = [];
function calcPace(steps) {
    const now = Date.now();
    paceHistory.push({ time: now, steps });

    // keep last 60 seconds
    paceHistory = paceHistory.filter(p => now - p.time <= 60000);

    if (paceHistory.length < 2) return 0;

    const diff = paceHistory[paceHistory.length - 1].steps - paceHistory[0].steps;
    return diff;
}

// -------------------------------
// GOAL RING DRAWING
// -------------------------------
const ctxRing = ui.goalRing.getContext("2d");

function drawGoalRing(percent) {
    const ctx = ctxRing;
    const radius = 80;
    const center = 90;

    ctx.clearRect(0, 0, 180, 180);

    // background ring
    ctx.beginPath();
    ctx.strokeStyle = "#ddd";
    ctx.lineWidth = 12;
    ctx.arc(center, center, radius, 0, Math.PI * 2);
    ctx.stroke();

    // progress ring
    ctx.beginPath();
    ctx.strokeStyle = "#4CAF50";
    ctx.lineWidth = 12;
    ctx.arc(center, center, radius, -Math.PI / 2, (percent * 2 * Math.PI) - Math.PI / 2);
    ctx.stroke();

    ui.goalValue.textContent = Math.round(percent * 100) + "%";
}

// -------------------------------
// WEEKLY CHART
// -------------------------------
let weeklyData = [0, 0, 0, 0, 0, 0, 0];

const chart = new Chart(ui.weeklyChart, {
    type: "bar",
    data: {
        labels: ["Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"],
        datasets: [{
            label: "Steps",
            data: weeklyData,
            backgroundColor: "#4CAF50"
        }]
    },
    options: {
        responsive: true,
        scales: { y: { beginAtZero: true } }
    }
});

function updateWeeklyChart(steps) {
    const day = new Date().getDay(); // 0 = Sun
    const index = (day + 6) % 7;     // convert to Mon=0
    weeklyData[index] = steps;
    chart.update();
}

// -------------------------------
// HANDLE INCOMING MQTT DATA
// -------------------------------
function handleIncoming(data) {
    const steps = data.steps;
    const goal = data.goal;

    // Update UI
    ui.steps.textContent = steps;

    // Calculations
    ui.calories.textContent = calcCalories(steps);
    ui.distance.textContent = calcDistance(steps) + " km";
    ui.pace.textContent = calcPace(steps);

    // Goal ring
    const percent = Math.min(steps / goal, 1);
    drawGoalRing(percent);

    // Weekly chart
    updateWeeklyChart(steps);
}

// -------------------------------
// SEND COMMANDS TO ESP32
// -------------------------------
function sendCommand(cmd) {
    const msg = new Paho.MQTT.Message(cmd);
    msg.destinationName = TOPIC_CONFIG;
    client.send(msg);
}

// Reset button
ui.resetBtn.onclick = () => {
    sendCommand("reset");
};

// Apply settings
ui.applyBtn.onclick = () => {
    const goal = ui.goalInput.value;
    const sens = ui.sensInput.value;

    if (goal) sendCommand("goal:" + goal);
    if (sens) sendCommand("sens:" + sens);
};
