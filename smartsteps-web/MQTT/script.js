// =====================================
// SmartSteps Dashboard - script.js
// MQTT.js version (no Paho)
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
// MQTT CLIENT USING MQTT.JS
// -------------------------------
console.log("[MQTT] Connecting...");

const client = mqtt.connect(MQTT_BROKER, {
    clientId: "WebDashboard_" + Math.random().toString(16).substr(2, 8),
    clean: true,
    reconnectPeriod: 2000
});

client.on("connect", () => {
    console.log("[MQTT] Connected!");
    ui.status.textContent = "Connected";
    ui.status.style.color = "lime";
    client.subscribe(TOPIC_STEPS);
});

client.on("reconnect", () => {
    ui.status.textContent = "Reconnecting...";
    ui.status.style.color = "orange";
});

client.on("close", () => {
    ui.status.textContent = "Disconnected";
    ui.status.style.color = "red";
});

client.on("error", (err) => {
    console.error("[MQTT] Error:", err);
});

client.on("message", (topic, payload) => {
    const text = payload.toString();
    console.log("[MQTT] Message:", text);

    try {
        const data = JSON.parse(text);
        handleIncoming(data);
    } catch (e) {
        console.error("[MQTT] Invalid JSON:", text);
    }
});

// -------------------------------
// SEND COMMANDS TO ESP32 (MQTT.js)
// -------------------------------
function sendCommand(cmd) {
    if (!client.connected) {
        console.warn("[MQTT] Not connected, cannot send:", cmd);
        return;
    }

    client.publish(TOPIC_CONFIG, cmd, { qos: 1 });
    console.log("[MQTT] Sent:", cmd);
}

// -------------------------------
// CALCULATIONS
// -------------------------------
function calcCalories(steps) {
    return (steps * 0.04).toFixed(1);
}

function calcDistance(steps) {
    return (steps * 0.00078).toFixed(2);
}

let paceHistory = [];
function calcPace(steps) {
    const now = Date.now();
    paceHistory.push({ time: now, steps });

    paceHistory = paceHistory.filter(p => now - p.time <= 60000);

    if (paceHistory.length < 2) return 0;

    return paceHistory[paceHistory.length - 1].steps - paceHistory[0].steps;
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

    ctx.beginPath();
    ctx.strokeStyle = "#ddd";
    ctx.lineWidth = 12;
    ctx.arc(center, center, radius, 0, Math.PI * 2);
    ctx.stroke();

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
    const day = new Date().getDay();
    const index = (day + 6) % 7;
    weeklyData[index] = steps;
    chart.update();
}

// -------------------------------
// HANDLE INCOMING MQTT DATA
// -------------------------------
function handleIncoming(data) {
    const steps = data.steps;
    const goal = data.goal;

    ui.steps.textContent = steps;

    ui.calories.textContent = calcCalories(steps);
    ui.distance.textContent = calcDistance(steps) + " km";
    ui.pace.textContent = calcPace(steps);

    const percent = Math.min(steps / goal, 1);
    drawGoalRing(percent);

    updateWeeklyChart(steps);
}

// -------------------------------
// BUTTON HANDLERS
// -------------------------------
ui.resetBtn.onclick = () => sendCommand("reset");

ui.applyBtn.onclick = () => {
    const goal = ui.goalInput.value;
    const sens = ui.sensInput.value;

    if (goal) sendCommand("goal:" + goal);
    if (sens) sendCommand("sens:" + sens);
};
