// =====================================
// SmartSteps Dashboard - script.js
// MQTT.js version (no Paho)
// + Prediction (activity) support from VM
// =====================================

// -------------------------------
// MQTT CONFIG
// -------------------------------
const MQTT_BROKER = "wss://broker.emqx.io:8084/mqtt";

// Existing topics
const TOPIC_STEPS = "smartsteps/steps";
const TOPIC_CONFIG = "smartsteps/config";

// NEW: predictions published by VM inference service
// expected: smartsteps/<deviceId>/pred
const TOPIC_PRED_SUB = "smartsteps/+/pred";

// Optional: if you want to force showing only one device, set it here.
// Leave null to accept any device.
const FILTER_DEVICE_ID = null; // e.g. "esp32_01"

// Labels for prediction classes (match your training order!)
const PRED_LABELS = ["activity_0", "activity_1", "activity_2", "activity_3", "activity_4", "activity_5"];

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
  applyBtn: document.getElementById("applyBtn"),
  batteryValue: document.getElementById("batteryValue"),
  batteryStatus: document.getElementById("batteryStatus"),
  achievementBadge: document.getElementById("achievementBadge"),

  // NEW: will be created dynamically (see ensureActivityWidget)
  activityLine: null,
};

// -------------------------------
// Create "Activity" line dynamically
// -------------------------------
function ensureActivityWidget() {
  if (ui.activityLine) return;

  const statusBar = document.querySelector(".status-bar");
  if (!statusBar) return;

  const activity = document.createElement("div");
  activity.id = "activityLine";
  activity.style.fontSize = "14px";
  activity.style.fontWeight = "600";
  activity.style.opacity = "0.9";
  activity.style.marginTop = "8px";
  activity.style.width = "100%";
  activity.textContent = "Activity: --";

  // Put it just under the status bar
  // (statusBar is flex; easiest is insert after it)
  statusBar.parentNode.insertBefore(activity, statusBar.nextSibling);

  ui.activityLine = activity;
}

function setActivityText(text) {
  ensureActivityWidget();
  if (ui.activityLine) ui.activityLine.textContent = text;
}

// -------------------------------
// MQTT CLIENT USING MQTT.JS
// -------------------------------
console.log("[MQTT] Connecting...");

const client = mqtt.connect(MQTT_BROKER, {
  clientId: "WebDashboard_" + Math.random().toString(16).substr(2, 8),
  clean: true,
  reconnectPeriod: 2000,
});

client.on("connect", () => {
  console.log("[MQTT] Connected!");
  ui.status.textContent = "Connected";
  ui.status.style.color = "lime";

  client.subscribe(TOPIC_STEPS);
  client.subscribe(TOPIC_PRED_SUB);

  ensureActivityWidget();
});

client.on("reconnect", () => {
  ui.status.textContent = "Reconnecting...";
  ui.status.style.color = "orange";
});

client.on("close", () => {
  ui.status.textContent = "Disconnected";
  ui.status.style.color = "red";
  ui.achievementBadge.style.display = "none";
  ui.batteryValue.textContent = "--%";
  ui.batteryStatus.className = "battery-status";
  setActivityText("Activity: --");
});

client.on("error", (err) => {
  console.error("[MQTT] Error:", err);
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
  paceHistory = paceHistory.filter((p) => now - p.time <= 60000);
  if (paceHistory.length < 2) return 0;
  return paceHistory[paceHistory.length - 1].steps - paceHistory[0].steps;
}

// -------------------------------
// BATTERY STATUS UPDATE
// -------------------------------
function updateBatteryStatus(batteryPercent) {
  ui.batteryValue.textContent = `${batteryPercent}%`;

  if (batteryPercent <= 20) {
    ui.batteryStatus.className = "battery-status low";
  } else if (batteryPercent <= 50) {
    ui.batteryStatus.className = "battery-status medium";
  } else {
    ui.batteryStatus.className = "battery-status high";
  }
}

// -------------------------------
// GOAL ACHIEVEMENT BADGE
// -------------------------------
function updateGoalAchievement(goalReachedToday) {
  ui.achievementBadge.style.display = goalReachedToday ? "block" : "none";
}

// -------------------------------
// GOAL RING DRAWING
// -------------------------------
const ctxRing = ui.goalRing ? ui.goalRing.getContext("2d") : null;

function drawGoalRing(percent) {
  const ctx = ctxRing;
  if (!ctx) return; // canvas/context unavailable

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
  ctx.arc(center, center, radius, -Math.PI / 2, percent * 2 * Math.PI - Math.PI / 2);
  ctx.stroke();

  if (ui.goalValue) ui.goalValue.textContent = Math.round(percent * 100) + "%";
}

// -------------------------------
// WEEKLY CHART
// -------------------------------
let weeklyData = [0, 0, 0, 0, 0, 0, 0];

let chart = null;
if (typeof Chart !== "undefined" && ui.weeklyChart) {
  chart = new Chart(ui.weeklyChart, {
    type: "bar",
    data: {
      labels: ["Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"],
      datasets: [
        {
          label: "Steps",
          data: weeklyData,
          backgroundColor: "#4CAF50",
        },
      ],
    },
    options: {
      responsive: true,
      scales: { y: { beginAtZero: true } },
    },
  });
} else {
  console.warn("[Chart] Chart.js not available or weeklyChart canvas missing");
}

function updateWeeklyChart(steps) {
  const day = new Date().getDay();
  const index = (day + 6) % 7;
  weeklyData[index] = steps;
  if (chart && typeof chart.update === "function") chart.update();
}

// -------------------------------
// HANDLE INCOMING STEPS DATA
// -------------------------------
function handleIncomingSteps(data) {
  const steps = data.steps || 0;
  const goal = data.goal || 6000;
  const battery = data.battery ?? 100;
  const goalReachedToday = data.goalReachedToday || false;

  ui.steps.textContent = steps.toLocaleString();

  ui.calories.textContent = calcCalories(steps);
  ui.distance.textContent = calcDistance(steps) + " km";
  ui.pace.textContent = calcPace(steps);

  updateBatteryStatus(battery);
  updateGoalAchievement(goalReachedToday);

  const percent = Math.min(steps / goal, 1);
  drawGoalRing(percent);

  updateWeeklyChart(steps);
}

// -------------------------------
// HANDLE INCOMING PREDICTIONS
// Topic: smartsteps/<deviceId>/pred
// Payload: {"class": <int>, "score": <float>}
// -------------------------------
function parseDeviceIdFromPredTopic(topic) {
  // matches smartsteps/<deviceId>/pred
  const m = topic.match(/^smartsteps\/([^/]+)\/pred$/);
  return m ? m[1] : null;
}

function handleIncomingPred(topic, data) {
  const deviceId = parseDeviceIdFromPredTopic(topic) || "unknown";

  if (FILTER_DEVICE_ID && deviceId !== FILTER_DEVICE_ID) return;

  const idx = Number.isFinite(Number(data["class"])) ? Number(data["class"]) : -1;
  const score = Number.isFinite(Number(data["score"])) ? Number(data["score"]) : 0;

  const label = (idx >= 0 && idx < PRED_LABELS.length) ? PRED_LABELS[idx] : `class_${idx}`;
  const pct = Math.max(0, Math.min(1, score)) * 100;

  // Show it
  setActivityText(`Activity (${deviceId}): ${label} (${pct.toFixed(1)}%)`);
  console.log("[PRED]", deviceId, label, score);
}

// -------------------------------
// MQTT MESSAGE ROUTER
// -------------------------------
client.on("message", (topic, payload) => {
  let text = "";

  // Decode payload safely for browser/Node (Uint8Array, Buffer, string)
  try {
    if (typeof payload === "string") {
      text = payload;
    } else if (payload instanceof Uint8Array) {
      text = new TextDecoder().decode(payload);
    } else if (payload && typeof payload.toString === "function") {
      text = payload.toString();
    } else {
      text = String(payload);
    }
  } catch (decErr) {
    console.error("[MQTT] Could not decode payload:", decErr, payload);
    return;
  }

  try {
    const data = JSON.parse(text);

    if (topic === TOPIC_STEPS) {
      handleIncomingSteps(data);
      return;
    }

    // predictions
    if (topic.startsWith("smartsteps/") && topic.endsWith("/pred")) {
      handleIncomingPred(topic, data);
      return;
    }

    // ignore other topics
  } catch (e) {
    console.error("[MQTT] Invalid JSON:", topic, text);
  }
});

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

// -------------------------------
// INITIALIZATION
// -------------------------------
document.addEventListener("DOMContentLoaded", () => {
  ui.batteryValue.textContent = "--%";
  ui.batteryStatus.className = "battery-status";
  if (ui.achievementBadge) ui.achievementBadge.style.display = "none";

  ensureActivityWidget();
  setActivityText("Activity: --");
});
