// Connect using WebSockets over port 443
const client = new Paho.MQTT.Client(
    "broker.emqx.io",
    443,
    "/mqtt",
    "webclient-" + Math.random().toString(16).substr(2, 8)
);

client.onConnectionLost = () => {
    document.getElementById("status").textContent = "Disconnected";
};

client.onMessageArrived = (message) => {
    document.getElementById("stepCount").textContent = message.payloadString;
};

client.connect({
    useSSL: true,
    onSuccess: () => {
        document.getElementById("status").textContent = "Connected";
        client.subscribe("smartsteps/steps");
    },
    onFailure: () => {
        document.getElementById("status").textContent = "Connection failed";
    }
});
