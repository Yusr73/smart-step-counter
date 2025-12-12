import 'package:flutter/material.dart';
import 'package:mqtt_client/mqtt_client.dart';
import 'package:mqtt_client/mqtt_server_client.dart';
import 'dart:async';

void main() {
  runApp(const SmartStepsApp());
}

class SmartStepsApp extends StatelessWidget {
  const SmartStepsApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'SmartSteps',
      theme: ThemeData(
        primarySwatch: Colors.blue,
        useMaterial3: true,
      ),
      home: const SmartStepsHomePage(),
    );
  }
}

class SmartStepsHomePage extends StatefulWidget {
  const SmartStepsHomePage({super.key});

  @override
  State<SmartStepsHomePage> createState() => _SmartStepsHomePageState();
}

class _SmartStepsHomePageState extends State<SmartStepsHomePage> {
  MqttServerClient? client;
  String connectionStatus = 'Déconnecté';
  String stepCount = '0';
  bool isConnecting = false;

  @override
  void initState() {
    super.initState();
    connectToMqtt();
  }

  Future<void> connectToMqtt() async {
    setState(() {
      isConnecting = true;
      connectionStatus = 'Connexion en cours...';
    });

    // Créer le client MQTT
    final clientId = 'mobile-${DateTime.now().millisecondsSinceEpoch}';
    client = MqttServerClient.withPort('broker.emqx.io', clientId, 1883);
    
    // Configuration du client
    client!.logging(on: false);
    client!.keepAlivePeriod = 60;
    client!.onDisconnected = onDisconnected;
    client!.onConnected = onConnected;
    client!.autoReconnect = true;

    // Message de connexion
    final connMessage = MqttConnectMessage()
        .withClientIdentifier(clientId)
        .startClean()
        .withWillQos(MqttQos.atLeastOnce);
    
    client!.connectionMessage = connMessage;

    try {
      await client!.connect();
    } catch (e) {
      setState(() {
        connectionStatus = 'Échec de connexion';
        isConnecting = false;
      });
      client!.disconnect();
      return;
    }

    // Vérifier l'état de connexion
    if (client!.connectionStatus!.state == MqttConnectionState.connected) {
      setState(() {
        connectionStatus = 'Connecté';
        isConnecting = false;
      });

      // S'abonner au topic
      client!.subscribe('smartsteps/steps', MqttQos.atLeastOnce);

      // Écouter les messages
      client!.updates!.listen((List<MqttReceivedMessage<MqttMessage>> messages) {
        final recMessage = messages[0].payload as MqttPublishMessage;
        final payload = MqttPublishPayload.bytesToStringAsString(
          recMessage.payload.message,
        );

        setState(() {
          stepCount = payload;
        });
      });
    } else {
      setState(() {
        connectionStatus = 'Échec de connexion';
        isConnecting = false;
      });
      client!.disconnect();
    }
  }

  void onConnected() {
    setState(() {
      connectionStatus = 'Connecté';
    });
  }

  void onDisconnected() {
    setState(() {
      connectionStatus = 'Déconnecté';
      stepCount = '0';
    });
  }

  @override
  void dispose() {
    client?.disconnect();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('SmartSteps'),
        centerTitle: true,
        elevation: 2,
      ),
      body: Container(
        decoration: BoxDecoration(
          gradient: LinearGradient(
            begin: Alignment.topCenter,
            end: Alignment.bottomCenter,
            colors: [
              Colors.blue.shade50,
              Colors.white,
            ],
          ),
        ),
        child: Center(
          child: Padding(
            padding: const EdgeInsets.all(24.0),
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                // Icône de pas
                Container(
                  padding: const EdgeInsets.all(24),
                  decoration: BoxDecoration(
                    color: Colors.blue.shade100,
                    shape: BoxShape.circle,
                  ),
                  child: Icon(
                    Icons.directions_walk,
                    size: 80,
                    color: Colors.blue.shade700,
                  ),
                ),
                
                const SizedBox(height: 40),

                // Status de connexion
                Container(
                  padding: const EdgeInsets.symmetric(
                    horizontal: 16,
                    vertical: 8,
                  ),
                  decoration: BoxDecoration(
                    color: connectionStatus == 'Connecté'
                        ? Colors.green.shade100
                        : Colors.red.shade100,
                    borderRadius: BorderRadius.circular(20),
                  ),
                  child: Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Container(
                        width: 12,
                        height: 12,
                        decoration: BoxDecoration(
                          color: connectionStatus == 'Connecté'
                              ? Colors.green
                              : Colors.red,
                          shape: BoxShape.circle,
                        ),
                      ),
                      const SizedBox(width: 8),
                      Text(
                        connectionStatus,
                        style: TextStyle(
                          fontSize: 16,
                          fontWeight: FontWeight.w600,
                          color: connectionStatus == 'Connecté'
                              ? Colors.green.shade900
                              : Colors.red.shade900,
                        ),
                      ),
                    ],
                  ),
                ),

                const SizedBox(height: 40),

                // Compteur de pas
                Card(
                  elevation: 4,
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(20),
                  ),
                  child: Container(
                    width: double.infinity,
                    padding: const EdgeInsets.all(32),
                    child: Column(
                      children: [
                        Text(
                          'Nombre de Pas',
                          style: TextStyle(
                            fontSize: 20,
                            color: Colors.grey.shade700,
                            fontWeight: FontWeight.w500,
                          ),
                        ),
                        const SizedBox(height: 16),
                        Text(
                          stepCount,
                          style: TextStyle(
                            fontSize: 72,
                            fontWeight: FontWeight.bold,
                            color: Colors.blue.shade700,
                          ),
                        ),
                      ],
                    ),
                  ),
                ),

                const SizedBox(height: 40),

                // Bouton de reconnexion
                if (connectionStatus != 'Connecté' && !isConnecting)
                  ElevatedButton.icon(
                    onPressed: connectToMqtt,
                    icon: const Icon(Icons.refresh),
                    label: const Text('Reconnecter'),
                    style: ElevatedButton.styleFrom(
                      padding: const EdgeInsets.symmetric(
                        horizontal: 32,
                        vertical: 16,
                      ),
                      shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(12),
                      ),
                    ),
                  ),

                if (isConnecting)
                  const CircularProgressIndicator(),
              ],
            ),
          ),
        ),
      ),
    );
  }
}