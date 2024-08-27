import 'package:flutter/material.dart';
import 'package:mqtt_client/mqtt_server_client.dart';
import 'package:mqtt_client/mqtt_client.dart';

class DashboardPage extends StatefulWidget {
  @override
  _DashboardPageState createState() => _DashboardPageState();
}

class _DashboardPageState extends State<DashboardPage> {
  MqttServerClient? client;
  String statusText = "Disconnected";
  final String broker = '1e466a5c4df845ad88a6b74159d81393.s1.eu.hivemq.cloud';
  final int port = 8883;
  final String username = 'izzy2';
  final String password = '123456Is';
  final List<String> topics = [
    'esp32/temp',
    'esp32/humidity',
    'esp32/heartRate',
    'esp32/light'
  ];
  final String sosTopic = 'esp32/sos'; // Define a topic for SOS messages

  Map<String, String> receivedData = {
    'esp32/temp': '',
    'esp32/humidity': '',
    'esp32/heartRate': '',
    'esp32/light': ''
  };

  @override
  void initState() {
    super.initState();
    connectToMqtt();

    Future.delayed(Duration(seconds: 2), () {
      if (mounted) {
        setState(() {
          statusText = "Connected";
        });
      }
    });
  }

  Future<void> connectToMqtt() async {
    client = MqttServerClient.withPort(broker, '', port);
    client!.logging(on: true);
    client!.secure = true;
    client!.setProtocolV311();
    client!.onConnected = onConnected;
    client!.onDisconnected = onDisconnected;
    client!.onSubscribed = onSubscribed;

    final connMessage = MqttConnectMessage()
        .withClientIdentifier('flutter_client')
        .authenticateAs(username, password)
        .withWillTopic('willtopic')
        .withWillMessage('Disconnected')
        .startClean()
        .withWillQos(MqttQos.atLeastOnce);

    client!.connectionMessage = connMessage;

    try {
      await client!.connect(username, password);
      if (client?.connectionStatus!.state == MqttConnectionState.connected) {
        for (String topic in topics) {
          client!.subscribe(topic, MqttQos.atMostOnce);
        }
      }
    } catch (e) {
      if (mounted) {
        setState(() {
          statusText = "Connection Failed: $e";
        });
      }
      client!.disconnect();
    }

    client!.updates!.listen((List<MqttReceivedMessage<MqttMessage>> c) {
      final MqttPublishMessage recMess = c[0].payload as MqttPublishMessage;
      final String pt =
          MqttPublishPayload.bytesToStringAsString(recMess.payload.message);

      print('Received message: $pt from topic: ${c[0].topic}');

      if (mounted) {
        setState(() {
          if (receivedData.containsKey(c[0].topic)) {
            receivedData[c[0].topic] = pt;
          } else {
            print('Received data for an unknown topic: ${c[0].topic}');
          }
        });
      }
    }).onError((error) {
      print('Error in MQTT updates: $error');
    });
  }

  void onConnected() {
    if (mounted) {
      setState(() {
        statusText = "Connected";
      });
    }
  }

  void onDisconnected() {
    if (mounted) {
      setState(() {
        statusText = "Disconnected";
      });
    }
  }

  void onSubscribed(String topic) {
    if (mounted) {
      setState(() {
        statusText = "Subscribed to $topic";
      });
    }
  }

  void _onSettingsOptionSelected(String option) {
    if (option == 'Log Out') {
      Navigator.of(context)
          .pushReplacementNamed('/home'); // Redirect to HomePage
    } else if (option == 'Manual Control') {
      Navigator.of(context).pushNamed('/manualControl');
    } else if (option == 'Puzzle Game') {
      Navigator.of(context).pushNamed('/game');
    } else if (option == 'SOS') {
      _sendSOSAlert(); // Handle SOS alert
    }
  }

  void _sendSOSAlert() {
    if (client?.connectionStatus!.state == MqttConnectionState.connected) {
      final builder = MqttClientPayloadBuilder();
      builder.addString('Emergency alert triggered');
      client!.publishMessage(sosTopic, MqttQos.exactlyOnce, builder.payload!);

      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('SOS alert sent to emergency contacts!'),
          backgroundColor: Colors.red,
        ),
      );
    } else {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Failed to send SOS alert: Not connected to MQTT'),
          backgroundColor: Colors.red,
        ),
      );
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text("Smart Carehouse Dashboard"),
        actions: [
          PopupMenuButton<String>(
            onSelected: _onSettingsOptionSelected,
            itemBuilder: (BuildContext context) {
              return {'Log Out', 'Manual Control', 'Puzzle Game', 'SOS'}
                  .map((String choice) {
                return PopupMenuItem<String>(
                  value: choice,
                  child: Text(choice),
                );
              }).toList();
            },
          ),
        ],
      ),
      body: Center(
        child: Padding(
          padding: const EdgeInsets.all(8.0),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.center,
            children: [
              Text(
                'Status: $statusText',
                style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
              ),
              SizedBox(height: 20),
              Expanded(
                child: GridView.builder(
                  gridDelegate: SliverGridDelegateWithFixedCrossAxisCount(
                    crossAxisCount: 2,
                    crossAxisSpacing: 8.0,
                    mainAxisSpacing: 8.0,
                    childAspectRatio: 1.5 / 1,
                  ),
                  itemCount: topics.length,
                  itemBuilder: (context, index) {
                    String topic = topics[index];
                    return Card(
                      elevation: 4.0,
                      shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(10.0),
                      ),
                      child: Padding(
                        padding: const EdgeInsets.all(8.0),
                        child: Column(
                          mainAxisAlignment: MainAxisAlignment.center,
                          children: [
                            Icon(
                              getIconForTopic(topic),
                              size: 40.0,
                              color: Colors.blue,
                            ),
                            SizedBox(height: 5.0),
                            Text(
                              topic == 'esp32/light'
                                  ? 'SUNLIGHT'
                                  : topic.split('/').last.toUpperCase(),
                              style: TextStyle(
                                fontSize: 16.0,
                                fontWeight: FontWeight.bold,
                              ),
                              textAlign: TextAlign.center,
                            ),
                            SizedBox(height: 2.0),
                            Expanded(
                              child: Center(
                                child: Text(
                                  receivedData[topic]!.isNotEmpty
                                      ? receivedData[topic]!
                                      : "No data received",
                                  style: TextStyle(
                                    fontSize: 14.0,
                                  ),
                                  textAlign: TextAlign.center,
                                ),
                              ),
                            ),
                          ],
                        ),
                      ),
                    );
                  },
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  IconData getIconForTopic(String topic) {
    switch (topic) {
      case 'esp32/temp':
        return Icons.thermostat;
      case 'esp32/humidity':
        return Icons.water_drop;
      case 'esp32/heartRate':
        return Icons.favorite;
      case 'esp32/light':
        return Icons.light_mode;
      default:
        return Icons.device_unknown;
    }
  }

  @override
  void dispose() {
    print('Disposing DashboardPage');
    super.dispose();
  }
}
