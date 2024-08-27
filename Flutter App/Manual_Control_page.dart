import 'package:flutter/material.dart';
import 'package:mqtt_client/mqtt_server_client.dart';
import 'package:mqtt_client/mqtt_client.dart';

class ManualControlPage extends StatefulWidget {
  @override
  _ManualControlPageState createState() => _ManualControlPageState();
}

class _ManualControlPageState extends State<ManualControlPage> {
  MqttServerClient? client;
  bool lightStatus = false;
  bool windowStatus = false;
  double fanSpeed = 0.0;

  final String broker = '1e466a5c4df845ad88a6b74159d81393.s1.eu.hivemq.cloud';
  final int port = 8883;
  final String username = 'izzy2';
  final String password = '123456Is';
  final String lightTopic = 'esp32/light';
  final String windowTopic = 'esp32/window';
  final String fanTopic = 'esp32/fan';

  @override
  void initState() {
    super.initState();
    connectToMqtt();
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
      if (client?.connectionStatus?.state == MqttConnectionState.connected) {
        print('Connected to MQTT broker');
      }
    } catch (e) {
      print('Connection failed: $e');
      client!.disconnect();
    }
  }

  void onConnected() {
    print('Connected');
  }

  void onDisconnected() {
    print('Disconnected');
  }

  void onSubscribed(String topic) {
    print('Subscribed to $topic');
  }

  void _toggleLight(bool value) {
    setState(() {
      lightStatus = value;
    });
    _publishMessage(lightTopic, lightStatus ? 'on' : 'off');
  }

  void _toggleWindow(bool value) {
    setState(() {
      windowStatus = value;
    });
    _publishMessage(windowTopic, windowStatus ? 'open' : 'close');
  }

  void _setFanSpeed(double value) {
    setState(() {
      fanSpeed = value;
    });

    String fanMode;
    if (value < 0.33) {
      fanMode = 'off';
    } else if (value < 0.66) {
      fanMode = 'low';
    } else if (value < 1.0) {
      fanMode = 'medium';
    } else {
      fanMode = 'high';
    }

    _publishMessage(fanTopic, fanMode);
  }

  void _publishMessage(String topic, String message) {
    if (client?.connectionStatus?.state == MqttConnectionState.connected) {
      final builder = MqttClientPayloadBuilder();
      builder.addString(message);
      client!.publishMessage(topic, MqttQos.atMostOnce, builder.payload!);
      print('Published message: $message to topic: $topic');
    } else {
      print('MQTT client is not connected');
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Manual Control'),
        backgroundColor: Colors.blueAccent,
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            // Light Control
            ListTile(
              leading: Icon(
                lightStatus ? Icons.lightbulb : Icons.lightbulb_outline,
                color: lightStatus ? Colors.yellow : Colors.grey,
              ),
              title: Text('Light'),
              trailing: Switch(
                value: lightStatus,
                onChanged: _toggleLight,
              ),
            ),
            SizedBox(height: 16),
            // Window Control
            ListTile(
              leading: Icon(
                windowStatus ? Icons.window : Icons.window_outlined,
                color: windowStatus ? Colors.green : Colors.grey,
              ),
              title: Text('Window'),
              trailing: Switch(
                value: windowStatus,
                onChanged: _toggleWindow,
              ),
            ),
            SizedBox(height: 16),
            // Fan Speed Control
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                Icon(Icons.air, color: Colors.blue),
                Text('Fan Speed: ${_fanMode()}'),
                Icon(Icons.wind_power, color: Colors.blue),
              ],
            ),
            Slider(
              value: fanSpeed,
              min: 0,
              max: 1,
              divisions: 3, // 3 divisions for 4 modes
              onChanged: _setFanSpeed,
              activeColor: Colors.blue,
              inactiveColor: Colors.blue.withOpacity(0.3),
            ),
          ],
        ),
      ),
    );
  }

  String _fanMode() {
    if (fanSpeed < 0.33) {
      return 'Off';
    } else if (fanSpeed < 0.66) {
      return 'Low';
    } else if (fanSpeed < 1.0) {
      return 'Medium';
    } else {
      return 'High';
    }
  }

  @override
  void dispose() {
    print('Disposing ManualControlPage');
    client?.disconnect();
    super.dispose();
  }
}
