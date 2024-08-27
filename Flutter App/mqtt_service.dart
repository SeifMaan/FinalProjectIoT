import 'package:mqtt_client/mqtt_server_client.dart';

MqttServerClient initializeMqttClient() {
  final client = MqttServerClient.withPort(
    '1e466a5c4df845ad88a6b74159d81393.s1.eu.hivemq.cloud',
    '',
    8883,
  );

  client.logging(on: true);
  client.keepAlivePeriod = 60;
  client.onDisconnected = () {
    print('Disconnected');
  };

  return client;
}
