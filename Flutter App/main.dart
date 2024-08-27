import 'package:firebase_core/firebase_core.dart';
import 'package:flutter/material.dart';
import 'package:mqtt_client/mqtt_server_client.dart';
import 'home_page.dart';
import 'login_page.dart';
import 'signuppage.dart';
import 'dashboard_page.dart';
import 'manual_control_page.dart';
import 'game_page.dart';

void main() async {
  WidgetsFlutterBinding.ensureInitialized();

  await Firebase.initializeApp(
    options: const FirebaseOptions(
      apiKey: "AIzaSyBMyI35lyKGfb8pQ4oFzUoHqh6rsDmx6xs",
      projectId: "carehouse-f0492",
      messagingSenderId: "31015981693",
      appId: "1:31015981693:web:a615183ff4cb3a12ad9714",
      databaseURL: "https://carehouse-f0492-default-rtdb.firebaseio.com/",
    ),
  );

  runApp(MyApp());
}

class MyApp extends StatelessWidget {
  final MqttServerClient client = MqttServerClient('broker.hivemq.com', '');

  MyApp() {
    _initializeMqttClient();
  }

  Future<void> _initializeMqttClient() async {
    client.logging(on: true); // Enable logging for debugging
    client.onConnected = () => print('Connected to MQTT broker');
    client.onDisconnected = () => print('Disconnected from MQTT broker');
    client.onSubscribed = (topic) => print('Subscribed to $topic');
    client.onUnsubscribed = (topic) => print('Unsubscribed from $topic');

    try {
      await client.connect();
    } catch (e) {
      print('Exception: $e');
      client.disconnect();
    }
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Caregiver App',
      initialRoute: '/home',
      onGenerateRoute: (settings) {
        switch (settings.name) {
          case '/home':
            return MaterialPageRoute(builder: (_) => HomePage(client: client));
          case '/login':
            return MaterialPageRoute(builder: (_) => LoginPage());
          case '/signUp':
            return MaterialPageRoute(builder: (_) => SignUpPage());
          case '/dashboard':
            return MaterialPageRoute(builder: (_) => DashboardPage());
          case '/manualControl':
            return MaterialPageRoute(builder: (_) => ManualControlPage());
          case '/game':
            return MaterialPageRoute(builder: (_) => GamePage());
          default:
            return null;
        }
      },
    );
  }
}
