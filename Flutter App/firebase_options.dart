import 'package:firebase_core/firebase_core.dart' show FirebaseOptions;
import 'package:flutter/foundation.dart'
    show defaultTargetPlatform, kIsWeb, TargetPlatform;

class DefaultFirebaseOptions {
  static FirebaseOptions get currentPlatform {
    if (kIsWeb) {
      return web;
    }
    switch (defaultTargetPlatform) {
      case TargetPlatform.android:
        return android;
      case TargetPlatform.iOS:
        throw UnsupportedError(
          'DefaultFirebaseOptions have not been configured for ios - '
          'you can reconfigure this by running the FlutterFire CLI again.',
        );
      case TargetPlatform.macOS:
        throw UnsupportedError(
          'DefaultFirebaseOptions have not been configured for macos - '
          'you can reconfigure this by running the FlutterFire CLI again.',
        );
      case TargetPlatform.windows:
        return windows;
      case TargetPlatform.linux:
        throw UnsupportedError(
          'DefaultFirebaseOptions have not been configured for linux - '
          'you can reconfigure this by running the FlutterFire CLI again.',
        );
      default:
        throw UnsupportedError(
          'DefaultFirebaseOptions are not supported for this platform.',
        );
    }
  }

  static const FirebaseOptions web = FirebaseOptions(
    apiKey: 'AIzaSyBnDVZUzTN2inha_WySg87ExcGXCK3Ng3I',
    appId: '1:394621626934:web:14a0102977882654310697',
    messagingSenderId: '394621626934',
    projectId: 'fir-1-5edfc',
    authDomain: 'fir-1-5edfc.firebaseapp.com',
    storageBucket: 'fir-1-5edfc.appspot.com',
  );

  static const FirebaseOptions android = FirebaseOptions(
    apiKey: 'AIzaSyCqfC4Re4jjriLGNvjcWyB5TCAS_lvzNDc',
    appId: '1:394621626934:android:d259bc6b140c9180310697',
    messagingSenderId: '394621626934',
    projectId: 'fir-1-5edfc',
    storageBucket: 'fir-1-5edfc.appspot.com',
  );

  static const FirebaseOptions windows = FirebaseOptions(
    apiKey: 'AIzaSyBnDVZUzTN2inha_WySg87ExcGXCK3Ng3I',
    appId: '1:394621626934:web:1fda186308536d72310697',
    messagingSenderId: '394621626934',
    projectId: 'fir-1-5edfc',
    authDomain: 'fir-1-5edfc.firebaseapp.com',
    storageBucket: 'fir-1-5edfc.appspot.com',
  );
}
