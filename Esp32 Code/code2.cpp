#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include "Adafruit_AHTX0.h"
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>

Adafruit_AHTX0 aht;  // temperature sensor object

// pins assigning
const int smokeSensorPin = 35; 
const int buzzerPin = 19;
const int ldrPin = 32;
const int threshold = 2000;

#define HEARTRATE_PIN 34
#define WINDOW_SERVO_PIN 33
#define DOOR_SERVO_PIN 23
#define IN3 2
#define IN4 5
#define ENB 14
#define IR_SENSOR_PIN 13

//keypad initialization and assigning
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}};

    
byte rowPins[ROWS] = {4, 16, 17, 18};  
byte colPins[COLS] = {27, 26, 25, 15}; 
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

const char *correctPassword = "1";  // password for smart door
String enteredPassword = "";

LiquidCrystal_I2C lcd(0x27, 16, 2);  // lcd object

const char *ssid = "isis";
const char *password = "isisssss";

// mqtt configuration
const char *mqttServer = "1e466a5c4df845ad88a6b74159d81393.s1.eu.hivemq.cloud";
const int mqttPort = 8883;
const char *mqttUsername = "izzy2";
const char *mqttPassword = "123456Is";

WiFiClientSecure client;
PubSubClient mqttClient(client);

// initializing servo objects
Servo windowServo;
Servo doorServo;

bool doorOpen = false;
unsigned long doorOpenTime = 0;
const long doorInterval = 5000; 
String fanCommand = "";
bool manualFanControl = false;

// function to turn on fan based on mapped speed from temperature sensor reading
void moveMotorForward(int speed)
{
    analogWrite(ENB, speed); 
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

// Function to stop the fan
void stopMotor()
{
    analogWrite(ENB, 0); // Set speed to 0 (stop)
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

// call back function to handle data from each mqtt topic 
void callback(char *topic, byte *message, unsigned int length)
{
    String command = "";
    for (unsigned int i = 0; i < length; i++)
    {
        command += (char)message[i];
    }
    Serial.print("Message arrived on topic: ");
    Serial.println(topic);
    Serial.print("Message: ");
    Serial.println(command);

    // Handle window servo commands (open, close or automated with ldr sensor)
    if (String(topic) == "esp32/window")
    {
        if (command == "open")
        {
            Serial.println("Opening window...");
            windowServo.write(90);
        }
        else if (command == "close")
        {
            Serial.println("Closing window...");
            windowServo.write(0);
        }
        else
        {
            int pos = command.toInt();
            if (pos >= 0 && pos <= 180)
            {
                Serial.print("Setting window servo to position: ");
                Serial.println(pos);
                windowServo.write(pos);
                if (windowServo.read() == 0)
                {                          // Window is closed
                    digitalWrite(4, HIGH); // Turn LED on
                }
                else
                {
                    digitalWrite(4, LOW); // Turn LED off
                }
            }
            else
            {
                Serial.println("Invalid command received!");
            }
        }
    }

    // Handle fan speed commands
    else if (String(topic) == "esp32/fan")
    {
        manualFanControl = true; // Indicate that manual fan control is active
        fanCommand = command;
        Serial.print("Received fan command: ");
        Serial.println(fanCommand);
    }
}

void handleFan(double temperature)
{
    if (manualFanControl)
    {
        // Handle manual fan commands from MQTT
        if (fanCommand == "low")
        {
            Serial.println("Fan set to low speed");
            moveMotorForward(80);
        }
        else if (fanCommand == "medium")
        {
            Serial.println("Fan set to medium speed");
            moveMotorForward(180);
        }
        else if (fanCommand == "high")
        {
            Serial.println("Fan set to high speed");
            moveMotorForward(255);
        }
        else if (fanCommand == "off")
        {
            Serial.println("Fan turned off");
            stopMotor();
        }
        else
        {
            Serial.println("Invalid fan command");
        }

        // Reset manual control flag
        manualFanControl = false;
        fanCommand = "";
    }
    else
    {
        // Automatic fan control based on temperature
        if (temperature > 32.0)
        {                                                      
            int speed = map(temperature, 32.0, 40.0, 128, 255); // Adjust speed based on temperature
            Serial.print("Temperature high, setting fan speed to ");
            Serial.println(speed);
            moveMotorForward(speed); // Fan turns on with variable speed
        }
        else
        {
            Serial.println("Temperature normal, turning fan off...");
            stopMotor(); // Fan turns off
        }
    }
 
}

void handleSmartDoor()
{
    static bool personDetected = false;

    if (digitalRead(IR_SENSOR_PIN) == HIGH)
    {
        if (!personDetected)
        {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Person detected");
            lcd.setCursor(0, 1);
            lcd.print("Enter password");
            personDetected = true;
        }

        char key = keypad.getKey();
        if (key)
        {
            if (key == '#')
            {
                if (enteredPassword == correctPassword)
                {
                    Serial.println("Password correct! Opening door...");
                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.print("Password correct");
                    lcd.setCursor(0, 1);
                    lcd.print("Opening door...");
                    doorServo.write(90);  // Open door
                    delay(5000);          // Keep door open for 5 seconds
                    doorServo.write(180); // Close door
                    lcd.clear();
                    lcd.print("Door closed");
                }
                else
                {
                    Serial.println("Password incorrect!");
                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.print("Password incorrect");
                }

                enteredPassword = ""; // Reset entered password
            }
            else if (key == '*')
            {
                enteredPassword = ""; // Reset entered password
                lcd.setCursor(0, 1);
                lcd.print("Password reset   ");
            }
            else
            {
                enteredPassword += key; // Append key to entered password
                lcd.setCursor(0, 1);
                lcd.print(enteredPassword);
            }
        }
        delay(200);
    }
    else
    {
        personDetected = false;   // Reset detection flag when person is not detected
        lcd.setCursor(0, 0);
      
    }

    if (doorOpen && (millis() - doorOpenTime >= doorInterval))
    {
        doorServo.write(180); // Close door
        lcd.clear();
        lcd.print("Door closed");
        doorOpen = false;
    }
}

// if smoke level above threshold sends buzz alarm and opens room window
void handleSmokeDetection(int smokeLevel, int lightLevel)
{
    if (smokeLevel > threshold)
    {
        digitalWrite(buzzerPin, HIGH);
        Serial.println("Smoke detected - Alarm triggered");
        windowServo.write(90); // Open window
        if (WiFi.status() == WL_CONNECTED)
        {
            mqttClient.publish("esp32/alarm", "alert");  // publishing alarm message to app
        }
    }
    // if no smoke is detected (normal conditions) window opens and closes based on daylight
    else
    {
        char disString[8];
        dtostrf(lightLevel, 1, 2, disString);
        Serial.print("Publishing sensor data: ");
        Serial.println(lightLevel);

        if (mqttClient.publish("esp32/light", disString))  //publishing light sensor data to process auto opening for window
        {
            Serial.println("Message published successfully.");
        }
        else
        {
            Serial.println("Failed to publish message.");
        }
    }
}

void setupMQTT()
{
    client.setInsecure();
    mqttClient.setServer(mqttServer, mqttPort);
    mqttClient.setCallback(callback);
}

void reconnect()
{
    while (!mqttClient.connected())
    {
        Serial.println("Attempting MQTT connection...");
        if (mqttClient.connect("ESP32Client", mqttUsername, mqttPassword))
        {
            Serial.println("Connected to MQTT broker");

            // Subscribe to all required topics
            mqttClient.subscribe("esp32/window");
            mqttClient.subscribe("esp32/door");
            mqttClient.subscribe("esp32/fan");
            Serial.println("Subscribed to topics: esp32/window, esp32/door, esp32/fan");
        }
        else
        {
            Serial.print("Failed to connect, state: ");
            Serial.println(mqttClient.state());
            delay(2000); // Wait before retrying
        }
    }
}
unsigned long startTime = 0;

// heartrate sensor reading and mapping
void heartrate()
{
    int sensorValue = 0;
    int threshold = 1530; // Initial threshold value
    bool pulseDetected = false;
    unsigned long lastBeatTime = 0;
    unsigned long currentTime;
    float bpm = 0;
    float smoothedBPM = 0;
    float alpha = 0.1;                    // Smoothing factor
    unsigned long debounceInterval = 500; // Debounce interval

    // Variables for 1-minute average calculation
    float bpmSum = 0;
    int readingCount = 0;
    unsigned long interval = 60000;
    sensorValue = analogRead(HEARTRATE_PIN);
    currentTime = millis();


    // Dynamically adjust the threshold based on a weighted average
    threshold = (threshold * 0.95) + (sensorValue * 0.05);

    // Check if the signal crosses the threshold from below (indicating a pulse)
    if (sensorValue > threshold && !pulseDetected)
    {
        unsigned long timeSinceLastBeat = currentTime - lastBeatTime;

        // Debounce: Only count as a valid beat if timeSinceLastBeat is above a minimum interval
        if (timeSinceLastBeat > debounceInterval)
        {
            lastBeatTime = currentTime;
            pulseDetected = true;

            // Calculate BPM (60000 ms per minute)
            bpm = (60000.0 / timeSinceLastBeat) *100;

            // Apply exponential smoothing
            smoothedBPM = (alpha * bpm) + ((1 - alpha) * smoothedBPM) ;

            // Filter out unrealistic BPM values
            if (smoothedBPM > 40 && smoothedBPM < 180)
            {
                // Add to sum and increase the reading count
                bpmSum += smoothedBPM;
                readingCount++;
            }
        }
    }

    // Reset pulseDetected when the signal drops below the threshold
    if (sensorValue < threshold)
    {
        pulseDetected = false;
    }

    // Check if 1 minute has passed
    if (currentTime - startTime >= interval)
    {
        if (readingCount > 0)
        {
            float averageBPM = bpmSum / readingCount;
            Serial.print("Average Heart Rate over 1 minute: ");
            Serial.print(averageBPM);
            Serial.println(" BPM");
        }
        else
        {
            Serial.println("No valid heart rate detected in the last minute.");
        }
        char disString[8];
        dtostrf(smoothedBPM, 1, 2, disString);
        Serial.print("Publishing heart data: ");
        Serial.println(smoothedBPM);

        // Reset for the next 1-minute interval
        bpmSum = 0;
        readingCount = 0;
        startTime = currentTime; // Reset the start time
        if (mqttClient.publish("esp32/heartRate", disString))
        {
            Serial.println("Message published successfully.");
        }
        else
        {
            Serial.println("Failed to publish message.");
        }
    }

    delay(100);
}

void setup()
{
    Serial.begin(115200);
    
    heartrate();
    startTime = millis();
    windowServo.attach(WINDOW_SERVO_PIN);
    doorServo.attach(DOOR_SERVO_PIN);

    pinMode(IR_SENSOR_PIN, INPUT);
    pinMode(ldrPin, INPUT);
    pinMode(buzzerPin, OUTPUT);
    pinMode(smokeSensorPin, INPUT);
    pinMode(HEARTRATE_PIN, INPUT);
 
    lcd.init(); // Initialize the LCD
    lcd.backlight();

    if (!aht.begin())
    {
        Serial.println("Failed to find AHT21B chip");
        while (1)
        {
            delay(10);
        }
    }
    Serial.println("AHT21B Found!");

    Serial.println("Connecting to WiFi...");
    lcd.setCursor(0, 0);
    lcd.print("Connecting WiFi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Connected");

    // Setup MQTT connection
    setupMQTT();

    windowServo.write(0);
    stopMotor();
}

void loop()
{
    if (!mqttClient.connected())
    {
        reconnect();
    }
    mqttClient.loop();
    heartrate();

    int lightlevel = analogRead(ldrPin);
    int smokelevel = analogRead(smokeSensorPin);
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);

    Serial.print("Smoke Level: ");
    Serial.println(smokelevel);

    Serial.print("Light Level: ");
    Serial.println(lightlevel);

    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" degrees C");

    char tempString[8];
    dtostrf(temp.temperature, 1, 2, tempString);
    mqttClient.publish("esp32/temp", tempString);

    Serial.print("Humidity: ");
    Serial.print(humidity.relative_humidity);
    Serial.println("% rH");

    char humidityString[8];
    dtostrf(humidity.relative_humidity, 1, 2, humidityString);
    mqttClient.publish("esp32/humidity", humidityString);

    handleSmokeDetection(smokelevel, lightlevel);
    handleSmartDoor();
    handleFan(temp.temperature);


    delay(1000);
}
