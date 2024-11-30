#include"esp_wifi.h"
#include"DHT.h"
#include <WiFi.h>
#include <AHT20.h>
#include <WebServer.h>
#include <Wire.h>
// #include <LiquidCrystal_I2C.h>


const char *ssid = "ACLAB";
const char *password = "ACLAB2023";

WebServer server(80);  // Initialize web server on port 80


//PIN CONFIGURATION

AHT20 dht20;

const int gasSensorPin = 34; // gas sensor pin
const int relayPin = 25;  // relay pin(fan)

// Initialize LCD (16 columns, 2 rows, and I2C address 0x27)
// LiquidCrystal_I2C lcd(0x27, 16, 2);

const int gasThreshHold = 20; // gas level for triggering the relay

bool manualMode = false; // flag to track if the relay is manual or not
bool relayState = false; // State of relay

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Initialize I2C for DHT20

  // ledcSetup(0, 5000, 8)   //PWM Channel 0, 5khz , 8 bits
  ledcAttach(relayPin, 5000, 8); //attach relayPin to PWM 


  pinMode(gasSensorPin, INPUT);

  

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

   // Define URL routes for the web interface
  server.on("/", handleRoot);
  server.on("/toggleManual", toggleManualMode);
  server.on("/toggleRelay", toggleRelayState);

  server.begin();
  Serial.println("HTTP server started");


  delay(2000); // wait 2sec to start processing
}

void loop() {
  // put your main code here, to run repeatedly:
  server.handleClient();  // Handle incoming client requests

  int sensorValue = analogRead(gasSensorPin); //read gas sensor value
  int gas_percentage = map(sensorValue, 0 , 4095, 0 , 100); // convert to percentage (ESP32 has 12 bits ADC)

  float humid = dht20.getHumidity(); // get humidity
  float temperature = dht20.getTemperature();// getTemperature

  if(isnan(humid) && isnan(temperature)){
    Serial.println(F("False to read sensor!"));
    return;
  }

  // Display sensor readings
  Serial.print("Humidity: ");
  Serial.print(humid);
  Serial.println("%");

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("℃");

  Serial.print("Gas sensor value: ");
  Serial.println(sensorValue);

  Serial.print("Percentage: ");
  Serial.print(gas_percentage);
  Serial.println("%");

  Serial.println();

  if(!manualMode){
    
    if(gas_percentage > gasThreshHold){
      relayState = true; // activate relay if gas level exceeds threshhold
      int dutyCycle = map(gas_percentage, gasThreshHold, 100, 128, 255); // adjust PWM duty cycle
      ledcWrite(0, dutyCycle);  // Update duty cycle for PWM
    }
    else{
      relayState = false;
      ledcWrite(0, 0);
    }

    digitalWrite(relayPin, relayState ? HIGH : LOW);
  }

  //display sensor value and relay status on lcd

  // lcd.clear();
  // lcd.setCursor(0, 0);
  // lcd.print("HUM: ");
  // lcd.print(humid);
  // lcd.print("%");

  // lcd.setCursor(0, 1);  
  // lcd.print("TEM: ");
  // lcd.print(temperature);
  // lcd.print("℃");

  // lcd.setCursor(8, 0);
  // lcd.print("Gas: ");
  // lcd.print(gas_percentage);
  // lcd.print("%");

  // lcd.setCursor(8, 1);
  // lcd.print("Relay: ");
  // lcd.print(relayState ? "ON " : "OFF");

  delay(10000); // wait before reading again
}


// Function to handle the main webpage
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><title>ESP32 Gas Sensor</title></head><body>";
  html += "<h1>ESP32 Gas Sensor Control</h1>";
  html += "<p>Humidity: " + String(dht20.getHumidity()) + "%</p>";
  html += "<p>Temperature: " + String(dht20.getTemperature()) + "°C</p>";
  html += "<p>Gas Sensor Level: " + String(analogRead(gasSensorPin)) + "</p>";
  html += "<p>Gas Percentage: " + String(map(analogRead(gasSensorPin), 0, 4095, 0, 100)) + "%</p>";

  // Control section for relay and mode
  html += "<h2>Control Relay</h2>";
  html += "<p>Current Mode: " + String(manualMode ? "Manual" : "Automatic") + "</p>";
  html += "<a href=\"/toggleManual\"><button>Toggle Mode</button></a>";

  if (manualMode) {
    html += "<p>Relay State: " + String(relayState ? "ON" : "OFF") + "</p>";
    html += "<a href=\"/toggleRelay\"><button>Toggle Relay</button></a>";
  }

  html += "</body></html>";

  server.send(200, "text/html", html); // Send HTML response
}

// Function to toggle between manual and automatic modes
void toggleManualMode() {
  manualMode = !manualMode;
  if (!manualMode) { // Return relay to safe state when switching back to auto
    relayState = false;
    ledcWrite(0, 0);
  }
  handleRoot();
}

// Function to toggle relay state (only in manual mode)
void toggleRelayState() {
  if (manualMode) { // Change relay state only if in manual mode
    relayState = !relayState;
    ledcWrite(0, relayState ? 255 : 0);  // Set PWM to max if ON, else turn off
  }
  handleRoot();
}