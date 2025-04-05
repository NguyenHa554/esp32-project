#include <PubSubClient.h>
#include "DHT.h"
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>
// #include <LiquidCrystal_I2C.h>
#include <LCD_I2C.h>

#define DHTTYPE DHT11
#define DHTPin 32

#define BUZZER_PIN 26     // Pin connected to the buzzer
#define FLAME_PIN 27        // Pin connected to the flame sensor

#define RELAY_PIN 25    // Pin connected to fan 

DHT dht(DHTPin, DHTTYPE);

LCD_I2C lcd(0x27, 16, 2);

const int MQ_PIN=34;                                //define which analog input channel you are going to use
int RL_VALUE=5;                                     //define the load resistance on the board, in kilo ohms
float RO_CLEAN_AIR_FACTOR=9.83;                     //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                    //which is derived from the chart in datasheet
 
/***********************Software Related Macros************************************/
int CALIBARAION_SAMPLE_TIMES=50;                    //define how many samples you are going to take in the calibration phase
int CALIBRATION_SAMPLE_INTERVAL=500;                //define the time interal(in milisecond) between each samples in the
                                                    //cablibration phase
int READ_SAMPLE_INTERVAL=50;                        //define how many samples you are going to take in normal operation
int READ_SAMPLE_TIMES=5;                            //define the time interal(in milisecond) between each samples in 
                                                    //normal operation
 
/**********************Application Related Macros**********************************/
#define         GAS_LPG             0   
#define         GAS_CO              1   
#define         GAS_SMOKE           2    
 
/*****************************Globals***********************************************/
float           LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
float           COCurve[3]  =  {2.3,0.72,-0.34};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
float           SmokeCurve[3] ={2.3,0.53,-0.44};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms
 /****************** MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/ 
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}
 
/***************************** MQCalibration ****************************************
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use  
         MQResistanceCalculation to calculates the sensor resistance in clean air 
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
         10, which differs slightly between different sensors.
************************************************************************************/ 
float MQCalibration(int mq_pin)
{
  int i;
  float val=0;
 
  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
 
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according to the chart in the datasheet 
 
  return val; 
}
/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/ 
float MQRead(int mq_pin)
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}
 
/*****************************  MQGetGasPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which 
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/ 
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }    
 
  return 0;
}
 
/*****************************  MQGetPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a 
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
         value.
************************************************************************************/ 
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}






// WiFi credentials
const char* ssid = "nguyen";
const char* password = "nguyen68";

// MQTT broker configuration
const char* mqtt_server = "574369a46aef46299f8c76500e62701c.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* MQTT_username = "AZKicute"; // MQTT Username
const char* MQTT_password = "AZsocute1";// MQTT Password

// MQTT client setup
WiFiClientSecure espClient;
PubSubClient client(espClient);

unsigned long previousMillis = 0;

void setup_wifi() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected - ESP IP address: ");
  Serial.println(WiFi.localIP());
}

// Function to reconnect to the MQTT broker
void reconnect() 
{
  while (!client.connected())
   {
    
        String client_id = "esp32-" + String(random(0xffff), HEX);
        Serial.printf("Connecting to MQTT Broker as %s.....\n", client_id.c_str());
        Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(client_id.c_str(), MQTT_username, MQTT_password)) 
    {
      Serial.println("connected\n");
      client.subscribe("esp32");
    } 
    else 
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 5 seconds\n");
      delay(5000);
    }
  }
}

// MQTT callback function for handling incoming messages
void callback(char* topic, byte* payload, unsigned int length) {
  String incomingMessage = "";
  for (int i = 0; i < length; i++) {
    incomingMessage += (char)payload[i];
  }
  Serial.println("Message received [" + String(topic) + "]: " + incomingMessage);
}

// Function to publish messages to MQTT
void publishMessage(const char* topic, String payload, boolean retained) {
  if (client.publish(topic, payload.c_str(), retained)) {
    Serial.println("Message published [" + String(topic) + "]: " + payload);
  } else {
    Serial.println("Failed to publish message");
  }
}
void setup() 
{
  Serial.begin(9600);
  dht.begin();

  ledcAttach(RELAY_PIN, 5000, 8); // connect Fan to PWM channel with 5kHz, 8 bits

  pinMode(MQ_PIN, INPUT);

  Serial.print("Calibrating...\n");                
  Ro = MQCalibration(MQ_PIN);                       //Calibrating the sensor. Please make sure the sensor is in clean air 
                                                    //when you perform the calibration                    
  Serial.print("Calibration is done...\n"); 
  Serial.print("Ro=");
  Serial.print(Ro);
  Serial.print("kohm");
  Serial.print("\n");

  setup_wifi();
  espClient.setInsecure();
  client.setServer(mqtt_server,mqtt_port);
  pinMode(BUZZER_PIN, OUTPUT); // Set the buzzer pin as output
  pinMode(FLAME_PIN, INPUT);   // Set the flame sensor pin as input
}
void loop() {

  if (!client.connected()) {
    reconnect();
  }

  // Maintain MQTT connection and handle incoming messages
  client.loop();

  int gasSensorValue = analogRead(MQ_PIN);
  int flameStatus = digitalRead(FLAME_PIN); // Read the flame sensor status
    
  unsigned long currentMillis = millis();
  // Publishes new temperature and humidity every n seconds
  int second = 5000;
  if (currentMillis - previousMillis > second) 
  {
    previousMillis = currentMillis;

    float humidity = dht.readHumidity();

    // Read temperature as Celsius (the default)
    float temperatureC = dht.readTemperature();

    // Read temperature as Fahrenheit (isFahrenheit = true)
    float temperatureF = dht.readTemperature(true);

    // Read gas info
    float LPG = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG);

  


    // Check if any reads failed and exit early (to try again).
    if (isnan(humidity) || isnan(temperatureC) || isnan(temperatureF)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }

    if (flameStatus == 0 )
   {
    // tone(BUZZER_PIN, 100, 2000);  // Turn on the buzzer
      digitalWrite(BUZZER_PIN, LOW);

    Serial.println("Fire! Fire!");
  }
   else {
    // noTone(BUZZER_PIN);     // Turn off the buzzer
      digitalWrite(BUZZER_PIN, HIGH);

  }


    Serial.print("");
    Serial.print("\nHumidity: ");
    Serial.print(humidity);
    Serial.println(" %");
    Serial.print("Temperature: ");
    Serial.print(temperatureC);
    Serial.println(" ºC");
    Serial.print("    ");   
    Serial.print(temperatureF);
    Serial.println(" ºF");

   Serial.print("LPG:"); 
   Serial.print(LPG);
   Serial.print( "ppm" );
   Serial.print("    ");   
   Serial.print("CO:"); 
   Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO) );
   Serial.print( "ppm" );
   Serial.print("    ");   
   Serial.print("SMOKE:"); 
   Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE) );
   Serial.print( "ppm" );
   Serial.print("\n");
   //note all below to unsend data :v

    // Format data as JSON
    DynamicJsonDocument doc(1024);
    doc["humidity"] = humidity;
    doc["temperatureC"] = temperatureC;
    doc["temperatureF"] = temperatureF;
    doc["LPG"] = LPG;
    doc["flameStatus"] = flameStatus;
    char mqtt_message[128];
    serializeJson(doc, mqtt_message);

    // Publish to MQTT
    publishMessage("esp32", mqtt_message, true);
  }
} 