#include <WiFiManager.h> // For managing Wi-Fi credentials
#include <PubSubClient.h> // For MQTT
#include <IRremoteESP8266.h>
#include <WiFi.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include <FastLED.h>
#include <IRsend.h>
#include <IRrecv.h>
#include <IRutils.h>

// IR REMOTE PROTOCOL //
#include <ir_Airton.h>
#include <ir_Airwell.h>
#include <ir_Amcor.h>
#include <ir_Argo.h>
#include <ir_Bosch.h>
#include <ir_Carrier.h>
#include <ir_Coolix.h>
#include <ir_Corona.h>
#include <ir_Daikin.h>
#include <ir_Daikin.h>
#include <ir_Ecoclim.h>
#include <ir_Goodweather.h>
#include <ir_Gree.h>
#include <ir_Kelon.h>
#include <ir_Kelvinator.h>
#include <ir_Lg.h>
#include <ir_Magiquest.h>
#include <ir_Midea.h>
#include <ir_Mirage.h>
#include <ir_Mitsubishi.h>
#include <ir_Nec.h>
#include <ir_Neoclima.h>
#include <ir_Panasonic.h>
#include <ir_Rhoss.h>
#include <ir_Samsung.h>
#include <ir_Sanyo.h>
#include <ir_Sharp.h>
#include <ir_Teco.h>
#include <ir_Tcl.h>
#include <ir_Transcold.h>
#include <ir_Trotec.h>
#include <ir_Truma.h>
#include <ir_Voltas.h>
#include <ir_York.h>




#define WORK_PACKAGE                                "1209"
#define DEVICE_TYPE                                 "00"
#define DEVICE_CODE_UPLOAD_DATE                     "240912"
#define DEVICE_SERIAL_ID                            "0000"

#define DEVICE_ID                                   WORK_PACKAGE DEVICE_TYPE DEVICE_CODE_UPLOAD_DATE DEVICE_SERIAL_ID
//#define SUB_TOPIC_B                               ("A/C/" DEVICE_ID "/D")

// MQTT topics
#define AIRFLOW_SENSOR_TOPIC                        ("DMA/AC/" DEVICE_ID "/AIRFLOW")         //PUBLISH
#define AMBIENT_SENSOR_TOPIC                        ("DMA/AC/" DEVICE_ID "/AMBIENT")         //PUBLISH
#define STATUS_TOPIC                                ("DMA/AC/" DEVICE_ID "/STATUS")          //PUBLISH
#define REMOTE_TOPIC                                ("DMA/AC/" DEVICE_ID "/REMOTE")          //PUBLISH
#define CONTROL_TOPIC                               ("DMA/AC/" DEVICE_ID "/CONTROL")         //SUBSCRIBE


// Define the number of LEDs and the data pin
#define NUM_LEDS 1
#define DATA_PIN 4

Adafruit_SHT31 AirFlow = Adafruit_SHT31();  // Sensor 1
Adafruit_SHT31 Ambient = Adafruit_SHT31();  // Sensor 2


// Create an array of LEDs
CRGB leds[NUM_LEDS];

// IR setup &&&& PIN
const uint16_t kRecvPin = 36;
const uint16_t kIrLedPin = 27; // GPIO for IR LED
const int LED_PIN = 2; // LED status indicator
const int RESET_BUTTON_PIN = 35; // Wi-Fi reset button
int count = 0;//     loop count

const uint16_t kCaptureBufferSize = 1024;
const uint8_t kTimeout = 50;
const uint8_t kTolerancePercentage = 25; // Default tolerance of 25%

IRsend irsend(kIrLedPin);
IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, true);
decode_results results;

// Create objects for each protocol
IRCoolixAC coolixAC(kIrLedPin);
IRGoodweatherAc goodweatherAC(kIrLedPin); // Goodweather AC object
IRMitsubishiAC mitsubishiAC(kIrLedPin);
IRLgAc lgAC(kIrLedPin);
IRTcl112Ac tcl112AC(kIrLedPin);
IRMirageAc mirageAC(kIrLedPin);
// Add additional objects as needed

// MQTT client
WiFiClient espClient;
PubSubClient client(espClient);

// WiFiManager Object
WiFiManager wifiManager;


// Timing Variables
unsigned long buttonPressedTime = 0;
unsigned long longPressTime = 2000;  // 2 seconds
bool isButtonPressed = false;

// MQTT settings
const char* mqtt_broker = "broker2.dma-bd.com";
const int mqtt_port = 1883;
const char* mqtt_user = "broker2";
const char* mqtt_pass = "Secret!@#$1234";



struct state {
  uint8_t temperature = 24;
  uint8_t fan = 0;
  uint8_t operation = 0;
  uint8_t choose_protocol = 0;
  bool powerStatus;
};

state acState;

// All Functions.......................
void blinkLED();
void connectToWiFi();
void startWiFiConfigMode();
void reconnectMQTT();
void cllback(char* topic, byte* payload, unsigned int length);
void publishState();
void controlAC();

void handleButtonPress();

/*********************************************************************/
// Define whether to use FastLED (1) or RGB LED pins (0)
#define USE_FASTLED 0

#if USE_FASTLED
// FastLED control
void GreenBlink() {
  leds[0] = CRGB::Green; // green blink
  FastLED.show();
  delay(120);
  leds[0] = CRGB::Black;
  FastLED.show();
  delay(120);
  leds[0] = CRGB::Green;
  FastLED.show();
  delay(120);
  leds[0] = CRGB::Black;
}

void BlueBlink() {
  leds[0] = CRGB::Blue; // blue blink
  FastLED.show();
  delay(120);
  leds[0] = CRGB::Black;
  FastLED.show();
  delay(120);
  leds[0] = CRGB::Blue;
  FastLED.show();
  delay(120);
  leds[0] = CRGB::Black;
  FastLED.show();
}

void RedLed() {
  leds[0] = CRGB::Red;
  FastLED.show();
}

void GreenLed() {
  leds[0] = CRGB::Green;
  FastLED.show();
}

void BlueLed() {
  leds[0] = CRGB::Blue;
  FastLED.show();
}

void BlackLed() {
  leds[0] = CRGB::Black;
  FastLED.show();
}

#else
// RGB LED control via pins
#define RED_LED_PIN    14
#define GREEN_LED_PIN  27
#define BLUE_LED_PIN   26

void GreenBlink() {
  digitalWrite(GREEN_LED_PIN, HIGH);    
  delay(120);
  digitalWrite(GREEN_LED_PIN, LOW);
  delay(120);
  digitalWrite(GREEN_LED_PIN, HIGH);
  delay(120);
  digitalWrite(GREEN_LED_PIN, LOW);
}

void BlueBlink() {
  digitalWrite(BLUE_LED_PIN, HIGH);
  delay(120);
  digitalWrite(BLUE_LED_PIN, LOW);
  delay(120);
  digitalWrite(BLUE_LED_PIN, HIGH);
  delay(120);
  digitalWrite(BLUE_LED_PIN, LOW);
}

void RedLed() {
  digitalWrite(RED_LED_PIN, HIGH);
}

void GreenLed() {
  digitalWrite(GREEN_LED_PIN, HIGH);
}

void BlueLed() {
  digitalWrite(BLUE_LED_PIN, HIGH);
}

void BlackLed() {
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);
}
#endif


/*********************************************************************/


// Function to calculate the CRC for the SHT3x data (polynomial 0x31)
uint8_t calculateCRC(uint8_t data[], uint8_t length) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t bit = 8; bit > 0; --bit) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31;
      } else {
        crc = (crc << 1);
      }
    }
  }
  return crc;
}



// Handle button press for Wi-Fi reset
void handleButtonPress() {
    if (digitalRead(RESET_BUTTON_PIN) == LOW) {
    if (!isButtonPressed) {
      buttonPressedTime = millis();
      isButtonPressed = true;
    }

    // If button is held for more than 5 seconds, start WiFiManager AP
    if (isButtonPressed && (millis() - buttonPressedTime >= longPressTime)) {
      Serial.println("Long press detected. Entering WiFi configuration mode...");
      startWiFiConfigMode();
      isButtonPressed = false;  // Reset button press state
    }
  } else {
    isButtonPressed = false;
  }
}





/********************************************************************************* */
    //     ******    WiFi  --- MQTT  ----   publish  ----   controlAC    ******
/********************************************************************************* */
// Connect to Wi-Fi using saved credentials
void connectToWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Attempting to connect to Wi-Fi...");
    WiFi.begin();
    int counter = 0;
    while (WiFi.status() != WL_CONNECTED && counter < 30) {  // Wait for max 15 seconds
      RedLed();
      delay(250);
      Serial.print(".");
      BlackLed();
      delay(250);
      counter++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nConnected to Wi-Fi!");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("\nFailed to connect to Wi-Fi.");
    }
  }
}
// Function to start Wi-Fi configuration mode (AP mode)
void startWiFiConfigMode() {
  digitalWrite(LED_PIN, LOW);  // Turn off LED during configuration mode
  wifiManager.resetSettings(); // Reset saved credentials (optional)
  
  // Enter AP mode with default settings or custom parameters
  if (!wifiManager.startConfigPortal("DMA IR Blaster")) {           ///////////////////////-----AP_Name----//////////////////////////
    Serial.println("Failed to enter configuration mode.");
  } else {
    Serial.println("Configuration mode started.");
  }

  // After config, reconnect to the Wi-Fi network
  connectToWiFi();
}


// Reconnect to the MQTT broker
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }
    else if (client.connect("ESP32Client", mqtt_broker, mqtt_pass)) {
      Serial.println("Connected to MQTT broker");
      client.subscribe(CONTROL_TOPIC);
    } else {
      Serial.print("  Failed, rc=");
      Serial.println(client.state());
      delay(5000); // Wait 5 seconds before retrying
        RedLed();delay(500);RedLed();delay(500);RedLed();delay(500);RedLed();delay(500);RedLed();delay(500);RedLed();delay(500);RedLed();delay(500);RedLed();delay(500);
    }
  }

}

// MQTT message callback handler
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("Message received: ");
  Serial.println(message);

  // Split the message by spaces - comma
  int firstSpace = message.indexOf(',');
  int secondSpace = message.indexOf(',', firstSpace + 1);
  int thirdSpace = message.indexOf(',', secondSpace + 1);
  int fourthSpace = message.indexOf(',', thirdSpace + 1);

  // Parse power command (on/off)
  String powerCmd = message.substring(0, firstSpace);
  acState.powerStatus = (powerCmd == "on") ? true : false;

  // Parse temperature
  String tempCmd = message.substring(firstSpace + 1, secondSpace);
  acState.temperature = tempCmd.toInt();

  // Parse mode (cool/heat/fan/auto)
  String modeCmd = message.substring(secondSpace + 1, thirdSpace);
  if (modeCmd == "cool") {
    acState.operation = 1;
  } else if (modeCmd == "heat") {
    acState.operation = 3;
  } else if (modeCmd == "fan") {
    acState.operation = 4;
  } else if (modeCmd == "auto") {
    acState.operation = 0;
  }

  // Parse fan speed (auto/min/med/max)
  String fanCmd = message.substring(thirdSpace + 1);
  if (fanCmd == "auto") {
    acState.fan = 0;
  } else if (fanCmd == "min") {
    acState.fan = 1;
  } else if (fanCmd == "med") {
    acState.fan = 2;
  } else if (fanCmd == "max") {
    acState.fan = 3;
  }

    // Parse fan speed (auto/min/med/max)
  String protocolCmd = message.substring(fourthSpace + 1);
  if (protocolCmd == "coolix") {
    acState.choose_protocol = 0;
  } else if (protocolCmd == "goodweather") {
    acState.choose_protocol = 1;
  } else if (protocolCmd == "mitsubishi") {
    acState.choose_protocol = 2;
  } else if (protocolCmd == "lg") {
    acState.choose_protocol = 3;
  } else if (protocolCmd == "tcl112") {
    acState.choose_protocol = 4;
  } else if (protocolCmd == "mirage") {
    acState.choose_protocol = 5;
  }/* else if (protocolCmd == "protocol6") {
    acState.choose_protocol = 6;
  } else if (protocolCmd == "protocol7") {
    acState.choose_protocol = 7;
  } else if (protocolCmd == "protocol8") {
    acState.choose_protocol = 8;
  } else if (protocolCmd == "protocol9") {
    acState.choose_protocol = 9;
  }*/

  controlAC();  // Control the AC based on the parsed commands
}





/////////////////////////////////////////////////////////////////////////////////////
/**********************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////
void controlAC() {

  // 0. coolix Protocol
  if (acState.choose_protocol == 0) {
    if (acState.powerStatus) {
      coolixAC.on();
      
      coolixAC.setTemp(acState.temperature);
      switch (acState.operation) {
        case 0: coolixAC.setMode(kCoolixAuto); break;
        case 1: coolixAC.setMode(kCoolixCool); break;
        case 3: coolixAC.setMode(kCoolixHeat); break;
        case 4: coolixAC.setMode(kCoolixFan); break;
      }
      switch (acState.fan) {
        case 0: coolixAC.setFan(kCoolixFanAuto); break;
        case 1: coolixAC.setFan(kCoolixFanMin); break;
        case 2: coolixAC.setFan(kCoolixFanMed); break;
        case 3: coolixAC.setFan(kCoolixFanMax); break;
      }
    } else {
      coolixAC.off();
    }
    coolixAC.send();
    publishState();
  }
  // 1. goodweather Protocol
  if (acState.choose_protocol == 1) {
    if (acState.powerStatus) {
      goodweatherAC.on();
      
      goodweatherAC.setTemp(acState.temperature);
      switch (acState.operation) {
        case 0: goodweatherAC.setMode(kGoodweatherAuto); break;
        case 1: goodweatherAC.setMode(kGoodweatherCool); break;
        case 3: goodweatherAC.setMode(kGoodweatherHeat); break;
        case 4: goodweatherAC.setMode(kGoodweatherFan); break;
      }
      switch (acState.fan) {
        case 0: goodweatherAC.setFan(kGoodweatherFanAuto); break;
        case 1: goodweatherAC.setFan(kGoodweatherFanLow); break;
        case 2: goodweatherAC.setFan(kGoodweatherFanMed); break;
        case 3: goodweatherAC.setFan(kGoodweatherFanHigh); break;
      }
    } else {
      goodweatherAC.off();
    }
    goodweatherAC.send();
    publishState();
  }

  // 2. Mitsubishi Protocol
  else if (acState.choose_protocol == 2) {
    if (acState.powerStatus) {
      mitsubishiAC.on();
      mitsubishiAC.setTemp(acState.temperature);
      switch (acState.operation) {
        case 0: mitsubishiAC.setMode(kMitsubishiAcAuto); break;
        case 1: mitsubishiAC.setMode(kMitsubishiAcCool); break;
        case 3: mitsubishiAC.setMode(kMitsubishiAcHeat); break;
        case 4: mitsubishiAC.setMode(kMitsubishiAcFan); break;
      }
      switch (acState.fan) {
        case 0: mitsubishiAC.setFan(kMitsubishiAcFanAuto); break;
        case 1: mitsubishiAC.setFan(kMitsubishiAcFanQuiet); break;
        case 2: mitsubishiAC.setFan(kMitsubishiAcFanAuto); break;
        case 3: mitsubishiAC.setFan(kMitsubishiAcFanMax); break;
      }
    } else {
      mitsubishiAC.off();
    }
    mitsubishiAC.send();
    publishState();
  }
  // 3. LG Protocol
  else if (acState.choose_protocol == 3) {
    if (acState.powerStatus) {
      lgAC.on();
      lgAC.setTemp(acState.temperature);
      switch (acState.operation) {
        case 0: lgAC.setMode(kLgAcAuto); break;
        case 1: lgAC.setMode(kLgAcCool); break;
        case 3: lgAC.setMode(kLgAcHeat); break;
        case 4: lgAC.setMode(kLgAcFan); break;
      }
      switch (acState.fan) {
        case 0: lgAC.setFan(kLgAcFanAuto); break;
        case 1: lgAC.setFan(kLgAcFanLow); break;
        case 2: lgAC.setFan(kLgAcFanMedium); break;
        case 3: lgAC.setFan(kLgAcFanMax); break;
      }
    } else {
      lgAC.off();
    }
    lgAC.send();
    publishState();
  }
  // 4. tcl112 Protocol
  else if (acState.choose_protocol == 4) {
    if (acState.powerStatus) {
      tcl112AC.on();
      tcl112AC.setTemp(acState.temperature);
      switch (acState.operation) {
        case 0: tcl112AC.setMode(kTcl112AcAuto); break;
        case 1: tcl112AC.setMode(kTcl112AcCool); break;
        case 3: tcl112AC.setMode(kTcl112AcHeat); break;
        case 4: tcl112AC.setMode(kTcl112AcFan); break;
      }
      switch (acState.fan) {
        case 0: tcl112AC.setFan(kTcl112AcFanAuto); break;
        case 1: tcl112AC.setFan(kTcl112AcFanLow); break;
        case 2: tcl112AC.setFan(kTcl112AcFanMed); break;
        case 3: tcl112AC.setFan(kTcl112AcFanHigh); break;
      }
    } else {
      tcl112AC.off();
    }
    tcl112AC.send();
    publishState();
  }
  // 5. Mirage Protocol
  else if (acState.choose_protocol == 5) {
    if (acState.powerStatus) {
      mirageAC.on();
      mirageAC.setTemp(acState.temperature);
      switch (acState.operation) {
        case 0: mirageAC.setMode(kMirageAcCool); break;
        case 1: mirageAC.setMode(kMirageAcCool); break;
        case 3: mirageAC.setMode(kMirageAcHeat); break;
        case 4: mirageAC.setMode(kMirageAcFan); break;
      }
      switch (acState.fan) {
        case 0: mirageAC.setFan(kMirageAcFanAuto); break;
        case 1: mirageAC.setFan(kMirageAcFanLow); break;
        case 2: mirageAC.setFan(kMirageAcFanMed); break;
        case 3: mirageAC.setFan(kMirageAcFanHigh); break;
      }
    } else {
      mirageAC.off();
    }
    mirageAC.send();
    publishState();
  }


/////////////////////////////////////////////////////////////////////////////////////
/**********************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////


/*
  
  // 6. Panasonic Protocol
  else if (acState.choose_protocol == 6) {
    if (acState.powerStatus) {
      panasonicAC.on();
      panasonicAC.setTemp(acState.temperature);
      switch (acState.operation) {
        case 0: panasonicAC.setMode(kPanasonicAuto); break;
        case 1: panasonicAC.setMode(kPanasonicCool); break;
        case 3: panasonicAC.setMode(kPanasonicHeat); break;
        case 4: panasonicAC.setMode(kPanasonicFan); break;
      }
      switch (acState.fan) {
        case 0: panasonicAC.setFan(kPanasonicFanAuto); break;
        case 1: panasonicAC.setFan(kPanasonicFanLow); break;
        case 2: panasonicAC.setFan(kPanasonicFanMed); break;
        case 3: panasonicAC.setFan(kPanasonicFanHigh); break;
      }
    } else {
      panasonicAC.off();
    }
    panasonicAC.send();
    publishState();
  }*/

  // Continue for the remaining protocols following the same structure:

  // 7. Toshiba Protocol
  // 8. Sharp Protocol
  // 9. Hitachi Protocol
  // 10. Haier Protocol
  // 11. Midea Protocol
  // 12. Gree Protocol
  // 13. Whirlpool Protocol
  // 14. Electra Protocol
  // 15. Sanyo Protocol
  // 16. Trane Protocol
  // 17. Airwell Protocol
  // 18. Daewoo Protocol
  // 19. Delonghi Protocol
  // 20. Hisense Protocol
  // 21. Carrier Protocol
  // 22. TCL Protocol
  // 23. Siemens Protocol
  // 24. Voltas Protocol
  // 25. NEC Protocol
  // 26. JVC Protocol
  // 27. Mirage Protocol
  // 28. AUX Protocol
  // 29. Onida Protocol
  // 30. Kenwood Protocol
}










  
void publishState() {
  String powerCmd = acState.powerStatus ? "on" : "off";
  String modeCmd;
  String fanCmd;

  switch (acState.operation) {
    case 0: modeCmd = "auto"; break;
    case 1: modeCmd = "cool"; break;
    case 3: modeCmd = "heat"; break;
    case 4: modeCmd = "fan"; break;
  }
  switch (acState.fan) {
    case 0: fanCmd = "auto"; break;
    case 1: fanCmd = "min"; break;
    case 2: fanCmd = "med"; break;
    case 3: fanCmd = "max"; break;
  }

  String message = powerCmd + " " + String(acState.temperature) + " " + modeCmd + " " + fanCmd;
  client.publish(STATUS_TOPIC, message.c_str());
  GreenBlink();
}



/*************************************  ---  End  --  ************************************ */
    //     ******    WiFi  --- MQTT  ----   publish  ----   controlAC    ******
/********************************************************************************* */






/************************************************************************************* */
/**********----  -----   -------   -----   main    ------   -------   -------------***** */
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(50); // Wait for serial connection
  Wire.begin();  // Initialize I2C
  Serial.println("--sabbir--");
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);
  delay(255);

  pinMode(LED_PIN, OUTPUT);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

  irrecv.setTolerance(kTolerancePercentage);
  irrecv.enableIRIn();  // Start the IR receiver

//     ------------------------    wifi reset    ---------------------------
  unsigned long startTime = millis();  // Store the starting time
  startTime = millis();  // Initialize startTime
  Serial.println("You can reset Wi-Fi");
  while (millis() - startTime <= 7000) {  // 7 second without delay
    handleButtonPress();  // Correctly call the function
  }
  Serial.println("End Wi-Fi reset");
//     ------------------------ end   wifi reset    ---------------------------

  connectToWiFi();     // Attempt to connect to saved Wi-Fi credentials

  coolixAC.begin();
  goodweatherAC.begin();
  mitsubishiAC.begin();
  lgAC.begin();
  tcl112AC.begin();
  irsend.begin();


 //connecting to a mqtt broker
 client.setServer(mqtt_broker, mqtt_port);
 client.setCallback(callback);
 reconnectMQTT();

  // Initialize Sensor 1 at address 0x44
  if (!AirFlow.begin(0x44)) {
    Serial.println("Couldn't find Airflow Sensor");
    while (1) delay(1);
  }
  if (!Ambient.begin(0x45)) {
    Serial.println("Couldn't find Ambient Sensor");
    while (1) delay(1);
  }
  
  // Initialize Sensor 2 at address 0x45
  /*if (!sht31_2.begin(0x45)) {
    Serial.println("Couldn't find SHT31 at sensor-2");
    while (1) delay(1);
  }*/
}
//************************************************************************** */
//*************************************************************************** */
void loop() {
  
  if (WiFi.status() != WL_CONNECTED) {// If Wi-Fi is not connected, attempt to reconnect
    connectToWiFi();
    RedLed();
  }
  if (!client.connected()) {
    reconnectMQTT();
    RedLed();
  }
  if (WiFi.status() == WL_CONNECTED && client.connected()) {BlackLed();}


  // Read temperature and humidity using the function
  if (count == 8400) {

    // Reading Sensor 1 (address 0x44)
    float temperature_1 = AirFlow.readTemperature();
    float humidity_1 = AirFlow.readHumidity();
    float temperature_2 = Ambient.readTemperature();
    float humidity_2 = Ambient.readHumidity();
    
    if (!isnan(temperature_1) && !isnan(humidity_1)) {
      const char* airflow = String("Temperature: " + String(temperature_1) + ", Humidity: " + String(humidity_1)).c_str();
      if (WiFi.status() == WL_CONNECTED || client.connected()) {
        client.publish(AIRFLOW_SENSOR_TOPIC, airflow);
      } else {RedLed();delay(500);BlackLed();}
    } else {
      Serial.println("Failed to read from AirFlow");
      client.publish(AIRFLOW_SENSOR_TOPIC, "Error");
    }
    if (!isnan(temperature_2) && !isnan(humidity_2)) {
      const char* ambient = String("Temperature: " + String(temperature_2) + ", Humidity: " + String(humidity_2)).c_str();
      if (WiFi.status() == WL_CONNECTED || client.connected()) {
        client.publish(AMBIENT_SENSOR_TOPIC, ambient);
      } else {RedLed();delay(500);BlackLed();}
    } else {
      Serial.println("Failed to read from Ambient");
      client.publish(AMBIENT_SENSOR_TOPIC, "Error");
    } GreenBlink(); count = 0;
  }
 


  if (irrecv.decode(&results)) {
    BlueBlink();
    // Print the decoded data in human-readable form
    const char* mycode = (resultToHumanReadableBasic(&results)).c_str();
    Serial.println(mycode);
    client.publish(REMOTE_TOPIC, mycode);
    GreenBlink();
    // Resume the IR receiver to capture the next signal
    irrecv.resume();
  }

  client.loop(); // Handle incoming MQTT messages
  delay(1); // Publish data every .1 seconds
  count = count + 1;
}

/**********----  -----   -------   ----- end  main    ------   -------   -------------***** */
/************************************************************************************* */
