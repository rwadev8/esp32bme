// get temp from MAX6675 for buffer and warm water tanks
// 1.0.3 filter bad values, deal with inital value problem
// 1.0.4 add buf2
// 1.1.0 ota update feature
// 1.2.0 add on board led color info, solder first!!, add wifi reconnect and check mqtt publish error
// 2.0.0a copy of the esp32_temp to test bme688

#define PROD 0 //  REMEMBER to change to 1 for prod deploy

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_BME680.h>
#include "Secret.h"
#include "esp32-hal-cpu.h"
#include "time.h"

const char VERSION[] = "v2.0.0a";

Adafruit_BME680 bme;
#define I2C_SDA 21
#define I2C_SCL 8  // instead of 22

WiFiClient network;
// Set web server port number to 80
WebServer server(80);

MQTTClient mqtt = MQTTClient(256);
int mqttPubInt = 30 * 1000;

#if PROD == 1
  const char mqttTopicBufferTemp[] = "esp32/voc/data"; 
#else
  const char mqttTopicBufferTemp[] = "esp32/voc/TEST"; 
#endif

unsigned long mqttPublishTime = 0;
struct tm mqttTimeInfo;
char mqttLastPublishDate[40];
int cntMReCon = 0;
int cntMDisCon = 0;
int cntMPub = 0;
int cntMPubErr = 0;
int cntWifiReConn = 0;
int cntBadBme = 0;
int cntBme = 0;
int cntConsBadBme = 0;

const int initTemp = 15.0;

String header;
#if PROD == 0
  float bmeTemp = 22.0;
  float bmeHum = 50;
  float bmePres = 1000;
#else
  float bmeTemp = 1;
  float bmeHum = 30;
  float bmePres = 900;
#endif
float deltaBuf5 = 0.0;
float deltaBuf2 = 0.0;
float deltaWW = 0.0;

float rawBmeTemp, rawBmeHum, rawBmePres;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

String bootTimeStr;  // Save formatted boot time string

// task handles
TaskHandle_t TReadBME;
TaskHandle_t TPostMqtt;
TaskHandle_t TGetNtpTime;
TaskHandle_t TMonLED;
TaskHandle_t TWifiCheckReconn;
TaskHandle_t TWatchDog;

// ntp
const long  gmtOffset_sec = 3600;
const char* ntpServer = "de.pool.ntp.org"; 
const int   daylightOffset_sec = 3600;

// update
unsigned long lastFailedUpdate = 0;
const unsigned long updateDelay = 5000; // 10 seconds in milliseconds

// HTML page for OTA update
const char* updatePage = R"rawliteral(
<!DOCTYPE html>
<html>
<head><title>ESP32  voc  OTA Update</title></head>
<body>
  <h1>ESP32 OTA Update</h1>
  <form method="POST" action="/update" enctype="multipart/form-data">
    <input type="file" name="firmware">
    <input type="submit" value="Upload Firmware">
  </form>
</body>
</html>
)rawliteral";

// led tests
// Define the pin where the built-in RGB LED is connected
#define LED_PIN 48
// Define the number of LEDs in the strip (usually 1 for built-in LED)
#define NUM_LEDS 1
Adafruit_NeoPixel led(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
struct rgbColor { int r; int g; int b;};
volatile rgbColor ledColor = {0,0,0};


// helper functions

// round to digits, standard round() returns integer here
float FloatRound(float value, int digits) {
  return round(value * 10.0 * digits) / 10.0 * digits;
}

// caluclate temp delta, deal with 0 C issue, suggested by ChatGPT
float CalcDelta(float curTemp, float temp, float eps) {
  float deltaPerc = 0;
  const float maxPercDelta = 50.0; // max allowed change bound

  if(abs(temp) < eps) {
    deltaPerc = (curTemp - temp) / eps;
    deltaPerc = fmin(fmax(deltaPerc * 100, -maxPercDelta), maxPercDelta);
  } else {
    deltaPerc = (curTemp - temp) / temp;
  }
  return deltaPerc;
}

// tasks
void ReadBME(void * parameter) {
  //int curBuf5, curBuf2, curWW; // made global to also display the raw value saw values 800 at one point in time, filter those out
  // at startup wait a few seconds to allow connections to stavilize
  delay(3000);

  for(;;) {
    const int tMin = 15;
    const int tMax = 100;
    const int tDelta = 7;
    const int cntConsCorr = 10;
    const int corrPerc = 2;
    const float eps = 0.01;
    float delta;

    delay(100);
    if (bme.performReading()) {
      rawBmeTemp = bme.temperature;
      rawBmeHum = bme.humidity;
      rawBmePres = bme.pressure / 100.0;

      // Ignore bme.gas_resistance if not needed
    } else {
      Serial.println("BME688 read failed");
      cntBadBme++;
    }
  
    // for now use raw values without adjustement
    bmeTemp = rawBmeTemp;
    bmeHum = rawBmeHum;
    bmePres = rawBmePres;
    
#if PROD == 0
    bmeTemp++;
#endif

  vTaskDelay(pdMS_TO_TICKS(2000)); // 2-second delay

  } // for
}

// ============  task loops =============================
     
void PostMQTT(void * parameter) {
  // at startup wait a few seconds to allow connections to stablize
  delay(5000);

  for(;;) {
    if(!mqtt.connected()) {
      //TODO a reconnect would be really helpful here
      Serial.print("m");
      delay(1000);
      connectMQTT();
      cntMReCon++;
    }
    mqtt.loop(); // this i supposed to keep the connection alive according to chatcpt

    if(millis() - mqttPublishTime > mqttPubInt) {
      sendMQTT();
      mqttPublishTime = millis();
    }
    delay(250);
  } // for
}

void MonLED(void * parameter) {
  // at startup wait a few seconds to allow connections to stablize
  //delay(1000);
  led.setBrightness(16); // set global brightness 255 is 100 %
  led.clear();
  led.show();

  for(;;) {
    led.setPixelColor(0, led.Color(ledColor.r, ledColor.g, ledColor.b));
    led.show(); // Turn off LED initially
    delay(50);
  } // for
}

void GetNtpTime(void * parameter) {
  // seems the ntp client does not need to run in a loop
  for(;;) {
     vTaskDelay(pdMS_TO_TICKS(10*60*1000)); 
  }
}

void WifiCheckReconn(void * parameter) {
  // at startup wait a few seconds to allow connections to stablize
  delay(1000);

  for(;;) {
    ledColor.r = 255; ledColor.g = 100;
    delay(500);
    ledColor.r = 0; ledColor.g = 0;

    if (WiFi.status() != WL_CONNECTED) {
        ledColor.r = 255;
        cntWifiReConn++;
        Serial.printf("wifi NOT connected, %d\n", cntWifiReConn);
        WiFi.reconnect();     
        delay(2000);
        if (WiFi.status() == WL_CONNECTED) {
          Serial.printf("wifi reconn ok\n");
          ledColor.r = 0;
        }
        else {
          Serial.printf("wifi reconn FAILED\n");
        }
    }  
    vTaskDelay(pdMS_TO_TICKS(45*1000));
  } // for
}

void WatchDog(void * parameter) {
  // at startup wait a few seconds to allow connections to stablize
  delay(5000);

  for(;;) {
    ledColor.b = 255;
    delay(500);
    ledColor.b = 0;

    // this works fine
    //led.setPixelColor(0, led.Color(0, 0, 255));  // Set blue
    //led.show();

    vTaskDelay(pdMS_TO_TICKS(60*1000));
  } // for
}


//  -----------------------------

void connectMQTT() {
  // Connect to the MQTT broker
  //mqtt.begin(MQTT_BROKER, MQTT_PORT, network); // move to setup to be able to use this call for reconnects

  Serial.printf("Connecting to MQTT broker %s:%d\n", MQTT_BROKER, MQTT_PORT);
  
  while (!mqtt.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
    ledColor.g = 255; ledColor.b = 255;
    Serial.print("!m");
    delay(250);
    ledColor.g = 0;  ledColor.b = 0;
    delay(500);
    cntMDisCon++;
  }

  if (!mqtt.connected()) {
      Serial.println("MQTT Connection failed");
      return;
  }
  cntMReCon++;
}

void sendMQTT() {
  bool published;
  StaticJsonDocument<200> message;

  message["timestamp"] = millis();
  message["bmeTemp"] = bmeTemp;
  message["bmeHum"] = bmeHum;
  message["bmePres"] = bmePres;
  char messageBuffer[512];
  serializeJson(message, messageBuffer);

  getLocalTime(&mqttTimeInfo);
  strftime(mqttLastPublishDate, sizeof(mqttLastPublishDate), "%Y-%m-%d %H:%M:%S", &mqttTimeInfo);
 
  // starting to supect that the retCode is not meaningful in this case, the original code did not have it
  published = mqtt.publish(mqttTopicBufferTemp, messageBuffer); //, false, 1); // no retain, qos 0, without them getting retCode 1 even if data arrive in HA, qos 1 still responds with 1
  if (published) {
    ledColor.g = 255;
    Serial.printf("%s  sent MQTT, topic: %s, payload: %s\n", mqttLastPublishDate, mqttTopicBufferTemp, messageBuffer);
    cntMPub++;
    delay(200);
    ledColor.g = 0;
  }
  else {
    ledColor.g = 255;   ledColor.b = 255;
    Serial.printf("%s  ERROR sending MQTT, topic: %s, payload: %s\n", mqttLastPublishDate, mqttTopicBufferTemp, messageBuffer);
    cntMPubErr++;
    delay(500);
    ledColor.g = 0;   ledColor.b = 0;
  }
}

// ==================== html server pages, thanks ChatGPT, well, besides the test or testLED fiasko
void handleRoot() {
  IPAddress ip = WiFi.localIP();
  int tempInt = temperatureRead();
  unsigned long uptime = millis() / 1000;
  struct tm timeinfo;
  char timeString[50];
  int mqttConnected = 0;

  getLocalTime(&timeinfo);
  strftime(timeString, sizeof(timeString), "%A, %B %d %Y %H:%M:%S", &timeinfo);
  if(mqtt.connected()) {
    mqttConnected = 1;
  } else {
    mqttConnected = 0;
    cntMDisCon++;
  }

  String html = "<!DOCTYPE html><html>";
  html += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  html += "<link rel=\"icon\" href=\"data:,\">";
  html += "<title>ESP32 voc</title></head><body>";
  html += "<style>table { border-collapse: collapse; width: 30%; text-align: center; } th, td { border: 1px solid black; padding: 3px; }  </style>";

#if PROD == 1  
  html += "<h1>Prod VOC</h1>";
#else
  html += "<h1>Test VOC</h1>";
#endif  
  html += String("<p>time: ") + timeString + String(", IP: ") + ip.toString() + String(", RSSI: ") + WiFi.RSSI() + "</p>";
  html += String("<p>cpu Frequency: ") + getCpuFrequencyMhz() + String(" MHz, Core: ") + xPortGetCoreID() +
          String(", Internal Temp: ") + tempInt + String(" C</p>");
  html += String("<p>uptime: ") + uptime + String(" seconds <br>  boot at: " + bootTimeStr + "</p>");
  html += String("<p><ul><li>bmeTem: ") + String(bmeTemp) + " C</li><li>bmeHum: " + String(bmeHum) + " %</li><li>bmePres: " + String(bmePres) + " hPa</li>";
  html += String("<li>bad val counts: bme: ") + String(cntBadBme) + "</li></ul></p>";
  //html += "<p><table><colgroup><col style=\"width: 12%;\"><col style=\"width: 20%;\"><col style=\"width: 20%;\"><col style=\"width: 20%;\"></colgroup>";
  //html += "<tr><th>desc</th><th>temp C</th><th>delta %</th><th>raw C</th><th>bad vals</th><th>consect bad</th></tr>";
  //html += String("<tr><td>buf5</td><td>") + tempBuf5  + String("</td><td>") + FloatRound(deltaBuf5*100, 1)  + String("</td><td>") + rawBuf5 + String("</td><td>") + cntBadBuf5 + String("</td><td>") + cntConsBadBuf5 + String("</td></tr>");
  //html += String("<tr><td>buf2</td><td>") + tempBuf2  + String("</td><td>") + FloatRound(deltaBuf2*100, 1)  + String("</td><td>") + rawBuf2 + String("</td><td>") + cntBadBuf2 + String("</td><td>") + cntConsBadBuf2 + String("</td></tr>");
  //html += String("<tr><td>ww</td><td>") + tempWW  + String("</td><td>") + FloatRound(deltaWW*100, 1)  + String("</td><td>") + rawWW + String("</td><td>") + cntBadWW + String("</td><td>") + cntConsBadWW + String("</td></tr>");
  //html += "</table></p>";
  html += String("<p>mqtt broker: ") + MQTT_BROKER + ", client: " + MQTT_CLIENT_ID + ", topic: " + mqttTopicBufferTemp  +
          "<br>Last Published: " + mqttLastPublishDate + ", connected: " + mqttConnected + "</p>";
  html += String("<p><ul><li>mqtt pubs: ") + cntMPub + "</li><li>mqtt errors: " + cntMPubErr + "</li><li>mqtt reconnects: " + cntMReCon + 
          "</li><li>mqtt disconnects: " + cntMDisCon + "</li><li>WiFi reconnects: " + cntWifiReConn + "</li></ul></p>";
  html += "<p><a href=\"/info\">info</a> <a href=\"/ota\">ota</a> </p>";        
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleInfo() {
  String html = "<!DOCTYPE html><html>";
  html += "<head><title>ESP32 voc info</title></head><body>";
  html += "<h1>hw info</h1> <ul> <li>esp32s3wromm dev board</li> <li>bme688</li> </ul>";
  html += "<h1>sw info</h1> <ul> <li>arduino ide</li> <li>espressif 3.3.1alpha</li> </ul>";
  html += "<p> to build update, in arduino, Sketch, Export Compiled Binary, upload the esp32_temp_http_mqtt_ntp.ino.bin file";
  html += String("<p>") + VERSION + "</p>";
  html += "<p><a href=\"/\">Back to Home</a></p>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleTestLED() {
  Serial.println("LED test triggered via REST API");
  server.send(200, "text/plain", "LED test starting");
  testLED(300, 150);
}

void handleTest() {
  Serial.println("test url called");
  String html = "<!DOCTYPE html><html>";
  html += "<head><title>test</title></head><body>";
  html += "<h1>test/h1> <p>test</p>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}


// Function to check authentication with delay
bool checkAuthentication() {
  unsigned long currentTime = millis();
  if (currentTime - lastFailedUpdate < updateDelay) {
    server.send(429, "text/html", "<h1>Too many failed attempts. Try again later.</h1>");
    return false;
  }
  
  if (!server.authenticate(otaUsr, otaPW)) {
    lastFailedUpdate = millis();
    server.requestAuthentication();
    return false;
  }
  
  return true;
}

// Serve OTA update page
void handleOTAUpdatePage() {
  Serial.println("handleOTAUpdatePage()");
  //if (!checkAuthentication()) return;
  server.send(200, "text/html", updatePage);
}

// Handle firmware upload
void handleFirmwareUpload() {
  //Serial.println("handleFirmwareUpload()"); // gets called very often during upload
  HTTPUpload& upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("OTA Update: Start uploading: %s\n", upload.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { // Start with unknown size
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      Serial.printf("OTA Update: Success, %u bytes received, resetting\n", upload.totalSize);
      ESP.restart();
    } else {
      Update.printError(Serial);
    }
  }
}

void handleUpdate() {
  Serial.println("handleUpdate()");
  if (!checkAuthentication()) return;
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", "<h1>Update Success! Rebooting...</h1>");
  delay(1000);
  ESP.restart();
}

void testLED(int onTime, int pauseTime) {
  ledColor.r = 255;   delay(onTime);   ledColor.r = 0;   delay(pauseTime);
  ledColor.g = 255;   delay(onTime);   ledColor.g = 0;   delay(pauseTime);
  ledColor.b = 255;   delay(onTime);   ledColor.b = 0;   delay(pauseTime);

  ledColor.r = 255; ledColor.g = 255;   delay(onTime);   ledColor.r = 0; ledColor.g = 0;   delay(pauseTime); // yellow
  ledColor.r = 255; ledColor.g = 100;   delay(onTime);   ledColor.r = 0; ledColor.g = 0;   delay(pauseTime); // orange
  ledColor.r = 255; ledColor.g =  80; ledColor.b =  10;   delay(onTime);    ledColor.r = 0; ledColor.g = 0;  ledColor.b = 0;   delay(pauseTime); // redish orange
  ledColor.r =  64; ledColor.g = 224; ledColor.b = 208;   delay(onTime);    ledColor.r = 0; ledColor.g = 0;  ledColor.b = 0;   delay(pauseTime); // turquoise
}

/////////////////////////////////

void setup() {
  IPAddress ip;
  struct tm timeinfo;
  char upTimeBuf[32];
  
  Serial.begin(115200);
  Serial.printf("\n\nstarting ---------- %s -------------\n", VERSION);

  Serial.printf("\nbme setup\n");

  Wire.begin(I2C_SDA, I2C_SCL);
  if (!bme.begin(0x77)) {
    Serial.println("BME688 not found");
  } 
  else
  {
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setGasHeater(0, 0);  // Disable gas readings for now
  }

  // Connect to Wi-Fi network with SSID and password
  Serial.printf("connecting to ssid: %s\n", ssid);

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    ledColor.r = 255;
    delay(500);
    Serial.print("!w");
    cntWifiReConn++;
    ledColor.r = 0;
    delay(150);
  }
  // Print local IP address and start web server
  ledColor.r = 0;
  ip = WiFi.localIP();
  Serial.printf("\nWiFi connected to: %s, ip: %d.%d.%d.%d\n", ssid, ip[0], ip[1], ip[2], ip[3]);

  Serial.printf("testing rbg led...\n");

  xTaskCreatePinnedToCore(
      MonLED, "TaskMonLED", 10000,  NULL,  /* Task input parameter */
      0,  /* Priority of the task */  &TMonLED,  /* Task handle. */
      1); /* Core where the task should run */

  // test blink leds
  delay(500);
  testLED(500, 250);
  delay(1000);

  // start tasks
  xTaskCreatePinnedToCore(
      ReadBME, /* Function to implement the task */ "TaskBME", /* Name of the task */
      4096,  /* Stack size in words */ NULL,  /* Task input parameter */
      1,  /* Priority of the task */  &TReadBME,  /* Task handle. */
      0); /* Core where the task should run */

  xTaskCreatePinnedToCore(
      PostMQTT, "TaskMqtt", 10000,  NULL,  /* Task input parameter */
      0,  /* Priority of the task */  &TPostMqtt,  /* Task handle. */
      0); /* Core where the task should run */

  xTaskCreatePinnedToCore(
      WifiCheckReconn, "TaskWifiCheckReconn", 10000,  NULL,  /* Task input parameter */
      0,  /* Priority of the task */  &TWifiCheckReconn,  /* Task handle. */
      1); /* Core where the task should run */

  xTaskCreatePinnedToCore(
      WatchDog, "TaskWatchDog", 10000,  NULL,  /* Task input parameter */
      0,  /* Priority of the task */  &TWatchDog,  /* Task handle. */
      1); /* Core where the task should run */


  // setup ntp
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.printf("ntp server: %s, gmtOff: %i s\n", ntpServer, gmtOffset_sec);
  getLocalTime(&timeinfo);
  Serial.println();
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

  // connect to mqtt
  mqtt.begin(MQTT_BROKER, MQTT_PORT, network);
  connectMQTT();

  // start web server Define routes and start, next time you test, make sure run tests against the test board ip, NOT the prod one...
  server.on("/", handleRoot);
  server.on("/test", handleTest);
  server.on("/info", handleInfo);
  server.on("/testled", handleTestLED);
  server.on("/ota", handleOTAUpdatePage);
  server.on("/update", HTTP_POST, handleUpdate, handleFirmwareUpload);
  server.on("/led", HTTP_GET, []() {
    server.send(200, "text/plain", "LED test starting");
    testLED(300, 150);  
  });
  server.begin();
  Serial.printf("HTTP web server started\n");

  while (!getLocalTime(&timeinfo)) {
    Serial.println("Waiting for NTP time...");
    delay(500);
  }
  strftime(upTimeBuf, sizeof(upTimeBuf), "%Y-%m-%d %H:%M:%S", &timeinfo);
  bootTimeStr = String(upTimeBuf);

}

void loop() {
  //Serial.println("Waiting for HTTP requests...");
  server.handleClient(); // WebServer client connection handling
  //delay(1000);  // Prevents flooding the serial monitor
}