//energ
char ssid[] = "ssdid";
char pass[] = "pw";

const char MQTT_BROKER[] = "10.10.10.10";
const int MQTT_PORT = 1883;
#if PROD == 1
  const char MQTT_CLIENT_ID[] = "esp32voc01";
#else
    const char MQTT_CLIENT_ID[] = "esp32vocTEST";
#endif  
const char MQTT_USERNAME[] = "user";                 
const char MQTT_PASSWORD[] = "pw";

const char* otaUsr = "user";
const char* otaPW = "pw";