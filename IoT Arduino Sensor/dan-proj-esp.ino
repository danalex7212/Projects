
                  
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
// WiFi parameters
#define WLAN_SSID       "myhotspot"
#define WLAN_PASS       "mypassword"
// Adafruit IO
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "dan7212"
#define AIO_KEY         "aio_UzFE75jcIf8VAICkauALDOWh5AGg" 
WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish pulse = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pulse");

int val;
void setup() {
  Serial.begin(115200);
  
  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  delay(10);
  Serial.print(F("Connecting to "));
  Serial.println(WLAN_SSID);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());

  // connect to adafruit io
  connect();

}

// connect to adafruit io via MQTT
void connect() {
  Serial.print(F("Connecting to Adafruit IO... "));
  int8_t ret;
  while ((ret = mqtt.connect()) != 0) {
    switch (ret) {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }

    if(ret >= 0)
      mqtt.disconnect();

    Serial.println(F("Retrying connection..."));
    delay(10000);
  }
  Serial.println(F("Adafruit IO Connected!"));
}

void loop() {
  // ping adafruit io a few times to make sure we remain connected
  if(! mqtt.ping(3)) {
    // reconnect to adafruit io
    if(! mqtt.connected())
      connect();
  }
  //dht11.read(&temp, &hum, NULL);
  //String msg = Serial.readStringUntil('\r');
  String msg = Serial.readStringUntil('\r');
  val = msg.toInt();
  Serial.println(val);
  //Serial.println(msg.toInt());
  //Serial.print((int)temp); Serial.print(" *C, "); 
  //Serial.print((int)hum); Serial.println(" H");
  
   if(val<600)
   {
   if (! pulse.publish(val)) {                     //Publish to Adafruit
      Serial.println(F("Failed"));
    } 
       //if (! Humidity1.publish(hum)) {                     //Publish to Adafruit
      //Serial.println(F("Failed"));
   // }
    else {
      Serial.println(F("Sent!"));
    }}
    else
{
  if (! pulse.publish(0)) {                     //Publish to Adafruit
      Serial.println(F("Failed"));
    } 
       //if (! Humidity1.publish(hum)) {                     //Publish to Adafruit
      //Serial.println(F("Failed"));
   // }
    else {
      Serial.println(F("Sent!"));
    }
  }
    delay(2000);
}
