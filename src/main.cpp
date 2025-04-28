
#include <Arduino.h>
#include <Esp.h>
#include <esp_sleep.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_NeoPixel.h>

#include <PubSubClient.h>

#define PRINT_DEBUG 1

#if PRINT_DEBUG == 1
#define debugp(x) Serial.print(x)
#define debugpln(x, ...) Serial.println(x)
#define debugpf(x, ...) Serial.printf(x)
#else
#define debugp(x)
#define debugpln(x, ...)
#define debugpf(x, ...)
#endif

#define LED_PIN D3
#define BUTTON_PIN D2
#define FIVE_V_PIN D1
#define LED_COUNT 1
#define MQTT_BATT_VOLTAGE 0

#define uS_TO_S_FACTOR 1000000ULL // Conversion factor for microseconds to seconds
#define TIME_TO_SLEEP_uS 20       // Time ESP32 will go to sleep (in seconds)

#define mS_TO_S_FACTOR 1000ULL // Conversion factor for milliseconds to seconds
#define TIME_TO_SLEEP_mS 20    // Time ESP32 will go to sleep (in seconds)

#define mS_TO_M_FACTOR 60000ULL // Conversion factor for milliseconds to minute
#define SLEEP_DELAY_M 1         // Wait Time after button push for the ESP32 to go to sleep (in Minutes)

/*
   Configuration (HA) :
    sensor:
      platform: mqtt
      state_topic: 'bedroom/remote/battery'
      name: 'Battery Voltage'
      unit_of_measurement: 'v'
      value_template: '{{ value_json.batt }}'
*/

// MQTT: ID, server IP, port, username and password
const PROGMEM char *MQTT_CLIENT_ID = "remote_button1";
const PROGMEM char *MQTT_SERVER_IP = "192.168.15.92";
const PROGMEM uint16_t MQTT_SERVER_PORT = 1883;
const PROGMEM char *MQTT_USER = "mqtt";
const PROGMEM char *MQTT_PASSWORD = "mqtt";
// MQTT: topic
const PROGMEM char *MQTT_SENSOR_TOPIC = "bedroom/lamp/remote";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

int buttonState = 0;

Adafruit_NeoPixel pixel(LED_COUNT, LED_PIN, NEO_RGB + NEO_KHZ800);

/* Your WiFi Credentials */
const char *ssid = "ATT";       // SSID
const char *password = "pwd"; // Password

// Your Domain name with URL path or IP address with path
const char *serverName = "http://192.168.15.96/cm?cmnd=Power%20Toggle";

unsigned long button_time = 0;
unsigned long last_button_time = 0;
unsigned long sleep_time = 0;
unsigned long last_sleep_time = 0;

String httpGETRequest(const char *serverName);
void setupWiFi();
void WiFiEvent(WiFiEvent_t event);
void print_wakeup_reason();
void callback(char *p_topic, byte *p_payload, unsigned int p_length);
void checkBatteryVoltage();

void setup()
{
    Serial.begin(115200);
    delay(1000); // Take some time to open up the Serial Monitor

    setupWiFi();

    pinMode(A0, INPUT); // ADC
    // For detecting if USB is plugged in or not
    pinMode(FIVE_V_PIN, INPUT);

    // initialize the pushbutton pin as an input:
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // initialize the NeoPixel LED
    pixel.begin();
    pixel.show();
    pixel.setBrightness(50);

    // Print the wakeup reason for ESP32
    print_wakeup_reason();

// To test if the GPIO is a valid wakeup pin
    // if (esp_sleep_is_valid_wakeup_gpio(GPIO_NUM_2))
    // {
    //     debugpln("GPIO Number 2 is a valid wakeup pin.");
    // }
// Timer wakeup code
    // if (esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP_uS * uS_TO_S_FACTOR) == ESP_OK)
    // {
    //     debugpln("Starting Sleep Timer");
    // }
    // else
    // {
    //     debugpln("Error Starting Sleep Timer");
    // }

    if (esp_deep_sleep_enable_gpio_wakeup(BIT(BUTTON_PIN), ESP_GPIO_WAKEUP_GPIO_LOW) == ESP_OK) // GPIO2
    {
        debugpln("Starting Sleep Button");
    }
    else
    {
        debugpln("Error Starting Sleep Button");
    }

    // init the MQTT connection
    mqttClient.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
    mqttClient.setCallback(callback);
    checkBatteryVoltage();

} // void setup()

void loop()
{
    sleep_time = millis();

    buttonState = digitalRead(BUTTON_PIN);

    if (buttonState == HIGH)
    {
    }

    if (buttonState == LOW)
    {
        button_time = millis();
        String response = "";
        if (button_time - last_button_time > 250)
        {
            debugpln("Lights On.");
            response = httpGETRequest(serverName);
            debugpln(response);
            if (response.substring(10, 12) == "ON")
            {
                pixel.setPixelColor(0, pixel.Color(150, 0, 0));
                pixel.show();
                delay(2000);
                pixel.setPixelColor(0, pixel.Color(0, 0, 0));
                pixel.show();
            }
            if (response.substring(10, 13) == "OFF")
            {
                pixel.setPixelColor(0, pixel.Color(0, 150, 0));
                pixel.show();
                delay(2000);
                pixel.setPixelColor(0, pixel.Color(0, 0, 0));
                pixel.show();
            }
            last_button_time = button_time;
        }
        // button pressed reset sleep timer
        last_sleep_time = sleep_time;
    } // if (buttonState == LOW)

    if (WiFi.isConnected())
    {
        pixel.setPixelColor(0, pixel.Color(0, 0, 150));
        pixel.show();
    }

    if (sleep_time - last_sleep_time > ((SLEEP_DELAY_M * mS_TO_M_FACTOR)))
    {
        pixel.setPixelColor(0, pixel.Color(0, 0, 0));
        pixel.show();
        last_sleep_time = sleep_time;
        WiFi.disconnect(true); // disconnect and turn off wifi
        delay(1000);
        WiFi.mode(WIFI_OFF);
        debugpln("Going to sleep now");
        // Serial.flush();
        esp_deep_sleep_start();
        debugpln("This will never be printed");
    }

} // void loop()

String httpGETRequest(const char *serverName)
{
    WiFiClient client;
    HTTPClient http;
    String payload = "{}";

    debugpln(serverName);

    // Check WiFi connection status
    if (WiFi.status() == WL_CONNECTED)
    {
        // digitalWrite(LED_BLUE, LOW);
        //  Your Domain name with URL path or IP address with path
        http.begin(client, serverName);

        // Send HTTP POST request
        int httpResponseCode = http.GET();

        if (httpResponseCode > 0)
        {
            Serial.print("HTTP Response code: ");
            debugpln(httpResponseCode);
            payload = http.getString();
        }
        else
        {
            Serial.print("Error code: ");
            debugpln(httpResponseCode);
        }
        // Free resources
        http.end();
    } // if (WiFi.status() == WL_CONNECTED)

    return payload;
} // String httpGETRequest(const char *serverName)

void setupWiFi()
{
    // Connect WiFi
    debugpln("---------- Start WiFi Connect ----------");
    int connectCount = 0;
    // Auto reconnect is set true as default
    // To set auto connect off, use the following function
    WiFi.setAutoReconnect(true);

    WiFi.onEvent(WiFiEvent);

    debugpln(F("Connect to WiFi"));

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    debugpln("Connecting");
    while (WiFi.status() != WL_CONNECTED && connectCount < 101)
    {
        delay(500);
        debugp(".");
        connectCount++;
    }
    debugpln("");
    debugp("Connected to WiFi network with IP Address: ");
    debugpln(WiFi.localIP());
    debugpln("---------- End WiFi Connect ----------");

} // void setupWiFi()

void WiFiEvent(WiFiEvent_t event)
{
    // debugpf("[WiFi-event] event: %d\n", event);

    switch (event)
    {
    case ARDUINO_EVENT_WIFI_READY:
        debugpln("WiFi interface ready");
        break;
    case ARDUINO_EVENT_WIFI_SCAN_DONE:
        debugpln("Completed scan for access points");
        break;
    case ARDUINO_EVENT_WIFI_STA_START:
        debugpln("WiFi client started");
        break;
    case ARDUINO_EVENT_WIFI_STA_STOP:
        debugpln("WiFi clients stopped");
        break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
        debugpln("Connected to access point");
        break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        debugpln("Disconnected from WiFi access point");
        // digitalWrite(LED_BLUE, HIGH);
        break;
    case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
        debugpln("Authentication mode of access point has changed");
        break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        debugp("Obtained IP address: ");
        debugpln(WiFi.localIP());
        debugp("WiFi Signal Strength: ");
        debugpln(WiFi.RSSI());
        // digitalWrite(LED_BLUE, LOW);
        break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
        debugpln("Lost IP address and IP address is reset to 0");
        break;
    case ARDUINO_EVENT_WPS_ER_SUCCESS:
        debugpln("WiFi Protected Setup (WPS): succeeded in enrollee mode");
        break;
    case ARDUINO_EVENT_WPS_ER_FAILED:
        debugpln("WiFi Protected Setup (WPS): failed in enrollee mode");
        break;
    case ARDUINO_EVENT_WPS_ER_TIMEOUT:
        debugpln("WiFi Protected Setup (WPS): timeout in enrollee mode");
        break;
    case ARDUINO_EVENT_WPS_ER_PIN:
        debugpln("WiFi Protected Setup (WPS): pin code in enrollee mode");
        break;
    case ARDUINO_EVENT_WIFI_AP_START:
        debugpln("WiFi access point started");
        break;
    case ARDUINO_EVENT_WIFI_AP_STOP:
        debugpln("WiFi access point  stopped");
        break;
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
        debugpln("Client connected");
        break;
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
        debugpln("Client disconnected");
        break;
    case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
        debugpln("Assigned IP address to client");
        break;
    case ARDUINO_EVENT_WIFI_AP_PROBEREQRECVED:
        debugpln("Received probe request");
        break;
    case ARDUINO_EVENT_WIFI_AP_GOT_IP6:
        debugpln("AP IPv6 is preferred");
        break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
        debugpln("STA IPv6 is preferred");
        break;
    case ARDUINO_EVENT_ETH_GOT_IP6:
        debugpln("Ethernet IPv6 is preferred");
        break;
    case ARDUINO_EVENT_ETH_START:
        debugpln("Ethernet started");
        break;
    case ARDUINO_EVENT_ETH_STOP:
        debugpln("Ethernet stopped");
        break;
    case ARDUINO_EVENT_ETH_CONNECTED:
        debugpln("Ethernet connected");
        break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
        debugpln("Ethernet disconnected");
        break;
    case ARDUINO_EVENT_ETH_GOT_IP:
        debugpln("Obtained IP address");
        break;
    default:
        break;
    }
} // void WiFiEvent(WiFiEvent_t event)

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason()
{
    esp_sleep_wakeup_cause_t wakeup_reason;

    wakeup_reason = esp_sleep_get_wakeup_cause();

    switch (wakeup_reason)
    {
    case ESP_SLEEP_WAKEUP_EXT0:
        debugpln("Wakeup caused by external signal using RTC_IO");
        break;
    case ESP_SLEEP_WAKEUP_EXT1:
        debugpln("Wakeup caused by external signal using RTC_CNTL");
        break;
    case ESP_SLEEP_WAKEUP_TIMER:
        debugpln("Wakeup caused by timer");
        break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
        debugpln("Wakeup caused by touchpad");
        break;
    case ESP_SLEEP_WAKEUP_ULP:
        debugpln("Wakeup caused by ULP program");
        break;
    case ESP_SLEEP_WAKEUP_GPIO:
        debugpln("Wakeup caused by GPIO");
        break;
    default:
        Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
        break;
    }
} // void print_wakeup_reason()

// function called when a MQTT message arrived
void callback(char *p_topic, byte *p_payload, unsigned int p_length)
{
}

void checkBatteryVoltage()
{
    bool fiveVoltPin = digitalRead(FIVE_V_PIN);
    debugp("USB is Plugged In: ");
    debugpln(fiveVoltPin);
    float Vbattf = 0.0;
    String pwrSource = "";
    // If USB Plugged In, don't measure battery
    if(!fiveVoltPin) {
        // Check battery voltage
        pwrSource = "On Battery";
        uint32_t Vbatt = 0;
        for (int i = 0; i < 16; i++)
        {
            Vbatt = Vbatt + analogReadMilliVolts(A0); // ADC with correction
        }
        Vbattf = 2 * Vbatt / 16 / 1000.0; // attenuation ratio 1/2, mV --> V
        debugpln(Vbattf, 3);
    }
    else {
        pwrSource = "On USB";
    }
    debugpln("Attempting MQTT connection...");

    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD))
    {
        debugpln("Connected to MQTT Server");
    }
    else
    {
        debugpln("Error Connecting to MQTT Server");
    }
    if (mqttClient.connected())
    {
        String data = "";
        data = "{\"batt\":\"" + String(Vbattf) + "\"}";

        debugp("Voltage: ");
        debugpln((data));

        if (mqttClient.publish(MQTT_SENSOR_TOPIC, data.c_str(), true))
        {
            debugpln("MQTT Message sent successfully");
        }
        else
        {
            debugpln("Error sending MQTT Message");
        }

        data = "";
        data = "{\"pwr\":\"" + pwrSource + "\"}";
        //data = "{\"usb\":\"" + String(fiveVoltPin) + "\"}";

        debugp("Power: ");
        debugpln((data));

        if (mqttClient.publish(MQTT_SENSOR_TOPIC, data.c_str(), true))
        {
            debugpln("MQTT Message sent successfully");
        }
        else
        {
            debugpln("Error sending MQTT Message");
        }

        debugpln("INFO: Closing the MQTT connection");
        mqttClient.disconnect();
    } // if (mqttClient.connected())
} // void checkBatteryVoltage()