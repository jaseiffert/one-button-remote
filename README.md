# one-button-remote
XIAO ESP32C3 One Button Remote for SONOFF Basic Switch Programmed with Tasmota

Things you will need to configure in the code.
- WiFi SSID and Password
- MQTT_CLIENT_ID
- MQTT_SERVER_IP
- MQTT_SERVER_PORT (if different than the default port)
- MQTT_USER
- MQTT_PASSWORD
- MQTT_SENSOR_TOPIC
- serverName (just the IP Address. Leave the rest of the line alone.)

If you want to change the amount of time before it goes to sleep change this: SLEEP_DELAY_M (number in minutes)

I'm just a Hobbiest Programmer and at Electronics so feel free to make suggestions to make it better.

I use this One Button Remote every day.

For the Hardware assembly and 3D printed case go here: https://www.thingiverse.com/thing:7022509

There is only one program file so you should be able to bring in the code into Arduino 2.x and program your ESP32C3 from there if your not familiar with VSCode and PlatformIO. Probably just create a new project and copy in the code. Make sure to add the libraries listed in the platformio.ini file.

I haven't tried it but programming the ESP32C3 with ESPHome should work though not directly with the SONOFF Basic with Tasmota.
You would have to set it up to go through Home Assistant to control the SONOFF Basic or any other HA compatable switch.