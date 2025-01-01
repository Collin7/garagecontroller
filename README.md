# Garage Controller

This firmware is slightly more than just a garage door controller.

- In this sketch exists the functunality to report back on garage door states (OPEN/CLOSED)
- It allows for opening or closing the garage doors via MQTT from an APP
- The garage door keypad project will communicate with this device top open or close a door if the access code is successful.
- Added to this microprocessor is also a DHT22 sensor to report Temperature and Relative Humidity of the garage environment.
- Changed board to ESP32 and added two sonic sensors to monitor if cars are in the garage

