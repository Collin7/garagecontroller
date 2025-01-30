# Garage Controller

This firmware is slightly more than just a garage door controller.

- In this sketch exists the functunality to report back on garage door states (OPEN/CLOSED)
- It allows for opening or closing the garage doors via MQTT from an APP
- The garage door keypad project will communicate with this device to open or close a door if the access code is successful.
- Added to this microprocessor is also a DHT22 sensor to report Temperature and Relative Humidity of the garage environment.
- Updated project to use a ESP32 devkit board and added two ultra sonic sensors to monitor if cars are parked in the garage

