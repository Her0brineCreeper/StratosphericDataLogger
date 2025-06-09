# StratosphericDataLogger
This repository contains the design and source code of the Stratospheric Data Logger developed by Bryan Hoxha during the European Project Semester 2025 at Politehnica Bucharest.
It was developed to log the measurements of a number of sensors onto a microSD card during a stratospheric balloon flight.
# Hardware
- ESP32 microcontroller (a FireBeetle 2 ESP32-E was used for this project)
- 2x BME280 for inside and outside the payload
- MS5611 pressure sensor (inside)
- SCD41 CO₂ sensor (inside)
- SI1145 light/UV sensor (outside)
- RCB-09T-01 GPS module (other GPS modules should work fine as well)
- SPI microSD card module
- custom-made PCB (made in KiCad v8) with modular design and two extra outputs (I²C) for additional sensors
- one of which was used for an MPU6050 accelerometer/gyroscope (inside)
# Software
- made in Arduino IDE
- logs sensor measurements to a CSV file and outputs them to Serial Monitor
- Sensors are hot pluggable during operation because their presence is checked during every measurement cycle.
