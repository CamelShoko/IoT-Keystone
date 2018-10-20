# IoT-Keystone: _The_ easiest and most secure way to move sensor data to the cloud.

[![Build Status](https://travis-ci.org/contiki-ng/contiki-ng.svg?branch=master)](https://travis-ci.org/thisisiot/iot-keystone/branches)
[![license](https://img.shields.io/badge/license-3--clause%20bsd-brightgreen.svg)](https://github.com/thisisiot/iot-keystone/blob/master/LICENSE.md)
[![Latest release](https://img.shields.io/github/release/contiki-ng/contiki-ng.svg)](https://github.com/thisisiot/iot-keystone/releases/latest)
[![GitHub Release Date](https://img.shields.io/github/release-date/contiki-ng/contiki-ng.svg)](https://github.com/thisisiot/iot-keystone/releases/latest)
[![Last commit](https://img.shields.io/github/last-commit/contiki-ng/contiki-ng.svg)](https://github.com/thisisiot/iot-keystone/commit/HEAD)


IoT-Keystone is a simple-to-use yet powerful and secure IoT sensor device communication platform. It is designed to run on the This.Is.IoT. Keystone hardware boards and modules aimed at building "sensor to cloud" IoT solutions.  The This.Is.IoT. Keystone firmware and hardawre platform uniquely utilizes the sub-GHz band by making available both 6LoWPAN mesh and LoRaWAN to the same application.  This. Is. IoT. Keystone is built on *open* and *standardized* communication and security protocols that will power the next phase of global interoperable IoT.

## IoT.Keystone Firmware Features

* Supports 6LoWPAN IPv6 mesh stack standard and CC1352 802.15.4 sub-GHz radio.
* Integrates latest Semtech LoRaWAN stack standard for SX1262 LoRa radio.
* Supports dynamic BLE-compatible beacon generation.
* Elliptic curve crypto engine for securely signing authentic transactions to the cloud.
* Supports a variety of cloud data platforms including _blockchain_
* Supports a comprehensive array of sensors, including:
  * Acceleration
  * Gyroscope
  * Magnetometer
  * Light 
  * Sound 
  * Temperature
  * Pressure
  * Humidity
  * Magnetic Hall effect
* Console interface
* Variety of example applications including:
  * LoRaWAN-to-cloud
  * mesh-to-cloud
  * data logger
  * dynamic beacon
  

* Mesh communication offers solution builders a medium range, lower latency and bi-directional communication channel for sensing and control applications.

* LoRaWAN offers solution builders a long range smart-city sensor reporting channel that is growing in popularity around the world.

IoT-Keystone is based on Contiki-NG: an open-source, cross-platform operating system for Next-Generation IoT devices. It focuses on dependable (secure and reliable) low-power communication and standard protocols, such as IPv6/6LoWPAN, 6TiSCH, RPL, and CoAP.  IoT-Keystone adds a number of new features to Contiki-NG, including a LoRaWAN MAC.

## IoT.Keystone Community Experimenter Board v1 Features

The IoT.Keystone community development board is the easiest and most secure way to get your sensor data to where you need it.  It features these unprecedented features in an extremely low power and robust design, ready to go out of the box:

* All-in-one board, no external components required.
* Integrated efficient sub-GHz antenna.
* Integrated 2.4 GHz antenna.
* Integrated USB type-A connector for battery charging, console IO, application configuration and interaction.
* Integrated Li-Po battery charger.
* Integrated Li-Po battery.
* Powerful yet low power ARM Cortex-M4F MCU with 352 KB RAM operating at 48 MHz with integrated IPv6 sub-GHz mesh radio and BLE-compatible 2.4 GHz radio.
* Enhanced ESD protection.
* BME280 sensor (temperature, pressure, humidity)
* OPT3001 sensor (light)
* MP34DT05 sensor (sound)
* DRV5032 sensor (magnetic hall effect)
* ICM-20948 9-axis advanced IMU (acceleration, gyroscope, magnetometer)



Unless excplicitly stated otherwise, IoT-Keystone sources are distributed under
the terms of the [3-clause BSD license](LICENSE.md). This license gives
everyone the right to use and distribute the code, either in binary or
source code format, as long as the copyright license is retained in
the source code.

IoT-Keystone started as a fork of the Contiki-NG OS and retains many of its feature as well as adds new ones.

Find out more:

* GitHub repository: https://github.com/ThisIsIoT/IoT-Keystone
* Documentation: https://github.com/ThisIsIoT/IoT-Keystone/wiki
* Web site: http://thisisiot.io

Engage with the community:

* Twitter: https://twitter.com/thisisiot_io
