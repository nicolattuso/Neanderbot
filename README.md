# Neanderbot

This repository contains the code used in 2014 at the French Robotics Cup.
The theme was "Prehistobot"

The code runs on a Raspberry Pi model 1 B

## Using the Raspberry Pi

- to log in, branch the Raspberry to your laptop (direct ethernet connection)
- our Raspberry Pi has a fixed IP: 10.42.0.10
- on ubuntu, click on the network icon in the upper right of the desktop
  - click on modify connections
  - click on ethernet
  - click on IPv4 Parameters
  - choose the method "Shared with other computers"
  - save the settings
- check that your computer has an IP in the 10.42.0.0/16 range: `ifconfig`
- if it is the case, you can log in: `ssh pi@10.42.0.10`

## Installing WiringPi

- log in to the Raspberry pi
- run the following commands:
  - cd /tmp
  - git clone git://git.drogon.net/wiringPi
  - cd wiringPi
  - ./build
- check that the library and tools were correctly installed: `gpio -v`
