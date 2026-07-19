# 6-axis stepper Robot controller firmware

[![Latest release](https://img.shields.io/github/v/release/BlazGo/robot_controller?include_prereleases&display_name=release)](https://github.com/BlazGo/robot_controller/releases)

Basic robotic controller implementation on a RP2350 based microcontroller. Enables functionality such as homing, kinematics and inverse kinematics calculations, joint moves, cartesian moves, reading of values, software based self checking joint limits and control via USB connection from PC.

## Basic info and specs of the robot

6axis stepper motor based. Similar configuration as most common industrial robots.

## Requirements

### Hardware

1x Raspberry Pi RP2350B microcontroller development board (WeAct Studio)
1x RS485 to TTL (UART) module
5x WeAct STM32G030 development board
5x I2C magnetic encoders MT6701
5X RS485 to TTL (UART) module (small)
1x I2C based OLED screen 2.4"
2x Stepper motor interface boards
1x Nema 23 stepper
1x Nema 17 stepper with 5.14 ratio planetary gearbox
1x Nema 17 stepper with 14.0 ratio planetary gearbox
2x Nema 17 stepper
1x Nema 17 stepper (short)
2x End switches
1x 150W 12V power supply
1x EU power socket
1x 12V 40mm fan
1x DC-DC step down converter (12V -> 3.3V)
1x bib bearing j0
6x medium bearings j1, j2, j3
2x medium small bearings j4
1x small bearing j5
6x small bearing j0 (stabilisation -> if we would want just one really big bearing it would be much more expensive while Lazy susan bearing was discovered only later during my build process... (also not that cheap if you want a proper higher end one))
Bunch of GT2 belts of various lengths 6mm and 10mm
Pulleys to fit on stepper motors, while the rest of them on the actual joints are custom and 3D printed and are incorporated into the actual structure of the robot.
6mm, 10mm idlers
(a bunch of screws and wires)
The actual structure of the robot is 3D printed. Model will be uploaded and made avaliable via link.

TODO: count the actual numbers and check precise names/designations of items

### Software

Visual studio Code
PlatformIO extension
Raspbery PI Pico related Toolchain

## How to build

This project was programmed and is tested to work with Visual studio Code, and PlatformIO extension
The hardware used is RP2350B WeAct based development board. More on used hardware: [link]

There is also firmware required on secondary MCU-s. The encoder RS485 nodes have individual MCU-s which continuously measure I2C magnetic encoder angle and make it avaliable to the main MCU via a RS485 connection. Firmware for that is avaliable here: [link]

## Known issues

- Standing still despite accepting a movement command 
    During development I came across a bug that despite commanding a joint move the robot remained still. Upon closer inspection some super small speed was applied but it seems that during update the effect of that speed in that single step was not enough (or what is more likely too little time passed since setting the speed and AccelStepper library update of the steppers "update" call causing no steps to be executed) Because of that with implementation of trapezoidal profile we were always at the original start position which means we will again command the minimum speed with applied single step acceleration. (current theory) -> This was solved with implementation of a separate value of desired speed which has no actual feedback of the true current position of the steppers... Not ideal!
- Encoder updating
    Currently encoder updating of position is not intended to be implemented to constantly work. Instead it will only update and checkl the position when commanded to and during the homing procedure (also after boot)
- The codebase is a mess... :(
