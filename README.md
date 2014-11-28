PoolSpaController
=================

An Arduiino-based pool and spa controller

This is a custom-built combination pool and spa controller for a system that has:
 - two circulation pumps (pool, spa)
 - a bubbler pump
 - an electronically-controlled heater (Raypack 266A)
 - three electric Jandy valves that provide four configurations:
    - heat spa
    - heat pool
    - fill spa
    - empty spa
 - pool lights

The controller contains the following components:
 - Arduino MEGA 2560 microcontroller
 - A 4-line by 20 character LCD display
 - ten waterproof lighted pushbutton
 - a rotary encoder for temperature control
 - a Maxim DS18B20 thermometer
 - a Chronodot battery backed-up realtime clock with a Maxim DS1307 clock chip
 - ten optically-isolated relays for controlling pumps, heater, valves, and lights

For some photos and the schematic of the controller see the "issues" for this project.
