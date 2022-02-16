PoolSpaController
=================

A microprocessor-based pool and spa controller

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

 - A 4-line by 20 character LCD display
 - ten waterproof lighted pushbutton
 - a rotary encoder for temperature control
 - a Maxim DS18B20 thermometer
 - a battery backed-up realtime clock using a Maxim DS1307 clock chip
 - ten relays for controlling pumps, heater, valves, and lights

Version 2 of the controller dates from 2014. It used an Arduino MEGA 2560 
microcontroller and was implemented with breadboards and jumper wires. It worked 
fine for 8 years, but the Pandemic was a good opportunity to redo it with a more 
modern processor and more robust printed circuit boards. 

Version 3 was created in 2022 and works, but as of 2/15/22 hasn't yet been 
installed. It uses an ESP32 dual-core processor with WiFi, so you can query and 
control the system from any web browser. I'm currently using the Adafruit 
HUZZAH32 ESP32 Feather Board, but I might switch to an ESP32 development board 
like https://www.amazon.com/dp/B09BM1QW29 in order to be able to use a 
higher-gain external antenna. 

There is a little PCB that mounts to the back of each lighted pushbutton, and 
they are daisy-chained together with short flexible flat cables. The PCB 
"rotates" the lines to the lights and buttons so that each has a unique address 
to the CPU yet all the boards are identical. This is a simplification of a 
scheme we patented 40 years ago at Nestar Systems! 
https://patents.google.com/patent/US4253087A/en. 
