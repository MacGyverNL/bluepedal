# BluePedal (Arduino)

Arduino software for my (currently) nRF52840-based Bluetooth Low Energy
foot pedals for page turning.

Runs completely without Arduino's loop, using digital interrupts,
timers, and callbacks for all functionality. Because why spend a
shitload of energy on polling input pins if you only press a switch
every twenty-ish seconds?

Probably requires the edits in SoftwareTimer.h in
https://github.com/MacGyverNL/Adafruit_nRF52_Arduino
because resetting / starting timers in an ISR is somehow special in
freeRTOS. Have not actually attempted running the normal start/stop/
reset functions from an ISR.

