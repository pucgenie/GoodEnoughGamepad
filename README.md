# Gampad firmwae for Arduino-compatible Pro Micro µC

## Why
My dev environment broke down while playing with DInput, so I re-implemented a gamepad firmware using [XInput](https://github.com/dmadison/ArduinoXInput).

## Differences to example code
Tried to implement basic Reflex-like strategies to account for input lag synchronization to USB poll "events".

## Flaws
Joystick code is untested.
Trigger code is untested.

## Known errors
Both triggers report a state of PRESSED.
