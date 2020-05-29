# Test Utilities for Gottlieb System 80/80A/80B CPU Boards

These sketches allow an [Arduino Mega 2560](https://store.arduino.cc/usa/mega-2560-r3) to be connected to the CPU board for Gottlieb System 80 pinball machines to diagnose boot issues.

These have been tested with an Ice Fever (System 80A) and an Arena (System 80B). Your mileage may vary with others.


## gottlieb\_sys80\_test.ino

Remove the CPU (U1) and connect the Arduino to its socket using jumper wires. (The connection diagram is in the sketch.)

Provides the following utilities, which can be accessed through the serial terminal:

- dump U2 ROM (System 80B: lower half of the piggyback ROM)
- dump U3 ROM (System 80B: upper half of the piggyback ROM)
- dump game PROM
- test U4 RIOT RAM
- test U5 RIOT RAM
- test U6 RIOT RAM
- test all RIOT RAMs
- test CMOS RAM


## address\_decode\_tester.ino

Displays the state of the chip select lines to ensure proper address decoding. It does not drive the bus in its currents state; you'll have to pull the CPU and manually tie address lines high/low to verify them. Plenty of room for improvement here.


## trace\_6502.ino

6502 bus capture and code disassembly. When a trigger condition occurs (a reset vector fetch or an immediate capture), the sketch will capture the state of the 6502 bus into a buffer in RAM (currently 2048 entries) and print a disassembly of the executed instructions on the serial console.


## License

The contents of this repository are released to the public domain.

## About

By Matt Sarnoff. [twitter.com/txsector](https://twitter.com/txsector)

June 2018
