# StandAloneAlarmSystem

The standalone alarm system is a multiple priority trigger based alarm system programmed on
the PIC18F4520 microcontroller in C. More specifically, it was programmed via in-circuit
programming using a PICkit3 and the MPLAB X IDE.

The main idea behind the security of the alarm is that the user sets authentication at the first
boot up, and from there on will be enforced based on the passcode given. The alarm itself is
triggered through a rise in temperature, or any unusual motion picked up from either sensor.
There are multiple configurations for this alarm system including: changing the passcode,
turning on the motion alarm, turning on the temperature alarm or just changing the main input
from the keyboard to a keypad hooked on to the circuit.

The system’s status can always be monitored through the LED’s. The LED colors corresponding
to the status are as follows: green indicated a running system, red indicated a trigger in motion,
yellow a sample of temperature and blue indicated the keypad is ready for use. Alternatively,
the system’s status can be seen on the output of the screen above the menu options.
