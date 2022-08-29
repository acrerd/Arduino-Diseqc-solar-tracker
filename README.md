Arduino-Diseqc-solar-tracker
============================

callisto_drive uses an arduino to control a DiSEqC 1.2 satellite motor so
an antenna can track the Sun in right ascension.
The arduino calculates the position of the Sun and sends tone controls to
the drive via pin 8. External electronics are required to inject the tone
signals onto the drive power lines.

Single-precision arithmetic limits the angular precision. A comparison with the JPL Horizons
system show discrete errors of up to 1 minute in hour angle without the lookup tables.
With the tables the errors are less (not checked how much less!)

22 kHz signal should be set to 650 mV p-p

see position_app_note_v1.pdf in https://www.eutelsat.com/files/PDF/DiSEqC-documentation.zip
but note these notes are incomplete.

Set the time at the serial console (9600 baud) with the format "yyyy mm dd hh mm ss".

Version 1.1 includes code to mitigaste the 2020 GPS rolloover bug, for receiver modules that are subject to it (like ours!).

Graham Woan 
