Arduino-Diseqc-solar-tracker
============================

// e_callisto_drive uses an arduino to control a DiSEqC 1.2 satellite motor to
// an antenna can track the Sun in right ascension.
// The arduino calculates the position of the Sun and sends tone controls to
// the drive via pin 8. Extrernal electronics are required to inject the tone
// signals onto the drive power lines.
//
// Single precision limits the angular precision. A comparison with the JPL Horizons
// system show discrete errors of up to 1 minute in hour angle without the lookup tables.
// With the tables the errors are less (not checcked how much less!)
//
// 22 kHz signal should be set to 650 mV p-p
// see http://www.eutelsat.com/satellites/pdf/Diseqc/Reference%20docs/bus_spec.pdf
// and http://www.eutelsat.com/satellites/pdf/Diseqc/associated%20docs/position_app_note_v1.pdf
// but note these are incomplete.
//
// Set the time at the serial console (9600 baud) with the format "yyyy mm dd hh mm ss".
//
// Graham Woan 13/10/2012
