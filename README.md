LPC1114 Synthesizer
===================

![](http://farm4.staticflickr.com/3831/11042374936_e6a76aa30f.jpg)

Overview video: [http://youtu.be/DU8IyjyjDfk](http://youtu.be/DU8IyjyjDfk)

Demonstration video: [http://youtu.be/o-3MleRX-aE](http://youtu.be/o-3MleRX-aE)

This is a digital monophonic synthesizer built using the [NXP LPC1114FN28](http://www.nxp.com/products/microcontrollers/cortex_m0_m0/LPC1114FN28.html), a 32-bit ARM Cortex-M0 microcontroller in a breadboardable DIP-28 package. Features include:

- 4 oscillators; sawtooth or pulse with adjustable duty cycle with coarse and fine tuning
- 2-pole ("Chamberlin") state-variable filter with lowpass, highpass, and bandpass modes
- Attack-release envelopes for amplitude and modulation
- Low-frequency oscillator with four shapes (triangle, ramp, square, random)
- LFO and/or modulation envelope can affect filter cutoff frequency, pitch, and pulse width
- Keyboard tracking for filter cutoff frequency
- Glide with 3 different rate presets
- MIDI input; monophonic with last-note priority
- 250kHz, 12-bit output
- Powered by 3 AA batteries

OK, so it's not exactly a Moog, but it was a fun project to build a synth capable of cool sounds using relatively minimal hardware. The only essential chips used are the microcontroller and an [MCP4921 SPI DAC](http://ww1.microchip.com/downloads/en/DeviceDoc/21897a.pdf). A 74HC165 and 74HC4053 increase the number of input channels; they probably wouldn't be necessary if I used one of the 48-pin surface-mount variants of the LPC1114.

The firmware is written in C, except for the sample-generation code, which is written in assembly. It is responsible for updating the four oscillators, running the filter, and sending samples to the DAC. For ultimate speed, it's run from RAM, and self-modifying code is used to update parameters, minimizing the number of branches required. With a 50MHz clock speed, the microcontroller can output samples at 250kHz without breaking a sweat.

Less-critical functions, like updating the LFO, envelopes, and glide, are performed at a lower frequency using routines written in C.

The synth has bare-bones MIDI support; as of this writing it only supports note-on/note-off (without velocity sensitivity) and pitch-bend events, on channel 1. I have been using it with a [Rock Band keyboard controller](http://www.amazon.com/Rock-Band-Wireless-Keyboard-Xbox-360/dp/B003RS19N4) and it works great.

## The controls

To play, connect a MIDI controller to the MIDI in port, and connect the stereo jack to some form of audio equipment. The synth currently cannot make any sound without MIDI input. There is an unused ADC channel that's reserved for controlling pitch with an external sensor input, but this has not been implemented.

### Waveform knob
- At its halfway point, all oscillators will output square waves.
- Turning the knob to the left decreases the pulse width.
- Turning the knob past the halfway point outputs sawtooth waves on two oscillators and pulse waves on the other two.
- Turning the knob all the way to the right outputs sawtooth waves on all oscillators.

### Detune knob

The detune knob's range is divided into four sections. Each section detunes the oscillators in a different way.

- When all the way to the left, all oscillators will play in tune.
- In the first section, two oscillators will be detuned sharp by an amount determined by the knob's position within the section. (clockwise = more detuned)
- In the second section, one oscillator will be detuned sharp by the specified amount, and another oscillator will be detuned flat by the same amount.
- In the third section, one oscillator will be detuned sharp by the specified amount, another will be detuned sharp by twice the specified amount, and a third will be detuned flat by half the specified amount.
- In the fourth section, one oscillator will be detuned by the specified amount, another will be detuned sharp by three times the specified amount, and a third will be detuned flat by two times the specified amount.

#### Chord programming

Although the synthesizer is monophonic, each oscillator can be tuned to play a different note if desired.

- First, press the "chord pgm." button. The red LED next to it should illuminate.
- Play up to four notes. The first note pressed will be treated as the root note, and the intervals between the successive notes and the root note will be used to tune the other three oscillators.
    - For example, if you play C4, G4, C5, and C3, the second oscillator will be tuned a perfect fifth above the root note, the third will be tuned an octave above the root note, and the fourth will be tuned an octave below the root note.
- Chord programming mode ends after the fourth note has been played, or if the button is pressed again. If less than four notes were played, the remaining oscillators will play in unison with the first.
- Pressing the "chord pgm." button twice without playing any notes resets all oscillators to unison tuning.

### Amplitude envelope

When the green "amp" light is on, the "attack" and "release" knobs control the amplitude's attack and release time. Turning the knobs all the way counterclockwise sets instantaneous attack/release. Turning the knobs clockwise increases the attack/release time.

The "keyed/sustain/repeat" switch determines what happens when a note is held.

- In the "keyed" position, a note release immediately after it is played. There is no sustain.
- In the "sustain" position, a note continues to sound as long as the key is held down.
- In the "repeat" position, the attack/release envelope is triggered repeatedly (without sustain) until the key is released.

### Filter

The "cutoff" knob controls the filter's cutoff frequency. In its leftmost position, no notes should sound at all. Turning the knob all the way to the right increases the cutoff frequency.

The "resonance" knob controls the "Q" parameter; peak emphasis at the cutoff frequency. Be careful when turning the knob all the way to the right, large resonance values currently cause the filter output to clip violently, and possibly overflow. The filter does not self-oscillate.

The filter is in lowpass mode on startup. To change the filter mode, hold down the "shift" button (the same as the "chord pgm." button) and press the "lfo shape/filter mode" button. The color of the LED indicates the filter mode:

- Cyan: lowpass
- Magenta: highpass
- Yellow: bandpass
- Off: filter is bypassed

### Modulation

The synthesizer has two modulation sources: a low-frequency oscillator, and a second envelope. The two "modulation" switches control how these sources are route to various parameters:

- The "cutoff/pw" switch sets the modulation sources that affect the filter cutoff frequency **unless** the filter is off, in which case the switch sets the sources that affect the pulse width of oscillators set to pulse.
- The "pitch" switch sets the modulation sources that affect oscillator pitch.
- The "off" setting turns off modulation for a given parameter.
- The "env" setting allows the modulation envelope to modulate a given parameter.
- The "lfo" setting allows the low-frequency oscillator to modulate a given parameter.
- The "both" setting allows both the modulation envelope **and** the low-frequency oscillator to modulate a given parameter.

#### Low-frequency oscillator

The "lfo rate" controls the speed of the LFO. Turning it to the right increases the speed. Pressing the "lfo shape" button cycles through LFO waveforms. The color of the LED indicates the current waveform:

- Red: triangle
- Green: ramp
- Blue: square
- White: random

#### Modulation envelope

Pressing the "env select" button changes which envelope is affected by the "attack" and "release" knobs. On startup, the amplitude envelope is selected, indicated by the green "amp" LED. Press "env select" to select the modulation envelope, indicated by the yellow "mod" LED.

When the modulation envelope is selected, turning the "attack" and "release" knobs changes the attack and release time of the modulation envelope. The attack and release of the amplitude envelope are not affected.

#### Modulation amount

The "cutoff/pw mod amt." controls how much the cutoff frequency (or pulse width if the filter is off) is affected by the modulation sources.

- When the knob is in the center position, modulation has no effect.
- When the knob is turned past center to the right, modulation **increases** the cutoff frequency/pulse width.
- When the knob is turned past center to the left, modulation **decreases** the cutoff frequency/pulse width.

The pitch modulation range can be set using the "pitch mod pgm." button. On startup, it is set to +12 semitones.

- Press the "pitch mod pgm." button. The red LED next to it should illuminate.
- Play two notes. The difference in pitch between the two notes is then used as the new pitch modulation range.
- If the second note was higher than the first, modulation will **increase** the pitch.
- If the second note was lower than the first, modulation will **decrease** the pitch.
- Pitch modulation programming mode can be cancelled by pressing the button again before two notes have been played.

### Glide

Glide, or portamento, causes smooth sliding transitions between pitches when a note is played while another is held down. Pressing the "glide" button enables or disables glide, with one of three rate presets.

- On startup, glide is off.
- Pressing the "glide" button once enables glide, with a fast rate. The LED next to the button illuminates.
- Pressing the button again sets a medium glide rate. The LED next to the button blinks twice and stays on.
- Pressing the button a third time sets a slow glide rate. The LED next to the button blinks three times and stays on.
- Pressing the button a fourth time disables glide. The LED next to the button turns off.

### Keyboard tracking

Keyboard tracking, which is on by default at startup, adjusts the filter's cutoff frequency based on the pitch of the note being played. This gives a uniform timbre regardless of note pitch.

Keyboard tracking can be toggled on and off by holding the "shift" button and pressing the "pitch mod pgm./kb tracking" button. When the "shift" button is held down, the red LED next to the button indicates whether keyboard tracking is enabled.

### Legato

Legato is off by default at startup. When it is off, every new note played retriggers the modulation envelope and LFO. When it is on, if a second note is played while the first is still held down, the modulation envelope and LFO will not be affected.

Legato can be toggled on and off by holding the "shift" button and pressing the "glide/legato" button. When the "shift" button is held down, the red LED next to the button indicates whether legato is enabled.

The modulation envelope and LFO are always reset when a note is played without any keys held down.

## Building it/hacking it

You'll need:

- The [bill of materials](circuit/bill_of_materials.txt) and [schematic](circuit/lpc1114-synth.pdf)
- The [GNU ARM toolchain](https://launchpad.net/gcc-arm-embedded)
- [lpc21isp](https://github.com/capiman/lpc21isp) for programming the chip
- An [FTDI cable](https://www.sparkfun.com/products/9717) or other USB-TTL serial interface (5V or 3.3V, doesn't matter)
- The source code, of course

After downloading the source code, just type `make` to build. To send the firmware to the chip:

1. Connect the FTDI cable to a USB port on your computer and the 6-pin header on the synthesizer.
2. Set the `FTDI_DEV` environment variable to the name of the cable's serial device. On Mac OS X, this will be something like `/dev/cu.usbserial-A400fYRW`. On Linux, it should be something like `/dev/ttyUSB0`.
3. Disconnect any attached MIDI devices.
4. While holding the "env select" button, turn on the synthesizer. The microcontroller should be in programming mode; no LEDs should be lit.
5. Run `make flash` on your computer.
6. After programming the console should say `Now launching the brand new code`, but nothing seems to happen. Press the "chord pgm." button to reset the microcontroller and should now be running the firmware.

To reprogram the firmware while the device is powered on:

1. Disconnect any attached MIDI devices.
2. Hold down the "env select" button for a couple seconds. When the LEDs turn off, the microcontroller is in programming mode.
3. Follow steps 5-6 above.

(Note that the firmware must already be programmed for the above to work.)

## Code overview

- [main.c](main.c) contains all the code for setting up the hardware, reading the switches/knobs, and controlling the LEDs.
    - The `SysTick` timer is set up to trigger an interrupt at 250kHz.
    - 32-bit Timer 0 is used to trigger an interrupt at 250Hz; this is used to update the LFO, envelopes, glide, and LEDs.
    - All 6 ADC channels are used, and the upper 3 channels are multiplexed using a 74HC4053, for a total of 9 digital inputs. One is unused.
    - Two digital inputs are used to read the "env select" and "chord pgm." buttons; the rest are read in serially via the 74HC165 shift register during an SPI transfer.
- [sound.c](sound.c) controls all the functions that update the synth's parameters. It also contains the Timer 0 interrupt handler.
- [midi.c](midi.c) implements the UART interrupt handler and interprets MIDI commands. If you want to add more MIDI functionality, this is the place to do so.
- [hardware.c](hardware.c) contains some hardware setup routines.
- [kernel.S](kernel.S) contains the `SysTick` interrupt handler, written in assembly language. It updates the 4 oscillators and filter, and outputs samples to the DAC via SPI.
- The [lpc1xxx/](lpc1xxx) directory contains header files and setup functions for the LPC1114, as well as a linker script. Most of its contents were adapted from the [Microbuilder LPC1114 codebase](http://www.microbuilder.eu/Projects/LPC1114ReferenceDesign/LPC1114CodeBase.aspx) and stripped down to the bare essentials.
- [tablegen.py](tablegen.py) is a script that generates lookup tables from Python code. The synthesizer uses numerous lookup tables to convert knob positions to parameter values, etc. Modifying this file and any of the associated .py files will let you change knob scales and more.
    - The Python scripts generate C code at compile time; tablegen.py is automatically invoked by `make`.
    - If you change any of the generator scripts, a simple `make` is all you need to do.
    - If you change any of the global parameters that affect all tables (like clock frequency) you should `make clean all`.
    - The global parameters are not in any way tied to the C code (yet), so if you change one parameter (e.g. clock frequency) you will have to change it in both main.c **and** tablegen.py.

## Shortcomings/bugs/TODO

- The filter is prone to hard-clip with high resonance values, and this doesn't sound very good.
- Turning resonance all the way up can overflow the filter state and cause the synthesizer to go silent.
- Having to disconnect the MIDI cable to reprogram is kind of inconvenient.
- No MIDI control change or program change messages are interpreted.
- Some of the knob scales need to be tweaked.
- LEDs cannot be dimmed, so the RGB LED can only display 7 colors.
- There's an unused ADC channel that could be used to play the synthesizer without a MIDI controller.

## Thanks/references

- [*Musical Applications of Microprocessors*](http://www.scribd.com/doc/40791523/Musical-Applications-of-Microprocessors-2ed-Chamberlin-H-1987) by Hal Chamberlin. An absolute goldmine of information about sound synthesis on small embedded systems.
- [Digital Sound Generation - Part 2](http://courses.cs.washington.edu/courses/cse490s/11au/Readings/Digital_Sound_Generation_2.pdf) by Beat Frei. Contains tips for improving the stability of the Chamberlin filter.
- [The Microbuilder LPC1114 codebase](http://www.microbuilder.eu/Projects/LPC1114ReferenceDesign/LPC1114CodeBase.aspx) for a decent Makefile and hardware routines for LPC1114 projects.
- [Marc Doty](http://www.youtube.com/user/AutomaticGainsay) for his awesome synth videos.
- [Josh Myer](http://joshisanerd.com) for giving me a free LPC1114 after drinks at Shotwell's.
