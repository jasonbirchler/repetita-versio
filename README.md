# Repetita Versio

A versatile looper for the Versio platform.

## Etymology

Repetita: from Latin Repetere for “repeat” (as in Repetita Iuvant, meaning “repeating does good”)

Versio: from Latin for “versatile”

“Versatile repetitor”

## Controls

- Knob 1 > [blend]
- Knob 2 > [start]
- Knob 3 > [tone]
- Knob 4 > [size]
- knob 5 > [decay]
- Knob 6 > [rate]
- Knob 7 > [freeze]
- ABC > [channel]
- XYZ > [play]
- FSU > [trigger]

**Blend:** Dry/wet balance control.

**Start:** Sets the loop starting/retriggering point.

**Tone:** Applies a band-pass resonant filter to the dry signal, pre-recording:
- ccw > no filter;
- cw > cutoff at 1000 Hz.

**Size:** Changes the loop size:
- ccw > full loop length but reversed, goes backwards;
- noon > note mode;
- cw > full loop length, goes forward.

**Decay:** Controls the level of decay of the recorded signal:
- ccw > instant decay;
- cw > no decay.

**Rate:** The rate of reproduction of the buffer. In note mode, controls the transposition (from -4 octaves to +1 octave):
- ccw > slowest rate (0.2x);
- noon > normal rate (1x);
- cw > fastes rate (4x).

**Freeze:** Progressively freezes the buffer:
- ccw > completely un-frozen (recording);
- cw > completely frozen (no recording).

**Channel:** The top switch set the channel currently controlled:
- left > left channel;
- center > both channels are controlled;
- right > right channel.

**Play mode:** The bottom switch sets how the play is triggered:
- left > momentary, no restart, slowly accelerates/decelerates while changing the pitch;
- center > one shot, restarts at loop start point;
- right > continuously loops.

**Play input:** Gate or trigger depending on the relative switch.

## Buffering

When the module starts the buffer is empty, and because of this no wet signal
(the one read from the buffer) is present.
The initial buffering can be stopped by pressing the button, this way the
maximum loop length will be shorter than the total buffer length (150 seconds).
Cleaning the buffer and restarting the buffering operation can be done in edit
mode (see below) by pressing and holding the button for more than 1 second.

## Triggering the looper

When the looper is in either loop or trigger mode the rapid press of the button
restarts the looper at the **start point**.
In gate mode the looper plays while the button is pressed.
The same behaviour applies when the module receives a positive voltage at the
**play input**.

## Edit mode

Edit mode is entered by pressing and holding the button for more than 0.25
seconds when the module is either in loop or in trigger mode.

## Global options

Global options are accessed in any mode keeping the button pressed for more than 0.25 seconds.
In this scenario the knobs acts as follow:

**Blend:** Input gain:
- ccw > 0 (no input);
- cw > 5x.

**Tone:** Filter type:
- ccw > low pass;
- noon > band pass;
- cw > high pass.

**Decay:** Stereo image:
- ccw > no image;
- cw > full image.