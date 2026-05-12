# Daisy Patch.init() port of the SynthUX Audrey II Feedback Synth

## Description

Reconfigured controls to work with the Patch.init(), added CV control for primary params, and incorporated the audio input (L) into the feedback path

## Controls

| Pin Name | Pin Location | Function | Comment |
| --- | --- | --- | --- |
| CV_1 | C5 | Frequency | Base freqency for string excitation |
| CV_2 | C4 | Feedback | Feedback amount; mix of audio input and feedback |
| CV_3 | C3 | Body - LP/HP filter | Both filters are active in the feedback path; control sweeps the frequency of low and high pass filters |
| CV_4 | C2 | Delay > Feedback | Mixes delay amount and delay time into the feedback path |
| CV_5 | C6 | 1V/Oct Input | Pitch tracking input (WIP; not perfectly calibrated yet) |
| CV_6 | C7 | Feedback CV | (adds to feedback pot) |
| CV_7 | C8 | Filter CV | (adds to filter pot) |
| CV_8 | C9 | Delay CV | (adds to delay pot) |
| B_8 | B8 | Warp Toggle | Doubles / Halves delay time|

## To do

Cleanup!

Add proper 1v/oct freq tracking and calibration

## License

This project is licensed under the MIT License. See LICENSE for details.

This project uses code from DaisySP, libDaisy, RtMidi, and RtAudio which are all also licensed under the MIT license. The relevant license documentation and source code for these can be found in lib/ as git submodules.
