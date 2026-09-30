# Audrey2sm — Audrey II for Daisy patch.Init()

A Daisy Patch Submodule / patch.Init() adaptation of [Synthux Academy's Audrey II](https://github.com/Synthux-Academy/Audrey-II) feedback synthesizer. A continuously noise-excited Karplus–Strong resonator passes through overdrive, low-pass and high-pass filters, and reverb; its post-reverb signal feeds a longer feedback delay, while a separate echo follows the feedback tap. The left audio input can be blended into the resonator excitation. Audio is stereo at the output.

This README describes the development branch `feature/three-layer-controls`.

## Panel controls

The four panel pots (`CV_1`–`CV_4`) are calibrated unipolar 0–1 controls. B8 selects Voice or Loop Shaping. Holding the B7 momentary button selects Echo Delay from **either** B8 position; release B7 to return to the B8-selected layer.

| Pot | B8 down — Voice | B8 up — Loop Shaping | Hold B7 — Echo Delay |
| --- | --- | --- | --- |
| P1 (`CV_1`, C5) | Frequency: coarse pitch ±1 octave around C3, plus calibrated `CV_5` 1 V/oct | Excitation blend: internal noise → left audio input | Echo time: 0.05–5 s |
| P2 (`CV_2`, C4) | Body: outer-feedback delay, 0.001–0.1 s | Master feedback: −60 to +12 dB | Echo feedback: 0–1.5 (can exceed unity) |
| P3 (`CV_3`, C3) | Low-pass cutoff: 100 Hz–18 kHz | Reverb mix: dry to wet | Echo send: 0–1 |
| P4 (`CV_4`, C2) | High-pass cutoff: 10 Hz–4 kHz | Reverb decay/feedback: 0.2–1.0 | Echo character: delay-time lag, 0.005–0.5 s |

Overdrive is fixed at `0.6` rather than controlled by a pot. There is no Space/FX macro or B8 Warp-delay toggle in this branch. Echo Character changes the rate at which echo delay time slews; it is not a separate audio mode. Echo Send adjusts what enters the echo, not an independently implemented final wet/dry crossfade.

### Pickup between layers

Each layer retains four independent pot values. On a layer change, moving a pot does not change its new parameter until the calibrated physical value crosses or comes within 0.015 of its stored position. CV modulation remains live while a pot waits for pickup. The layer selected at startup follows the physical pots immediately. On pickup, the `CV_OUT_2` panel LED gives three short off/on flashes, then resumes metering; simultaneous pickups restart the flash pattern.

## CV and audio I/O

| Jack/output | Patch SM ID | Behavior |
| --- | --- | --- |
| Pitch CV input | `CV_5` (C6) | Dedicated calibrated 1 V/oct input. The P1 Frequency value supplies the coarse octave offset. |
| Body CV input | `CV_6` (C7) | Bipolar modulation added to Body at 0.25 of the calibrated normalized CV range. |
| LP CV input | `CV_7` (C8) | Bipolar modulation added to LP cutoff at the same depth. |
| HP CV input | `CV_8` (C9) | Bipolar modulation added to HP cutoff at the same depth. |
| Audio input | `IN_L` | Left input excites both resonator channels when Excitation Blend is raised. `IN_R` is not used. |
| Audio outputs | `OUT_L`, `OUT_R` | Stereo synth/echo output; an output limiter runs after the engine. |
| CV output | `CV_OUT_1` (C10) | 0–5 V level-following CV, scaled from the normal LED meter signal. It measures the engine's pre-limiter block peak, not calibrated RMS or a separate slow envelope follower. Pickup flashes do not affect this CV. |
| Panel LED | `CV_OUT_2` (C1) | Normally meters audio activity; three pickup flashes temporarily override it. The normal meter spans 0–3 V and flashes use 0/5 V. |

The internal noise source is set to −90 dBFS. B8-up P1 blends its gain from 100% down to 10% while increasing the `IN_L` excitation gain from zero to 25%. The delayed feedback return is not crossfaded away. Because the noise and external audio have different levels, the knob is a source blend rather than a loudness-matched crossfade.

`GATE_IN_1`, `GATE_IN_2`, `GATE_OUT_1`, `GATE_OUT_2`, and `IN_R` are not currently assigned firmware functions. Do not confuse the four CV input jacks with the four panel pots.

## Calibration and safety

The firmware reads saved board-specific calibration for all pots/CV inputs; `CV_5` pitch uses that calibration through `CalibrationRuntime::GetPitchHz()`. At boot the LED blinks once for unknown/invalid calibration, twice for factory calibration, or three times for user calibration. The stored calibration format is version 4 in this development branch; this README does not describe a procedure for creating new calibration data.

Master feedback can reach +12 dB, echo feedback can reach 1.5, and reverb feedback can reach 1.0. Begin testing with monitoring levels low, particularly with external audio excitation. The final limiter does not undo distortion already created inside the feedback path.

## License

This project is licensed under the MIT License; see `LICENSE` where available. It builds on the original Audrey II codebase and libDaisy/DaisySP; consult their repositories for their respective license terms.
