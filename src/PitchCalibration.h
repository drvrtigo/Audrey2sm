#pragma once

#include "daisy_patch_sm.h"

namespace PitchCalibration
{
    // Call once after hw.Init()
    void Init(daisy::patch_sm::DaisyPatchSM& hw);

    // Call once at boot after Init():
    //   - reads B7 as a momentary button
    //   - if held, runs the 1 V / 3 V calibration on CV_5
    void MaybeRunCalibration(daisy::patch_sm::DaisyPatchSM& hw);

    // Call from the audio callback:
    //   - k_pitch_knob is 0..1 from your pitch knob
    //   - returns calibrated MIDI-like note value
    float GetCalibratedNote(daisy::patch_sm::DaisyPatchSM& hw,
                            float k_pitch_knob);
} // namespace PitchCalibration