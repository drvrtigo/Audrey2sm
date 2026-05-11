#include "PitchCalibration.h"
#include <daisysp.h>
#include <util/VoctCalibration.h>

using namespace daisy;
using namespace daisy::patch_sm;

namespace
{
    VoctCalibration pitch_cal;

    // These will be filled by calibration; start with safe defaults.
    float g_pitch_scale  = 60.0f;
    float g_pitch_offset = 0.0f;

    // Use Patch SM pin B7 as the calibration button
    Switch calib_sw;

    void ConfigureCalibration()
    {
        pitch_cal.SetData(g_pitch_scale, g_pitch_offset);
    }

    float MeasureCv5Averaged(DaisyPatchSM& hw, int samples)
    {
        float acc = 0.0f;
        for(int i = 0; i < samples; ++i)
        {
            hw.ProcessAnalogControls();
            acc += hw.GetAdcValue(CV_5);
            System::Delay(1); // 1 ms per sample
        }
        return acc / samples;
    }

    void RunPitchCalibration(DaisyPatchSM& hw)
    {
        const int kSamples = 1024;

        // 1. User patches 1 V into CV_5
        float adc_at_1v = MeasureCv5Averaged(hw, kSamples);

        // 2. User patches 3 V into CV_5
        float adc_at_3v = MeasureCv5Averaged(hw, kSamples);

        // 3. Compute scale/offset and configure
        pitch_cal.Record(adc_at_1v, adc_at_3v);

        float scale  = 0.0f;
        float offset = 0.0f;
        pitch_cal.GetData(scale, offset);

        g_pitch_scale  = scale;
        g_pitch_offset = offset;

        ConfigureCalibration();

        // Optional: log scale/offset via UART/USB here.
    }

    inline float Clamp(float x, float lo, float hi)
    {
        return x < lo ? lo : (x > hi ? hi : x);
    }

} // anonymous namespace

namespace PitchCalibration
{
    void Init(DaisyPatchSM& hw)
    {
        // Init B7 as a momentary button with pull-up
        calib_sw.Init(hw.B7);

        ConfigureCalibration();
    }

    void MaybeRunCalibration(DaisyPatchSM& hw)
    {
        // Sample once at boot
        calib_sw.Debounce();
        bool do_calibration = calib_sw.Pressed();

        if(do_calibration)
        {
            RunPitchCalibration(hw);
        }
        else
        {
            ConfigureCalibration();
        }
    }

    float GetCalibratedNote(DaisyPatchSM& hw, float k_pitch_knob)
    {
        // Base pitch from knob (16–72 as before)
        float knob_note = 16.0f + Clamp(k_pitch_knob, 0.0f, 1.0f) * (72.0f - 16.0f);

        // Calibrated note from CV_5
        float cv_raw  = hw.GetAdcValue(CV_5);
        float cv_note = pitch_cal.ProcessInput(cv_raw);

        // Treat knob as transpose around 40.0
        float note = cv_note + (knob_note - 40.0f);
        return Clamp(note, 0.0f, 127.0f);
    }
} // namespace PitchCalibration