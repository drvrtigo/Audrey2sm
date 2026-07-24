// main.cpp — Audrey II-style feedback synth on Daisy Patch Submodule

#include "daisy_patch_sm.h"
#include "daisysp.h"
#include "src/FeedbackSynthEngine.h"
#include "src/calibration/calibration_runtime.h"

using namespace daisy;
using namespace daisy::patch_sm;
using namespace daisysp;
using namespace infrasonic;

// Audio settings
static const auto   kSampleRate = SaiHandle::Config::SampleRate::SAI_48KHZ;
static const size_t kBlockSize  = 4;

// Hardware and DSP objects
static DaisyPatchSM              hw;
static FeedbackSynth::Engine     engine;
static Limiter                   limiter[2];
static Switch                    delay_sw;
static calib::CalibrationRuntime calib_rt(hw);

static float led_env = 0.0f;

// Pitch CV cleanup
static constexpr float kPitchCvDeadbandVolts = 0.015f; // ~15 mV deadband near 0V
static constexpr float kPitchCvOffsetVolts   = 0.000f; // manual trim if needed after testing

static inline float Clamp(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static inline float Map0To1(float norm, float min, float max)
{
    return min + Clamp(norm, 0.0f, 1.0f) * (max - min);
}

static inline float Map0To1Exp(float norm, float min, float max)
{
    norm = Clamp(norm, 0.0f, 1.0f);
    return min * powf(max / min, norm);
}

static inline float SmoothEnv(float in, float state, float coeff)
{
    return state + coeff * (in - state);
}

static inline float ApplyDeadband(float x, float deadband)
{
    if(fabsf(x) <= deadband)
        return 0.0f;

    if(x > 0.0f)
        return x - deadband;

    return x + deadband;
}

static inline float CleanPitchCvVolts(float volts)
{
    volts += kPitchCvOffsetVolts;
    return ApplyDeadband(volts, kPitchCvDeadbandVolts);
}

static void AudioCallback(AudioHandle::InputBuffer in,
                          AudioHandle::OutputBuffer out,
                          size_t size)
{
    hw.ProcessAnalogControls();
    delay_sw.Debounce();

    // Calibrated panel knobs: 0..1
    const float k_pitch = calib_rt.GetKnob01(CV_1);
    const float k_feed  = calib_rt.GetKnob01(CV_2);
    const float k_body  = calib_rt.GetKnob01(CV_3);
    const float k_space = calib_rt.GetKnob01(CV_4);

    // Calibrated external CVs
    float cv_pitch_v = calib_rt.GetCvVolts(CV_5);
    const float cv_feed_n  = calib_rt.GetCvNorm(CV_6);  // -1..1
    const float cv_body_n  = calib_rt.GetCvNorm(CV_7);  // -1..1
    const float cv_space_n = calib_rt.GetCvNorm(CV_8);  // -1..1

    // Clean pitch CV around 0V to reduce idle detune
    cv_pitch_v = CleanPitchCvVolts(cv_pitch_v);

    // Pitch: knob sets base MIDI note number, CV_5 adds 1V/oct offset
    const float base_note  = Map0To1(k_pitch, 16.0f, 72.0f);
    float       pitch_note = base_note + (12.0f * cv_pitch_v);
    pitch_note             = Clamp(pitch_note, 0.0f, 127.0f);
    engine.SetStringPitch(pitch_note);

    // Feedback gain in dBFS
    const float fb_norm = Clamp(k_feed + 0.25f * cv_feed_n, 0.0f, 1.0f);
    const float fb_db   = Map0To1(fb_norm, -30.0f, 12.0f);
    engine.SetFeedbackGain(fb_db);

    // Body / delay / tone
    const float body_norm = Clamp(k_body + 0.25f * cv_body_n, 0.0f, 1.0f);
    const float fb_delay  = Map0To1(body_norm, 0.001f, 0.1f);
    engine.SetFeedbackDelay(fb_delay);

    const float lpf_hz = Map0To1Exp(body_norm, 200.0f, 16000.0f);
    engine.SetFeedbackLPFCutoff(lpf_hz);

    const float hpf_hz = Map0To1Exp(body_norm, 10.0f, 3000.0f);
    engine.SetFeedbackHPFCutoff(hpf_hz);

    // Space / reverb / echo
    const float space_norm = Clamp(k_space + 0.25f * cv_space_n, 0.0f, 1.0f);
    engine.SetReverbMix(space_norm);
    engine.SetReverbFeedback(Map0To1(space_norm, 0.2f, 1.0f));
    engine.SetEchoDelaySendAmount(space_norm);

    float echo_time = Map0To1(space_norm, 0.05f, 2.0f);
    const float delay_scale = delay_sw.Pressed() ? 0.5f : 1.0f;
    echo_time *= delay_scale;
    engine.SetEchoDelayTime(echo_time);

    const float echo_fb = Clamp(fb_norm, 0.0f, 1.0f);
    engine.SetEchoDelayFeedback(echo_fb);

    engine.SetOutputLevel(0.5f);

    float peak = 0.0f;

    for(size_t i = 0; i < size; i++)
    {
        const float in_l = IN_L[i];

        float out_l = 0.0f;
        float out_r = 0.0f;

        engine.Process(in_l, out_l, out_r);

        OUT_L[i] = out_l;
        OUT_R[i] = out_r;

        const float mag = fmaxf(fabsf(out_l), fabsf(out_r));
        if(mag > peak)
            peak = mag;
    }

    limiter[0].ProcessBlock(OUT_L, size, 0.7f);
    limiter[1].ProcessBlock(OUT_R, size, 0.7f);

    led_env = SmoothEnv(peak, led_env, 0.1f);

    // patch.Init CV outs are 0–5V; using CV_OUT_2 here as an LED driver
    const float led_level = sqrtf(Clamp(led_env * 2.0f, 0.0f, 1.0f));
    hw.WriteCvOut(CV_OUT_2, led_level * 3.0f);
}

int main(void)
{
    hw.Init();
    hw.SetAudioSampleRate(kSampleRate);
    hw.SetAudioBlockSize(kBlockSize);

    calib_rt.Init();
    delay_sw.Init(hw.B8);

    engine.Init(hw.AudioSampleRate());

    for(auto& lim : limiter)
        lim.Init();

    hw.StartAudio(AudioCallback);

    while(1) {}
}