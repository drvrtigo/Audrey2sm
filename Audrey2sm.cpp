// main.cpp — Audrey II-style feedback synth on Daisy Patch Submodule
// Debug build: boot-time LED blink indicates calibration state.
//
// Blink codes:
// 1 blink = invalid / unknown calibration
// 2 blinks = valid factory calibration
// 3 blinks = valid user calibration

#include "daisy_patch_sm.h"
#include "daisysp.h"
#include "src/FeedbackSynthEngine.h"
#include "src/calibration/calibration_runtime.h"

using namespace daisy;
using namespace daisy::patch_sm;
using namespace daisysp;
using namespace infrasonic;

// ------------------------------------------------------------
// Config
// ------------------------------------------------------------

static const auto   kSampleRate = SaiHandle::Config::SampleRate::SAI_48KHZ;
static const size_t kBlockSize  = 4;

#define CAL_DEBUG_LED 1

// 0V reference pitch = C3
static constexpr float kPitchBaseHz = 130.81278f;

// CV_1 coarse pitch range in octaves, centered at 0 when knob = 0.5
static constexpr float kPitchKnobMinOct = -2.0f;
static constexpr float kPitchKnobMaxOct =  2.0f;

// ------------------------------------------------------------
// Globals
// ------------------------------------------------------------

static DaisyPatchSM              hw;
static FeedbackSynth::Engine     engine;
static Limiter                   limiter[2];
static Switch                    delay_button;
static Switch                    layer_toggle;
static calib::CalibrationRuntime calib_rt(hw);

static float led_env = 0.0f;

struct ControlState
{
    // Layer 1: Voice
    float frequency = 0.5f;
    float body = 0.5f;
    float feedback = 0.5f;
    float space = 0.5f;

    // Layer 2: Loop shaping
    float lpf_cutoff = 0.5f;
    float hpf_cutoff = 0.5f;
    float reverb_mix = 0.5f;
    float reverb_feedback = 0.5f;

    // Layer 3: Echo delay
    float echo_time = 0.5f;
    float echo_feedback = 0.5f;
    float echo_send = 0.5f;
    float echo_character = 0.5f;
};

static ControlState controls;

enum class ControlLayer
{
    Voice,
    LoopShaping,
    Delay,
};

// ------------------------------------------------------------
// Helpers
// ------------------------------------------------------------

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

static inline ControlLayer GetActiveLayer()
{
    if(delay_button.Pressed())
        return ControlLayer::Delay;

    return layer_toggle.Pressed() ? ControlLayer::LoopShaping
                                  : ControlLayer::Voice;
}

#if CAL_DEBUG_LED
static inline void SetDebugLed(float norm)
{
    hw.WriteCvOut(CV_OUT_2, fclamp(norm, 0.0f, 1.0f) * 5.0f);
}

static void BlinkDebugLed(int count,
                          int on_ms  = 140,
                          int off_ms = 140,
                          int gap_ms = 500)
{
    for(int i = 0; i < count; ++i)
    {
        SetDebugLed(1.0f);
        System::Delay(on_ms);
        SetDebugLed(0.0f);
        System::Delay(off_ms);
    }

    System::Delay(gap_ms);
}
#endif

// ------------------------------------------------------------
// Audio
// ------------------------------------------------------------

static void AudioCallback(AudioHandle::InputBuffer in,
                          AudioHandle::OutputBuffer out,
                          size_t size)
{
    hw.ProcessAnalogControls();
    delay_button.Debounce();
    layer_toggle.Debounce();

    // Calibrated panel knobs: 0..1
    const float p1 = calib_rt.GetKnob01(CV_1);
    const float p2 = calib_rt.GetKnob01(CV_2);
    const float p3 = calib_rt.GetKnob01(CV_3);
    const float p4 = calib_rt.GetKnob01(CV_4);

    // Fixed calibrated external CV routing: Body, Feedback, Space.
    const float cv_body  = calib_rt.GetCvNorm(CV_6); // -1..1
    const float cv_feed  = calib_rt.GetCvNorm(CV_7); // -1..1
    const float cv_space = calib_rt.GetCvNorm(CV_8); // -1..1

    switch(GetActiveLayer())
    {
        case ControlLayer::Voice:
            controls.frequency = p1;
            controls.body = Clamp(p2 + 0.25f * cv_body, 0.0f, 1.0f);
            controls.feedback = Clamp(p3 + 0.25f * cv_feed, 0.0f, 1.0f);
            controls.space = Clamp(p4 + 0.25f * cv_space, 0.0f, 1.0f);
            break;

        case ControlLayer::LoopShaping:
            controls.lpf_cutoff = p1;
            controls.hpf_cutoff = p2;
            controls.reverb_mix = p3;
            controls.reverb_feedback = p4;
            break;

        case ControlLayer::Delay:
            controls.echo_time = p1;
            controls.echo_feedback = p2;
            controls.echo_send = p3;
            controls.echo_character = p4;
            break;
    }

    // --- Pitch ---
    // CV_1: coarse octave offset around C3
    // CV_5: calibrated 1V/oct pitch input
    const float coarse_oct = Map0To1(controls.frequency,
                                     kPitchKnobMinOct,
                                     kPitchKnobMaxOct);
    const float pitch_hz = calib_rt.GetPitchHz(CV_5,
                                                kPitchBaseHz,
                                                coarse_oct);
    engine.SetStringPitchHz(pitch_hz);

    // --- Main feedback / body ---
    const float fb_db = Map0To1(controls.feedback, -30.0f, 12.0f);
    engine.SetFeedbackGain(fb_db);

    const float fb_delay = Map0To1(controls.body, 0.001f, 0.1f);
    engine.SetFeedbackDelay(fb_delay);

    // --- Loop shaping ---
    const float lpf_hz = Map0To1Exp(controls.lpf_cutoff, 200.0f, 16000.0f);
    const float hpf_hz = Map0To1Exp(controls.hpf_cutoff, 10.0f, 3000.0f);
    engine.SetFeedbackLPFCutoff(lpf_hz);
    engine.SetFeedbackHPFCutoff(hpf_hz);

    engine.SetReverbMix(controls.reverb_mix);
    engine.SetReverbFeedback(Map0To1(controls.reverb_feedback, 0.2f, 0.98f));

    // --- Echo delay ---
    // Space remains a macro for the existing normal-state signal path, while
    // the held B7 delay layer provides direct access to each echo parameter.
    const float echo_time = Map0To1(controls.echo_time, 0.05f, 2.0f);
    const float echo_fb = Clamp(controls.echo_feedback, 0.0f, 1.0f);
    const float echo_send = Clamp(controls.echo_send, 0.0f, 1.0f);
    const float echo_lag = Map0To1(controls.echo_character, 0.005f, 0.5f);

    engine.SetEchoDelayTime(echo_time);
    engine.SetEchoDelayFeedback(echo_fb);
    engine.SetEchoDelaySendAmount(echo_send);
    engine.SetEchoDelayLagTime(echo_lag);

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

#if !CAL_DEBUG_LED
    const float led_level = sqrtf(Clamp(led_env * 2.0f, 0.0f, 1.0f));
    hw.WriteCvOut(CV_OUT_2, led_level * 3.0f);
#else
    // In debug build, keep the normal runtime LED meter enabled after boot.
    const float led_level = sqrtf(Clamp(led_env * 2.0f, 0.0f, 1.0f));
    hw.WriteCvOut(CV_OUT_2, led_level * 3.0f);
#endif
}

// ------------------------------------------------------------
// Main
// ------------------------------------------------------------

int main(void)
{
    hw.Init();
    hw.SetAudioSampleRate(kSampleRate);
    hw.SetAudioBlockSize(kBlockSize);

    calib_rt.Init();

#if CAL_DEBUG_LED
    const bool cal_valid = calib_rt.IsValid();
    const bool cal_user  = calib_rt.IsUserCalibration();
    const auto cal_state = calib_rt.State();

    (void)cal_user; // cal_user is redundant with cal_state, but useful while debugging

    if(!cal_valid)
    {
        // Invalid or unreadable calibration
        BlinkDebugLed(1, 350, 350, 700);
    }
    else
    {
        switch(cal_state)
        {
            case daisy::PersistentStorage<calib::CalibrationData>::State::USER:
                BlinkDebugLed(3, 90, 90, 700);
                break;

            case daisy::PersistentStorage<calib::CalibrationData>::State::FACTORY:
                BlinkDebugLed(2, 180, 180, 700);
                break;

            case daisy::PersistentStorage<calib::CalibrationData>::State::UNKNOWN:
            default:
                BlinkDebugLed(1, 350, 350, 700);
                break;
        }
    }

    SetDebugLed(0.0f);
#endif

    // B7 is the global held delay layer; B8 selects Voice/Loop Shaping.
    delay_button.Init(hw.B7);
    layer_toggle.Init(hw.B8);

    engine.Init(hw.AudioSampleRate());

    for(auto& lim : limiter)
        lim.Init();

    hw.StartAudio(AudioCallback);

    while(1) {}
}
