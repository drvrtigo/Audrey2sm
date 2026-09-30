// Audrey II-style feedback synth on Daisy Patch Submodule
#include "daisy_patch_sm.h"
#include "daisysp.h"
#include "src/FeedbackSynthEngine.h"
#include "src/calibration/calibration_runtime.h"

using namespace daisy;
using namespace daisy::patch_sm;
using namespace daisysp;
using namespace infrasonic;

static const auto kSampleRate = SaiHandle::Config::SampleRate::SAI_48KHZ;
static const size_t kBlockSize = 4;
#define CAL_DEBUG_LED 1
static constexpr float kPitchBaseHz = 130.81278f;
static constexpr float kPitchKnobMinOct = -1.0f;
static constexpr float kPitchKnobMaxOct = 1.0f;

static DaisyPatchSM hw;
static FeedbackSynth::Engine engine;
static Limiter limiter[2];
static Switch delay_button;
static Switch layer_toggle;
static calib::CalibrationRuntime calib_rt(hw);
static float led_env = 0.0f;

struct ControlState
{
    float frequency = 0.5f;
    float body = 0.5f;
    float lpf_cutoff = 0.5f;
    float hpf_cutoff = 0.5f;
    float drive = 0.4f;
    float feedback = 0.5f;
    float reverb_mix = 0.5f;
    float reverb_feedback = 0.5f;
    float echo_time = 0.5f;
    float echo_feedback = 0.5f;
    float echo_send = 0.5f;
    float echo_character = 0.5f;
};
static ControlState controls;

enum class ControlLayer { Voice, LoopShaping, Delay };
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
    if(delay_button.Pressed()) return ControlLayer::Delay;
    return layer_toggle.Pressed() ? ControlLayer::LoopShaping : ControlLayer::Voice;
}
#if CAL_DEBUG_LED
static inline void SetDebugLed(float norm)
{
    hw.WriteCvOut(CV_OUT_2, fclamp(norm, 0.0f, 1.0f) * 5.0f);
}
static void BlinkDebugLed(int count, int on_ms = 140, int off_ms = 140, int gap_ms = 500)
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

static void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
    hw.ProcessAnalogControls();
    delay_button.Debounce();
    layer_toggle.Debounce();

    const float p1 = calib_rt.GetKnob01(CV_1);
    const float p2 = calib_rt.GetKnob01(CV_2);
    const float p3 = calib_rt.GetKnob01(CV_3);
    const float p4 = calib_rt.GetKnob01(CV_4);

    switch(GetActiveLayer())
    {
        case ControlLayer::Voice:
            controls.frequency = p1;
            controls.body = p2;
            controls.lpf_cutoff = p3;
            controls.hpf_cutoff = p4;
            break;
        case ControlLayer::LoopShaping:
            controls.drive = p1;
            controls.feedback = p2;
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

    // CV is applied after page selection so modulation never freezes on another page.
    const float body_norm = Clamp(controls.body + 0.25f * calib_rt.GetCvNorm(CV_6), 0.0f, 1.0f);
    const float lp_norm = Clamp(controls.lpf_cutoff + 0.25f * calib_rt.GetCvNorm(CV_7), 0.0f, 1.0f);
    const float hp_norm = Clamp(controls.hpf_cutoff + 0.25f * calib_rt.GetCvNorm(CV_8), 0.0f, 1.0f);

    const float coarse_oct = Map0To1(controls.frequency, kPitchKnobMinOct, kPitchKnobMaxOct);
    const float pitch_hz = calib_rt.GetPitchHz(CV_5, kPitchBaseHz, coarse_oct);
    engine.SetStringPitchHz(pitch_hz);

    engine.SetFeedbackDelay(Map0To1(body_norm, 0.001f, 0.1f));
    engine.SetFeedbackLPFCutoff(Map0To1Exp(lp_norm, 200.0f, 16000.0f));
    engine.SetFeedbackHPFCutoff(Map0To1Exp(hp_norm, 10.0f, 3000.0f));
    engine.SetDrive(controls.drive);
    engine.SetFeedbackGain(Map0To1(controls.feedback, -30.0f, 12.0f));
    engine.SetReverbMix(controls.reverb_mix);
    engine.SetReverbFeedback(Map0To1(controls.reverb_feedback, 0.2f, 0.98f));
    engine.SetEchoDelayTime(Map0To1(controls.echo_time, 0.05f, 2.0f));
    engine.SetEchoDelayFeedback(Clamp(controls.echo_feedback, 0.0f, 1.0f));
    engine.SetEchoDelaySendAmount(Clamp(controls.echo_send, 0.0f, 1.0f));
    engine.SetEchoDelayLagTime(Map0To1(controls.echo_character, 0.005f, 0.5f));
    engine.SetOutputLevel(0.5f);

    float peak = 0.0f;
    for(size_t i = 0; i < size; ++i)
    {
        const float in_l = IN_L[i];
        float out_l = 0.0f;
        float out_r = 0.0f;
        engine.Process(in_l, out_l, out_r);
        OUT_L[i] = out_l;
        OUT_R[i] = out_r;
        const float mag = fmaxf(fabsf(out_l), fabsf(out_r));
        if(mag > peak) peak = mag;
    }
    limiter[0].ProcessBlock(OUT_L, size, 0.7f);
    limiter[1].ProcessBlock(OUT_R, size, 0.7f);
    led_env = SmoothEnv(peak, led_env, 0.1f);
    const float led_level = sqrtf(Clamp(led_env * 2.0f, 0.0f, 1.0f));
    hw.WriteCvOut(CV_OUT_2, led_level * 3.0f);
}

int main(void)
{
    hw.Init();
    hw.SetAudioSampleRate(kSampleRate);
    hw.SetAudioBlockSize(kBlockSize);
    calib_rt.Init();
#if CAL_DEBUG_LED
    const bool cal_valid = calib_rt.IsValid();
    const auto cal_state = calib_rt.State();
    if(!cal_valid) BlinkDebugLed(1, 350, 350, 700);
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
    delay_button.Init(hw.B7);
    layer_toggle.Init(hw.B8);
    engine.Init(hw.AudioSampleRate());
    for(auto& lim : limiter) lim.Init();
    hw.StartAudio(AudioCallback);
    while(1) {}
}
