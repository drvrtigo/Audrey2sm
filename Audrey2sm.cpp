// Audrey II-style feedback synth on Daisy Patch Submodule
#include "daisy_patch_sm.h"
#include "daisysp.h"
#include "src/FeedbackSynthEngine.h"
#include "src/calibration/calibration_runtime.h"
#include <cmath>

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
static constexpr float kPickupTolerance = 0.015f;
static constexpr uint32_t kPickupOffBlocks = 720;   // 60 ms
static constexpr uint32_t kPickupOnBlocks  = 1080;  // 90 ms
static constexpr uint32_t kPickupCycles    = 3;
static constexpr uint32_t kPickupTotalBlocks =
    kPickupCycles * (kPickupOffBlocks + kPickupOnBlocks);

static DaisyPatchSM hw;
static FeedbackSynth::Engine engine;
static Limiter limiter[2];
static Switch delay_button;
static Switch layer_toggle;
static calib::CalibrationRuntime calib_rt(hw);
static float led_env = 0.0f;
static uint32_t pickup_led_blocks = 0;

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

enum class ControlLayer
{
    Voice,
    LoopShaping,
    Delay
};
static ControlLayer last_layer = ControlLayer::Voice;
static bool pickup_initialized = false;
static bool picked_up[4] = {};
static float previous_pot[4] = {};

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
static inline float Tension(float norm, float factor)
{
    norm = Clamp(norm, 0.0f, 1.0f);
    if (factor == 0.0f)
        return norm;
    return expm1f(norm * factor) / expm1f(factor);
}
static inline ControlLayer GetActiveLayer()
{
    if (delay_button.Pressed())
        return ControlLayer::Delay;
    return layer_toggle.Pressed() ? ControlLayer::LoopShaping : ControlLayer::Voice;
}
static void GetLayerTargets(ControlLayer layer, float *targets[4])
{
    switch (layer)
    {
    case ControlLayer::Voice:
        targets[0] = &controls.frequency;
        targets[1] = &controls.body;
        targets[2] = &controls.lpf_cutoff;
        targets[3] = &controls.hpf_cutoff;
        break;
    case ControlLayer::LoopShaping:
        targets[0] = &controls.drive;
        targets[1] = &controls.feedback;
        targets[2] = &controls.reverb_mix;
        targets[3] = &controls.reverb_feedback;
        break;
    case ControlLayer::Delay:
        targets[0] = &controls.echo_time;
        targets[1] = &controls.echo_feedback;
        targets[2] = &controls.echo_send;
        targets[3] = &controls.echo_character;
        break;
    }
}
static void UpdatePots(ControlLayer layer, const float pot[4])
{
    float *targets[4] = {};
    GetLayerTargets(layer, targets);
    if (!pickup_initialized)
    {
        for (int i = 0; i < 4; ++i)
        {
            *targets[i] = pot[i];
            picked_up[i] = true;
            previous_pot[i] = pot[i];
        }
        last_layer = layer;
        pickup_initialized = true;
        return;
    }
    if (layer != last_layer)
    {
        for (int i = 0; i < 4; ++i)
        {
            picked_up[i] = false;
            previous_pot[i] = pot[i];
        }
        last_layer = layer;
        return;
    }
    for (int i = 0; i < 4; ++i)
    {
        const float target = *targets[i];
        const bool near = fabsf(pot[i] - target) <= kPickupTolerance;
        const bool crossed = (previous_pot[i] <= target && pot[i] >= target) || (previous_pot[i] >= target && pot[i] <= target);
        if (!picked_up[i] && (near || crossed))
        {
            picked_up[i] = true;
            pickup_led_blocks = kPickupTotalBlocks;
        }
        if (picked_up[i])
            *targets[i] = pot[i];
        previous_pot[i] = pot[i];
    }
}
#if CAL_DEBUG_LED
static inline void SetDebugLed(float norm)
{
    hw.WriteCvOut(CV_OUT_2, fclamp(norm, 0.0f, 1.0f) * 5.0f);
}
static void BlinkDebugLed(int count, int on_ms = 140, int off_ms = 140, int gap_ms = 500)
{
    for (int i = 0; i < count; ++i)
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
    const float pot[4] = {
        calib_rt.GetKnob01(CV_1), calib_rt.GetKnob01(CV_2),
        calib_rt.GetKnob01(CV_3), calib_rt.GetKnob01(CV_4)};
    UpdatePots(GetActiveLayer(), pot);

    const float body_norm = Clamp(controls.body + 0.25f * calib_rt.GetCvNorm(CV_6), 0.0f, 1.0f);
    const float lp_norm = Clamp(controls.lpf_cutoff + 0.25f * calib_rt.GetCvNorm(CV_7), 0.0f, 1.0f);
    const float hp_norm = Clamp(controls.hpf_cutoff + 0.25f * calib_rt.GetCvNorm(CV_8), 0.0f, 1.0f);

    const float coarse_oct = Map0To1(controls.frequency, kPitchKnobMinOct, kPitchKnobMaxOct);
    const float pitch_hz = calib_rt.GetPitchHz(CV_5, kPitchBaseHz, coarse_oct);
    engine.SetStringPitchHz(pitch_hz);

    engine.SetFeedbackDelay(daisysp::fmap(body_norm, 0.001f, 0.1f, daisysp::Mapping::EXP));
    engine.SetFeedbackLPFCutoff(Map0To1Exp(lp_norm, 100.0f, 18000.0f));
    engine.SetFeedbackHPFCutoff(Map0To1Exp(hp_norm, 10.0f, 4000.0f));
    engine.SetDrive(Map0To1(controls.drive, 0.4f, 0.96f));
    engine.SetFeedbackGain(Map0To1(controls.feedback, -60.0f, 12.0f));
    engine.SetReverbMix(controls.reverb_mix);
    engine.SetReverbFeedback(Map0To1(Tension(controls.reverb_feedback, -3.0f), 0.2f, 1.0f));
    engine.SetEchoDelayTime(daisysp::fmap(controls.echo_time, 0.05f, 5.0f, daisysp::Mapping::EXP));
    engine.SetEchoDelayFeedback(Map0To1(controls.echo_feedback, 0.0f, 1.5f));
    engine.SetEchoDelaySendAmount(daisysp::fmap(controls.echo_send, 0.0f, 1.0f, daisysp::Mapping::EXP));
    engine.SetEchoDelayLagTime(Map0To1(controls.echo_character, 0.005f, 0.5f));
    engine.SetOutputLevel(0.5f);

    float peak = 0.0f;
    for (size_t i = 0; i < size; ++i)
    {
        const float in_l = IN_L[i];
        float out_l = 0.0f;
        float out_r = 0.0f;
        engine.Process(in_l, out_l, out_r);
        OUT_L[i] = out_l;
        OUT_R[i] = out_r;
        const float mag = fmaxf(fabsf(out_l), fabsf(out_r));
        if (mag > peak)
            peak = mag;
    }
    limiter[0].ProcessBlock(OUT_L, size, 0.7f);
    limiter[1].ProcessBlock(OUT_R, size, 0.7f);
    led_env = SmoothEnv(peak, led_env, 0.1f);
    const float led_level = sqrtf(Clamp(led_env * 2.0f, 0.0f, 1.0f));
    if (pickup_led_blocks > 0)
    {
        const uint32_t elapsed = kPickupTotalBlocks - pickup_led_blocks;
        const uint32_t cycle = kPickupOffBlocks + kPickupOnBlocks;
        const bool led_on = (elapsed % cycle) >= kPickupOffBlocks;

        hw.WriteCvOut(CV_OUT_2, led_on ? 5.0f : 0.0f);
        --pickup_led_blocks;
    }
    else
    {
        hw.WriteCvOut(CV_OUT_2, led_level * 3.0f);
    }
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
    if (!cal_valid)
        BlinkDebugLed(1, 350, 350, 700);
    else
    {
        switch (cal_state)
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
    for (auto &lim : limiter)
        lim.Init();
    hw.StartAudio(AudioCallback);
    while (1)
    {
    }
}
