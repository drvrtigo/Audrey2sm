// main.cpp — Audrey II-style feedback synth on Daisy Patch Submodule

#include "daisy_patch_sm.h"
#include "daisysp.h"
#include "src/FeedbackSynthEngine.h"
#include "src/PitchCalibration.h"

using namespace daisy;
using namespace daisy::patch_sm;
using namespace daisysp;
using namespace infrasonic;

// Audio settings
static const auto   kSampleRate = SaiHandle::Config::SampleRate::SAI_48KHZ;
static const size_t kBlockSize  = 4;

// Hardware and DSP objects
static DaisyPatchSM         hw;
static FeedbackSynth::Engine engine;
static Limiter              limiter[2];
static Switch delay_sw;

static float led_env = 0.0f;

static inline float SmoothEnv(float in, float state, float coeff)
{
    return state + coeff * (in - state);
}

// Helper clamp
static inline float Clamp(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

// Map a 0–1 value to [min, max]
static inline float Map0To1(float norm, float min, float max)
{
    return min + Clamp(norm, 0.0f, 1.0f) * (max - min);
}

// Optional: exponential mapping for frequency-ish controls
static inline float Map0To1Exp(float norm, float min, float max)
{
    norm = Clamp(norm, 0.0f, 1.0f);
    return min * powf(max / min, norm);
}

// Audio callback
static void AudioCallback(AudioHandle::InputBuffer in,
                          AudioHandle::OutputBuffer out,
                          size_t size)
{
    // Update and filter Patch SM analog controls once per block
    hw.ProcessAnalogControls();
    delay_sw.Debounce();

    // Patch SM exposes 4 unipolar ADC inputs as ADC_9..ADC_12
    // Use these for the 4 Patch.Init panel knobs if that's how your carrier maps them.
    float k_pitch = hw.GetAdcValue(CV_1);
    float k_feed  = hw.GetAdcValue(CV_2);
    float k_body  = hw.GetAdcValue(CV_3);
    float k_space = hw.GetAdcValue(CV_4);

	float cv_pitch = hw.GetAdcValue(CV_5);
	float cv_feed  = hw.GetAdcValue(CV_6);
	float cv_body  = hw.GetAdcValue(CV_7);
    float cv_space = hw.GetAdcValue(CV_8);

    // Pitch: keep inverted feel
    //float freq_norm = 1.0f - k_pitch;
	float freq_norm = Clamp(k_pitch + 0.5f * cv_pitch, 0.0f, 1.0f);
    float note      = Map0To1(freq_norm, 16.0f, 72.0f);
    engine.SetStringPitch(note);

	// Calibrated pitch from knob + CV_5
	/*float note = PitchCalibration::GetCalibratedNote(hw, k_pitch);
	engine.SetStringPitch(note);*/

    // Feedback gain in dBFS
    //float fb_norm = 1.0f - k_feed;
	float fb_norm = Clamp(k_feed + 0.5f * cv_feed, 0.0f, 1.0f);
    float fb_db   = Map0To1(fb_norm, -30.0f, 12.0f); // changed from -60.0f to 12.0f
    engine.SetFeedbackGain(fb_db);

    // Body / delay / tone
    //float body_norm = 1.0f - k_body;
	float body_norm = Clamp(k_body + 0.5f * cv_body, 0.0f, 1.0f);
	float fb_delay  = Map0To1(body_norm, 0.001f, 0.1f);
    engine.SetFeedbackDelay(fb_delay);

    // LPF cutoff
    float lpf_norm = body_norm;
    float lpf_hz   = Map0To1Exp(lpf_norm, 200.0f, 16000.0f);
    engine.SetFeedbackLPFCutoff(lpf_hz);

    // HPF cutoff
    float hpf_norm = body_norm;
    float hpf_hz   = Map0To1Exp(hpf_norm, 10.0f, 3000.0f);
    engine.SetFeedbackHPFCutoff(hpf_hz);

    // Space / reverb / echo
    //float space_norm = 1.0f - k_space;
	float space_norm = Clamp(k_space + 0.5f * cv_space, 0.0f, 1.0f);
    engine.SetReverbMix(space_norm);
    engine.SetReverbFeedback(Map0To1(space_norm, 0.2f, 1.0f));
    engine.SetEchoDelaySendAmount(space_norm);

    // Base echo time from knob/CV
    float echo_time = Map0To1(space_norm, 0.05f, 2.0f);

    // --- B8 switch: delay time scaler ---
    // Example: switch up = normal, switch down = half-time (or double)
    float delay_scale = delay_sw.Pressed() ? 0.5f : 1.0f;
    echo_time *= delay_scale;

    engine.SetEchoDelayTime(echo_time);

    float echo_fb = Clamp(fb_norm, 0.0f, 1.0f);
    engine.SetEchoDelayFeedback(echo_fb);

    engine.SetOutputLevel(0.5f);

    float peak = 0.0f;

    for(size_t i = 0; i < size; i++)
    {
        float in_l = IN_L[i];

        float out_l = 0.0f;
        float out_r = 0.0f;

        engine.Process(in_l, out_l, out_r);

        OUT_L[i] = out_l;
        OUT_R[i] = out_r;

        float mag = fmaxf(fabsf(out_l), fabsf(out_r));
        if(mag > peak)
            peak = mag;
    }

    limiter[0].ProcessBlock(OUT_L, size, 0.7f);
    limiter[1].ProcessBlock(OUT_R, size, 0.7f);

    // Smooth the envelope a bit
    led_env = SmoothEnv(peak, led_env, 0.1f);

    // Map envelope (0–~1) to 0–5 V for CV_OUT_2 LED brightness
    float led_level = sqrtf(Clamp(led_env * 2.0f, 0.0f, 1.0f));
    hw.WriteCvOut(CV_OUT_2, led_level * 3.0f);
}

int main(void)
{
    hw.Init();
    hw.SetAudioSampleRate(kSampleRate);
    hw.SetAudioBlockSize(kBlockSize);

	PitchCalibration::Init(hw);
    PitchCalibration::MaybeRunCalibration(hw);

    // Init B8 as a toggle switch
    delay_sw.Init(hw.B8);


    engine.Init(hw.AudioSampleRate());

    for(auto& lim : limiter)
        lim.Init();

    hw.StartAudio(AudioCallback);

    while(1) {}
}