#include "FeedbackSynthEngine.h"
#include "DSPUtils.h"
#include "memory/sdram_alloc.h"
using namespace infrasonic;
using namespace infrasonic::FeedbackSynth;
using namespace daisysp;
void Engine::Init(const float sample_rate) {
  using ED = EchoDelay<kMaxEchoDelaySamp>;
  echo_delay_[0] = EchoDelayPtr(SDRAM::allocate<ED>());
  echo_delay_[1] = EchoDelayPtr(SDRAM::allocate<ED>());
  verb_ = VerbPtr(SDRAM::allocate<ReverbSc>());
  sample_rate_ = sample_rate;
  fb_delay_smooth_coef_ = onepole_coef(0.2f, sample_rate);
  noise_.Init();
  noise_.SetAmp(dbfs2lin(-90.0f));
  for (unsigned int i = 0; i < 2; i++) {
    strings_[i].Init(sample_rate);
    strings_[i].SetBrightness(0.98f);
    strings_[i].SetFreq(mtof(40.0f));
    strings_[i].SetDamping(0.4f);
    fb_delayline_[i].Init();
    echo_delay_[i]->Init(sample_rate);
    echo_delay_[i]->SetDelayTime(5.0f, true);
    echo_delay_[i]->SetFeedback(0.5f);
    echo_delay_[i]->SetLagTime(0.5f);
    overdrive_[i].Init();
    overdrive_[i].SetDrive(0.4);
  }
  verb_->Init(sample_rate);
  verb_->SetFeedback(0.85f);
  verb_->SetLpFreq(12000.0f);
  fb_lpf_.Init(sample_rate);
  fb_lpf_.SetQ(0.9f);
  fb_hpf_.Init(sample_rate);
  fb_hpf_.SetQ(0.9f);
  fb_hpf_.SetCutoff(60.f);
}
void Engine::SetStringPitch(const float nn) {
  const auto freq = mtof(nn);
  strings_[0].SetFreq(freq);
  strings_[1].SetFreq(freq);
}
void Engine::SetStringPitchHz(const float hz) {
  const float safe_hz = fmaxf(hz, 10.0f);
  strings_[0].SetFreq(safe_hz);
  strings_[1].SetFreq(safe_hz);
}
void Engine::SetFeedbackGain(const float gain_db) { fb_gain_ = dbfs2lin(gain_db); }
void Engine::SetFeedbackDelay(const float delay_s) {
  fb_delay_samp_target_ = DSY_CLAMP(delay_s * sample_rate_, 1.0f, static_cast<float>(kMaxFeedbackDelaySamp - 1));
}
void Engine::SetFeedbackLPFCutoff(const float cutoff_hz) { fb_lpf_.SetCutoff(cutoff_hz); }
void Engine::SetFeedbackHPFCutoff(const float cutoff_hz) { fb_hpf_.SetCutoff(cutoff_hz); }
void Engine::SetDrive(const float drive) {
  const float safe_drive = fclamp(drive, 0.0f, 1.0f);
  overdrive_[0].SetDrive(safe_drive);
  overdrive_[1].SetDrive(safe_drive);
}
void Engine::SetExcitationBlend(float blend) {
  excitation_blend_ = fclamp(blend, 0.0f, 1.0f);
}
void Engine::SetEchoDelayTime(const float echo_time) {
  echo_delay_[0]->SetDelayTime(echo_time);
  echo_delay_[1]->SetDelayTime(echo_time);
}
void Engine::SetEchoDelayFeedback(const float echo_fb) {
  echo_delay_[0]->SetFeedback(echo_fb);
  echo_delay_[1]->SetFeedback(echo_fb);
}
void Engine::SetEchoDelaySendAmount(const float echo_send) { echo_send_ = echo_send; }
void Engine::SetEchoDelayLagTime(const float lag_time_s) {
  echo_delay_[0]->SetLagTime(lag_time_s);
  echo_delay_[1]->SetLagTime(lag_time_s);
}
void Engine::SetReverbMix(const float mix) { verb_mix_ = fclamp(mix, 0.0f, 1.0f); }
void Engine::SetReverbFeedback(const float time) { verb_->SetFeedback(time); }
void Engine::SetOutputLevel(const float level) { output_level_ = level; }
void Engine::Process(float in, float &outL, float &outR) {
  fonepole(fb_delay_samp_, fb_delay_samp_target_, fb_delay_smooth_coef_);
  float inL, inR, sampL, sampR, echoL, echoR, verbL, verbR;
  const float noise_samp = noise_.Process();
  const float noise_gain = 1.0f - 0.9f * excitation_blend_;
  const float input_gain = 0.25f * excitation_blend_;
  const float excitation = noise_samp * noise_gain + in * input_gain;
  inL = fb_delayline_[0].Read(fb_delay_samp_) + excitation;
  inR = fb_delayline_[1].Read(daisysp::fmax(1.0f, fb_delay_samp_ - 4.f)) + excitation;
  sampL = strings_[0].Process(inL);
  sampR = strings_[1].Process(inR);
  sampL = overdrive_[0].Process(sampL);
  sampR = overdrive_[1].Process(sampR);
  fb_lpf_.ProcessStereo(sampL, sampR);
  fb_hpf_.ProcessStereo(sampL, sampR);
  verb_->Process(sampL, sampR, &verbL, &verbR);
  sampL -= (sampL - verbL) * verb_mix_;
  sampR -= (sampR - verbR) * verb_mix_;
  fb_delayline_[0].Write(sampL * fb_gain_);
  fb_delayline_[1].Write(sampR * fb_gain_);
  echoL = echo_delay_[0]->Process(sampL * echo_send_);
  echoR = echo_delay_[1]->Process(sampR * echo_send_);
  sampL = 0.5f * (sampL + echoL);
  sampR = 0.5f * (sampR + echoR);
  outL = sampL * output_level_;
  outR = sampR * output_level_;
}
