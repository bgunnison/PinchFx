// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#pragma once


#include "PitchTrackerACF.h"
#include "PitchTrackerAlgo.h"
#include "TwoPoleResonator.h"
#include "OnePoleLP.h"
#include "TubeStage.h"
#include "PeakLimiter.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace pinchfx::dsp {

// Pinch harmonic synth path:
// input -> pitch tracker -> pitch-control smoothing -> AGC -> resonator @ harmonic -> heat blend -> 4-pole tone LP -> limiter
class PinchFxAlgorithm {
public:
    struct Params {
        static constexpr double DEFAULT_TRIG = 0.0; // Manual trigger off.
        static constexpr double DEFAULT_POSITION = 0.5; // Middle harmonic selection.
        static constexpr double DEFAULT_LOCK = 0.5; // Mid Q and tracking tightness.
        static constexpr double DEFAULT_GLIDE = 0.25; // Legacy state slot (unused).
        static constexpr double DEFAULT_TONE = 0.5; // Balanced brightness.
        static constexpr double DEFAULT_MIX = 0.35; // Preserve dry signal.
        static constexpr double DEFAULT_MONITOR = 0.0; // Monitor off.
        static constexpr double DEFAULT_MODE = 0.0; // Legacy state slot (unused).
        static constexpr double DEFAULT_HEAT = 0.0; // No added drive.
        static constexpr double DEFAULT_SENS = 0.5; // Legacy state slot (unused).

        double trig{DEFAULT_TRIG};
        double position{DEFAULT_POSITION};
        double lock{DEFAULT_LOCK};
        double glide{DEFAULT_GLIDE}; // Legacy state slot (unused).
        double tone{DEFAULT_TONE};
        double mix{DEFAULT_MIX};
        double monitor{DEFAULT_MONITOR};
        double mode{DEFAULT_MODE}; // Legacy state slot (unused).
        double heat{DEFAULT_HEAT};
        double sens{DEFAULT_SENS}; // Legacy state slot (unused).
    };

    struct SampleOut {
        double gate{0.0}; // Analysis gate from tracker confidence/error only.
        double agc{0.0}; // AGC-driven resonator input monitor.
        double f0{0.0};
        double fh{0.0};
        double conf{0.0};
        double pitchGate{0.0};
        double xbpRaw{0.0};
        double xbp{0.0};
        double xr{0.0};
        double xtone{0.0};
        double xtube{0.0};
        double xlim{0.0};
    };

    void prepare(double sampleRate, int maxBlock) {
        sampleRate_ = sampleRate > 0.0 ? sampleRate : DEFAULT_SAMPLE_RATE;
        maxBlock_ = maxBlock;
        trigWindowSamples_ = static_cast<int>(std::round(TRIG_WINDOW_REF_SAMPLES * sampleRate_ / TRIG_WINDOW_REF_SR));
        updateAgcCoeffs();

        const double minPitch = MIN_PITCH_HZ;
        int frameSize = static_cast<int>(std::round(sampleRate_ / minPitch));
        frameSize = std::clamp(frameSize, MIN_FRAME_SIZE, MAX_FRAME_SIZE);
        frameSize = ((frameSize + FRAME_ALIGN - 1) / FRAME_ALIGN) * FRAME_ALIGN;
        const int hopSize = std::max(MIN_HOP_SIZE, frameSize / HOP_DIV);
        pitchTracker_.prepare(sampleRate_, frameSize, hopSize, minPitch, MAX_PITCH_HZ);
        pitchControl_.prepare(sampleRate_);
        pitchControl_.setTimeConstants(PITCH_CTRL_TAU_SLOW_SEC, PITCH_CTRL_TAU_FAST_SEC);
        pitchControl_.setMaxCentsPerSecond(PITCH_CTRL_MAX_CENTS_PER_SEC);

        resonator_.setSampleRate(sampleRate_);
        toneLP_.setSampleRate(sampleRate_);
        toneLP2_.setSampleRate(sampleRate_);
        toneLP3_.setSampleRate(sampleRate_);
        toneLP4_.setSampleRate(sampleRate_);
        tubeStage_.setSampleRate(sampleRate_);
        limiter_.setSampleRate(sampleRate_);
        limiter_.setReleaseMs(LIMITER_RELEASE_MS);
        limiter_.setThreshold(LIMITER_THRESHOLD);

        updateDerived();
    }

    void reset() {
        pitchTracker_.reset();
        pitchControl_.reset();
        resonator_.reset();
        toneLP_.reset();
        toneLP2_.reset();
        toneLP3_.reset();
        toneLP4_.reset();
        tubeStage_.reset();
        limiter_.reset();
        f0Smoothed_ = DEFAULT_F0_HZ;
        agcDetector_ = 0.0;
        agcGain_ = 1.0;
    }

    void setParams(const Params& params) {
        params_ = params;
        updateDerived();
    }

    void triggerManual() {
        (void)trigWindowSamples_;
    }

    void resetGate() {
        // Gate is now an analysis-only signal (not in audio path), so nothing to reset.
    }

    SampleOut processSample(double monoIn) {
        SampleOut out{};
        pitchTracker_.processSample(monoIn);
        const double conf = pitchTracker_.lastConfidence();
        const double fHat = pitchTracker_.lastFrequency();
        const bool hasEstimate = std::isfinite(fHat) && (fHat > MIN_VALID_F0_HZ);
        const double fCtrl = pitchControl_.update(fHat, conf, hasEstimate);
        f0Smoothed_ = std::clamp(fCtrl, MIN_PITCH_HZ, MAX_PITCH_HZ);
        const double fh = static_cast<double>(partial_) * f0Smoothed_;
        double fhFinal = std::clamp(fh, MIN_HARMONIC_HZ, MAX_HARMONIC_NYQUIST * sampleRate_);
        if (!std::isfinite(fhFinal)) fhFinal = DEFAULT_F0_HZ;
        resonator_.setFrequency(fhFinal);

        const double absIn = std::abs(monoIn);
        const double agcEnvCoeff = (absIn > agcDetector_) ? agcEnvAttackCoeff_ : agcEnvReleaseCoeff_;
        agcDetector_ += agcEnvCoeff * (absIn - agcDetector_);
        const double desiredGain = std::clamp(AGC_TARGET_LEVEL / std::max(agcDetector_, AGC_INPUT_FLOOR), AGC_MIN_GAIN, AGC_MAX_GAIN);
        const double agcGainCoeff = (desiredGain > agcGain_) ? agcGainAttackCoeff_ : agcGainReleaseCoeff_;
        agcGain_ += agcGainCoeff * (desiredGain - agcGain_);
        const double resonatorIn = std::clamp(monoIn * agcGain_ * AGC_DRIVE, -AGC_DRIVE_CLIP, AGC_DRIVE_CLIP);
        out.agc = resonatorIn;

        const double xbpRaw = resonator_.process(resonatorIn);
        out.xbpRaw = xbpRaw;
        const double xbpNorm = (xbpRaw / std::max(MIN_Q_NORM, qValue_)) * RESONATOR_TRIM;
        out.xbp = xbpNorm;
        out.f0 = f0Smoothed_;
        out.fh = fhFinal;
        out.conf = conf;
        const double pitchRef = std::max(f0Smoothed_, MIN_VALID_F0_HZ);
        const double pitchError = hasEstimate ? std::abs(fHat - f0Smoothed_) / pitchRef : 1.0;
        const bool pitchGate = hasEstimate && (conf >= MIN_CONFIDENCE_GATE) && (pitchError <= PITCH_ERROR_RATIO_GATE);
        out.gate = pitchGate ? 1.0 : 0.0;
        out.pitchGate = out.gate;
        out.xr = RESONATOR_OUTPUT_GAIN * out.xbp;
        const double tubeOut = tubeStage_.process(out.xr);
        const double heatMix = std::clamp(params_.heat, 0.0, 1.0);
        out.xtube = out.xr + heatMix * (tubeOut - out.xr);
        const double t1 = toneLP_.process(out.xtube);
        const double t2 = toneLP2_.process(t1);
        const double t3 = toneLP3_.process(t2);
        out.xtone = toneLP4_.process(t3);
        out.xlim = limiter_.process(out.xtone);
        return out;
    }

    double qValue() const { return qValue_; }

private:
    static constexpr double DEFAULT_SAMPLE_RATE = 44100.0; // Fallback when host reports 0 Hz.
    static constexpr double TRIG_WINDOW_REF_SR = 48000.0; // Reference for manual trigger length.
    static constexpr double TRIG_WINDOW_REF_SAMPLES = 4096.0; // Manual trigger window at 48 kHz.
    static constexpr double MIN_PITCH_HZ = 70.0; // Lower bound for tracker and clamp.
    static constexpr double MAX_PITCH_HZ = 1200.0; // Upper bound for tracker and clamp.
    static constexpr int MIN_FRAME_SIZE = 512; // Stable ACF window for low pitches.
    static constexpr int MAX_FRAME_SIZE = 1024; // Avoids long latency.
    static constexpr int FRAME_ALIGN = 32; // SIMD/cache-friendly block size.
    static constexpr int HOP_DIV = 8; // Overlap ratio (frame/8).
    static constexpr int MIN_HOP_SIZE = 16; // Avoids tiny hops at small frames.
    static constexpr double LIMITER_RELEASE_MS = 50.0; // Prevents limiter pumping.
    static constexpr double LIMITER_THRESHOLD = 0.7; // Matches project spec.
    static constexpr double DEFAULT_F0_HZ = 110.0; // A2 reference pitch.
    static constexpr double MIN_VALID_F0_HZ = 1.0; // Reject invalid tracker output.
    static constexpr double MAX_HARMONIC_NYQUIST = 0.45; // Keep SVF below Nyquist.
    static constexpr double MIN_HARMONIC_HZ = 30.0; // Avoid sub-audio center freqs.
    static constexpr double RESONATOR_TRIM = 0.25; // Output trim after Q normalization.
    static constexpr double MIN_Q_NORM = 1.0; // Avoid divide-by-zero when normalizing.
    static constexpr double RESONATOR_OUTPUT_GAIN = 0.7; // Fixed output gain now that SQUEAL control is removed.
    static constexpr double PITCH_CTRL_TAU_SLOW_SEC = 0.20; // Slow f0 response when confidence is weak.
    static constexpr double PITCH_CTRL_TAU_FAST_SEC = 0.01; // Fast f0 response when confidence is strong.
    static constexpr double PITCH_CTRL_MAX_CENTS_PER_SEC = 1800.0; // Limits abrupt octave jumps.
    static constexpr double Q_MIN = 0.5; // Minimum resonator Q.
    static constexpr double Q_RANGE = 4.5; // Q range up to 5.0.
    static constexpr double TONE_CUTOFF_MIN = 250.0; // Darker minimum so tone control has clear effect.
    static constexpr double TONE_CUTOFF_RANGE = 9750.0; // Sweep up to ~10 kHz.
    static constexpr double TUBE_DRIVE_BASE = 1.0; // Neutral drive at heat=0.
    static constexpr double TUBE_DRIVE_RANGE = 7.0; // Heat adds up to 8x.
    static constexpr double TUBE_BIAS_BASE = 0.18; // Slight asymmetry for bite.
    static constexpr double TUBE_BIAS_RANGE = 0.16; // Tone reduces asymmetry.
    static constexpr double POSITION_SCALE = 4.0; // Map 0..1 position to 5 discrete harmonics.
    static constexpr double MIN_CONFIDENCE_GATE = 0.2; // Gate trace threshold for "pitch is trustworthy".
    static constexpr double PITCH_ERROR_RATIO_GATE = 0.08; // Gate trace tolerance for estimate deviation.
    static constexpr double AGC_TARGET_LEVEL = 0.2; // Target absolute level driving resonator.
    static constexpr double AGC_INPUT_FLOOR = 0.002; // Prevent runaway gain near silence.
    static constexpr double AGC_MIN_GAIN = 1.0; // Keep excitation strong once note starts.
    static constexpr double AGC_MAX_GAIN = 24.0; // Cap boost to avoid instability bursts.
    static constexpr double AGC_ENV_ATTACK_MS = 2.0; // Detect transients quickly.
    static constexpr double AGC_ENV_RELEASE_MS = 120.0; // Hold gain through note decay.
    static constexpr double AGC_GAIN_ATTACK_MS = 20.0; // Smooth gain increase to avoid zippering.
    static constexpr double AGC_GAIN_RELEASE_MS = 5.0; // Fast trim when input jumps.
    static constexpr double AGC_DRIVE = 1.0; // Resonator drive scalar.
    static constexpr double AGC_DRIVE_CLIP = 2.0; // Clip drive to keep resonator bounded.
    static constexpr double DEFAULT_Q_VALUE = Q_MIN + Q_RANGE * Params::DEFAULT_LOCK; // Derived default Q.
    static constexpr double DEFAULT_TONE_CUTOFF = TONE_CUTOFF_MIN + TONE_CUTOFF_RANGE * Params::DEFAULT_TONE; // Derived default tone cutoff.
    static constexpr double DEFAULT_TUBE_DRIVE = TUBE_DRIVE_BASE + TUBE_DRIVE_RANGE * Params::DEFAULT_HEAT; // Derived default drive.
    static constexpr double DEFAULT_TUBE_BIAS = TUBE_BIAS_BASE - TUBE_BIAS_RANGE * Params::DEFAULT_TONE; // Derived default bias.

    void updateDerived() {
        const double lock = params_.lock;
        const double tone = params_.tone;
        const double heat = params_.heat;
        const double glide = params_.glide;
        (void)glide; // Legacy parameter kept for state compatibility; not used in DSP path.

        qValue_ = Q_MIN + Q_RANGE * lock;
        toneCutoff_ = TONE_CUTOFF_MIN + TONE_CUTOFF_RANGE * tone;
        tubeDrive_ = TUBE_DRIVE_BASE + TUBE_DRIVE_RANGE * heat;
        tubeBias_ = TUBE_BIAS_BASE - TUBE_BIAS_RANGE * tone;

        static constexpr std::array<int, 5> PARTIALS{5, 7, 9, 12, 15}; // Odd/near-odd harmonics for pinch tone.
        const int posIndex = static_cast<int>(std::round(std::clamp(params_.position, 0.0, 1.0) * POSITION_SCALE));
        partial_ = PARTIALS[static_cast<size_t>(std::clamp(posIndex, 0, 4))];

        toneLP_.setCutoff(toneCutoff_);
        toneLP2_.setCutoff(toneCutoff_);
        toneLP3_.setCutoff(toneCutoff_);
        toneLP4_.setCutoff(toneCutoff_);
        resonator_.setQ(qValue_);
        tubeStage_.setDrive(tubeDrive_);
        tubeStage_.setBias(tubeBias_);
    }

    void updateAgcCoeffs() {
        const double envAttackSec = AGC_ENV_ATTACK_MS * 0.001;
        const double envReleaseSec = AGC_ENV_RELEASE_MS * 0.001;
        const double gainAttackSec = AGC_GAIN_ATTACK_MS * 0.001;
        const double gainReleaseSec = AGC_GAIN_RELEASE_MS * 0.001;
        agcEnvAttackCoeff_ = 1.0 - std::exp(-1.0 / (envAttackSec * sampleRate_));
        agcEnvReleaseCoeff_ = 1.0 - std::exp(-1.0 / (envReleaseSec * sampleRate_));
        agcGainAttackCoeff_ = 1.0 - std::exp(-1.0 / (gainAttackSec * sampleRate_));
        agcGainReleaseCoeff_ = 1.0 - std::exp(-1.0 / (gainReleaseSec * sampleRate_));
    }

    Params params_{};

    PitchTrackerACF pitchTracker_{};
    PitchTrackerAlgo pitchControl_{};
    TwoPoleResonator resonator_{};
    OnePoleLP toneLP_{};
    OnePoleLP toneLP2_{};
    OnePoleLP toneLP3_{};
    OnePoleLP toneLP4_{};
    TubeStage tubeStage_{};
    PeakLimiter limiter_{};

    double sampleRate_{DEFAULT_SAMPLE_RATE};
    int maxBlock_{0};
    int trigWindowSamples_{static_cast<int>(TRIG_WINDOW_REF_SAMPLES)};
    int partial_{5};

    double f0Smoothed_{DEFAULT_F0_HZ};
    double agcDetector_{0.0};
    double agcGain_{1.0};
    double agcEnvAttackCoeff_{0.0};
    double agcEnvReleaseCoeff_{0.0};
    double agcGainAttackCoeff_{0.0};
    double agcGainReleaseCoeff_{0.0};

    double qValue_{DEFAULT_Q_VALUE};
    double toneCutoff_{DEFAULT_TONE_CUTOFF};
    double tubeDrive_{DEFAULT_TUBE_DRIVE};
    double tubeBias_{DEFAULT_TUBE_BIAS};
};

} // namespace pinchfx::dsp
