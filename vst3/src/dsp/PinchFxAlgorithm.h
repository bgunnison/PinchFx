// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#pragma once


#include "PitchTrackerACF.h"
#include "PitchTrackerAlgo.h"
#include "../PinchFxPartials.h"
#include "TwoPoleControlledResonator.h"
#include "MiniComb.h"
#include "OnePoleLP.h"
#include "TubeStage.h"
#include "PeakLimiter.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace pinchfx::dsp {

// Pinch harmonic synth path:
// input -> pitch tracker -> pitch-control smoothing -> AGC -> 3x parallel (resonator @ harmonic -> comb feedback)
// -> per-path gain sum -> heat blend -> 4-pole tone LP -> limiter
class PinchFxAlgorithm {
public:
    struct Params {
        static constexpr double DEFAULT_TRIG = 0.0; // Manual trigger off.
        static constexpr double DEFAULT_POSITION = 0.5; // Middle harmonic selection.
        static constexpr double DEFAULT_POSITION2 = 0.5; // Voice B harmonic selection.
        static constexpr double DEFAULT_POSITION3 = 0.5; // Voice C harmonic selection.
        static constexpr double DEFAULT_LOCK = 0.0; // Resonance control in normalized 0..1 space.
        static constexpr double DEFAULT_LOCK2 = DEFAULT_LOCK; // Voice B resonance default.
        static constexpr double DEFAULT_LOCK3 = DEFAULT_LOCK; // Voice C resonance default.
        static constexpr double DEFAULT_GLIDE = 0.25; // Tracker time-constant control.
        static constexpr double DEFAULT_TONE = 1.0; // Default fully open tone.
        static constexpr double DEFAULT_MIX = 1.0; // Full wet by default.
        static constexpr double DEFAULT_MONITOR = 0.0; // Monitor off.
        static constexpr double DEFAULT_MODE = 0.0; // Legacy state slot (unused).
        static constexpr double DEFAULT_HEAT = 0.0; // No added drive.
        static constexpr double DEFAULT_SENS = 0.5; // AGC drive trim neutral point.
        static constexpr double DEFAULT_FEEDBACK = 0.0; // Comb stage off by default.
        static constexpr double DEFAULT_FEEDBACK2 = 0.0; // Voice B comb off by default.
        static constexpr double DEFAULT_FEEDBACK3 = 0.0; // Voice C comb off by default.
        static constexpr double DEFAULT_GAIN1 = 1.0; // Main voice on by default.
        static constexpr double DEFAULT_GAIN2 = 0.0; // Extra voices off by default.
        static constexpr double DEFAULT_GAIN3 = 0.0; // Extra voices off by default.

        double trig{DEFAULT_TRIG};
        double position{DEFAULT_POSITION};
        double position2{DEFAULT_POSITION2};
        double position3{DEFAULT_POSITION3};
        double lock{DEFAULT_LOCK};
        double lock2{DEFAULT_LOCK2};
        double lock3{DEFAULT_LOCK3};
        double glide{DEFAULT_GLIDE}; // Tracker response time.
        double tone{DEFAULT_TONE};
        double mix{DEFAULT_MIX};
        double monitor{DEFAULT_MONITOR};
        double mode{DEFAULT_MODE}; // Legacy state slot (unused).
        double heat{DEFAULT_HEAT};
        double sens{DEFAULT_SENS}; // AGC drive trim control (0.5 = unity).
        double feedback{DEFAULT_FEEDBACK}; // Mini-comb feedback amount.
        double feedback2{DEFAULT_FEEDBACK2}; // Voice B mini-comb feedback.
        double feedback3{DEFAULT_FEEDBACK3}; // Voice C mini-comb feedback.
        double gain1{DEFAULT_GAIN1}; // Voice A mix gain.
        double gain2{DEFAULT_GAIN2}; // Voice B mix gain.
        double gain3{DEFAULT_GAIN3}; // Voice C mix gain.
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

        for (int i = 0; i < kPathCount; ++i) {
            resonators_[i].setSampleRate(sampleRate_);
            combs_[i].prepare(sampleRate_, COMB_MAX_DELAY_SECONDS);
            combs_[i].setSampleRate(sampleRate_);
        }
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
        for (int i = 0; i < kPathCount; ++i) {
            resonators_[i].reset();
            combs_[i].reset();
        }
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
        const bool estimateInRange = std::isfinite(fHat) && (fHat >= MIN_PITCH_HZ) && (fHat <= MAX_PITCH_HZ);
        const bool trackerCanUpdate = estimateInRange && (conf >= MIN_CONFIDENCE_TRACK_UPDATE);
        const double fCtrl = pitchControl_.update(fHat, conf, trackerCanUpdate);
        f0Smoothed_ = std::clamp(fCtrl, MIN_PITCH_HZ, MAX_PITCH_HZ);

        const double absIn = std::abs(monoIn);
        const double agcEnvCoeff = (absIn > agcDetector_) ? agcEnvAttackCoeff_ : agcEnvReleaseCoeff_;
        agcDetector_ += agcEnvCoeff * (absIn - agcDetector_);
        const double desiredGain = std::clamp(AGC_TARGET_LEVEL / std::max(agcDetector_, AGC_INPUT_FLOOR), AGC_MIN_GAIN, AGC_MAX_GAIN);
        const double agcGainCoeff = (desiredGain > agcGain_) ? agcGainAttackCoeff_ : agcGainReleaseCoeff_;
        agcGain_ += agcGainCoeff * (desiredGain - agcGain_);
        const double resonatorIn = std::clamp(monoIn * agcGain_ * AGC_DRIVE * agcDriveTrim_, -AGC_DRIVE_CLIP, AGC_DRIVE_CLIP);
        out.agc = resonatorIn;

        double xbpRawSum = 0.0;
        double xbpNormSum = 0.0;
        double resonatorWetSum = 0.0;
        double fhOut = DEFAULT_F0_HZ;
        for (int i = 0; i < kPathCount; ++i) {
            const double fh = static_cast<double>(partials_[i]) * f0Smoothed_;
            double fhFinal = std::clamp(fh, MIN_HARMONIC_HZ, MAX_HARMONIC_NYQUIST * sampleRate_);
            if (!std::isfinite(fhFinal)) fhFinal = DEFAULT_F0_HZ;
            if (i == 0) fhOut = fhFinal;

            resonators_[i].setFrequency(fhFinal);
            combs_[i].setFrequency(fhFinal);

            const double xbpRaw = resonators_[i].process(resonatorIn);
            const double xbpNorm = xbpRaw * RESONATOR_TRIM;
            const double resonatorOut = RESONATOR_OUTPUT_GAIN * xbpNorm;
            const double voiceOut = combs_[i].process(resonatorOut);
            const double gain = pathGains_[i];

            xbpRawSum += gain * xbpRaw;
            xbpNormSum += gain * xbpNorm;
            resonatorWetSum += gain * voiceOut;
        }

        out.xbpRaw = xbpRawSum;
        out.xbp = xbpNormSum;
        out.f0 = f0Smoothed_;
        out.fh = fhOut;
        out.conf = conf;
        const double pitchRef = std::max(f0Smoothed_, MIN_VALID_F0_HZ);
        const double pitchError = estimateInRange ? std::abs(fHat - f0Smoothed_) / pitchRef : 1.0;
        const bool pitchGate = estimateInRange && (conf >= MIN_CONFIDENCE_GATE) && (pitchError <= PITCH_ERROR_RATIO_GATE);
        out.gate = pitchGate ? 1.0 : 0.0;
        out.pitchGate = out.gate;
        out.xr = resonatorWetSum;
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

    double qValue() const { return qValues_[0]; }

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
    static constexpr int kPathCount = 3; // Three parallel resonator/comb paths.
    static constexpr double RESONATOR_TRIM = 0.25; // Output trim after resonator stage.
    static constexpr double RESONATOR_OUTPUT_GAIN = 0.7; // Fixed output gain now that SQUEAL control is removed.
    static constexpr double PITCH_CTRL_TAU_SLOW_SEC = 0.20; // Base slow f0 response when confidence is weak.
    static constexpr double PITCH_CTRL_TAU_FAST_SEC = 0.01; // Base fast f0 response when confidence is strong.
    static constexpr double PITCH_CTRL_MAX_CENTS_PER_SEC = 1800.0; // Limits abrupt octave jumps.
    static constexpr double TRACKER_TC_SCALE_MIN = 0.4; // Fastest tracker setting.
    static constexpr double TRACKER_TC_SCALE_RANGE = 2.6; // Slowest setting is 3x base tau.
    static constexpr double MIN_CONFIDENCE_TRACK_UPDATE = 0.25; // Freeze F0 below this confidence.
    static constexpr double TONE_CUTOFF_MIN = 250.0; // Darker minimum so tone control has clear effect.
    static constexpr double TONE_CUTOFF_RANGE = 11750.0; // Sweep up to 12 kHz.
    static constexpr double TUBE_DRIVE_BASE = 1.0; // Neutral drive at heat=0.
    static constexpr double TUBE_DRIVE_RANGE = 7.0; // Heat adds up to 8x.
    static constexpr double TUBE_BIAS_BASE = 0.18; // Slight asymmetry for bite.
    static constexpr double TUBE_BIAS_RANGE = 0.16; // Tone reduces asymmetry.
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
    static constexpr double AGC_DRIVE_TRIM_DB_RANGE = 40.0; // sens maps to -20..+20 dB around unity.
    static constexpr double COMB_MAX_DELAY_SECONDS = 0.2; // Supports low tuning frequencies without wrap.
    static constexpr double COMB_DAMPING = 0.35; // Prevents harsh high-frequency buildup.
    static constexpr double DEFAULT_Q_VALUE = Params::DEFAULT_LOCK; // Resonance is already normalized 0..1.
    static constexpr double DEFAULT_TONE_CUTOFF = TONE_CUTOFF_MIN + TONE_CUTOFF_RANGE * Params::DEFAULT_TONE; // Derived default tone cutoff.
    static constexpr double DEFAULT_TUBE_DRIVE = TUBE_DRIVE_BASE + TUBE_DRIVE_RANGE * Params::DEFAULT_HEAT; // Derived default drive.
    static constexpr double DEFAULT_TUBE_BIAS = TUBE_BIAS_BASE - TUBE_BIAS_RANGE * Params::DEFAULT_TONE; // Derived default bias.
    static constexpr double DEFAULT_AGC_DRIVE_TRIM = 1.0; // sens default (0.5) keeps AGC drive unchanged.

    int selectPartial_(double positionNorm) const {
        return pinchfx::partialFromNormalized(positionNorm);
    }

    void updateDerived() {
        const double tone = params_.tone;
        const double heat = params_.heat;
        const double sens = std::clamp(params_.sens, 0.0, 1.0);
        const double glide = std::clamp(params_.glide, 0.0, 1.0);
        const double trackerScale = TRACKER_TC_SCALE_MIN + TRACKER_TC_SCALE_RANGE * glide;
        const double trackerTauSlow = PITCH_CTRL_TAU_SLOW_SEC * trackerScale;
        const double trackerTauFast = PITCH_CTRL_TAU_FAST_SEC * trackerScale;
        const double driveTrimDb = (sens - 0.5) * AGC_DRIVE_TRIM_DB_RANGE;
        toneCutoff_ = TONE_CUTOFF_MIN + TONE_CUTOFF_RANGE * tone;
        tubeDrive_ = TUBE_DRIVE_BASE + TUBE_DRIVE_RANGE * heat;
        tubeBias_ = TUBE_BIAS_BASE - TUBE_BIAS_RANGE * tone;
        agcDriveTrim_ = std::pow(10.0, driveTrimDb / 20.0);

        partials_[0] = selectPartial_(params_.position);
        partials_[1] = selectPartial_(params_.position2);
        partials_[2] = selectPartial_(params_.position3);

        qValues_[0] = std::clamp(params_.lock, 0.0, 1.0);
        qValues_[1] = std::clamp(params_.lock2, 0.0, 1.0);
        qValues_[2] = std::clamp(params_.lock3, 0.0, 1.0);

        combFeedbacks_[0] = std::clamp(params_.feedback, 0.0, 1.0);
        combFeedbacks_[1] = std::clamp(params_.feedback2, 0.0, 1.0);
        combFeedbacks_[2] = std::clamp(params_.feedback3, 0.0, 1.0);

        pathGains_[0] = std::clamp(params_.gain1, 0.0, 1.0);
        pathGains_[1] = std::clamp(params_.gain2, 0.0, 1.0);
        pathGains_[2] = std::clamp(params_.gain3, 0.0, 1.0);

        toneLP_.setCutoff(toneCutoff_);
        toneLP2_.setCutoff(toneCutoff_);
        toneLP3_.setCutoff(toneCutoff_);
        toneLP4_.setCutoff(toneCutoff_);
        for (int i = 0; i < kPathCount; ++i) {
            resonators_[i].setQ(qValues_[i]);
            combs_[i].setFeedback(combFeedbacks_[i]);
            combs_[i].setMix(1.0);
            combs_[i].setDamping(COMB_DAMPING);
        }
        pitchControl_.setTimeConstants(trackerTauSlow, trackerTauFast);
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
    std::array<TwoPoleResonator, kPathCount> resonators_{};
    std::array<MiniComb, kPathCount> combs_{};
    OnePoleLP toneLP_{};
    OnePoleLP toneLP2_{};
    OnePoleLP toneLP3_{};
    OnePoleLP toneLP4_{};
    TubeStage tubeStage_{};
    PeakLimiter limiter_{};

    double sampleRate_{DEFAULT_SAMPLE_RATE};
    int maxBlock_{0};
    int trigWindowSamples_{static_cast<int>(TRIG_WINDOW_REF_SAMPLES)};
    std::array<int, kPathCount> partials_{5, 5, 5};

    double f0Smoothed_{DEFAULT_F0_HZ};
    double agcDetector_{0.0};
    double agcGain_{1.0};
    double agcEnvAttackCoeff_{0.0};
    double agcEnvReleaseCoeff_{0.0};
    double agcGainAttackCoeff_{0.0};
    double agcGainReleaseCoeff_{0.0};

    std::array<double, kPathCount> qValues_{DEFAULT_Q_VALUE, DEFAULT_Q_VALUE, DEFAULT_Q_VALUE};
    std::array<double, kPathCount> combFeedbacks_{Params::DEFAULT_FEEDBACK, Params::DEFAULT_FEEDBACK2, Params::DEFAULT_FEEDBACK3};
    std::array<double, kPathCount> pathGains_{Params::DEFAULT_GAIN1, Params::DEFAULT_GAIN2, Params::DEFAULT_GAIN3};
    double toneCutoff_{DEFAULT_TONE_CUTOFF};
    double tubeDrive_{DEFAULT_TUBE_DRIVE};
    double tubeBias_{DEFAULT_TUBE_BIAS};
    double agcDriveTrim_{DEFAULT_AGC_DRIVE_TRIM};
};

} // namespace pinchfx::dsp
