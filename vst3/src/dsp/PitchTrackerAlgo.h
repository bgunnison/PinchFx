// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#pragma once

#include <algorithm>
#include <cmath>

namespace pinchfx::dsp {

    // PitchTrackerAlgo
    // ----------------
    // This is the *control* algorithm that turns a raw pitch estimate (f_est) +
    // confidence (c in [0..1]) into a smooth, real-time control frequency (f_ctrl)
    // suitable for driving a resonator without binary muting.
    //
    // Core update:
    //   alpha = alpha_min + (alpha_max - alpha_min) * c
    //   f_ctrl += alpha * (f_est - f_ctrl)
    //
    // Implemented in log-frequency domain (more musical across octaves) and
    // protected by a cents/second slew limiter (kills octave blips).
    //
    // Usage:
    // - Call prepare(sampleRate) once.
    // - Each time you have a new estimate, call update(fEstHz, confidence01, true).
    // - If no estimate is available for a tick, call update(0, 0, false) to hold.
    //
    // Notes:
    // - This class does NOT perform pitch detection. Pair it with PitchTrackerACF
    //   or MIDI note input.
    class PitchTrackerAlgo {
    public:
        static constexpr double DEFAULT_SAMPLE_RATE = 44100.0;
        static constexpr double DEFAULT_FREQ = 440.0;

        // Clamp bounds for control frequency
        static constexpr double MIN_FREQ = 10.0;
        static constexpr double MAX_NYQUIST_RATIO = 0.49;

        // Default smoothing (seconds)
        // Low confidence -> slow tracking (hold-ish)
        // High confidence -> fast tracking (bend/vibrato capable)
        static constexpr double DEFAULT_TAU_SLOW_SEC = 0.200;
        static constexpr double DEFAULT_TAU_FAST_SEC = 0.010;

        // Default slew limiter (cents/sec). 1200 = 1 octave per second.
        static constexpr double DEFAULT_MAX_CENTS_PER_SEC = 1200.0;

        // Confidence clamp edges (optional convenience if you want to bias confidence)
        static constexpr double CONF_MIN = 0.0;
        static constexpr double CONF_MAX = 1.0;

        void prepare(double sampleRate) {
            sampleRate_ = sampleRate > 0.0 ? sampleRate : DEFAULT_SAMPLE_RATE;
            reset();
            updateAlphas();
        }

        void reset() {
            hasCtrl_ = false;
            fCtrlHz_ = DEFAULT_FREQ;
            fPrevHz_ = DEFAULT_FREQ;
            lastAlpha_ = 0.0;
            lastConfidence_ = 0.0;
        }

        void setSampleRate(double sampleRate) {
            sampleRate_ = sampleRate > 0.0 ? sampleRate : DEFAULT_SAMPLE_RATE;
            updateAlphas();
        }

        void setTimeConstants(double tauSlowSec, double tauFastSec) {
            tauSlowSec_ = std::max(tauSlowSec, 1e-6);
            tauFastSec_ = std::max(tauFastSec, 1e-6);
            updateAlphas();
        }

        void setMaxCentsPerSecond(double centsPerSecond) {
            maxCentsPerSec_ = std::max(centsPerSecond, 0.0);
        }

        // Optional: if your tracker confidence is not in [0..1], normalize before calling.
        // If hasEstimate=false, the algorithm holds f_ctrl (no updates).
        double update(double fEstHz, double confidence01, bool hasEstimate = true) {
            const double fs = std::max(sampleRate_, 1.0);

            if (!hasCtrl_) {
                // On first valid estimate, initialize. If none, hold default.
                if (hasEstimate && std::isfinite(fEstHz) && fEstHz > 0.0) {
                    fCtrlHz_ = clampFreq_(fEstHz);
                    fPrevHz_ = fCtrlHz_;
                    hasCtrl_ = true;
                    lastAlpha_ = alphaFast_; // arbitrary; will be overwritten next update
                    lastConfidence_ = std::clamp(confidence01, CONF_MIN, CONF_MAX);
                    return fCtrlHz_;
                }
                // no estimate, remain at default
                hasCtrl_ = true;
                fPrevHz_ = fCtrlHz_;
                lastAlpha_ = alphaSlow_;
                lastConfidence_ = 0.0;
                return fCtrlHz_;
            }

            if (!hasEstimate || !std::isfinite(fEstHz) || fEstHz <= 0.0) {
                // Hold control frequency; still advance fPrev for slew limiter consistency
                fPrevHz_ = fCtrlHz_;
                lastAlpha_ = alphaSlow_;
                lastConfidence_ = 0.0;
                return fCtrlHz_;
            }

            confidence01 = std::clamp(confidence01, CONF_MIN, CONF_MAX);
            lastConfidence_ = confidence01;

            // Confidence-adaptive alpha
            const double alpha = alphaSlow_ + (alphaFast_ - alphaSlow_) * confidence01;
            lastAlpha_ = alpha;

            // Clamp inputs to safe range
            const double fEst = clampFreq_(fEstHz);
            const double fCur = clampFreq_(fCtrlHz_);

            // Log-frequency smoothing
            const double lEst = std::log(fEst);
            double lCtl = std::log(fCur);
            lCtl += alpha * (lEst - lCtl);
            double fNew = std::exp(lCtl);

            // Slew limiting in cents/sec (prevents octave blips)
            if (maxCentsPerSec_ > 0.0) {
                const double centsPerSample = maxCentsPerSec_ / fs;
                const double maxRatio = std::pow(2.0, centsPerSample / 1200.0);
                const double fLo = fPrevHz_ / maxRatio;
                const double fHi = fPrevHz_ * maxRatio;
                fNew = std::clamp(fNew, fLo, fHi);
            }

            fNew = clampFreq_(fNew);

            fPrevHz_ = fNew;
            fCtrlHz_ = fNew;
            return fCtrlHz_;
        }

        double controlFrequencyHz() const { return fCtrlHz_; }
        double lastConfidence() const { return lastConfidence_; }
        double lastAlpha() const { return lastAlpha_; }

    private:
        static double alphaFromTau_(double tauSec, double fs) {
            tauSec = std::max(tauSec, 1e-6);
            fs = std::max(fs, 1.0);
            return 1.0 - std::exp(-1.0 / (tauSec * fs));
        }

        void updateAlphas() {
            const double fs = std::max(sampleRate_, 1.0);

            // Ensure "fast" is actually faster than "slow" (smaller tau => larger alpha).
            const double tauSlow = std::max(tauSlowSec_, 1e-6);
            const double tauFast = std::max(tauFastSec_, 1e-6);

            alphaSlow_ = alphaFromTau_(tauSlow, fs);
            alphaFast_ = alphaFromTau_(tauFast, fs);

            // If user swapped them accidentally, fix ordering.
            if (alphaFast_ < alphaSlow_) std::swap(alphaFast_, alphaSlow_);
        }

        double clampFreq_(double f) const {
            const double fs = std::max(sampleRate_, 1.0);
            const double fMax = MAX_NYQUIST_RATIO * fs;
            if (!std::isfinite(f)) return MIN_FREQ;
            return std::clamp(f, MIN_FREQ, fMax);
        }

        double sampleRate_{ DEFAULT_SAMPLE_RATE };

        double tauSlowSec_{ DEFAULT_TAU_SLOW_SEC };
        double tauFastSec_{ DEFAULT_TAU_FAST_SEC };

        double maxCentsPerSec_{ DEFAULT_MAX_CENTS_PER_SEC };

        double alphaSlow_{ 0.0 };
        double alphaFast_{ 0.0 };

        bool hasCtrl_{ false };
        double fCtrlHz_{ DEFAULT_FREQ };
        double fPrevHz_{ DEFAULT_FREQ };

        double lastAlpha_{ 0.0 };
        double lastConfidence_{ 0.0 };
    };

} // namespace pinchfx::dsp
