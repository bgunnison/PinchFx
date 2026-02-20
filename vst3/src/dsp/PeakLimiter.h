// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#pragma once

#include <algorithm>
#include <cmath>

namespace pinchfx::dsp {

// Peak limiter:
// p[n] = max(|x[n]|, releaseCoeff * p[n-1])
// gain = min(1, threshold / (p + eps))
// y[n] = gain * x[n]
class PeakLimiter {
public:
    static constexpr double DEFAULT_SAMPLE_RATE = 44100.0; // Fallback when host reports 0 Hz.
    static constexpr double DEFAULT_RELEASE_MS = 50.0; // Fast recovery without pumping.
    static constexpr double MIN_RELEASE_MS = 1.0; // Prevent divide-by-zero.
    static constexpr double DEFAULT_THRESHOLD = 0.7; // Matches project spec.
    static constexpr double MIN_THRESHOLD = 0.01; // Avoid extreme gains.
    static constexpr double MS_TO_SEC = 0.001; // Unit conversion.
    static constexpr double EPS = 1e-9; // Prevent divide-by-zero.
    static constexpr double MAX_ABS_INPUT = 32.0; // Bound extreme bursts so release time stays practical.
    static constexpr double MAX_PEAK = 32.0; // Keep peak follower in a stable range.

    void setSampleRate(double sampleRate) {
        sampleRate_ = sampleRate > 0.0 ? sampleRate : DEFAULT_SAMPLE_RATE;
        updateCoeff();
    }

    void reset() { peak_ = 0.0; }

    void setReleaseMs(double releaseMs) {
        releaseMs_ = std::max(releaseMs, MIN_RELEASE_MS);
        updateCoeff();
    }

    void setThreshold(double threshold) { threshold_ = std::max(threshold, MIN_THRESHOLD); }

    double process(double x) {
        if (!std::isfinite(x)) {
            reset();
            return 0.0;
        }
        if (!std::isfinite(peak_)) {
            peak_ = 0.0;
        }
        const double xSafe = std::clamp(x, -MAX_ABS_INPUT, MAX_ABS_INPUT);
        const double absx = std::abs(xSafe);
        peak_ = std::max(absx, releaseCoeff_ * peak_);
        peak_ = std::min(peak_, MAX_PEAK);
        const double gain = std::min(1.0, threshold_ / (peak_ + EPS));
        const double y = xSafe * gain;
        if (!std::isfinite(y)) {
            reset();
            return 0.0;
        }
        return y;
    }

private:
    void updateCoeff() {
        const double releaseSec = releaseMs_ * MS_TO_SEC;
        releaseCoeff_ = std::exp(-1.0 / (releaseSec * sampleRate_));
    }

    double sampleRate_{DEFAULT_SAMPLE_RATE};
    double releaseMs_{DEFAULT_RELEASE_MS};
    double releaseCoeff_{0.0};
    double threshold_{DEFAULT_THRESHOLD};
    double peak_{0.0};
};

} // namespace pinchfx::dsp
