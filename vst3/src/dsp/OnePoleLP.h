// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#pragma once

#include <algorithm>
#include <cmath>

namespace pinchfx::dsp {

// One-pole lowpass:
// y[n] = (1 - a) * x[n] + a * y[n-1]
// a = exp(-2*pi*fc/Fs)
class OnePoleLP {
public:
    static constexpr double DEFAULT_SAMPLE_RATE = 44100.0; // Fallback when host reports 0 Hz.
    static constexpr double DEFAULT_CUTOFF = 1200.0; // Neutral-ish tone cutoff.
    static constexpr double MIN_CUTOFF = 10.0; // Avoid DC-only filter or divide-by-zero.
    static constexpr double PI = 3.14159265358979323846; // Math constant.

    void reset() { z_ = 0.0; }

    void setSampleRate(double sampleRate) {
        sampleRate_ = sampleRate > 0.0 ? sampleRate : DEFAULT_SAMPLE_RATE;
        updateCoeff();
    }

    void setCutoff(double freq) {
        cutoff_ = std::max(freq, MIN_CUTOFF);
        updateCoeff();
    }

    double process(double x) {
        if (!std::isfinite(x)) {
            x = 0.0;
        }
        if (!std::isfinite(z_)) {
            z_ = 0.0;
        }
        z_ = (1.0 - a_) * x + a_ * z_;
        return z_;
    }

private:
    void updateCoeff() {
        a_ = std::exp(-2.0 * PI * cutoff_ / sampleRate_);
    }

    double sampleRate_{DEFAULT_SAMPLE_RATE};
    double cutoff_{DEFAULT_CUTOFF};
    double a_{0.0};
    double z_{0.0};
};

} // namespace pinchfx::dsp
