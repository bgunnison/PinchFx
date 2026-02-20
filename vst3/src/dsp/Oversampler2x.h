// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#pragma once

#include <algorithm>
#include <cmath>

namespace pinchfx::dsp {

// Simple 2x oversampler:
// - Linear interpolation generates a mid-sample (0.5 * (prev + x))
// - Nonlinearity runs at 2x rate
// - One-pole lowpass smooths images
class Oversampler2x {
public:
    static constexpr double DEFAULT_SAMPLE_RATE = 44100.0; // Fallback when host reports 0 Hz.
    static constexpr double MIN_LP_CUTOFF = 10.0; // Avoid degenerate lowpass.
    static constexpr double LP_CUTOFF_RATIO = 0.45; // Gentle anti-imaging near Nyquist.
    static constexpr double OVERSAMPLE_FACTOR = 2.0; // Fixed 2x.
    static constexpr double PI = 3.14159265358979323846; // Math constant.

    void setSampleRate(double sampleRate) {
        sampleRate_ = sampleRate > 0.0 ? sampleRate : DEFAULT_SAMPLE_RATE;
        updateCoeff();
    }

    void reset() {
        prev_ = 0.0;
        lpState_ = 0.0;
    }

    template <typename NonlinearFn>
    double process(double x, NonlinearFn&& nonlinear) {
        if (!std::isfinite(x) || !std::isfinite(prev_) || !std::isfinite(lpState_)) {
            reset();
            x = std::isfinite(x) ? x : 0.0;
        }
        const double s0 = prev_;
        const double s1 = 0.5 * (prev_ + x);
        prev_ = x;

        double y0 = nonlinear(s0);
        double y1 = nonlinear(s1);

        y0 = (1.0 - lpA_) * y0 + lpA_ * lpState_;
        lpState_ = y0;
        y1 = (1.0 - lpA_) * y1 + lpA_ * lpState_;
        lpState_ = y1;

        if (!std::isfinite(y1)) {
            reset();
            return 0.0;
        }
        return y1;
    }

private:
    void updateCoeff() {
        const double cutoff = std::max(LP_CUTOFF_RATIO * sampleRate_, MIN_LP_CUTOFF);
        const double oversampledRate = sampleRate_ * OVERSAMPLE_FACTOR;
        lpA_ = std::exp(-2.0 * PI * cutoff / oversampledRate);
    }

    double sampleRate_{DEFAULT_SAMPLE_RATE};
    double prev_{0.0};
    double lpA_{0.0};
    double lpState_{0.0};
};

} // namespace pinchfx::dsp
