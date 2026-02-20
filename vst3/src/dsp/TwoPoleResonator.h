// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#pragma once

#include <algorithm>
#include <cmath>

namespace pinchfx::dsp {

// Two-pole resonator (TPT state-variable bandpass form):
// g = tan(pi * f / Fs), r = 1 / (2Q), h = 1 / (1 + g*(g + r))
// v1 = (v0 - ic2 - r*ic1) * h
// v2 = ic1 + g*v1
// ic1 = v2 + g*v1
// ic2 = ic2 + g*v2
class TwoPoleResonator {
public:
    static constexpr double DEFAULT_SAMPLE_RATE = 44100.0; // Fallback when host reports 0 Hz.
    static constexpr double DEFAULT_FREQ = 440.0; // Musically neutral placeholder.
    static constexpr double DEFAULT_Q = 8.0; // Stable, moderately resonant.
    static constexpr double MIN_FREQ = 1.0; // Avoid tan(0) and degenerate filter.
    static constexpr double MAX_NYQUIST_RATIO = 0.49; // Keep safely below Nyquist.
    static constexpr double MIN_Q = 0.1; // Prevent divide-by-zero.
    static constexpr double PI = 3.14159265358979323846; // Math constant.

    void reset() {
        ic1eq_ = 0.0;
        ic2eq_ = 0.0;
    }

    void setSampleRate(double sampleRate) {
        sampleRate_ = sampleRate > 0.0 ? sampleRate : DEFAULT_SAMPLE_RATE;
        updateCoeffs();
    }

    void setFrequency(double freq) {
        freq_ = std::max(freq, MIN_FREQ);
        updateCoeffs();
    }

    void setQ(double q) {
        q_ = std::max(q, MIN_Q);
        updateCoeffs();
    }

    double process(double x) {
        if (!std::isfinite(ic1eq_) || !std::isfinite(ic2eq_)) {
            reset();
        }
        const double v0 = x;
        const double v1 = (v0 - ic2eq_ - r_ * ic1eq_) * h_;
        const double v2 = ic1eq_ + g_ * v1;
        if (!std::isfinite(v1) || !std::isfinite(v2)) {
            reset();
            return 0.0;
        }
        ic1eq_ = v2 + g_ * v1;
        ic2eq_ = ic2eq_ + g_ * v2;
        return v1;
    }

private:
    void updateCoeffs() {
        double freq = freq_;
        if (!std::isfinite(freq) || freq <= 0.0) {
            freq = MIN_FREQ;
        }
        const double maxFreq = MAX_NYQUIST_RATIO * sampleRate_;
        if (freq > maxFreq) freq = maxFreq;
        const double omega = PI * freq / sampleRate_;
        g_ = std::tan(omega);
        if (!std::isfinite(g_)) {
            g_ = 0.0;
        }
        r_ = 1.0 / (2.0 * q_);
        h_ = 1.0 / (1.0 + g_ * (g_ + r_));
        if (!std::isfinite(h_) || h_ == 0.0) {
            h_ = 1.0;
        }
    }

    double sampleRate_{DEFAULT_SAMPLE_RATE};
    double freq_{DEFAULT_FREQ};
    double q_{DEFAULT_Q};
    double g_{0.0};
    double r_{0.0};
    double h_{0.0};
    double ic1eq_{0.0};
    double ic2eq_{0.0};
};

} // namespace pinchfx::dsp
