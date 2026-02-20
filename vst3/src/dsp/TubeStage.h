// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#pragma once

#include "Oversampler2x.h"

#include <cmath>

namespace pinchfx::dsp {

// Tube-style soft clipper:
// y = tanh(drive * x + bias) - tanh(bias)
// Bias term removes DC offset after applying asymmetry.
class TubeStage {
public:
    static constexpr double DEFAULT_DRIVE = 1.0; // Unity drive (no extra saturation).
    static constexpr double DEFAULT_BIAS = 0.0; // Symmetric curve.

    void setSampleRate(double sampleRate) {
        oversampler_.setSampleRate(sampleRate);
    }

    void reset() {
        oversampler_.reset();
    }

    void setDrive(double drive) { drive_ = drive; }
    void setBias(double bias) { bias_ = bias; }

    double process(double x) {
        const double drive = drive_;
        const double bias = bias_;
        auto nonlin = [drive, bias](double u) {
            const double v = drive * u + bias;
            return std::tanh(v) - std::tanh(bias);
        };
        return oversampler_.process(x, nonlin);
    }

private:
    Oversampler2x oversampler_{};
    double drive_{DEFAULT_DRIVE};
    double bias_{DEFAULT_BIAS};
};

} // namespace pinchfx::dsp
