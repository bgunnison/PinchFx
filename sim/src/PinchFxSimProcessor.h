// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#pragma once

#include "../../vst3/src/dsp/PinchFxAlgorithm.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace pinchfx::sim {

class PinchFxSimProcessor {
public:
    using Params = dsp::PinchFxAlgorithm::Params;

    void prepare(double sampleRate, int maxBlock) {
        algorithm_.prepare(sampleRate, maxBlock);
    }

    void reset() {
        algorithm_.reset();
    }

    void setParams(const Params& params) {
        params_ = params;
        algorithm_.setParams(params);
    }

    void triggerManual() {
        algorithm_.triggerManual();
    }

    void resetGate() {
        algorithm_.resetGate();
    }

    void processBlock(const float* in, float* out, int numChannels, int numSamples) {
        static constexpr double kPi = 3.14159265358979323846;
        static constexpr double kWetMakeup = 3.0; // Compensate lower wet-path level so WET/DRY balance is usable.
        const double mix = std::clamp(params_.mix, 0.0, 1.0);
        const double mixShaped = std::sqrt(mix); // Match plugin mix taper: more audible wet around mid knob.
        const double dryMix = std::cos(0.5 * kPi * mixShaped);
        const double wetMix = std::sin(0.5 * kPi * mixShaped);
        for (int i = 0; i < numSamples; ++i) {
            const int base = i * numChannels;
            const double inL = in[base];
            const double inR = (numChannels > 1) ? in[base + 1] : inL;
            const double monoIn = 0.5 * (inL + inR);

            const auto algOut = algorithm_.processSample(monoIn);
            lastAgc_ = algOut.agc;
            lastF0_ = algOut.f0;
            lastFh_ = algOut.fh;
            lastConf_ = algOut.conf;
            lastResonator_ = algOut.xbp;
            lastTone_ = algOut.xtone;
            lastTube_ = algOut.xtube;
            lastLimiter_ = algOut.xlim;
            const double wet = kWetMakeup * algOut.xlim;
            const double outL = dryMix * inL + wetMix * wet;
            const double outR = dryMix * inR + wetMix * wet;

            out[base] = static_cast<float>(outL);
            if (numChannels > 1) out[base + 1] = static_cast<float>(outR);
        }
    }

    double lastAgc() const { return lastAgc_; }
    double lastF0() const { return lastF0_; }
    double lastFh() const { return lastFh_; }
    double lastConf() const { return lastConf_; }
    double lastResonator() const { return lastResonator_; }
    double lastTone() const { return lastTone_; }
    double lastTube() const { return lastTube_; }
    double lastLimiter() const { return lastLimiter_; }
    double qValue() const { return algorithm_.qValue(); }

private:
    Params params_{};
    dsp::PinchFxAlgorithm algorithm_{};
    double lastAgc_{0.0};
    double lastF0_{0.0};
    double lastFh_{0.0};
    double lastConf_{0.0};
    double lastResonator_{0.0};
    double lastTone_{0.0};
    double lastTube_{0.0};
    double lastLimiter_{0.0};
};

} // namespace pinchfx::sim
