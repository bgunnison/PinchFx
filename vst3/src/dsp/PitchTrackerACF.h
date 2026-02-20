// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace pinchfx::dsp {

// Autocorrelation pitch tracker (frame-based ACF with Hann window):
// r[k] = sum_{n=0}^{N-k-1} x[n] * x[n+k]
// pick k in [lagMin, lagMax] with max r[k], then refine via parabolic fit
// f0 = Fs / k, confidence = r[k] / r[0]
class PitchTrackerACF {
public:
    static constexpr double DEFAULT_SAMPLE_RATE = 44100.0; // Fallback when host reports 0 Hz.
    static constexpr int DEFAULT_FRAME_SIZE = 1024; // Balanced resolution for guitar range.
    static constexpr int DEFAULT_HOP_SIZE = 128; // 8x overlap at 1024 frame.
    static constexpr int MIN_FRAME_SIZE = 64; // Avoid degenerate ACF buffers.
    static constexpr int MIN_HOP_SIZE = 1; // Always advance.
    static constexpr double DEFAULT_FMIN = 70.0; // Low E on guitar ~82 Hz, allow down to ~70 Hz.
    static constexpr double DEFAULT_FMAX = 1200.0; // Upper range for pinch harmonics.
    static constexpr double MIN_FREQ = 10.0; // Avoid huge lag sizes.
    static constexpr double MIN_FREQ_GAP = 1.0; // Ensure fMax > fMin.
    static constexpr double PI = 3.14159265358979323846; // Math constant.
    static constexpr double WINDOW_SCALE = 0.5; // Hann window scale.
    static constexpr double MIN_ENERGY = 1e-12; // Reject silent frames.
    static constexpr double LAG_REFINE_EPS = 1e-9; // Parabolic fit denominator guard.
    static constexpr double LAG_REFINE_HALF = 0.5; // Parabolic refinement factor.

    void prepare(double sampleRate, int frameSize, int hopSize, double fMin, double fMax) {
        sampleRate_ = sampleRate > 0.0 ? sampleRate : DEFAULT_SAMPLE_RATE;
        frameSize_ = std::max(frameSize, MIN_FRAME_SIZE);
        hopSize_ = std::max(hopSize, MIN_HOP_SIZE);
        fMin_ = std::max(fMin, MIN_FREQ);
        fMax_ = std::max(fMax, fMin_ + MIN_FREQ_GAP);
        buffer_.assign(static_cast<size_t>(frameSize_), 0.0);
        frame_.assign(static_cast<size_t>(frameSize_), 0.0);
        window_.assign(static_cast<size_t>(frameSize_), 0.0);
        for (int i = 0; i < frameSize_; ++i) {
            const double phase = (2.0 * PI * i) / (frameSize_ - 1);
            window_[static_cast<size_t>(i)] = WINDOW_SCALE - WINDOW_SCALE * std::cos(phase);
        }
        writeIndex_ = 0;
        filled_ = 0;
        hopCounter_ = 0;
        lastF0_ = 0.0;
        lastConfidence_ = 0.0;
        updateLagBounds();
    }

    void reset() {
        std::fill(buffer_.begin(), buffer_.end(), 0.0);
        std::fill(frame_.begin(), frame_.end(), 0.0);
        writeIndex_ = 0;
        filled_ = 0;
        hopCounter_ = 0;
        lastF0_ = 0.0;
        lastConfidence_ = 0.0;
    }

    void setFrequencyRange(double fMin, double fMax) {
        fMin_ = std::max(fMin, MIN_FREQ);
        fMax_ = std::max(fMax, fMin_ + MIN_FREQ_GAP);
        updateLagBounds();
    }

    void processSample(double x) {
        buffer_[static_cast<size_t>(writeIndex_)] = x;
        writeIndex_ = (writeIndex_ + 1) % frameSize_;
        if (filled_ < frameSize_) {
            filled_ += 1;
        }
        hopCounter_ += 1;
        if (filled_ >= frameSize_ && hopCounter_ >= hopSize_) {
            hopCounter_ = 0;
            computeFrame();
            computeACF();
        }
    }

    double lastFrequency() const { return lastF0_; }
    double lastConfidence() const { return lastConfidence_; }

private:
    void updateLagBounds() {
        lagMax_ = static_cast<int>(std::floor(sampleRate_ / fMin_));
        lagMin_ = static_cast<int>(std::floor(sampleRate_ / fMax_));
        lagMin_ = std::clamp(lagMin_, 1, frameSize_ - 2);
        lagMax_ = std::clamp(lagMax_, lagMin_ + 1, frameSize_ - 1);
    }

    void computeFrame() {
        int idx = writeIndex_;
        for (int i = 0; i < frameSize_; ++i) {
            double x = buffer_[static_cast<size_t>(idx)];
            frame_[static_cast<size_t>(i)] = x * window_[static_cast<size_t>(i)];
            idx = (idx + 1) % frameSize_;
        }
    }

    void computeACF() {
        double best = 0.0;
        int bestLag = lagMin_;
        double r0 = 0.0;
        for (int i = 0; i < frameSize_; ++i) {
            const double v = frame_[static_cast<size_t>(i)];
            r0 += v * v;
        }
        if (r0 <= MIN_ENERGY) {
            lastConfidence_ = 0.0;
            return;
        }
        for (int lag = lagMin_; lag <= lagMax_; ++lag) {
            double sum = 0.0;
            const int limit = frameSize_ - lag;
            for (int i = 0; i < limit; ++i) {
                sum += frame_[static_cast<size_t>(i)] * frame_[static_cast<size_t>(i + lag)];
            }
            if (sum > best) {
                best = sum;
                bestLag = lag;
            }
        }
        double refinedLag = static_cast<double>(bestLag);
        if (bestLag > lagMin_ && bestLag < lagMax_) {
            const double r1 = autocorrAt(bestLag - 1);
            const double r2 = best;
            const double r3 = autocorrAt(bestLag + 1);
            const double denom = (r1 - 2.0 * r2 + r3);
            if (std::abs(denom) > LAG_REFINE_EPS) {
                refinedLag = static_cast<double>(bestLag) + LAG_REFINE_HALF * (r1 - r3) / denom;
            }
        }
        if (!std::isfinite(refinedLag) || refinedLag <= 0.0) {
            lastConfidence_ = 0.0;
            return;
        }
        lastF0_ = sampleRate_ / refinedLag;
        if (!std::isfinite(lastF0_)) {
            lastF0_ = 0.0;
            lastConfidence_ = 0.0;
            return;
        }
        lastConfidence_ = best / r0;
    }

    double autocorrAt(int lag) const {
        double sum = 0.0;
        const int limit = frameSize_ - lag;
        for (int i = 0; i < limit; ++i) {
            sum += frame_[static_cast<size_t>(i)] * frame_[static_cast<size_t>(i + lag)];
        }
        return sum;
    }

    double sampleRate_{DEFAULT_SAMPLE_RATE};
    int frameSize_{DEFAULT_FRAME_SIZE};
    int hopSize_{DEFAULT_HOP_SIZE};
    double fMin_{DEFAULT_FMIN};
    double fMax_{DEFAULT_FMAX};
    int lagMin_{1};
    int lagMax_{2};
    std::vector<double> buffer_{};
    std::vector<double> frame_{};
    std::vector<double> window_{};
    int writeIndex_{0};
    int filled_{0};
    int hopCounter_{0};
    double lastF0_{0.0};
    double lastConfidence_{0.0};
};

} // namespace pinchfx::dsp
