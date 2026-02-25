// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace pinchfx::dsp {

    // MiniComb
    // --------
    // Small feedback comb resonator with fractional delay and one-pole damping
    // in the feedback path.
    //
    // Intended use (per prior suggestion):
    // - NOT as the primary pinch-harmonic resonator
    // - As a subtle "body/air/string" color stage after the TwoPole resonator / tube
    //
    // API is kept similar in spirit to setFrequency()/setFeedback().
    // Mapping:
    //   setFrequency(f): tunes delay length D ~= Fs / f
    //   setFeedback(a): maps 0..1 control to comb decay/Q range.
    //
    // Notes:
    // - Stable as long as |feedback| < 1 after damping.
    // - Use low mix / low feedback for subtle enhancement.
    // - Call prepare(sampleRate, maxDelaySeconds) before use.
    class MiniComb {
    public:
        static constexpr double DEFAULT_SAMPLE_RATE = 44100.0;
        static constexpr double DEFAULT_FREQ_HZ = 440.0;
        static constexpr double DEFAULT_FEEDBACK = 0.0;
        static constexpr double MIN_FREQ_HZ = 20.0;
        static constexpr double MAX_NYQUIST_RATIO = 0.45;
        static constexpr double MIN_Q = 0.1;
        static constexpr double MAX_Q = 50.0;
        static constexpr double MAX_FEEDBACK_GAIN = 0.997; // Keep below 1.0 for stability while allowing longer ring.
        static constexpr double FEEDBACK_CURVE = 0.65; // <1 expands upper control range so max FEEDBACK is more audible.

        void prepare(double sampleRate, double maxDelaySeconds = 0.1) {
            sampleRate_ = (sampleRate > 1.0) ? sampleRate : DEFAULT_SAMPLE_RATE;

            const double maxDelaySampsD = std::ceil(std::max(1e-3, maxDelaySeconds) * sampleRate_);
            const std::size_t maxDelaySamps = static_cast<std::size_t>(std::max(8.0, maxDelaySampsD));

            buffer_.assign(maxDelaySamps + 4, 0.0); // pad for safe interpolation wrap
            writeIndex_ = 0;

            reset();
            updateDelaySamples_();
            updateFeedback_();
            updateDamping_();
        }

        void reset() {
            std::fill(buffer_.begin(), buffer_.end(), 0.0);
            writeIndex_ = 0;
            fbLpState_ = 0.0;
        }

        void setSampleRate(double sampleRate) {
            sampleRate_ = (sampleRate > 1.0) ? sampleRate : DEFAULT_SAMPLE_RATE;
            // keep existing buffer size; caller can call prepare() to resize if needed
            updateDelaySamples_();
            updateFeedback_();
            updateDamping_();
        }

        // Tune comb pitch. Delay D ~= Fs/f.
        void setFrequency(double hz) {
            freqHz_ = hz;
            updateDelaySamples_();
            updateFeedback_();
            updateDamping_();
        }

        // UI-facing FEEDBACK control (0..1).
        // Internally mapped into a Q/decay range.
        void setFeedback(double amount01) {
            feedback01_ = std::clamp(amount01, 0.0, 1.0);
            q_ = MIN_Q + (MAX_Q - MIN_Q) * feedback01_;
            updateFeedback_();
        }

        // Feedback polarity. Negative can sound more "hollow"/stringy in some contexts.
        void setNegativeFeedback(bool enabled) {
            negativeFeedback_ = enabled;
            updateFeedback_();
        }

        // Damping in feedback path (0..1). Higher = darker/faster HF decay.
        void setDamping(double damping01) {
            damping01_ = std::clamp(damping01, 0.0, 1.0);
            updateDamping_();
        }

        // Extra "body" modulation (optional, subtle only).
        // Depth in samples, rate in Hz.
        void setModulation(double depthSamples, double rateHz) {
            modDepthSamples_ = std::max(0.0, depthSamples);
            modRateHz_ = std::max(0.0, rateHz);
        }

        void setMix(double mix01) {
            mix_ = std::clamp(mix01, 0.0, 1.0);
        }

        // Wet-only comb output (for inserting in a larger wet path)
        double processWet(double x) {
            if (buffer_.empty()) return x;

            // Optional delay modulation (very subtle recommended)
            double mod = 0.0;
            if (modDepthSamples_ > 0.0 && modRateHz_ > 0.0) {
                const double phaseInc = (2.0 * kPi * modRateHz_) / std::max(sampleRate_, 1.0);
                lfoPhase_ += phaseInc;
                if (lfoPhase_ > 2.0 * kPi) lfoPhase_ -= 2.0 * kPi;
                mod = modDepthSamples_ * std::sin(lfoPhase_);
            }

            const double delay = std::clamp(delaySamples_ + mod, 1.0, static_cast<double>(buffer_.size() - 3));
            const double delayed = readFrac_(delay);

            // Damping in feedback path (one-pole LP on delayed sample)
            // fbLpState_ = (1-a)*delayed + a*fbLpState_
            fbLpState_ = (1.0 - dampA_) * delayed + dampA_ * fbLpState_;

            // Write/output sample for a standard feedback comb:
            // y[n] = x[n] + g * y[n-D]
            const double fb = feedback_ * fbLpState_;
            const double y = sanitize_(x + fb);

            buffer_[writeIndex_] = y;
            advanceWrite_();

            return y;
        }

        // Dry/wet convenience
        double process(double x) {
            const double wet = processWet(x);
            return (1.0 - mix_) * x + mix_ * wet;
        }

    private:
        static constexpr double kPi = 3.14159265358979323846;

        double sanitize_(double v) const {
            if (!std::isfinite(v)) return 0.0;
            // Hard clamp to avoid runaway if externally misused
            return std::clamp(v, -8.0, 8.0);
        }

        void advanceWrite_() {
            ++writeIndex_;
            if (writeIndex_ >= buffer_.size()) writeIndex_ = 0;
        }

        // 4-point cubic Hermite interpolation
        double readFrac_(double delaySamples) const {
            const double readPos = static_cast<double>(writeIndex_) - delaySamples;
            double rp = readPos;
            const double size = static_cast<double>(buffer_.size());

            while (rp < 0.0) rp += size;
            while (rp >= size) rp -= size;

            const int i1 = static_cast<int>(std::floor(rp));
            const double frac = rp - static_cast<double>(i1);

            const int i0 = wrapIndex_(i1 - 1);
            const int i2 = wrapIndex_(i1 + 1);
            const int i3 = wrapIndex_(i1 + 2);

            const double y0 = buffer_[i0];
            const double y1 = buffer_[wrapIndex_(i1)];
            const double y2 = buffer_[i2];
            const double y3 = buffer_[i3];

            // Cubic Hermite (Catmull-Rom style tangents)
            const double c0 = y1;
            const double c1 = 0.5 * (y2 - y0);
            const double c2 = y0 - 2.5 * y1 + 2.0 * y2 - 0.5 * y3;
            const double c3 = 0.5 * (y3 - y0) + 1.5 * (y1 - y2);

            return ((c3 * frac + c2) * frac + c1) * frac + c0;
        }

        int wrapIndex_(int i) const {
            const int n = static_cast<int>(buffer_.size());
            while (i < 0) i += n;
            while (i >= n) i -= n;
            return i;
        }

        void updateDelaySamples_() {
            const double fs = std::max(sampleRate_, 1.0);
            const double fMax = MAX_NYQUIST_RATIO * fs;
            const double f = std::clamp(freqHz_, MIN_FREQ_HZ, fMax);

            // Fundamental comb period
            delaySamples_ = fs / f;

            // Keep inside buffer bounds
            if (!buffer_.empty()) {
                const double maxDelay = static_cast<double>(buffer_.size() - 3);
                delaySamples_ = std::clamp(delaySamples_, 1.0, maxDelay);
            }
        }

        void updateFeedback_() {
            // Map q_ to a musically safe feedback range.
            // This is heuristic (comb "Q" is not a standard Q).
            //
            // Lower Q -> shorter, subtle body resonance
            // Higher Q -> longer, stronger metallic/string-like ring
            const double qNorm = std::clamp((q_ - MIN_Q) / (MAX_Q - MIN_Q), 0.0, 1.0);

            // Start at zero so FEEDBACK=0 is effectively no comb regeneration.
            // Curve pushes more resolution toward the top of the slider.
            double g = MAX_FEEDBACK_GAIN * std::pow(qNorm, FEEDBACK_CURVE);

            g = std::clamp(g, 0.0, MAX_FEEDBACK_GAIN); // hard safety cap
            feedback_ = negativeFeedback_ ? -g : g;
        }

        void updateDamping_() {
            // Map damping01 to one-pole coefficient in feedback path.
            // Higher damping => higher a => darker feedback.
            // Effective LP cutoff roughly from ~12k down to ~800 Hz at 48k (tunable).
            const double fs = std::max(sampleRate_, 1.0);
            const double fc = 12000.0 - damping01_ * (12000.0 - 800.0);
            const double a = std::exp(-2.0 * kPi * fc / fs);
            dampA_ = std::clamp(a, 0.0, 0.9999);
        }

    private:
        double sampleRate_ = DEFAULT_SAMPLE_RATE;

        double freqHz_ = DEFAULT_FREQ_HZ;
        double q_ = MIN_Q;
        double feedback01_ = DEFAULT_FEEDBACK;

        double delaySamples_ = 100.0;
        double feedback_ = 0.5;
        bool   negativeFeedback_ = false;

        double damping01_ = 0.35;
        double dampA_ = 0.0;
        double fbLpState_ = 0.0;

        double mix_ = 1.0;

        // Optional subtle modulation
        double modDepthSamples_ = 0.0;
        double modRateHz_ = 0.0;
        double lfoPhase_ = 0.0;

        std::vector<double> buffer_;
        std::size_t writeIndex_ = 0;
    };

} // namespace pinchfx::dsp
