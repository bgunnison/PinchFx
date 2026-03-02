// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#pragma once

#include <algorithm>
#include <cmath>

namespace pinchfx::dsp {

    // TwoPoleResonator (TPT SVF Bandpass) with "Resonance" control in [0..1]
    //
    // Resonance mapping (temporary linear mapping):
    //   0.0 -> Q = 0.5
    //   1.0 -> Q = 5.0
    //
    // Key control features:
    // - Resonance parameter is normalized [0..1] (setQ expects 0..1 now).
    // - Frequency-dependent Q cap via BWmin to prevent ultra-narrow whistle at high fh.
    // - Output AGC limits excessive output at higher Qs.
    // - Optional state leak when gate is off (kills limit cycles / endless ringing).
    //
    // Notes:
    // - No internal sine exciter.
    // - No artificial self-oscillation path.
    // - Keep your fast limiter AFTER this stage.
    class TwoPoleResonator {
    public:
        static constexpr double DEFAULT_SAMPLE_RATE = 44100.0;
        static constexpr double DEFAULT_FREQ = 440.0;

        static constexpr double MIN_FREQ = 1.0;
        static constexpr double MAX_NYQUIST_RATIO = 0.49;
        static constexpr double PI = 3.14159265358979323846;

        // Linear resonance-to-Q map endpoints.
        static constexpr double Q_MIN_LINEAR = 0.5;
        static constexpr double Q_MAX_LINEAR = 5.0;

        // Minimum bandwidth clamp in Hz:
        // BW ~= f / Q  =>  Qcap = f / BWmin
        static constexpr double DEFAULT_BW_MIN_HZ = 35.0;

        // AGC defaults
        static constexpr double DEFAULT_ENV_ATTACK_MS = 2.0;
        static constexpr double DEFAULT_ENV_RELEASE_MS = 80.0;
        static constexpr double DEFAULT_TARGET_LEVEL = 0.35;
        static constexpr double DEFAULT_MAX_GAIN = 8.0;
        static constexpr double DEFAULT_MIN_GAIN = 0.08; // Keep oscillation audible; avoid full AGC mute.
        static constexpr double AGC_ENV_CLAMP = 4.0; // Prevent large spikes from forcing long gain recovery.

        // Input is unity at minimum-Q setting.
        static constexpr double INPUT_Q_NORM_REF = Q_MIN_LINEAR;

        void reset() {
            ic1eq_ = 0.0;
            ic2eq_ = 0.0;
            env_ = 0.0;
            gain_ = 1.0;
        }

        void setSampleRate(double sampleRate) {
            sampleRate_ = (sampleRate > 0.0) ? sampleRate : DEFAULT_SAMPLE_RATE;
            updateAgcCoeffs_();
            updateCoeffs_();
        }

        void setFrequency(double freq) {
            freq_ = std::max(freq, MIN_FREQ);
            updateCoeffs_();
        }

        // Normalized resonance control [0..1]
        void setQ(double resonance01) {
            resonance01_ = std::clamp(resonance01, 0.0, 1.0);
            updateCoeffs_();
        }

        void setResonance(double resonance01) { setQ(resonance01); }

        // Returns current effective Q after internal mapping and bandwidth cap.
        double effectiveQ() const { return qEff_; }

        void setBwMinHz(double bwMinHz) {
            bwMinHz_ = std::max(1.0, bwMinHz);
            updateCoeffs_();
        }

        void setTargetLevel(double target) {
            targetLevel_ = std::clamp(target, 1e-6, 4.0);
        }

        void setAgcTimes(double attackMs, double releaseMs) {
            envAttackMs_ = std::max(0.1, attackMs);
            envReleaseMs_ = std::max(0.1, releaseMs);
            updateAgcCoeffs_();
        }

        void setMaxGain(double maxGain) {
            maxGain_ = std::clamp(maxGain, 0.1, 100.0);
        }

        void setMinGain(double minGain) {
            minGain_ = std::clamp(minGain, 0.0, 1.0);
        }

        // Optional state leak when gate is low to kill limit cycles / long tails
        // leak in (0,1], e.g. 0.9995 gentle, 0.995 aggressive
        void applyLeak(double leak) {
            leak = std::clamp(leak, 0.0, 1.0);
            ic1eq_ *= leak;
            ic2eq_ *= leak;
        }

        // Safe default
        double process(double x) { return process(x, 1.0); }

        // gate01 can be your event envelope. If gate is near zero, states are gently leaked.
        double process(double x, double gate01) {
            if (!std::isfinite(ic1eq_) || !std::isfinite(ic2eq_)) reset();

            gate01 = std::clamp(gate01, 0.0, 1.0);

            if (gate01 < 1e-6) {
                applyLeak(0.995);
            }

            double xIn = x;

            // Q-dependent input normalization at the resonator input.
            // Higher effective Q needs less drive to avoid internal overload.
            const double qNorm = std::max(1.0, qEff_ / INPUT_Q_NORM_REF);
            xIn /= qNorm;

            // TPT SVF bandpass core
            const double v0 = xIn;
            const double v1 = (v0 - ic2eq_ - r_ * ic1eq_) * h_;
            const double v2 = ic1eq_ + g_ * v1;

            if (!std::isfinite(v1) || !std::isfinite(v2)) {
                reset();
                return 0.0;
            }

            ic1eq_ = v2 + g_ * v1;
            ic2eq_ = ic2eq_ + g_ * v2;

            // Envelope follower on raw output
            const double a = std::min(std::abs(v1), AGC_ENV_CLAMP);
            if (a > env_) {
                env_ = envAttack_ * env_ + (1.0 - envAttack_) * a;
            }
            else {
                env_ = envRelease_ * env_ + (1.0 - envRelease_) * a;
            }

            // AGC
            double desiredGain = 1.0;
            if (env_ > targetLevel_) {
                desiredGain = targetLevel_ / env_;
            }
            desiredGain = std::clamp(desiredGain, minGain_, maxGain_);

            // Smooth gain changes: fast down, slower up
            if (desiredGain < gain_) {
                gain_ = gainAttack_ * gain_ + (1.0 - gainAttack_) * desiredGain;
            }
            else {
                gain_ = gainRelease_ * gain_ + (1.0 - gainRelease_) * desiredGain;
            }

            const double yOut = v1 * gain_;
            return std::clamp(yOut, -4.0, 4.0);
        }

    private:
        static double lerp_(double a, double b, double t) {
            t = std::clamp(t, 0.0, 1.0);
            return a + (b - a) * t;
        }

        static double alphaFromMs_(double ms, double fs) {
            const double tauSec = std::max(ms, 0.001) * 0.001;
            return std::exp(-1.0 / (tauSec * std::max(fs, 1.0)));
        }

        // Linear mapping resonance01 -> Quser (before BW cap)
        static double mapResToQ_(double r01) {
            r01 = std::clamp(r01, 0.0, 1.0);
            return lerp_(Q_MIN_LINEAR, Q_MAX_LINEAR, r01);
        }

        void updateAgcCoeffs_() {
            const double fs = std::max(sampleRate_, 1.0);

            envAttack_ = alphaFromMs_(envAttackMs_, fs);
            envRelease_ = alphaFromMs_(envReleaseMs_, fs);

            // Gain smoothing: faster attenuation, slower recovery
            gainAttack_ = alphaFromMs_(1.0, fs);
            gainRelease_ = alphaFromMs_(20.0, fs);
        }

        void updateCoeffs_() {
            // Clamp freq for stability and Nyquist safety
            double f = freq_;
            if (!std::isfinite(f) || f <= 0.0) f = MIN_FREQ;

            const double fs = std::max(sampleRate_, 1.0);
            const double fMax = MAX_NYQUIST_RATIO * fs;
            if (f > fMax) f = fMax;
            freqClamped_ = f;

            // User-mapped Q, then BW cap
            const double Quser = mapResToQ_(resonance01_);
            const double BWmin = std::max(1.0, bwMinHz_);
            const double Qcap = f / BWmin;
            const double Qeff = std::max(0.1, std::min(Quser, Qcap));
            qEff_ = Qeff;

            // SVF coefficients
            const double omega = PI * f / fs;
            g_ = std::tan(omega);
            if (!std::isfinite(g_)) g_ = 0.0;

            r_ = 1.0 / (2.0 * Qeff);
            h_ = 1.0 / (1.0 + g_ * (g_ + r_));
            if (!std::isfinite(h_) || h_ == 0.0) h_ = 1.0;
        }

    private:
        double sampleRate_{ DEFAULT_SAMPLE_RATE };
        double freq_{ DEFAULT_FREQ };
        double freqClamped_{ DEFAULT_FREQ };

        // Normalized 0..1
        double resonance01_{ 0.7 };
        double qEff_{ 1.0 };

        // BW cap
        double bwMinHz_{ DEFAULT_BW_MIN_HZ };

        // SVF coeffs/state
        double g_{ 0.0 };
        double r_{ 0.0 };
        double h_{ 1.0 };
        double ic1eq_{ 0.0 };
        double ic2eq_{ 0.0 };

        // AGC state
        double targetLevel_{ DEFAULT_TARGET_LEVEL };
        double envAttackMs_{ DEFAULT_ENV_ATTACK_MS };
        double envReleaseMs_{ DEFAULT_ENV_RELEASE_MS };
        double maxGain_{ DEFAULT_MAX_GAIN };
        double minGain_{ DEFAULT_MIN_GAIN };

        double env_{ 0.0 };
        double gain_{ 1.0 };

        double envAttack_{ 0.0 };
        double envRelease_{ 0.0 };
        double gainAttack_{ 0.0 };
        double gainRelease_{ 0.0 };
    };

} // namespace pinchfx::dsp
