// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#pragma once

#include <array>

namespace pinchfx {

// Centralized parameter defaults shared by DSP, VST controller/processor, and simulator UI.
// A/B/C PARTIAL choice list (actual semitone/partial values exposed by the PARTIAL controls).
inline constexpr std::array<int, 8> kDefaultPartialChoices{2, 4, 5, 7, 8, 9, 12, 15};

// A/B/C RES mapping: normalized control [0.0..1.0] -> Q [kResonanceQMin..kResonanceQMax].
inline constexpr double kResonanceQMin = 0.5;
inline constexpr double kResonanceQMax = 8.0;

inline constexpr double kDefaultInput = 0.5; // INPUT, normalized [0.0..1.0]
inline constexpr double kDefaultTrackDelay = 0.25; // TRACK DELAY, normalized [0.0..1.0]
inline constexpr double kDefaultPartialA = 0.0; // A PARTIAL default = 2
inline constexpr double kDefaultPartialB = 2.0 / 7.0; // B PARTIAL default = 5
inline constexpr double kDefaultPartialC = 3.0 / 7.0; // C PARTIAL default = 7
inline constexpr double kDefaultResonanceA = 0.0; // A RES, normalized [0.0..1.0] => Q [0.5..8.0]
inline constexpr double kDefaultResonanceB = 0.0; // B RES, normalized [0.0..1.0] => Q [0.5..8.0]
inline constexpr double kDefaultResonanceC = 0.0; // C RES, normalized [0.0..1.0] => Q [0.5..8.0]
inline constexpr double kDefaultFeedbackA = 0.0; // A FEEDBACK, normalized [0.0..1.0]
inline constexpr double kDefaultFeedbackB = 0.0; // B FEEDBACK, normalized [0.0..1.0]
inline constexpr double kDefaultFeedbackC = 0.0; // C FEEDBACK, normalized [0.0..1.0]
inline constexpr double kDefaultGainA = 1.0; // A GAIN, normalized [0.0..1.0]
inline constexpr double kDefaultGainB = 0.0; // B GAIN, normalized [0.0..1.0]
inline constexpr double kDefaultGainC = 0.0; // C GAIN, normalized [0.0..1.0]
inline constexpr double kDefaultTone = 1.0; // TONE, normalized [0.0..1.0]
inline constexpr double kDefaultHeat = 0.0; // HEAT, normalized [0.0..1.0]
inline constexpr double kDefaultWetDry = 1.0; // WET/DRY, normalized [0.0..1.0]

} // namespace pinchfx
