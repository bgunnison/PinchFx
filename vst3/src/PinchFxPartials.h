// Copyright (c) 2026 Brian R. Gunnison
// MIT License
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace pinchfx {

// Edit this one list to change available PARTIAL choices everywhere
// (DSP mapping, controller strings/step count, and sim labels).
inline constexpr std::array<int, 8> kPartialChoices{2, 4, 5, 7, 8, 9, 12, 15};

inline constexpr int kPartialChoiceCount = static_cast<int>(kPartialChoices.size());
inline constexpr int kPartialStepCount = (kPartialChoiceCount > 0) ? (kPartialChoiceCount - 1) : 0;

inline int partialIndexFromNormalized(double normalized) {
    if (kPartialChoiceCount <= 1) return 0;
    const int maxIndex = kPartialChoiceCount - 1;
    const int idx = static_cast<int>(std::round(std::clamp(normalized, 0.0, 1.0) * static_cast<double>(maxIndex)));
    return std::clamp(idx, 0, maxIndex);
}

inline int partialFromNormalized(double normalized) {
    return kPartialChoices[static_cast<std::size_t>(partialIndexFromNormalized(normalized))];
}

inline double normalizedFromPartialIndex(int index) {
    if (kPartialChoiceCount <= 1) return 0.0;
    const int maxIndex = kPartialChoiceCount - 1;
    const int clamped = std::clamp(index, 0, maxIndex);
    return static_cast<double>(clamped) / static_cast<double>(maxIndex);
}

} // namespace pinchfx
