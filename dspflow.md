# PinchFX DSP Flow

```mermaid
flowchart LR
  INL[In L] --> SUM[Mono Sum]
  INR[In R] --> SUM

  SUM --> PT[PitchTrackerACF]
  PT --> PCTRL[PitchTrackerAlgo]
  PCTRL --> F0[F0 Smoothed<br/>confidence-gated update]

  SUM --> INAGC[Input AGC + Drive Trim]

  F0 --> FH1[Fh1 = F0 * Partial A]
  F0 --> FH2[Fh2 = F0 * Partial B]
  F0 --> FH3[Fh3 = F0 * Partial C]

  INAGC --> R1[Resonator A]
  INAGC --> R2[Resonator B]
  INAGC --> R3[Resonator C]
  FH1 --> R1
  FH2 --> R2
  FH3 --> R3

  R1 --> C1[MiniComb A]
  R2 --> C2[MiniComb B]
  R3 --> C3[MiniComb C]

  C1 --> SUMP[Path Gain Sum]
  C2 --> SUMP
  C3 --> SUMP

  SUM --> WM[Wet Match AGC<br/>target wet ~= dry env]
  SUMP --> WM
  WM --> HEAT[Heat Blend<br/>clean vs tube]
  HEAT --> TONE[Tone LP x4]
  TONE --> LIM[Peak Limiter]

  INL --> MIX[Wet/Dry Mix]
  INR --> MIX
  LIM --> MIX
  MIX --> OUTL[Out L]
  MIX --> OUTR[Out R]
```

## Control mapping

- `INPUT`: AGC drive trim into resonators (`-20 dB .. +20 dB` around unity).
- `TRACK DELAY`: tracker response time constant (slower/faster F0 smoothing).
- `A/B/C PARTIAL`: harmonic multiplier per voice (from `PinchFxPartials.h`: `2, 4, 5, 7, 8, 9, 12, 15`).
- `A/B/C RES`: normalized resonance control (`0..1`) sent to `TwoPoleControlledResonator`.
- `A/B/C FEEDBACK`: per-voice mini-comb feedback amount.
- `A/B/C GAIN`: per-voice gain before summing all wet voices.
- `HEAT`: tube blend amount (clean vs tube stage output).
- `TONE`: cutoff of cascaded 4-pole lowpass (`250 Hz .. 12 kHz`).
- `WET/DRY`: wet/dry equal-power crossfade (with wet-biased taper).

## Notes

- Pitch-confidence gate is analysis-only (scope/debug), not used to hard-gate wet audio.
- Resonator output level is matched to dry envelope by wet-match AGC before HEAT/TONE.
- Limiter is final wet-stage protection before host wet/dry mix.
- Hidden legacy parameter IDs are still kept for state compatibility (older presets/sessions).
