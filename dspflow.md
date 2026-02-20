# PinchFX DSP Flow

```mermaid
flowchart LR
  INL[In L] --> SUM[Mono Sum]
  INR[In R] --> SUM

  SUM --> PT[PitchTrackerACF]
  PT --> PCTRL[PitchTrackerAlgo]
  PCTRL --> F0[F0 Control]
  F0 --> HSEL[Harmonic Select<br/>5,7,9,12,15]

  SUM --> AGC[AGC]
  AGC --> RES[TwoPoleResonator]
  HSEL --> RES

  RES --> PREWET[Resonator Wet]
  PREWET --> HEAT[Heat Blend<br/>clean vs tube]
  HEAT --> TONE[Tone LP x4]
  TONE --> LIM[Peak Limiter]

  INL --> MIX[Wet/Dry Mix]
  INR --> MIX
  LIM --> MIX
  MIX --> OUTL[Out L]
  MIX --> OUTR[Out R]
```

## Control mapping

- `POSITION`: selects resonator harmonic multiple of tracked F0 (`5, 7, 9, 12, 15`)
- `RES`: sets resonator Q
- `HEAT`: increases tube drive and blends in more tube output
- `TONE`: sets cutoff of the cascaded 4-pole lowpass
- `MIX`: wet/dry crossfade (wet-biased equal-power taper)
- `MONITOR`: output tap selector for debugging

## Notes

- Gate is now analysis-only (confidence indicator), not part of wet audio gating.
- Legacy parameters (`SQUEAL`, `GLIDE`, `MODE`, `SENS`) are retained only for state compatibility.
