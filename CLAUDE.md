# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository status

Pre-implementation. The repo currently contains only `mrd.md` (a market requirements document in Chinese). No source code, build system, or tests exist yet. Do not invent build/lint/test commands until the project is scaffolded.

## Project: AeroThrow 3D (云端纸飞机)

A WeChat Mini Game that simulates throwing a paper airplane using the phone itself as the airplane — accelerometer captures the throw impulse, gyroscope locks the launch pitch/roll. Target engine is **Cocos Creator 3.x**. See `mrd.md` for the full spec.

### Key design constraints (from MRD)

These are load-bearing product decisions, not generic preferences. Preserve them when scaffolding code:

- **Holding pose is the input contract.** Phone bottom (mic/charging port) points forward = airplane nose. Screen faces left (for right-handed players); right side of phone is gripped by thumb + index finger. UI in the "ready" stage must depict the plane oriented to match this grip — nose toward the phone's bottom, body on the right edge.
- **Throw detection** = accelerometer burst (back-to-front impulse) + gyroscope sample of pitch/roll at release instant. Sensor sample rate must be ≥60Hz.
- **Physics model is non-negotiable.** Weight, angle of attack, velocity → lift/drag computed in real time. Initial roll angle must affect lateral trajectory (left/right curve). Stall behavior (excessive pitch or insufficient velocity → nose-dive) must be modeled — don't replace with an arcade approximation.
- **Distance metric is the perpendicular** from final landing position to the baseline reference line on the ground (not raw flight path length). Flight HUD shows: live distance, max altitude, flight time, lateral offset. Result screen also shows initial velocity, pitch, roll.
- **Camera** during flight = first-person from the plane, with grass terrain and a baseline line marked with a distance scale.
- **Monetization hook** in current spec: stamina system limiting daily throws, with rewarded-video refill.

### Stage flow

`Ready (plane image matches grip orientation)` → `Throw (vibration on release detected)` → `Flight (first-person, live HUD)` → `Result (full stats)`. Stage 2 in the MRD is intentionally missing — if implementing, ask before inventing one.
