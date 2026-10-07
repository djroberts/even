~~# Neve 1073 EQ — AI context & implementation brief

> Purpose: give any AI (or human) full context so this EQ plugin never has to be
> researched or designed from scratch again. Companion to `docs/review.txt`
> (review of the preamp plugin, `plugins/neve1073`).

## What this is

Context document for building a **separate plugin** (`plugins/neve1073eq`) that
models the **EQ section of the Neve 1073**, standalone from the preamp plugin
(`plugins/neve1073`, which models the full preamp: LO1166 input transformer,
Class-A BA215/BA284-style stage, BA283 output stage).

Key architectural fact: **the EQ does not need the preamp, and needs no input
transformer.** In the original console, the EQ sat *after* the fader (line level),
not inside the mic amp. It only requires: the switched LCR network + one BA283
makeup/output stage. The output transformer (LO1166) is optional color, not a
functional requirement.

## Naming

- The EQ never received its own product number — it is simply "the 1073 EQ".
  AMS Neve's standalone hardware unit is the **1073EQ**.
- Original physical boards inside the 1073 module:
  - **B205** board, drawing **D10042** — HI/LO frequency EQ (LF + HF shelves)~~
  - **B211** board — presence/mid EQ (LMF + HMF bells)
  - **B182** board, drawing **D10019 C/D** — high-pass filter (optional in scope)
  - **BA283** amplifier board — post-EQ makeup/output stage
- Plugin id: `neve1073eq`, display name "Neve 1073 EQ".

## Signal-flow position in the original module

```
Mic in -> LO1166 input xfmr -> sensitivity -> Class-A stage -> fader
       -> [ B205/B211 EQ network -> BA283 makeup/output amp -> LO1166 out xfmr ] -> bus
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
           this bracket = what the neve1073eq plugin models
```

The EQ is **not** a passive insert: it is an **active, inductor-based (LCR) EQ**
where switched L-C resonant networks sit in the feedback/shunt paths of
transistor stages. The passive network alone is deeply lossy (~15–20 dB insertion
loss at full boost) — the surrounding amplifiers are integral to the design.

- **Inductors are the defining component** (St. Ives / original Marinair; modern
  Sowter equivalents). They are mandatory to the sound.
- **Input transformer**: preamp-only component; a line-in EQ needs none.
- **Output transformer**: optional for the plugin; its saturation/hysteresis
  color should be shared with the existing `TransformerModel` work in
  `plugins/common/even/`.

## Controls & frequency table (fixed, stepped like the original)

| Control        | Positions                              | Gain                        |
|----------------|----------------------------------------|-----------------------------|
| EQ IN          | button                                 | —                           |
| LF shelf       | 60 / 110 Hz                            | ±18 dB                      |
| LMF bell       | 220 / 320 / 450 / 680 / 720 Hz         | ±18 dB, continuous pot      |
| HMF bell       | 1.6 / 3.2 / 4.8 / 7.2 kHz              | ±18 dB, continuous pot      |
| HF shelf       | 12 kHz (older units 10/12k switch)     | ±18 dB, continuous pot      |
| HPF (B182)     | off / 50 / 70 / 120 / 220 / 320 Hz     | ~18 dB/oct                  |

The shelving bands' fixed corner frequencies are set by the L-C pairs; the
mid bands' frequency switch retunes the tapped inductor + capacitor selection.

## Reference schematics (primary sources)

- **Full official service pack** (module schematic EH10023, sensitivity switch
  EK20033, BA283 amplifier board ×5 pages, BA284 board, B182 high-pass filter
  D10019C/D, **B205 HI AND LOW FREQUENCY EQ BOARD D10042**, **B211 presence/mid
  EQ board**):
  https://archive.org/details/neve_1073-fullpak
- Single-sheet channel amplifier schematic EH10023:
  https://archive.org/details/neve_1073_channel_amplifer_schematic_EH10023
- Community: groupdiy.com — Neve 1073 EQ threads, Sowter/St. Ives inductor
  measurements and substitution data.
- Gyraf Audio's free Neve-style EQ project (measured inductor values, hobbyist-
  verified L/C/R part values).

## DSP modeling approach (consistent with this repo's philosophy)

This repo models circuits at component level (MNA + Newton-Raphson,
`even::BjtModel`, `even::NewtonSolver`, `even::TransformerModel` — see
`plugins/common/even/`). Plan for the EQ:

1. **LCR network as physics, not generic biquads.** Each band is a series
   L-C(-R) resonant branch. Model each branch as a 2nd-order state-space /
   TPT (Zavalishin topology-preserving transform) resonator so a frequency
   switch retunes coefficients without clicks, mirroring how the original
   switch changes taps/wiring.
2. **Bell bands (LMF/HMF)**: series L-C branch in the feedback path of the
   stage around it — boost = deeper resonance in feedback; cut = branch
   shunted across the signal path. This gives the authentic proportional-Q
   (Q rises with boost) behavior.
3. **Shelf bands (LF/HF)**: series-LC in the feedback path around the makeup
   stage (HF) / across the signal path (LF), giving the characteristic gentle
   slope and overshoot-free shoulders.
4. **Nonlinearity / color**: the EQ's "warmth" comes from the BA283 makeup
   stage operating near Class A at typical levels. Reuse the preamp plugin's
   BA283-style push-pull model (already implemented in
   `plugins/neve1073/Source/Neve1073Circuit.h`) as the post-EQ stage; the EQ
   bandpassing itself is linear.
5. **Gain staging**: original EQ is inserted at roughly −10 to 0 dBu nominal;
   drive into the BA283 stage is a modeling choice, expose as an input trim.
6. **Oversampling**: the linear EQ needs none; the nonlinear makeup stage
   reuses the preamp's 2× oversampling approach.
7. **Switched inductance**: real inductors are tapped coils; a switch tap
   equals a scaled L — model as one L with scaled tap values (document that
   this matches the original wiring).

## Suggested folder layout (mirrors plugins/neve1073)

```
plugins/neve1073eq/CMakeLists.txt
plugins/neve1073eq/Source/PluginProcessor.{h,cpp}
plugins/neve1073eq/Source/PluginEditor.{h,cpp}
plugins/neve1073eq/Source/Neve1073EqCircuit.h   // B205+B211 network + BA283 stage
plugins/neve1073eq/Source/tests/
```

## Parameter set (APVS ids)

- `eqIn` (bool)
- `lfFreq` (Choice: 60, 110), `lfGain` (±18 dB)
- `lmfFreq` (Choice: 220/320/450/680/720), `lmfGain` (continuous ±18)
- `hmfFreq` (Choice: 1.6k/3.2k/4.8k/7.2k), `hmfGain` (continuous ±18)
- `hfFreq` (fixed 12 kHz; optional 10/12k choice), `hfGain` (continuous ±18)
- `hpf` (Choice: off/50/70/120/220/320) — if HPF is in scope
- `outTrim`

## Open decisions

- Include the B182 high-pass filter, or keep the plugin strictly EQ?
- Gain steps: some original bands are click-stepped, others continuous —
  decide stepped vs. continuous for the shelf bands.
- Extract the shared BA283 model code from `plugins/neve1073` into
  `plugins/common/even/` (recommended — avoid duplicating the nonlinear stage).


## Implementation status (updated after build)

The plugin is implemented in `plugins/neve1073eq/` and **deliberately deviates
from this brief in one respect**: it ships **two mid bands** (both with the
identical 6-position frequency switch 0.36/0.7/1.6/3.2/4.8/7.2 kHz), a
"1073-plus" layout the user explicitly requested ("software but no one makes a
1073 with two mid bands"). Everything else follows the real 1073:

- **Bands**: HPF (3rd-order Butterworth, -3 dB at the switch label,
  50/80/160/300 Hz, on/off) -> LF shelf
  (B205 RC ladder, 35/60/110/220 Hz, +/-16 dB) -> MID 1 (+/-18 dB) ->
  MID 2 (+/-18 dB) -> HF shelf (fixed 12 kHz plateau, +/-16 dB) ->
  BA283 output stage (see below) ->
  8 Hz/40 kHz iron bandwidth (LO1166 output-transformer coupling; linear)
  -> output trim. EQ IN button is a hard-wired network lift.
- **BA283 makeup stage: LIVE, shared with the preamp plugin.** The full
  component-level model (Ebers-Moll driver, Class-AB pair, spreader, global
  NFB into the driver base, collector-saturation floor, 4-node Newton solve)
  was extracted from `plugins/neve1073` into
  `plugins/common/even/Ba283OutputStage.h`; both plugins instantiate the
  identical class, so chaining preamp -> EQ no longer doubles the stage.
  - *Level anchor*: 0 dBu == -18 dBFS. The network output is scaled by
    `ba283DriveScale = 2.0` into the driver base, putting a nominal 0 dBu
    sine (~0.25 V of drive) in the stage's near-Class-A region; saturation
    engages only on peaks well above nominal.
  - *Small-signal normalized*: `prepare()` calibrates a trim so the stage's
    small-signal gain is exactly unity -- at low levels the plugin measures
    the arithmetic sum of the band curves (two stacked +18 dB mids read
    +35.98 dB at -40 and -20 dBFS); at high drive the stage compresses and
    generates the signature even-order harmonic (full-scale into the +36 dB
    stack reads ~+29.9 dB, heavily saturated but bounded -- regression-tested).
  - The nonlinear path runs per-sample Live Newton solves. 2x oversampling
    of the nonlinear path (as in the preamp) is a planned follow-up.
- **Topology (important)**: NOT RBJ biquads, and NOT symmetric. The mids are
  passive series R-L-C resonators integrated with the trapezoidal
  (zero-delay-feedback) rule, and the boost/cut pot selects two DIFFERENT
  component arrangements — the source of the authentic 1073 boost/cut
  asymmetry:
  - *Boost*: LCR in the BA283 feedback path; `y = x + (G-1)*R_d*i`
    (at resonance Z = R_d so mix gives exactly G). Narrow bell, Q = 1.1.
  - *Cut*: LCR shunting the signal to ground through the pot/source
    resistance R_s = K*R_d*c (passive divider
    `y/x = Z_branch/(R_s + Z_branch)`, K = 10^(18/20)-1 so full travel is
    -18 dB at resonance). The divider gives the wide, shallow-skirted Neve
    cut (~±2 octaves at -6 dB at full cut) while boost stays narrow.
  L/C/R use component-scale values (C = 100 nF, L from f0, R from Q = 1.1).
  Shelves are two cascaded TPT one-pole sections at the corner (12 dB/oct
  settle) mixed by the pot — a true shelf with flat plateaus on both sides,
  NOT a single 6 dB/oct skirt that never flattens. The LF corner sits 2x
  above the switch frequency so the plateau is fully established AT the
  switch frequency (the "full boost at 60" kick recipe); the HF corner is
  12 kHz/2. Shelf one-poles use exact trapezoidal TPT gains
  (G = g/(1+g), g = tan(pi*f/fs)) so the 12 kHz pole is corner-accurate at
  48 kHz.
- **Fixed Q = 1.1** for both mid boosts (AMS spec says "fixed Q"; value
  chosen to match published curve plots; ~±0.5 octave -3 dB at full boost).
  Cuts are much wider than boosts (see topology above) — that asymmetry is
  in the hardware, by design.
- **Gain pots**: continuous 0.5 dB steps (hardware is stepped; open decision).
- **Tests**: `neve1073eq_tests` CMake target measures steady-state sine gain
  through the whole circuit; run `./build/neve1073eq/neve1073eq_tests`.
  `neve1073eq_tests sweep` dumps frequency-response tables (10 points/octave)
  for LF boost @60, mid boost/cut @700 and HF boost, for pasting into EQ
  Curve Analyzer and comparing against reference curves (e.g. Lindell 80).
  Note: at low levels (-24 dBFS and below) the BA283 is small-signal
  normalized to unity, so readings match the arithmetic sum of band gains
  (two stacked +18 dB mids read +36 dB); at high drive the output stage
  saturates and the sum compresses.

## Chaining with the preamp plugin

Both plugins now instantiate the SAME `even::Ba283OutputStage`
(`plugins/common/even/Ba283OutputStage.h`). In the original console there is
exactly ONE BA283 per channel (post-EQ); chaining `neve1073` -> `neve1073eq`
therefore runs the stage twice and doubles its saturation and iron color.
That is acceptable as two standalone plugins, and the planned follow-up is a
combined `neve1073channel` composition plugin (in xfmr -> sensitivity ->
fader -> EQ -> one BA283 -> out xfmr) rather than merging the two plugins.

## Measuring with EQ Curve Analyzer (important caveat)

The mid cut depth verified against the numeric ground truth is **-18 dB**
(clean 360 Hz sine through the full circuit, measured -18.07 dB at input
amplitudes 0.001/0.25/1.0 -- ground truth taken while the makeup stage was
bypassed; at 0 dBFS the now-live stage fills the notch in, by design). If
the analyzer's **Impulse mode** shows the
notch shallower (historically ~-12 dB), that is a **measurement artifact,
not an EQ error**: impulse deconvolution assumes an LTI system, and any
output-stage nonlinearity generates harmonic/IM distortion that lands
*inside* the notch and fills it in (the analyzer cannot distinguish
passed-through signal from distortion-generated signal). The BA283 makeup
stage is now LIVE (shared, level-anchored -- see Implementation status), so
impulse mode WILL read deep notches shallower at hot levels: verify deep
notches with a sine at -24/-30 dBFS (where the stage is small-signal
linear), or drop the impulse level accordingly.
