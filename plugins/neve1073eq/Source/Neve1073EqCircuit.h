#pragma once

#include <algorithm>
#include <cmath>
#include <juce_dsp/juce_dsp.h>
#include <even/Ba283OutputStage.h>

//==============================================================================
// Neve 1073 EQ section circuit model (B205 hi/lo shelf board + B211 presence
// board, followed by a BA283-style makeup/output stage).
//
// TOPOLOGY NOTE: this models the original's PASSIVE EQ NETWORK, not generic
// RBJ biquads. The real 1073 EQ is a passive LC/RC sandwich between two
// BA283 gain stages:
//
//   line in -> HPF (switchable, 3rd-order Butterworth: -3 dB at label)
//           -> LF shelf  (passive RC turnover network, 35/60/110/220 Hz)
//           -> MID 1     (passive LCR series resonator, 6-position switch)
//           -> MID 2     (passive LCR series resonator, 6-position switch)
//           -> HF shelf  (passive RC turnover network, fixed 12 kHz)
//           -> iron bandwidth (8 Hz / 40 kHz LO1166 output-transformer
//              coupling filters; linear)
//           -> out trim (applied by the processor)
//
// - The mids are modelled as a SERIES R-L-C resonator driven by the signal.
//   The real B211 pot is a two-position network, and each position is a
//   DIFFERENT component arrangement (this is where the famous Neve boost/cut
//   asymmetry comes from -- it is in the hardware, not a "voicing" trick):
//
//   * BOOST: the LCR branch sits in the feedback path of the BA283 stage.
//     The branch current i = x/Z(f), and the stage adds R_f * i to the
//     output:   y = x + (G-1) * R_d * i.   At resonance Z = R_d (the
//     damping resistor), so the mix gives exactly G. Bell shape, Q fixed
//     by R_d.
//   * CUT:   the LCR branch shunts the signal to ground THROUGH the pot /
//     source resistance R_s -- a passive voltage divider:
//         y/x = Z_branch / (R_s + Z_branch),  Z_branch = R_d + jwL + 1/jwC
//     At resonance the branch is just R_d, so full cut depth is
//     R_d/(R_s+R_d); away from resonance |Z_branch| grows and the dip
//     floors out gently -- a WIDE, shallow-skirted cut (roughly +/-2
//     octaves at -6 dB at full cut) while the boost bell stays narrow
//     (~+/-0.5 octave at -3 dB). Exactly the curve shapes on the original
//     1073 response chart.
//   The pot law for cut is the component one: series resistance
//   R_s(c) = K * R_d * c with c the normalized cut travel (c=1 full cut,
//   c=0 flat), K = 10^(maxCutDb/20) - 1 so full travel gives the rated
//   -18 dB at resonance.
//   L, C, R are component-scale values (henrys / farads / ohms) computed
//   per switch position. The network is integrated with the trapezoidal
//   (zero-delay-feedback) rule: unconditionally stable and frequency-
//   accurate at 44.1-192 kHz.
// - The shelves are modelled as the B205's RC turnover ladders: TWO
//   cascaded one-pole sections at the corner (12 dB/oct settle) mixed
//   against the dry signal by the gain pot. That gives the hardware's
//   true shelf -- a flat plateau on one side of the corner that SETTLES
//   back to unity on the other -- not a single 6 dB/oct skirt that never
//   flattens. LF corner sits at 1.4x the switch frequency so the plateau
//   is fully established AT the switch frequency (full boost by the
//   selected frequency, settled ~2 octaves above); HF corner at 12 kHz/1.4.
// - The HPF is a 3rd-order Butterworth highpass built the analogue way:
//   one real pole (bilinear one-pole) cascaded with a TPT state-variable
//   2-pole at Q = 1, all cornered at the switch frequency. That gives
//   exactly -3 dB AT the labelled frequency (matching the original CR
//   ladder) with a 18 dB/oct asymptotic skirt. (The previous "3 real
//   poles at the corner" ladder read -9 dB at the label -- wrong.)
//
// The BA283-style makeup stage is the REAL component-level BA283 (shared
// plugins/common/even/Ba283OutputStage.h, extracted from the neve1073 preamp
// plugin): Ebers-Moll driver + Class-AB pair + global NFB into the driver
// base. It is LEVEL-ANCHORED and SMALL-SIGNAL NORMALIZED:
//
//   - 0 dBu (the console's nominal line level) is defined as -18 dBFS, i.e.
//     a full-scale-referenced amplitude of 10^(-18/20) ~ 0.126. The network
//     output is scaled by ba283DriveScale into the stage's driver-base node;
//     2.0 puts a nominal 0 dBu sine at ~0.25 V of drive -- comfortably in
//     the stage's near-Class-A region, with saturation only on peaks well
//     above nominal.
//   - prepare() calibrates a normalization constant so the stage's
//     SMALL-SIGNAL gain is exactly unity: at low levels the whole plugin is
//     linear (response = arithmetic sum of the band curves), and the
//     saturation only becomes audible as level rises. This mirrors the
//     preamp plugin's normalization calibration.
//   - The nonlinear path runs per-sample Newton solves and is oversampled
//     2x in the processor (same scheme as the preamp plugin).
//
// The 8 Hz / 40 kHz "iron bandwidth" coupling filters of the LO1166 output
// transformer path remain in the chain after the stage.
class Neve1073EqCircuit {
public:
  // Fixed hardware constants for the iron/coupling color stage.
  static constexpr double ironHpHz = 8.0;
  static constexpr double ironLpHz = 40000.0;

  // Level anchor: volts of BA283 driver-base drive per unit of network
  // output. 0 dBu == -18 dBFS (amplitude 0.1259); 2.0 puts that nominal
  // level at 0.25 V of drive -- near-Class-A, peaks only above that
  // saturate (see the class comment).
  static constexpr double ba283DriveScale = 2.0;

  // Mid resonator Q, set by the damping resistor of the LCR network (the
  // original's "fixed Q" mid; ~1.1 matches published curve plots).
  static constexpr double midQ = 1.1;
  // Damping capacitor for the LCR sections (component-scale value).
  static constexpr double midC = 100.0e-9;

  // Rated maximum cut at full pot travel (same as boost, per AMS spec).
  // Sets the shunt-divider series resistance: K = 10^(maxCutDb/20) - 1.
  static constexpr double maxCutDb = 18.0;
  // Shelf ladder: corner multiplier. The LF plateau must be essentially
  // fully established AT the switch frequency, so the RC corner sits 2x
  // above it (the 2-pole ladder is -1 dB at the switch frequency, flat an
  // octave below, and settles back to unity by ~2 octaves above -- the
  // hardware shelf does NOT keep falling 6 dB/oct forever); the HF shelf
  // corner is 12 kHz / 2.
  static constexpr double shelfCornerMult = 2.0;

  struct BandSettings {
    // Switch positions (fixed hardware frequencies).
    double hpfF = 0.0;     // 50/80/160/300 Hz switch; 0 = HPF off
    double lfF = 60.0;     // 35/60/110/220 Hz switch
    double mid1F = 700.0;  // 0.36/0.7/1.6/3.2/4.8/7.2 kHz switch
    double mid2F = 3200.0; // same 6-position switch as MID 1
    // Boost/cut pots: linear gain (1.0 = flat). Shelves +/-16 dB,
    // mids +/-18 dB max (enforced by the parameter ranges).
    double lfGain = 1.0;
    double mid1Gain = 1.0;
    double mid2Gain = 1.0;
    double hfGain = 1.0;
  };

  void prepare(double sampleRate) {
    fs = sampleRate;
    // Coupling filter coefficients are fixed (8 Hz HP / 40 kHz LP -- the
    // "iron bandwidth" of the LO1166 output transformer path). These use
    // implicit one-poles (a = 1 - e^{-wT}), not RBJ IIRs, because a
    // 40 kHz corner is up at the Nyquist shoulder where the bilinear
    // tan(w/2) design goes unstable (fs/4 crossing).
    ironHpA = 1.0 - std::exp(-2.0 * M_PI * ironHpHz / fs);
    ironLpA = 1.0 - std::exp(-2.0 * M_PI * std::min(ironLpHz, 0.45 * fs) / fs);
    settingsDirty = true;
    retune();

    // ---- BA283 output stage: DC operating point + normalization ----------
    // Same pattern as the preamp plugin. The feedback path is AC-coupled
    // (output transformer), referenced at the output node's own quiescent
    // DC; that reference depends on the DC solve and vice versa (only
    // weakly), so a fixed-point iteration from an analytic first guess
    // converges in a couple of passes.
    ba283.coldStart();
    ba283.nfbRef = 12.7;
    for (int pass = 0; pass < 6; ++pass) {
      ba283Seed = { 1.5, 0.85, 13.0, 12.5 }; // rough hand estimate
      ba283.solve(0.0, ba283Seed);
      const double prev = ba283.nfbRef;
      ba283.nfbRef = ba283Seed[3];
      if (std::abs(ba283Seed[3] - prev) < 1.0e-9)
        break;
    }
    ba283.quiescentSeed = ba283Seed; // physical fallback seed
    ba283QuiescentVo = ba283Seed[3];

    // Small-signal normalization: run a quiet sine through the stage alone
    // (Live solves, exactly as the audio path does) and scale so the
    // stage's small-signal gain is exactly unity. Calibration amplitude is
    // deliberately small (well inside the linear region); the drive scale
    // (ba283DriveScale) -- not this trim -- is the level anchor.
    ba283Norm = 1.0;
    {
      const double amp = 0.05, freq = 100.0;
      const int n = (int)(fs * 0.05);
      double peak = 0.0;
      for (int i = 0; i < n; ++i) {
        const double d = amp * std::sin(2.0 * M_PI * freq * i / fs) * ba283DriveScale;
        const auto out = ba283.solve(d, ba283Seed);
        if (i > n / 2)
          peak = std::max(peak, std::abs(out.o - ba283QuiescentVo));
      }
      if (peak > 1.0e-6)
        // peak corresponds to drive amplitude amp*driveScale*gain_ss; the
        // trim must make the NETWORK -> output path unity, so it is
        // amp/peak (the driveScale cancels: unity THROUGH the stage
        // including the input scaling).
        ba283Norm = amp / peak;
    }

    // Reset continuation states so playback starts from the DC bias.
    ba283Seed = ba283.quiescentSeed;
    ba283.coldStart();
  }

  void reset() {
    mid1 = {};
    mid2 = {};
    hpfZ1 = 0.0;         // real-pole one-pole state
    svfS = 0.0;          // TPT SVF (band) state; low state derived below
    svfS2 = 0.0;
    lfLpZ1 = lfLpZ2 = hfLpZ1 = hfLpZ2 = 0.0;
    ironLpZ = ironHpZ = 0.0;
    ba283Seed = ba283.quiescentSeed;
  }

  // Called once per audio block: retune the passive network to the
  // (smoothed) settings. Recomputing L/R values is trivial; the smoothing
  // in the processor keeps pot moves zipper-free.
  void setSettings(const BandSettings &s) {
    settings = s;
    settingsDirty = true;
  }

  void process(float *x, int n) {
    if (settingsDirty) {
      retune();
      settingsDirty = false;
    }

    const double lfMix = settings.lfGain - 1.0;
    const double m1Mix = settings.mid1Gain - 1.0;
    const double m2Mix = settings.mid2Gain - 1.0;
    const double hfMix = settings.hfGain - 1.0;

    for (int i = 0; i < n; ++i) {
      double y = x[i];

      // ---- HPF: 3rd-order Butterworth -------------------------------
      // Real pole (bilinear TPT one-pole highpass) -> TPT SVF 2-pole, Q=1.
      if (hpfG > 0.0) {
        double v = hpfG * (y - hpfZ1);
        const double lp = hpfZ1 + v;
        hpfZ1 = lp + v;
        y -= lp;
        // TPT SVF (Zavalishin): v3 = highpass, k = 1/Q = 1.
        const double v3 = (y - svfS - svfS2) * hpfSvfA;
        const double v1 = hpfSvfG * v3 + svfS;
        const double v2 = hpfSvfG * v1 + svfS2;
        svfS = 2.0 * v1 - svfS;
        svfS2 = 2.0 * v2 - svfS2;
        y = v3;
      }

      // ---- LF shelf (B205): 2-pole RC ladder + pot mix --------------
      // TPT one-pole: v = G*(x - z); y = z + v; z_new = y + v.
      double v = lfLpG * (y - lfLpZ1);
      const double lp1 = lfLpZ1 + v;
      lfLpZ1 = lp1 + v;
      v = lfLpG * (lp1 - lfLpZ2);
      const double lp2 = lfLpZ2 + v;
      lfLpZ2 = lp2 + v;
      y += lfMix * lp2;

      // ---- MID 1 / MID 2 (B211): passive LCR resonators -------------
      y = stepLcr(mid1, y, m1Mix);
      y = stepLcr(mid2, y, m2Mix);

      // ---- HF shelf (B205), fixed 12 kHz plateau ---------------------
      v = hfLpG * (y - hfLpZ1);
      const double hfLp1 = hfLpZ1 + v;
      hfLpZ1 = hfLp1 + v;
      const double hfHp1 = y - hfLp1;
      v = hfLpG * (hfHp1 - hfLpZ2);
      const double hfLp2 = hfLpZ2 + v;
      hfLpZ2 = hfLp2 + v;
      y += hfMix * (hfHp1 - hfLp2);

      // ---- BA283 output stage (level-anchored saturation) -----------
      // Per-sample Newton solves of the shared BA283 (see class comment):
      // small-signal unity by calibration, drive anchored so 0 dBu
      // (-18 dBFS) sits near Class A.
      {
        const double d = y * ba283DriveScale;
        const auto out = ba283.solve(d, ba283Seed);
        y = (out.o - ba283QuiescentVo) * ba283Norm;
      }

      // ---- LO1166 "iron bandwidth" coupling filters -------------------
      ironLpZ += ironLpA * (y - ironLpZ); // HF iron loss
      ironHpZ += ironHpA * (y - ironHpZ); // LF coupling / DC
      double out = ironLpZ - ironHpZ;

      x[i] = (float)out;
    }
  }

  // Debug helper (tests only): one sample through every stage, printing
  // the value after each stage.
  double debugStage(float in) {
    double y = in;
    if (hpfG > 0.0) {
      double v = hpfG * (y - hpfZ1);
      const double lp = hpfZ1 + v;
      hpfZ1 = lp + v;
      y -= lp;
      const double v3 = (y - svfS - svfS2) * hpfSvfA;
      const double v1 = hpfSvfG * v3 + svfS;
      const double v2 = hpfSvfG * v1 + svfS2;
      svfS = 2.0 * v1 - svfS;
      svfS2 = 2.0 * v2 - svfS2;
      y = v3;
    }
    double y0 = y;
    double v = lfLpG * (y - lfLpZ1);
    const double lp1 = lfLpZ1 + v;
    lfLpZ1 = lp1 + v;
    v = lfLpG * (lp1 - lfLpZ2);
    const double lp2 = lfLpZ2 + v;
    lfLpZ2 = lp2 + v;
    y += (settings.lfGain - 1.0) * lp2;
    double y1 = y;
    y = stepLcr(mid1, y, settings.mid1Gain - 1.0);
    double y2 = y;
    y = stepLcr(mid2, y, settings.mid2Gain - 1.0);
    double y3 = y;
    v = hfLpG * (y - hfLpZ1);
    const double hfLp1 = hfLpZ1 + v;
    hfLpZ1 = hfLp1 + v;
    const double hp1 = y - hfLp1;
    v = hfLpG * (hp1 - hfLpZ2);
    const double hfLp2 = hfLpZ2 + v;
    hfLpZ2 = hfLp2 + v;
    y += (settings.hfGain - 1.0) * (hp1 - hfLp2);
    {
      const double d = y * ba283DriveScale;
      const auto out = ba283.solve(d, ba283Seed);
      y = (out.o - ba283QuiescentVo) * ba283Norm;
    }
    ironLpZ += ironLpA * (y - ironLpZ);
    ironHpZ += ironHpA * (y - ironHpZ);
    double out = ironLpZ - ironHpZ;
    std::printf("in=%g hpf=%g lf=%g m1=%g m2=%g hf=%g out=%g\n", (double)in,
                y0, y1, y2, y3, y, out);
    return out;
  }

  // Shared BA283 output stage + its Newton continuation seed (public: the
  // test harness probes the stage mapping directly).
  even::Ba283OutputStage ba283;
  even::Ba283OutputStage::Vec ba283Seed {};
  double ba283Norm = 1.0;        // small-signal unity trim (prepare calibration)
  double ba283QuiescentVo = 0.0; // output node quiescent DC

private:
  // Passive series-LCR resonator, trapezoidal (zero-delay) integration.
  // States: inductor current i, capacitor voltage vC, previous inductor
  // voltage vL. The mix sign selects the pot position:
  //   mix >= 0 : BOOST  -- LCR in the BA283 feedback path; output adds
  //              mix * (R * i) (the damping-resistor voltage, == G at
  //              resonance since Z = R there).
  //   mix <  0 : CUT    -- LCR shunts the signal to ground through the
  //              pot/source resistance cutRs (passive divider, wide skirt).
  struct LcrResonator {
    double L = 1.0, R = 1.0; // C is the shared constant midC
    double cutRs = 0.0;      // shunt-path series resistance (cut)
    double i = 0.0, vC = 0.0, vL = 0.0;
    double a = 0.0, b = 0.0; // trapezoidal integrator gains
  };

  static double stepLcr(LcrResonator &r, double x, double mix) {
    if (mix >= 0.0) {
      // BOOST: voltage x across the series R-L-C; trapezoidal rules:
      //   i_n = i + a*(vL_n + vL),  vC_n = vC + b*(i_n + i)
      // KVL: vL_n = x - R*i_n - vC_n. Solving for i_n:
      const double iN = (r.i * (1.0 - r.a * r.b) + r.a * (x - r.vC + r.vL)) /
                        (1.0 + r.a * r.R + r.a * r.b);
      const double vCN = r.vC + r.b * (iN + r.i);
      const double vLN = x - r.R * iN - vCN;
      const double vR = r.R * iN;
      r.i = iN;
      r.vC = vCN;
      r.vL = vLN;
      return x + mix * vR;
    }

    // CUT: series (R + Rs)-L-C shunt to ground; the output node sits
    // before Rs, so KVL gives vL_n = x - (R+Rs)*i_n - vC_n and the
    // divider output is x - Rs*i_n = Z_branch/(Rs+Z_branch) * x.
    const double rTot = r.R + r.cutRs;
    const double iN = (r.i * (1.0 - r.a * r.b) + r.a * (x - r.vC + r.vL)) /
                      (1.0 + r.a * rTot + r.a * r.b);
    const double vCN = r.vC + r.b * (iN + r.i);
    const double vLN = x - rTot * iN - vCN;
    r.i = iN;
    r.vC = vCN;
    r.vL = vLN;
    return x - r.cutRs * iN;
  }

  void retune() {
    // Shelf ladder corners: LF at 2x the switch frequency (plateau
    // fully established AT the switch frequency), HF at 12 kHz / 2.
    // TPT one-pole gain G = g/(1+g), g = tan(pi*f/fs): the exact
    // trapezoidal one-pole, so the 12 kHz pole is corner-accurate even
    // at 48 kHz (the implicit 1-exp rule sags ~4 dB there).
    const double lfPole = settings.lfF * shelfCornerMult;
    const double hfPole = 12000.0 / shelfCornerMult;
    const double lfG = std::tan(M_PI * lfPole / fs);
    const double hfG = std::tan(M_PI * hfPole / fs);
    lfLpG = lfG / (1.0 + lfG);
    hfLpG = hfG / (1.0 + hfG);
    // HPF: 3rd-order Butterworth. Pole Qs for N=3 are 1.0 (complex pair)
    // and a single real pole, all at the switch frequency -> -3 dB at the
    // label. Real pole: bilinear TPT one-pole gain G = g/(1+g). SVF:
    // g = tan(pi*f/fs), k = 1/Q = 1.
    const double g = std::tan(M_PI * settings.hpfF / fs);
    hpfG = settings.hpfF > 0.0 ? g / (1.0 + g) : 0.0;
    hpfSvfG = settings.hpfF > 0.0 ? g : 0.0;  // k = 1/Q = 1
    hpfSvfA = settings.hpfF > 0.0 ? 1.0 / (1.0 + g * (g + 1.0)) : 0.0;
    // LCR component values per switch position (and pot position).
    tuneLcr(mid1, settings.mid1F, settings.mid1Gain);
    tuneLcr(mid2, settings.mid2F, settings.mid2Gain);
  }

  void tuneLcr(LcrResonator &r, double f, double gain) {
    // Choose L so that f0 = 1/(2*pi*sqrt(L*C)); R sets Q = sqrt(L/C)/R.
    const double w = 2.0 * M_PI * f;
    r.L = 1.0 / (w * w * midC);
    r.R = std::sqrt(r.L / midC) / midQ;
    r.a = 1.0 / (2.0 * fs * r.L);  // Ts/(2L)
    r.b = 1.0 / (2.0 * fs * midC); // Ts/(2C)
    // Cut pot: series resistance R_s = K * R_d * c, c = normalized cut
    // travel. Full travel (gain = 10^(-maxCutDb/20)) gives depth
    // R_d/(R_s+R_d) = 1/(1+K) = -maxCutDb at resonance; the wiper
    // towards the top adds series R (shallower cut), like the hardware.
    const double gMin = std::pow(10.0, -maxCutDb / 20.0);
    const double c = std::clamp((1.0 - gain) / (1.0 - gMin), 0.0, 1.0);
    r.cutRs = (std::pow(10.0, maxCutDb / 20.0) - 1.0) * r.R * c;
  }

  double fs = 48000.0;
  BandSettings settings;
  bool settingsDirty = true;

  LcrResonator mid1, mid2;

  // HPF (3rd-order Butterworth) + shelf ladder + iron coupling states.
  double hpfG = 0.0, hpfZ1 = 0.0; // real pole: TPT one-pole gain + state
  double hpfSvfG = 0.0;           // TPT SVF gain (Q = 1)
  double hpfSvfA = 0.0;           // TPT SVF normalizer 1/(1+g(g+k))
  double svfS = 0.0, svfS2 = 0.0; // SVF band/low states
  double lfLpG = 0.0, hfLpG = 0.0; // TPT one-pole gains
  double lfLpZ1 = 0.0, lfLpZ2 = 0.0, hfLpZ1 = 0.0, hfLpZ2 = 0.0;
  double ironHpA = 0.0, ironLpA = 0.0, ironHpZ = 0.0, ironLpZ = 0.0;
};
