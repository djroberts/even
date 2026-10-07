#include "../../Source/Neve1073EqCircuit.h"

#include <cmath>
#include <cstdio>
#include <cstring>

using S = Neve1073EqCircuit::BandSettings;

// Measures the steady-state gain (dB) of the circuit for a sine at `f`,
// averaged over the second half of a 1-second run. Fails loudly (prints and
// returns -999) on any non-finite sample, so instability cannot hide.
static double measureGainDb(double f, const S &s, double fs = 48000.0,
                            double amp = 0.1) {
  Neve1073EqCircuit e;
  e.prepare(fs);
  e.setSettings(s);

  const int n = (int)fs;
  double sum = 0.0;
  for (int i = 0; i < n; ++i) {
    float x = (float)std::sin(2.0 * M_PI * f * i / fs) * (float)amp;
    e.process(&x, 1);
    if (!std::isfinite(x)) {
      std::printf("NON-FINITE output at sample %d (f=%g)\n", i, f);
      return -999.0;
    }
    if (i > n / 2)
      sum += (double)x * x;
  }
  return 10.0 * std::log10(2.0 * sum / (n / 2.0) / (amp * amp));
}

static int failures = 0;
static void expectNear(const char *what, double got, double want,
                       double tol = 1.5) {
  const bool ok = std::abs(got - want) <= tol;
  std::printf("%s %-28s got %+7.2f dB, want %+7.2f dB\n", ok ? "PASS" : "FAIL",
              what, got, want);
  if (!ok)
    ++failures;
}

int main(int argc, char **argv) {
  // `neve1073eq_tests sweep` dumps a frequency-response table (10 points
  // per octave) for the classic recipes, for pasting into EQ Curve
  // Analyzer and comparing against reference curves (e.g. Lindell 80).
  if (argc > 1 && std::strcmp(argv[1], "sweep") == 0) {
    const char *names[] = {"LF+16@60", "MID1+18@700", "MID1-18@700", "HF+16"};
    S cfgs[4];
    cfgs[0].lfF = 60.0;
    cfgs[0].lfGain = 6.3096;
    cfgs[1].mid1F = 700.0;
    cfgs[1].mid1Gain = 7.9433;
    cfgs[2].mid1F = 700.0;
    cfgs[2].mid1Gain = 0.1259;
    cfgs[3].hfGain = 6.3096;
    for (int c = 0; c < 4; ++c) {
      std::printf("# %s\nfreq_Hz\tgain_dB\n", names[c]);
      for (double oct = -2.0; oct <= 10.5; oct += 0.1) {
        const double f = 20.0 * std::pow(2.0, oct);
        std::printf("%8.1f\t%7.2f\n", f, measureGainDb(f, cfgs[c]));
      }
    }
    return 0;
  }

  const double midBoost = 7.9433;   // +18 dB
  const double shelfBoost = 6.3096; // +16 dB


  S flat;
  expectNear("flat @1k", measureGainDb(1000.0, flat), 0.0, 0.5);

  // ---- MID 1 (passive LCR resonator) -----------------------------------
  // Authentic B211 asymmetry: BOOST = LCR in the BA283 feedback path
  // (narrow bell, Q ~1.1); CUT = LCR shunting to ground through the pot
  // resistance (wide, shallow-skirted divider). Boost and cut are NOT
  // mirror images -- that asymmetry lives in the hardware network.
  S m1 = flat;
  m1.mid1F = 700.0;
  m1.mid1Gain = midBoost;
  expectNear("Mid1 +18 @700", measureGainDb(700.0, m1), 18.0);
  // Boost bell: -6 dB (half amplitude) at ~1 octave off centre.
  expectNear("Mid1 +18 @1.6k", measureGainDb(1600.0, m1), 10.0, 3.0);
  S m1c = flat;
  m1c.mid1F = 700.0;
  m1c.mid1Gain = 0.1259; // -18 dB
  expectNear("Mid1 -18 @700", measureGainDb(700.0, m1c), -18.0);
  // Cut divider: much wider than the boost bell. At 1.19 oct off centre
  // the shunt divider is still ~-11 dB deep (vs +10 dB boost there).
  expectNear("Mid1 -18 @1.6k (wide cut)", measureGainDb(1600.0, m1c), -11.0,
             3.0);
  expectNear("Mid1 -18 @280 (wide cut)", measureGainDb(280.0, m1c), -11.0, 3.0);

  // ---- MID 2 (identical topology, independent switch/pot) --------------
  S m2 = flat;
  m2.mid2F = 360.0;
  m2.mid2Gain = midBoost;
  expectNear("Mid2 +18 @360", measureGainDb(360.0, m2), 18.0);
  S both = flat;
  both.mid1F = both.mid2F = 1600.0;
  both.mid1Gain = both.mid2Gain = midBoost;
  // Linear network: two +18 dB mids stacked sum to +36 dB at resonance.
  // Measured at a LOW level (-40 dBFS): the BA283 output stage is now live,
  // and at -20 dBFS a +36 dB sum drives it deep into saturation (level-
  // dependent compression is the stage's job; the arithmetic sum only holds
  // in the small-signal region the stage is normalized to).
  expectNear("Mid1+Mid2 stacked @1.6k",
             measureGainDb(1600.0, both, 48000.0, 0.01), 36.0, 1.0);

  // Extreme drive safety: full-scale into the +36 dB stack drives the BA283
  // far past its rails (~126 V of drive). The stage must clip bounded and
  // stay finite (no fold-back, no solver blow-up) -- this is the level at
  // which the signature saturation is fully engaged.
  {
    Neve1073EqCircuit e;
    e.prepare(48000.0);
    e.setSettings(both);
    double peak = 0.0;
    for (int i = 0; i < 48000; ++i) {
      float x = (float)std::sin(2.0 * M_PI * 1600.0 * i / 48000.0);
      e.process(&x, 1);
      if (!std::isfinite(x)) {
        std::printf("FAIL non-finite output at sample %d (extreme drive)\n", i);
        ++failures;
        break;
      }
      if (i > 24000)
        peak = std::max(peak, std::abs((double)x));
    }
    const bool ok = std::isfinite(peak) && peak > 0.5 && peak < 8.0;
    std::printf("%s extreme drive bounded         peak %6.3f (0.5 < p < 8.0)\n",
                ok ? "PASS" : "FAIL", peak);
    if (!ok) ++failures;
  }

  // ---- LF shelf (B205 RC ladder, +/-16 dB) ------------------------------
  // Two-pole ladder cornered 2x above the switch frequency: the plateau
  // is fully established AT the switch frequency (the kick recipe "full
  // boost at 60" -- only -1 dB of ladder loss there), and it settles back
  // to unity by ~2 octaves above the corner (a real shelf, not a skirt
  // that never flattens). Chain is linear, so readings are exact.
  S lf = flat;
  lf.lfF = 60.0;
  lf.lfGain = shelfBoost;
  expectNear("LF +16 @30 (plateau)", measureGainDb(30.0, lf), 15.1, 1.0);
  expectNear("LF +16 @60 (at switch)", measureGainDb(60.0, lf), 13.8, 1.0);
  expectNear("LF +16 @1k (settled)", measureGainDb(1000.0, lf), -0.7, 0.5);

  // ---- HF shelf (fixed 12 kHz plateau) -----------------------------------
  S hf = flat;
  hf.hfGain = shelfBoost;
  // -1 dB of ladder loss at 12k (corner 6 kHz), ~-1.1 dB iron at 12k.
  expectNear("HF +16 @12k", measureGainDb(12000.0, hf), 13.9, 1.0);
  expectNear("HF +16 @100", measureGainDb(100.0, hf), 0.0, 0.5);

  // ---- HPF (3rd-order Butterworth, -3 dB at the switch label) ------------
  // One octave below the corner: |H| = r^3/sqrt(1+r^6), r = 0.5 -> -18.1 dB
  // (the asymptotic 18 dB/oct slope is already reached); one octave above:
  // ~-0.3 dB.
  S hp = flat;
  hp.hpfF = 50.0;
  expectNear("HPF50 @corner", measureGainDb(50.0, hp), -3.0, 1.0);
  expectNear("HPF50 @25 (1 oct dn)", measureGainDb(25.0, hp), -18.1, 1.5);
  expectNear("HPF50 @100 (1 oct up)", measureGainDb(100.0, hp), -0.3, 0.5);

  // ---- Stability: everything maxed must stay finite ----------------------
  S all = flat;
  all.lfF = 220.0;
  all.lfGain = shelfBoost;
  all.mid1F = all.mid2F = 3600.0;
  all.mid1Gain = all.mid2Gain = midBoost;
  all.hfGain = shelfBoost;
  // Sum of band skirts at 1k (linear chain).
  expectNear("all bands maxed @1k", measureGainDb(1000.0, all), 11.0, 2.0);

  // 96 kHz sanity run (retune accuracy at another rate).
  S m96 = flat;
  m96.mid1F = 1600.0;
  m96.mid1Gain = midBoost;
  expectNear("Mid1 +18 @1.6k @96k", measureGainDb(1600.0, m96, 96000.0), 18.0);

  std::printf(failures == 0 ? "ALL TESTS PASSED\n" : "%d TEST(S) FAILED\n",
              failures);
  return failures == 0 ? 0 : 1;
}
