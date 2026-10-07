#pragma once

#include <cmath>
#include <algorithm>

namespace even
{

//==============================================================================
// Physical-ish audio transformer model.
//
// Topology (per transformer):
//
//   vs --Cc-- Rw --+-- Lm (nonlinear core) --+   ideal 1:n  -- Lleak + Rs2 -- Rload
//                  |            i_m(lam)     |
//                 v_p                       i_p = n * i_s
//
//   Cc     primary coupling capacitor (the real circuit's DC block; keeps
//          residual DC from integrating the core flux into saturation)
//   Rw     primary winding resistance (sets the LF corner against Lm)
//   Lm     magnetizing inductance with a saturating B-H curve:
//             i_m(lam) = lam/Lm + imSat * t^3/(1+t^2),  t = lam/lamK
//          exactly linear in the small-signal regime (the cubic term is
//          O(t^3)), the excess current reaching ~imSat/2 at the knee and
//          growing only linearly beyond it (bounded slope, Newton-friendly)
//   lam    core flux, integrated from v_p by the trapezoidal rule
//   leak   leakage (+ secondary winding) inductance; sets the HF corner
//          against the secondary load, like the old one-pole LP but with
//          the correct phase and load interaction
//
// Hysteresis: a history-dependent offset current on the core branch. A slow
// (~2 ms) tracker M follows the flux; the branch current gains an offset
// (hystK/Lm)*(lam - M). Quasi-static signals see none (M keeps up); dynamic
// ones see loop widening that peaks at low frequencies -- exactly where the
// real iron's hysteresis loss lives -- and it vanishes at DC and decays to
// zero in silence (no drift). The term's slope is positive and bounded, so
// i_m stays strictly monotone in lam and the Newton root-find stays unique.
// hystK = 0 bypasses it entirely (pure saturating inductor).
//
// Discretization: the leakage branch under the trapezoidal rule makes the
// secondary current algebraic in the primary node voltage v_p, and the core
// branch is a monotone C1 function of lam (whose increment is also linear in
// v_p). The whole sample therefore reduces to ONE scalar root-find in v_p --
// F is strictly increasing, the root is unique, and a few damped Newton
// iterations from the previous sample's v_p converge to machine precision.
// Cost: a dozen flops plus ~2-4 iterations, far below the BJT stage solves.
struct TransformerModel
{
    struct Params
    {
        double ratio = 1.0;    // 1:n primary:secondary voltage ratio
        double rw    = 100.0;  // primary winding resistance [ohm]
        double lm    = 2.0;    // magnetizing (small-signal) inductance [H]
        double leak  = 2.0e-3; // leakage inductance [H]
        double rs2   = 40.0;   // secondary winding resistance [ohm]
        double rload = 600.0;  // secondary load [ohm]
        double cc    = 6.8e-4; // primary coupling capacitor [F] (DC block)
        double imSat = 2.5e-3; // core saturation current scale [A]
        double lamK  = 2.5e-2; // flux knee [Wb] (~full-scale LF flux swing)
        double hystK = 0.0;    // hysteresis intensity (0 = off); offset slope
                               // is hystK/Lm against a ~2 ms flux tracker
    };

    void setParams (const Params& p) { prm = p; }
    const Params& getParams() const { return prm; }

    void prepare (double sampleRate) { fs = sampleRate; reset(); }

    void reset()
    {
        lam = 0.0; vC = 0.0; vP = 0.0; iS0 = 0.0; iRw0 = 0.0; M = 0.0;
    }

    // Everything decayed (digital silence fast path).
    bool quiet() const
    {
        return std::abs (lam) < 1.0e-12 && std::abs (vC) < 1.0e-12
            && std::abs (vP) < 1.0e-12 && std::abs (iS0) < 1.0e-15
            && std::abs (iRw0) < 1.0e-15;
    }

    // vs: primary-side input voltage. Returns the secondary load voltage.
    double process (double vs)
    {
        const double Ts   = 1.0 / fs;
        const double Ts2  = 0.5 * Ts;
        const double G    = 2.0 * prm.leak / Ts;     // leakage companion [ohm]
        const double Rtot = prm.rs2 + prm.rload;
        const double n    = prm.ratio;

        // Coupling cap, fully implicit trapezoidal update. The cap current is
        // the winding current, which depends on the node voltage, which
        // depends on the cap voltage -- solved together with v_p below. (An
        // explicit/semi-implicit cap update here puts a one-sample delay in
        // a loop with two integrators -- cap and core flux -- and goes
        // exponentially unstable; the implicit form is A-stable.)
        //   vC[n] = (vC + Ts/(2cc) * (iRw[n] + iRw0)) / (1 - Ts/(2*cc*rw))
        // where iRw is the charging current (vsEff -> v_p direction).
        const double aC = Ts / (2.0 * prm.cc * prm.rw);
        auto capV = [&] (double vp, double vsEff)
        {
            const double iR = (vsEff - vp) / prm.rw; // charging current
            return (vC + Ts2 / prm.cc * (iR + iRw0)) / (1.0 - aC);
        };

        // Core branch: i_m(lam) = lam/Lm                  (linear inductance)
        //                       + imSat * t^3/(1+t^2)     (B-H saturation)
        //                       + (hystK/Lm)*(lam - M)    (hysteresis)
        // where M is a slow first-order tracker of the flux (see below). The
        // tracker term is a history-dependent offset current: quasi-static
        // signals see none (M follows lam), dynamic ones see an offset in
        // proportion to how far the flux has run ahead of the tracked value
        // -- loop widening that peaks at low frequencies (where the real
        // iron's hysteresis loss lives) and vanishes at DC. Crucially the
        // term's slope is positive and bounded, so i_m stays strictly
        // monotone in lam and the Newton below keeps its unique root.
        auto iCore = [&] (double l, double& didl)
        {
            const double t   = l / prm.lamK;
            const double den = 1.0 + t * t;
            const double cur = l / prm.lm
                             + prm.imSat * (t * t * t / den)
                             + prm.hystK * (l - M) / prm.lm;
            didl = (1.0 + prm.hystK) / prm.lm
                 + prm.imSat / prm.lamK * (3.0 * t * t + t * t * t * t) / (den * den);
            return cur;
        };

        // Secondary loop, trapezoidal rule on the leakage inductor. The
        // leakage voltage is NOT a free state: it must satisfy the KVL
        // constraint vL = n*vP - Rtot*iS exactly. Letting it drift as an
        // independent trapezoid state creates an unstable alternating mode
        // (measured eigenvalue ~ -1.12); deriving it from the stored
        // (vP, iS0) pair keeps the discretization A-stable.
        //   n*vp = vL + Rtot*iS,  vL = G*(iS - iS0) - vL0
        //   -> iS = (n*(vp + vP) + (G - Rtot)*iS0) / (G + Rtot)

        // Scalar Newton in v_p. F(v_p) = (v_p - vsEff)/Rw + i_m(lam(v_p))
        //                                + n * i_s(v_p)
        // is strictly increasing (winding, core and reflected-load terms all
        // have positive slope), so the root is unique; clamping the step keeps
        // the iteration well-behaved from a cold or far seed.
        const double dAl = Ts2; // d(lam)/d(v_p) of the trapezoid flux increment
        double vp = vP;         // seed: previous sample's node voltage
        double vsEff = vs;
        for (int it = 0; it < 12; ++it)
        {
            vsEff = vs - capV (vp, vsEff);
            double didl = 0.0;
            const double im = iCore (lam + dAl * (vp + vP), didl);
            const double iS = (n * (vp + vP) + (G - Rtot) * iS0) / (G + Rtot);
            const double F  = (vp - vsEff) / prm.rw + im + n * iS;
            if (std::abs (F) < 1.0e-9)
                break;
            const double J = (1.0 - aC) / prm.rw + didl * dAl + n * n / (G + Rtot);
            const double maxStep = 2.0 * (std::abs (vs) + 1.0);
            vp -= std::clamp (F / J, -maxStep, maxStep);
            if (! std::isfinite (vp)) { reset(); return 0.0; }
        }

        // Finalize the sample at the converged v_p.
        const double lNew  = lam + dAl * (vp + vP);
        vsEff = vs - capV (vp, vsEff);
        const double vcNew = capV (vp, vsEff);
        const double iS  = (n * (vp + vP) + (G - Rtot) * iS0) / (G + Rtot);
        const double out = prm.rload * iS;                // secondary load voltage
        const double iRw = (vsEff - vp) / prm.rw;

        // Hysteresis tracker update on the true flux: first-order lag with a
        // ~2 ms time constant. Slow enough to hold memory across audio-rate
        // swings, fast enough to fully forget during silence (no DC drift).
        M += std::min (1.0, Ts / hystTau) * (lNew - M);

        lam = lNew; vC = vcNew; vP = vp; iS0 = iS; iRw0 = iRw;
        return out;
    }

private:
    Params prm;
    double fs = 48000.0;

    double lam = 0.0;   // core flux [Wb]
    double vC  = 0.0;   // coupling capacitor voltage
    double vP  = 0.0;   // primary node voltage (Newton seed)
    double iS0 = 0.0;   // previous secondary current (trapezoid history)
    double iRw0 = 0.0;  // previous winding current (cap update)
    double M   = 0.0;   // slow flux tracker for the hysteresis offset

    static constexpr double hystTau = 2.0e-3; // hysteresis tracker tau [s]
};

} // namespace even
