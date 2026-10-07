#pragma once

#include <cmath>

namespace even
{

//==============================================================================
// Ebers-Moll transport-model BJT with analytic derivatives, suitable for
// Newton-Raphson nodal analysis (the "Newton for transistors" part).
//
//   Ic = Is * (exp(vbe/Vt) - exp(vbc/Vt)) - (Is/betaR) * (exp(vbc/Vt) - 1)
//   Ib = (Is/betaF) * (exp(vbe/Vt) - 1) + (Is/betaR) * (exp(vbc/Vt) - 1)
//
// Exponentials are clamped to exp(30) for numerical safety under Newton
// over-shoot; the damped line search in NewtonSolver recovers from clamps.
//
// Optional reverse-junction current cap (irCap > 0): the raw model lets the
// base->collector reverse current grow without bound in deep saturation,
// which in a stage whose collector load can only SOURCE current produces the
// classic "fold-back" artifact -- the collector is dragged back up toward the
// base and the transfer curve reverses direction instead of clipping (heard
// as a double-dip "W" on one half of the waveform). Capping ir with a smooth
// rational saturator (C1 everywhere, analytic derivative included) makes
// deep saturation bottom out at a finite reverse current: the stage clips
// flat instead of folding. irCap = 0 (default) keeps the raw model.
struct BjtModel
{
    double Is    = 1.0e-15;   // saturation current [A]
    double betaF = 300.0;     // forward current gain
    double betaR = 4.0;       // reverse current gain
    double Vt    = 0.025852;  // thermal voltage @ ~27C
    double irCap = 0.0;       // reverse-current cap [A]; 0 = uncapped

    struct Operating
    {
        double ic = 0.0, ib = 0.0;                       // currents [A]
        double dic_dvbe = 0.0, dic_dvbc = 0.0;           // transconductances [S]
        double dib_dvbe = 0.0, dib_dvbc = 0.0;           // input conductances [S]
    };

    // Smoothly tapering exponential: identical to exp(x) below x0, then the
    // effective slope drops to `k` (per natural-log unit) so the exponential
    // keeps *growing* -- just slowly. This is what makes deep saturation
    // behave physically: in a saturated stage the base-collector junction
    // current must be able to out-grow the forward current so that ic can
    // actually reverse and clamp the collector near the emitter. A hard
    // exp-ceiling instead freezes both junction currents at the same value,
    // leaving a residual negative ic that pushes the collector *above the
    // supply rail* (and below ground on the other swing) -- the classic
    // "output tops overshoot and never clip" artifact.
    static double ex (double x)
    {
        // Knee: above x = 30 (~0.78 V of limited junction voltage) the slope
        // tapers. The stage's maximum physical conduction is Vcc/Rc ~= 7 mA,
        // i.e. x ~= 29.5, so normal operation (bias x ~= 29) stays below the
        // knee; only wayward Newton iterates enter the tail, and there the
        // baseline current is ~11 mA rather than the 600 mA a knee of 34
        // produced (which made the residual a near-vertical wall that the
        // line search could never descend -- the "jagged clip top" bug).
        constexpr double x0 = 30.0;
        constexpr double k  = 0.15;
        if (x <= x0) return std::exp (x);
        return std::exp (x0) * std::exp ((x - x0) * k);
    }

    // d/dx of ex(x) -- must stay consistent with ex() so the Newton
    // Jacobian matches the residual.
    static double exd (double x)
    {
        constexpr double x0 = 30.0;
        constexpr double k  = 0.15;
        if (x <= x0) return std::exp (x);
        return k * std::exp (x0) * std::exp ((x - x0) * k);
    }

    // e = ex(x) and d = exd(x) from a SINGLE exp evaluation. Hot-path helper:
    // residual and Jacobian always need both, and exp is the dominant cost.
    static void exPair (double x, double& e, double& d)
    {
        constexpr double x0 = 30.0;
        constexpr double k  = 0.15;
        if (x <= x0) { e = std::exp (x); d = e; }
        else { e = std::exp (x0) * std::exp ((x - x0) * k); d = k * e; }
    }

    // Two-lane exPair: identical results to two exPair calls. (A NEON 2-lane
    // exp was measured at parity with two scalar libm exps on Apple Silicon --
    // libm exp dual-issues across the FP pipes -- so this is kept as the
    // single scalar fast path with a deep-reverse-bias early out instead:
    // exp(x) < e^-40 ~ 4e-18 is indistinguishable from 0 at the solver's
    // 1e-10 tolerance, and the reverse junction sits there most of the time.)
    static void exPair2 (double xa, double xb, double& ea, double& da, double& eb, double& db)
    {
        if (xa > -40.0) exPair (xa, ea, da); else { ea = 0.0; da = 0.0; }
        if (xb > -40.0) exPair (xb, eb, db); else { eb = 0.0; db = 0.0; }
    }

    Operating eval (double vbeIn, double vbcIn) const
    {
        // Junction limiting: beyond +/-0.7 V the junction voltage is allowed
        // to grow only with a small slope (20 mV per volt of overdrive).
        // Crucially the returned derivatives are scaled by the same limiter
        // slope, so the Jacobian stays consistent with the residual and the
        // Newton iteration remains well-behaved far from the bias point
        // (large-signal audio sweeps).
        const double vbe = lim (vbeIn), vbc = lim (vbcIn);

        double eF, dF, eR, dR;
        exPair2 (vbe / Vt, vbc / Vt, eF, dF, eR, dR); // one vector exp for both junctions
        const double if_ = Is * (eF - 1.0);
        const double ir  = Is * (eR - 1.0);

        // Smooth reverse-current cap: irEff = irCap*ir/(irCap+ir) tends to
        // irCap for ir >> irCap and to ir (slope 1) for ir << irCap, so the
        // cap engages C1-continuously exactly where saturation begins.
        // The Jacobian uses the same rational slope, keeping Newton exact.
        double irEff = ir, capSlope = 1.0;
        if (irCap > 0.0 && ir > 0.0)
        {
            const double s = irCap / (irCap + ir);
            irEff = irCap * s * (ir / irCap); // = irCap*ir/(irCap+ir)
            capSlope = s * s;
        }

        // Consistent derivatives: d(e)/dv uses the tapered-slope form so the
        // Jacobian matches the residual even deep in the saturation tail.
        const double gF  = Is / Vt * dF;          // d(exp)/dv
        const double gR  = Is / Vt * dR * capSlope;

        Operating op;
        op.ic = if_ - irEff - irEff / betaR;
        op.ib = if_ / betaF + irEff / betaR;

        op.dic_dvbe = gF * limd (vbeIn);
        op.dic_dvbc = -gR * (1.0 + 1.0 / betaR) * limd (vbcIn);
        op.dib_dvbe = gF / betaF * limd (vbeIn);
        op.dib_dvbc = gR / betaR * limd (vbcIn);
        return op;
    }

    // Piecewise-linear junction limiter: identity within +/-0.72 V, then a
    // 0.08 slope tail (12.5x attenuation of overdrive into the exponential).
    // The tail doubles as the soft-saturation mechanism for large drives.
    // NOTE: the negative tail must keep the junction voltage *decreasing*
    // (more reverse bias -> more negative); getting its sign wrong maps deep
    // reverse bias onto a FORWARD junction (~+0.66 V at -18 V), which creates
    // phantom conduction, extra spurious roots, and jagged clip tops.
    static double lim (double v)
    {
        if (v >  0.72) return 0.72 + 0.08 * (v - 0.72);
        if (v < -0.72) return -0.72 + 0.08 * (v + 0.72);
        return v;
    }

    static double limd (double v) { return (v > 0.72 || v < -0.72) ? 0.08 : 1.0; }
};

} // namespace even
