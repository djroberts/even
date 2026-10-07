 #pragma once

#include <cmath>
#include <algorithm>
#include <limits>
#include <mutex>
#include <cstdio>
#include <vector>
#include <even/NewtonSolver.h>
#include <even/BjtModel.h>
#include <even/TransformerModel.h>

//==============================================================================
// Neve 1073 preamp circuit model.
//
// Signal path modelled after the 1073 module schematic:
//
//   Marinair LO1166 input transformer (1:2 step-up)
//     -> sensitivity/attenuator network (the gain knob)
//     -> Class-A common-emitter voltage amplifier (BA215/BA284 style, self-
//        biased through a divider, emitter degenerated)  [3-node MNA + Newton]
//     -> BA283 output amplifier: driver stage + Class-AB complementary
//        emitter-follower pair (BC184L NPN / BC214L PNP) with ~2*Vbe bias
//        spreader and global feedback from the output into the driver
//        emitter  [4-node MNA + Newton]
//
// Junction capacitances are folded out (audio-band algebraic model); coupling
// and transformer behaviour are handled by the surrounding one-pole filters,
// which also carry the model's memory between samples.
class Neve1073Circuit
{
public:
    struct CeResult { double vb, ve, vc; };
    struct PpState  { double b, e, c, o; }; // driver b/e/c + output node

    // Audio engine selection. Both modes share the precomputed static
    // transfer curve (see buildTransferTable):
    //
    //   Live : pure per-sample Newton solves (the original behaviour;
    //          validation/benchmark reference, not exposed in the UI)
    //   Exact: table lookup seeds the Newton solves, which polish to the
    //          usual 1e-10 tolerance. Result is within ~1e-10 of the Live
    //          root -- far below one float32 ULP, so bit-transparent at the
    //          plugin's output precision, at roughly half the CPU.
    //   Fast : pure interpolated table lookup, no Newton in the audio path.
    //          Validated against Live to a worst-case deviation below the
    //          float32 LSB before being shipped as a selectable mode.
    enum class Engine { Live, Exact, Fast };

    void setEngine (Engine e) { engine = e; }
    Engine getEngine() const { return engine; }

    void prepare (double sampleRate, bool buildTable = true)
    {
        fs = sampleRate;

        // Reset the homotopy continuation state BEFORE the DC solves and the
        // calibration run below: prepare() may be called on a circuit that
        // has already processed audio (e.g. the oversampling toggle swapping
        // the model between 1x and 2x the host rate), and a stale `lastCeVin`
        // from heavy drive steers the Live-solve homotopy during calibration
        // to a different operating branch -- permanently mis-calibrating
        // `normalization`. (The reset at the end of prepare only cleans up
        // for playback; it happens too late for the calibration itself.)
        lastCeVin = 0.0;
        // Cold-start the push-pull junction warm-start caches too: a stale
        // cache from a previously prepared rate would extrapolate from a
        // far-away operating point on the first samples (harmless -- the
        // solve still converges to the same unique root -- but wasteful).
        pp.jnV = pp.jpV = std::numeric_limits<double>::quiet_NaN();

        // DC operating point of the Class-A stage (signal source at 0 V),
        // starting from a rough hand-computed bias estimate.
        dcSolution = { 2.9, 1.8, 13.0 };
        const auto mid = ceSolve (0.0, dcSolution);
        ceDc = dcSolution; // physical fallback seed for unconverged solves

        // Output stage. The driver is fed the Class-A collector signal minus
        // its DC level (the behavioural equivalent of the interstage
        // coupling), so the driver's own divider sets the idle point and the
        // solve below starts from the quiescent state at zero input.
        ppInRef = mid.vc;
        // The feedback path is AC-coupled (output transformer), referenced at
        // the output node's own quiescent DC. That reference depends on the
        // DC solve, and the DC solve depends on the reference -- but only
        // weakly (the output node's sensitivity to it is ~rl/(rf*gm) << 1),
        // so a fixed-point iteration from an analytic first guess (collector
        // idle minus spreader drop minus the pair's conduction offset)
        // converges in a couple of passes.
        pp.nfbRef = 12.7;
        for (int pass = 0; pass < 6; ++pass)
        {
            ppDc = { 1.5, 0.85, 13.0, 12.5 }; // rough hand estimate; polished below
            ppSolve (0.0, ppDc);
            const double prev = pp.nfbRef;
            pp.nfbRef = ppDc[3];
            if (std::abs (ppDc[3] - prev) < 1.0e-9)
                break;
        }
        ppDcQuiescent = ppDc; // physical fallback seed for unconverged solves
        ppQuiescentVo = ppDc[3];

        // Precompute the static transfer curve (both stages are memoryless:
        // the only circuit memory lives in the one-pole filters). Built with
        // the live Newton solves, warm-seeded along the sweep, so the table
        // IS the solver's output. Shared across instances and rebuilt never
        // (independent of sample rate and gain).
        if (buildTable)
        {
            const auto& st = sharedTables();
            table = st.main;
            dense = st.dense;
            micro = st.micro;
            for (int f = 0; f < 7; ++f)
                seedCoefPtr[f] = st.seedCoef[f].data();
        }

        // Transformer models. These must be prepared BEFORE the calibration
        // run below, which exercises the real signal chain (processSample
        // depends on them).
        //
        // Input: Marinair LO1166, 1:2 step-up. Winding + source resistance
        // against the magnetizing inductance gives the ~12 Hz LF corner; the
        // core knee is placed at the flux swing of a full-scale ~40 Hz signal
        // so hot low end saturates the iron while mids stay clean.
        even::TransformerModel::Params pin;
        pin.ratio = 2.0;      // 1:2 step-up
        pin.rw    = 1200.0;   // source + primary winding resistance
        pin.lm    = 16.0;     // 1200 / (2*pi*12 Hz) -> ~12 Hz LF corner
        pin.leak  = 0.012;    // ~134 kHz HF corner against the ~10k input
        pin.rs2   = 40.0;
        pin.rload = 10000.0;  // sensitivity network input impedance
        pin.cc    = 4.7e-5;   // ~3 Hz DC block
        pin.imSat = 1.0e-3;
        pin.lamK  = 8.0e-3;   // full-scale 40 Hz flux swing
        pin.hystK = 0.35;     // hysteresis loop widening
        inTf.setParams (pin);
        inTf.prepare (fs);

        // Output: BA283 output iron. Winding resistance against Lm gives the
        // ~8 Hz LF corner the old HP one-pole approximated; leakage against
        // the 600 ohm load gives the ~48 kHz HF corner of the old LP.
        even::TransformerModel::Params pout;
        pout.ratio = 1.0;
        pout.rw    = 100.0;   // 100 / (2*pi*8 Hz) -> ~8 Hz LF corner
        pout.lm    = 2.0;
        pout.leak  = 1.2e-3;  // ~85 kHz HF corner against the 640 ohm loop
        pout.rs2   = 40.0;
        pout.rload = 600.0;   // reflected load
        pout.cc    = 6.8e-4;  // ~2 Hz DC block
        pout.imSat = 2.5e-3;
        pout.lamK  = 2.5e-2;  // full-scale 40 Hz flux swing
        pout.hystK = 0.35;
        outTf.setParams (pout);
        outTf.prepare (fs);

        // Calibrate normalization by running the real chain: with unity
        // normalization, measure the output for a -12 dBFS-ish sine and set
        // the trim so 0 dB sensitivity passes it at unity. Runs in Live mode
        // so calibration never depends on the (possibly not-yet-built) table
        // -- and so the calibration result is identical for every engine.
        //
        // The calibration MUST run at unity drive: prepare() may be called on
        // a circuit that has already processed audio (e.g. the oversampling
        // toggle re-preparing at 2x the host rate), and a leftover hot
        // driveScale would make the calibration measure an overdriven peak,
        // permanently mis-scaling `normalization` (observed: 0.079 -> 0.043
        // after re-preparing while the knob sat at -40 dB). Save and restore
        // the caller's gain around it.
        const float savedDriveGain = driveGain;
        driveGain = 1.0f;
        driveScale = inputScale;
        normalization = 1.0;
        const Engine savedEngine = engine;
        engine = Engine::Live;
        {
            const double amp = 0.25, freq = 100.0;
            const int n = (int) (fs * 0.05);
            double peak = 0.0;
            for (int i = 0; i < n; ++i)
            {
                const float y = processSample ((float) (amp * std::sin (2.0 * 3.14159265358979 * freq * i / fs)));
                if (i > n / 2) peak = std::max (peak, std::abs ((double) y));
            }
            if (peak > 1.0e-6)
                normalization = amp / peak;
        }
        engine = savedEngine;
        driveGain = savedDriveGain;
        driveScale = (double) driveGain * inputScale;

        // The calibration run above exercises the real signal chain, which
        // drags the continuation states to wherever the last sine sample
        // landed. Restore them to the quiescent operating point so playback
        // starts from the DC bias, not a stale signal snapshot.
        dcSolution = ceDc;
        ppDc = ppDcQuiescent;
        lastCeVin = 0.0;
        inTf.reset();
        outTf.reset();
    }

    // Debug hook (harness only): current DC operating point and calibration.
    struct DebugState { double vb, ve, vc, pb, pe, pc, po, norm; double res; };
    DebugState debugState() const
    { return { dcSolution[0], dcSolution[1], dcSolution[2], ppDc[0], ppDc[1], ppDc[2], ppDc[3], normalization, lastResidual }; }

#ifdef NEVE_TABLE_DEBUG
    // Validation hook (harness only): table output vs a live solve at an
    // arbitrary drive. The live solve mutates the seed state; validation
    // instances are throwaway.
    struct TableProbe { double tableVo, liveVo; };
    TableProbe probeTable (double drive)
    {
        const double tableVo = shapeCurve (drive, evalTableVo (drive) - ppQuiescentVo);
        const double live = solveOutput (drive);
        return { tableVo, live };
    }

    // Table-build debug hook: one live solve step with continuation, exactly
    // as the build sweeps do (advances the internal seed state).
    double probeSweepVo (double drive) { return solveOutput (drive); }

    // Debug hooks: stored knot values.
    double debugMainKnot (int i) const  { return table[(size_t) i].vo; }
    double debugDenseKnot (int i) const { return dense[(size_t) i]; }
    void debugPrintMicro() const
    {
        for (const auto& w : micro)
            printf ("micro window: %.6f .. %.6f (%zu pts)\n", w.d0, w.d1, w.vo.size());
    }
    mutable double lastDrive = 0.0;     // drive of the last processed sample
    double debugDrive() const { return lastDrive; }
#endif

    // gainKnob: -80..+10 dB, exactly like the 1073 sensitivity control.
    // Turning it *down* records hotter -> more drive into the saturation
    // stages; turning it up pads the input. 40 dB/knob-decade keeps the
    // sweep musically matched to the hardware's 5 dB steps.
    void setGainDb (float db)
    {
        driveGain = std::pow (10.0f, -db / 40.0f);
        driveScale = (double) driveGain * inputScale; // hoisted per-sample multiply
    }

    // Process a block in place. Preferred over per-sample calls from the
    // plugin's processBlock: keeps the hot loop in one function so the
    // solver state and coefficients stay in registers across samples.
    void process (float* samples, int numSamples)
    {
        for (int i = 0; i < numSamples; ++i)
            samples[i] = processSample (samples[i]);
    }

    float processSample (float in)
    {
        // --- Input transformer: step-up + physical coupling (state model) ---
        const double x = inTf.process ((double) in);

        // Digital-silence fast path: once the transformer states have decayed
        // to nothing, a silent input is the DC operating point exactly -- no
        // Newton solves needed, the transformer states stay at zero.
        if (x == 0.0 && inTf.quiet() && outTf.quiet())
        {
            inTf.reset();
            outTf.reset();
            return 0.0f;
        }

        const double drive = x * driveScale;
#ifdef NEVE_TABLE_DEBUG
        lastDrive = drive; // harness observability; not in the plugin build
#endif

        // --- Nonlinear stages: Live solve, table-seeded solve, or table ---
        double y;
        switch (engine)
        {
            case Engine::Fast:
            {
                y = shapeCurve (drive, evalTableVo (drive) - ppQuiescentVo);
                break;
            }
            case Engine::Exact:
            {
                // Table-interpolated state as the Newton seed: within
                // interpolation error of the root, so the polish converges
                // in 1-2 iterations to the usual 1e-10 tolerance.
                const auto tp = evalTable (drive);
                even::NewtonSolver<3>::Vec ceSeed { tp.vb, tp.ve, tp.vc };
                even::NewtonSolver<4>::Vec ppSeed { tp.pb, tp.pe, tp.pc, tp.vo };
                const auto op = ceSolve (drive, ceSeed);
                const auto out = ppSolve (op.vc - ppInRef, ppSeed);
                y = shapeCurve (drive, out.o - ppQuiescentVo);
                break;
            }
            case Engine::Live: // original per-sample solves
            {
                y = solveOutput (drive);
                break;
            }
        }

        // --- Output transformer: physical coupling, removes DC itself ---
        return (float) (outTf.process (y) * normalization * outputTrim);
    }

    // Validation/benchmark hook: Live-engine output through the same filter
    // chain, regardless of the current engine selection. Run on a SEPARATE
    // circuit instance (it advances the filter state).
    float processSampleLive (float in)
    {
        const double drive = inTf.process ((double) in) * driveScale;
        double y = solveOutput (drive);
        return (float) (outTf.process (y) * normalization * outputTrim);
    }

private:
    // Live Newton chain: Class-A stage solve feeding the push-pull stage.
    // This is the behavioural definition of the model.
    double solveOutput (double drive)
    {
        const auto op = ceSolve (drive, dcSolution);
        const auto out = ppSolve (op.vc - ppInRef, ppDc);
        return shapeCurve (drive, out.o - ppQuiescentVo);
    }

    //==========================================================================
    // Clip envelope.
    //
    // Past its saturation point the composed curve does not clip -- it
    // FOLDS. When the driver saturates, its emitter follows the base and
    // (with the Vce floor above) the collector rides just above the emitter,
    // so the whole output stage slides back UP as drive keeps growing: the
    // negative branch reaches its minimum at drive ~ -1.45 and then reverses
    // toward -1.9 (the "double-dip W" on a scope). A symmetric slide happens
    // on the positive side past drive ~ +14 (the Class-A stage drifting out
    // of its own saturation). A real amp absorbs overdrive at the clip
    // level instead of folding, and the natural slope at both extremes is
    // ~0, so holding the curve at its measured extreme beyond the fold
    // onset joins C1 and is exactly the "clip flat" behaviour of the real
    // stage. Applied to the composed curve at the engine output points
    // (solveOutput / Exact path / Fast table read); table STORAGE stays raw
    // so the residual-verified repair pass still sees true roots.
    static constexpr double clipNegDrive = -1.344;
    static constexpr double clipNegLevel = -5.6463;
    static constexpr double clipPosDrive = 11.96;
    static constexpr double clipPosLevel =  6.9816;

    static double shapeCurve (double drive, double y)
    {
        if (drive < clipNegDrive) return clipNegLevel;
        if (drive > clipPosDrive) return clipPosLevel;
        return y;
    }

    //==========================================================================
    // Static transfer curve.
    //
    // Both nonlinear stages are algebraic, so the composed stage chain is a
    // pure function of `drive`; the only circuit memory is in the one-pole
    // filters. The curve is independent of sample rate and of the gain knob
    // (the knob scales the lookup coordinate via driveScale, not the curve).
    //
    // Domain: the curve is exponentially saturated outside this range
    // (validated: f identical to full double precision from |drive| ~ 130
    // upward on the positive side and ~ -16 on the negative side; margins
    // included). Clamped lookup beyond the edges.
    static constexpr int    tableN   = 1 << 17; // 131072 points
    static constexpr double tableDMin = -40.0;
    static constexpr double tableDMax = 160.0;
    // The push-pull turn-on region (drive ~ -2.6..+1.6) has a near-vertical
    // stretch (slope up to ~5 per unit drive with curvature concentrated in
    // a ~1e-2-wide window); a uniform table that resolves it everywhere else
    // cannot also resolve it here. A dense sub-table covers just this zone.
    static constexpr int    denseN    = 1 << 19;
    static constexpr double denseDMin = -2.7;
    static constexpr double denseDMax = 2.1;
    // The junction limiters put a handful of near-kinks in the curve at
    // isolated drive values (measured: 4 clusters outside the dense window,
    // each only a few main-table knots wide). Around each cluster a small
    // micro-table is built automatically; quintic interpolation across a
    // kink at main-table spacing leaves ~1e-4 error, at micro spacing
    // ~1e-10.
    struct MicroWindow { double d0, d1; std::vector<double> vo; };
    static constexpr int microN = 1 << 14;   // points per micro window
    // Node states vb/ve/pb/pe/pc float (they only seed the Newton polish in
    // Exact mode); vc/vo in double: vo is the audio in Fast mode (a float ULP
    // near the 13 V rail is ~1e-6), and vc drives the push-pull stage.
    struct TablePoint { double vb, ve, vc, pb, pe, pc, vo; };

    struct SharedTables
    {
        std::vector<TablePoint> main;
        std::vector<double> dense;
        std::vector<MicroWindow> micro;
        // Precomputed Catmull-Rom polynomial coefficients (c1..c3) per field
        // per knot, for the Exact engine's seed lookup. c0 is the knot value
        // itself, already stored in `main`. 3 floats x 7 fields x tableN.
        std::vector<float> seedCoef[7];
    };

    // The curve depends on neither sample rate nor gain, so one table is
    // built per process and shared by every circuit instance.
    static const SharedTables& sharedTables()
    {
        static SharedTables shared;
        static std::once_flag once;
        std::call_once (once, []
        {
            Neve1073Circuit scratch;
            scratch.prepare (48000.0, /*buildTable=*/false); // DC + calibration only
            scratch.buildTransferTable();
            shared.main = std::move (scratch.table);
            shared.dense = std::move (scratch.dense);
            shared.micro = std::move (scratch.micro);
            for (int f = 0; f < 7; ++f)
                shared.seedCoef[f] = std::move (scratch.seedCoef[f]);
        });
        return shared;
    }

    void buildTransferTable()
    {
        table.assign ((size_t) tableN, {});
        dense.assign ((size_t) denseN, 0.0);
        const double hMain  = (tableDMax - tableDMin) / (double) (tableN - 1);
        const double hDense = (denseDMax - denseDMin) / (double) (denseN - 1);

        auto resetSeeds = [&]
        {
            dcSolution = ceDc;
            lastCeVin = 0.0;
            // NOTE: the push-pull solve is deliberately NOT reset here. Its
            // cold solve (quiescent seed) does not converge in the driver's
            // deep-saturation windows (drive ~100..105, ~151..154), while
            // warm continuation from the previous knot converges everywhere
            // to < 1e-9 -- the same warm tracking the audio path performs.
            // Root-sheet drift is bounded by the residual-verified repair
            // pass below.
        };
        // Cold solve: every main-table knot starts from the quiescent
        // operating point (plus the solver's internal homotopy). Warm
        // continuation from the previous knot is NOT safe here: at high
        // drive the composed system has a second, converged root sheet that
        // the audio path's per-sample jumps never visit -- a cold solve
        // reliably reproduces the branch the Live engine lands on.
        auto solveInto = [&] (double drive, TablePoint& p)
        {
            resetSeeds();
            const auto op = ceSolve (drive, dcSolution);
            const auto out = ppSolve (op.vc - ppInRef, ppDc);
            p.vb = op.vb; p.ve = op.ve;
            p.vc = op.vc; p.pb = out.b; p.pe = out.e; p.pc = out.c; p.vo = out.o;
        };
        // Warm solve (for the dense window, which sits well inside the
        // smooth region; outliers are caught by the scan below).
        auto warmVo = [&] (double drive) -> double
        {
            const auto op = ceSolve (drive, dcSolution);
            const auto out = ppSolve (op.vc - ppInRef, ppDc);
            return out.o;
        };
        auto coldVo = [&] (double drive) -> double
        {
            resetSeeds();
            return warmVo (drive);
        };

        // Main table: cold solve at every knot.
        for (int i = 0; i < tableN; ++i)
            solveInto (tableDMin + (double) i * hMain, table[(size_t) i]);

        // Dense sub-table over the push-pull turn-on window: warm-seeded
        // fine sweep (fast), then an outlier scan -- a knot whose second
        // difference against its neighbours is way above the curve's
        // legitimate curvature there is re-solved cold.
        for (int i = 0; i < denseN; ++i)
            dense[(size_t) i] = warmVo (denseDMin + (double) i * hDense);
        for (int i = 1; i < denseN - 1; ++i)
        {
            const double d2 = dense[(size_t) i + 1] - 2.0 * dense[(size_t) i]
                            + dense[(size_t) i - 1];
            if (std::abs (d2) > 1.0e-5)
                dense[(size_t) i] = coldVo (denseDMin + (double) i * hDense);
        }

        // Main-table repair pass: verify every stored point against both
        // stage residuals (the solver converges to 1e-10; a little slack
        // avoids re-churn on borderline knots); re-solve any offender cold.
        size_t repairedMain = 0;
        for (int i = 0; i < tableN; ++i)
        {
            const double drive = tableDMin + (double) i * hMain;
            auto& p = table[(size_t) i];

            even::NewtonSolver<3>::Vec ceX { p.vb, p.ve, p.vc };
            even::NewtonSolver<3>::Vec F3 {};
            ceResidual (ce, drive, ceX, F3);
            const double rCe = even::NewtonSolver<3>::norm (F3);

            even::NewtonSolver<4>::Vec ppX { p.pb, p.pe, p.pc, p.vo };
            even::NewtonSolver<4>::Vec F2 {};
            ppResidual (pp, p.vc - ppInRef, ppX, F2);
            const double rPp = even::NewtonSolver<4>::norm (F2);

            // The push-pull threshold is looser than the Class-A one: in the
            // driver's deep saturation the Jacobian is poorly conditioned and
            // the residual bottoms out around 1e-8..1e-7 (a few nA on currents
            // of mA) -- far below anything audible or representable downstream.
            if (rCe <= 1.0e-9 && rPp <= 1.0e-7)
                continue;

            solveInto (drive, p); // cold re-solve (seeds reset inside)
            ++repairedMain;
            (void) repairedMain; // only read under NEVE_TABLE_DEBUG
        }
        // Micro windows around the remaining limiter kinks (outside the
        // dense window): detect clusters of large second difference in the
        // main table, pad, and build a fine warm-seeded vo sub-table per
        // cluster (outliers re-solved cold, as in the dense window).
        micro.clear();
        const double d2Threshold = 3.0e-5;
        const double pad = 32.0 * hMain;
        int runStart = -1;
        for (int i = 1; i <= tableN; ++i)
        {
            const bool steep = i < tableN - 1
                && std::abs (table[(size_t) i + 1].vo - 2.0 * table[(size_t) i].vo
                             + table[(size_t) i - 1].vo) > d2Threshold;
            if (steep && runStart < 0)
                runStart = i;
            if (! steep && runStart >= 0)
            {
                const double w0 = std::max (tableDMin, tableDMin + (runStart - 1) * hMain - pad);
                const double w1 = std::min (tableDMax, tableDMin + (i + 1) * hMain + pad);
                runStart = -1;
                if (w0 > denseDMin && w1 < denseDMax)
                    continue; // already covered by the dense window
                MicroWindow w { w0, w1, {} };
                w.vo.assign ((size_t) microN, 0.0);
                const double hm = (w1 - w0) / (double) (microN - 1);
                for (int k = 0; k < microN; ++k)
                    w.vo[(size_t) k] = warmVo (w0 + (double) k * hm);
                for (int k = 1; k < microN - 1; ++k)
                {
                    const double d2 = w.vo[(size_t) k + 1] - 2.0 * w.vo[(size_t) k]
                                    + w.vo[(size_t) k - 1];
                    if (std::abs (d2) > 1.0e-5)
                        w.vo[(size_t) k] = coldVo (w0 + (double) k * hm);
                }
                micro.push_back (std::move (w));
            }
        }
#ifdef NEVE_TABLE_DEBUG
        fprintf (stderr, "table build: repaired %zu main, %zu micro windows\n",
                 repairedMain, micro.size());
#endif

        // Precompute the per-knot Catmull-Rom polynomial coefficients for the
        // Exact engine's seed lookup: value(u) between knots i and i+1 is
        //   b + u*((c-a)/2 + u*((2a-5b+4c-d)/2 + u*(3b-a-3c+d)/2))
        // with a..d the clamped neighbouring knots -- the exact polynomial the
        // old on-the-fly reconstruction evaluated, just factored for Horner
        // with c0 = the knot value. Storing c1..c3 (float) turns evalTable
        // into 3 FMAs per field instead of a 4-knot reconstruction.
        for (int f = 0; f < 7; ++f)
            seedCoef[f].assign ((size_t) 3 * tableN, 0.0f);
        for (int i = 0; i < tableN; ++i)
        {
            const auto& pa = table[(size_t) std::max (i - 1, 0)];
            const auto& pb = table[(size_t) i];
            const auto& pc = table[(size_t) std::min (i + 1, tableN - 1)];
            const auto& pd = table[(size_t) std::min (i + 2, tableN - 1)];
            for (int f = 0; f < 7; ++f)
            {
                const double va = tpField (pa, f), vb = tpField (pb, f);
                const double vc = tpField (pc, f), vd = tpField (pd, f);
                float* cf = seedCoef[f].data() + 3 * (size_t) i;
                cf[0] = (float) (0.5 * (vc - va));
                cf[1] = (float) (0.5 * (2.0 * va - 5.0 * vb + 4.0 * vc - vd));
                cf[2] = (float) (0.5 * (3.0 * vb - va - 3.0 * vc + vd));
            }
        }
    }

    // Field accessor for the coefficient build (field order matches the
    // TablePoint member order: vb, ve, vc, pb, pe, pc, vo).
    static double tpField (const TablePoint& p, int f)
    {
        switch (f)
        {
            case 0: return p.vb;
            case 1: return p.ve;
            case 2: return p.vc;
            case 3: return p.pb;
            case 4: return p.pe;
            case 5: return p.pc;
            default: return p.vo;
        }
    }

    // Catmull-Rom interpolation of the table state at `drive`. Smooth (C1),
    // exact at the knots, clamped outside the domain. Uses the per-knot
    // precomputed coefficients when available (3 FMAs per field); identical
    // polynomial to the on-the-fly reconstruction below.
    TablePoint evalTable (double drive) const
    {
        const double t = std::clamp ((drive - tableDMin) * ((tableN - 1) / (tableDMax - tableDMin)),
                                     0.0, (double) (tableN - 1));
        const int i = (int) t;
        const double u = t - (double) i;

        if (seedCoefPtr[0] != nullptr)
        {
            const TablePoint& p1 = table[(size_t) i];
            auto ev = [this, i, u] (int f, double knot)
            {
                const float* cf = seedCoefPtr[f] + 3 * (size_t) i;
                return knot + u * ((double) cf[0] + u * ((double) cf[1] + u * (double) cf[2]));
            };
            TablePoint r;
            r.vb = (float) ev (0, p1.vb);
            r.ve = (float) ev (1, p1.ve);
            r.vc = (float) ev (2, p1.vc);
            r.pb = (float) ev (3, p1.pb);
            r.pe = (float) ev (4, p1.pe);
            r.pc = (float) ev (5, p1.pc);
            r.vo = (float) ev (6, p1.vo);
            return r;
        }

        // Fallback (coefficient build not run -- e.g. the table-build scratch
        // instance): the original 4-knot Catmull-Rom reconstruction.
        const auto p0 = table[(size_t) std::max (i - 1, 0)];
        const auto p1 = table[(size_t) i];
        const auto p2 = table[(size_t) std::min (i + 1, tableN - 1)];
        const auto p3 = table[(size_t) std::min (i + 2, tableN - 1)];

        const double u2 = u * u, u3 = u2 * u;
        auto cr = [u, u2, u3] (double a, double b, double c, double d)
        { return 0.5 * (2.0 * b + (c - a) * u
                          + (2.0 * a - 5.0 * b + 4.0 * c - d) * u2
                          + (3.0 * b - a - 3.0 * c + d) * u3); };

        TablePoint r;
        r.vb = (float) cr (p0.vb, p1.vb, p2.vb, p3.vb);
        r.ve = (float) cr (p0.ve, p1.ve, p2.ve, p3.ve);
        r.vc = (float) cr (p0.vc, p1.vc, p2.vc, p3.vc);
        r.pb = (float) cr (p0.pb, p1.pb, p2.pb, p3.pb);
        r.pe = (float) cr (p0.pe, p1.pe, p2.pe, p3.pe);
        r.pc = (float) cr (p0.pc, p1.pc, p2.pc, p3.pc);
        r.vo = (float) cr (p0.vo, p1.vo, p2.vo, p3.vo);
        return r;
    }

    // High-order (6-point quintic Lagrange) interpolation of the output
    // node for the Fast engine: with a plain cubic, the sharp conduction
    // knee of the transfer curve leaves ~1e-5 absolute interpolation error;
    // quintic drops it below the float32 LSB without densifying the table.
    double evalTableVo (double drive) const
    {
        for (const auto& w : micro)
            if (drive >= w.d0 && drive <= w.d1)
                return quinticVo (w.vo.data(), (int) w.vo.size(), w.d0, w.d1, drive);
        if (drive >= denseDMin && drive <= denseDMax)
            return quinticVo (dense.data(), (int) dense.size(), denseDMin, denseDMax, drive);
        return quinticVoMain (drive);
    }

private:
    double quinticVoMain (double drive) const
    {
        const double tPos = std::clamp ((drive - tableDMin) * ((tableN - 1) / (tableDMax - tableDMin)),
                                        0.0, (double) (tableN - 1));
        const int i = (int) tPos;
        const double u = tPos - (double) i;
        double v[6];
        for (int k = 0; k < 6; ++k)
            v[k] = table[(size_t) std::min (std::max (i + k - 2, 0), tableN - 1)].vo;
        return lagrange6 (v, u);
    }

    static double quinticVo (const double* vo, int n,
                             double d0, double d1, double drive)
    {
        const double tPos = std::clamp ((drive - d0) * ((n - 1) / (d1 - d0)),
                                        0.0, (double) (n - 1));
        const int i = (int) tPos;
        const double u = tPos - (double) i; // in [0,1] between knot i and i+1

        double v[6];
        for (int k = 0; k < 6; ++k)
            v[k] = vo[std::min (std::max (i + k - 2, 0), n - 1)];

        return lagrange6 (v, u);
    }

    static double lagrange6 (const double* v, double u)
    {
        double y = 0.0;
        for (int k = 0; k < 6; ++k)
        {
            double w = 1.0;
            for (int m = 0; m < 6; ++m)
                if (m != k)
                    w *= (u - (double) (m - 2)) / (double) (k - m);
            y += w * v[k];
        }
        return y;
    }

public:

    Engine engine = Engine::Exact;
    std::vector<TablePoint> table;
    std::vector<double> dense; // vo only (Fast engine reads just the output node)
    std::vector<MicroWindow> micro;
    std::vector<float> seedCoef[7]; // built by buildTransferTable, moved into sharedTables
    // Precomputed seed-interpolation coefficients (into sharedTables, which
    // outlives every instance); null until prepare(buildTable=true).
    const float* seedCoefPtr[7] = {};

public:
    //==========================================================================

    static constexpr double transformerRatio = 2.0;  // LO1166, 1:2 step-up
                                                     // (now modelled inside
                                                     // even::TransformerModel)
    static constexpr double inputScale       = 2.5;  // attenuator + drive in
    static constexpr double outputTrim       = 1.0;

    //==========================================================================
    // Class-A common-emitter stage.
    // Nodes: 0 = base, 1 = emitter, 2 = collector.
    // Supply 24 V, self-bias divider Rb1/Rb2, emitter degeneration Re.
    struct CeStage
    {
        double vcc = 24.0;
        double rs  = 1.2e3;   // source / attenuator impedance
        double rb1 = 9.0e3;   // vcc -> base (Thevenin incl. rs to ground);
                              // biased so the collector idles near the mid-
                              // point (~13 V) for even up/down headroom
        double rb2 = 14.0e3;  // base -> gnd
        double rc  = 3.3e3;   // collector load
        double re  = 470.0;   // emitter degeneration
        even::BjtModel q;     // silicon NPN (BC184L family)

        even::NewtonSolver<3>::Vec x {}; // {vb, ve, vc}
    } ce;
    static void ceResidual (const CeStage& s, double vin,
                            const even::NewtonSolver<3>::Vec& x,
                            even::NewtonSolver<3>::Vec& F)
    {
        ceEvaluate (s, vin, x, F, nullptr);
    }

    // Residual and Jacobian in one pass: both need the same BjtModel
    // operating point, so the junction exponentials are evaluated once.
    // J == nullptr means residual-only (line search / verification calls).
    static void ceEvaluate (const CeStage& s, double vin,
                            const even::NewtonSolver<3>::Vec& x,
                            even::NewtonSolver<3>::Vec& F,
                            even::NewtonSolver<3>::Mat* J)
    {
        const double vb = x[0], ve = x[1], vc = x[2];
        const auto op = s.q.eval (vb - ve, vb - vc);

        // Base: source current + bias divider + transistor base current.
        F[0] = (vb - vin) / s.rs + (vb - s.vcc) / s.rb1 + vb / s.rb2 + op.ib;
        // Emitter: emitter resistor carries Ic + Ib.
        F[1] = ve / s.re - (op.ic + op.ib);
        // Collector: current into the collector comes through the load from
        // the supply.
        F[2] = (vc - s.vcc) / s.rc + op.ic;

        if (J == nullptr)
            return;

        auto& j = *J;
        for (auto& row : j) row.fill (0.0);

        // dF0/dv*
        j[0][0] = 1.0 / s.rs + 1.0 / s.rb1 + 1.0 / s.rb2 + op.dib_dvbe + op.dib_dvbc;
        j[0][1] = -op.dib_dvbe;
        j[0][2] = -op.dib_dvbc;
        // dF1/dv*  (F1 = ve/Re - Ic - Ib; vbe = vb-ve, vbc = vb-vc)
        j[1][0] = -(op.dic_dvbe + op.dic_dvbc) - (op.dib_dvbe + op.dib_dvbc);
        j[1][1] = 1.0 / s.re + op.dic_dvbe + op.dib_dvbe;
        // vbc = vb - vc: dvbc/dvc = -1, and F1 subtracts both currents, so the
        // two negatives cancel -> positive sign. Getting this wrong makes the
        // full Newton step an ASCENT direction on ||F|| in deep saturation,
        // which the line search can never repair (the stuck-state bug).
        j[1][2] = op.dic_dvbc + op.dib_dvbc;
        // dF2/dv*  (F2 = (vc-Vcc)/Rc + Ic)
        j[2][0] = op.dic_dvbe + op.dic_dvbc;
        j[2][1] = -op.dic_dvbe;
        j[2][2] = 1.0 / s.rc - op.dic_dvbc;
    }

    CeResult ceSolve (double vin, even::NewtonSolver<3>::Vec& seed) const
    {
        // Under extreme drive a single seed is not enough: continuation from
        // the previous sample diverges when the stage switches operating
        // region (cutoff <-> saturation), and the quiescent point is far from
        // both rails. Strategy, in order:
        //
        //  1. direct continuation from the previous sample's state
        //  2. input homotopy: re-solve at intermediate inputs between the
        //     previous sample's input and this one, each sub-solve seeded from
        //     the last converged state -- this walks the root smoothly through
        //     the region switch
        //  3. physically-motivated seeds (cutoff / saturation / quiescent)
        //
        // The first *converged* result wins; homotopy makes that essentially
        // always happen, so a non-root iterate is never handed to the audio
        // path (which is what produced the jagged, multi-valued clip tops).
        auto solveAt = [stage = &ce] (double v, even::NewtonSolver<3>::Vec x)
        {
            auto ev = [stage, v] (const even::NewtonSolver<3>::Vec& xv,
                                  even::NewtonSolver<3>::Vec& F,
                                  even::NewtonSolver<3>::Mat& J)
            { ceEvaluate (*stage, v, xv, F, &J); };
            auto ro = [stage, v] (const even::NewtonSolver<3>::Vec& xv,
                                  even::NewtonSolver<3>::Vec& F)
            { ceEvaluate (*stage, v, xv, F, nullptr); };
            auto r = even::NewtonSolver<3>::solveCombined (ev, ro, x, 64);
            return std::make_pair (x, r);
        };

        // 1. direct continuation
        auto direct = solveAt (vin, seed);
        if (direct.second.converged)
        {
            seed = direct.first;
            lastCeVin = vin;
            lastResidual = 0.0;
            return { seed[0], seed[1], seed[2] };
        }

        // 2. input homotopy from the previous sample's input: walk the root
        // all the way to this sample's input; each converged sub-solve seeds
        // the next, and only the final substep's convergence is accepted.
        const double v0 = lastCeVin;
        even::NewtonSolver<3>::Vec x = direct.first;
        double bestRes = direct.second.residualNorm;
        even::NewtonSolver<3>::Vec best = direct.first;
        bool haveConverged = false;
        if (std::abs (vin - v0) > 1.0e-12)
        {
            constexpr int nSub = 16;
            for (int k = 1; k <= nSub; ++k)
            {
                const double vk = v0 + (vin - v0) * (double) k / (double) nSub;
                auto sub = solveAt (vk, x);
                x = sub.first;
                if (! sub.second.converged && sub.second.residualNorm < bestRes)
                {
                    bestRes = sub.second.residualNorm;
                    best = sub.first;
                }
            }

            // Verify the final state actually is a root at vin.
            even::NewtonSolver<3>::Vec Fv {};
            ceResidual (ce, vin, x, Fv);
            haveConverged = std::isfinite (Fv[0]) && std::isfinite (Fv[1])
                         && std::isfinite (Fv[2])
                         && even::NewtonSolver<3>::norm (Fv) < 1.0e-6;
            if (haveConverged) best = x;
        }

        // 3. physical seed estimates for the two rails
        if (! haveConverged)
        {
            const double gTot = 1.0 / ce.rs + 1.0 / ce.rb1 + 1.0 / ce.rb2;
            const double vbOff = (vin / ce.rs + ce.vcc / ce.rb1) / gTot;
            const double veSat = ce.re * (ce.vcc - 0.1) / (ce.rc + ce.re);

            const even::NewtonSolver<3>::Vec seeds[3] =
            {
                ceDc,
                { vbOff, 0.0, ce.vcc },
                { veSat + 0.75, veSat, veSat + 0.1 }
            };
            for (const auto& s0 : seeds)
            {
                auto attempt = solveAt (vin, s0);
                if (attempt.second.converged) { best = attempt.first; haveConverged = true; break; }
                if (attempt.second.residualNorm < bestRes) { bestRes = attempt.second.residualNorm; best = attempt.first; }
            }
        }

        // Safety guard only: never overwrite a legitimate root (which can sit
        // far below ground under heavy pad, the source pulls the base through
        // rs against the bias divider). Just ensure the continuation state is
        // finite so one bad sample cannot poison the next.
        if (! haveConverged)
        {
            for (int i = 0; i < 3; ++i)
                if (! std::isfinite (best[(size_t) i]))
                    best[(size_t) i] = ceDc[(size_t) i];
        }

        seed = best;
        lastCeVin = vin;
        // Debug residual: only meaningful for the non-root fallback path; a
        // converged state needs no extra residual evaluation here.
        if (haveConverged)
        {
            lastResidual = 0.0;
        }
        else
        {
            even::NewtonSolver<3>::Vec Fr {};
            ceResidual (ce, vin, seed, Fr);
            lastResidual = even::NewtonSolver<3>::norm (Fr);
        }
        return { seed[0], seed[1], seed[2] };
    }
    //==========================================================================
    // BA283-style output stage: driver + Class-AB complementary pair.
    //
    // Nodes: 0 = driver base, 1 = driver emitter, 2 = driver collector (the
    // bias-spreader drive), 3 = output. The driver is a full Ebers-Moll BJT
    // biased by its own divider (Rb1/Rb2) and degenerated by ReD; the global
    // feedback resistor Rf returns the output to the driver emitter, so
    // driver gain compression progressively fills in the pair's crossover
    // notch as drive increases -- the characteristic BA283 behaviour a
    // bare 2-node pair cannot reproduce. The driver collector feeds the
    // spreader (~2*Vbe) and the pair, whose junction currents use the same
    // reduced exponential model as before.
    //
    // The stage input is the Class-A collector signal minus its DC level
    // (ppInRef): the behavioural equivalent of the interstage coupling, so
    // the driver's divider alone sets the idle point.
    struct PushPullStage
    {
        double vcc = 24.0;     // positive rail (driver collector load)
        double rs   = 600.0;   // drive source impedance
        double rb1  = 33.0e3;  // vcc -> driver base
        double rb2  = 12.1e3;  // driver base -> gnd. Divider + reD set the
                               // driver's idle. reD = 2.4k sets the idle
                               // current (~2.5 mA => ve ~ 6.0 V); the divider
                               // Thevenin (~6.44 V) idles the collector
                               // deliberately slightly below the midpoint of
                               // its swing, giving the single-supply stage its
                               // mild hardware-style asymmetry (clip levels
                               // ~ -5.6 / +6.9, ratio ~1.2:1) and the touch
                               // of even-order harmonic that is part of the
                               // 1073 character. (The original 16.2k put vth
                               // at 7.9 V: only +2.2 V of drive headroom
                               // before driver saturation versus 7.9 V to
                               // cutoff -- the negative half clipped (and
                               // then folded) at less than half the level of
                               // the positive half.)
        double vth  = 24.0 * 12.1e3 / (33.0e3 + 12.1e3); // divider Thevenin
        double reD  = 2.4e3;   // driver emitter degeneration
        double rcD  = 3.6e3;   // driver collector load
        double rf   = 4.7e3;   // global feedback, output -> driver BASE.
                               // Sign matters: the CE->driver->pair chain is
                               // net inverting (the pair is an emitter
                               // follower), so returning NFB to the driver's
                               // EMITTER makes the loop positive (measured:
                               // relaxation oscillation, 44% THD). Summed into
                               // the base node it is negative, and the loop
                               // gain is self-limiting: highest in the pair's
                               // crossover notch (driver gain ~7, pair gm
                               // collapsed -> |T| ~ 0.5, which is exactly the
                               // notch-filling compression) and ~0.12 where
                               // the pair conducts.
        double nfbRef = 0.0;   // output quiescent DC (set in prepare): the
                               // feedback path is AC-coupled through the
                               // output transformer, so only the AC part of
                               // the output reaches the driver base
        double rl   = 600.0;   // load (output transformer primary reflected)
        double bias = 0.62;    // ~2*Vbe/2 spreader drop per half
        double reP  = 4.7;     // output device emitter degeneration. The
                               // pair's raw exponential (gm ~ 0.85 S at the
                               // ~20 mA idle) is a razor-edged crossover the
                               // driver's gain then amplifies into a buzz;
                               // degeneration flattens the turn-on exactly as
                               // the physical emitter resistors do.
        double IsP  = 2.0e-6;  // power-device scaled saturation current
        double beta = 100.0;   // output device beta
        double Vt   = 0.025852;
        // even::BjtModel drv: driver BJT. The 5th initializer is the
        // reverse-junction current cap (BjtModel::irCap): without it, deep
        // driver saturation lets the base->collector current grow unbounded,
        // dragging the collector (and with it the output) back UP -- the
        // transfer curve fold-back that made the negative half of the
        // waveform reverse (double-dip "W") and engage at a fraction of the
        // positive half's level. Capped, the driver bottoms out and the
        // stage clips flat. The cap sits a little above the stage's max
        // legitimate conduction (vcc/rcD ~ 6.7 mA) so normal saturation
        // still behaves physically; only the pathological deep-saturation
        // region is limited.
        even::BjtModel drv { 3.5e-14, 100.0, 4.0, 0.025852, 8.0e-3 };

        // Warm-start caches for the pair's degenerated junction solves: the
        // last (drive argument, root current, dI/dvArg) per junction. Mutable:
        // updated from inside the otherwise-const ppEvaluate. NaN vArg = cold.
        mutable double jnV = std::numeric_limits<double>::quiet_NaN();
        mutable double jnI = 0.0, jnS = 0.0;
        mutable double jpV = std::numeric_limits<double>::quiet_NaN();
        mutable double jpI = 0.0, jpS = 0.0;
    } pp;

    // One degenerated pair junction: solve i - IsP*(ex((vLim(vArg) - i*reP)/Vt) - 1) = 0
    // for the junction current i. Strictly monotone in i (slope 1 + reP*gm >= 1)
    // => unique root, so ANY converged result is THE root regardless of seed --
    // warm starting changes only the iteration count, never the answer.
    //
    // Warm start: the root's slope di/dvArg is smooth in vArg, so a linear
    // extrapolation from the cache's previous (vArg, i) pair lands within a
    // fraction of the degeneration width of the new root and the Newton hits
    // its quadratic phase immediately (1-2 iterations instead of the 5+ a
    // cold undegenerated seed needs). |dV| beyond 4 V (region switch, table
    // build jumps) falls back to the physical cold seed.
    static double juncSolve (const PushPullStage& s,
                             double& cacheV, double& cacheI, double& cacheS,
                             double vArg, double& di_dvArg)
    {
        const double v    = even::BjtModel::lim (vArg);
        const double limd = even::BjtModel::limd (vArg);

        double i;
        const double dV = vArg - cacheV;
        if (dV == dV && std::abs (dV) < 4.0) // NaN compare: cold cache
            i = cacheI + cacheS * dV;
        else
            i = s.IsP * (std::exp (std::min (v / s.Vt, 30.0)) - 1.0);

        for (int it = 0; it < 24; ++it)
        {
            const double x = std::min ((v - i * s.reP) / s.Vt, 30.0);
            double e, d;
            even::BjtModel::exPair (x, e, d);
            const double f    = s.IsP * (e - 1.0);  // target current
            const double gm   = s.IsP / s.Vt * d;   // d(target)/dvj
            const double df   = 1.0 + gm * s.reP;   // d(i - f)/di
            const double step = (i - f) / df;
            i -= step;
            if (std::abs (step) < 1.0e-13)
                break;
        }
        const double x = std::min ((v - i * s.reP) / s.Vt, 30.0);
        double e, d;
        even::BjtModel::exPair (x, e, d);
        const double gm = s.IsP / s.Vt * d;
        di_dvArg = gm * limd / (1.0 + gm * s.reP);
        cacheV = vArg;
        cacheI = i;
        cacheS = di_dvArg;
        return s.IsP * (e - 1.0);
    }

    static void ppResidual (const PushPullStage& s, double vin,
                            const even::NewtonSolver<4>::Vec& x,
                            even::NewtonSolver<4>::Vec& F)
    {
        ppEvaluate (s, vin, x, F, nullptr);
    }

    // Residual and Jacobian in one pass: both use the same junction limiting
    // and exponential evaluations. J == nullptr means residual-only.
    static void ppEvaluate (const PushPullStage& s, double vin,
                            const even::NewtonSolver<4>::Vec& x,
                            even::NewtonSolver<4>::Vec& F,
                            even::NewtonSolver<4>::Mat* J)
    {
        const double vb = x[0], ve = x[1], vc = x[2], vo = x[3];

        // Driver: full Ebers-Moll (forward + reverse junction), so its
        // saturation -- the collector bottoming against the emitter under
        // drive -- is modelled properly. That saturation is the stage's
        // gain-compression mechanism.
        const auto op = s.drv.eval (vb - ve, vb - vc);

        // Pair junctions, driven from the driver collector through the
        // spreader offset (same reduced exponential model as before), each
        // degenerated by its emitter resistor. The degenerated junction
        // current i = IsP*(ex((vLim - i*reP)/Vt) - 1) is a monotone scalar
        // equation (unique root, d/di = 1 + reP*gm > 0); it is solved by a
        // short inner Newton and its analytic slope feeds the outer Jacobian.
        const double uN = vc - vo - s.bias;
        double diN, diP;
        const double iN = juncSolve (s, s.jnV, s.jnI, s.jnS,  uN, diN); // sources output node
        const double iP = juncSolve (s, s.jpV, s.jpI, s.jpS, -uN, diP); // sinks output node
        const double iSum = iN - iP;
        const double gb = 1.0 / (1.0 + s.beta);

        // Driver base: the drive is coupled in DC-free (interstage coupling),
        // so the source is referenced at the divider's Thevenin voltage: it
        // sets the idle point with rs carrying no DC current, while keeping
        // the rs source impedance at signal frequencies.
        F[0] = (vb - vin - s.vth) / s.rs + (vb - s.vcc) / s.rb1 + vb / s.rb2
             + (vb + s.nfbRef - vo) / s.rf + op.ib;
        // Driver emitter: degeneration to ground carries the emitter current
        // (ic + ib). The global feedback returns to the BASE node (see the
        // rf note above), not here.
        F[1] = ve / s.reD - (op.ic + op.ib);
        // Driver collector: load from the supply, collector current, and the
        // pair's base current drawn through the spreader.
        F[2] = (vc - s.vcc) / s.rcD + op.ic + iSum * gb;

        // Collector saturation floor. A real BJT's collector cannot sink
        // below its emitter (Vce bottoms at Vce_sat ~ 0.2 V: the internal
        // base->collector junction conducts hard and clamps the collector to
        // the emitter). The junction limiter's 0.08-slope tail suppresses
        // exactly that conduction, which admits nonphysical roots in deep
        // saturation where the collector sits BELOW the emitter and then
        // tracks the base upward as drive grows -- the transfer-curve
        // fold-back that reversed one half of the waveform (the double-dip
        // "W"). This term re-clamps the collector smoothly: current injected
        // into the collector node grows smoothly once vc tries to fall below
        // ve + vceSat. C1 everywhere (soft-abs), Jacobian consistent.
        const double vceSat = 0.25;
        const double over   = (ve + vceSat) - vc;   // > 0 below the floor
        const double w      = 0.03;                 // soft-knee width [V]
        const double root   = std::sqrt (over * over + w * w);
        const double sSat   = 0.5 * (over + root);  // smooth max(0, over)
        const double dsd    = 0.5 * (1.0 + over / root);
        constexpr double kSat = 0.08;               // clamp conductance [S]
        F[2] -= kSat * sSat;
        // Output node: device current must feed load + feedback.
        F[3] = -iSum + vo / s.rl + (vo - s.nfbRef - vb) / s.rf;

        if (J == nullptr)
            return;

        // Driver derivative shorthand (w.r.t. vbe = vb-ve, vbc = vb-vc).
        const double ibB = op.dib_dvbe, ibC = op.dib_dvbc;
        const double icB = op.dic_dvbe, icC = op.dic_dvbc;
        // Pair transconductance: diSum/dvc = g, diSum/dvo = -g (degenerated
        // junction slopes; both junctions share the differential drive uN).
        const double g = diN + diP;

        auto& j = *J;
        for (auto& row : j) row.fill (0.0);

        j[0][0] = 1.0 / s.rs + 1.0 / s.rb1 + 1.0 / s.rb2 + 1.0 / s.rf + ibB + ibC;
        j[0][1] = -ibB;
        j[0][2] = -ibC;
        j[0][3] = -1.0 / s.rf;
        j[1][0] = -(icB + icC) - (ibB + ibC);
        j[1][1] = 1.0 / s.reD + icB + ibB;
        j[1][2] = icC + ibC;
        j[2][0] = icB + icC;
        j[2][1] = -icB - kSat * dsd;
        j[2][2] = 1.0 / s.rcD - icC + g * gb + kSat * dsd;
        j[2][3] = -g * gb;
        j[3][0] = -1.0 / s.rf;
        j[3][2] = -g;
        j[3][3] = g + 1.0 / s.rl + 1.0 / s.rf;
    }

    PpState ppSolve (double vin, even::NewtonSolver<4>::Vec& seed) const
    {
        auto evaluate = [&] (const even::NewtonSolver<4>::Vec& x,
                             even::NewtonSolver<4>::Vec& F,
                             even::NewtonSolver<4>::Mat& J)
        { ppEvaluate (pp, vin, x, F, &J); };
        auto residualOnly = [&] (const even::NewtonSolver<4>::Vec& x,
                                 even::NewtonSolver<4>::Vec& F)
        { ppEvaluate (pp, vin, x, F, nullptr); };

        auto firstAttempt = seed;
        auto r1 = even::NewtonSolver<4>::solveCombined (evaluate, residualOnly, firstAttempt);

        // Same nonphysical-iterate guard as the Class-A stage: retry from the
        // quiescent state, but prefer a *converged* attempt over one that
        // merely has a smaller (still far from zero) residual.
        if (! r1.converged)
        {
            auto retry = ppDcQuiescent;
            auto r2 = even::NewtonSolver<4>::solveCombined (evaluate, residualOnly, retry, 64);
            if (r2.converged)
            {
                firstAttempt = retry;
                r1 = r2;
            }
            else if (r2.residualNorm < r1.residualNorm)
            {
                firstAttempt = retry;
                r1 = r2;
            }
        }

        seed = firstAttempt;
        // Physical range guards: the driver and output nodes have no path
        // outside the supply-referenced range in any real root (the output
        // node is fed by junction current sources into rl/rf only).
        for (int i = 0; i < 4; ++i)
            seed[(size_t) i] = std::clamp (seed[(size_t) i], -20.0, pp.vcc + 1.0);
        return { seed[0], seed[1], seed[2], seed[3] };
    }

    //==========================================================================
    static double pole (double hz, double fs) { return std::exp (-2.0 * 3.14159265358979 * hz / fs); } // (retired with the one-pole chain)

    double fs = 48000.0;
    float  driveGain = 1.0f;
    double driveScale = inputScale; // driveGain * inputScale, set by setGainDb

    double normalization = 1.0;

    // Physical transformer models (input LO1166 / output BA283 iron).
    even::TransformerModel inTf, outTf;

    even::NewtonSolver<3>::Vec dcSolution {};
    even::NewtonSolver<3>::Vec ceDc {};         // quiescent fallback seed
    even::NewtonSolver<4>::Vec ppDc {};
    even::NewtonSolver<4>::Vec ppDcQuiescent {}; // quiescent fallback seed
    double ppQuiescentVo = 0.0;
    double ppInRef = 13.0;                      // Class-A collector DC level
    mutable double lastCeVin = 0.0;             // previous input for homotopy
    mutable double lastResidual = 0.0;          // debug: accepted state's residual
};
