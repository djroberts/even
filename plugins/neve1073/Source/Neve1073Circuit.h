#pragma once

#include <cmath>
#include <algorithm>
#include <limits>
#include <even/NewtonSolver.h>
#include <even/BjtModel.h>

//==============================================================================
// Neve 1073 preamp circuit model.
//
// Signal path modelled after the 1073 module schematic:
//
//   Marinair LO1166 input transformer (1:2 step-up)
//     -> sensitivity/attenuator network (the gain knob)
//     -> Class-A common-emitter voltage amplifier (BA215/BA284 style, self-
//        biased through a divider, emitter degenerated)  [3-node MNA + Newton]
//     -> BA283 output amplifier: Class-AB complementary emitter-follower pair
//        (BC184L NPN / BC214L PNP) with ~2*Vbe bias spreader, local feedback,
//        driving the output transformer  [2-node MNA + Newton]
//
// Junction capacitances are folded out (audio-band algebraic model); coupling
// and transformer behaviour are handled by the surrounding one-pole filters,
// which also carry the model's memory between samples.
class Neve1073Circuit
{
public:
    struct CeResult { double vb, ve, vc; };
    struct PpState  { double vd, vo; };

    void prepare (double sampleRate)
    {
        fs = sampleRate;

        // DC operating point of the Class-A stage (signal source at 0 V),
        // starting from a rough hand-computed bias estimate.
        dcSolution = { 2.9, 1.8, 13.0 };
        const auto mid = ceSolve (0.0, dcSolution);
        ceDc = dcSolution; // physical fallback seed for unconverged solves

        // Output stage gain around its quiescent collector voltage. Seed the
        // solve from the physical quiescent state: the pair idles just at the
        // edge of conduction, output ~0.62 V below the drive node.
        ppDc = { mid.vc - 0.05, mid.vc - 0.67 };
        ppSolve (mid.vc, ppDc);
        ppDcQuiescent = ppDc; // physical fallback seed for unconverged solves
        ppQuiescentVo = ppDc[1];

        // One-pole coefficients (transformer / coupling behaviour). These must
        // be set BEFORE the calibration run below, which exercises the real
        // signal chain (processSample depends on them).
        inputHp  = pole (12.0, fs);    // input transformer LF roll-off
        outputHp = pole (8.0, fs);     // output transformer / coupling cap
        outputLp = pole (48000.0, fs); // gentle HF loss of the iron & silicon

        // Calibrate normalization by running the real chain: with unity
        // normalization, measure the output for a -12 dBFS-ish sine and set
        // the trim so 0 dB sensitivity passes it at unity.
        normalization = 1.0;
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

        // The calibration run above exercises the real signal chain, which
        // drags the continuation states to wherever the last sine sample
        // landed. Restore them to the quiescent operating point so playback
        // starts from the DC bias, not a stale signal snapshot.
        dcSolution = ceDc;
        ppDc = ppDcQuiescent;
        lastCeVin = 0.0;
        inputHpZ = outputHpZ = outputLpZ = 0.0;
    }

    // Debug hook (harness only): current DC operating point and calibration.
    struct DebugState { double vb, ve, vc, vd, vo, norm; double res; };
    DebugState debugState() const { return { dcSolution[0], dcSolution[1], dcSolution[2], ppDc[0], ppDc[1], normalization, lastResidual }; }

    // gainKnob: -80..+10 dB, exactly like the 1073 sensitivity control.
    // Turning it *down* records hotter -> more drive into the saturation
    // stages; turning it up pads the input. 40 dB/knob-decade keeps the
    // sweep musically matched to the hardware's 5 dB steps.
    void setGainDb (float db) { driveGain = std::pow (10.0f, -db / 40.0f); }

    float processSample (float in)
    {
        // --- Input transformer: step-up + LF pole (state-variable form) ---
        const double x = (double) in * transformerRatio;
        inputHpZ = inputHp * inputHpZ + (1.0 - inputHp) * x;
        const double xf = x - inputHpZ;                       // AC-coupled
        const double drive = xf * (double) driveGain * inputScale;

        // --- Class-A gain stage (3-node Newton nodal solve) ---
        const auto op = ceSolve (drive, dcSolution);

        // --- BA283 Class-AB output stage (2-node Newton nodal solve) ---
        const auto out = ppSolve (op.vc, ppDc);

        // --- Output transformer: remove DC, shape band ---
        double y = out.vo - ppQuiescentVo;
        outputHpZ = outputHp * outputHpZ + (1.0 - outputHp) * y;
        y = y - outputHpZ;
        outputLpZ = outputLp * outputLpZ + (1.0 - outputLp) * y;

        return (float) (outputLpZ * normalization * outputTrim);
    }

private:
    static constexpr double transformerRatio = 2.0;  // LO1166, 1:2 step-up
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
        const double vb = x[0], ve = x[1], vc = x[2];
        const auto op = s.q.eval (vb - ve, vb - vc);

        // Base: source current + bias divider + transistor base current.
        F[0] = (vb - vin) / s.rs + (vb - s.vcc) / s.rb1 + vb / s.rb2 + op.ib;
        // Emitter: emitter resistor carries Ic + Ib.
        F[1] = ve / s.re - (op.ic + op.ib);
        // Collector: current into the collector comes through the load from
        // the supply.
        F[2] = (vc - s.vcc) / s.rc + op.ic;
    }

    static void ceJacobian (const CeStage& s,
                            const even::NewtonSolver<3>::Vec& x,
                            even::NewtonSolver<3>::Mat& J)
    {
        const auto op = s.q.eval (x[0] - x[1], x[0] - x[2]);

        for (auto& row : J) row.fill (0.0);

        // dF0/dv*
        J[0][0] = 1.0 / s.rs + 1.0 / s.rb1 + 1.0 / s.rb2 + op.dib_dvbe + op.dib_dvbc;
        J[0][1] = -op.dib_dvbe;
        J[0][2] = -op.dib_dvbc;
        // dF1/dv*  (F1 = ve/Re - Ic - Ib; vbe = vb-ve, vbc = vb-vc)
        J[1][0] = -(op.dic_dvbe + op.dic_dvbc) - (op.dib_dvbe + op.dib_dvbc);
        J[1][1] = 1.0 / s.re + op.dic_dvbe + op.dib_dvbe;
        // vbc = vb - vc: dvbc/dvc = -1, and F1 subtracts both currents, so the
        // two negatives cancel -> positive sign. Getting this wrong makes the
        // full Newton step an ASCENT direction on ||F|| in deep saturation,
        // which the line search can never repair (the stuck-state bug).
        J[1][2] = op.dic_dvbc + op.dib_dvbc;
        // dF2/dv*  (F2 = (vc-Vcc)/Rc + Ic)
        J[2][0] = op.dic_dvbe + op.dic_dvbc;
        J[2][1] = -op.dic_dvbe;
        J[2][2] = 1.0 / s.rc - op.dic_dvbc;
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
        auto solveAt = [] (auto& res, auto& jv, double v,
                           even::NewtonSolver<3>::Vec x)
        {
            auto r = even::NewtonSolver<3>::solve (
                [&] (const even::NewtonSolver<3>::Vec& xv,
                     even::NewtonSolver<3>::Vec& F) { res (xv, F, v); },
                [&] (const even::NewtonSolver<3>::Vec& xv,
                     even::NewtonSolver<3>::Mat& J) { jv (xv, J, v); },
                x, 64);
            return std::make_pair (x, r);
        };

        auto residual = [&] (const even::NewtonSolver<3>::Vec& x,
                             even::NewtonSolver<3>::Vec& F, double v)
        { ceResidual (ce, v, x, F); };
        auto jac = [&] (const even::NewtonSolver<3>::Vec& x,
                        even::NewtonSolver<3>::Mat& J, double)
        { ceJacobian (ce, x, J); };

        // 1. direct continuation
        auto direct = solveAt (residual, jac, vin, seed);
        if (direct.second.converged)
        {
            seed = direct.first;
            lastCeVin = vin;
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
                auto sub = solveAt (residual, jac, vk, x);
                x = sub.first;
                if (! sub.second.converged && sub.second.residualNorm < bestRes)
                {
                    bestRes = sub.second.residualNorm;
                    best = sub.first;
                }
            }

            // Verify the final state actually is a root at vin.
            even::NewtonSolver<3>::Vec Fv {};
            residual (x, Fv, vin);
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
                auto attempt = solveAt (residual, jac, vin, s0);
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
        {
            even::NewtonSolver<3>::Vec Fr {};
            residual (seed, Fr, vin);
            lastResidual = even::NewtonSolver<3>::norm (Fr);
        }
        return { seed[0], seed[1], seed[2] };
    }
    //==========================================================================
    // BA283-style output stage: Class-AB complementary pair.
    // Nodes: 0 = drive, 1 = output. Bias spreader ~2*Vbe splits the pair;
    // each half conducts around its junction voltage. Local feedback
    // resistor Rf from output back to the drive node.
    struct PushPullStage
    {
        double rs   = 600.0;   // drive source impedance
        double rf   = 6.8e3;   // local feedback
        double rl   = 600.0;   // load (output transformer primary reflected)
        double bias = 0.62;    // ~2*Vbe/2 spreader drop per half
        double IsP  = 2.0e-6;  // power-device scaled saturation current
        double beta = 100.0;
        double Vt   = 0.025852;
    } pp;

    static void ppResidual (const PushPullStage& s, double vin,
                            const even::NewtonSolver<2>::Vec& x,
                            even::NewtonSolver<2>::Vec& F)
    {
        const double vd = x[0], vo = x[1];
        const double vN = even::BjtModel::lim (vd - vo - s.bias); // NPN junction
        const double vP = even::BjtModel::lim (vo - vd - s.bias); // PNP junction
        const double lN = even::BjtModel::limd (vd - vo - s.bias);
        const double lP = even::BjtModel::limd (vo - vd - s.bias);
        const double eN = even::BjtModel::ex (vN / s.Vt);
        const double eP = even::BjtModel::ex (vP / s.Vt);
        const double iN = s.IsP * (eN - 1.0);            // sources output node
        const double iP = s.IsP * (eP - 1.0);            // sinks output node
        const double iSum = iN - iP;

        // Drive node: input current + feedback current + base currents.
        F[0] = (vd - vin) / s.rs + (vd - vo) / s.rf + iSum / (1.0 + s.beta);
        // Output node: device current must feed load + feedback return.
        F[1] = -iSum + vo / s.rl + (vo - vd) / s.rf;
    }

    static void ppJacobian (const PushPullStage& s,
                            const even::NewtonSolver<2>::Vec& x,
                            even::NewtonSolver<2>::Mat& J)
    {
        const double vd = x[0], vo = x[1];
        const double vN = even::BjtModel::lim (vd - vo - s.bias);
        const double vP = even::BjtModel::lim (vo - vd - s.bias);
        const double lN = even::BjtModel::limd (vd - vo - s.bias);
        const double lP = even::BjtModel::limd (vo - vd - s.bias);
        const double gN = s.IsP / s.Vt * even::BjtModel::exd (vN / s.Vt) * lN;
        const double gP = s.IsP / s.Vt * even::BjtModel::exd (vP / s.Vt) * lP;
        const double gb = 1.0 / (1.0 + s.beta);

        for (auto& row : J) row.fill (0.0);

        // diSum/dvd = gN + gP ; diSum/dvo = -(gN + gP)
        const double g = gN + gP;

        J[0][0] = 1.0 / s.rs + 1.0 / s.rf + g * gb;
        J[0][1] = -1.0 / s.rf - g * gb;
        J[1][0] = -g - 1.0 / s.rf;
        J[1][1] = g + 1.0 / s.rl + 1.0 / s.rf;
    }

    PpState ppSolve (double vin, even::NewtonSolver<2>::Vec& seed) const
    {
        auto residual = [&] (const even::NewtonSolver<2>::Vec& x,
                             even::NewtonSolver<2>::Vec& F)
        { ppResidual (pp, vin, x, F); };
        auto jac = [&] (const even::NewtonSolver<2>::Vec& x,
                        even::NewtonSolver<2>::Mat& J)
        { ppJacobian (pp, x, J); };

        auto firstAttempt = seed;
        auto r1 = even::NewtonSolver<2>::solve (residual, jac, firstAttempt);

        // Same nonphysical-iterate guard as the Class-A stage: retry from the
        // quiescent state, but prefer a *converged* attempt over one that
        // merely has a smaller (still far from zero) residual.
        if (! r1.converged)
        {
            auto retry = ppDcQuiescent;
            auto r2 = even::NewtonSolver<2>::solve (residual, jac, retry, 64);
            if (r2.converged && ! r1.converged)
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
        // The output node is fed by junction current sources into rl/rf; it
        // has no path outside the supply-referenced range in any real root.
        seed[0] = std::clamp (seed[0], -1.0, pp.bias + 24.0 + 1.0);
        seed[1] = std::clamp (seed[1], -1.0, 24.0 + 1.0);
        return { seed[0], seed[1] };
    }

    //==========================================================================
    static double pole (double hz, double fs) { return std::exp (-2.0 * 3.14159265358979 * hz / fs); }

    double fs = 48000.0;
    float  driveGain = 1.0f;

    double inputHp = 0.0, inputHpZ = 0.0;
    double outputHp = 0.0, outputHpZ = 0.0;
    double outputLp = 0.0, outputLpZ = 0.0;
    double normalization = 1.0;

    even::NewtonSolver<3>::Vec dcSolution {};
    even::NewtonSolver<3>::Vec ceDc {};         // quiescent fallback seed
    even::NewtonSolver<2>::Vec ppDc {};
    even::NewtonSolver<2>::Vec ppDcQuiescent {}; // quiescent fallback seed
    double ppQuiescentVo = 0.0;
    mutable double lastCeVin = 0.0;             // previous input for homotopy
    mutable double lastResidual = 0.0;          // debug: accepted state's residual
};
