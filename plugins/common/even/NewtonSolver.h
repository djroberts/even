#pragma once

#include <array>
#include <cmath>

namespace even
{

//==============================================================================
// Newton-Raphson solver for F(x) = 0 with a damped (backtracking line search)
// update and closed-form Gaussian elimination with partial pivoting.
// This is the nonlinear engine behind the nodal (DK-method) circuit models.
template <int N>
class NewtonSolver
{
public:
    using Vec = std::array<double, N>;
    using Mat = std::array<std::array<double, N>, N>;

    struct Result
    {
        bool converged = false;
        int iterations = 0;
        double residualNorm = 0.0;
    };

    template <typename Residual, typename Jacobian>
    static Result solve (const Residual& residual, const Jacobian& jacobian,
                         Vec& x, int maxIterations = 16, double tol = 1e-10)
    {
        // Two-callback form: residual and Jacobian evaluated separately. Kept
        // for compatibility; solveCombined below is preferred in hot paths
        // because it evaluates residual + Jacobian in a single pass.
        auto evaluate = [&] (const Vec& xv, Vec& F, Mat& J)
        {
            residual (xv, F);
            jacobian (xv, J);
        };
        return solveCombined (evaluate, residual, x, maxIterations, tol);
    }

    // Combined form: `evaluate` fills the residual AND the Jacobian at the
    // same iterate, so shared transcendental work (BjtModel junction
    // exponentials etc.) is computed once instead of twice per Newton
    // iteration. `residualOnly` is used for line-search candidates, where no
    // Jacobian is needed. Numerically identical to the two-callback solve.
    template <typename Evaluate, typename Residual>
    static Result solveCombined (const Evaluate& evaluate, const Residual& residual,
                                 Vec& x, int maxIterations = 16, double tol = 1e-10)
    {
        Result r;
        Vec F {}, xNew {}, FNew {}, dx {};
        Mat J {};

        // A wild Newton step (or a poisoned seed) can overflow the device
        // exponentials to inf, which propagates NaN through the residual.
        // Treat any non-finite residual as a rejected candidate.
        auto finite = [] (const Vec& v)
        {
            for (auto e : v) if (! std::isfinite (e)) return false;
            return true;
        };

        for (int iter = 0; iter < maxIterations; ++iter)
        {
            evaluate (x, F, J); // residual + Jacobian in one pass
            if (! finite (F))
                return r; // unconverged; caller falls back to a physical seed
            r.residualNorm = norm (F);
            r.iterations = iter;

            if (r.residualNorm < tol)
            {
                r.converged = true;
                return r;
            }

            if (! solveLinear (J, F, dx))
                return r; // singular Jacobian, keep last x

            bool improved = false;
            double lambda = 1.0;
            for (int ls = 0; ls < 8; ++ls)
            {
                for (int i = 0; i < N; ++i)
                    xNew[i] = x[i] - lambda * dx[i];

                residual (xNew, FNew);
                if (finite (FNew) && norm (FNew) < r.residualNorm)
                {
                    improved = true;
                    break;
                }
                lambda *= 0.5;
            }

            if (! improved)
            {
                // Line search exhausted: taking the full Newton step can fling
                // a node hundreds of volts away on a saturated/lumped
                // exponential residual (after which the junction limiter
                // blinds the iteration). Instead take the most-damped
                // candidate that was evaluated -- a tiny but finite step that
                // keeps the iteration moving and stays physical.
                bool ok = true;
                for (int i = 0; i < N; ++i)
                    if (! std::isfinite (xNew[i])) { ok = false; break; }
                if (! ok) return r;
            }

            x = xNew;
        }

        residual (x, F);
        if (! finite (F)) return r;
        r.residualNorm = norm (F);
        r.iterations = maxIterations;
        r.converged = r.residualNorm < 1e-6;
        return r;
    }

    static double norm (const Vec& v)
    {
        double s = 0.0;
        for (auto e : v) s += e * e;
        return std::sqrt (s);
    }

    // Solves J * dx = b (b is overwritten by callers as needed). Returns false
    // if the matrix is numerically singular.
    static bool solveLinear (Mat A, Vec b, Vec& dx)
    {
        for (int col = 0; col < N; ++col)
        {
            int pivot = col;
            double best = std::abs (A[(size_t) col][(size_t) col]);
            for (int row = col + 1; row < N; ++row)
                if (std::abs (A[(size_t) row][(size_t) col]) > best)
                {
                    best = std::abs (A[(size_t) row][(size_t) col]);
                    pivot = row;
                }

            if (best < 1e-18)
                return false;

            if (pivot != col)
            {
                std::swap (A[(size_t) pivot], A[(size_t) col]);
                std::swap (b[(size_t) pivot], b[(size_t) col]);
            }

            const double d = A[(size_t) col][(size_t) col];
            for (int row = col + 1; row < N; ++row)
            {
                const double m = A[(size_t) row][(size_t) col] / d;
                if (m == 0.0) continue;
                for (int k = col; k < N; ++k)
                    A[(size_t) row][(size_t) k] -= m * A[(size_t) col][(size_t) k];
                b[(size_t) row] -= m * b[(size_t) col];
            }
        }

        for (int row = N - 1; row >= 0; --row)
        {
            double s = b[(size_t) row];
            for (int k = row + 1; k < N; ++k)
                s -= A[(size_t) row][(size_t) k] * dx[(size_t) k];
            dx[(size_t) row] = s / A[(size_t) row][(size_t) row];
        }

        return true;
    }
};

} // namespace even
