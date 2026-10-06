#include <cstdio>
#include <cmath>
#include <algorithm>
#include <array>
#include "even/BjtModel.h"
#include "even/NewtonSolver.h"

// replicate CE residual/jac (post-fix limiter)
struct Stage { double vcc=24, rs=1.2e3, rb1=9e3, rb2=14e3, rc=3.3e3, re=470; even::BjtModel q; };
Stage s;
void F (double vin, const std::array<double,3>& x, std::array<double,3>& f)
{
    const double vb=x[0], ve=x[1], vc=x[2];
    const auto op = s.q.eval (vb-ve, vb-vc);
    f[0] = (vb-vin)/s.rs + (vb-s.vcc)/s.rb1 + vb/s.rb2 + op.ib;
    f[1] = ve/s.re - (op.ic + op.ib);
    f[2] = (vc-s.vcc)/s.rc + op.ic;
}
void J (const std::array<double,3>& x, std::array<std::array<double,3>,3>& m)
{
    const auto op = s.q.eval (x[0]-x[1], x[0]-x[2]);
    for (auto& r : m) r.fill (0.0);
    m[0][0] = 1.0/s.rs + 1.0/s.rb1 + 1.0/s.rb2 + op.dib_dvbe + op.dib_dvbc;
    m[0][1] = -op.dib_dvbe;
    m[0][2] = -op.dib_dvbc;
    m[1][0] = -(op.dic_dvbe + op.dic_dvbc) - (op.dib_dvbe + op.dib_dvbc);
    m[1][1] = 1.0/s.re + op.dic_dvbe + op.dib_dvbe;
    m[1][2] = op.dic_dvbc + op.dib_dvbc;
    m[2][0] = op.dic_dvbe + op.dic_dvbc;
    m[2][1] = -op.dic_dvbe;
    m[2][2] = 1.0/s.rc - op.dic_dvbc;
}
int main()
{
    std::array<double,3> q0 = { 2.614, 1.5753, 12.9761 };
    for (double vin : { -250.0, -100.0, -50.0, -20.0, -10.0, -5.0, 5.0, 10.0, 20.0, 50.0, 100.0, 250.0 })
    {
        auto seed = q0;
        auto r = even::NewtonSolver<3>::solve (
            [&] (const auto& x, auto& f) { F (vin, x, f); },
            [&] (const auto& x, auto& m) { J (x, m); }, seed, 64);
        std::array<double,3> f {}; F (vin, seed, f);
        printf ("vin=%+6.1f conv=%d res=%9.2e | vb=%+9.3f ve=%+8.4f vc=%+8.4f\n",
                vin, (int) r.converged, r.residualNorm, seed[0], seed[1], seed[2]);
    }
    return 0;
}
