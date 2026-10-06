#include <cstdio>
#include <cmath>
#include <algorithm>
#include <array>
#include "even/BjtModel.h"
#include "even/NewtonSolver.h"
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
    const double vin = 5.0;
    std::array<double,3> x = { 4.683, 3.026, 3.5427 };
    std::array<double,3> f {}; F (vin, x, f);
    printf ("F = [%9.2e %9.2e %9.2e] norm=%.3e\n", f[0], f[1], f[2], even::NewtonSolver<3>::norm(f));
    std::array<std::array<double,3>,3> m {}; J (x, m);
    for (int i=0;i<3;++i) printf ("J[%d] = [%9.2e %9.2e %9.2e]\n", i, m[i][0], m[i][1], m[i][2]);
    auto dx = f; // solve J dx = F (we want x - step)
    even::NewtonSolver<3>::solveLinear (m, dx, dx);
    printf ("dx = [%9.2e %9.2e %9.2e]\n", dx[0], dx[1], dx[2]);
    for (double lam : {1.0, 0.5, 0.1, 0.01})
    {
        std::array<double,3> xn { x[0]-lam*dx[0], x[1]-lam*dx[1], x[2]-lam*dx[2] };
        std::array<double,3> fn {}; F (vin, xn, fn);
        printf ("lam=%.2f -> x=[%8.3f %8.3f %8.3f] Fnorm=%.3e\n", lam, xn[0], xn[1], xn[2], even::NewtonSolver<3>::norm(fn));
    }
    return 0;
}
