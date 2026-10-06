#include <cstdio>
#include <cmath>
#include <algorithm>
#include <array>
#include "even/BjtModel.h"

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
    for (auto x : { std::array<double,3>{-0.43, 0.49, 17.5}, std::array<double,3>{3.0, 0.5, 17.5}, std::array<double,3>{0.65, 0.05, 19.5} })
    {
        const double vin = 0.495;
        std::array<std::array<double,3>,3> m {};
        J (x, m);
        std::array<double,3> f0 {}; F (vin, x, f0);
        printf ("state vb=%+.2f ve=%.2f vc=%.2f | F=[%9.2e %9.2e %9.2e]\n", x[0], x[1], x[2], f0[0], f0[1], f0[2]);
        for (int j = 0; j < 3; ++j)
        {
            const double h = 1e-6;
            auto xh = x; xh[(size_t) j] += h;
            std::array<double,3> fh {}; F (vin, xh, fh);
            for (int i = 0; i < 3; ++i)
            {
                const double fd = (fh[(size_t) i] - f0[(size_t) i]) / h;
                const double an = m[(size_t) i][(size_t) j];
                if (std::abs (fd - an) > 1e-4 * std::max (1.0, std::abs (an)))
                    printf ("   MISMATCH dF[%d]/dx[%d]: analytic=%9.3e fd=%9.3e\n", i, j, an, fd);
            }
        }
    }
    printf ("jac check done\n");
    return 0;
}
