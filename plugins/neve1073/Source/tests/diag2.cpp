#include <cstdio>
#include <cmath>
#include <algorithm>
#include <vector>
#include "Neve1073Circuit.h"
int main()
{
    const double fs = 48000.0;
    for (float gain : { 0.0f, -10.0f, -20.0f, -40.0f })
      for (float amp : { 0.25f, 0.7f, 1.0f })
    {
        Neve1073Circuit c;
        c.prepare (fs);
        c.setGainDb (gain);
        const int n = (int) fs;
        std::vector<double> y; y.reserve (n);
        for (int i = 0; i < n; ++i)
            y.push_back (c.processSample (amp * std::sin (2.0 * 3.14159265358979 * 20.0 * i / fs)));
        int disc = 0; double worstRatio = 0.0; bool nan = false;
        for (int i = n / 5 + 10; i < n - 10; ++i)
        {
            if (! std::isfinite (y[(size_t) i])) { nan = true; continue; }
            double local = 0; int cnt = 0;
            for (int j = i - 10; j <= i + 10; ++j) if (j != i && j != i - 1) { local += std::abs (y[(size_t) j] - y[(size_t) j - 1]); ++cnt; }
            local /= std::max (1, cnt);
            const double step = std::abs (y[(size_t) i] - y[(size_t) i - 1]);
            if (step > 1e-3 && step > 6.0 * local)
            {
                ++disc;
                worstRatio = std::max (worstRatio, step / std::max (local, 1e-12));
            }
        }
        printf ("gain=%5.0f amp=%.2f : discontinuities=%3d worstStep/local=%.1f nan=%d\n", gain, amp, disc, worstRatio, (int) nan);
    }
    return 0;
}
