#include <cstdio>
#include <cmath>
#include <algorithm>
#include "Neve1073Circuit.h"

int main()
{
    const double fs = 48000.0;
    for (float gain : { 0.0f, -40.0f })
      for (float amp : { 0.25f, 0.7f, 1.0f })
    {
        Neve1073Circuit c;
        c.prepare (fs);
        c.setGainDb (gain);
        const int n = (int) (fs * 0.1);
        const double freq = 20.0;
        const double slewLimit = 8.0 * amp * 2.0 * 3.14159265358979 * freq / fs;
        double maxStep = 0.0, prevY = 0.0, peak = 0.0; int glitches = 0; bool nan = false;
        double worstRes = 0.0;
        for (int i = 0; i < n; ++i)
        {
            const double x = amp * std::sin (2.0 * 3.14159265358979 * freq * i / fs);
            const float y = c.processSample ((float) x);
            if (! std::isfinite (y)) nan = true;
            if (i > n / 5)
            {
                const double step = std::abs ((double) y - prevY);
                if (step > slewLimit) ++glitches;
                maxStep = std::max (maxStep, step);
                peak = std::max (peak, std::abs ((double) y));
                worstRes = std::max (worstRes, std::abs (c.debugState().res));
            }
            prevY = y;
        }
        printf ("gain=%5.0f amp=%.2f : glitches=%3d maxStep=%.2e (lim %.2e) peak=%.3f worstRes=%.1e nan=%d\n",
                gain, amp, glitches, maxStep, slewLimit, peak, worstRes, (int) nan);
    }
    return 0;
}
