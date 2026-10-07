#include <cstdio>
#include <cmath>
#include <algorithm>
#include <vector>
#include "Neve1073Circuit.h"

// Offline harness: sweep drive into the 1073 circuit model and report
// output peaks, symmetry, and the raw node states at the extremes.

int main()
{
    const double fs = 48000.0;

    struct Case { const char* label; float gainDb; float amp; };
    const Case cases[] =
    {
        { "cal   (0 dB, -12ish)",  0.0f, 0.25f },
        { "hot   (0 dB, -6)",      0.0f, 0.50f },
        { "hott  (0 dB, -3)",      0.0f, 0.71f },
        { "smash (0 dB, 0)",       0.0f, 1.00f },
        { "pad   (-20 dB, 1.0)", -20.0f, 1.00f },
        { "pad   (-40 dB, 1.0)", -40.0f, 1.00f },
    };

    for (const auto& c : cases)
    {
        Neve1073Circuit circ;
        circ.prepare (fs);
        circ.setGainDb (c.gainDb);

        const int n = (int) (fs * 0.25);
        double peakPos = 0.0, peakNeg = 0.0;
        double lastPos = 0.0, lastNeg = 0.0;
        int flatTopSamples = 0, flatBotSamples = 0;

        for (int i = 0; i < n; ++i)
        {
            const float x = (float) (c.amp * std::sin (2.0 * 3.14159265358979 * 440.0 * i / fs));
            const float y = circ.processSample (x);
            const double yd = y;
            if (yd > peakPos) { peakPos = yd; lastPos = x; }
            if (yd < peakNeg) { peakNeg = yd; lastNeg = x; }
            if (i > n / 4)
            {
                if (yd > 0.98 * peakPos && peakPos > 0.5) ++flatTopSamples;
                if (yd < 0.98 * peakNeg && peakNeg < -0.5) ++flatBotSamples;
            }
        }

        printf ("%-22s gain=%6.1f amp=%.2f | out pos=%8.4f neg=%9.4f | ratio pos/neg=%.3f | flatTop=%4d flatBot=%4d\n",
                c.label, c.gainDb, c.amp, peakPos, peakNeg,
                peakPos / std::max (1e-9, -peakNeg), flatTopSamples, flatBotSamples);
    }

    // Debug: expose the DC solve
    {
        Neve1073Circuit c;
        c.prepare (fs);
        const auto d = c.debugState();
        printf ("ceDc = vb=%.4f ve=%.4f vc=%.4f | ppDc = b=%.4f e=%.4f c=%.4f vo=%.4f | norm=%.6f\n",
                d.vb, d.ve, d.vc, d.pb, d.pe, d.pc, d.po, d.norm);
    }

    {
        Neve1073Circuit circ;
        circ.prepare (fs);
        circ.setGainDb (0.0f);
        const float amp = 1.0f;
        printf ("\nExtreme drive node states (output, sample):\n");
        for (int i = 0; i < (int) (fs * 0.1); ++i)
        {
            const float x = (float) (amp * std::sin (2.0 * 3.14159265358979 * 440.0 * i / fs));
            const float y = circ.processSample (x);
            if (i > (int) (fs * 0.02))
            {
                static double prev = 0.0;
                if (std::abs (y) > 1.0 && std::abs (y) < std::abs (prev)) { /* near peak */ }
                prev = y;
            }
        }
        // simple sweep of instantaneous transfer: slow ramp (quasi-DC) to see the clip shape
        Neve1073Circuit ramp;
        ramp.prepare (fs);
        ramp.setGainDb (0.0f);
        printf ("\nQuasi-static transfer (ramp input -> output):\n");
        for (int i = 0; i <= 100; ++i)
        {
            const double x = -1.0 + 2.0 * i / 100.0;
            const float y = ramp.processSample ((float) x * 0.5f);
            if (i % 5 == 0) printf ("  x=%+.3f -> y=%+.4f\n", x * 0.5, y);
        }
    }
    // Harmonic spectrum at a couple of drive points: the asymmetric
    // (bottom-heavier) clip must show even harmonics rising with drive and
    // then freezing once both rails engage.
    for (float amp : { 0.25f, 0.7f, 1.0f })
    {
        Neve1073Circuit c;
        c.prepare (fs);
        c.setGainDb (-20.0f);
        const int n = (int) fs; // 1 s
        std::vector<float> buf ((size_t) n);
        for (int i = 0; i < n; ++i)
            buf[(size_t) i] = c.processSample ((float) (amp * std::sin (2.0 * 3.14159265358979 * 440.0 * i / fs)));

        auto mag = [&] (int harm)
        {
            double re = 0.0, im = 0.0;
            const int start = n / 2; // skip transient
            for (int i = start; i < n; ++i)
            {
                const double ph = 2.0 * 3.14159265358979 * harm * 440.0 * i / fs;
                re += buf[(size_t) i] * std::cos (ph);
                im -= buf[(size_t) i] * std::sin (ph);
            }
            const double m = 2.0 * std::sqrt (re * re + im * im) / (n - start);
            return 20.0 * std::log10 (std::max (m, 1e-9));
        };

        printf ("harmonics @ amp=%.2f (gain -20): 2f=%6.1f  3f=%6.1f  4f=%6.1f  5f=%6.1f dB\n",
                amp, mag (2), mag (3), mag (4), mag (5));
    }
    return 0;
}
