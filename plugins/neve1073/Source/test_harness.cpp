#include <cstdio>
#include <cmath>
#include <algorithm>
#include <chrono>
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

    //==========================================================================
    // Transformer-model validation.
    //
    // 1. Small-signal frequency response: the linear part of the transformer
    //    models must show the intended LF corners (~12 Hz input, ~8 Hz output)
    //    and gentle HF loss, measured relative to a 1 kHz reference.
    {
        printf ("\nfrequency response (small signal, 0 dB gain, relative to 1 kHz):\n");
        const double freqs[] = { 20.0, 40.0, 100.0, 1000.0, 10000.0, 20000.0 };
        std::vector<double> mags;
        double ref = 0.0;
        for (double f : freqs)
        {
            Neve1073Circuit c;
            c.prepare (fs);
            c.setGainDb (0.0f);
            const int n = (int) fs;
            std::vector<float> buf ((size_t) n);
            for (int i = 0; i < n; ++i)
                buf[(size_t) i] = c.processSample ((float) (0.01 * std::sin (2.0 * 3.14159265358979 * f * i / fs)));
            double re = 0.0, im = 0.0;
            const int start = n / 2;
            for (int i = start; i < n; ++i)
            {
                const double ph = 2.0 * 3.14159265358979 * f * i / fs;
                re += buf[(size_t) i] * std::cos (ph);
                im -= buf[(size_t) i] * std::sin (ph);
            }
            const double m = 2.0 * std::sqrt (re * re + im * im) / (n - start);
            mags.push_back (m);
            if (f == 1000.0) ref = m;
        }
        for (size_t k = 0; k < mags.size(); ++k)
            printf ("  %8.1f Hz: %7.3f dB\n", freqs[k],
                    20.0 * std::log10 (std::max (mags[k] / ref, 1e-12)));
    }

    // 2. Core saturation vs frequency: at matched drive the low frequencies
    //    must distort MORE (flux ~ V/f): 2nd/3rd harmonic at 40 Hz should sit
    //    well above the 440 Hz numbers from the sweep above.
    {
        Neve1073Circuit c;
        c.prepare (fs);
        c.setGainDb (0.0f);
        const double f = 40.0, amp = 0.5;
        const int n = (int) fs;
        std::vector<float> buf ((size_t) n);
        for (int i = 0; i < n; ++i)
            buf[(size_t) i] = c.processSample ((float) (amp * std::sin (2.0 * 3.14159265358979 * f * i / fs)));
        auto mag = [&] (int harm)
        {
            double re = 0.0, im = 0.0;
            const int start = n / 2;
            for (int i = start; i < n; ++i)
            {
                const double ph = 2.0 * 3.14159265358979 * harm * f * i / fs;
                re += buf[(size_t) i] * std::cos (ph);
                im -= buf[(size_t) i] * std::sin (ph);
            }
            const double m = 2.0 * std::sqrt (re * re + im * im) / (n - start);
            return 20.0 * std::log10 (std::max (m, 1e-9));
        };
        printf ("\n40 Hz core-saturation check (amp=%.2f, gain 0): 2f=%6.1f  3f=%6.1f dB (expect above the 440 Hz values)\n",
                amp, mag (2), mag (3));
    }

    // 3. DC / drift guard: after a loud burst, 2 s of silence must decay the
    //    transformer states (flux through the winding resistance, cap through
    //    the source impedance) back to digital silence. The first ~0.5 s is
    //    the legitimate relaxation transient; after that nothing may remain.
    {
        Neve1073Circuit c;
        c.prepare (fs);
        c.setGainDb (0.0f);
        for (int i = 0; i < (int) (fs * 0.1); ++i)
            c.processSample (i % 2 == 0 ? 1.0f : -1.0f);
        double worst = 0.0;
        for (int i = 0; i < (int) (fs * 2.0); ++i)
        {
            const double y = std::abs ((double) c.processSample (0.0f));
            if (i > (int) (fs * 0.5))
                worst = std::max (worst, y);
        }
        printf ("\nsilence-decay check: worst residual after 0.5 s = %.3e (expect < 1e-5)\n", worst);
    }

    // 4. Engine equivalence: Fast must track Live through the transformers to
    //    better than the float32 LSB at typical levels.
    {
        Neve1073Circuit live, fast;
        live.prepare (fs); fast.prepare (fs);
        live.setEngine (Neve1073Circuit::Engine::Live);
        fast.setEngine (Neve1073Circuit::Engine::Fast);
        double worst = 0.0;
        for (int i = 0; i < (int) fs; ++i)
        {
            const float x = (float) (0.8 * std::sin (2.0 * 3.14159265358979 * 997.0 * i / fs));
            const double a = live.processSample (x);
            const double b = fast.processSample (x);
            worst = std::max (worst, std::abs (a - b));
        }
        printf ("engine check: worst |Fast - Live| = %.3e (expect < 6e-8)\n", worst);
    }

    // 5. CPU benchmark: 10 s of audio through the Fast engine, single
    //    channel, reported as multiple-of-realtime.
    {
        Neve1073Circuit c;
        c.prepare (fs);
        c.setGainDb (0.0f);
        c.setEngine (Neve1073Circuit::Engine::Fast);
        const int n = (int) (fs * 10.0);
        std::vector<float> buf ((size_t) n, 0.0f);
        for (int i = 0; i < n; ++i)
            buf[(size_t) i] = (float) (0.5 * std::sin (2.0 * 3.14159265358979 * 440.0 * i / fs));
        const auto t0 = std::chrono::steady_clock::now();
        c.process (buf.data(), n);
        const double secs = std::chrono::duration<double> (std::chrono::steady_clock::now() - t0).count();
        printf ("cpu benchmark (Fast, 1 ch): %.2f s for 10 s audio -> %.0fx realtime\n", secs, 10.0 / secs);
    }
    return 0;
}
