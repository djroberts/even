#pragma once

#include <juce_audio_processors/juce_audio_processors.h>
#include <cmath>
#include <algorithm>
#include "Neve1073EqCircuit.h"

//==============================================================================
// Neve 1073 EQ -- standalone EQ section plugin (B205 + B211 + BA283 makeup
// stage). Parameters follow the original stepped/continuous control layout;
// state is an AudioProcessorValueTreeState (XML), unlike the preamp plugin's
// hand-rolled chunk, because this plugin has many more parameters.
class Neve1073EqAudioProcessor : public juce::AudioProcessor
{
public:
    Neve1073EqAudioProcessor();
    ~Neve1073EqAudioProcessor() override = default;

    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override {}
    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;
    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override { return true; }
    const juce::String getName() const override { return JucePlugin_Name; }
    bool acceptsMidi() const override { return false; }
    bool producesMidi() const override { return false; }
    bool isMidiEffect() const override { return false; }
    double getTailLengthSeconds() const override { return 0.0; }
    int getNumPrograms() override { return 1; }
    int getCurrentProgram() override { return 0; }
    void setCurrentProgram (int) override {}
    const juce::String getProgramName (int) override { return {}; }
    void changeProgramName (int, const juce::String&) override {}
    void getStateInformation (juce::MemoryBlock& destData) override;
    void setStateInformation (const void* data, int sizeInBytes) override;

    juce::AudioProcessorValueTreeState apvts;

private:
    static juce::AudioProcessorValueTreeState::ParameterLayout createLayout();

    // Per-channel smoothed settings (block-rate one-pole smoothing; the
    // 0.5 dB parameter step + ~1 ms/block time constant keeps pot moves
    // zipper-free; switch changes retune the passive network at block rate).
    struct SmoothedSettings
    {
        Neve1073EqCircuit::BandSettings current;
        double lfT = 0.0, m1T = 0.0, m2T = 0.0, hfT = 0.0; // targets, dB

        void update (double lfDb, double m1Db, double m2Db, double hfDb,
                     double hpfF, double lfF, double m1F, double m2F)
        {
            lfT = lfDb; m1T = m1Db; m2T = m2Db; hfT = hfDb;
            constexpr double smooth = 0.15; // per block (~1 ms @ 128/48k)

            auto move = [] (double cur, double target) { return cur + smooth * (target - cur); };
            current.hpfF = hpfF; current.lfF = lfF;
            current.mid1F = m1F; current.mid2F = m2F;

            current.lfGain   = dbToLin (move (dbFromLin (current.lfGain),   lfT));
            current.mid1Gain = dbToLin (move (dbFromLin (current.mid1Gain), m1T));
            current.mid2Gain = dbToLin (move (dbFromLin (current.mid2Gain), m2T));
            current.hfGain   = dbToLin (move (dbFromLin (current.hfGain),   hfT));
        }

        static double dbToLin (double db)  { return std::pow (10.0, db / 20.0); }
        static double dbFromLin (double g) { return 20.0 * std::log10 (std::max (g, 1.0e-6)); }
    };

    Neve1073EqCircuit circuit[2];
    SmoothedSettings smoothed[2];

    std::atomic<float>* eqInParam    = nullptr;
    std::atomic<float>* hpfOnParam   = nullptr;
    std::atomic<float>* hpfFreqParam = nullptr;
    std::atomic<float>* lfFreqParam  = nullptr;
    std::atomic<float>* mid1FreqParam = nullptr;
    std::atomic<float>* mid2FreqParam = nullptr;
    std::atomic<float>* lfGainParam  = nullptr;
    std::atomic<float>* mid1GainParam = nullptr;
    std::atomic<float>* mid2GainParam = nullptr;
    std::atomic<float>* hfGainParam  = nullptr;
    std::atomic<float>* outTrimParam = nullptr;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (Neve1073EqAudioProcessor)
};
