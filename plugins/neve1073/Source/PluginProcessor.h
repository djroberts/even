#pragma once

#include <juce_audio_processors/juce_audio_processors.h>
#include <juce_dsp/juce_dsp.h>
#include "Neve1073Circuit.h"

//==============================================================================
class Neve1073AudioProcessor : public juce::AudioProcessor
{
public:
    Neve1073AudioProcessor();
    ~Neve1073AudioProcessor() override = default;

    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override;
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

    // Gain: maps to the 1073 sensitivity/attenuator network (amount of drive
    // into the BA283 saturation stage). Range -80..+10 as on the hardware.
    // Plain linear mapping, 0.01 dB resolution (2 decimals). Fully
    // counterclockwise = +10 dB, fully clockwise = -80 dB (knob is inverted
    // on the faceplate; the underlying value mapping is unchanged).
    juce::AudioParameterFloat gainParam {
        "gain", "Gain",
        juce::NormalisableRange<float> { -80.0f, 10.0f, 0.01f },
        0.0f
    };

    // Engine selection (dropdown). "Exact" = precomputed curve seeds the
    // Newton solves, which polish to full solver tolerance (quality-neutral,
    // bit-transparent at float32). "Fast" = pure curve lookup, no Newton in
    // the audio path (validated below the float32 LSB; lowest CPU).
    juce::AudioParameterChoice qualityParam {
        "quality", "Engine", juce::StringArray { "Exact", "Fast" }, 0
    };

    // 2x oversampling toggle. Runs the circuit model at twice the host rate
    // (polyphase IIR half-band up/downsampling) so the saturation stages see
    // less aliasing. Toggling re-prepares the circuits at the new rate; the
    // transfer tables are rate-independent and shared, so this is cheap.
    juce::AudioParameterBool oversampleParam {
        "oversample", "2x Oversample", false
    };

    void applyOversampling (bool on);

private:
    std::atomic<float> gainDb { 0.0f };
    std::atomic<int>   engineMode { 0 }; // 0 = Exact, 1 = Fast
    Neve1073Circuit circuit[2];
    float lastGain[2] = { -999.0f, -999.0f }; // last value handed to setGainDb

    double hostSampleRate = 0.0;
    bool   oversamplingActive = false;   // circuits currently run at 2x rate
    std::unique_ptr<juce::dsp::Oversampling<float>> oversampling;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (Neve1073AudioProcessor)
};
