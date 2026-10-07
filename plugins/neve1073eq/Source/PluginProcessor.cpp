#include "PluginProcessor.h"
#include "PluginEditor.h"

//==============================================================================
Neve1073EqAudioProcessor::Neve1073EqAudioProcessor()
    : AudioProcessor (BusesProperties()
                          .withInput  ("Input",  juce::AudioChannelSet::stereo(), true)
                          .withOutput ("Output", juce::AudioChannelSet::stereo(), true)),
      apvts (*this, nullptr, "PARAMS", createLayout())
{
    eqInParam    = apvts.getRawParameterValue ("eqIn");
    hpfOnParam   = apvts.getRawParameterValue ("hpfOn");
    hpfFreqParam = apvts.getRawParameterValue ("hpfFreq");
    lfFreqParam  = apvts.getRawParameterValue ("lfFreq");
    mid1FreqParam = apvts.getRawParameterValue ("mid1Freq");
    mid2FreqParam = apvts.getRawParameterValue ("mid2Freq");
    lfGainParam  = apvts.getRawParameterValue ("lfGain");
    mid1GainParam = apvts.getRawParameterValue ("mid1Gain");
    mid2GainParam = apvts.getRawParameterValue ("mid2Gain");
    hfGainParam  = apvts.getRawParameterValue ("hfGain");
    outTrimParam = apvts.getRawParameterValue ("outTrim");
}

juce::AudioProcessorValueTreeState::ParameterLayout
Neve1073EqAudioProcessor::createLayout()
{
    using P = juce::AudioParameterChoice;
    using F = juce::AudioParameterFloat;
    using B = juce::AudioParameterBool;

    juce::AudioProcessorValueTreeState::ParameterLayout layout;

    layout.add (std::make_unique<B> ("eqIn", "EQ In", true));

    // Fixed hardware switch positions (per the AMS Neve 1073 spec sheet).
    // "1073-plus": TWO mid bands, both with the identical 6-position switch.
    layout.add (std::make_unique<B> ("hpfOn", "HPF On", false));
    layout.add (std::make_unique<P> ("hpfFreq", "HPF Freq",
        juce::StringArray { "50 Hz", "80 Hz", "160 Hz", "300 Hz" }, 0));
    layout.add (std::make_unique<P> ("lfFreq", "LF Freq",
        juce::StringArray { "35 Hz", "60 Hz", "110 Hz", "220 Hz" }, 1));
    layout.add (std::make_unique<P> ("mid1Freq", "Mid 1 Freq",
        juce::StringArray { "360", "700", "1.6k", "3.2k", "4.8k", "7.2k" }, 1));
    layout.add (std::make_unique<P> ("mid2Freq", "Mid 2 Freq",
        juce::StringArray { "360", "700", "1.6k", "3.2k", "4.8k", "7.2k" }, 3));

    // Boost/cut pots: shelves +/-16 dB, mids +/-18 dB (0.5 dB steps).
    const juce::NormalisableRange<float> shelfRange { -16.0f, 16.0f, 0.5f };
    const juce::NormalisableRange<float> midRange   { -18.0f, 18.0f, 0.5f };
    layout.add (std::make_unique<F> ("lfGain",   "LF Gain",   shelfRange, 0.0f));
    layout.add (std::make_unique<F> ("mid1Gain", "Mid 1 Gain", midRange, 0.0f));
    layout.add (std::make_unique<F> ("mid2Gain", "Mid 2 Gain", midRange, 0.0f));
    layout.add (std::make_unique<F> ("hfGain",   "HF Gain",   shelfRange, 0.0f));

    layout.add (std::make_unique<F> ("outTrim", "Output Trim",
        juce::NormalisableRange<float> { -12.0f, 12.0f, 0.1f }, 0.0f));

    return layout;
}

bool Neve1073EqAudioProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
    return layouts.getMainOutputChannelSet() == layouts.getMainInputChannelSet()
        && layouts.getMainOutputChannelSet() == juce::AudioChannelSet::stereo();
}

void Neve1073EqAudioProcessor::prepareToPlay (double sampleRate, int /*samplesPerBlock*/)
{
    for (int ch = 0; ch < 2; ++ch)
    {
        circuit[ch].prepare (sampleRate);
        circuit[ch].reset();
        smoothed[ch] = {};  // flat gains, hardware default switch positions
    }
}

void Neve1073EqAudioProcessor::processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer&)
{
    // ---- Switch positions (stepped hardware controls).
    static constexpr double hpfHz[]  = { 50.0, 80.0, 160.0, 300.0 };
    static constexpr double lfHz[]   = { 35.0, 60.0, 110.0, 220.0 };
    static constexpr double midHz[]  = { 360.0, 700.0, 1600.0, 3200.0, 4800.0, 7200.0 };
    const bool   eqIn = eqInParam->load() > 0.5f;
    const double hpfF = hpfOnParam->load() > 0.5f
                      ? hpfHz[(int) juce::jlimit (0, 3, (int) hpfFreqParam->load())] : 0.0;
    const double lfF  = lfHz [(int) juce::jlimit (0, 3, (int) lfFreqParam->load())];
    const double m1F  = midHz[(int) juce::jlimit (0, 5, (int) mid1FreqParam->load())];
    const double m2F  = midHz[(int) juce::jlimit (0, 5, (int) mid2FreqParam->load())];
    const float  trim = juce::Decibels::decibelsToGain (outTrimParam->load());

    const int numSamples = buffer.getNumSamples();

    for (int ch = 0; ch < juce::jmin (2, buffer.getNumChannels()); ++ch)
    {
        float* x = buffer.getWritePointer (ch);

        if (! eqIn)
        {
            // EQ out: hard-wired bypass (the original EQ button lifts the
            // network entirely). Only the output trim still applies.
            if (trim != 1.0f)
                for (int i = 0; i < numSamples; ++i)
                    x[i] *= trim;
            continue;
        }

        // Advance the block-rate smoothing toward the pot positions and
        // retune the passive network.
        smoothed[ch].update (lfGainParam->load(), mid1GainParam->load(),
                             mid2GainParam->load(), hfGainParam->load(),
                             hpfF, lfF, m1F, m2F);
        circuit[ch].setSettings (smoothed[ch].current);
        circuit[ch].process (x, numSamples);

        if (trim != 1.0f)
            for (int i = 0; i < numSamples; ++i)
                x[i] *= trim;
    }
}

juce::AudioProcessorEditor* Neve1073EqAudioProcessor::createEditor()
{
    return new Neve1073EqAudioProcessorEditor (*this);
}

void Neve1073EqAudioProcessor::getStateInformation (juce::MemoryBlock& destData)
{
    if (auto xml = apvts.copyState().createXml())
        copyXmlToBinary (*xml, destData);
}

void Neve1073EqAudioProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    if (auto xml = getXmlFromBinary (data, sizeInBytes);
        xml != nullptr && xml->hasTagName (apvts.state.getType()))
        apvts.replaceState (juce::ValueTree::fromXml (*xml));
}

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new Neve1073EqAudioProcessor();
}
