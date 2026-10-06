#include "PluginProcessor.h"
#include "PluginEditor.h"

//==============================================================================
Neve1073AudioProcessor::Neve1073AudioProcessor()
    : AudioProcessor (BusesProperties()
                          .withInput  ("Input",  juce::AudioChannelSet::stereo(), true)
                          .withOutput ("Output", juce::AudioChannelSet::stereo(), true))
{
    gainDb.store (gainParam.get());
}

bool Neve1073AudioProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
    return layouts.getMainOutputChannelSet() == layouts.getMainInputChannelSet()
        && layouts.getMainOutputChannelSet() == juce::AudioChannelSet::stereo();
}

void Neve1073AudioProcessor::prepareToPlay (double sampleRate, int)
{
    for (auto& c : circuit)
    {
        c.prepare (sampleRate);
        c.setGainDb (gainParam.get());
    }
}

void Neve1073AudioProcessor::releaseResources() {}

void Neve1073AudioProcessor::processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer&)
{
    gainDb.store (gainParam.get());
    const float gain = gainDb.load();

    for (int ch = 0; ch < buffer.getNumChannels(); ++ch)
    {
        if (ch >= 2) break;
        circuit[ch].setGainDb (gain);
        auto* data = buffer.getWritePointer (ch);
        for (int i = 0; i < buffer.getNumSamples(); ++i)
            data[i] = circuit[ch].processSample (data[i]);
    }
}

juce::AudioProcessorEditor* Neve1073AudioProcessor::createEditor()
{
    return new Neve1073AudioProcessorEditor (*this);
}

void Neve1073AudioProcessor::getStateInformation (juce::MemoryBlock& destData)
{
    juce::MemoryOutputStream mos (destData, false);
    mos.writeFloat (gainParam.get());
}

void Neve1073AudioProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    juce::MemoryInputStream mis (data, (size_t) sizeInBytes, false);
    // The chunk stores the plain dB value; setValueNotifyingHost expects a
    // normalised 0..1 value. (Writing the raw dB value here used to slam the
    // knob to -80 dB whenever the host restored the session.)
    gainParam.setValueNotifyingHost (gainParam.convertTo0to1 (mis.readFloat()));
}

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new Neve1073AudioProcessor();
}
