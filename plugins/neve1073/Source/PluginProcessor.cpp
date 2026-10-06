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
    lastGain[0] = lastGain[1] = gainParam.get();
}

void Neve1073AudioProcessor::releaseResources() {}

void Neve1073AudioProcessor::processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer&)
{
    const float gain = gainParam.get();
    gainDb.store (gain);

    // Engine selection, cached so the circuit is only poked when it changed.
    const int engine = (int) qualityParam.getIndex();
    if (engine != engineMode.exchange (engine))
    {
        const auto e = engine == 1 ? Neve1073Circuit::Engine::Fast
                                   : Neve1073Circuit::Engine::Exact;
        for (auto& c : circuit) c.setEngine (e);
    }

    for (int ch = 0; ch < buffer.getNumChannels(); ++ch)
    {
        if (ch >= 2) break;
        // Only recompute the drive scaling when the knob actually moved.
        if (gain != lastGain[ch])
        {
            circuit[ch].setGainDb (gain);
            lastGain[ch] = gain;
        }
        circuit[ch].process (buffer.getWritePointer (ch), buffer.getNumSamples());
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
    mos.writeInt (qualityParam.getIndex());
}

void Neve1073AudioProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    juce::MemoryInputStream mis (data, (size_t) sizeInBytes, false);
    // The chunk stores the plain dB value; setValueNotifyingHost expects a
    // normalised 0..1 value. (Writing the raw dB value here used to slam the
    // knob to -80 dB whenever the host restored the session.)
    gainParam.setValueNotifyingHost (gainParam.convertTo0to1 (mis.readFloat()));

    // Engine index appended after the gain. Older sessions (pre-dropdown)
    // have no bytes left; they keep the Exact default.
    if (mis.getNumBytesRemaining() >= 4)
    {
        const int idx = juce::jlimit (0, qualityParam.choices.size() - 1, mis.readInt());
        engineMode.store (idx);
        qualityParam.setValueNotifyingHost (qualityParam.convertTo0to1 ((float) idx));
    }
}

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new Neve1073AudioProcessor();
}
