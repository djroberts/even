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

// Re-prepare the circuit models for the current oversampling mode: 1x runs
// them at the host rate, 2x at twice the host rate. The transfer tables are
// static and rate-independent (built once per process), so this only costs a
// handful of DC solves. Called from prepareToPlay and, when the toggle flips,
// from the top of processBlock.
void Neve1073AudioProcessor::applyOversampling (bool on)
{
    oversamplingActive = on;
    const double fs = hostSampleRate * (on ? 2.0 : 1.0);

    for (auto& c : circuit)
    {
        c.prepare (fs);   // circuits run Engine::Exact (verified table read)
        c.setGainDb (gainParam.get());
    }
    lastGain[0] = lastGain[1] = gainParam.get();
}

void Neve1073AudioProcessor::prepareToPlay (double sampleRate, int samplesPerBlock)
{
    hostSampleRate = sampleRate;

    // 2x oversampler, stereo, polyphase IIR half-band filters (linear-phase
    // enough for a colour box, lowest CPU of the JUCE options). NB: JUCE's
    // constructor takes a factor EXPONENT -- 2 ^ factor -- so 1 means 2x.
    oversampling = std::make_unique<juce::dsp::Oversampling<float>> (
        2, 1, juce::dsp::Oversampling<float>::filterHalfBandPolyphaseIIR,
        /*isMaximumQuality*/ true);
    oversampling->initProcessing ((size_t) samplesPerBlock);
    oversampling->reset();

    applyOversampling (oversampleParam.get());
}

void Neve1073AudioProcessor::releaseResources()
{
    oversampling.reset();
}

void Neve1073AudioProcessor::processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer&)
{
    const float gain = gainParam.get();
    gainDb.store (gain);

    // Oversampling toggle: re-prepare the circuits (and swap their rate) only
    // when it actually changed.
    const bool wantOs = oversampleParam.get();
    if (wantOs != oversamplingActive)
        applyOversampling (wantOs);

    // (Engine selection removed: the plugin runs the verified table read
    // unconditionally. The old Exact/Fast block was here.)

    const int numSamples = buffer.getNumSamples();

    if (oversamplingActive)
    {
        // Upsample, run the circuit model at 2x the host rate, downsample.
        auto up = oversampling->processSamplesUp (buffer);
        for (int ch = 0; ch < 2; ++ch)
        {
            if (gain != lastGain[ch])
            {
                circuit[ch].setGainDb (gain);
                lastGain[ch] = gain;
            }
            circuit[ch].process (up.getChannelPointer ((size_t) ch), numSamples * 2);
        }
        // NB: the down-stage reads the oversampler's internal (upsampled)
        // buffer -- which `up` aliases -- and writes the result into
        // outputBlock, so outputBlock must NOT alias that buffer. Pass the
        // host buffer wrapped as an AudioBlock (JUCE 9 has no AudioBuffer
        // overload; passing `up` here corrupts the up-buffer in place and
        // leaves the host buffer untouched).
        auto downBlock = juce::dsp::AudioBlock<float> { buffer };
        oversampling->processSamplesDown (downBlock);
        return;
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
        circuit[ch].process (buffer.getWritePointer (ch), numSamples);
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
    // Legacy engine slot: kept in the chunk (written as 0) so old sessions
    // (which stored gain float + engine int + os bool) still parse with the
    // oversampling flag aligned. New sessions ignore it -- the table read
    // runs regardless of what old sessions had selected.
    mos.writeInt (0);
    mos.writeBool (oversampleParam.get());
}

void Neve1073AudioProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    juce::MemoryInputStream mis (data, (size_t) sizeInBytes, false);
    // The chunk stores the plain dB value; setValueNotifyingHost expects a
    // normalised 0..1 value. (Writing the raw dB value here used to slam the
    // knob to -80 dB whenever the host restored the session.)
    gainParam.setValueNotifyingHost (gainParam.convertTo0to1 (mis.readFloat()));

    // Legacy engine index appended after the gain: skipped (not applied --
    // the verified table read runs unconditionally). Older sessions
    // (pre-dropdown) have no bytes left; nothing to skip.
    if (mis.getNumBytesRemaining() >= 4)
        mis.readInt();

    // Oversampling flag appended last (one byte). Absent in older sessions:
    // keep the default (off).
    if (mis.getNumBytesRemaining() >= 1)
    {
        const bool os = mis.readBool();
        oversampleParam.setValueNotifyingHost (oversampleParam.convertTo0to1 (os ? 1.0f : 0.0f));
        if (hostSampleRate > 0.0)
            applyOversampling (os);
    }
}

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new Neve1073AudioProcessor();
}
