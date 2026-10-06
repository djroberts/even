#pragma once

#include <juce_gui_basics/juce_gui_basics.h>
#include "PluginProcessor.h"

//==============================================================================
class Neve1073AudioProcessorEditor : public juce::AudioProcessorEditor
{
public:
    explicit Neve1073AudioProcessorEditor (Neve1073AudioProcessor&);
    ~Neve1073AudioProcessorEditor() override = default;

    void paint (juce::Graphics&) override;
    void resized() override;

private:
    Neve1073AudioProcessor& processorRef;

    juce::Slider gainKnob;
    juce::Label  gainLabel { {}, "Gain" };
    juce::SliderParameterAttachment gainAttachment;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (Neve1073AudioProcessorEditor)
};
