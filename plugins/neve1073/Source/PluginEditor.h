#pragma once

#include <juce_gui_basics/juce_gui_basics.h>
#include "PluginProcessor.h"

//==============================================================================
/** Vintage-console look: grey brushed panel, big red gain knob with pointer
    and calibration tick marks, like the channel faceplate of a 1073. */
class Neve1073LookAndFeel : public juce::LookAndFeel_V4
{
public:
    void drawRotarySlider (juce::Graphics& g, int x, int y, int w, int h,
                           float sliderPosProportional,
                           const float rotaryStartAngle,
                           const float rotaryEndAngle,
                           juce::Slider&) override;
};

//==============================================================================
class Neve1073AudioProcessorEditor : public juce::AudioProcessorEditor
{
public:
    explicit Neve1073AudioProcessorEditor (Neve1073AudioProcessor&);
    ~Neve1073AudioProcessorEditor() override;

    void paint (juce::Graphics&) override;
    void resized() override;

private:
    Neve1073AudioProcessor& processorRef;

    Neve1073LookAndFeel lookAndFeel;

    juce::Slider gainKnob;
    juce::Label  gainLabel { {}, "GAIN" };
    juce::SliderParameterAttachment gainAttachment;

    // Engine selection (Exact / Fast) shown as a small console dropdown.
    juce::ComboBox engineBox;
    juce::Label    engineLabel { {}, "ENGINE" };
    juce::ComboBoxParameterAttachment engineAttachment;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (Neve1073AudioProcessorEditor)
};
