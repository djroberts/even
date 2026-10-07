#pragma once

#include <juce_gui_basics/juce_gui_basics.h>
#include "PluginProcessor.h"

//==============================================================================
/** Vintage-console look matching the neve1073 preamp plugin: grey brushed
    panel, dark rotary knobs with ivory pointers, hardware freq switches. */
class Neve1073EqLookAndFeel : public juce::LookAndFeel_V4
{
public:
    void drawRotarySlider (juce::Graphics& g, int x, int y, int w, int h,
                           float sliderPosProportional,
                           const float rotaryStartAngle,
                           const float rotaryEndAngle,
                           juce::Slider&) override;
};

//==============================================================================
class Neve1073EqAudioProcessorEditor : public juce::AudioProcessorEditor
{
public:
    explicit Neve1073EqAudioProcessorEditor (Neve1073EqAudioProcessor&);
    ~Neve1073EqAudioProcessorEditor() override;

    void paint (juce::Graphics&) override;
    void resized() override;

private:
    Neve1073EqLookAndFeel lookAndFeel;

    // Faceplate: HF | MID 1 | MID 2 | LF columns; each mid has its own
    // 6-position frequency switch + boost/cut pot.
    juce::ComboBox mid1FreqBox, mid2FreqBox, lfFreqBox, hpfFreqBox;
    juce::Label    mid1FreqLabel, mid2FreqLabel, lfFreqLabel;
    juce::Slider   mid1GainKnob, mid2GainKnob, lfGainKnob, hfGainKnob;
    juce::Label    mid1Label, mid2Label, lfLabel, hfLabel;

    juce::ToggleButton hpfOnButton, eqInButton;
    juce::Slider       outTrimKnob;
    juce::Label        outTrimLabel;

    std::vector<std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment>>   sliderAttachments;
    std::vector<std::unique_ptr<juce::AudioProcessorValueTreeState::ComboBoxAttachment>> comboAttachments;
    std::vector<std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment>>   buttonAttachments;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (Neve1073EqAudioProcessorEditor)
};
