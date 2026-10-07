#pragma once

#include <juce_gui_basics/juce_gui_basics.h>
#include "PluginProcessor.h"
#include "even/gui/EvenLookAndFeel.h"
#include "even/gui/ConsoleFaceplate.h"

//==============================================================================
/** Vintage-console look: brushed-steel faceplate, big red gain knob with
    pointer and calibration tick marks, like the channel faceplate of a 1073. */
class Neve1073AudioProcessorEditor : public juce::AudioProcessorEditor
{
public:
    explicit Neve1073AudioProcessorEditor (Neve1073AudioProcessor&);
    ~Neve1073AudioProcessorEditor() override;

    void resized() override;

private:
    Neve1073AudioProcessor& processorRef;

    even::gui::EvenLookAndFeel lookAndFeel { even::gui::EvenLookAndFeel::KnobStyle::Red };
    even::gui::ConsoleFaceplate faceplate { "NEVE", "1073" };

    juce::Slider gainKnob;
    juce::Label  gainLabel { {}, "GAIN" };
    juce::SliderParameterAttachment gainAttachment;

    // 2x oversampling, shown as a small console toggle switch.
    juce::ToggleButton oversampleButton;
    juce::ButtonParameterAttachment oversampleAttachment;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (Neve1073AudioProcessorEditor)
};

