#include "PluginEditor.h"
#include <BuildStamp.h>

//==============================================================================
Neve1073AudioProcessorEditor::Neve1073AudioProcessorEditor (Neve1073AudioProcessor& p)
    : AudioProcessorEditor (p),
      processorRef (p),
      gainAttachment (p.gainParam, gainKnob)
{
    gainKnob.setSliderStyle (juce::Slider::RotaryHorizontalVerticalDrag);
    gainKnob.setTextBoxStyle (juce::Slider::TextBoxBelow, false, 80, 20);
    gainKnob.setTextValueSuffix (" dB");
    addAndMakeVisible (gainKnob);

    gainLabel.setJustificationType (juce::Justification::centred);
    addAndMakeVisible (gainLabel);

    setSize (240, 180);
}

void Neve1073AudioProcessorEditor::paint (juce::Graphics& g)
{
    g.fillAll (juce::Colour (0xff1a1c1e));
    g.setColour (juce::Colours::whitesmoke);
    g.setFont (16.0f);
    g.drawText ("NEVE 1073", getLocalBounds().removeFromTop (40), juce::Justification::centred);

    // Build stamp footer: proves which binary is actually loaded.
    g.setColour (juce::Colour (0xff8a8f94));
    g.setFont (juce::FontOptions (10.0f));
    g.drawText (juce::String ("build ") + EVEN_BUILD_ID + "  " + EVEN_BUILD_TIME,
                getLocalBounds().removeFromBottom (14), juce::Justification::centredRight);
}

void Neve1073AudioProcessorEditor::resized()
{
    auto area = getLocalBounds().removeFromBottom (140);
    gainKnob.setBounds (area.removeFromLeft (120).reduced (10));
    gainLabel.setBounds (gainKnob.getBounds().removeFromTop (0).expanded (0)); // label under title area
    gainLabel.setBounds (getLocalBounds().removeFromTop (60).removeFromBottom (20));
}
