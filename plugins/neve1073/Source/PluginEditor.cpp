#include "PluginEditor.h"
#include <BuildStamp.h>

namespace
{
    // Dial calibration: the dB positions printed on the faceplate. The knob
    // is inverted: fully counterclockwise = +10 dB, fully clockwise = -80 dB,
    // so the tick for a given dB is mirrored across the sweep.
    constexpr int tickLabels[] = { 10, 0, -20, -40, -60, -80 };

    // 270 degrees of travel.
    constexpr float startAngle = juce::MathConstants<float>::pi * 1.25f;
    constexpr float endAngle   = juce::MathConstants<float>::pi * 2.75f;
}

//==============================================================================
Neve1073AudioProcessorEditor::Neve1073AudioProcessorEditor (Neve1073AudioProcessor& p)
    : AudioProcessorEditor (p),
      processorRef (p),
      gainAttachment (p.gainParam, gainKnob),
      oversampleAttachment (p.oversampleParam, oversampleButton)
{
    setLookAndFeel (&lookAndFeel);
    addAndMakeVisible (faceplate);

    // Faceplate calibration ticks, mirrored across the inverted sweep.
    juce::StringArray tickTexts;
    std::vector<float> tickProps;
    for (auto db : tickLabels)
    {
        tickProps.push_back (juce::jlimit (0.0f, 1.0f, ((float) db + 80.0f) / 90.0f));
        tickTexts.add (db > 0 ? "+10" : juce::String (db));
    }
    lookAndFeel.setTickMarks (tickTexts, tickProps);

    // Click + drag UP sweeps the knob clockwise towards -80 dB, matching the
    // inverted faceplate: +10 dB fully counterclockwise, -80 dB fully clockwise.
    gainKnob.setSliderStyle (juce::Slider::RotaryVerticalDrag);
    gainKnob.setRotaryParameters (startAngle, endAngle, true);

    // Invert the knob's normalised mapping (NOT the parameter, which stays
    // plain linear -80..+10): normalised 0 = +10 dB (CCW), 1 = -80 dB (CW).
    // The slider's value is still the true dB, so the attachment, host
    // automation and state are unaffected; dragging up now sweeps the knob
    // clockwise towards -80 dB, matching the inverted faceplate.
    auto to01   = [] (double, double, double db) { return (10.0 - db) / 90.0; };
    auto from01 = [] (double, double, double pos)
    {
        return juce::jlimit (-80.0, 10.0, 10.0 - 90.0 * pos);
    };
    auto snap = [] (double, double, double v)
    {
        return juce::jlimit (-80.0, 10.0, std::round (v * 100.0) / 100.0);
    };
    gainKnob.setNormalisableRange ({ -80.0, 10.0, std::move (from01),
                                     std::move (to01), std::move (snap) });
    gainKnob.setValue (processorRef.gainParam.get(), juce::dontSendNotification);
    gainKnob.setTextBoxStyle (juce::Slider::TextBoxBelow, false, 100, 20);
    gainKnob.setTextValueSuffix (" dB");
    gainKnob.setColour (juce::Slider::textBoxTextColourId, juce::Colour (0xffd7dadd));
    gainKnob.setColour (juce::Slider::textBoxOutlineColourId, juce::Colours::transparentBlack);
    addAndMakeVisible (gainKnob);

    gainLabel.setJustificationType (juce::Justification::centred);
    gainLabel.setColour (juce::Label::textColourId, juce::Colour (0xffd7dadd));
    gainLabel.setFont (juce::FontOptions (13.0f, juce::Font::bold));
    addAndMakeVisible (gainLabel);

    // Oversampling toggle, styled by the shared console LookAndFeel.
    oversampleButton.setButtonText ("2x OS");
    oversampleButton.setTooltip ("Runs the circuit model at twice the sample rate "
                                 "(less aliasing, higher CPU).");
    addAndMakeVisible (oversampleButton);

    setSize (280, 258);
}

Neve1073AudioProcessorEditor::~Neve1073AudioProcessorEditor()
{
    setLookAndFeel (nullptr);
}

void Neve1073AudioProcessorEditor::resized()
{
    faceplate.setBounds (getLocalBounds());

    auto body = getLocalBounds().removeFromTop (getHeight() - 14); // keep build stamp clear
    body.removeFromTop (34);                                       // badge

    gainKnob.setBounds (body.removeFromTop (body.getHeight() - 22 - 30).reduced (26));
    gainLabel.setBounds (body.removeFromTop (22));

    // Control strip along the bottom of the faceplate: oversampling toggle,
    // centred now that the engine dropdown is gone.
    auto strip = body.removeFromTop (30).reduced (30, 5);
    oversampleButton.setBounds (strip.removeFromLeft (64).removeFromTop (20));
    oversampleButton.setBounds (oversampleButton.getBounds()
                                    .translated ((strip.getWidth() - 64) / 2, 0));
}
