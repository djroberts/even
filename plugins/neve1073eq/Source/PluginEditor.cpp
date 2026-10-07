#include "PluginEditor.h"
#include <BuildStamp.h>

//==============================================================================
Neve1073EqAudioProcessorEditor::Neve1073EqAudioProcessorEditor (Neve1073EqAudioProcessor& p)
    : AudioProcessorEditor (p)
{
    setLookAndFeel (&lookAndFeel);
    addAndMakeVisible (faceplate);

    auto& ap = p.apvts;

    auto makeKnob = [this] (juce::Slider& s, const juce::String& name)
    {
        s.setSliderStyle (juce::Slider::RotaryHorizontalVerticalDrag);
        s.setTextBoxStyle (juce::Slider::NoTextBox, false, 0, 0);
        s.setName (name);
        addAndMakeVisible (s);
    };
    auto makeLabel = [this] (juce::Label& l, const juce::String& text)
    {
        l.setText (text, juce::dontSendNotification);
        l.setJustificationType (juce::Justification::centred);
        l.setColour (juce::Label::textColourId, juce::Colour (0xffd7dadd));
        l.setFont (juce::FontOptions (12.0f, juce::Font::bold));
        addAndMakeVisible (l);
    };
    auto makeCombo = [this] (juce::ComboBox& c)
    {
        // Colours come from the shared console LookAndFeel.
        addAndMakeVisible (c);
    };

    // Frequency switches (identical 6 positions on both mid bands).
    const juce::StringArray midChoices { "360", "700", "1.6k", "3.2k", "4.8k", "7.2k" };
    mid1FreqBox.addItemList (midChoices, 1);
    mid2FreqBox.addItemList (midChoices, 1);
    lfFreqBox.addItemList  ({ "35 Hz", "60 Hz", "110 Hz", "220 Hz" }, 1);
    hpfFreqBox.addItemList ({ "50 Hz", "80 Hz", "160 Hz", "300 Hz" }, 1);
    makeCombo (mid1FreqBox); makeCombo (mid2FreqBox);
    makeCombo (lfFreqBox);   makeCombo (hpfFreqBox);

    makeLabel (mid1FreqLabel, "FREQ");
    makeLabel (mid2FreqLabel, "FREQ");
    makeLabel (lfFreqLabel,   "TURNOVER");

    makeLabel (hfLabel,   "HF\n+/-16 dB");
    makeLabel (mid1Label, "MID 1\n+/-18 dB");
    makeLabel (mid2Label, "MID 2\n+/-18 dB");
    makeLabel (lfLabel,   "LF\n+/-16 dB");

    makeKnob (hfGainKnob,   "hfGain");
    makeKnob (mid1GainKnob, "mid1Gain");
    makeKnob (mid2GainKnob, "mid2Gain");
    makeKnob (lfGainKnob,   "lfGain");
    makeKnob (outTrimKnob,  "outTrim");

    hpfOnButton.setButtonText ("HPF");
    eqInButton.setButtonText ("EQ IN");
    addAndMakeVisible (hpfOnButton);
    addAndMakeVisible (eqInButton);
    makeLabel (outTrimLabel, "TRIM");

    sliderAttachments.push_back (std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (ap, "hfGain",   hfGainKnob));
    sliderAttachments.push_back (std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (ap, "mid1Gain", mid1GainKnob));
    sliderAttachments.push_back (std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (ap, "mid2Gain", mid2GainKnob));
    sliderAttachments.push_back (std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (ap, "lfGain",   lfGainKnob));
    sliderAttachments.push_back (std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (ap, "outTrim",  outTrimKnob));
    comboAttachments.push_back (std::make_unique<juce::AudioProcessorValueTreeState::ComboBoxAttachment> (ap, "mid1Freq", mid1FreqBox));
    comboAttachments.push_back (std::make_unique<juce::AudioProcessorValueTreeState::ComboBoxAttachment> (ap, "mid2Freq", mid2FreqBox));
    comboAttachments.push_back (std::make_unique<juce::AudioProcessorValueTreeState::ComboBoxAttachment> (ap, "lfFreq",   lfFreqBox));
    comboAttachments.push_back (std::make_unique<juce::AudioProcessorValueTreeState::ComboBoxAttachment> (ap, "hpfFreq",  hpfFreqBox));
    buttonAttachments.push_back (std::make_unique<juce::AudioProcessorValueTreeState::ButtonAttachment> (ap, "hpfOn", hpfOnButton));
    buttonAttachments.push_back (std::make_unique<juce::AudioProcessorValueTreeState::ButtonAttachment> (ap, "eqIn",  eqInButton));

    setSize (640, 330);
}

Neve1073EqAudioProcessorEditor::~Neve1073EqAudioProcessorEditor()
{
    setLookAndFeel (nullptr);
}

void Neve1073EqAudioProcessorEditor::resized()
{
    faceplate.setBounds (getLocalBounds());

    auto body = getLocalBounds().removeFromTop (getHeight() - 14); // build stamp clear
    body.removeFromTop (34);                                       // badge

    // Four band columns: HF | MID 1 | MID 2 | LF.
    auto bands = body.removeFromTop (200).reduced (16, 6);
    const int colW = bands.getWidth() / 4;

    auto layoutColumn = [&] (juce::ComboBox* freqBox, juce::Label* freqLabel,
                             juce::Slider& knob, juce::Label& bandLabel)
    {
        auto colBounds = bands.removeFromLeft (colW).reduced (4, 0);
        bandLabel.setBounds (colBounds.removeFromTop (28));
        if (freqBox != nullptr)
        {
            freqBox->setBounds (colBounds.removeFromTop (22).reduced (6, 0));
            freqLabel->setBounds (colBounds.removeFromTop (14).reduced (6, 0));
        }
        knob.setBounds (colBounds.reduced (10, 4));
    };

    layoutColumn (nullptr,       nullptr,        hfGainKnob,   hfLabel);
    layoutColumn (&mid1FreqBox,  &mid1FreqLabel, mid1GainKnob, mid1Label);
    layoutColumn (&mid2FreqBox,  &mid2FreqLabel, mid2GainKnob, mid2Label);
    layoutColumn (&lfFreqBox,    &lfFreqLabel,   lfGainKnob,   lfLabel);

    // Bottom strip: HPF (toggle + freq switch) | EQ IN | TRIM.
    auto strip = body.reduced (40, 6);
    auto hpfArea = strip.removeFromLeft (150);
    hpfOnButton.setBounds (hpfArea.removeFromLeft (70).removeFromTop (22));
    hpfFreqBox.setBounds  (hpfArea.removeFromLeft (86).removeFromTop (22));
    eqInButton.setBounds (strip.removeFromLeft (90).removeFromTop (22));
    outTrimKnob.setBounds (strip.removeFromRight (90).removeFromTop (72));
    outTrimLabel.setBounds (outTrimKnob.getBounds().removeFromBottom (18));
}
