#include "PluginEditor.h"
#include <BuildStamp.h>

//==============================================================================
void Neve1073EqLookAndFeel::drawRotarySlider (juce::Graphics& g, int x, int y, int w, int h,
                                              float sliderPosProportional,
                                              const float rotaryStartAngle,
                                              const float rotaryEndAngle,
                                              juce::Slider&)
{
    const auto centre = juce::Rectangle<float> ((float) x, (float) y, (float) w, (float) h).getCentre();
    const float radius = juce::jmin (w, h) * 0.5f;
    const float angle  = rotaryStartAngle
                       + sliderPosProportional * (rotaryEndAngle - rotaryStartAngle);

    // ---- knob cap: dark grey with a subtle radial sheen (like the EQ pots
    //      on the 1073 faceplate; the preamp's big red knob is the gain) ----
    const float capR = radius * 0.86f;
    juce::ColourGradient capGrad (juce::Colour (0xff8b8f93), centre.x - capR * 0.4f,
                                  centre.y - capR * 0.6f,
                                  juce::Colour (0xff3a3d40), centre.x + capR * 0.3f,
                                  centre.y + capR * 0.5f, true);
    g.setGradientFill (capGrad);
    g.fillEllipse (centre.x - capR, centre.y - capR, capR * 2.0f, capR * 2.0f);

    g.setColour (juce::Colour (0xff222426));
    g.drawEllipse (centre.x - capR, centre.y - capR, capR * 2.0f, capR * 2.0f, 1.6f);

    g.setColour (juce::Colour (0x30ffffff));
    g.drawEllipse (centre.x - capR * 0.7f, centre.y - capR * 0.7f,
                   capR * 1.4f, capR * 1.4f, 1.0f);

    // ---- ivory pointer -----------------------------------------------------
    const float pr = capR * 0.82f;
    const float ca = std::cos (angle), sa = std::sin (angle);
    g.setColour (juce::Colour (0xfff0e6d2));
    g.drawLine ({ centre.x, centre.y,
                  centre.x + pr * ca, centre.y + pr * sa }, 2.2f);
}

//==============================================================================
Neve1073EqAudioProcessorEditor::Neve1073EqAudioProcessorEditor (Neve1073EqAudioProcessor& p)
    : AudioProcessorEditor (p)
{
    setLookAndFeel (&lookAndFeel);
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
        c.setColour (juce::ComboBox::textColourId, juce::Colour (0xffd7dadd));
        c.setColour (juce::ComboBox::backgroundColourId, juce::Colour (0xff2e3134));
        c.setColour (juce::ComboBox::arrowColourId, juce::Colour (0xffd7dadd));
        c.setColour (juce::ComboBox::outlineColourId, juce::Colour (0xff222426));
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
    hpfOnButton.setColour (juce::ToggleButton::textColourId, juce::Colour (0xffd7dadd));
    eqInButton.setColour  (juce::ToggleButton::textColourId, juce::Colour (0xffd7dadd));
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

void Neve1073EqAudioProcessorEditor::paint (juce::Graphics& g)
{
    // Brushed grey console faceplate, matching the preamp plugin.
    juce::ColourGradient panel (juce::Colour (0xff565a5e), 0.0f, 0.0f,
                                juce::Colour (0xff3a3d40), 0.0f, (float) getHeight(), false);
    g.setGradientFill (panel);
    g.fillAll();

    g.setColour (juce::Colour (0x20ffffff));
    g.drawHorizontalLine (1, 0.0f, (float) getWidth());

    // Marconi-style badge.
    auto title = getLocalBounds().removeFromTop (34);
    g.setColour (juce::Colour (0xfff0e6d2));
    g.setFont (juce::FontOptions (17.0f, juce::Font::bold));
    g.drawText ("NEVE", title.removeFromLeft (title.getWidth() / 4), juce::Justification::centred);
    g.setFont (juce::FontOptions (17.0f));
    g.drawText ("1073", title.removeFromLeft (title.getWidth() / 3), juce::Justification::centred);
    g.drawText ("EQ", title, juce::Justification::centred);

    // Corner screws, because it's a console.
    g.setColour (juce::Colour (0xff2a2c2e));
    for (auto c : { juce::Point<float> (10, 34), juce::Point<float> ((float) getWidth() - 10, 34),
                    juce::Point<float> (10, (float) getHeight() - 22),
                    juce::Point<float> ((float) getWidth() - 10, (float) getHeight() - 22) })
        g.fillEllipse (c.x - 3.5f, c.y - 3.5f, 7.0f, 7.0f);

    // Build stamp footer.
    g.setColour (juce::Colour (0xff8a8f94));
    g.setFont (juce::FontOptions (10.0f));
    g.drawText (juce::String ("build ") + EVEN_BUILD_ID + "  " + EVEN_BUILD_TIME,
                getLocalBounds().removeFromBottom (14), juce::Justification::centredRight);
}

void Neve1073EqAudioProcessorEditor::resized()
{
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
    hpfOnButton.setBounds (hpfArea.removeFromLeft (60).removeFromTop (22));
    hpfFreqBox.setBounds  (hpfArea.removeFromLeft (86).removeFromTop (22));
    eqInButton.setBounds (strip.removeFromLeft (80).removeFromTop (22));
    outTrimKnob.setBounds (strip.removeFromRight (90).removeFromTop (72));
    outTrimLabel.setBounds (outTrimKnob.getBounds().removeFromBottom (18));
}
