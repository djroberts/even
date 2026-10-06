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
void Neve1073LookAndFeel::drawRotarySlider (juce::Graphics& g, int x, int y, int w, int h,
                                            float sliderPosProportional,
                                            const float rotaryStartAngle,
                                            const float rotaryEndAngle,
                                            juce::Slider&)
{
    const auto centre = juce::Rectangle<float> ((float) x, (float) y, (float) w, (float) h).getCentre();
    const float radius = juce::jmin (w, h) * 0.5f;
    const float angle  = rotaryStartAngle
                       + sliderPosProportional * (rotaryEndAngle - rotaryStartAngle);

    // ---- calibration ticks + labels (drawn on the faceplate) --------------
    const float tickOuter = radius * 0.94f;
    const float tickInner = radius * 0.84f;
    g.setFont (juce::FontOptions (juce::jmax (9.0f, radius * 0.15f)));
    for (auto db : tickLabels)
    {
        const float prop = juce::jlimit (0.0f, 1.0f, ((float) db + 80.0f) / 90.0f);
        const float a    = rotaryStartAngle + prop * (rotaryEndAngle - rotaryStartAngle);
        const float ca = std::cos (a), sa = std::sin (a);

        g.setColour (juce::Colour (0xffd7dadd));
        g.drawLine ({ centre.x + tickInner * ca, centre.y + tickInner * sa,
                      centre.x + tickOuter * ca, centre.y + tickOuter * sa }, 1.5f);

        const float labelR = radius * 0.70f;
        auto labelBox = juce::Rectangle<float> (labelR * 2.0f, labelR * 0.5f);
        labelBox.setCentre (centre.x + labelR * ca, centre.y + labelR * sa);
        g.setColour (juce::Colour (0xffb9bec3));
        g.drawText (db > 0 ? "+10" : juce::String (db),
                    (int) labelBox.getX(), (int) labelBox.getY(),
                    (int) labelBox.getWidth(), (int) labelBox.getHeight(),
                    juce::Justification::centred, false);
    }

    // ---- knob cap: classic red 1073 gain knob -----------------------------
    const float capR = radius * 0.58f;
    juce::ColourGradient capGrad (juce::Colour (0xffe0554a), centre.x - capR * 0.4f,
                                  centre.y - capR * 0.6f,
                                  juce::Colour (0xff8f1d14), centre.x + capR * 0.3f,
                                  centre.y + capR * 0.5f, true);
    g.setGradientFill (capGrad);
    g.fillEllipse (centre.x - capR, centre.y - capR, capR * 2.0f, capR * 2.0f);

    g.setColour (juce::Colour (0xff2a2626));
    g.drawEllipse (centre.x - capR, centre.y - capR, capR * 2.0f, capR * 2.0f, 2.0f);

    // inner skirt ring, like the machined ridge on the real cap
    g.setColour (juce::Colour (0x40ffffff));
    g.drawEllipse (centre.x - capR * 0.72f, centre.y - capR * 0.72f,
                   capR * 1.44f, capR * 1.44f, 1.0f);

    // ---- ivory pointer ----------------------------------------------------
    // Thin line from the centre hub out towards the cap edge, like the
    // printed pointer on the real 1073 knob.
    const float pLen = capR * 0.85f;
    const float pW   = capR * 0.10f;
    juce::Path pointer;
    pointer.addRoundedRectangle (-pW * 0.5f, -pLen, pW, pLen, pW * 0.5f);
    g.setColour (juce::Colour (0xfff0e6d2));
    g.fillPath (pointer, juce::AffineTransform::rotation (angle).translated (centre));

    // dark centre hub on top of the pointer base
    const float hubR = capR * 0.16f;
    g.setColour (juce::Colour (0xff2a2626));
    g.fillEllipse (centre.x - hubR, centre.y - hubR, hubR * 2.0f, hubR * 2.0f);
}

//==============================================================================
Neve1073AudioProcessorEditor::Neve1073AudioProcessorEditor (Neve1073AudioProcessor& p)
    : AudioProcessorEditor (p),
      processorRef (p),
      gainAttachment (p.gainParam, gainKnob)
{
    setLookAndFeel (&lookAndFeel);

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

    setSize (280, 230);
}

Neve1073AudioProcessorEditor::~Neve1073AudioProcessorEditor()
{
    setLookAndFeel (nullptr);
}

void Neve1073AudioProcessorEditor::paint (juce::Graphics& g)
{
    // Brushed grey console faceplate
    juce::ColourGradient panel (juce::Colour (0xff565a5e), 0.0f, 0.0f,
                                juce::Colour (0xff3a3d40), 0.0f, (float) getHeight(), false);
    g.setGradientFill (panel);
    g.fillAll();

    g.setColour (juce::Colour (0x20ffffff));
    g.drawHorizontalLine (1, 0.0f, (float) getWidth());

    // Marconi-style badge
    auto title = getLocalBounds().removeFromTop (34);
    g.setColour (juce::Colour (0xfff0e6d2));
    g.setFont (juce::FontOptions (17.0f, juce::Font::bold));
    g.drawText ("NEVE", title.removeFromLeft (title.getWidth() / 2), juce::Justification::centred);
    g.setFont (juce::FontOptions (17.0f));
    g.drawText ("1073", title, juce::Justification::centred);

    // Corner screws, because it's a console
    g.setColour (juce::Colour (0xff2a2c2e));
    for (auto c : { juce::Point<float> (10, 34), juce::Point<float> ((float) getWidth() - 10, 34),
                    juce::Point<float> (10, (float) getHeight() - 22),
                    juce::Point<float> ((float) getWidth() - 10, (float) getHeight() - 22) })
        g.fillEllipse (c.x - 3.5f, c.y - 3.5f, 7.0f, 7.0f);

    // Build stamp footer: proves which binary is actually loaded.
    g.setColour (juce::Colour (0xff8a8f94));
    g.setFont (juce::FontOptions (10.0f));
    g.drawText (juce::String ("build ") + EVEN_BUILD_ID + "  " + EVEN_BUILD_TIME,
                getLocalBounds().removeFromBottom (14), juce::Justification::centredRight);
}

void Neve1073AudioProcessorEditor::resized()
{
    auto body = getLocalBounds().removeFromTop (getHeight() - 14); // keep build stamp clear
    body.removeFromTop (34);                                       // badge

    gainKnob.setBounds (body.removeFromTop (body.getHeight() - 22).reduced (26));
    gainLabel.setBounds (body.removeFromTop (22));
}
