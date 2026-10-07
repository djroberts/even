#pragma once

#include <juce_gui_basics/juce_gui_basics.h>
#include "even/gui/GuiAssets.h"

namespace even::gui
{

/** Shared vintage-console look for all "even" plugins: asset-rendered rotary
    knobs (red gain / grey EQ pots) with ivory pointers, recessed combo fields,
    and LED-style toggle switches. */
class EvenLookAndFeel : public juce::LookAndFeel_V4
{
public:
    enum class KnobStyle { Red, Grey };

    explicit EvenLookAndFeel (KnobStyle style = KnobStyle::Grey)
    {
        knobAsset = createDrawableFromSvgString (makeKnobSvg (style == KnobStyle::Red));

        const juce::Colour ivory (0xffd7dadd);
        setColour (juce::Label::textColourId, ivory);
        setColour (juce::ToggleButton::textColourId, ivory);
        setColour (juce::ToggleButton::tickColourId, juce::Colour (0xffe0554a));
        setColour (juce::ToggleButton::tickDisabledColourId, juce::Colour (0xff565a5e));
        setColour (juce::ComboBox::textColourId, ivory);
        setColour (juce::ComboBox::backgroundColourId, juce::Colour (0xff2e3134));
        setColour (juce::ComboBox::arrowColourId, ivory);
        setColour (juce::ComboBox::outlineColourId, juce::Colour (0xff1c1e20));
        setColour (juce::ComboBox::buttonColourId, juce::Colour (0xff2e3134));
        setColour (juce::PopupMenu::backgroundColourId, juce::Colour (0xff2e3134));
        setColour (juce::PopupMenu::highlightedBackgroundColourId, juce::Colour (0xff7c160e));
        setColour (juce::PopupMenu::textColourId, ivory);
        setColour (juce::PopupMenu::highlightedTextColourId, juce::Colour (0xfff0e6d2));
        setColour (juce::Slider::textBoxTextColourId, ivory);
        setColour (juce::Slider::textBoxOutlineColourId, juce::Colours::transparentBlack);
        setColour (juce::Slider::textBoxBackgroundColourId, juce::Colour (0x40222426));
        setColour (juce::Slider::textBoxHighlightColourId, juce::Colour (0xff7c160e));
    }

    /** Optional calibration ticks printed around the knob (faceplate text).
        props are the 0..1 rotational positions of each tick. */
    void setTickMarks (const juce::StringArray& textsIn, const std::vector<float>& propsIn)
    {
        tickTexts = textsIn;
        tickProps = propsIn;
    }

    void drawRotarySlider (juce::Graphics& g, int x, int y, int w, int h,
                           float sliderPosProportional,
                           const float rotaryStartAngle,
                           const float rotaryEndAngle,
                           juce::Slider&) override
    {
        const auto centre = juce::Rectangle<float> ((float) x, (float) y, (float) w, (float) h).getCentre();
        const float r     = juce::jmin (w, h) * 0.5f;
        const float capR  = r * 0.90f;
        const float angle = rotaryStartAngle
                          + sliderPosProportional * (rotaryEndAngle - rotaryStartAngle);

        // ---- soft drop shadow pooling under the knob -------------------------
        {
            const auto sc = centre + juce::Point<float> (0.0f, r * 0.07f);
            const float shR = capR * 1.18f;
            juce::ColourGradient sh (juce::Colours::black.withAlpha (0.42f),
                                     sc.x, sc.y,
                                     juce::Colours::black.withAlpha (0.0f),
                                     sc.x, sc.y + shR, true);
            g.setGradientFill (sh);
            g.fillEllipse (sc.x - shR, sc.y - shR, shR * 2.0f, shR * 2.0f);
        }

        // ---- knob cap asset ---------------------------------------------------
        if (knobAsset != nullptr)
            knobAsset->drawWithin (g, juce::Rectangle<float> (capR * 2.0f, capR * 2.0f).withCentre (centre),
                                   juce::RectanglePlacement::centred, 1.0f);

        // ---- ivory pointer, with its own tiny shadow --------------------------
        const float pLen = capR * 0.72f;
        const float pW   = juce::jmax (2.0f, capR * 0.085f);
        juce::Path pointer;
        pointer.addRoundedRectangle (-pW * 0.5f, -pLen, pW, pLen, pW * 0.5f);

        g.saveState();
        g.setColour (juce::Colours::black.withAlpha (0.35f));
        g.fillPath (pointer, juce::AffineTransform::rotation (angle).translated (centre.x + 0.8f, centre.y + 1.4f));
        g.setColour (juce::Colour (0xfff0e6d2));
        g.fillPath (pointer, juce::AffineTransform::rotation (angle).translated (centre));
        g.restoreState();
    }

private:
    void drawTicks (juce::Graphics& g, juce::Point<float> centre, float r,
                    float rotaryStartAngle, float rotaryEndAngle)
    {
        const float tickOuter = r * 0.985f;
        const float tickInner = r * 0.90f;
        g.setFont (juce::FontOptions (juce::jmax (9.0f, r * 0.14f), juce::Font::bold));
        for (int i = 0; i < tickTexts.size(); ++i)
        {
            const float a  = rotaryStartAngle + tickProps[(size_t) i] * (rotaryEndAngle - rotaryStartAngle);
            const float ca = std::cos (a), sa = std::sin (a);

            g.setColour (juce::Colour (0xffd7dadd));
            g.drawLine ({ centre.x + tickInner * ca, centre.y + tickInner * sa,
                          centre.x + tickOuter * ca, centre.y + tickOuter * sa }, 1.6f);

            const float labelR = r * 0.72f;
            auto labelBox = juce::Rectangle<float> (labelR * 2.0f, labelR * 0.5f);
            labelBox.setCentre (centre.x + labelR * ca, centre.y + labelR * sa);
            g.setColour (juce::Colour (0xffb9bec3));
            g.drawText (tickTexts[i], (int) labelBox.getX(), (int) labelBox.getY(),
                        (int) labelBox.getWidth(), (int) labelBox.getHeight(),
                        juce::Justification::centred, false);
        }
    }

public:
    void drawToggleButton (juce::Graphics& g, juce::ToggleButton& b,
                           bool, bool) override
    {
        const auto bounds = b.getLocalBounds().toFloat();
        const bool on     = b.getToggleState();

        // Recessed metal switch slot on the left.
        const float slotH  = juce::jmin (bounds.getHeight(), 18.0f);
        const float slotW  = slotH * 2.0f;
        auto slot = juce::Rectangle<float> (slotW, slotH).withY (bounds.getCentreY() - slotH * 0.5f)
                                                        .withX (bounds.getX());
        g.setColour (juce::Colour (0xff232527));
        g.fillRoundedRectangle (slot, slotH * 0.5f);
        g.setColour (juce::Colours::black.withAlpha (0.55f));
        g.drawRoundedRectangle (slot.reduced (0.5f), slotH * 0.5f, 1.0f);
        g.setColour (juce::Colours::white.withAlpha (0.09f));
        g.drawRoundedRectangle (slot.reduced (1.6f), juce::jmax (0.5f, slotH * 0.5f - 1.1f), 1.0f);

        // LED pilot lamp: red when engaged, dead grey when not.
        const float ledR = slotH * 0.27f;
        const auto  ledC = slot.getCentre();
        if (on)
        {
            juce::ColourGradient glow (juce::Colour (0x60e0554a), ledC.x, ledC.y,
                                       juce::Colours::transparentBlack, ledC.x, ledC.y + ledR * 2.4f, true);
            g.setGradientFill (glow);
            g.fillEllipse (ledC.x - ledR * 2.4f, ledC.y - ledR * 2.4f, ledR * 4.8f, ledR * 4.8f);
        }
        g.setColour (on ? juce::Colour (0xffff6a55) : juce::Colour (0xff4b4f53));
        g.fillEllipse (ledC.x - ledR, ledC.y - ledR, ledR * 2.0f, ledR * 2.0f);
        g.setColour (on ? juce::Colour (0xff8f1d14) : juce::Colour (0xff222426));
        g.drawEllipse (ledC.x - ledR, ledC.y - ledR, ledR * 2.0f, ledR * 2.0f, 1.0f);

        // Legend to the right of the switch.
        auto textArea = juce::Rectangle<float> (bounds.getWidth() - (slot.getRight() + 6.0f - bounds.getX()),
                                                bounds.getHeight())
                            .withX (slot.getRight() + 6.0f).withY (bounds.getY());
        g.setFont (juce::FontOptions (12.0f, juce::Font::bold));
        g.setColour (on ? juce::Colour (0xfff0e6d2) : juce::Colour (0xffb9bec3));
        g.drawText (b.getButtonText(), textArea, juce::Justification::centredLeft);
    }

    void drawComboBox (juce::Graphics& g, int width, int height, bool,
                       int, int, int, int, juce::ComboBox& box) override
    {
        auto b = juce::Rectangle<float> (0.0f, 0.0f, (float) width, (float) height);

        // Recessed dark field with a top inner shadow.
        g.setColour (box.findColour (juce::ComboBox::backgroundColourId));
        g.fillRoundedRectangle (b, 3.0f);
        g.setColour (juce::Colours::black.withAlpha (0.35f));
        g.fillRoundedRectangle (b.removeFromTop (b.getHeight() * 0.45f), 3.0f);
        g.setColour (box.findColour (juce::ComboBox::backgroundColourId));
        g.fillRoundedRectangle (b.reduced (0.0f, 1.5f), 3.0f);

        g.setColour (box.findColour (juce::ComboBox::outlineColourId));
        g.drawRoundedRectangle (b.reduced (0.5f, 1.5f), 3.0f, 1.2f);

        // Small engraved arrow.
        auto arrow = b.removeFromRight (b.getHeight() * 0.6f).withSizeKeepingCentre (10.0f, 5.0f);
        juce::Path tri;
        tri.addTriangle (arrow.getTopLeft(), arrow.getTopRight(),
                         { arrow.getCentreX(), arrow.getBottom() });
        g.setColour (juce::Colour (0xffb9bec3));
        g.fillPath (tri);
    }

    void positionComboBoxText (juce::ComboBox& box, juce::Label& label) override
    {
        auto b = box.getLocalBounds();
        b.removeFromRight ((int) ((float) b.getHeight() * 0.6f)); // arrow
        label.setBounds (b.reduced (9, 2));
        label.setJustificationType (juce::Justification::centredLeft);
        label.setFont (juce::FontOptions (13.0f));
        label.setColour (juce::Label::textColourId, box.findColour (juce::ComboBox::textColourId));
    }

    void drawLabel (juce::Graphics& g, juce::Label& label) override
    {
        g.setFont (label.getFont());
        g.setColour (label.findColour (juce::Label::textColourId));
        g.drawText (label.getText(), label.getLocalBounds(), label.getJustificationType());
    }

private:
    std::unique_ptr<juce::Drawable> knobAsset;
    juce::StringArray tickTexts;
    std::vector<float> tickProps;
};

} // namespace even::gui

