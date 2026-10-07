#pragma once

#include <juce_gui_basics/juce_gui_basics.h>
#include "even/gui/GuiAssets.h"

namespace even::gui
{

/** The shared brushed-steel faceplate: SVG panel, corner screws, engraved
    Marconi-style badge, and the build-stamp footer. Renders once per size
    into a cached image. Not mouse-interactive. */
class ConsoleFaceplate : public juce::Component
{
public:
    ConsoleFaceplate (juce::String badgeLine1, juce::String badgeLine2, juce::String footerText = {})
        : badge1 (std::move (badgeLine1)), badge2 (std::move (badgeLine2)), footer (std::move (footerText))
    {
        setInterceptsMouseClicks (false, false);
    }

    void paint (juce::Graphics& g) override
    {
        g.drawImage (cached, getLocalBounds().toFloat(), juce::RectanglePlacement::stretchToFit);
    }

    void resized() override
    {
        if (getWidth() > 0 && getHeight() > 0)
            cached = render (getWidth(), getHeight());
    }

private:
    juce::Image render (int w, int h) const
    {
        juce::Image img (juce::Image::ARGB, w, h, true);
        juce::Graphics g (img);

        if (auto panel = createDrawableFromSvgString (makeFaceplateSvg()))
            panel->drawWithin (g, img.getBounds().toFloat(), juce::RectanglePlacement::stretchToFit, 1.0f);

        // ---- corner screws (with a 45-degree machinist slot) ------------------
        auto screw = [&] (juce::Point<float> c)
        {
            const float r = 4.2f;
            juce::ColourGradient gr (juce::Colour (0xff767a7e), c.x - r * 0.4f, c.y - r * 0.6f,
                                     juce::Colour (0xff2b2e31), c.x + r * 0.4f, c.y + r * 0.6f, true);
            g.setGradientFill (gr);
            g.fillEllipse (c.x - r, c.y - r, r * 2.0f, r * 2.0f);
            g.setColour (juce::Colour (0xff17191b));
            g.drawEllipse (c.x - r, c.y - r, r * 2.0f, r * 2.0f, 1.0f);

            juce::Path slot;
            slot.addRoundedRectangle (-r * 0.7f, -0.9f, r * 1.4f, 1.8f, 0.9f);
            g.setColour (juce::Colour (0xd9131517));
            g.fillPath (slot, juce::AffineTransform::rotation (juce::MathConstants<float>::pi * 0.25f).translated (c));
        };
        screw ({ 12.0f, 34.0f });
        screw ({ (float) w - 12.0f, 34.0f });
        screw ({ 12.0f, (float) h - 24.0f });
        screw ({ (float) w - 12.0f, (float) h - 24.0f });

        // ---- engraved Marconi-style badge -------------------------------------
        // Bold name on the left half, model number on the right half.
        const juce::Colour ivory (0xfff0e6d2);
        const juce::Colour shadow (0xd9101214);
        const juce::FontOptions boldOpt (17.0f, juce::Font::bold);
        const juce::FontOptions regOpt  (17.0f);
        const juce::Font fb (boldOpt), fr (regOpt);

        auto badgeBounds = juce::Rectangle<int> (0, 2, w, 30);
        auto leftHalf  = badgeBounds.removeFromLeft (badgeBounds.getWidth() / 2);
        auto rightHalf = badgeBounds;

        g.setFont (fb);
        g.setColour (shadow);
        g.drawText (badge1, leftHalf.translated (1, 1), juce::Justification::centred);
        g.setFont (fr);
        g.drawText (badge2, rightHalf.translated (1, 1), juce::Justification::centred);

        g.setFont (fb);
        g.setColour (ivory);
        g.drawText (badge1, leftHalf, juce::Justification::centred);
        g.setFont (fr);
        g.drawText (badge2, rightHalf, juce::Justification::centred);

        // ---- footer: build stamp ----------------------------------------------
        if (footer.isNotEmpty())
        {
            g.setColour (juce::Colour (0xff8a8f94));
            g.setFont (juce::FontOptions (10.0f));
            g.drawText (footer, juce::Rectangle<int> (0, h - 14, w - 8, 14),
                        juce::Justification::centredRight);
        }

        // ---- outer bevel --------------------------------------------------------
        g.setColour (juce::Colours::black.withAlpha (0.45f));
        g.drawRect (0, 0, w, h, 1);

        return img;
    }

    juce::String badge1, badge2, footer;
    juce::Image cached;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (ConsoleFaceplate)
};

} // namespace even::gui
