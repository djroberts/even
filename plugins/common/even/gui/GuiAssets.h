#pragma once

#include <juce_gui_basics/juce_gui_basics.h>

namespace even::gui
{

/** Parse an SVG string into a Drawable. Returns nullptr on malformed input. */
inline std::unique_ptr<juce::Drawable> createDrawableFromSvgString (const juce::String& svg)
{
    return juce::Drawable::createFromImageData (svg.toRawUTF8(), (size_t) svg.getNumBytesAsUTF8());
}

/** Brushed-steel console faceplate. A 100x100 viewBox stretched to fit, so the
    vertical shading and the fine horizontal brushing lines survive any size.
    Screws and badge are drawn by ConsoleFaceplate (aspect-safe, live text). */
inline juce::String makeFaceplateSvg()
{
    juce::String s (
        R"svg(<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100" preserveAspectRatio="none">
<defs>
<linearGradient id="panel" x1="0" y1="0" x2="0" y2="1">
<stop offset="0" stop-color="#5d6165"/>
<stop offset="0.06" stop-color="#4e5155"/>
<stop offset="0.45" stop-color="#43464a"/>
<stop offset="0.92" stop-color="#34373b"/>
<stop offset="1" stop-color="#2c2f33"/>
</linearGradient>
<linearGradient id="toplight" x1="0" y1="0" x2="0" y2="1">
<stop offset="0" stop-color="#ffffff" stop-opacity="0.14"/>
<stop offset="1" stop-color="#ffffff" stop-opacity="0"/>
</linearGradient>
<linearGradient id="bottomshade" x1="0" y1="0" x2="0" y2="1">
<stop offset="0" stop-color="#000000" stop-opacity="0"/>
<stop offset="1" stop-color="#000000" stop-opacity="0.18"/>
</linearGradient>
</defs>
<rect width="100" height="100" fill="url(#panel)"/>
<rect width="100" height="8" fill="url(#toplight)"/>
<rect y="92" width="100" height="8" fill="url(#bottomshade)"/>)svg");

    // Fine brushed texture: alternating light/dark micro-rows.
    for (int i = 0; i < 64; ++i)
    {
        const float y   = (float) i * (100.0f / 64.0f);
        const bool  lit = (i % 2) == 0;
        s << "<rect y=\"" << juce::String (y, 2)
          << "\" width=\"100\" height=\"0.55\" fill=\""
          << (lit ? "#ffffff" : "#000000")
          << "\" fill-opacity=\"" << (lit ? "0.020" : "0.016") << "\"/>";
    }

    s << "</svg>";
    return s;
}

/** Rotary knob cap. Red = the classic 1073 gain knob; Grey = the EQ pots. */
inline juce::String makeKnobSvg (bool red)
{
    const char* capStops = red
        ? "<stop offset=\"0\" stop-color=\"#f5806f\"/>"
          "<stop offset=\"0.42\" stop-color=\"#c93d2e\"/>"
          "<stop offset=\"0.78\" stop-color=\"#8c1c12\"/>"
          "<stop offset=\"1\" stop-color=\"#661108\"/>"
        : "<stop offset=\"0\" stop-color=\"#a9adb2\"/>"
          "<stop offset=\"0.42\" stop-color=\"#6d7176\"/>"
          "<stop offset=\"0.78\" stop-color=\"#43464a\"/>"
          "<stop offset=\"1\" stop-color=\"#313437\"/>";

    juce::String s (R"svg(<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" viewBox="-50 -50 100 100">
<defs>
<radialGradient id="cap" cx="0.38" cy="0.30" r="0.80">)svg");
    s << capStops << R"svg(</radialGradient>
<linearGradient id="sheen" x1="0" y1="0" x2="0" y2="1">
<stop offset="0" stop-color="#ffffff" stop-opacity="0.26"/>
<stop offset="0.35" stop-color="#ffffff" stop-opacity="0"/>
<stop offset="0.85" stop-color="#000000" stop-opacity="0"/>
<stop offset="1" stop-color="#000000" stop-opacity="0.28"/>
</linearGradient>
</defs>
<circle r="48" fill="#211e1d"/>
<circle r="45" fill="url(#cap)"/>
<circle r="45" fill="url(#sheen)"/>
<circle r="30" fill="none" stroke="#ffffff" stroke-opacity="0.13" stroke-width="1.8"/>
<circle r="28.5" fill="none" stroke="#000000" stroke-opacity="0.28" stroke-width="1"/>
<circle r="8" fill="#2a2626"/>
<circle r="8" fill="none" stroke="#000000" stroke-opacity="0.45" stroke-width="1"/>
<circle cx="-2.6" cy="-2.6" r="2.1" fill="#ffffff" fill-opacity="0.20"/>
</svg>)svg";
    return s;
}

} // namespace even::gui
