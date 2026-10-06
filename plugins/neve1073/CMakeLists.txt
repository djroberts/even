juce_add_plugin(
    neve1073
    PLUGIN_MANUFACTURER_CODE Even
    PLUGIN_CODE              Nv73
    FORMATS                  VST3 AU Standalone
    PRODUCT_NAME             "Neve 1073 Preamp"
    COMPANY_NAME             "even"
    IS_SYNTH                 FALSE
    NEEDS_MIDI_INPUT         FALSE
    NEEDS_MIDI_OUTPUT        FALSE
    IS_MIDI_EFFECT           FALSE
    COPY_PLUGIN_AFTER_BUILD  TRUE
)

# Build stamp: a generated 8-hex build id + wall-clock build time, refreshed on
# EVERY build (not a git hash -- builds get unique ids even before any commit).
# The stamp header is regenerated on every build via the always-run
# `stamp_update` target, which also echoes the id to the build console.

set (EVEN_STAMP_DIR ${CMAKE_CURRENT_BINARY_DIR}/stamp)
file (MAKE_DIRECTORY ${EVEN_STAMP_DIR})

# Generate the initial stamp at configure time so the first build has it.
execute_process(
    COMMAND ${CMAKE_COMMAND}
            -DSTAMP_DIR=${EVEN_STAMP_DIR}
            -P ${CMAKE_CURRENT_LIST_DIR}/MakeStamp.cmake
)

add_custom_target (stamp_update
    COMMAND ${CMAKE_COMMAND}
            -DSTAMP_DIR=${EVEN_STAMP_DIR}
            -P ${CMAKE_CURRENT_LIST_DIR}/MakeStamp.cmake
    BYPRODUCTS ${EVEN_STAMP_DIR}/BuildStamp.h
    COMMENT "Updating build stamp"
)

target_include_directories (neve1073 PRIVATE ${EVEN_STAMP_DIR})
add_dependencies (neve1073 stamp_update)

# After the VST3 is built and installed, print the bare build id as the last
# plugin-related line of the build output so it can be verified against the
# footer shown in Reaper.
add_custom_command (TARGET neve1073_VST3 POST_BUILD
    COMMAND ${CMAKE_COMMAND}
            -DSTAMP_DIR=${EVEN_STAMP_DIR}
            -P ${CMAKE_CURRENT_LIST_DIR}/PrintBuildId.cmake
    COMMENT "Build id"
    VERBATIM
)

target_sources(neve1073 PRIVATE
    Source/PluginProcessor.cpp
    Source/PluginEditor.cpp
)

target_compile_definitions(neve1073 PUBLIC
    JUCE_WEB_BROWSER=0
    JUCE_USE_CURL=0
    JUCE_VST3_CAN_REPLACE_VST2=0
)

target_link_libraries(neve1073
    PRIVATE
        juce::juce_audio_utils
        juce::juce_dsp
        even_common
    PUBLIC
        juce::juce_recommended_config_flags
        juce::juce_recommended_lto_flags
        juce::juce_recommended_warning_flags
)

# CLAP build (via clap-juce-extensions; requires JUCE target defined above)
clap_juce_extensions_plugin(TARGET neve1073
    CLAP_ID "com.even.neve1073"
    CLAP_FEATURES audio-effect utility "channel amplifier" "preamp" saturation
)
