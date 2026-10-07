juce_add_plugin(
    neve1073eq
    PLUGIN_MANUFACTURER_CODE Even
    PLUGIN_CODE              Nv7E
    FORMATS                  VST3 AU Standalone
    PRODUCT_NAME             "Neve 1073 EQ"
    COMPANY_NAME             "even"
    IS_SYNTH                 FALSE
    NEEDS_MIDI_INPUT         FALSE
    NEEDS_MIDI_OUTPUT        FALSE
    IS_MIDI_EFFECT           FALSE
    COPY_PLUGIN_AFTER_BUILD  TRUE
)

# Build stamp: shared with the neve1073 preamp plugin (same scripts, separate
# stamp dir so each plugin reports its own build id/time).
set (EVEN_STAMP_DIR ${CMAKE_CURRENT_BINARY_DIR}/stamp)
file (MAKE_DIRECTORY ${EVEN_STAMP_DIR})

execute_process(
    COMMAND ${CMAKE_COMMAND}
            -DSTAMP_DIR=${EVEN_STAMP_DIR}
            -P ${CMAKE_CURRENT_LIST_DIR}/../neve1073/MakeStamp.cmake
)

add_custom_target (stamp_update_eq
    COMMAND ${CMAKE_COMMAND}
            -DSTAMP_DIR=${EVEN_STAMP_DIR}
            -P ${CMAKE_CURRENT_LIST_DIR}/../neve1073/MakeStamp.cmake
    BYPRODUCTS ${EVEN_STAMP_DIR}/BuildStamp.h
    COMMENT "Updating build stamp"
)

target_include_directories (neve1073eq PRIVATE ${EVEN_STAMP_DIR})
add_dependencies (neve1073eq stamp_update_eq)

add_custom_command (TARGET neve1073eq_VST3 POST_BUILD
    COMMAND ${CMAKE_COMMAND}
            -DSTAMP_DIR=${EVEN_STAMP_DIR}
            -P ${CMAKE_CURRENT_LIST_DIR}/../neve1073/PrintBuildId.cmake
    COMMENT "Build id"
    VERBATIM
)

target_sources(neve1073eq PRIVATE
    Source/PluginProcessor.cpp
    Source/PluginEditor.cpp
)

target_compile_definitions(neve1073eq PUBLIC
    JUCE_WEB_BROWSER=0
    JUCE_USE_CURL=0
    JUCE_VST3_CAN_REPLACE_VST2=0
)

target_link_libraries(neve1073eq
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
clap_juce_extensions_plugin(TARGET neve1073eq
    CLAP_ID "com.even.neve1073eq"
    CLAP_FEATURES audio-effect utility eq "channel amplifier"
)

# ---- Smoke tests: steady-state gain measurement through the circuit model --
# Run with: cmake --build build --target neve1073eq_tests && ./build/neve1073eq/neve1073eq_tests
add_executable (neve1073eq_tests EXCLUDE_FROM_ALL Source/tests/eq_test.cpp)
target_compile_features (neve1073eq_tests PRIVATE cxx_std_20)
target_compile_definitions (neve1073eq_tests PRIVATE
    JUCE_WEB_BROWSER=0
    JUCE_USE_CURL=0
)
target_link_libraries (neve1073eq_tests PRIVATE
    juce::juce_dsp
    juce::juce_audio_utils
    juce::juce_recommended_config_flags
    even_common
)
