cmake_minimum_required(VERSION 3.24)

project(EvenPlugins VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---- Dependencies (referenced, fetched at configure time; never committed) ----
include(FetchContent)

FetchContent_Declare(
    JUCE
    GIT_REPOSITORY https://github.com/juce-framework/JUCE.git
    GIT_TAG        9.0.3   # https://github.com/juce-framework/JUCE/releases
    GIT_SHALLOW    TRUE
)

# No tagged releases upstream; pinned to latest main commit (2026-09-12).
FetchContent_Declare(
    clap_juce_extensions
    GIT_REPOSITORY https://github.com/free-audio/clap-juce-extensions.git
    GIT_TAG        55525c9858d4b25687be7759a5e0f70eccef218e
    GIT_SHALLOW    TRUE
)

FetchContent_MakeAvailable(JUCE clap_juce_extensions)

# ---- Shared code: component-level circuit modeling primitives ----
add_subdirectory(common)

# ---- Plugins ----
add_subdirectory(neve1073)
add_subdirectory(neve1073eq)
