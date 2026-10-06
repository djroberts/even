# Shared circuit-modeling library used by all "even" plugins.
# Newton–Raphson nodal (DK-method) solver, component models, etc. land here.

add_library(even_common INTERFACE)
target_include_directories(even_common INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(even_common INTERFACE cxx_std_20)
