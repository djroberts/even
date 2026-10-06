# Writes BuildStamp.h into STAMP_DIR. Generates a fresh build hash on every
# run: an 8-hex-char id derived from the wall-clock timestamp (seeded random),
# so each build -- even with zero code changes -- gets its own id. This
# identifies the *build*, not the source state (there is no git dependency:
# works fine before any commits exist).
string(TIMESTAMP build_time "%Y-%m-%d %H:%M:%S")
string(TIMESTAMP seed "%Y%m%d%H%M%S")
string(RANDOM LENGTH 8 ALPHABET "0123456789abcdef" RANDOM_SEED ${seed} build_hash)
# Show the fresh build id in the build console on every build.
message(STATUS "── neve1073 build ${build_hash}  ${build_time} ──")
set(stamp_content
"// Auto-generated build stamp - do not edit.
#define EVEN_BUILD_ID \"${build_hash}\"
#define EVEN_BUILD_TIME \"${build_time}\"
")
set(stamp_file "${STAMP_DIR}/BuildStamp.h")
set(old "")
if (EXISTS "${stamp_file}")
    file(READ "${stamp_file}" old)
endif()
if (NOT old STREQUAL stamp_content)
    file(WRITE "${stamp_file}" "${stamp_content}")
endif()

# Bare id (hash only, no decoration) so POST_BUILD can print it as the last
# line of the build output for easy comparison against the plugin UI in the DAW.
file(WRITE "${STAMP_DIR}/BuildId.txt" "${build_hash}\n")
