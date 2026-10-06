# Prints the bare build id (no decoration) from the stamp dir.
file(READ "${STAMP_DIR}/BuildId.txt" build_id)
string(STRIP "${build_id}" build_id)
message("${build_id}")
