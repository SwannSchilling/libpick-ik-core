#pragma once

// Vendored RSL export-macro shim.
//
// Upstream RSL generates `rsl/export.hpp` with CMake's GenerateExportHeader.
// pick_ik_core builds the vendored RSL sources into a static library, so no
// export decoration is required.

#define RSL_EXPORT
