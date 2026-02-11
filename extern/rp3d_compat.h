#pragma once
#include <cstdint>

// Guard to avoid redefinition if RP3D introduces these later
#ifndef RP3D_UINT_TYPES_DEFINED
#define RP3D_UINT_TYPES_DEFINED

#ifndef uint32
using uint32 = uint32_t;
#endif
#ifndef uint64
using uint64 = uint64_t;
#endif
#ifndef int32
using int32 = int32_t;
#endif
#ifndef int64
using int64 = int64_t;
#endif

// (Optional) cover other widths if they pop up
#ifndef uint8
using uint8 = uint8_t;
#endif
#ifndef int16
using int16 = int16_t;
#endif

#endif // RP3D_UINT_TYPES_DEFINED