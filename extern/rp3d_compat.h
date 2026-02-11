#pragma once
#include <cstdint>

// Only define if not already defined in this translation unit
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

#endif  // RP3D_UINT_TYPES_DEFINED
