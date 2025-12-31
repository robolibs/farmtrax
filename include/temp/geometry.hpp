#pragma once

/**
 * @file geometry.hpp
 * @brief Temporary geometry algorithms for farmtrax
 *
 * This header provides geometry algorithms that are not available in datapod.
 * These are temporary implementations that may be moved to datapod in the future.
 *
 * Includes:
 * - buffer.hpp: Polygon buffer/offset operations (shrink/expand)
 * - clip.hpp: Line-polygon intersection/clipping
 * - boolean.hpp: Line-polygon difference operations
 * - validate.hpp: Polygon validation and winding order correction
 * - centroid.hpp: Centroid calculation
 * - uuid.hpp: UUID generation
 */

#include "boolean.hpp"
#include "buffer.hpp"
#include "centroid.hpp"
#include "clip.hpp"
#include "uuid.hpp"
#include "validate.hpp"
