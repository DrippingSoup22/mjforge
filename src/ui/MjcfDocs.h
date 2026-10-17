// MJCF attribute documentation (from the MuJoCo XML reference) for tooltips.
#pragma once

namespace mjf {

// Key format: "<element>.<attribute>", e.g. "geom.friction", "position.kp", "option.timestep".
// Returns nullptr if unknown.
const char* mjcfDoc(const char* key);

}  // namespace mjf
