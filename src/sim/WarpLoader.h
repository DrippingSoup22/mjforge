// Generates a ready-to-run Python script that loads an exported model in MuJoCo Warp with buffer
// sizes (nconmax / njmax) derived from measured contact peaks.
#pragma once

#include <string>

#include "core/Document.h"
#include "sim/Simulator.h"

namespace mjf {

// Peaks of a short headless settle (Hold mode) of the document: what the model needs at least,
// even when nobody pressed Play. Returns false if the model doesn't compile.
bool measureSettleSizing(const Document& doc, WarpSizing* out, double seconds = 1.5);

// `measured`: peaks from a simulation preview, or nullptr to measure a settle now.
std::string mjwarpLoaderScript(const Document& doc, const std::string& xmlFile, const WarpSizing* measured);

}  // namespace mjf
