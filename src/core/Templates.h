// Built-in starting points: an environment with a floor and lights, and example creatures.
#pragma once

#include <string>
#include <vector>

#include "core/Document.h"

namespace mjf {

struct TemplateInfo {
  const char* id;
  const char* name;
  const char* description;
};
const std::vector<TemplateInfo>& templateList();
// Returns false if the template id is unknown.
bool makeTemplate(const std::string& id, Document& out);

// Floor (checker), skybox, a directional key light and viewer-friendly visual settings.
void addDefaultEnvironment(Document& doc);
// Creatures. Each returns the root body ID. `name` becomes the root body name / prefix.
Id addQuadruped(Document& doc, const Vec3& pos, const std::string& name = "quadruped");
Id addHexapod(Document& doc, const Vec3& pos, const std::string& name = "hexapod");
Id addBiped(Document& doc, const Vec3& pos, const std::string& name = "biped");
Id addArm(Document& doc, const Vec3& pos, const std::string& name = "arm");

}  // namespace mjf
