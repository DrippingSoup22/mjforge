// Editing operations on a Document. All functions keep references consistent and call touch().
#pragma once

#include <string>
#include <vector>

#include "core/Document.h"

namespace mjf {

// ----- creation -----
Id addBody(Document& doc, Id parent, const Pose& local, const std::string& name = "body", Role role = Role::Model);
Id addGeom(Document& doc, Id body, GeomType type, const Vec3& size, const Pose& local, const std::string& name = "");
Id addJoint(Document& doc, Id body, JointType type, const Vec3& pos = {}, const Vec3& axis = {0, 0, 1},
            const std::string& name = "");
Id addSite(Document& doc, Id body, const Pose& local, const std::string& name = "");
Id addCamera(Document& doc, Id body, const Pose& local, const std::string& name = "");
Id addLight(Document& doc, Id body, const Vec3& pos, const Vec3& dir, const std::string& name = "");
Id addActuator(Document& doc, ActuatorType type, Id joint, const std::string& name = "");
Id addSensor(Document& doc, SensorType type, Id obj, const std::string& name = "");
Id addExclude(Document& doc, Id body1, Id body2);
Id addTexture(Document& doc, const Texture& t);
Id addMaterial(Document& doc, const Material& m);

// Sets type-specific defaults when an actuator changes type (kp/kv/ranges).
void setActuatorType(Document& doc, Actuator& a, ActuatorType type);

// Capsule / cylinder / box helpers: describe the geom by its two axis endpoints (body frame).
void geomEndpoints(const Geom& g, Vec3* a, Vec3* b);
void setGeomFromTo(Geom& g, const Vec3& a, const Vec3& b);

// ----- deletion -----
// Deletes the given elements. Bodies take their subtree with them; actuators, sensors and
// exclusions referencing deleted elements are removed too. The world cannot be deleted.
void deleteElements(Document& doc, const std::vector<Id>& ids);

// ----- hierarchy -----
bool canReparent(const Document& doc, Id body, Id newParent);
bool reparentBody(Document& doc, Id body, Id newParent, bool keepWorldPose = true);
// Moves a geom/joint/site/camera/light to another body.
bool moveToBody(Document& doc, Id element, Id newBody, bool keepWorldPose = true);
// Moves an element one step up (-1) or down (+1) among its siblings of the same kind.
bool moveSibling(Document& doc, Id element, int direction);

// ----- copy -----
// Duplicates elements (bodies with their subtree). Actuators, sensors and exclusions that only
// reference copied elements are duplicated as well. Returns the new top-level IDs.
std::vector<Id> duplicate(Document& doc, const std::vector<Id>& ids, const Vec3& worldOffset);

// Copies of an element (body with subtree, geom, site, ...): each copy is the previous one
// transformed by `step`, a pose in the element's parent frame (translation for linear arrays,
// rotation about the parent origin for radial arrays). Returns the new top-level IDs.
std::vector<Id> arrayDuplicate(Document& doc, Id element, int copies, const Pose& step);

// ----- limbs -----
struct LimbSpec {
  int segments = 2;
  double length = 0.2;           // per segment
  double radius = 0.025;
  double taper = 0.85;           // radius factor per segment
  double range = 1.0471975512;   // hinge range +-, radians (60 deg)
  Vec3 direction{0, 0, -1};      // world direction the limb points to
  Vec3 axis{0, 1, 0};            // world hinge axis (made orthogonal to the direction)
  bool actuators = true;
  ActuatorType actuatorType = ActuatorType::Position;
  double maxTorque = 10;         // actuator strength at the first joint (later joints get less)
  bool foot = true;              // sphere at the tip
  std::string name = "limb";
};
// Attaches a chain of hinged capsule segments to `parent` at a world point. Returns the first
// segment's body.
Id addLimb(Document& doc, Id parent, const Vec3& worldPoint, const LimbSpec& spec);

// ----- start pose (keyframe 0) -----
// The start keyframe; a "home" keyframe is created when the document has none.
Keyframe& startKeyframe(Document& doc);
// Value of a hinge or slide joint in a keyframe (the joint's reference position if omitted).
double keyJointValue(const Keyframe* key, const Joint& joint);
void setKeyJointQpos(Keyframe& key, Id joint, std::vector<double> qpos);
// Position servos on hinge/slide joints aim at the keyframe pose (ctrl = gear * q), so the pose is
// held after mj_resetDataKeyframe.
void syncKeyframeControls(const Document& doc, Keyframe& key);
// Puts a model's hinge, slide and ball joints back to their reference positions in the keyframe.
void resetKeyPose(const Document& doc, Keyframe& key, Id root);
// Copies joint values from one side of a model to the other (left = +y in the model's frame),
// pairing joints by mirrorName. Returns the number of joints set.
int mirrorKeyPose(const Document& doc, Keyframe& key, Id root, bool leftToRight);
// Moves a model's start pose up by dz: its free joint's value in the first keyframe if it has one,
// otherwise the body itself.
void liftStartPose(Document& doc, Id root, double dz);

// ----- placement & collision presets -----
// Moves a top-level body so its lowest colliding geom rests `clearance` above z = floorZ at the
// start pose (the first keyframe if there is one).
void placeOnFloor(Document& doc, Id root, double floorZ = 0.0, double clearance = 0.002);
// Geoms at the bottom of a model (within `tolerance` of its lowest point), e.g. the feet.
std::vector<Id> lowestGeoms(const Document& doc, Id root, double tolerance = -1);
// Collision bitmask presets for a model subtree (MuJoCo: collide if (c1 & a2) || (c2 & a1)).
enum class CollisionPreset {
  AllCollide,       // contype = conaffinity = 1 everywhere (MuJoCo default)
  NoSelfCollision,  // model geoms: contype 2, conaffinity 1 -> touch the world, not each other
  FeetOnly,         // like NoSelfCollision for the feet; all other model geoms collide with nothing
};
// Returns the number of geoms changed.
int applyCollisionPreset(Document& doc, Id root, CollisionPreset preset);

// Names: "left_leg" <-> "right_leg", "l_hip" <-> "r_hip", "hip_L" <-> "hip_R", ...
std::string mirrorName(const std::string& name, bool* changed = nullptr);
// Mirrors a body subtree across the plane through the parent's origin with the given normal
// axis (0 = x, 1 = y, 2 = z) in the parent's frame. Hinge axes are mirrored as pseudo-vectors so
// equal joint angles produce mirror-symmetric motion. Returns the new body ID.
Id mirrorSubtree(Document& doc, Id body, int axis);

}  // namespace mjf
