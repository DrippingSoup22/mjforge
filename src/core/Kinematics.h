// Forward kinematics of the document (reference configuration or a keyframe) and mass properties.
#pragma once

#include <unordered_map>
#include <vector>

#include "core/Document.h"

namespace mjf {

// World anchor and axis of a joint, as MuJoCo computes them: in the body frame moved by the
// body's earlier joints only (a later joint of the same body does not turn an earlier one's axis).
struct JointFrame {
  Vec3 anchor;
  Vec3 axis{0, 0, 1};
};

// World poses of all bodies, indexed like doc.bodies.
std::vector<Pose> computeBodyPoses(const Document& doc);
// ... with the joints at the positions of a keyframe (joints without a value stay at reference).
// `joints` (optional) receives every joint's frame, indexed like doc.joints.
std::vector<Pose> computeBodyPoses(const Document& doc, const Keyframe* key, std::vector<JointFrame>* joints = nullptr);
// World pose of a single body (walks up the parents).
Pose bodyWorldPose(const Document& doc, Id body);
// World pose of any spatial element (body, geom, joint, site, camera, light).
// For joints the orientation maps +Z onto the joint axis.
Pose elementWorldPose(const Document& doc, const std::vector<Pose>& bodyPoses, Id id);
// Local pose of an element relative to its owning body (inverse of the above).
Pose elementLocalPose(const Document& doc, Id id);
void setElementLocalPose(Document& doc, Id id, const Pose& local);

// World-space joint anchor and axis from body poses: exact for the reference configuration and for
// bodies with a single joint. With a keyframe use the frames computeBodyPoses returns.
void jointWorldFrame(const Document& doc, const std::vector<Pose>& bodyPoses, const Joint& j, Vec3* anchor, Vec3* axis);
void jointWorldFrame(const Document& doc, const std::vector<JointFrame>& frames, const Joint& j, Vec3* anchor, Vec3* axis);

// Accumulates point masses / rigid bodies and yields total mass, COM and inertia about the COM.
class MassAccumulator {
 public:
  // Adds a rigid body with mass m, center c and inertia Icom about c (world-aligned axes).
  void add(double m, const Vec3& c, const Mat3& Icom);
  double mass() const { return mass_; }
  Vec3 com() const { return mass_ > 0 ? first_ / mass_ : Vec3{}; }
  Mat3 inertiaAboutCom() const;
  // Inertia about an arbitrary axis through point p (unit axis a).
  double inertiaAboutAxis(const Vec3& p, const Vec3& a) const;

 private:
  double mass_ = 0;
  Vec3 first_;                    // sum m*c
  Mat3 originInertia_ = Mat3::zero();  // inertia about the world origin
};

// Inertia a joint feels when it moves the `moved` part of a free-floating `whole` (which contains
// it) and every other joint holds still: the moved part and the rest push against each other
// (angular = hinge about the axis through p, else slide along it). Exact for rigid bodies.
double floatingJointInertia(const MassAccumulator& moved, const MassAccumulator& whole, const Vec3& p, const Vec3& axis,
                            bool angular);

struct MassProps {
  double mass = 0;
  Vec3 com;
  Mat3 inertia = Mat3::zero();  // about com, world axes
};

// A body's inertial properties in its own frame, as a compiled MuJoCo model has them: they include
// every compiler option (mesh inertia modes, shell inertia, total mass) that the geometry-based
// estimate below leaves out.
struct BodyInertial {
  double mass = 0;
  Vec3 ipos;
  Quat iquat;
  Vec3 diaginertia;
};
using InertialTable = std::unordered_map<Id, BodyInertial>;

// Mass properties of a single body: from `table` when it has the body, else estimated like MuJoCo's
// compiler does (explicit inertial or the geoms, following inertiafromgeom, boundmass and
// boundinertia; settotalmass is not applied).
void accumulateBodyMass(const Document& doc, const std::vector<Pose>& poses, Id body, MassAccumulator& acc,
                        const InertialTable* table = nullptr);
MassProps bodyMassProps(const Document& doc, const std::vector<Pose>& poses, Id body, const InertialTable* table = nullptr);
// Mass properties of the subtree rooted at body.
MassProps subtreeMassProps(const Document& doc, const std::vector<Pose>& poses, Id body, const InertialTable* table = nullptr);
MassAccumulator subtreeAccumulator(const Document& doc, const std::vector<Pose>& poses, Id body,
                                   const InertialTable* table = nullptr);

// Whether a geom takes part in collisions at all (visual-only geoms have contype = conaffinity = 0).
inline bool geomCollides(const Geom& g) { return g.contype != 0 || g.conaffinity != 0; }
// World AABB of a body subtree's geoms (excluding planes/hfields; optionally only colliding geoms).
Aabb subtreeAabb(const Document& doc, const std::vector<Pose>& poses, Id body, bool collidingOnly = false);
// Lowest world z of a subtree's geoms (+inf if none).
double subtreeMinZ(const Document& doc, const std::vector<Pose>& poses, Id body, bool collidingOnly = false);

}  // namespace mjf
