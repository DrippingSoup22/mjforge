// Forward kinematics of the document at its reference configuration, and mass properties.
#pragma once

#include <vector>

#include "core/Document.h"

namespace mjf {

// World poses of all bodies, indexed like doc.bodies.
std::vector<Pose> computeBodyPoses(const Document& doc);
// World pose of a single body (walks up the parents).
Pose bodyWorldPose(const Document& doc, Id body);
// World pose of any spatial element (body, geom, joint, site, camera, light).
// For joints the orientation maps +Z onto the joint axis.
Pose elementWorldPose(const Document& doc, const std::vector<Pose>& bodyPoses, Id id);
// Local pose of an element relative to its owning body (inverse of the above).
Pose elementLocalPose(const Document& doc, Id id);
void setElementLocalPose(Document& doc, Id id, const Pose& local);

// World-space joint anchor and axis.
void jointWorldFrame(const Document& doc, const std::vector<Pose>& bodyPoses, const Joint& j, Vec3* anchor, Vec3* axis);

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

struct MassProps {
  double mass = 0;
  Vec3 com;
  Mat3 inertia = Mat3::zero();  // about com, world axes
};

// Mass properties of a single body (explicit inertial or inferred from its geoms).
void accumulateBodyMass(const Document& doc, const std::vector<Pose>& poses, Id body, MassAccumulator& acc);
MassProps bodyMassProps(const Document& doc, const std::vector<Pose>& poses, Id body);
// Mass properties of the subtree rooted at body.
MassProps subtreeMassProps(const Document& doc, const std::vector<Pose>& poses, Id body);
MassAccumulator subtreeAccumulator(const Document& doc, const std::vector<Pose>& poses, Id body);

// World AABB of a body subtree's geoms (excluding planes/hfields).
Aabb subtreeAabb(const Document& doc, const std::vector<Pose>& poses, Id body);
// Lowest world z of a subtree's geoms (+inf if none).
double subtreeMinZ(const Document& doc, const std::vector<Pose>& poses, Id body);

}  // namespace mjf
