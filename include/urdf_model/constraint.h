/* Author: Matt Chignoli */

#ifndef URDF_INTERFACE_CONSTRAINT_H
#define URDF_INTERFACE_CONSTRAINT_H

#include <string>
#include <vector>

#include "urdf_model/pose.h"
#include "urdf_model/types.h"


namespace urdf{

class Constraint
{
public:

  Constraint() { this->clear(); };

  virtual ~Constraint() = default;

  std::string name;
  enum
  {
    UNKNOWN, JOINT, LOOP
  } type;

  /// child Link element
  ///   child link frame is the same as the Joint frame
  std::string child_link_name;

  /// parent Link element
  ///   origin specifies the transform from Parent Link to Joint Frame
  std::string parent_link_name;

  /// nearest common ancestor
  ///   the nearest link that is an ancestor of both the child and parent links
  std::string nearest_common_ancestor_name;

  void clear()
  {
    this->child_link_name.clear();
    this->parent_link_name.clear();
  };
};

// TODO(@MatthewChignoli): How to deal with differentials?
class JointConstraint : public Constraint
{
public:
  JointConstraint()
  {
    this->clear();
    this->type = JOINT;
  };

  /// Gear ratio
  ///   gear_ratio = child_velocity / parent_velocity
  double gear_ratio;

  void clear()
  {
    Constraint::clear();
    this->gear_ratio = 1.0;
  };
};

class LoopConstraint : public Constraint
{
public:
  LoopConstraint()
  {
    this->clear();
    this->type = LOOP;
  };

  /// transform from Parent/Child Link frame to the respective Constraints frames on each link
  Pose parent_to_constraint_origin_transform;
  Pose child_to_constraint_origin_transform;
 
  /// indicate which axis or axes are constrained
  Vector3 position_axis;
  Vector3 rotation_axis;

  void clear()
  {
    Constraint::clear();
    this->parent_to_constraint_origin_transform.clear();
    this->child_to_constraint_origin_transform.clear();
    this->position_axis.clear();
    this->rotation_axis.clear();
  };
};

}

#endif
