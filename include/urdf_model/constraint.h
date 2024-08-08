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
    UNKNOWN, LOOP, COUPLING
  } class_type;

  /// successor Link element
  ///   successor link frame is the same as the Loop frame
  std::string successor_link_name;

  /// predecessor Link element
  ///   origin specifies the transform from Predecessor Link to Loop Frame
  std::string predecessor_link_name;

  /// nearest common ancestor
  ///   the nearest link that is an ancestor of both the child and parent links
  std::string nearest_common_ancestor_name;

  void clear()
  {
    this->successor_link_name.clear();
    this->predecessor_link_name.clear();
    this->nearest_common_ancestor_name.clear();
    this->class_type = UNKNOWN;
  };
};

class CouplingConstraint : public Constraint
{
public:
  CouplingConstraint()
  {
    this->clear();
    this->class_type = COUPLING;
  };

  /// Ratio
  ///   ratio = child_velocity / parent_velocity
  double ratio;

  void clear()
  {
    Constraint::clear();
    this->ratio = 1.0;
  };
};

class LoopConstraint : public Constraint
{
public:
  LoopConstraint()
  {
    this->clear();
    this->class_type = LOOP;
  };

  enum
  {
    UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, PLANAR, FIXED
  } type;

  /// \brief     type_       meaning of axis_
  /// ------------------------------------------------------
  ///            UNKNOWN     unknown type
  ///            REVOLUTE    rotation axis
  ///            PRISMATIC   translation axis
  ///            PLANAR      plane normal axis
  ///            FIXED       N/A
  Vector3 axis;

  /// transform from Successor Link frame to Loop frame
  Pose  successor_to_joint_origin_transform;

  /// transform from Predecessor Link frame to Loop frame
  Pose  predecessor_to_joint_origin_transform;

  void clear()
  {
    Constraint::clear();
    this->axis.clear();
    this->predecessor_to_joint_origin_transform.clear();
    this->successor_to_joint_origin_transform.clear();
    this->type = UNKNOWN;
  };
};

}

#endif
