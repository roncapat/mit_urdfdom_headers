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

  std::string name;

  /// child Link element
  ///   child link frame is the same as the Joint frame
  std::string child_link_name;

  /// parent Link element
  ///   origin specifies the transform from Parent Link to Joint Frame
  std::string parent_link_name;

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
  JointConstraint() { this->clear(); };

  /// Gear ratio
  ///   gear_ratio = parent_velocity / child_velocity
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
  LoopConstraint() { this->clear(); };

  // TODO(@MatthewChignoli): Add info about axes and constraint locations later

  void clear()
  {
    Constraint::clear();
  };
};

}

#endif
