/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, mirsking.com
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Mirs King */

#ifndef MOVEIT_COLLISION_DETECTION_BULLET3_COLLISION_ROBOT_
#define MOVEIT_COLLISION_DETECTION_BULLET3_COLLISION_ROBOT_

#include <moveit/collision_detection_bullet3/collision_common.h>

namespace collision_detection
{

  class CollisionRobotBULLET3 : public CollisionRobot
  {
    friend class CollisionWorldBULLET3;

  public:

    CollisionRobotBULLET3(const robot_model::RobotModelConstPtr &kmodel, double padding = 0.0, double scale = 1.0);

    CollisionRobotBULLET3(const CollisionRobotBULLET3 &other);

    virtual void checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state) const;
    virtual void checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state, const AllowedCollisionMatrix &acm) const;
    virtual void checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2) const;
    virtual void checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2, const AllowedCollisionMatrix &acm) const;

    virtual void checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
                                     const CollisionRobot &other_robot, const robot_state::RobotState &other_state) const;
    virtual void checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
                                     const CollisionRobot &other_robot, const robot_state::RobotState &other_state,
                                     const AllowedCollisionMatrix &acm) const;
    virtual void checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2,
                                     const CollisionRobot &other_robot, const robot_state::RobotState &other_state1, const robot_state::RobotState &other_state2) const;
    virtual void checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2,
                                     const CollisionRobot &other_robot, const robot_state::RobotState &other_state1, const robot_state::RobotState &other_state2,
                                     const AllowedCollisionMatrix &acm) const;

    virtual double distanceSelf(const robot_state::RobotState &state) const;
    virtual double distanceSelf(const robot_state::RobotState &state, const AllowedCollisionMatrix &acm) const;
    virtual double distanceOther(const robot_state::RobotState &state,
                                 const CollisionRobot &other_robot, const robot_state::RobotState &other_state) const;
    virtual double distanceOther(const robot_state::RobotState &state, const CollisionRobot &other_robot,
                                 const robot_state::RobotState &other_state, const AllowedCollisionMatrix &acm) const;

  protected:

    /*
    virtual void updatedPaddingOrScaling(const std::vector<std::string> &links);
    void constructBULLET3Object(const robot_state::RobotState &state, BULLET3Object &bullet3_obj) const;
    void allocSelfCollisionBroadPhase(const robot_state::RobotState &state, BULLET3Manager &manager) const;
    void getAttachedBodyObjects(const robot_state::AttachedBody *ab, std::vector<BULLET3GeometryConstPtr> &geoms) const;
    */

    void checkSelfCollisionHelper(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
                                  const AllowedCollisionMatrix *acm) const;
    void checkOtherCollisionHelper(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
                                   const CollisionRobot &other_robot, const robot_state::RobotState &other_state,
                                   const AllowedCollisionMatrix *acm) const;
    double distanceSelfHelper(const robot_state::RobotState &state, const AllowedCollisionMatrix *acm) const;
    double distanceOtherHelper(const robot_state::RobotState &state, const CollisionRobot &other_robot,
                               const robot_state::RobotState &other_state, const AllowedCollisionMatrix *acm) const;

    void constructBULLET3Object(const robot_state::RobotState &state, BULLET3Objects &bullet3_objs) const;
    std::vector<BULLET3Geometry::ConstPtr> geoms_;

    BULLET3Objects::Ptr b3_objs_ptr_;

    b3GpuCollisionDetectionManager::Ptr manager_;
  };

}

#endif
