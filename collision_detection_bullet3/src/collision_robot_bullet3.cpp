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

#include <moveit/collision_detection_bullet3/collision_robot_bullet3.h>
#include <Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h>

collision_detection::CollisionRobotBULLET3::CollisionRobotBULLET3(const robot_model::RobotModelConstPtr &model, double padding, double scale)
  : CollisionRobot(model, padding, scale)
{
  b3Config config;
  manager_ = b3GpuCollisionDetectionManager::Ptr(new b3GpuCollisionDetectionManager(config));

  const std::vector<const robot_model::LinkModel*>& links = robot_model_->getLinkModelsWithCollisionGeometry();
  geoms_.resize(robot_model_->getLinkGeometryCount());
  // we keep the same order of objects as what RobotState *::getLinkState() returns
  for (std::size_t i = 0 ; i < links.size() ; ++i)
    for (std::size_t j = 0 ; j < links[i]->getShapes().size() ; ++j)
    {
      BULLET3GeometryConstPtr g = createCollisionGeometry(links[i]->getShapes()[j], getLinkScale(links[i]->getName()), getLinkPadding(links[i]->getName()), links[i], j, manager_);
      if (g)
        geoms_[links[i]->getFirstCollisionBodyTransformIndex() + j] = g;
      else
        logError("Unable to construct collision geometry for link '%s'", links[i]->getName().c_str());
    }
}

collision_detection::CollisionRobotBULLET3::CollisionRobotBULLET3(const CollisionRobotBULLET3 &other) : CollisionRobot(other)
{
  geoms_ = other.geoms_;
  manager_ = other.manager_;
}
/*
void collision_detection::CollisionRobotBULLET3::getAttachedBodyObjects(const robot_state::AttachedBody *ab, std::vector<BULLET3GeometryConstPtr> &geoms) const
{
  const std::vector<shapes::ShapeConstPtr> &shapes = ab->getShapes();
  for (std::size_t i = 0 ; i < shapes.size() ; ++i)
  {
    BULLET3GeometryConstPtr co = createCollisionGeometry(shapes[i], ab, i);
    if (co)
      geoms.push_back(co);
  }
}

void collision_detection::CollisionRobotBULLET3::constructBULLET3Object(const robot_state::RobotState &state, BULLET3Object &bullet3_obj) const
{
  bullet3_obj.collision_objects_.reserve(geoms_.size());
  
  for (std::size_t i = 0 ; i < geoms_.size() ; ++i)
    if (geoms_[i] && geoms_[i]->collision_geometry_)
    {
      bullet3::CollisionObject *collObj = new bullet3::CollisionObject
        (geoms_[i]->collision_geometry_, transform2bullet3(state.getCollisionBodyTransform(geoms_[i]->collision_geometry_data_->ptr.link, geoms_[i]->collision_geometry_data_->shape_index)));
      bullet3_obj.collision_objects_.push_back(boost::shared_ptr<bullet3::CollisionObject>(collObj));
      // the CollisionGeometryData is already stored in the class member geoms_, so we need not copy it
    }
  
  std::vector<const robot_state::AttachedBody*> ab;
  state.getAttachedBodies(ab);
  for (std::size_t j = 0 ; j < ab.size() ; ++j)
  {
    std::vector<BULLET3GeometryConstPtr> objs;
    getAttachedBodyObjects(ab[j], objs);
    const EigenSTL::vector_Affine3d &ab_t = ab[j]->getGlobalCollisionBodyTransforms();
    for (std::size_t k = 0 ; k < objs.size() ; ++k)
      if (objs[k]->collision_geometry_)
      {
        bullet3::CollisionObject *collObj = new bullet3::CollisionObject(objs[k]->collision_geometry_, transform2bullet3(ab_t[k]));
        bullet3_obj.collision_objects_.push_back(boost::shared_ptr<bullet3::CollisionObject>(collObj));
        // we copy the shared ptr to the CollisionGeometryData, as this is not stored by the class itself,
        // and would be destroyed when objs goes out of scope.
        bullet3_obj.collision_geometry_.push_back(objs[k]);
      }
  }
}

void collision_detection::CollisionRobotBULLET3::allocSelfCollisionBroadPhase(const robot_state::RobotState &state, BULLET3Manager &manager) const
{
  bullet3::DynamicAABBTreeCollisionManager* m = new bullet3::DynamicAABBTreeCollisionManager();
  // m->tree_init_level = 2;
  manager.manager_.reset(m);
  constructBULLET3Object(state, manager.object_);
  manager.object_.registerTo(manager.manager_.get());
  // manager.manager_->update();
}
*/

void collision_detection::CollisionRobotBULLET3::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state) const
{
  checkSelfCollisionHelper(req, res, state, NULL);
}

void collision_detection::CollisionRobotBULLET3::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
                                                                const AllowedCollisionMatrix &acm) const
{
  checkSelfCollisionHelper(req, res, state, &acm);
}

void collision_detection::CollisionRobotBULLET3::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2) const
{
  logError("BULLET3 continuous collision checking not yet implemented");
}

void collision_detection::CollisionRobotBULLET3::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2, const AllowedCollisionMatrix &acm) const
{
  logError("BULLET3 continuous collision checking not yet implemented");
}

void collision_detection::CollisionRobotBULLET3::checkSelfCollisionHelper(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
                                                                      const AllowedCollisionMatrix *acm) const
{
  if (req.distance)
  {
    res.distance = 0.0;
    logError("distance require in collision detection is not supported yet!");
  }
  BULLET3Objects b3_objs;
  constructBULLET3Object(state, b3_objs);

  res.collision = manager_->calculateCollision();

  logError("checkSelfCollisionHelper result: %d\n", res.collision);
}


void collision_detection::CollisionRobotBULLET3::constructBULLET3Object(const robot_state::RobotState &state, BULLET3Objects &bullet3_objs) const
{
  bullet3_objs.collision_objects_.reserve(geoms_.size());

  for (std::size_t i = 0 ; i < geoms_.size() ; ++i)
    if (geoms_[i] && geoms_[i]->collision_geometry_id_ != -1)
    {
      float position[3], orientation[4];
      transform2bullet3(state.getCollisionBodyTransform(geoms_[i]->collision_geometry_data_->ptr.link, geoms_[i]->collision_geometry_data_->shape_index), position, orientation);

      //get Aabb
      b3SapAabb aabb = manager_->m_data->m_narrowphase->getLocalSpaceAabb(geoms_[i]->collision_geometry_id_);

      b3::CollisionObject collObj =  manager_->m_data->m_narrowphase->registerRigidBody(
        geoms_[i]->collision_geometry_id_,
                  0.0,// mass
                  position,
                  orientation,
                  &aabb.m_min[0],
                  &aabb.m_max[0],
                  true);
      bullet3_objs.collision_objects_.push_back(collObj);
      // the CollisionGeometryData is already stored in the class member geoms_, so we need not copy it
    }
/*
  std::vector<const robot_state::AttachedBody*> ab;
  state.getAttachedBodies(ab);
  for (std::size_t j = 0 ; j < ab.size() ; ++j)
  {
    std::vector<BULLET3GeometryConstPtr> objs;
    getAttachedBodyObjects(ab[j], objs);
    const EigenSTL::vector_Affine3d &ab_t = ab[j]->getGlobalCollisionBodyTransforms();
    for (std::size_t k = 0 ; k < objs.size() ; ++k)
      if (objs[k]->collision_geometry_)
      {
        bullet3::CollisionObject *collObj = new bullet3::CollisionObject(objs[k]->collision_geometry_, transform2bullet3(ab_t[k]));
        bullet3_obj.collision_objects_.push_back(boost::shared_ptr<bullet3::CollisionObject>(collObj));
        // we copy the shared ptr to the CollisionGeometryData, as this is not stored by the class itself,
        // and would be destroyed when objs goes out of scope.
        bullet3_obj.collision_geometry_.push_back(objs[k]);
      }
  }
*/
}

void collision_detection::CollisionRobotBULLET3::checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
                                                                 const CollisionRobot &other_robot, const robot_state::RobotState &other_state) const
{
  checkOtherCollisionHelper(req, res, state, other_robot, other_state, NULL);
}

void collision_detection::CollisionRobotBULLET3::checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
                                                                 const CollisionRobot &other_robot, const robot_state::RobotState &other_state,
                                                                 const AllowedCollisionMatrix &acm) const
{
  checkOtherCollisionHelper(req, res, state, other_robot, other_state, &acm);
}

void collision_detection::CollisionRobotBULLET3::checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2,
                                                                 const CollisionRobot &other_robot, const robot_state::RobotState &other_state1, const robot_state::RobotState &other_state2) const
{
  logError("BULLET3 continuous collision checking not yet implemented");
}

void collision_detection::CollisionRobotBULLET3::checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2,
                                                                 const CollisionRobot &other_robot, const robot_state::RobotState &other_state1, const robot_state::RobotState &other_state2,
                                                                 const AllowedCollisionMatrix &acm) const
{
  logError("BULLET3 continuous collision checking not yet implemented");
}

void collision_detection::CollisionRobotBULLET3::checkOtherCollisionHelper(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
                                                                       const CollisionRobot &other_robot, const robot_state::RobotState &other_state,
                                                                       const AllowedCollisionMatrix *acm) const
{
    /*
  BULLET3Manager manager;
  allocSelfCollisionBroadPhase(state, manager);

  const CollisionRobotBULLET3 &bullet3_rob = dynamic_cast<const CollisionRobotBULLET3&>(other_robot);
  BULLET3Object other_bullet3_obj;
  bullet3_rob.constructBULLET3Object(other_state, other_bullet3_obj);

  CollisionData cd(&req, &res, acm);
  cd.enableGroup(getRobotModel());
  for (std::size_t i = 0 ; !cd.done_ && i < other_bullet3_obj.collision_objects_.size() ; ++i)
    manager.manager_->collide(other_bullet3_obj.collision_objects_[i].get(), &cd, &collisionCallback);
  if (req.distance)
    res.distance = distanceOtherHelper(state, other_robot, other_state, acm);
    */
    logError("checkOtherCollisionHelper: in progress");
    res.collision = false;
}
/*
void collision_detection::CollisionRobotBULLET3::updatedPaddingOrScaling(const std::vector<std::string> &links)
{
  for (std::size_t i = 0 ; i < links.size() ; ++i)
  {
    const robot_model::LinkModel *lmodel = robot_model_->getLinkModel(links[i]);
    if (lmodel)
    {
      for (std::size_t j = 0 ; j < lmodel->getShapes().size() ; ++j)
      {
        BULLET3GeometryConstPtr g = createCollisionGeometry(lmodel->getShapes()[j], getLinkScale(lmodel->getName()), getLinkPadding(lmodel->getName()), lmodel, j);
        if (g)
          geoms_[lmodel->getFirstCollisionBodyTransformIndex() + j] = g;
      }
    }
    else
      logError("Updating padding or scaling for unknown link: '%s'", links[i].c_str());
  }
}
*/

double collision_detection::CollisionRobotBULLET3::distanceSelf(const robot_state::RobotState &state) const
{
  return distanceSelfHelper(state, NULL);
}

double collision_detection::CollisionRobotBULLET3::distanceSelf(const robot_state::RobotState &state,
                                                            const AllowedCollisionMatrix &acm) const
{
  return distanceSelfHelper(state, &acm);
}

double collision_detection::CollisionRobotBULLET3::distanceSelfHelper(const robot_state::RobotState &state,
                                                                  const AllowedCollisionMatrix *acm) const
{
    /*
  BULLET3Manager manager;
  allocSelfCollisionBroadPhase(state, manager);

  CollisionRequest req;
  CollisionResult res;
  CollisionData cd(&req, &res, acm);
  cd.enableGroup(getRobotModel());

  manager.manager_->distance(&cd, &distanceCallback);

  return res.distance;
  */
    logError("DistanceSelfHelper: in progress");
    return 0.0;
}

double collision_detection::CollisionRobotBULLET3::distanceOther(const robot_state::RobotState &state,
                                                             const CollisionRobot &other_robot,
                                                             const robot_state::RobotState &other_state) const
{
  return distanceOtherHelper(state, other_robot, other_state, NULL);
}

double collision_detection::CollisionRobotBULLET3::distanceOther(const robot_state::RobotState &state,
                                                             const CollisionRobot &other_robot,
                                                             const robot_state::RobotState &other_state,
                                                             const AllowedCollisionMatrix &acm) const
{
  return distanceOtherHelper(state, other_robot, other_state, &acm);
}

double collision_detection::CollisionRobotBULLET3::distanceOtherHelper(const robot_state::RobotState &state,
                                                                   const CollisionRobot &other_robot,
                                                                   const robot_state::RobotState &other_state,
                                                                   const AllowedCollisionMatrix *acm) const
{
    /*
  BULLET3Manager manager;
  allocSelfCollisionBroadPhase(state, manager);

  const CollisionRobotBULLET3& bullet3_rob = dynamic_cast<const CollisionRobotBULLET3&>(other_robot);
  BULLET3Object other_bullet3_obj;
  bullet3_rob.constructBULLET3Object(other_state, other_bullet3_obj);

  CollisionRequest req;
  CollisionResult res;
  CollisionData cd(&req, &res, acm);
  cd.enableGroup(getRobotModel());
  for(std::size_t i = 0; !cd.done_ && i < other_bullet3_obj.collision_objects_.size(); ++i)
    manager.manager_->distance(other_bullet3_obj.collision_objects_[i].get(), &cd, &distanceCallback);

  return res.distance;
  */
    logError("DistanceOtherHelper: in progress");
    return 0.0;
}
