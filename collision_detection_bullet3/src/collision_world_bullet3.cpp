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

#include <moveit/collision_detection_bullet3/collision_world_bullet3.h>

collision_detection::CollisionWorldBULLET3::CollisionWorldBULLET3() :
  CollisionWorld()
{
    /*
  bullet3::DynamicAABBTreeCollisionManager* m = new bullet3::DynamicAABBTreeCollisionManager();
  // m->tree_init_level = 2;
  manager_.reset(m);

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldBULLET3::notifyObjectChange, this, _1, _2));
  */
}

collision_detection::CollisionWorldBULLET3::CollisionWorldBULLET3(const WorldPtr& world) :
  CollisionWorld(world)
{
    /*
  bullet3::DynamicAABBTreeCollisionManager* m = new bullet3::DynamicAABBTreeCollisionManager();
  // m->tree_init_level = 2;
  manager_.reset(m);

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldBULLET3::notifyObjectChange, this, _1, _2));
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
  */
}

collision_detection::CollisionWorldBULLET3::CollisionWorldBULLET3(const CollisionWorldBULLET3 &other, const WorldPtr& world) :
  CollisionWorld(other, world)
{
    /*
  logError("*******************=============init bullet3==============********************");
  bullet3::DynamicAABBTreeCollisionManager* m = new bullet3::DynamicAABBTreeCollisionManager();
  // m->tree_init_level = 2;
  manager_.reset(m);

  bullet3_objs_ = other.bullet3_objs_;
  for (std::map<std::string, BULLET3Object>::iterator it = bullet3_objs_.begin() ; it != bullet3_objs_.end() ; ++it)
    it->second.registerTo(manager_.get());
  // manager_->update();

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldBULLET3::notifyObjectChange, this, _1, _2));
  */
}

collision_detection::CollisionWorldBULLET3::~CollisionWorldBULLET3()
{
  getWorld()->removeObserver(observer_handle_);
}

void collision_detection::CollisionWorldBULLET3::checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const robot_state::RobotState &state) const
{
  checkRobotCollisionHelper(req, res, robot, state, NULL);
}

void collision_detection::CollisionWorldBULLET3::checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const robot_state::RobotState &state, const AllowedCollisionMatrix &acm) const
{
  checkRobotCollisionHelper(req, res, robot, state, &acm);
}

void collision_detection::CollisionWorldBULLET3::checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const robot_state::RobotState &state1, const robot_state::RobotState &state2) const
{
  logError("BULLET3 continuous collision checking not yet implemented");
}

void collision_detection::CollisionWorldBULLET3::checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const robot_state::RobotState &state1, const robot_state::RobotState &state2, const AllowedCollisionMatrix &acm) const
{
  logError("BULLET3 continuous collision checking not yet implemented");
}

void collision_detection::CollisionWorldBULLET3::checkRobotCollisionHelper(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const robot_state::RobotState &state, const AllowedCollisionMatrix *acm) const
{
    /*
  const CollisionRobotBULLET3 &robot_bullet3 = dynamic_cast<const CollisionRobotBULLET3&>(robot);
  BULLET3Object bullet3_obj;
  robot_bullet3.constructBULLET3Object(state, bullet3_obj);

  CollisionData cd(&req, &res, acm);
  cd.enableGroup(robot.getRobotModel());
  for (std::size_t i = 0 ; !cd.done_ && i < bullet3_obj.collision_objects_.size() ; ++i)
    manager_->collide(bullet3_obj.collision_objects_[i].get(), &cd, &collisionCallback);

  if (req.distance)
    res.distance = distanceRobotHelper(robot, state, acm);
    */
}

void collision_detection::CollisionWorldBULLET3::checkWorldCollision(const CollisionRequest &req, CollisionResult &res, const CollisionWorld &other_world) const
{
  checkWorldCollisionHelper(req, res, other_world, NULL);
}

void collision_detection::CollisionWorldBULLET3::checkWorldCollision(const CollisionRequest &req, CollisionResult &res, const CollisionWorld &other_world, const AllowedCollisionMatrix &acm) const
{
  checkWorldCollisionHelper(req, res, other_world, &acm);
}

void collision_detection::CollisionWorldBULLET3::checkWorldCollisionHelper(const CollisionRequest &req, CollisionResult &res, const CollisionWorld &other_world, const AllowedCollisionMatrix *acm) const
{
    /*
  const CollisionWorldBULLET3 &other_bullet3_world = dynamic_cast<const CollisionWorldBULLET3&>(other_world);
  CollisionData cd(&req, &res, acm);
  manager_->collide(other_bullet3_world.manager_.get(), &cd, &collisionCallback);

  if (req.distance)
    res.distance = distanceWorldHelper(other_world, acm);
    */

    logError("checkWorldCollisionHelper: in progress");
    res.collision = false;
}
/*
void collision_detection::CollisionWorldBULLET3::constructBULLET3Object(const World::Object *obj, BULLET3Object &bullet3_obj) const
{
  for (std::size_t i = 0 ; i < obj->shapes_.size() ; ++i)
  {
    BULLET3GeometryConstPtr g = createCollisionGeometry(obj->shapes_[i], obj);
    if (g)
    {
      bullet3::CollisionObject *co = new bullet3::CollisionObject(g->collision_geometry_,  transform2bullet3(obj->shape_poses_[i]));
      bullet3_obj.collision_objects_.push_back(boost::shared_ptr<bullet3::CollisionObject>(co));
      bullet3_obj.collision_geometry_.push_back(g);
    }
  }
}
void collision_detection::CollisionWorldBULLET3::updateBULLET3Object(const std::string &id)
{
  // remove BULLET3 objects that correspond to this object
  std::map<std::string, BULLET3Object>::iterator jt = bullet3_objs_.find(id);
  if (jt != bullet3_objs_.end())
  {
    jt->second.unregisterFrom(manager_.get());
    jt->second.clear();
  }

  // check to see if we have this object
  collision_detection::World::const_iterator it = getWorld()->find(id);
  if (it != getWorld()->end())
  {
    // construct BULLET3 objects that correspond to this object
    if (jt != bullet3_objs_.end())
    {
      constructBULLET3Object(it->second.get(), jt->second);
      jt->second.registerTo(manager_.get());
    }
    else
    {
      constructBULLET3Object(it->second.get(), bullet3_objs_[id]);
      bullet3_objs_[id].registerTo(manager_.get());
    }
  }
  else
  {
    if (jt != bullet3_objs_.end())
      bullet3_objs_.erase(jt);
  }

  // manager_->update();
}
*/
void collision_detection::CollisionWorldBULLET3::setWorld(const WorldPtr& world)
{
    /*
  if (world == getWorld())
    return;

  // turn off notifications about old world
  getWorld()->removeObserver(observer_handle_);

  // clear out objects from old world
  manager_->clear();
  bullet3_objs_.clear();
  cleanCollisionGeometryCache();

  CollisionWorld::setWorld(world);

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldBULLET3::notifyObjectChange, this, _1, _2));

  // get notifications any objects already in the new world
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
  */
}

void collision_detection::CollisionWorldBULLET3::notifyObjectChange(const ObjectConstPtr& obj, World::Action action)
{
    /*
  if (action == World::DESTROY)
  {
    std::map<std::string, BULLET3Object>::iterator it = bullet3_objs_.find(obj->id_);
    if (it != bullet3_objs_.end())
    {
      it->second.unregisterFrom(manager_.get());
      it->second.clear();
      bullet3_objs_.erase(it);
    }
    cleanCollisionGeometryCache();
  }
  else
  {
    updateBULLET3Object(obj->id_);
    if (action & (World::DESTROY|World::REMOVE_SHAPE))
      cleanCollisionGeometryCache();
  }
  */
}

double collision_detection::CollisionWorldBULLET3::distanceRobotHelper(const CollisionRobot &robot, const robot_state::RobotState &state, const AllowedCollisionMatrix *acm) const
{
    /*
  const CollisionRobotBULLET3& robot_bullet3 = dynamic_cast<const CollisionRobotBULLET3&>(robot);
  BULLET3Object bullet3_obj;
  robot_bullet3.constructBULLET3Object(state, bullet3_obj);

  CollisionRequest req;
  CollisionResult res;
  CollisionData cd(&req, &res, acm);
  cd.enableGroup(robot.getRobotModel());

  for(std::size_t i = 0; !cd.done_ && i < bullet3_obj.collision_objects_.size(); ++i)
    manager_->distance(bullet3_obj.collision_objects_[i].get(), &cd, &distanceCallback);


  return res.distance;
  */

    logError("distanceRobotHelper: in progress");
    return 0.0;
}

double collision_detection::CollisionWorldBULLET3::distanceRobot(const CollisionRobot &robot, const robot_state::RobotState &state) const
{
  return distanceRobotHelper(robot, state, NULL);
}

double collision_detection::CollisionWorldBULLET3::distanceRobot(const CollisionRobot &robot, const robot_state::RobotState &state, const AllowedCollisionMatrix &acm) const
{
  return distanceRobotHelper(robot, state, &acm);
}

double collision_detection::CollisionWorldBULLET3::distanceWorld(const CollisionWorld &world) const
{
  return distanceWorldHelper(world, NULL);
}

double collision_detection::CollisionWorldBULLET3::distanceWorld(const CollisionWorld &world, const AllowedCollisionMatrix &acm) const
{
  return distanceWorldHelper(world, &acm);
}

double collision_detection::CollisionWorldBULLET3::distanceWorldHelper(const CollisionWorld &other_world, const AllowedCollisionMatrix *acm) const
{
    /*
  const CollisionWorldBULLET3& other_bullet3_world = dynamic_cast<const CollisionWorldBULLET3&>(other_world);
  CollisionRequest req;
  CollisionResult res;
  CollisionData cd(&req, &res, acm);
  manager_->distance(other_bullet3_world.manager_.get(), &cd, &distanceCallback);

  return res.distance;
  */

    logError("distanceWorldHelper: in progress");
    return 0.0;
}

#include <moveit/collision_detection_bullet3/collision_detector_allocator_bullet3.h>
const std::string collision_detection::CollisionDetectorAllocatorBULLET3::NAME_("BULLET3");
