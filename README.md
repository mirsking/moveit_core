# This Repository Has Moved

The new location as of August 5th, 2016 is [https://github.com/ros-planning/moveit](https://github.com/ros-planning/moveit)

See [migration notes](https://github.com/davetcoleman/moveit_merge/blob/master/README.md) for more details.

Please do not open new pull requests or issues in this old location.

## Build Status

 * Devel Job Status: [![Devel Job Status](http://jenkins.ros.org/buildStatus/icon?job=devel-indigo-moveit_core)](http://jenkins.ros.org/job/devel-indigo-moveit_core)
 * AMD64 Debian Job Status: [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-moveit-core_binarydeb_trusty_amd64)](http://jenkins.ros.org/job/ros-indigo-moveit-core_binarydeb_trusty_amd64/)


## Bullet3
### Link to own library

1. Add own library path into /etc/ld.so.conf.d/
2. rm all libmoveit*.so in /opt/ros/indigo/lib/
3. sudo ldconfig

### Where is FCL used
1. in PlanningScene class, in the initialize function
```
setActiveCollisionDetector // set FCL allocator
```
, this function set the allocator into active_collision_, and active_collision_ will allocate the world by collision_world_fcl's third constructor

2. Collision function used:
* checkSelfCollision in collision_robot
* checkRobotCollision in collision_world
