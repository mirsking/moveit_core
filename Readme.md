## Link to own library

1. Add own library path into /etc/ld.so.conf.d/
2. rm all libmoveit*.so in /opt/ros/indigo/lib/
3. sudo ldconfig

## Where is FCL used
1. in PlanningScene class, in the initialize function
```
setActiveCollisionDetector // set FCL allocator
```
, this function set the allocator into active_collision_, and active_collision_ will allocate the world by collision_world_fcl's third constructor
