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

#include <moveit/collision_detection_bullet3/collision_common.h>
#include <boost/thread/mutex.hpp>

namespace collision_detection
{

struct BULLET3ShapeCache
{
  BULLET3ShapeCache() : clean_count_(0) {}

  void bumpUseCount(bool force = false)
  {
    clean_count_++;

    // clean-up for cache (we don't want to keep infinitely large number of weak ptrs stored)
    if (clean_count_ > MAX_CLEAN_COUNT || force)
    {
      clean_count_ = 0;
      for (std::map<boost::weak_ptr<const shapes::Shape>, BULLET3GeometryConstPtr>::iterator it = map_.begin() ; it != map_.end() ; )
      {
        std::map<boost::weak_ptr<const shapes::Shape>, BULLET3GeometryConstPtr>::iterator nit = it; ++nit;
        if (it->first.expired())
          map_.erase(it);
        it = nit;
      }
      //      logDebug("Cleaning up cache for BULLET3 objects that correspond to static shapes. Cache size reduced from %u to %u", from, (unsigned int)map_.size());
    }
  }

  static const unsigned int MAX_CLEAN_COUNT = 100; // every this many uses of the cache, a cleaning operation is executed (this is only removal of expired entries)
  std::map<boost::weak_ptr<const shapes::Shape>, BULLET3GeometryConstPtr> map_;
  unsigned int clean_count_;
  boost::mutex lock_;
};


/* We template the function so we get a different cache for each of the template arguments combinations */
template<typename BV, typename T>
BULLET3ShapeCache& GetShapeCache()
{
  static BULLET3ShapeCache cache;
  return cache;
}

template<typename T1, typename T2>
struct IfSameType
{
  enum { value = 0 };
};

template<typename T>
struct IfSameType<T, T>
{
  enum { value = 1 };
};

template<typename BV, typename T>
BULLET3GeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape, const T *data, int shape_index)
{
  BULLET3ShapeCache &cache = GetShapeCache<BV, T>();

  boost::weak_ptr<const shapes::Shape> wptr(shape);
  {
    boost::mutex::scoped_lock slock(cache.lock_);
    std::map<boost::weak_ptr<const shapes::Shape>, BULLET3GeometryConstPtr>::const_iterator cache_it = cache.map_.find(wptr);
    if (cache_it != cache.map_.end())
    {
      if (cache_it->second->collision_geometry_data_->ptr.raw == (void*)data)
      {
        //        logDebug("Collision data structures for object %s retrieved from cache.", cache_it->second->collision_geometry_data_->getID().c_str());
        return cache_it->second;
      }
      else
        if (cache_it->second.unique())
        {
          const_cast<BULLET3Geometry*>(cache_it->second.get())->updateCollisionGeometryData(data, shape_index, false);
          //          logDebug("Collision data structures for object %s retrieved from cache after updating the source object.", cache_it->second->collision_geometry_data_->getID().c_str());
          return cache_it->second;
        }
    }
  }

  // attached objects could have previously been World::Object; we try to move them
  // from their old cache to the new one, if possible. the code is not pretty, but should help
  // when we attach/detach objects that are in the world
  if (IfSameType<T, robot_state::AttachedBody>::value == 1)
  {
    // get the cache that corresponds to objects; maybe this attached object used to be a world object
    BULLET3ShapeCache &othercache = GetShapeCache<BV, World::Object>();

    // attached bodies could be just moved from the environment.
    othercache.lock_.lock(); // lock manually to avoid having 2 simultaneous locks active (avoids possible deadlock)
    std::map<boost::weak_ptr<const shapes::Shape>, BULLET3GeometryConstPtr>::iterator cache_it = othercache.map_.find(wptr);
    if (cache_it != othercache.map_.end())
    {
      if (cache_it->second.unique())
      {
        // remove from old cache
        BULLET3GeometryConstPtr obj_cache = cache_it->second;
        othercache.map_.erase(cache_it);
        othercache.lock_.unlock();

        // update the CollisionGeometryData; nobody has a pointer to this, so we can safely modify it
        const_cast<BULLET3Geometry*>(obj_cache.get())->updateCollisionGeometryData(data, shape_index, true);

        //        logDebug("Collision data structures for attached body %s retrieved from the cache for world objects.", obj_cache->collision_geometry_data_->getID().c_str());

        // add to the new cache
        boost::mutex::scoped_lock slock(cache.lock_);
        cache.map_[wptr] = obj_cache;
        cache.bumpUseCount();
        return obj_cache;
      }
    }
    othercache.lock_.unlock();
  }
  else
    // world objects could have previously been attached objects; we try to move them
    // from their old cache to the new one, if possible. the code is not pretty, but should help
    // when we attach/detach objects that are in the world
    if (IfSameType<T, World::Object>::value == 1)
    {
      // get the cache that corresponds to objects; maybe this attached object used to be a world object
      BULLET3ShapeCache &othercache = GetShapeCache<BV, robot_state::AttachedBody>();

      // attached bodies could be just moved from the environment.
      othercache.lock_.lock(); // lock manually to avoid having 2 simultaneous locks active (avoids possible deadlock)
      std::map<boost::weak_ptr<const shapes::Shape>, BULLET3GeometryConstPtr>::iterator cache_it = othercache.map_.find(wptr);
      if (cache_it != othercache.map_.end())
      {
        if (cache_it->second.unique())
        {
          // remove from old cache
          BULLET3GeometryConstPtr obj_cache = cache_it->second;
          othercache.map_.erase(cache_it);
          othercache.lock_.unlock();

          // update the CollisionGeometryData; nobody has a pointer to this, so we can safely modify it
          const_cast<BULLET3Geometry*>(obj_cache.get())->updateCollisionGeometryData(data, shape_index, true);

          //          logDebug("Collision data structures for world object %s retrieved from the cache for attached bodies.",
          //                   obj_cache->collision_geometry_data_->getID().c_str());

          // add to the new cache
          boost::mutex::scoped_lock slock(cache.lock_);
          cache.map_[wptr] = obj_cache;
          cache.bumpUseCount();
          return obj_cache;
        }
      }
      othercache.lock_.unlock();
    }

  b3::CollisionGeometry cg_g = -1;
  if (shape->type == shapes::PLANE) // shapes that directly produce CollisionGeometry
  {
    // handle cases individually
    switch (shape->type)
    {
    case shapes::PLANE:
      {
        const shapes::Plane* p = static_cast<const shapes::Plane*>(shape.get());
        //cg_g = new bullet3::Plane(p->a, p->b, p->c, p->d);
        //TODO: new bullet3 rigid
      }
      break;
    default:
      break;
    }
  }
  else
  {
    switch (shape->type)
    {
    case shapes::SPHERE:
      {
        const shapes::Sphere* s = static_cast<const shapes::Sphere*>(shape.get());
        //cg_g = new bullet3::Sphere(s->radius);
        //TODO: new bullet3 rigid
      }
      break;
    case shapes::BOX:
      {
        const shapes::Box* s = static_cast<const shapes::Box*>(shape.get());
        const double* size = s->size;
        //cg_g = new bullet3::Box(size[0], size[1], size[2]);
        //TODO: new bullet3 rigid
      }
      break;
    case shapes::CYLINDER:
      {
        const shapes::Cylinder* s = static_cast<const shapes::Cylinder*>(shape.get());
        //cg_g = new bullet3::Cylinder(s->radius, s->length);
        //TODO: new bullet3 rigid
      }
      break;
    case shapes::CONE:
      {
        const shapes::Cone* s = static_cast<const shapes::Cone*>(shape.get());
        //cg_g = new bullet3::Cone(s->radius, s->length);
        //TODO: new bullet3 rigid
      }
      break;
    case shapes::MESH:
      {
        /*
        bullet3::BVHModel<BV>* g = new bullet3::BVHModel<BV>();
        const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(shape.get());
        if (mesh->vertex_count > 0 && mesh->triangle_count > 0)
        {
          std::vector<bullet3::Triangle> tri_indices(mesh->triangle_count);
          for(unsigned int i = 0; i < mesh->triangle_count; ++i)
            tri_indices[i] = bullet3::Triangle(mesh->triangles[3 * i], mesh->triangles[3 * i + 1], mesh->triangles[3 * i + 2]);

          std::vector<bullet3::Vec3f> points(mesh->vertex_count);
          for (unsigned int i = 0; i < mesh->vertex_count; ++i)
            points[i] = bullet3::Vec3f(mesh->vertices[3 * i], mesh->vertices[3 * i + 1], mesh->vertices[3 * i + 2]);

          g->beginModel();
          g->addSubModel(points, tri_indices);
          g->endModel();
        }
        cg_g = g;
        */
        //TODO: new bullet3 rigid
      }
      break;
    case shapes::OCTREE:
      {
        const shapes::OcTree* g = static_cast<const shapes::OcTree*>(shape.get());
        //cg_g = new bullet3::OcTree(g->octree);
        //TODO: new bullet3 rigid
      }
      break;
    default:
      logError("This shape type (%d) is not supported using BULLET3 yet", (int)shape->type);
      cg_g = -1;
    }
  }
  if (cg_g!=-1)
  {
    //cg_g->computeLocalAABB();
    BULLET3GeometryConstPtr res(new BULLET3Geometry(cg_g, data, shape_index));
    boost::mutex::scoped_lock slock(cache.lock_);
    cache.map_[wptr] = res;
    cache.bumpUseCount();
    return res;
  }
  return BULLET3GeometryConstPtr();
}


/////////////////////////////////////////////////////
/*
BULLET3GeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape,
                                            const robot_model::LinkModel *link,
                                            int shape_index)
{
  return createCollisionGeometry<bullet3::OBBRSS, robot_model::LinkModel>(shape, link, shape_index);
}

BULLET3GeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape,
                                            const robot_state::AttachedBody *ab,
                                            int shape_index)
{
  return createCollisionGeometry<bullet3::OBBRSS, robot_state::AttachedBody>(shape, ab, shape_index);
}

BULLET3GeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape,
                                            const World::Object *obj)
{
  return createCollisionGeometry<bullet3::OBBRSS, World::Object>(shape, obj, 0);
}
*/
/*
template<typename BV, typename T>
BULLET3GeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape, double scale, double padding, const T *data, int shape_index)
{
  if (fabs(scale - 1.0) <= std::numeric_limits<double>::epsilon() && fabs(padding) <= std::numeric_limits<double>::epsilon())
    return createCollisionGeometry<BV, T>(shape, data, shape_index);
  else
  {
    boost::shared_ptr<shapes::Shape> scaled_shape(shape->clone());
    scaled_shape->scaleAndPadd(scale, padding);
    return createCollisionGeometry<BV, T>(scaled_shape, data, shape_index);
  }
}

BULLET3GeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape, double scale, double padding,
                                            const robot_model::LinkModel *link, int shape_index)
{
  return createCollisionGeometry<bullet3::OBBRSS, robot_model::LinkModel>(shape, scale, padding, link, shape_index);
}

BULLET3GeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape, double scale, double padding,
                                            const robot_state::AttachedBody *ab, int shape_index)
{
  return createCollisionGeometry<bullet3::OBBRSS, robot_state::AttachedBody>(shape, scale, padding, ab, shape_index);
}

BULLET3GeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape, double scale, double padding,
                                            const World::Object *obj)
{
  return createCollisionGeometry<bullet3::OBBRSS, World::Object>(shape, scale, padding, obj, 0);
}
*/

/*
void cleanCollisionGeometryCache()
{
  BULLET3ShapeCache &cache1 = GetShapeCache<bullet3::OBBRSS, World::Object>();
  {
    boost::mutex::scoped_lock slock(cache1.lock_);
    cache1.bumpUseCount(true);
  }
  BULLET3ShapeCache &cache2 = GetShapeCache<bullet3::OBBRSS, robot_state::AttachedBody>();
  {
    boost::mutex::scoped_lock slock(cache2.lock_);
    cache2.bumpUseCount(true);
  }
}
*/

}
