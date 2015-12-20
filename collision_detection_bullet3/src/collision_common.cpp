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
#include <Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h>
#include <Bullet3OpenCL/RigidBody/b3GpuNarrowPhaseInternalData.h>

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
      for (std::map<boost::weak_ptr<const shapes::Shape>, BULLET3Geometry::ConstPtr>::iterator it = map_.begin() ; it != map_.end() ; )
      {
        std::map<boost::weak_ptr<const shapes::Shape>, BULLET3Geometry::ConstPtr>::iterator nit = it; ++nit;
        if (it->first.expired())
          map_.erase(it);
        it = nit;
      }
      //      logDebug("Cleaning up cache for BULLET3 objects that correspond to static shapes. Cache size reduced from %u to %u", from, (unsigned int)map_.size());
    }
  }

  static const unsigned int MAX_CLEAN_COUNT = 10000; // every this many uses of the cache, a cleaning operation is executed (this is only removal of expired entries)
  std::map<boost::weak_ptr<const shapes::Shape>, BULLET3Geometry::ConstPtr> map_;
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

void cleanCollisionGeometryCache()
{
  BULLET3ShapeCache &cache1 = GetShapeCache<b3Aabb, World::Object>();
  {
    boost::mutex::scoped_lock slock(cache1.lock_);
    cache1.bumpUseCount(true);
  }
  BULLET3ShapeCache &cache2 = GetShapeCache<b3Aabb, robot_state::AttachedBody>();
  {
    boost::mutex::scoped_lock slock(cache2.lock_);
    cache2.bumpUseCount(true);
  }
}


template<typename BV, typename T>
BULLET3Geometry::ConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape, const T *data, int shape_index,
                                                b3GpuCollisionDetectionManager::ConstPtr manager)
{
  BULLET3ShapeCache &cache = GetShapeCache<BV, T>();

  boost::weak_ptr<const shapes::Shape> wptr(shape);
  {
    boost::mutex::scoped_lock slock(cache.lock_);
    std::map<boost::weak_ptr<const shapes::Shape>, BULLET3Geometry::ConstPtr>::const_iterator cache_it = cache.map_.find(wptr);
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
    std::map<boost::weak_ptr<const shapes::Shape>, BULLET3Geometry::ConstPtr>::iterator cache_it = othercache.map_.find(wptr);
    if (cache_it != othercache.map_.end())
    {
      if (cache_it->second.unique())
      {
        // remove from old cache
        BULLET3Geometry::ConstPtr obj_cache = cache_it->second;
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
      std::map<boost::weak_ptr<const shapes::Shape>, BULLET3Geometry::ConstPtr>::iterator cache_it = othercache.map_.find(wptr);
      if (cache_it != othercache.map_.end())
      {
        if (cache_it->second.unique())
        {
          // remove from old cache
          BULLET3Geometry::ConstPtr obj_cache = cache_it->second;
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
        logError("This shape type (%d: PLANE) is not supported using BULLET3 yet", (int)shape->type);
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
        //logWarn("loading shape type (%d: SPHERE)", (int)shape->type);
        cg_g = manager->m_data->m_narrowphaseGPU->registerSphereShape(s->radius);
      }
      break;
    case shapes::BOX:
      {
        const shapes::Box* s = static_cast<const shapes::Box*>(shape.get());
        const double* size = s->size;
        //logWarn("loading shape type (%d: BOX)", (int)shape->type);
        b3::ConvexHullShape ch;
        b3::createBoxConvexHullVerticles(size, ch);
        cg_g = manager->m_data->m_narrowphaseGPU->registerConvexHullShape(ch.vertices, ch.strideInBytes, ch.numVertices, &ch.scales[0]);
        delete ch.vertices;
      }
      break;
    case shapes::CYLINDER:
      {
        const shapes::Cylinder* s = static_cast<const shapes::Cylinder*>(shape.get());
        //logWarn("loading shape type (%d: CYLINDER)", (int)shape->type);
        b3::ConvexHullShape ch;
        b3::createCylinderConvexHullVerticles(s->radius, s->length, ch);
        cg_g = manager->m_data->m_narrowphaseGPU->registerConvexHullShape(ch.vertices, ch.strideInBytes, ch.numVertices, &ch.scales[0]);
        delete ch.vertices;
      }
      break;
    case shapes::CONE:
      {
        const shapes::Cone* s = static_cast<const shapes::Cone*>(shape.get());
        //cg_g = new bullet3::Cone(s->radius, s->length);
        //TODO: new bullet3 rigid
        logError("This shape type (%d: CONE) is not supported using BULLET3 yet", (int)shape->type);
      }
      break;
    case shapes::MESH:
      {
        //logWarn("loading shape type (%d: MESH)", (int)shape->type);
        b3AlignedObjectArray<b3Vector3> b3_mesh_vertices;
        b3AlignedObjectArray<int> b3_mesh_indices;
        const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(shape.get());
        if(mesh->vertex_count>0 && mesh->triangle_count>0)
        {
            for(int i=0;i<mesh->vertex_count; i++)
            {
                b3Vector3 vt = b3MakeVector3(mesh->vertices[3*i],
                        mesh->vertices[3*i+1],
                        mesh->vertices[3*i+2]);
                b3_mesh_vertices.push_back(vt);
                b3_mesh_indices.push_back(i);
            }
            float scale[] = {1, 1, 1};
            cg_g = manager->m_data->m_narrowphaseGPU->registerConcaveMesh(&b3_mesh_vertices, &b3_mesh_indices, scale);
        }
      }
      break;
    case shapes::OCTREE:
      {
        const shapes::OcTree* g = static_cast<const shapes::OcTree*>(shape.get());
        //cg_g = new bullet3::OcTree(g->octree);
        //TODO: new bullet3 rigid
        logError("This shape type (%d: OCTREE) is not supported using BULLET3 yet", (int)shape->type);
      }
      break;
    default:
      logError("This shape type (%d) is not supported using BULLET3 yet", (int)shape->type);
      cg_g = -1;
    }
  }
  if (cg_g!=-1)
  {
    BULLET3Geometry::ConstPtr res(new BULLET3Geometry(cg_g, data, shape_index));
    boost::mutex::scoped_lock slock(cache.lock_);
    cache.map_[wptr] = res;
    cache.bumpUseCount();
    return res;
  }
  return BULLET3Geometry::ConstPtr();
}


/////////////////////////////////////////////////////
BULLET3Geometry::ConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape,
                                            const robot_model::LinkModel *link,
                                            int shape_index,
                                                b3GpuCollisionDetectionManager::ConstPtr manager)
{
  return createCollisionGeometry<b3Aabb, robot_model::LinkModel>(shape, link, shape_index, manager);//b3Aabb is just a placeholder
}
BULLET3Geometry::ConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape,
                                            const robot_state::AttachedBody *ab,
                                            int shape_index,
                                                b3GpuCollisionDetectionManager::ConstPtr manager)
{
  return createCollisionGeometry<b3Aabb, robot_state::AttachedBody>(shape, ab, shape_index, manager);
}

BULLET3Geometry::ConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape,
                                            const World::Object *obj,
                                                b3GpuCollisionDetectionManager::ConstPtr manager)
{
  return createCollisionGeometry<b3Aabb, World::Object>(shape, obj, 0, manager);
}

template<typename BV, typename T>
BULLET3Geometry::ConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape, double scale, double padding, const T *data, int shape_index,
                                                b3GpuCollisionDetectionManager::ConstPtr manager)
{
  if (std::fabs(scale - 1.0) <= std::numeric_limits<double>::epsilon() && std::fabs(padding) <= std::numeric_limits<double>::epsilon())
    return createCollisionGeometry<BV, T>(shape, data, shape_index, manager);
  else
  {
    boost::shared_ptr<shapes::Shape> scaled_shape(shape->clone());
    scaled_shape->scaleAndPadd(scale, padding);
    return createCollisionGeometry<BV, T>(scaled_shape, data, shape_index, manager);
  }
}

BULLET3Geometry::ConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape, double scale, double padding,
                                                    const robot_model::LinkModel *link, int shape_index,
                                                b3GpuCollisionDetectionManager::ConstPtr manager)
{
  return createCollisionGeometry<b3Aabb, robot_model::LinkModel>(shape, scale, padding, link, shape_index, manager);
}

BULLET3Geometry::ConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape, double scale, double padding,
                                            const robot_state::AttachedBody *ab, int shape_index,
                                                b3GpuCollisionDetectionManager::ConstPtr manager)
{
  return createCollisionGeometry<b3Aabb, robot_state::AttachedBody>(shape, scale, padding, ab, shape_index, manager);
}

BULLET3Geometry::ConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape, double scale, double padding,
                                            const World::Object *obj,
                                                b3GpuCollisionDetectionManager::ConstPtr manager)
{
  return createCollisionGeometry<b3Aabb, World::Object>(shape, scale, padding, obj, 0, manager);
}

void BULLET3Objects::convert2CollisionResult(const AllowedCollisionMatrix *acm, const CollisionRequest& req, CollisionResult& res)
{
    typedef std::pair<std::string, std::string> ContactIndex_t;
    typedef std::map< std::pair<b3::CollisionObject, b3::CollisionObject>, ContactIndex_t > Contacts2IndexMap_t;
    Contacts2IndexMap_t contacts2index_map_;

    if(num_contacts_<=0)
    {
        res.collision = false;
        res.contact_count = 0;
        return;
    }
    else
    {

        res.contacts.clear();
        int N = num_contacts_;
        for(int i=0; i<N; i++)
        {
            b3::CollisionObject id1 = contacts_[i].getBodyA();
            b3::CollisionObject id2 = contacts_[i].getBodyB();
            /*
            if(id1 > id2)
            {
                //swap id1 and id2, ensure id1 is smaller than id2
                b3::CollisionObject tmp = id1;
                id1 = id2;
                id2 = tmp;
            }
            */

            CollisionGeometryData::ConstPtr cd1 = getGeometry(id1);
            CollisionGeometryData::ConstPtr cd2 = getGeometry(id2);
            if(! cd1.get() || !cd2.get() )
            {
                continue;
            }

            logDebug("(%d) Collision between '%s' (type '%s') and '%s' (type '%s') is going to be checked",
                                num_contacts_,
                               cd1->getID().c_str(),
                               cd1->getTypeString().c_str(),
                               cd2->getID().c_str(),
                               cd2->getTypeString().c_str());
            // use the collision matrix (if any) to avoid certain collision checks
            DecideContactFn dcf;
            bool always_allow_collision = false;
            if (acm)
            {
                AllowedCollision::Type type;
                bool found = acm->getAllowedCollision(cd1->getID(), cd2->getID(), type);
                if (found)
                {
                    // if we have an entry in the collision matrix, we read it
                    if (type == AllowedCollision::ALWAYS)
                    {
                        always_allow_collision = true;
                        if (req.verbose)
                          logDebug("Collision between '%s' (type '%s') and '%s' (type '%s') is always allowed. No contacts are computed.",
                                   cd1->getID().c_str(),
                                   cd1->getTypeString().c_str(),
                                   cd2->getID().c_str(),
                                   cd2->getTypeString().c_str());
                    }
                    else if (type == AllowedCollision::CONDITIONAL)
                    {
                        acm->getAllowedCollision(cd1->getID(), cd2->getID(), dcf);
                        if (req.verbose)
                            logDebug("Collision between '%s' and '%s' is conditionally allowed", cd1->getID().c_str(), cd2->getID().c_str());
                    }
                }
                else
                {
                    if (req.verbose)
                      logDebug("Collision between '%s' (type '%s') and '%s' (type '%s') is stored.",
                               cd1->getID().c_str(),
                               cd1->getTypeString().c_str(),
                               cd2->getID().c_str(),
                               cd2->getTypeString().c_str());

               }


                if(always_allow_collision)
                    num_contacts_--;
                else
                {

                    if (req.verbose)
                      logError("Handling contacts between '%s' (type '%s') and '%s' (type '%s').",
                               cd1->getID().c_str(),
                               cd1->getTypeString().c_str(),
                               cd2->getID().c_str(),
                               cd2->getTypeString().c_str());

                    if(req.contacts && res.contacts.size() < req.max_contacts)
                    {
                        Contacts2IndexMap_t::iterator iter = contacts2index_map_.find(std::make_pair(id1, id2));
                        if(iter==contacts2index_map_.end())
                        {
                            std::vector<Contact> cts(1);
                            Contact& ct = cts[0];
                            ct.body_name_1 = cd1->getID();
                            ct.body_name_2 = cd2->getID();
                            ContactIndex_t contact_index = std::make_pair(cd1->getID(), cd2->getID());
                            res.contacts[contact_index] = cts;
                            contacts2index_map_[std::make_pair(id1, id2)] = contact_index;
                        }
                        else
                        {
                            std::vector<Contact>& cts = res.contacts[iter->second];
                            if(cts.size()<req.max_contacts_per_pair)
                            {
                                Contact ct;
                                ct.body_name_1 = cd1->getID();
                                ct.body_name_2 = cd2->getID();
                                cts.push_back(ct);
                            }
                        }

                    }
                }
            }
        }
        res.contact_count = std::min(num_contacts_, (int)res.contacts.size());
        logError("Finally, get %d contacts", num_contacts_);
        if(num_contacts_>0)
            res.collision = true;
        else
            res.collision = false;
    }
}


}

namespace b3 {

void createBoxConvexHullVerticles(const double* size, ConvexHullShape& ch)
{
    ch.strideInBytes = 3*sizeof(float);
    ch.numVertices = 8;
    ch.vertices = new float[ch.numVertices * 3];
    for(int i=0;i<3;i++)
        ch.scales[i] = 1.0;
    double x = size[0]/2, y=size[1]/2, z=size[2]/2;
    int index = 0;
    for(int i=-1;i<2;i+=2)
    {
        for(int j=-1;j<2;j+=2)
        {
            for(int k=-1;k<2;k+=2)
            {
                ch.vertices[index++] = x*i;
                ch.vertices[index++] = y*j;
                ch.vertices[index++] = z*k;
            }
        }
    }
}

void inline insertVertice(float* vertices, int& index, float x, float y, float z)
{
    vertices[index++] = x;
    vertices[index++] = y;
    vertices[index++] = z;
}

void createCylinderConvexHullVerticles(const double radius, const double length, ConvexHullShape& ch)
{
    ch.strideInBytes = 3*sizeof(float);
    ch.numVertices = 8*3;
    ch.vertices = new float[ch.numVertices*3];
    for(int i=0;i<3;i++)
        ch.scales[i] = 1.0;
    int index = 0;
    for(int k=-1;k<2;k++)// z=-length/2, 0, + length/2;
    {
        double z = -length/2*k;
        double x = radius, y = 0;
        insertVertice(ch.vertices, index, x, y, z);
        insertVertice(ch.vertices, index, y, x, z);
        insertVertice(ch.vertices, index, -x, y, z);
        insertVertice(ch.vertices, index, -y, x, z);
        x = radius*0.7071067812;
        insertVertice(ch.vertices, index, x, x, z);
        insertVertice(ch.vertices, index, -x, x, z);
        insertVertice(ch.vertices, index, -x, -x, z);
        insertVertice(ch.vertices, index, x, -x, z);
    }
}
}
