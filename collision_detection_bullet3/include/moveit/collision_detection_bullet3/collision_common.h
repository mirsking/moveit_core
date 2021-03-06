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

#ifndef MOVEIT_COLLISION_DETECTION_BULLET3_COLLISION_COMMON_
#define MOVEIT_COLLISION_DETECTION_BULLET3_COLLISION_COMMON_

#include <moveit/collision_detection/world.h>
#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_detection_bullet3/CollisionDetectionManager/b3GpuCollisionDetectionManager.h>
#include <set>
#include <algorithm>
#include <map>

namespace b3{
typedef int CollisionGeometry; // For bullet3, this type means shape id in narrowphase
typedef int CollisionObject;

struct ConvexHullShape
{
    float* vertices;
    int strideInBytes;
    int numVertices;
    float scales[3];
};
void inline insertVertice(float* vertices, int& index, float x, float y, float z);
void createBoxConvexHullVerticles(const double* size, ConvexHullShape& ch);
void createCylinderConvexHullVerticles(const double radius, const double length, ConvexHullShape& ch);

inline void transform2bullet3(const Eigen::Affine3d &b, float* position, float* orientation)
{
    Eigen::Quaterniond q(b.rotation());
    position[0] = b.translation().x();
    position[1] = b.translation().y();
    position[2] = b.translation().z();
    orientation[0] = q.x();
    orientation[1] = q.y();
    orientation[2] = q.z();
    orientation[3] = q.w();
}

}


namespace collision_detection
{
/**
 * @brief The CollisionGeometryData struct.
 * Store the a pointer to robot link or attached body or world object.
 * Also has an shape_index to get the corresponding shape of bullet3
 */
struct CollisionGeometryData
{
    typedef boost::shared_ptr<CollisionGeometryData> Ptr;
    typedef boost::shared_ptr<const CollisionGeometryData> ConstPtr;
    CollisionGeometryData(const robot_model::LinkModel *link, int index)
        : type(BodyTypes::ROBOT_LINK)
        , shape_index(index)
    {
        ptr.link = link;
    }

    CollisionGeometryData(const robot_state::AttachedBody *ab, int index)
        : type(BodyTypes::ROBOT_ATTACHED)
        , shape_index(index)
    {
        ptr.ab = ab;
    }

    CollisionGeometryData(const World::Object *obj, int index)
        : type(BodyTypes::WORLD_OBJECT)
        , shape_index(index)
    {
        ptr.obj = obj;
    }

    const std::string& getID() const
    {
        switch (type)
        {
        case BodyTypes::ROBOT_LINK:
            return ptr.link->getName();
        case BodyTypes::ROBOT_ATTACHED:
            return ptr.ab->getName();
        default:
            break;
        }
        return ptr.obj->id_;
    }

    std::string getTypeString() const
    {
        switch (type)
        {
        case BodyTypes::ROBOT_LINK:
            return "Robot link";
        case BodyTypes::ROBOT_ATTACHED:
            return "Robot attached";
        default:
            break;
        }
        return "Object";
    }

    /** \brief Check if two CollisionGeometryData objects point to the same source object */
    bool sameObject(const CollisionGeometryData &other) const
    {
        return type == other.type && ptr.raw == other.ptr.raw;
    }

    BodyType type;
    int shape_index;
    union
    {
        const robot_model::LinkModel    *link;
        const robot_state::AttachedBody *ab;
        const World::Object             *obj;
        const void                      *raw;
    } ptr;
};


struct BULLET3Geometry
{
    BULLET3Geometry()
    {
    }
    // collision_geometry maps to bullet3's rigid body id
    BULLET3Geometry(b3::CollisionGeometry collision_geometry_id, const robot_model::LinkModel *link, int shape_index) :
        collision_geometry_id_(collision_geometry_id), collision_geometry_data_(new CollisionGeometryData(link, shape_index))
    {
    }

    BULLET3Geometry(b3::CollisionGeometry collision_geometry_id, const robot_state::AttachedBody *ab, int shape_index) :
        collision_geometry_id_(collision_geometry_id), collision_geometry_data_(new CollisionGeometryData(ab, shape_index))
    {
    }

    BULLET3Geometry(b3::CollisionGeometry collision_geometry_id, const World::Object *obj, int shape_index) :
        collision_geometry_id_(collision_geometry_id), collision_geometry_data_(new CollisionGeometryData(obj, shape_index))
    {
    }

    template<typename T>
    void updateCollisionGeometryData(const T* data, int shape_index, bool newType)
    {
        if (!newType && collision_geometry_data_)
            if (collision_geometry_data_->ptr.raw == reinterpret_cast<const void*>(data))
                return;
        collision_geometry_data_.reset(new CollisionGeometryData(data, shape_index));
    }

    b3::CollisionGeometry collision_geometry_id_; // bullet3 shape index
    CollisionGeometryData::Ptr  collision_geometry_data_;

    typedef boost::shared_ptr<BULLET3Geometry> Ptr;
    typedef boost::shared_ptr<const BULLET3Geometry> ConstPtr;

};

BULLET3Geometry::ConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape,
                                                const robot_model::LinkModel *link,
                                                int shape_index,
                                                b3GpuCollisionDetectionManager::ConstPtr manager);
BULLET3Geometry::ConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape,
                                                const robot_state::AttachedBody *ab,
                                                int shape_index,
                                                b3GpuCollisionDetectionManager::ConstPtr manager);
BULLET3Geometry::ConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape,
                                                const World::Object *obj,
                                                b3GpuCollisionDetectionManager::ConstPtr manager);

BULLET3Geometry::ConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape, double scale, double padding,
                                                const robot_model::LinkModel *link, int shape_index,
                                                b3GpuCollisionDetectionManager::ConstPtr manager);
BULLET3Geometry::ConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape, double scale, double padding,
                                                const robot_state::AttachedBody *ab, int shape_index,
                                                b3GpuCollisionDetectionManager::ConstPtr manager);
BULLET3Geometry::ConstPtr createCollisionGeometry(const shapes::ShapeConstPtr &shape, double scale, double padding,
                                                const World::Object *obj,
                                                b3GpuCollisionDetectionManager::ConstPtr manager);

struct BULLET3Objects
{
    typedef boost::shared_ptr<BULLET3Objects> Ptr;
    typedef boost::shared_ptr<const BULLET3Objects> ConstPtr;

    typedef std::map<b3::CollisionObject, BULLET3Geometry::ConstPtr> Object2GeometryMap_t;
    typedef std::map<BULLET3Geometry::ConstPtr, b3::CollisionObject> Geometry2ObjectMap_t;
    Object2GeometryMap_t object2geometryptr_map_;
    Geometry2ObjectMap_t geometryptr2object_map_;
    int num_contacts_;
    const b3Contact4*  contacts_;

    void clear()
    {
        object2geometryptr_map_.clear();
        num_contacts_ = 0;
        contacts_ = NULL;
    }

    CollisionGeometryData::ConstPtr getGeometry(b3::CollisionObject id)
    {
        Object2GeometryMap_t::iterator   iter = object2geometryptr_map_.find(id);
        if(iter==object2geometryptr_map_.end())
        {
            logError("No Geometry Found for ID %d with map size %d!", id, object2geometryptr_map_.size());
            return CollisionGeometryData::ConstPtr();
        }

        return iter->second->collision_geometry_data_;
    }

    void convert2CollisionResult(const AllowedCollisionMatrix *acm, const CollisionRequest& req, CollisionResult& res);
};

}

#endif
