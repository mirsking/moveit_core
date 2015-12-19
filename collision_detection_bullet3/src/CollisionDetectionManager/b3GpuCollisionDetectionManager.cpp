/*
Copyright (c) 2015 mirsking.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Mirs King

#include <moveit/collision_detection_bullet3/CollisionDetectionManager/b3GpuCollisionDetectionManager.h>
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuSapBroadphase.h"
#include "Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h"
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3RigidBodyData.h"

//gpu
#include "Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h"
#include "Bullet3OpenCL/RigidBody/kernels/integrateKernel.h"
#include "Bullet3OpenCL/RigidBody/kernels/updateAabbsKernel.h"
#define B3_RIGIDBODY_INTEGRATE_PATH "src/Bullet3OpenCL/RigidBody/kernels/integrateKernel.cl"
#define B3_RIGIDBODY_UPDATEAABB_PATH "src/Bullet3OpenCL/RigidBody/kernels/updateAabbsKernel.cl"

//cpu
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3UpdateAabbs.h"
#include "Bullet3Geometry/b3AabbUtil.h"

#include "Bullet3Common/b3Logging.h"
#include <stdio.h>

bool gUseCPU = false;

b3GpuCollisionDetectionManager::b3GpuCollisionDetectionManager(const b3Config& config)
{
    m_data = new b3GpuCollisionDetectionInternalData;
    m_data->m_config = config;
    initCL();

    //    m_data->m_allAabbsGPU = new b3OpenCLArray<b3SapAabb>(m_data->m_context, m_data->m_queue, config.m_maxConvexBodies);
    //    m_data->m_overlappingPairsGPU = new b3OpenCLArray<b3BroadphasePair>(m_data->m_context, m_data->m_queue, config.m_maxBroadphasePairs);

    m_data->m_broadphase = new b3GpuSapBroadphase(m_data->m_context, m_data->m_device, m_data->m_queue, b3GpuSapBroadphase::B3_GPU_SAP_KERNEL_LOCAL_SHARED_MEMORY);
    m_data->m_narrowphaseGPU = new b3GpuNarrowPhase(m_data->m_context, m_data->m_device, m_data->m_queue, m_data->m_config);
    m_data->m_narrowphaseCPU = new b3CpuNarrowPhase(m_data->m_config);

    cl_int errNum=0;

    {
        cl_program prog = b3OpenCLUtils::compileCLProgramFromString(m_data->m_context,m_data->m_device,integrateKernelCL,&errNum,"",B3_RIGIDBODY_INTEGRATE_PATH);
        b3Assert(errNum==CL_SUCCESS);
        m_data->m_integrateTransformsKernel = b3OpenCLUtils::compileCLKernelFromString(m_data->m_context, m_data->m_device,integrateKernelCL, "integrateTransformsKernel",&errNum,prog);
        b3Assert(errNum==CL_SUCCESS);
        clReleaseProgram(prog);
    }
    {
        cl_program prog = b3OpenCLUtils::compileCLProgramFromString(m_data->m_context,m_data->m_device,updateAabbsKernelCL,&errNum,"",B3_RIGIDBODY_UPDATEAABB_PATH);
        b3Assert(errNum==CL_SUCCESS);
        m_data->m_updateAabbsKernel = b3OpenCLUtils::compileCLKernelFromString(m_data->m_context, m_data->m_device,updateAabbsKernelCL, "initializeGpuAabbsFull",&errNum,prog);
        b3Assert(errNum==CL_SUCCESS);


        m_data->m_clearOverlappingPairsKernel = b3OpenCLUtils::compileCLKernelFromString(m_data->m_context, m_data->m_device,updateAabbsKernelCL, "clearOverlappingPairsKernel",&errNum,prog);
        b3Assert(errNum==CL_SUCCESS);

        clReleaseProgram(prog);
    }
}

b3GpuCollisionDetectionManager::~b3GpuCollisionDetectionManager()
{
    if (m_data->m_integrateTransformsKernel)
        clReleaseKernel(m_data->m_integrateTransformsKernel);

    if (m_data->m_updateAabbsKernel)
        clReleaseKernel(m_data->m_updateAabbsKernel);

    if (m_data->m_clearOverlappingPairsKernel)
        clReleaseKernel(m_data->m_clearOverlappingPairsKernel);

    //delete m_data->m_allAabbsGPU;
    //delete m_data->m_overlappingPairsGPU;
    delete m_data->m_broadphase;
    delete m_data->m_narrowphaseCPU;
    delete m_data->m_narrowphaseGPU;

    exitCL();
    delete m_data;
}

void myfunc(const char*msg)
{
    printf ("fuck:%s\n", msg);
}

bool b3GpuCollisionDetectionManager::calculateCollision(int& numContacts, const b3Contact4* * contacts)
{
    //::b3SetCustomEnterProfileZoneFunc(myfunc);
    initBroadPhase();
    int numPairs = 0;
    if(gUseCPU)
    {
        m_data->m_broadphase->calculateOverlappingPairsHost(m_data->m_config.m_maxBroadphasePairs);
        numPairs = m_data->m_broadphase->getNumOverlap();
        b3Printf("get %d pairs collision by broadphase calculate CPU\n", numPairs);
    }
    else
    {
        m_data->m_broadphase->calculateOverlappingPairs(m_data->m_config.m_maxBroadphasePairs);
        numPairs = m_data->m_broadphase->getNumOverlap();
        b3Printf("get %d pairs collision by broadphase calculate GPU\n", numPairs);
        if(numPairs)
        {
            m_data->m_narrowphaseGPU->writeAllBodiesToGpu();
            m_data->m_narrowphaseGPU->computeContacts(
                        m_data->m_broadphase->getOverlappingPairBuffer(),
                        numPairs,
                        m_data->m_broadphase->getAllAabbsGPU().getBufferCL(),
                        m_data->m_narrowphaseGPU->getNumRigidBodies());

            numContacts = m_data->m_narrowphaseGPU->getNumContactsGpu();
            b3Printf("get %d contacts by narrowphase calculate\n", numContacts);

            m_data->m_narrowphaseGPU->readbackAllBodiesToCpu();
            *contacts = m_data->m_narrowphaseGPU->getContactsCPU();
            if(numContacts > 0)
                return true;
            else
                return false;
        }
        else
        {
            numContacts = 0;
            return false;
        }

    }
}

void b3GpuCollisionDetectionManager::initBroadPhase()
{
    int numBodies = m_data->m_narrowphaseGPU->getNumRigidBodies();
    if (!numBodies)
    {
        b3Warning("no rigid body in narrowphase");
        return;
    }

    //if(gUseCPU)//cpu
    if(1)
    {
        m_data->m_broadphase->getAllAabbsCPU().resize(numBodies);
        for (int i=0;i<numBodies;i++)
        {
            b3ComputeWorldAabb(  i,
                                 m_data->m_narrowphaseGPU->getBodiesCpu(),
                                 m_data->m_narrowphaseGPU->getCollidablesCpu(),
                                 m_data->m_narrowphaseGPU->getLocalSpaceAabbsCpu(),
                                 &m_data->m_broadphase->getAllAabbsCPU()[0]);
        }
        m_data->m_broadphase->writeAabbsToGpu();
    }
    else// Use GPU to calculate World Aabb
    {
        //__kernel void initializeGpuAabbsFull(  const int numNodes, __global Body* gBodies,__global Collidable* collidables, __global b3AABBCL* plocalShapeAABB, __global b3AABBCL* pAABB)
        b3LauncherCL launcher(m_data->m_queue,m_data->m_updateAabbsKernel,"m_updateAabbsKernel");
        launcher.setConst(numBodies);
        cl_mem bodies = m_data->m_narrowphaseGPU->getBodiesGpu();
        launcher.setBuffer(bodies);
        cl_mem collidables = m_data->m_narrowphaseGPU->getCollidablesGpu();
        launcher.setBuffer(collidables);
        cl_mem localAabbs = m_data->m_narrowphaseGPU->getAabbLocalSpaceBufferGpu();
        launcher.setBuffer(localAabbs);

        cl_mem worldAabbs = worldAabbs = m_data->m_broadphase->getAabbBufferWS();
        launcher.setBuffer(worldAabbs);
        launcher.launch1D(numBodies);
        //m_data->m_broadphase->writeAabbsToGpu();
    }
}

void inline coutb3Vector(b3Vector3& v)
{
    b3Error("(%f %f %f)\n", v.getX(), v.getY(), v.getZ());
}

int b3GpuCollisionDetectionManager::registerPhysicsInstance(float mass, const float* position, const float* orientation, int collidableIndex, int userIndex, bool writeInstanceToGpu)
{

    b3Vector3 aabbMin=b3MakeVector3(0,0,0),aabbMax=b3MakeVector3(0,0,0);


    if (collidableIndex>=0)
    {
        b3SapAabb localAabb = m_data->m_narrowphaseGPU->getLocalSpaceAabb(collidableIndex);
        b3Vector3 localAabbMin=b3MakeVector3(localAabb.m_min[0],localAabb.m_min[1],localAabb.m_min[2]);
        b3Vector3 localAabbMax=b3MakeVector3(localAabb.m_max[0],localAabb.m_max[1],localAabb.m_max[2]);

        b3Scalar margin = 0.01f;
        b3Transform t;
        t.setIdentity();
        t.setOrigin(b3MakeVector3(position[0],position[1],position[2]));
        t.setRotation(b3Quaternion(orientation[0],orientation[1],orientation[2],orientation[3]));
        b3TransformAabb(localAabbMin,localAabbMax, margin,t,aabbMin,aabbMax);
    } else
    {
        b3Error("registerPhysicsInstance using invalid collidableIndex\n");
        return -1;
    }


    //bool writeToGpu = false;
    int bodyIndex = m_data->m_narrowphaseGPU->getNumRigidBodies();
    bodyIndex = m_data->m_narrowphaseGPU->registerRigidBody(collidableIndex,mass,position,orientation,&aabbMin.getX(),&aabbMax.getX(), writeInstanceToGpu);

    if (bodyIndex>=0)
    {
        /*
        if (gUseDbvt)
        {
            m_data->m_broadphaseDbvt->createProxy(aabbMin,aabbMax,bodyIndex,0,1,1);
            b3SapAabb aabb;
            for (int i=0;i<3;i++)
            {
                aabb.m_min[i] = aabbMin[i];
                aabb.m_max[i] = aabbMax[i];
                aabb.m_minIndices[3] = bodyIndex;
            }
            m_data->m_allAabbsCPU.push_back(aabb);
            if (writeInstanceToGpu)
            {
                m_data->m_allAabbsGPU->copyFromHost(m_data->m_allAabbsCPU);
            }
        } else
        */
        {
            if (mass)
            {
                m_data->m_broadphase->createProxy(aabbMin,aabbMax,bodyIndex,1,1);//m_dispatcher);
            } else
            {
                m_data->m_broadphase->createLargeProxy(aabbMin,aabbMax,bodyIndex,1,1);//m_dispatcher);
            }
        }
    }

    /*
    if (mass>0.f)
        m_numDynamicPhysicsInstances++;

    m_numPhysicsInstances++;
    */

    return bodyIndex;
}
