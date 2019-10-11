//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

// ****************************************************************************
// This snippet illustrates simple use of physx
//
// It creates a number of box stacks on a plane, and if rendering, allows the
// user to create new stacks and fire a ball from the camera position
// ****************************************************************************

#include <ctype.h>

#include "PxPhysicsAPI.h"
#include "SkUtils.h"
#include "SnippetPrint.h"
#include "SnippetPVD.h"
#include "SnippetUtils.h"

using namespace physx;

PxDefaultAllocator gAllocator;
SkErrorCallback gErrorCallback;

PxFoundation *gFoundation = NULL;
PxPhysics *gPhysics = NULL;
PxCooking *gCooking = NULL;
PxTolerancesScale gScale;
PxDefaultCpuDispatcher *gDispatcher = NULL;
PxScene *gScene = NULL;

PxMaterial *gMaterial = NULL;

PxPvd *gPvd = NULL;

PxReal stackZ = 10.0f;
float rand(float loVal, float hiVal)
{
	return loVal + (float(rand()) / RAND_MAX) * (hiVal - loVal);
}

PxRigidDynamic *createDynamic(const PxTransform &t, const PxGeometry &geometry, const PxVec3 &velocity = PxVec3(0))
{
	PxRigidDynamic *dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
	return dynamic;
}

PxConvexMesh *createConvexMesh()
{
	const PxU32 numVerts = 64;
	PxVec3 *vertices = new PxVec3[numVerts];

	for (PxU32 i = 0; i < numVerts; i++)
	{
		vertices[i] = PxVec3(rand(-2.0f, 2.0f), rand(-2.0f, 2.0f), rand(-2.0f, 2.0f));
	}

	PxConvexMeshDesc desc;
	desc.points.data = vertices;
	desc.points.count = numVerts;
	desc.points.stride = sizeof(PxVec3);
	desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;
	PxConvexMesh *mesh = gCooking->createConvexMesh(desc, gPhysics->getPhysicsInsertionCallback());
	PX_ASSERT(mesh);

	delete[] vertices;
	return mesh;
}
PxJoint* CreateJoint(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1)
{
	PxSphericalJoint* j = PxSphericalJointCreate(*gPhysics, a0, t0, a1, t1);
	j->setLimitCone(PxJointLimitCone(PxPi/4, PxPi/4, 0.05f));
	j->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED, true);
	j->setConstraintFlags(PxConstraintFlag::eVISUALIZATION);
	j->setProjectionLinearTolerance(0.1f);
	j->setConstraintFlag(PxConstraintFlag::ePROJECTION,true);

	return j;
}
void createStack(const PxTransform &t, PxU32 size, PxReal halfExtent)
{
	PxShape *shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);
	PxConvexMesh *mesh = createConvexMesh();
	PxConvexMeshGeometry conGeo{mesh};
	PxShape *conShape = gPhysics->createShape(conGeo, *gMaterial);
	PxRigidDynamic *prev=NULL;
	PxTransform prePos;
	PxTransform topPos=t.transform(PxTransform(PxVec3( - 1.25f * PxReal(1), PxReal((size-1) * 2 + 5), 0) * halfExtent));
	for (PxU32 i = size - 1; i >= 0 && i < size; i--)
	{
		for (PxU32 j = 0; j < size - i; j++)
		{
			PxTransform localTm(PxVec3(PxReal(j * 2.5) - 1.25f * PxReal(size - i), PxReal(i * 2 + 5), 0) * halfExtent);
			PxRigidDynamic *body = gPhysics->createRigidDynamic(t.transform(localTm));
			body->attachShape(*conShape);
			PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
			if(prev)
			{
				PxVec3 t=0.5*(localTm.p-prePos.p);
				CreateJoint(prev,PxTransform(t),body,PxTransform(-t));
			}
			// else
			{
				CreateJoint(NULL,topPos,body,PxTransform((topPos.p-localTm.p)));
			}
			
			prev=body;
			prePos=localTm;
			gScene->addActor(*body);
		}
		prev=NULL;
	}
	shape->release();
}

void initPhysics(bool interactive)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport *transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);
	gScale = gPhysics->getTolerancesScale();
	if (!gPhysics)
		Error("PxCreatePhysics failed!");

	PxCookingParams params{gScale};
	params.midphaseDesc.setToDefault(PxMeshMidPhase::eBVH33);
	params.meshPreprocessParams = PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH;
	params.convexMeshCookingType = PxConvexMeshCookingType::eQUICKHULL;
	params.gaussMapLimit = 32;

	gCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, params);
	if (!gCooking)
		Error("PxCreateCooking failed!");

	PxSceneDesc sceneDesc(gScale);
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(sceneDesc);
	gScene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS,1.0f);
	gScene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES,1.0f);

	PxPvdSceneClient *pvdClient = gScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.2f);

	PxRigidStatic *groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	gScene->addActor(*groundPlane);

	for (PxU32 i = 0; i < 1; i++)
		createStack(PxTransform(PxVec3(0, 0, stackZ -= 10.0f)), 10, 2.0f);

	if (!interactive)
		createDynamic(PxTransform(PxVec3(0, 40, 100)), PxSphereGeometry(10), PxVec3(0, -50, -100));
}

void stepPhysics(bool /*interactive*/)
{
	gScene->simulate(1.0f / 30.0f);
	gScene->fetchResults(true);
}

void cleanupPhysics(bool /*interactive*/)
{
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	if (gPvd)
	{
		PxPvdTransport *transport = gPvd->getTransport();
		gPvd->release();
		gPvd = NULL;
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);

	printf("CleanUp done.\n");
}

void keyPress(unsigned char key, const PxTransform &camera)
{
	switch (toupper(key))
	{
	case 'B':
		createStack(PxTransform(PxVec3(0, 0, stackZ -= 10.0f)), 10, 2.0f);
		break;
	case ' ':
		createDynamic(camera, PxSphereGeometry(3.0f), camera.rotate(PxVec3(0, 0, -1)) * 200);
		break;
	}
}

int snippetMain(int, const char *const *)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for (PxU32 i = 0; i < frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}
