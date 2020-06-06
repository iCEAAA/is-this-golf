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
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

// ****************************************************************************
// This snippet illustrates simple use of physx
//
// It creates a number of box stacks on a plane, and if rendering, allows the
// user to create new stacks and fire a ball from the camera position
// ****************************************************************************

#include <ctype.h>
#include <Windows.h>

#include "PxPhysicsAPI.h"

#include "../SnippetCommon/SnippetPrint.h"
#include "../SnippetUtils/SnippetUtils.h"

#ifndef PHYSX_SNIPPET_PVD_H
#define PHYSX_SNIPPET_PVD_H

#define PVD_HOST "127.0.0.1"	//Set this to the IP address of the system running the PhysX Visual Debugger that you want to connect to.

#endif //PHYSX_SNIPPET_PVD_H


using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation*			gFoundation = NULL;
PxPhysics*				gPhysics = NULL;

PxDefaultCpuDispatcher*	gDispatcher = NULL;
PxScene*				gScene = NULL;

PxMaterial*				gMaterial = NULL;

PxPvd*                  gPvd = NULL;

PxReal stackZ = 10.0f;/*��ʼ�ѵ�λ��*/

PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity = PxVec3(0))
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 100.0f);/*��physx��λ�ã���״�����ʣ��ܶ�/������*/
	dynamic->setAngularDamping(0.5f);/*�Ƕ����ᣬԽ������Խ�Ѽ������нǶȱ仯*/
	dynamic->setLinearDamping(0.4f);/*�������ᣬ�������Ϊ������ٵĿ��������������Ĵ�С����*/
	dynamic->setLinearVelocity(velocity);/*�����ٶ�*/
	gScene->addActor(*dynamic);/* actor+1 */
	return dynamic;
}

PxRigidDynamic* Golf = nullptr;//��golf����Ϊȫ�ַ���
PxRigidStatic* Arrow = nullptr;//����ָʾ��ͷ


							   //���¼�ͷ���̵߳��õĺ���
DWORD WINAPI changeArrow(LPVOID lpParamter)
{
	while (!Golf->isSleeping());//����ûͣ��ʱ��loop
								//��ͣ��֮�󣬰�һ���µļ�ͷ
	PxTransform newPos = Golf->getGlobalPose().transform(PxTransform(0, 0, 5.0f));
	Arrow = gPhysics->createRigidStatic(newPos);//����ָʾ��
	PxTransform arrowPose(PxQuat(PxHalfPi, PxVec3(0, 1.0f, 0)));
	PxShape* arrowShape = PxRigidActorExt::createExclusiveShape(*Arrow, PxCapsuleGeometry(0.5f, 1.5f), *gMaterial);
	arrowShape->setLocalPose(arrowPose);
	gScene->addActor(*Arrow);
	return 0;
}

void hit()
{
	gScene->removeActor(*Arrow);//ɾ����ͷ
	//gScene->removeActor(*Golf);//ɾ���ϵ�Golf
	PxVec3 lacc = PxVec3(0, 0, -20.0f);
	Golf->addForce(lacc, PxForceMode::eVELOCITY_CHANGE);
	//Golf = createDynamic(Golf->getGlobalPose(), PxSphereGeometry(1.0f), PxVec3(0, 0, -1.0f) * 10);//��ĳ��������Golf
	HANDLE tem=CreateThread(NULL, 0, changeArrow, NULL, 0, NULL);//�½��̼߳�������˶���ֹͣʱ���¼�ͷ
	TerminateThread(tem, 0);
}

void reset()
{
	gScene->removeActor(*Arrow);
	Golf->setGlobalPose(PxTransform(PxVec3(30.0f, 0.0f, 30.0f)));
	Golf->setLinearVelocity(PxVec3(0, 0, 0),1);/*�����ٶ�*/
	Golf->setAngularVelocity(PxVec3(0, 0, 0), 1);/*�Ƕ��ٶ�*/
	Arrow->setGlobalPose(PxTransform(PxVec3(30.0f, 0.0f, 25.0f)));
	PxTransform arrowPose(PxQuat(PxHalfPi, PxVec3(0, 1.0f, 0)));
	PxShape* arrowShape = PxRigidActorExt::createExclusiveShape(*Arrow, PxCapsuleGeometry(0.5f, 1.5f), *gMaterial);
	arrowShape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);//�ر���ײ
	arrowShape->setLocalPose(arrowPose);
	gScene->addActor(*Arrow);
	//gScene->addActor(*Golf);
}

void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent)
{
	/*����geometryģ�ͣ����鿴�ĵ�Geometry*/
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);/*������ �����*/
																										  //PxShape* shape = gPhysics->createShape(PxSphereGeometry(halfExtent), *gMaterial);/*���� �뾶*/
	for (PxU32 i = 0; i<size; i++)
	{
		for (PxU32 j = 0; j<size - i; j++)
		{
			PxTransform localTm(PxVec3(PxReal(j * 2) - PxReal(size - i), PxReal(i * 2 + 1), 0) * halfExtent);/*������ѿ�λ��*/
			PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
			body->attachShape(*shape);
			PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
			gScene->addActor(*body);
		}
	}
	shape->release();
}

void initPhysics(bool interactive)
{
	//��һ�� foundation��physx�汾���ڴ������16�ֽڶ��룬����ص�������
	gFoundation = PxCreateFoundation(PX_FOUNDATION_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);/*����pvd����*/
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);/*������host���˿ںţ�����Ƶ�ʣ�*/
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);/*����*/

															  //�ڶ��� ����physx���󣨰汾��foundationָ�룬������̹�ģ��pvd�Ƿ��¼�ڴ濪����pvd����
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());/*������������*/
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);/*����*/
	gDispatcher = PxDefaultCpuDispatcherCreate(2);/*�߳�����Ĭ��2*/
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(sceneDesc);/*��������*/

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	/*����*/
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.8f);/*��Ħ���� ��Ħ���� ���Իָ�ϵ�����������1m��������0.6m�ĸ߶�*/

														   /*����ƽ��actor[0]*/
	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	gScene->addActor(*groundPlane);

	/*���actor[1]*/
	PxRigidStatic* aFlagActor = gPhysics->createRigidStatic(PxTransform(PxVec3(20.0f, 45.0f, -65.0f)));/*���λ��*/
	PxTransform relativePose(PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
	PxShape* aFlagShape = PxRigidActorExt::createExclusiveShape(*aFlagActor,
		PxCapsuleGeometry(1.0f, 45.0f), *gMaterial);/*����shape��״*/
	aFlagShape->setLocalPose(relativePose);/*��ת��y������*/
	gScene->addActor(*aFlagActor);


	/*��actor[2]*/
	Golf = gPhysics->createRigidDynamic(PxTransform(PxVec3(30.0f, 0.0f, 30.0f)));/*����ʼλ��*/
	PxShape* aGolfShape = PxRigidActorExt::createExclusiveShape(*Golf,
		PxSphereGeometry(1.0f), *gMaterial);/*��״������*/
	PxRigidBodyExt::updateMassAndInertia(*Golf, 40.0f);/*�����ܶȣ�������*/
	Golf->setAngularDamping(0.5f);/*���ýǶ�����*/
	Golf->setLinearDamping(0.4f);/*������������*/
	gScene->addActor(*Golf);
	/*Ӧ�û���Ҫ���ü������������� ����isSleeping()���ֶ��� ������ϰ����������������λ�� ���߻�������ķ��� ��С*/

	//����ָʾ��ͷactor[3]
	Arrow = gPhysics->createRigidStatic(PxTransform(PxVec3(30.0f, 0.0f, 27.0f)));//����ָʾ��
	PxTransform arrowPose(PxQuat(PxHalfPi, PxVec3(0, 1.0f, 0)));
	PxShape* arrowShape = PxRigidActorExt::createExclusiveShape(*Arrow, PxCapsuleGeometry(0.5f, 1.5f), *gMaterial);
	arrowShape->setFlag(PxShapeFlag:: eSIMULATION_SHAPE, false);//�ر���ײ
	arrowShape->setLocalPose(arrowPose);
	gScene->addActor(*Arrow);

	/*�����������*/
	for (PxU32 i = 0; i<4; i++)
		createStack(PxTransform(PxVec3(10, 0, stackZ -= 16.0f)), 10, 2.0f);

	if (!interactive)
		createDynamic(PxTransform(PxVec3(0, 40, 100)), PxSphereGeometry(4), PxVec3(0, -50, -100));
}

void stepPhysics(bool interactive)/*ÿ֡����*/
{
	PX_UNUSED(interactive);
	gScene->simulate(1.0f / 60.0f);/*ÿ���60�Σ�����һ�μ���һ��*/
								   /*�м���ԼӲ�����ʵ���ϵ���һ֡�Ż���Ч*/
								   /*�ⲿ�ּ��㽫�첽���У����˷Ѽ����еĵȴ�ʱ��*/
	gScene->fetchResults(true);/*��ȡ����Ľ����true����������simulate������ɺ��������������ʱ�����1/60s��Ῠ*/
}

void cleanupPhysics(bool interactive)/*˳���෴��release*/
{
	PX_UNUSED(interactive);
	gScene->release();
	gDispatcher->release();
	gPhysics->release();
	PxPvdTransport* transport = gPvd->getTransport();
	gPvd->release();
	transport->release();

	gFoundation->release();

	printf("SnippetHelloWorld done.\n");
}

void keyPress(unsigned char key, const PxTransform& camera)/*�������봦���ⲿ��Ӧ��ͦ�ؼ���*/
{
	switch (toupper(key))
	{
		//case 'H':	createStack(PxTransform(PxVec3(10,0,stackZ-=16.0f)), 10, 2.0f);						break;
		//case ' ':	createDynamic(camera, PxSphereGeometry(1.0f), camera.rotate(PxVec3(0,0,-1))*100);	break;/*Ŀǰ��Ȼ�����˿ո�������� �ٶ�Ϊ*100*/
	case 'K':	hit();	break;
	case 'R':   reset(); break;
	}
}

int snippetMain(int, const char*const*)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for (PxU32 i = 0; i<frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}
