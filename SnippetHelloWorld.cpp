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
#include <thread>
#include <cmath>
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

PxFoundation* gFoundation = NULL;
PxPhysics* gPhysics = NULL;

PxDefaultCpuDispatcher* gDispatcher = NULL;
PxScene* gScene = NULL;

PxMaterial* gMaterial = NULL;

PxPvd* gPvd = NULL;

PxReal stackZ = 10.0f;/*��ʼ�ѵ�λ��*/


extern PxRigidDynamic* Golf = nullptr;//��golf����Ϊȫ�ַ���
PxRigidStatic* Arrow = nullptr;//����ָʾ��ͷ
PxRigidStatic* aFlagActor = nullptr;
float arrowR = 5.0f;//��ͷ����֮��ľ���
int rotateDegree = 90;//��¼��ת��
bool won = false;
int Scene = 0;
extern int totalHit = 0;

float toRad(int degree)//�Ƕ�ת����
{
	return degree * (PxPi / 180.0f);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateArrow(float arrowR, int rotateDegree)
{
	gScene->removeActor(*Arrow);

	float x = arrowR * cos(toRad(rotateDegree));
	float z = -arrowR * sin(toRad(rotateDegree));
	PxVec3 posv(Golf->getGlobalPose().p + PxVec3(x, 0, z));
	Arrow = gPhysics->createRigidStatic(PxTransform(posv));//������������������ü�ͷλ����������ͷ

	PxTransform arrowPose(PxQuat(toRad(rotateDegree), PxVec3(0, 1, 0)));
	PxShape* arrowShape = PxRigidActorExt::createExclusiveShape(*Arrow, PxCapsuleGeometry(0.5f, arrowR - 2.0f), *gMaterial);//��ͷ��״
	arrowShape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);//�ر���ײ
	arrowShape->setLocalPose(arrowPose);//���ü�ͷ����

	gScene->addActor(*Arrow);
}

//���¼�ͷ���̵߳��õĺ���
void renewArrow()
{
	while (!Golf->isSleeping());//����ûͣ��ʱ��loop
								//��ͣ��֮�󣬰�һ���µļ�ͷ
	printf("createArrow\n");

	float x = arrowR * cos(toRad(rotateDegree));
	float z = -arrowR * sin(toRad(rotateDegree));
	PxVec3 posv(Golf->getGlobalPose().p + PxVec3(x, 0, z));
	Arrow = gPhysics->createRigidStatic(PxTransform(posv));//������������������ü�ͷλ����������ͷ
	PxTransform arrowPose(PxQuat(toRad(rotateDegree), PxVec3(0, 1, 0)));

	//PxVec3 posv = Golf->getGlobalPose().p;//��ȡ�����������  
	//rotateDegree = 90;//������ת��
	//Arrow = gPhysics->createRigidStatic(PxTransform(posv -= PxVec3(0.0f, 0.0f, 5.0f)));//������������������ü�ͷλ����������ͷ
	//PxTransform arrowPose(PxQuat(PxHalfPi, PxVec3(0, 1, 0)));
	PxShape* arrowShape = PxRigidActorExt::createExclusiveShape(*Arrow, PxCapsuleGeometry(0.5f, arrowR - 2.0f), *gMaterial);//��ͷ��״
	arrowShape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);//�ر���ײ
	arrowShape->setLocalPose(arrowPose);//���ü�ͷ����Ϊˮƽ��ǰ
										//Arrow->setGlobalPose(position);
	gScene->addActor(*Arrow);
}

/////////////////////�ж��Ƿ�ʤ���������˶�ʱ�½��߳��ж�///////////////////////////////////////////////////////////////
bool win = false;
void winning()
{
	if (Scene == 1)
	{
		while (!Golf->isSleeping())
		{
			if (win) {
				//͵���������Ժ�Ҫ������ײ����߼�
				if ((Golf->getGlobalPose().p - PxVec3(20.0f, Golf->getGlobalPose().p.y, -50.0f)).magnitude() < 3.0f)//�������
				{

					int x = MessageBox(GetForegroundWindow(), "��ϲ�㣬����򵽴�����ˣ�\n �������������ϷŶ��", "��ʤ����", 1);
					printf("%d\n", x);
					won = true;
				}
			}
			else
			{
				if ((Golf->getGlobalPose().p - PxVec3(20.0f, Golf->getGlobalPose().p.y, -65.0f)).magnitude() < 3.0f)//�������
				{
					aFlagActor->setGlobalPose(PxTransform(PxVec3(20.0f, 45.0f, -50.0f)));
					win = true;
				}
			}
		}
	}
	else
	{
		while (!Golf->isSleeping())
		{
			float flagX = aFlagActor->getGlobalPose().p.x;
			float flagZ = aFlagActor->getGlobalPose().p.z;
			if ((Golf->getGlobalPose().p - PxVec3(flagX, Golf->getGlobalPose().p.y, flagZ)).magnitude() < 3.0f)//�������
			{
				int x = MessageBox(GetForegroundWindow(), "��ϲ�㣬����򵽴�����ˣ�\n �������������ϷŶ��", "��ʤ����", 1);
				printf("%d\n", x);
				won = true;
			}
		}
	}
}

//////////////////////������/////////////////////////////////////////////////////////////
int i = 0;
PxVec3 flagPos[4] = {
	PxVec3(5.0f, 45.0f, -65.0f),
	PxVec3(55.0f,45.0f, -65.0f),
	PxVec3(55.0f,45.0f, -20.0f),
	PxVec3(5.0f, 45.0f, -20.0f)
};
void hit()
{
	totalHit++;
	if (Scene == 2)
	{
		aFlagActor->setGlobalPose(PxTransform(flagPos[(++i) % 4]));
	}
	if (!Golf->isSleeping()) return;//����û��ͣ��ʱ����������
	printf("hit\n");

	PxVec3 arrPos = Arrow->getGlobalPose().p;
	PxVec3 golfPos = Golf->getGlobalPose().p;
	float forceMagnitude = (arrPos - golfPos).magnitude() * 10;
	PxVec3 force = (arrPos - golfPos).getNormalized() * abs(forceMagnitude);//ʵ����LY��˼·
																			//ʩ�����ķ������С �ṩ˼·������ = ��ͷ���������� - ����������� ��getGlobalPose���ص���λ��+��ת��Ϣ��getGlobalPose().p�����õ�����λ�õ�Vec3
	force.y = force.y + 5.0f;//����y�� ���ϴ�
	gScene->removeActor(*Arrow);//ɾ����ͷ
	Golf->addForce(force, PxForceMode::eVELOCITY_CHANGE);//ʩ����
	Golf->setSleepThreshold(15.0f);//����״̬��ֵ
	std::thread renewArrow(renewArrow);//���������̸߳��¼�ͷ
	renewArrow.detach();//ʹ���߳��������̵߳Ŀ��ƣ�ִ�����Զ��˳������ͷ���Դ
	std::thread wining(winning);//�����̼߳���ʤ������
	wining.detach();//ͬ��
}

//��λ
void reset()
{
	if (!Golf->isSleeping()) return;//����û��ͣ��ʱ����ֹ���ã���ֹ����bug

	gScene->removeActor(*Arrow);
	Golf->setGlobalPose(PxTransform(PxVec3(30.0f, 0.0f, 30.0f)));
	Golf->setLinearVelocity(PxVec3(0, 0, 0), 1);/*�����ٶ�*/
	Golf->setAngularVelocity(PxVec3(0, 0, 0), 1);/*�Ƕ��ٶ�*/
	Golf->putToSleep();
	Arrow = gPhysics->createRigidStatic(PxTransform(PxVec3(30.0f, 1.0f, 25.0f)));

	PxTransform arrowPose(PxQuat(PxHalfPi, PxVec3(0, 1.0f, 0)));
	PxShape* arrowShape = PxRigidActorExt::createExclusiveShape(*Arrow, PxCapsuleGeometry(0.5f, arrowR - 2.0f), *gMaterial);
	arrowShape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);//�ر���ײ
	arrowShape->setLocalPose(arrowPose);
	gScene->addActor(*Arrow);
	//gScene->addActor(*Golf);
}

////////////////////////�Լ�ͷ���в���//////////////////////////////////////////////////////////////
void rotateArrow()
{
	if (!Golf->isSleeping()) return;
	rotateDegree = (rotateDegree + 2) % 360;//�ı�Ƕȣ�����ÿ�ζ�ֻ��60����
											//�任��ƽ������ϵ����
	updateArrow(arrowR, rotateDegree);
}
void rotateArrow2()
{
	if (!Golf->isSleeping()) return;
	rotateDegree = (rotateDegree - 2) % 360;//�ı�Ƕȣ�����ÿ�ζ�ֻ��60����
											//�任��ƽ������ϵ����
	updateArrow(arrowR, rotateDegree);
}
void Harder()
{
	if (!Golf->isSleeping()) return;
	arrowR = (arrowR <= 9.0f) ? arrowR + 0.2f : arrowR;

	updateArrow(arrowR, rotateDegree);
}
void Weaker()
{
	if (!Golf->isSleeping()) return;
	arrowR = (arrowR >= 3.0f) ? arrowR - 0.2f : arrowR;
	updateArrow(arrowR, rotateDegree);
}


//////////////////////////////////////�������ֳ���///////////////////////////////////////////////////
//��������
void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent)// PxTransform(PxVec3(30, 0, stackZ - 16.0f))   5   2.0f
{
	/*����geometryģ�ͣ����鿴�ĵ�Geometry*/
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);/*������ �����*/
	//PxShape* shape = gPhysics->createShape(PxSphereGeometry(halfExtent), *gMaterial);/*���� �뾶*/
	for (PxU32 i = 0; i < size; i++)
	{
		for (PxU32 j = 0; j < size - i; j++)
		{
			PxTransform localTm(PxVec3(PxReal(j*2) - PxReal(size-i), PxReal(i*2+1), i+1.0f) * halfExtent);
			//PxTransform localTm(PxVec3(PxReal(j * 2) - PxReal(size - i), 1, PxReal(i * 2 + 1)) * halfExtent); //����ƽ��
			PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
			body->attachShape(*shape);
			PxRigidBodyExt::updateMassAndInertia(*body, 1.0f);//mass
			gScene->addActor(*body);
		}
	}
	shape->release();
}
//original
void createScene()
{
	Scene = 0;
	/*����*/
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.8f);/*��Ħ���� ��Ħ���� ���Իָ�ϵ�����������1m��������0.6m�ĸ߶�*/

	/*����ƽ��actor[0]*/
	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	gScene->addActor(*groundPlane);

	/*���actor[1]*/
	aFlagActor = gPhysics->createRigidStatic(PxTransform(PxVec3(20.0f, 45.0f, -65.0f)));/*���λ��*/
	PxTransform relativePose(PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
	PxShape* aFlagShape = PxRigidActorExt::createExclusiveShape(*aFlagActor, PxCapsuleGeometry(1.0f, 45.0f), *gMaterial);/*����shape��״*/
	aFlagShape->setLocalPose(relativePose);/*��ת��y������*/
	gScene->addActor(*aFlagActor);

	/*��actor[2]*/
	Golf = gPhysics->createRigidDynamic(PxTransform(PxVec3(30.0f, 0.0f, 30.0f)));/*����ʼλ��*/
	PxShape* aGolfShape = PxRigidActorExt::createExclusiveShape(*Golf,
		PxSphereGeometry(1.0f), *gMaterial);/*��״������*/
	PxRigidBodyExt::updateMassAndInertia(*Golf, 40.0f);/*�����ܶȣ�������*/

	Golf->setAngularDamping(0.2f);/*���ýǶ�����*/
	Golf->setLinearDamping(0.2f);/*������������*/
	gScene->addActor(*Golf);
	/*Ӧ�û���Ҫ���ü������������� ����isSleeping()���ֶ��� ������ϰ����������������λ�� ���߻�������ķ��� ��С*/

	//����ָʾ��ͷactor[3]
	//Arrow = gPhysics->createRigidStatic(PxTransform(PxVec3(30.0f, 0.0f, 27.0f)));//����ָʾ��
	PxTransform newPos = Golf->getGlobalPose().transform(PxTransform(0, 1.0f, -5.0f));

	Arrow = gPhysics->createRigidStatic(newPos);//����ָʾ��
	PxTransform arrowPose(PxQuat(PxHalfPi, PxVec3(0, 1.0f, 0)));
	PxShape* arrowShape = PxRigidActorExt::createExclusiveShape(*Arrow, PxCapsuleGeometry(0.5f, arrowR - 2.0f), *gMaterial);
	arrowShape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);//�ر���ײ
	arrowShape->setLocalPose(arrowPose);
	gScene->addActor(*Arrow);
	
	//Χǽ
	PxRigidStatic* Wall1 = gPhysics->createRigidStatic(PxTransform(PxVec3(30.0f, 2.0f, 40.0f)));
	PxShape* WallShape1 = PxRigidActorExt::createExclusiveShape(*Wall1, PxBoxGeometry(30.0f, 4.0f, 1.0f), *gMaterial);
	gScene->addActor(*Wall1);

	PxRigidStatic* Wall2 = gPhysics->createRigidStatic(PxTransform(PxVec3(30.0f, 2.0f, -70.0f)));
	PxShape* WallShape2 = PxRigidActorExt::createExclusiveShape(*Wall2, PxBoxGeometry(30.0f, 4.0f, 1.0f), *gMaterial);
	gScene->addActor(*Wall2);

	PxRigidStatic* Wall3 = gPhysics->createRigidStatic(PxTransform(PxVec3(0, 2.0f, -25.0f)));
	PxShape* WallShape3 = PxRigidActorExt::createExclusiveShape(*Wall3, PxBoxGeometry(1.0f, 4.0f, 65.0f), *gMaterial);
	gScene->addActor(*Wall3);

	PxRigidStatic* Wall4 = gPhysics->createRigidStatic(PxTransform(PxVec3(60.0f, 2.0f, -25.0f)));
	PxShape* WallShape4 = PxRigidActorExt::createExclusiveShape(*Wall4, PxBoxGeometry(1.0f, 4.0f, 65.0f), *gMaterial);
	gScene->addActor(*Wall4);
	//
	/*�����������*/
	for (PxU32 i = 0; i < 1; i++)
		createStack(PxTransform(PxVec3(30, 0, stackZ - 35.0f)), 7, 2.0f); //stackZ -= 16.0f

}
//hfb�ĳ���
void createScene1()
{
	Scene = 1;
	/*����*/
	gMaterial = gPhysics->createMaterial(0.8f, 0.5f, 0.7f);/*��Ħ���� ��Ħ���� ���Իָ�ϵ�����������1m��������0.6m�ĸ߶�*/

	/*����ƽ��actor[0]*/
	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	gScene->addActor(*groundPlane);

	/*���actor[1]*/
	aFlagActor = gPhysics->createRigidStatic(PxTransform(PxVec3(20.0f, 45.0f, -65.0f)));/*���λ��*/
	//PxRigidStatic* aFlagActor = gPhysics->createRigidStatic(PxTransform(PxVec3(20.0f, 45.0f, -50.0f)));/*���λ��*/
	PxTransform relativePose(PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
	PxShape* aFlagShape = PxRigidActorExt::createExclusiveShape(*aFlagActor, PxCapsuleGeometry(1.0f, 45.0f), *gMaterial);/*����shape��״*/
	aFlagShape->setLocalPose(relativePose);/*��ת��y������*/
	gScene->addActor(*aFlagActor);

	/*��actor[2]*/
	Golf = gPhysics->createRigidDynamic(PxTransform(PxVec3(30.0f, 0.0f, 30.0f)));/*����ʼλ��*/
	PxShape* aGolfShape = PxRigidActorExt::createExclusiveShape(*Golf,
		PxSphereGeometry(1.0f), *gMaterial);/*��״������*/
	PxRigidBodyExt::updateMassAndInertia(*Golf, 40.0f);/*�����ܶȣ�������*/

	Golf->setAngularDamping(0.2f);/*���ýǶ�����*/
	Golf->setLinearDamping(0.2f);/*������������*/
	gScene->addActor(*Golf);
	/*Ӧ�û���Ҫ���ü������������� ����isSleeping()���ֶ��� ������ϰ����������������λ�� ���߻�������ķ��� ��С*/

	//����ָʾ��ͷactor[3]
	//Arrow = gPhysics->createRigidStatic(PxTransform(PxVec3(30.0f, 0.0f, 27.0f)));//����ָʾ��
	PxTransform newPos = Golf->getGlobalPose().transform(PxTransform(0, 1.0f, -5.0f));

	Arrow = gPhysics->createRigidStatic(newPos);//����ָʾ��
	PxTransform arrowPose(PxQuat(PxHalfPi, PxVec3(0, 1.0f, 0)));
	PxShape* arrowShape = PxRigidActorExt::createExclusiveShape(*Arrow, PxCapsuleGeometry(0.5f, arrowR - 2.0f), *gMaterial);
	arrowShape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);//�ر���ײ
	arrowShape->setLocalPose(arrowPose);
	gScene->addActor(*Arrow);

	//Χǽ
	PxRigidStatic* Wall1 = gPhysics->createRigidStatic(PxTransform(PxVec3(30.0f, 2.0f, 40.0f)));
	PxShape* WallShape1 = PxRigidActorExt::createExclusiveShape(*Wall1, PxBoxGeometry(30.0f, 4.0f, 1.0f), *gMaterial);
	gScene->addActor(*Wall1);

	PxRigidStatic* Wall2 = gPhysics->createRigidStatic(PxTransform(PxVec3(30.0f, 2.0f, -90.0f)));
	PxShape* WallShape2 = PxRigidActorExt::createExclusiveShape(*Wall2, PxBoxGeometry(30.0f, 4.0f, 1.0f), *gMaterial);
	gScene->addActor(*Wall2);

	PxRigidStatic* Wall3 = gPhysics->createRigidStatic(PxTransform(PxVec3(0, 2.0f, -25.0f)));
	PxShape* WallShape3 = PxRigidActorExt::createExclusiveShape(*Wall3, PxBoxGeometry(1.0f, 4.0f, 65.0f), *gMaterial);
	gScene->addActor(*Wall3);

	PxRigidStatic* Wall4 = gPhysics->createRigidStatic(PxTransform(PxVec3(60.0f, 2.0f, -25.0f)));
	PxShape* WallShape4 = PxRigidActorExt::createExclusiveShape(*Wall4, PxBoxGeometry(1.0f, 4.0f, 65.0f), *gMaterial);
	gScene->addActor(*Wall4);
	//
	/*�����������*/
	for (PxU32 i = 0; i < 1; i++)
		createStack(PxTransform(PxVec3(30, 0, stackZ - 35.0f)), 7, 2.0f); //stackZ -= 16.0f

	//���Χ��
	PxRigidStatic* fence1 = gPhysics->createRigidStatic(PxTransform(PxVec3(10.0f, 2.0f, -65.0f)));
	PxShape* fenceShape1 = PxRigidActorExt::createExclusiveShape(*fence1, PxBoxGeometry(1.0f, 4.0f, 20.0f), *gMaterial);
	gScene->addActor(*fence1);

	PxRigidStatic* fence2 = gPhysics->createRigidStatic(PxTransform(PxVec3(30.0f, 2.0f, -65.0f)));
	PxShape* fenceShape2 = PxRigidActorExt::createExclusiveShape(*fence2, PxBoxGeometry(1.0f, 4.0f, 20.0f), *gMaterial);
	gScene->addActor(*fence2);

	PxRigidStatic* fence3 = gPhysics->createRigidStatic(PxTransform(PxVec3(20, 2.0f, -55.0f)));
	PxShape* fenceShape3 = PxRigidActorExt::createExclusiveShape(*fence3, PxBoxGeometry(10.0f, 4.0f, 1.0f), *gMaterial);
	gScene->addActor(*fence3);
}
//ly�ĳ���
void createScene2()
{
	Scene = 2;
	/*����*/
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.8f);/*��Ħ���� ��Ħ���� ���Իָ�ϵ�����������1m��������0.6m�ĸ߶�*/

	/*����ƽ��actor[0]*/
	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	gScene->addActor(*groundPlane);

	/*���actor[1]*/
	aFlagActor = gPhysics->createRigidStatic(PxTransform(PxVec3(5.0f, 45.0f, -65.0f)));/*���λ��*/
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

	Golf->setAngularDamping(0.2f);/*���ýǶ�����*/
	Golf->setLinearDamping(0.2f);/*������������*/
	gScene->addActor(*Golf);
	/*Ӧ�û���Ҫ���ü������������� ����isSleeping()���ֶ��� ������ϰ����������������λ�� ���߻�������ķ��� ��С*/

	//����ָʾ��ͷactor[3]
	//Arrow = gPhysics->createRigidStatic(PxTransform(PxVec3(30.0f, 0.0f, 27.0f)));//����ָʾ��
	PxTransform newPos = Golf->getGlobalPose().transform(PxTransform(0, 1.0f, -5.0f));

	Arrow = gPhysics->createRigidStatic(newPos);//����ָʾ��
	PxTransform arrowPose(PxQuat(PxHalfPi, PxVec3(0, 1.0f, 0)));
	PxShape* arrowShape = PxRigidActorExt::createExclusiveShape(*Arrow, PxCapsuleGeometry(0.5f, arrowR - 2.0f), *gMaterial);
	arrowShape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);//�ر���ײ
	arrowShape->setLocalPose(arrowPose);
	gScene->addActor(*Arrow);

	{
		//ǰ��ǽ
		PxRigidStatic* Wall1 = gPhysics->createRigidStatic(PxTransform(PxVec3(30.0f, 2.5f, 40.0f)));
		PxShape* WallShape1 = PxRigidActorExt::createExclusiveShape(*Wall1, PxBoxGeometry(30.0f, 5.0f, 1.0f), *gMaterial);
		gScene->addActor(*Wall1);
		PxRigidStatic* Wall2 = gPhysics->createRigidStatic(PxTransform(PxVec3(30.0f, 2.5f, -70.0f)));
		Wall2->attachShape(*WallShape1);
		gScene->addActor(*Wall2);

		//����ǽ
		PxRigidStatic* Wall3 = gPhysics->createRigidStatic(PxTransform(PxVec3(0, 2.5f, -15.0f)));
		PxShape* WallShape3 = PxRigidActorExt::createExclusiveShape(*Wall3, PxBoxGeometry(1.0f, 5.0f, 55.0f), *gMaterial);
		gScene->addActor(*Wall3);
		PxRigidStatic* Wall4 = gPhysics->createRigidStatic(PxTransform(PxVec3(60.0f, 2.5f, -15.0f)));
		Wall4->attachShape(*WallShape3);
		gScene->addActor(*Wall4);

		//�м���Ƭ
		PxRigidStatic* wall5 = gPhysics->createRigidStatic(PxTransform(PxVec3(13.0f, 2.5f, -15.0f)));
		PxShape* WallShape5 = PxRigidActorExt::createExclusiveShape(*wall5, PxBoxGeometry(13.0f, 5.0f, 1.0f), *gMaterial);
		gScene->addActor(*wall5);
		PxRigidStatic* wall6 = gPhysics->createRigidStatic(PxTransform(PxVec3(47.0f, 2.5f, -15.0f)));
		wall6->attachShape(*WallShape5);
		gScene->addActor(*wall6);

		//�·���Ƭ
		PxRigidStatic* wall7 = gPhysics->createRigidStatic(PxTransform(PxVec3(25.0f, 2.5f, 22.0f)));
		PxShape* WallShape7 = PxRigidActorExt::createExclusiveShape(*wall7, PxBoxGeometry(25.0f, 5.0f, 1.0f), *gMaterial);
		gScene->addActor(*wall7);
		PxRigidStatic* wall8 = gPhysics->createRigidStatic(PxTransform(PxVec3(35.0f, 2.5f, 4.0f)));
		wall8->attachShape(*WallShape7);
		gScene->addActor(*wall8);

		//�Ϸ��ϰ�
		PxRigidStatic* wall9 = gPhysics->createRigidStatic(PxTransform(PxVec3(30.0f, 2.5f, -42.0f)));
		PxShape* WallShape9 = PxRigidActorExt::createExclusiveShape(*wall9, PxBoxGeometry(15.0f, 5.0f, 1.0f), *gMaterial);
		gScene->addActor(*wall9);
	}
}
void initPhysics(bool interactive, int scene)
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

	switch (scene)
	{
	case 0:createScene(); break;//original
	case 1:createScene1(); break;//hfb
	case 2:createScene2(); break;//ly
	default:
		break;
	}

}

///////////////////////////////////////////////////////////////////////////////////////////////
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

////////////////////////�������봦��//////////////////////////////////////////////////////////////////////////
void keyPress(unsigned char key, const PxTransform& camera)
{
	if (won)
	{
		exit(0);
	}
	switch (toupper(key))
	{
	case 'Y':	cleanupPhysics(true); initPhysics(true, 0); break;
	case 'U':   cleanupPhysics(true); initPhysics(true, 1); break;
	case 'I':	cleanupPhysics(true); initPhysics(true, 2); break;

	case 'Q':	rotateArrow(); break;
	case 'E':	rotateArrow2(); break;
	case ' ':	hit();	break;
	case 'R':   reset(); break;
	case 'H':	Harder(); break;
	case 'J':	Weaker(); break;
	}
}

int snippetMain(int, const char* const*)
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
