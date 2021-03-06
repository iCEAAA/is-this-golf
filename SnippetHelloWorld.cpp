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

PxReal stackZ = 10.0f;/*初始堆的位置*/


extern PxRigidDynamic* Golf = nullptr;//把golf设置为全局访问
PxRigidStatic* Arrow = nullptr;//方向指示箭头
PxRigidStatic* aFlagActor = nullptr;
float arrowR = 5.0f;//箭头与球之间的距离
int rotateDegree = 90;//记录旋转角
bool won = false;
int Scene = 0;
extern int totalHit = 0;
extern bool exitFlag = 0;

float toRad(int degree)//角度转弧度
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
	Arrow = gPhysics->createRigidStatic(PxTransform(posv));//根据球的世界坐标设置箭头位置来创建箭头

	PxTransform arrowPose(PxQuat(toRad(rotateDegree), PxVec3(0, 1, 0)));
	PxShape* arrowShape = PxRigidActorExt::createExclusiveShape(*Arrow, PxCapsuleGeometry(0.5f, arrowR - 2.0f), *gMaterial);//箭头形状
	arrowShape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);//关闭碰撞
	arrowShape->setLocalPose(arrowPose);//设置箭头方向

	gScene->addActor(*Arrow);
}

//更新箭头的线程调用的函数
void renewArrow()
{
	while (!Golf->isSleeping());//当球还没停下时，loop
							   //球停下之后，摆一个新的箭头
	printf("createArrow\n");

	float x = arrowR * cos(toRad(rotateDegree));
	float z = -arrowR * sin(toRad(rotateDegree));
	PxVec3 posv(Golf->getGlobalPose().p + PxVec3(x, 0, z));
	Arrow = gPhysics->createRigidStatic(PxTransform(posv));//根据球的世界坐标设置箭头位置来创建箭头
	PxTransform arrowPose(PxQuat(toRad(rotateDegree), PxVec3(0, 1, 0)));

	//PxVec3 posv = Golf->getGlobalPose().p;//获取球的世界坐标  
	//rotateDegree = 90;//重置旋转角
	//Arrow = gPhysics->createRigidStatic(PxTransform(posv -= PxVec3(0.0f, 0.0f, 5.0f)));//根据球的世界坐标设置箭头位置来创建箭头
	//PxTransform arrowPose(PxQuat(PxHalfPi, PxVec3(0, 1, 0)));
	PxShape* arrowShape = PxRigidActorExt::createExclusiveShape(*Arrow, PxCapsuleGeometry(0.5f, arrowR - 2.0f), *gMaterial);//箭头形状
	arrowShape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);//关闭碰撞
	arrowShape->setLocalPose(arrowPose);//设置箭头方向为水平向前
										//Arrow->setGlobalPose(position);
	if (!exitFlag)
	{
		gScene->addActor(*Arrow);
	}
	else exit(0);
}

/////////////////////判断是否胜利，在球运动时新建线程判断///////////////////////////////////////////////////////////////
bool win = false;
void winning()
{
	if (Scene == 1)
	{
		while (!Golf->isSleeping())
		{
			if (win) {
				//偷懒做法，以后要换成碰撞检测逻辑
				if ((Golf->getGlobalPose().p - PxVec3(20.0f, Golf->getGlobalPose().p.y, -50.0f)).magnitude() < 3.0f)//到达旗杆
				{

					int x = MessageBox(GetForegroundWindow(), "恭喜你，你的球到达了旗杆！\n 按任意键结束游戏哦！", "【胜利】", 1);
					printf("%d\n", x);
					won = true;
				}
			}
			else
			{
				if ((Golf->getGlobalPose().p - PxVec3(20.0f, Golf->getGlobalPose().p.y, -65.0f)).magnitude() < 3.0f)//到达旗杆
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
			if ((Golf->getGlobalPose().p - PxVec3(flagX, Golf->getGlobalPose().p.y, flagZ)).magnitude() < 3.0f)//到达旗杆
			{
				int x = MessageBox(GetForegroundWindow(), "恭喜你，你的球到达了旗杆！\n 按任意键结束游戏哦！", "【胜利】", 1);
				printf("%d\n", x);
				won = true;
			}
		}
	}
}

//////////////////////对球进行操作球/////////////////////////////////////////////////////////////
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
	if (!Golf->isSleeping()) return;//当球没有停下时，不允许打击
	printf("hit\n");

	PxVec3 arrPos = Arrow->getGlobalPose().p;
	PxVec3 golfPos = Golf->getGlobalPose().p;
	float forceMagnitude = (arrPos - golfPos).magnitude() * 10;
	PxVec3 force = (arrPos - golfPos).getNormalized() * abs(forceMagnitude);//实现了LY的思路
																			//施加力的方向与大小 提供思路：方向 = 箭头的世界坐标 - 球的世界坐标 ，getGlobalPose返回的是位置+旋转信息，getGlobalPose().p这样得到的是位置的Vec3
	force.y = force.y + 10.0f;//增加y轴 往上打
	gScene->removeActor(*Arrow);//删除箭头
	Golf->addForce(force, PxForceMode::eVELOCITY_CHANGE);//施加力
	Golf->setSleepThreshold(18.0f);//休眠状态阈值
	std::thread renewArrow(renewArrow);//创建监听线程更新箭头
	renewArrow.detach();//使得线程脱离主线程的控制，执行完自动退出并且释放资源
	std::thread wining(winning);//创建线程监听胜利条件
	wining.detach();//同上
}

//球复位
void reset()
{
	if (!Golf->isSleeping()) return;//当球没有停下时，禁止重置，防止发生bug

	gScene->removeActor(*Arrow);
	Golf->setGlobalPose(PxTransform(PxVec3(30.0f, 0.0f, 30.0f)));
	Golf->setLinearVelocity(PxVec3(0, 0, 0), 1);/*线性速度*/
	Golf->setAngularVelocity(PxVec3(0, 0, 0), 1);/*角度速度*/
	Golf->putToSleep();
	Arrow = gPhysics->createRigidStatic(PxTransform(PxVec3(30.0f, 1.0f, 25.0f)));

	PxTransform arrowPose(PxQuat(PxHalfPi, PxVec3(0, 1.0f, 0)));
	PxShape* arrowShape = PxRigidActorExt::createExclusiveShape(*Arrow, PxCapsuleGeometry(0.5f, arrowR - 2.0f), *gMaterial);
	arrowShape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);//关闭碰撞
	arrowShape->setLocalPose(arrowPose);
	gScene->addActor(*Arrow);
	//gScene->addActor(*Golf);
}

////////////////////////对箭头进行操作//////////////////////////////////////////////////////////////
void rotateArrow()
{
	if (!Golf->isSleeping()) return;
	rotateDegree = (rotateDegree + 2) % 360;//改变角度，否则每次都只变60度了
											//变换的平面坐标系计算
	updateArrow(arrowR, rotateDegree);
}
void rotateArrow2()
{
	if (!Golf->isSleeping()) return;
	rotateDegree = (rotateDegree - 2) % 360;//改变角度，否则每次都只变60度了
											//变换的平面坐标系计算
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


//////////////////////////////////////创建各种场景///////////////////////////////////////////////////
//创建方块
void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent)// PxTransform(PxVec3(30, 0, stackZ - 16.0f))   5   2.0f
{
	/*创建geometry模型，详情看文档Geometry*/
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);/*长方体 长宽高*/
	//PxShape* shape = gPhysics->createShape(PxSphereGeometry(halfExtent), *gMaterial);/*球体 半径*/
	for (PxU32 i = 0; i < size; i++)
	{
		for (PxU32 j = 0; j < size - i; j++)
		{
			PxTransform localTm(PxVec3(PxReal(j*2) - PxReal(size-i), PxReal(i*2+1), i+1.0f) * halfExtent);
			//PxTransform localTm(PxVec3(PxReal(j * 2) - PxReal(size - i), 1, PxReal(i * 2 + 1)) * halfExtent); //物体平放
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
	/*材质*/
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.8f);/*静摩擦力 动摩擦力 弹性恢复系数：距离地面1m的球弹起来0.6m的高度*/

	/*场景平面actor[0]*/
	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	gScene->addActor(*groundPlane);

	/*旗杆actor[1]*/
	aFlagActor = gPhysics->createRigidStatic(PxTransform(PxVec3(20.0f, 45.0f, -65.0f)));/*旗杆位置*/
	PxTransform relativePose(PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
	PxShape* aFlagShape = PxRigidActorExt::createExclusiveShape(*aFlagActor, PxCapsuleGeometry(1.0f, 45.0f), *gMaterial);/*创建shape形状*/
	aFlagShape->setLocalPose(relativePose);/*旋转至y中轴线*/
	gScene->addActor(*aFlagActor);

	/*球actor[2]*/
	Golf = gPhysics->createRigidDynamic(PxTransform(PxVec3(30.0f, 0.0f, 30.0f)));/*球起始位置*/
	PxShape* aGolfShape = PxRigidActorExt::createExclusiveShape(*Golf,
		PxSphereGeometry(1.0f), *gMaterial);/*形状、材质*/
	PxRigidBodyExt::updateMassAndInertia(*Golf, 40.0f);/*设置密度（质量）*/

	Golf->setAngularDamping(0.2f);/*设置角度阻尼*/
	Golf->setLinearDamping(0.2f);/*设置线性阻尼*/
	gScene->addActor(*Golf);
	/*应该还需要设置几个参数来控制 比如isSleeping()这种东西 比如配合按键操作来控制球的位置 或者击球的力的方向 大小*/

	//方向指示箭头actor[3]
	//Arrow = gPhysics->createRigidStatic(PxTransform(PxVec3(30.0f, 0.0f, 27.0f)));//方向指示器
	PxTransform newPos = Golf->getGlobalPose().transform(PxTransform(0, 1.0f, -5.0f));

	Arrow = gPhysics->createRigidStatic(newPos);//方向指示器
	PxTransform arrowPose(PxQuat(PxHalfPi, PxVec3(0, 1.0f, 0)));
	PxShape* arrowShape = PxRigidActorExt::createExclusiveShape(*Arrow, PxCapsuleGeometry(0.5f, arrowR - 2.0f), *gMaterial);
	arrowShape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);//关闭碰撞
	arrowShape->setLocalPose(arrowPose);
	gScene->addActor(*Arrow);
	
	//围墙
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
	/*画的立方体堆*/
	for (PxU32 i = 0; i < 1; i++)
		createStack(PxTransform(PxVec3(30, 0, stackZ - 35.0f)), 7, 2.0f); //stackZ -= 16.0f

}
//hfb的场景
void createScene1()
{
	Scene = 1;
	/*材质*/
	gMaterial = gPhysics->createMaterial(0.8f, 0.5f, 0.7f);/*静摩擦力 动摩擦力 弹性恢复系数：距离地面1m的球弹起来0.6m的高度*/

	/*场景平面actor[0]*/
	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	gScene->addActor(*groundPlane);

	/*旗杆actor[1]*/
	aFlagActor = gPhysics->createRigidStatic(PxTransform(PxVec3(20.0f, 45.0f, -65.0f)));/*旗杆位置*/
	//PxRigidStatic* aFlagActor = gPhysics->createRigidStatic(PxTransform(PxVec3(20.0f, 45.0f, -50.0f)));/*旗杆位置*/
	PxTransform relativePose(PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
	PxShape* aFlagShape = PxRigidActorExt::createExclusiveShape(*aFlagActor, PxCapsuleGeometry(1.0f, 45.0f), *gMaterial);/*创建shape形状*/
	aFlagShape->setLocalPose(relativePose);/*旋转至y中轴线*/
	gScene->addActor(*aFlagActor);

	/*球actor[2]*/
	Golf = gPhysics->createRigidDynamic(PxTransform(PxVec3(30.0f, 0.0f, 30.0f)));/*球起始位置*/
	PxShape* aGolfShape = PxRigidActorExt::createExclusiveShape(*Golf,
		PxSphereGeometry(1.0f), *gMaterial);/*形状、材质*/
	PxRigidBodyExt::updateMassAndInertia(*Golf, 40.0f);/*设置密度（质量）*/

	Golf->setAngularDamping(0.2f);/*设置角度阻尼*/
	Golf->setLinearDamping(0.2f);/*设置线性阻尼*/
	gScene->addActor(*Golf);
	/*应该还需要设置几个参数来控制 比如isSleeping()这种东西 比如配合按键操作来控制球的位置 或者击球的力的方向 大小*/

	//方向指示箭头actor[3]
	//Arrow = gPhysics->createRigidStatic(PxTransform(PxVec3(30.0f, 0.0f, 27.0f)));//方向指示器
	PxTransform newPos = Golf->getGlobalPose().transform(PxTransform(0, 1.0f, -5.0f));

	Arrow = gPhysics->createRigidStatic(newPos);//方向指示器
	PxTransform arrowPose(PxQuat(PxHalfPi, PxVec3(0, 1.0f, 0)));
	PxShape* arrowShape = PxRigidActorExt::createExclusiveShape(*Arrow, PxCapsuleGeometry(0.5f, arrowR - 2.0f), *gMaterial);
	arrowShape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);//关闭碰撞
	arrowShape->setLocalPose(arrowPose);
	gScene->addActor(*Arrow);

	//围墙
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
	/*画的立方体堆*/
	for (PxU32 i = 0; i < 1; i++)
		createStack(PxTransform(PxVec3(30, 0, stackZ - 35.0f)), 7, 2.0f); //stackZ -= 16.0f

	//旗杆围栏
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
//ly的场景
void createScene2()
{
	Scene = 2;
	/*材质*/
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.8f);/*静摩擦力 动摩擦力 弹性恢复系数：距离地面1m的球弹起来0.6m的高度*/

	/*场景平面actor[0]*/
	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	gScene->addActor(*groundPlane);

	/*旗杆actor[1]*/
	aFlagActor = gPhysics->createRigidStatic(PxTransform(PxVec3(5.0f, 45.0f, -65.0f)));/*旗杆位置*/
	PxTransform relativePose(PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
	PxShape* aFlagShape = PxRigidActorExt::createExclusiveShape(*aFlagActor,
		PxCapsuleGeometry(1.0f, 45.0f), *gMaterial);/*创建shape形状*/
	aFlagShape->setLocalPose(relativePose);/*旋转至y中轴线*/
	gScene->addActor(*aFlagActor);


	/*球actor[2]*/
	Golf = gPhysics->createRigidDynamic(PxTransform(PxVec3(30.0f, 0.0f, 30.0f)));/*球起始位置*/
	PxShape* aGolfShape = PxRigidActorExt::createExclusiveShape(*Golf,
		PxSphereGeometry(1.0f), *gMaterial);/*形状、材质*/
	PxRigidBodyExt::updateMassAndInertia(*Golf, 40.0f);/*设置密度（质量）*/

	Golf->setAngularDamping(0.2f);/*设置角度阻尼*/
	Golf->setLinearDamping(0.2f);/*设置线性阻尼*/
	gScene->addActor(*Golf);
	/*应该还需要设置几个参数来控制 比如isSleeping()这种东西 比如配合按键操作来控制球的位置 或者击球的力的方向 大小*/

	//方向指示箭头actor[3]
	//Arrow = gPhysics->createRigidStatic(PxTransform(PxVec3(30.0f, 0.0f, 27.0f)));//方向指示器
	PxTransform newPos = Golf->getGlobalPose().transform(PxTransform(0, 1.0f, -5.0f));

	Arrow = gPhysics->createRigidStatic(newPos);//方向指示器
	PxTransform arrowPose(PxQuat(PxHalfPi, PxVec3(0, 1.0f, 0)));
	PxShape* arrowShape = PxRigidActorExt::createExclusiveShape(*Arrow, PxCapsuleGeometry(0.5f, arrowR - 2.0f), *gMaterial);
	arrowShape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);//关闭碰撞
	arrowShape->setLocalPose(arrowPose);
	gScene->addActor(*Arrow);

	{
		//前后墙
		PxRigidStatic* Wall1 = gPhysics->createRigidStatic(PxTransform(PxVec3(30.0f, 2.5f, 40.0f)));
		PxShape* WallShape1 = PxRigidActorExt::createExclusiveShape(*Wall1, PxBoxGeometry(30.0f, 5.0f, 1.0f), *gMaterial);
		gScene->addActor(*Wall1);
		PxRigidStatic* Wall2 = gPhysics->createRigidStatic(PxTransform(PxVec3(30.0f, 2.5f, -70.0f)));
		Wall2->attachShape(*WallShape1);
		gScene->addActor(*Wall2);

		//左右墙
		PxRigidStatic* Wall3 = gPhysics->createRigidStatic(PxTransform(PxVec3(0, 2.5f, -15.0f)));
		PxShape* WallShape3 = PxRigidActorExt::createExclusiveShape(*Wall3, PxBoxGeometry(1.0f, 5.0f, 55.0f), *gMaterial);
		gScene->addActor(*Wall3);
		PxRigidStatic* Wall4 = gPhysics->createRigidStatic(PxTransform(PxVec3(60.0f, 2.5f, -15.0f)));
		Wall4->attachShape(*WallShape3);
		gScene->addActor(*Wall4);

		//中间两片
		PxRigidStatic* wall5 = gPhysics->createRigidStatic(PxTransform(PxVec3(13.0f, 2.5f, -15.0f)));
		PxShape* WallShape5 = PxRigidActorExt::createExclusiveShape(*wall5, PxBoxGeometry(13.0f, 5.0f, 1.0f), *gMaterial);
		gScene->addActor(*wall5);
		PxRigidStatic* wall6 = gPhysics->createRigidStatic(PxTransform(PxVec3(47.0f, 2.5f, -15.0f)));
		wall6->attachShape(*WallShape5);
		gScene->addActor(*wall6);

		//下方两片
		PxRigidStatic* wall7 = gPhysics->createRigidStatic(PxTransform(PxVec3(20.0f, 2.5f, 22.0f)));
		PxShape* WallShape7 = PxRigidActorExt::createExclusiveShape(*wall7, PxBoxGeometry(20.0f, 5.0f, 1.0f), *gMaterial);
		gScene->addActor(*wall7);
		PxRigidStatic* wall8 = gPhysics->createRigidStatic(PxTransform(PxVec3(40.0f, 2.5f, 4.0f)));
		wall8->attachShape(*WallShape7);
		gScene->addActor(*wall8);

		//上方障碍
		PxRigidStatic* wall9 = gPhysics->createRigidStatic(PxTransform(PxVec3(30.0f, 2.5f, -42.0f)));
		PxShape* WallShape9 = PxRigidActorExt::createExclusiveShape(*wall9, PxBoxGeometry(15.0f, 5.0f, 1.0f), *gMaterial);
		gScene->addActor(*wall9);
	}
}
void initPhysics(bool interactive, int scene)
{
	//第一步 foundation（physx版本，内存分配器16字节对齐，错误回调函数）
	gFoundation = PxCreateFoundation(PX_FOUNDATION_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);/*创建pvd对象*/
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);/*（本地host，端口号，调用频率）*/
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);/*连接*/

	//第二部 创建physx对象（版本，foundation指针，误差容忍规模，pvd是否记录内存开销，pvd对象）
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());/*场景描述对象*/
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);/*重力*/
	gDispatcher = PxDefaultCpuDispatcherCreate(2);/*线程数，默认2*/
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(sceneDesc);/*创建场景*/

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
void stepPhysics(bool interactive)/*每帧调用*/
{
	PX_UNUSED(interactive);
	gScene->simulate(1.0f / 60.0f);/*每秒调60次，调用一次计算一次*/
								   /*中间可以加操作，实际上到下一帧才会生效*/
								   /*这部分计算将异步进行，不浪费计算中的等待时间*/
	gScene->fetchResults(true);/*获取计算的结果，true：阻塞：等simulate计算完成后才输出，如果计算时间大于1/60s则会卡*/
}
void cleanupPhysics(bool interactive)/*顺序相反地release*/
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

////////////////////////按键输入处理//////////////////////////////////////////////////////////////////////////
void keyPress(unsigned char key, const PxTransform& camera)
{
	//if (won) exit(0);
	
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
//////////////////////main/////////////////////////////////////////////////////////////////////////
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
