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

#ifdef RENDER_SNIPPET

#include <vector>
#include <Windows.h>
#include "PxPhysicsAPI.h"

#include "../SnippetRender/SnippetRender.h"
#include "../SnippetRender/SnippetCamera.h"

using namespace physx;

extern void initPhysics(bool interactive);
extern void stepPhysics(bool interactive);	
extern void cleanupPhysics(bool interactive);
extern void keyPress(unsigned char key, const PxTransform& camera);
extern PxRigidDynamic* Golf;

//实现方案1
void glWindowPos2i(GLint x, GLint y)
{
	typedef void(__stdcall * ARBFUNC)(GLint x, GLint y);

	ARBFUNC glptr = 0;
	glptr = (ARBFUNC)wglGetProcAddress("glWindowPos2iARB");

	if (glptr)
		glptr(x, y);
	else printf("glWindowPos2iARB NOT exit\n");
}
void* bitmap_fonts[7] = {
		GLUT_BITMAP_9_BY_15,
		GLUT_BITMAP_8_BY_13,
		GLUT_BITMAP_TIMES_ROMAN_10,
		GLUT_BITMAP_TIMES_ROMAN_24,
		GLUT_BITMAP_HELVETICA_10,
		GLUT_BITMAP_HELVETICA_12,
		GLUT_BITMAP_HELVETICA_18
};
void print_bitmap_string(void* font, const char* s)
{
	if (s && strlen(s)) {
		while (*s) {
			glutBitmapCharacter(font, *s);
			s++;
		}
	}
}
int TextOut(float x, float y, const char* cstr)
{
	glWindowPos2i(x, y);
	print_bitmap_string(bitmap_fonts[3], cstr);
	return 1;
}

//实现方案2
#define MAX_CHAR 128
void drawString(std::string strn) {
	static int isFirstCall = 1;
	static GLuint lists;
	const char* str = strn.c_str();
	if (isFirstCall) { // 如果是第一次调用，执行初始化
						 // 为每一个ASCII字符产生一个显示列表
		isFirstCall = 0;

		// 申请MAX_CHAR个连续的显示列表编号
		lists = glGenLists(MAX_CHAR);

		// 把每个字符的绘制命令都装到对应的显示列表中
		wglUseFontBitmaps(wglGetCurrentDC(), 0, MAX_CHAR, lists);
	}
	// 调用每个字符对应的显示列表，绘制每个字符
	while (*str != '\0')
	{
		glCallList(lists + *str);
		++str;
	}
}


namespace
{
Snippets::Camera*	sCamera;

void motionCallback(int x, int y)
{
	sCamera->handleMotion(x, y);
}

void keyboardCallback(unsigned char key, int x, int y)
{
	if(key==27)/*ESC*/
		exit(0);

	if(!sCamera->handleKey(key, x, y))
		keyPress(key, sCamera->getTransform());
}

void mouseCallback(int button, int state, int x, int y)
{
	sCamera->handleMouse(button, state, x, y);
}

void idleCallback()
{
	glutPostRedisplay();
}
PxVec3 get_location()
{
	return Golf->getGlobalPose().p;
}
void renderCallback()
{
	stepPhysics(true);/*每帧调用，simulate，fetchresult*/

	Snippets::startRender(get_location() + sCamera->getEye(), sCamera->getDir());
	//Snippets::startRender(sCamera->getEye(), sCamera->getDir());

	PxScene* scene;/*创建scene*/
	PxGetPhysics().getScenes(&scene,1);
	PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
	if(nbActors)
	{
		std::vector<PxRigidActor*> actors(nbActors);
		scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
		/*这里对每个actor进行渲染*/
		Snippets::renderActors(&actors[0], static_cast<PxU32>(1), true, PxVec3(0.0f, 1.0f, 0.0f));/*（actors，numActors，shadow，color）*/
		Snippets::renderActors(&actors[1], static_cast<PxU32>(1), true, PxVec3(1.0f, 1.0f, 1.0f));
		Snippets::renderActors(&actors[2], static_cast<PxU32>(1), true, PxVec3(0.9f, 0.9f, 0.9f));
		Snippets::renderActors(&actors[3], static_cast<PxU32>(actors.size() - 3), true, PxVec3(0.5f, 0.5f, 0.5f));
	}
	TextOut(10, 100, "Press Q,E to rotate angle");
	TextOut(10, 70, "Press H,J to control power");
	TextOut(10, 40, "Press SPACE to Hit");
	TextOut(10, 10, "Press R to Reset");
	Snippets::finishRender();
}

void exitCallback(void)
{
	delete sCamera;
	cleanupPhysics(true);
}
}




void renderLoop()
{
	sCamera = new Snippets::Camera(PxVec3(30.0f,30.0f,30.0f), PxVec3(-0.5f,-0.5f,-0.5f));/*摄像机初始位置，摄像机初始面向*/

	Snippets::setupDefaultWindow("PhysX Snippet Golf");
	Snippets::setupDefaultRenderState();

	glutIdleFunc(idleCallback);/*一堆回调函数*/
	glutDisplayFunc(renderCallback);
	glutKeyboardFunc(keyboardCallback);
	glutMouseFunc(mouseCallback);
	glutMotionFunc(motionCallback);
	motionCallback(0,0);

	atexit(exitCallback);

	initPhysics(true);
	glutMainLoop();
}
#endif
