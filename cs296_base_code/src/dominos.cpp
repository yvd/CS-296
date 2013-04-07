/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 296 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Parag Chaudhuri
 */


#include "cs296_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

namespace cs296
{
  dominos_t::dominos_t()
  {
    //Bottom Ground
	/*
	It provides support to the two sea-saws kept over it. It is a where the simulation terminates.
	The basket kept on right sea-saw catches the heavy spehere kept on the revolving platform.
	It is simlpy a thin visible line created using b2EdgeShape in Box2D.
	*/
    b2Body* b1;
    {
      b2EdgeShape shape;
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
	
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }

    //Surface to support the plank
	/*
	Since, the trapezium plank is a movable/dynamic object, it needs a surface to support it. 
	This support is provided to it by this static thin surface. Without this horizontal surface,
	the trapezium plank will fall down under the influence of gravity.
	*/
    b2Body* b10;
    {
      b2EdgeShape shape;
      shape.Set(b2Vec2(31.0f, 30.0f), b2Vec2(41.0f, 30.0f));
	
      b2BodyDef bd;
      b10 = m_world->CreateBody(&bd);
      b10->CreateFixture(&shape, 0.0f);
    }
      


    }
  }
  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
