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
// Added :  Ground to support the sphere connected to the plank,  The sphere attached to the plank
namespace cs296
{
  dominos_t::dominos_t()
  {
    //Bottom Ground

    b2Body* b1;
    {
      b2EdgeShape shape;
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
	
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }

    //Surface to support the plank

    b2Body* b10;
    {
      b2EdgeShape shape;
      shape.Set(b2Vec2(31.0f, 30.0f), b2Vec2(41.0f, 30.0f));
	
      b2BodyDef bd;
      b10 = m_world->CreateBody(&bd);
      b10->CreateFixture(&shape, 0.0f);
    }
      
    //Ground to support the sphere connected to the plank

    b2Body* b11;
    {
      b2EdgeShape shape;
      shape.Set(b2Vec2(-0.6f, 25.0f), b2Vec2(1.0f, 25.0f));
	
      b2BodyDef bd;
      b11 = m_world->CreateBody(&bd);
      b11->CreateFixture(&shape, 0.0f);
    }


     {
      //The Trapezium Plank
	b2Body* plankBody;
      {
        
        b2PolygonShape poly;
        b2Vec2 vertices[4];
        vertices[0].Set(-5,0);
        vertices[1].Set(10,0);
        vertices[2].Set(10,5);
        vertices[3].Set(0,5);
        poly.Set(vertices, 4);
        b2FixtureDef wedgefd;
        wedgefd.shape = &poly;
        wedgefd.density = 0.5f;
        wedgefd.friction = 0.1f;
        wedgefd.restitution = 0.0f;
        b2BodyDef wedgebd;
        wedgebd.type = b2_dynamicBody;
        wedgebd.position.Set(31.0f, 30.0f);
        plankBody = m_world->CreateBody(&wedgebd);
        plankBody->CreateFixture(&wedgefd);
      }


      //The sphere attached to the plank
	b2Body* sphereBody;
      {
        
        b2CircleShape circle;
        circle.m_radius = 1.0;
	
        b2FixtureDef ballfd;
        ballfd.shape = &circle;
        ballfd.density = 1.0f;
        ballfd.friction = 0.0f;
        ballfd.restitution = 0.0f;
        b2BodyDef ballbd;
        ballbd.type = b2_dynamicBody;
        ballbd.position.Set(-0.2f, 26.0f);
        sphereBody = m_world->CreateBody(&ballbd);
        sphereBody->CreateFixture(&ballfd);
      }
  
	b2DistanceJointDef jointDef;
	b2Vec2 anchorA;
	anchorA.Set(31.0f, 30.0f);
	b2Vec2 anchorB;
	anchorB.Set(-0.2f, 26.0f);
	jointDef.Initialize(plankBody, sphereBody, anchorA, anchorB);
	m_world->CreateJoint(&jointDef);

    }


  }
  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
