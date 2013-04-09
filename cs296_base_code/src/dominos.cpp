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
//Added: Pulley System, Heavy Sphere on revolving platform, Revolving platform

#include "cs296_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;
//Added : 
#include "dominos.hpp"

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

    //Ground to keep sphere next to series of pendulams
    b2Body* b12;
    {
      b2EdgeShape shape;
      shape.Set(b2Vec2(-1.0f, 30.0f), b2Vec2(9.0f, 30.0f));
	
      b2BodyDef bd;
      b12 = m_world->CreateBody(&bd);
      b12->CreateFixture(&shape, 0.0f);
    }

    //Horizontal shelf on which dominos are kept
    {
      b2PolygonShape shape;
      shape.SetAsBox(16.0f, 0.25f);
	
      b2BodyDef bd;
      bd.position.Set(-17.0f, 19.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
	
    //Dominos
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 1.0f);
	
      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 2.0f;
      fd.friction = 0.1f;
		
      for (int i = 0; i < 10; ++i)
	{
	  b2BodyDef bd;
	  bd.type = b2_dynamicBody;
	  bd.position.Set(-19.5f + 1.0f * i, 20.25f);
	  b2Body* body = m_world->CreateBody(&bd);
	  body->CreateFixture(&fd);
	}
    }

    //The sphere which topples the dominos
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 0.1f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(-3.0f, 20.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }

    //The sphere which falls on the see-saw system
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 0.5f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(-22.0f, 20.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
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


    //The pendulum that knocks the ball off
    {
      b2Body* b2;
      {
	b2PolygonShape shape;
	shape.SetAsBox(1.5f, 0.25f);
	  
	b2BodyDef bd;
	bd.position.Set(42.0f, 43.0f);
	b2 = m_world->CreateBody(&bd);
	b2->CreateFixture(&shape, 10.0f);
      }
	
      b2Body* b4;
      {
        b2CircleShape circle;
        circle.m_radius = 1.0;
	  
	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position.Set(45.0f, 37.0f);
	b4 = m_world->CreateBody(&bd);
	b4->CreateFixture(&circle, 2.0f);
      }
	
      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(42.0f, 43.0f);
      jd.Initialize(b2, b4, anchor);
      m_world->CreateJoint(&jd);
    }
 
     
    //The train of pendulams
    {
     for (int i = 0; i < 8; ++i)
     {
      b2Body* b2;
      {
	b2PolygonShape shape;
	shape.SetAsBox(1.5f, 0.25f);
	  
	b2BodyDef bd;
	bd.position.Set(25.0f - i*2.1, 36.0f);
	b2 = m_world->CreateBody(&bd);
	b2->CreateFixture(&shape, 10.0f);
      }
	
      b2Body* b4;
      {
        b2CircleShape circle;
        circle.m_radius = 1.0;
	  
	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position.Set(25.0f - i*2.1, 31.0f);
	b4 = m_world->CreateBody(&bd);
	b4->CreateFixture(&circle,1.1f);
      }
	
      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(25.0f - i*2.1, 36.0f);
      jd.Initialize(b2, b4, anchor);
      m_world->CreateJoint(&jd);
     }
    }


    //The upper revolving dumble
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 3.0f);
	
      b2BodyDef bd;
      bd.position.Set(-2.0f, 28.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2BodyDef bd2;
      bd2.position.Set(-2.0f, 28.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    }


    //The lower revolving dumble
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 3.0f);
	
      b2BodyDef bd;
      bd.position.Set(-1.8f, 23.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2BodyDef bd2;
      bd2.position.Set(-1.8f, 23.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    }


    //The pulley system
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(25,15);
      bd->fixedRotation = true;
      
      //The open box
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
      fd3->shape = &bs3;
       
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);

      //The bar
      bd->position.Set(10,15);	
      fd1->density = 34.0;	  
      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(fd1);

      // The pulley joint
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(25, 15); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(10, 15); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(25, 20); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(10, 20); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);
    }


    //The revolving horizontal platform
    {
      b2PolygonShape shape;
      shape.SetAsBox(2.2f, 0.2f);
	
      b2BodyDef bd;
      bd.position.Set(6.0f, 16.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 2.0f);
      b2BodyDef bd2;
      bd2.position.Set(6.0f, 18.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    }


    //The heavy sphere on the revolving platform
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 50.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(6.0f, 22.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }


    //The sphere on the plank
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 2.0f;
      ballfd.friction = 0.3f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(39.0f, 34.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }

    //The sphere next to series of pendulams
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 1.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 1.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(8.2f, 31.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }

  }
  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
