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
    ///<B><HR>Bottom Ground</B>
	/*!
	* It provides support to the two sea-saws kept over it. It is a where the simulation terminates.<BR>
	* The basket kept on right sea-saw catches the heavy spehere kept on the revolving platform.<BR>
	* It is simlpy a thin visible line created using b2EdgeShape in Box2D.<BR>
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
    ///<B><HR>Surface to support the plank</B>
	/*!
	* Since, the trapezium plank is a movable/dynamic object, it needs a surface to support it. <BR>
	* This support is provided to it by this static thin surface. Without this horizontal surface,<BR>
	* the trapezium plank will fall down under the influence of gravity.<BR>
	* Same Specifications as bottom ground, except for position (From (31.0, 30.0) to (41.0, 30.0))<BR>
	*/
    b2Body* b10;
    {
      b2EdgeShape shape;
      shape.Set(b2Vec2(31.0f, 30.0f), b2Vec2(41.0f, 30.0f));
	
      b2BodyDef bd;
      b10 = m_world->CreateBody(&bd);
      b10->CreateFixture(&shape, 0.0f);
    }
      
    //Ground to support the sphere connected to the plank
    ///<B><HR>Ground to support the sphere connected to the plank</B>
	/*!
	* The trapezium plank is a movable object connected to another sphere which is also movable.<BR>
	* Since this is a dynamic object it needs a horizontal support which is provided by this ground.<BR>
	* Small in length, Edge Shaped, Static Body, No density<BR>
	*/
    b2Body* b11;
    {
      b2EdgeShape shape;
      shape.Set(b2Vec2(-0.6f, 25.0f), b2Vec2(1.0f, 25.0f));
	
      b2BodyDef bd;
      b11 = m_world->CreateBody(&bd);
      b11->CreateFixture(&shape, 0.0f);
    }

    //Ground to keep sphere next to series of pendulams.
    ///<B><HR>Ground to keep sphere next to series of pendulams.</B>
    /*!
     * The sphere placed next to the series of pendulums is also a dynamic object <BR>
     * which needs a horizontal surface to support it. This is provided by this surface. <BR>
     * Length 10 units, Edge Shaped, No density<BR>
     */
    b2Body* b12;
    {
      b2EdgeShape shape;
      shape.Set(b2Vec2(-2.2f, 30.0f), b2Vec2(9.0f, 30.0f));
	
      b2BodyDef bd;
      b12 = m_world->CreateBody(&bd);
      b12->CreateFixture(&shape, 0.0f);
    }

    //Horizontal shelf on which dominos are kept
    ///<B><HR>Horizontal shelf on which dominos are kept</B>
    /*!
     * This is a horizontal surface which supports the two spheres and the dominos. <BR>
     * Box Shaped, with Small thinkness(0.25 units) and length 16 units, No density<BR>
     */
    {
      b2PolygonShape shape;
      shape.SetAsBox(16.0f, 0.25f);
	
      b2BodyDef bd;
      bd.position.Set(-17.0f, 19.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
	
    //Dominos
    ///<B><HR>Dominos</B>
    /*!
     * Dominos are a series of vertical bars kept close to each other <BR>
     * which keep on toppling when one of them is hit.<BR>
     * There are 10 of them. Each one is identical and placed 1 m apart.<BR>
     * Density : 2 kg per meter Sq<BR>
     * Friction Coefficient : 0.1<BR>
     * Shape : Box Shaped-Thickness 0.1 metre, 1 metre height<BR>
     */
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

    //The sphere which topples the dominos.
    ///<B><HR>The sphere which topples the dominos.</B>
    /*!
     * This is a movable sphere which is set in motion by a vertical bar <BR>
     * which hits the dominos to topple them.<BR>
     * Shape : Circle of radius 1 meter<BR>
     * Position of COM: (-3.0f, 20.0f)<BR>
     * Density : 0.1 kg per metre Sq<BR>
     * No Friction and Coefficient of restitution = 0<BR>
     */
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
      ballbd.position.Set(-4.0f, 20.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }

    //The sphere which falls on the see-saw system.
    ///<B><HR>The sphere which falls on the see-saw system.</B>
    /*!
     * This is also a movable sphere which is set in motion by the toppling <BR>
     * of dominos and is pushed off the surface holding them in to a jar<BR>
     * placed on see-saw table.<BR>
     * Shape : Circle of radius 1 meter<BR>
     * Position of COM: (-22.0f, 20.0f)<BR>
     * Density : 0.5 kg per metre Sq<BR>
     * No Friction and Coefficient of restitution = 0<BR>
     */
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
      


    //The sphere on the plank
    ///<B><HR>The sphere on the plank</B>
    /*!

     * This is a movable object placed on the plank.This is set into motion when <BR>
     * the pendulum hits it and this hits the train of pendulums below.Later, this <BR>
     * falls into the open jar of pulley resulting in the movement of pulley.<BR>
     * Shape : Circle of radius 1 meter<BR>
     * Position of COM: (39.0f, 34.0f)<BR>
     * Density : 2.0 kg per metre Sq<BR>
     * Restitution = 0.0<BR>
     * Friction = 0.3<BR>
     */
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
    ///<B><HR>The sphere next to series of pendulams</B>
    /*!
     * This is also a movable object.This is set into motion when the last pendulum of <BR>
     * the train of pendulums hits this. This falls off the surface into the jar on the <BR>
     * see-saw which sets the see-saw into motion.<BR>

     * Shape : Circle of radius 1 meter<BR>
     * Position of COM: (8.2f, 31.0f)<BR>
     * Density : 1.0 kg per metre Sq<BR>
     * Restitution = 1.0<BR>
     * Friction = 0.0<BR>
     */
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
     {
      //The Trapezium Plank
	  ///<B><HR>The Trapezium Plank</B>
      /*!
       * This is a movable trapezium shaped light block which is connected to a sphere <BR>
       * that is placed on another horizontal surface and it temporarily <BR>
       * supports a sphere which is initially hit by a pendulum.<BR>
       * Shape : Polygon (lower edge length :15, upper edge length :10, Height 5)<BR>
       * Position of COM: (31.0f, 30.0f)<BR>
       * Density : 0.5 kg per metre Sq<BR>
       * No Friction and Coefficient of restitution = 0<BR>
       */
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
      ///<B><HR>The sphere attached to the plank</B>
      /*!
       * The trapezium plank is a movable object connected to another sphere which is also movable.<BR>
       * Motion of this sphere influences that of trapezium plank. <BR>
       * Both the objects are connected by a distance joint<BR>
       * Shape : Circle of radius 1 meter<BR>
       * Position of COM: (-0.2f, 26.0f)<BR>
       * Density : 1.0 kg per metre Sq<BR>
       * No Friction and Coefficient of restitution = 0<BR>
       */
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
      //Joint Between Plank and the sphere
      ///<B><HR>Joint Between Plank and the sphere</B>
      /*!
       * Both the objects are connected by a distance joint which says that the distance <BR>
       * between two points on two bodies must be constant.<BR>
       * Anchor on plank : (31.0f, 30.0f)<BR>
       * Anchor on sphere : (-0.2f, 26.0f)<BR>
       */
	b2DistanceJointDef jointDef;
	b2Vec2 anchorA;
	anchorA.Set(31.0f, 30.0f);
	b2Vec2 anchorB;
	anchorB.Set(-0.2f, 26.0f);
	jointDef.Initialize(plankBody, sphereBody, anchorA, anchorB);
	m_world->CreateJoint(&jointDef);

    }


    //The pendulum that knocks the ball off.
    ///<B><HR>The pendulum that knocks the ball off.</B>
    /*!
     * Initially this sphere is placed in its extreme postion .As soon as the <BR>
     * simulation starts,this sphere oscilllates and knocks the ball on the trapezium plank which <BR>
     * goes and hits the pendulums below.<BR>
     * It is made up of two bodies. The pendulam bob and a support from which it is hanged<BR>
     * 1. The Pendulam Bob <BR>
     * Shape : Circle of radius 1 meter<BR>
     * Initial Position of COM: (45.0f, 37.0f)<BR>
     * Density : 2.0 kg per metre Sq<BR>
     * 2. Upper support<BR>
     * Shape : Box Shape (1.5f, 0.25f)<BR>
     * Initial Position of COM: (45.0f, 37.0f)<BR>
     * Density : Nil, Static Body<BR>
     */
    {
      b2Body* b2;
      {
	b2PolygonShape shape;
	shape.SetAsBox(1.5f, 0.25f);
	  
	b2BodyDef bd;
	bd.position.Set(42.0f, 43.0f);
	b2 = m_world->CreateBody(&bd);
	b2->CreateFixture(&shape, 0.0f);
      }
	
      b2Body* b4;
      {
        b2CircleShape circle;
        circle.m_radius = 1.0;
	  
	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position.Set(45.0f, 37.0f);
	b4 = m_world->CreateBody(&bd);
        b2FixtureDef bobfd;
        bobfd.shape = &circle;
        bobfd.density = 2.0f;
        bobfd.restitution = 1.0f;
	b4->CreateFixture(&bobfd);
      }
	
      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(42.0f, 43.0f);
      jd.Initialize(b2, b4, anchor);
      m_world->CreateJoint(&jd);
    }
 
     
    //The train of pendulams
    ///<B><HR>The train of pendulams</B>
    /*!
     * This is a series of pendulums which is set into motion by the <BR>
     * sphere which is hit by the first pendulum.The last one of this <BR>
     * system hits the sphere placed next to it to set it into motion.<BR>
     * Same as the above pendulam, creating 8 copies of same pendulam  <BR>
     * and positioning them appropriately.<BR>
     */
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
        circle.m_radius = 1.0f;
	  
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


    //The upper revolving bar
    ///<B><HR>The upper revolving bar</B>
    /*!
     * This is a vertical bar which is hinged at its centre so that it <BR>
     * revolves about its centre.This is set into motion by the sphere which <BR>
     * is hit by train of pendulums.This revolving bar hits another revolving bar <BR>
     * present below it and the sphere adjacent to its lower end.<BR>
     * It is formed by a revolute joint between a bar and a point.<BR>
     * Anchors on both bodies are at their centres i. e. (-2.0f, 28.0f)<BR>
     */
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 2.5f);
	
      b2BodyDef bd;
      bd.position.Set(-3.9f, 28.5f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.f;
      fd3->friction = 0.f;
      fd3->restitution = 1.0f;
      fd3->shape = new b2CircleShape;
      b2CircleShape bs3;
      bs3.m_radius=0.8f;
      bs3.m_p.Set(0.0f, 2.5f);
      fd3->shape = &bs3;
      b2FixtureDef *fd4 = new b2FixtureDef;
      fd4->density = 0.f;
      fd4->friction = 0.f;
      fd4->restitution = 0.0f;
      fd4->shape = new b2CircleShape;
      b2CircleShape bs4;
      bs4.m_radius=0.8f;
      bs4.m_p.Set(0.0f, -2.5f);
      fd4->shape = &bs4;
      body->CreateFixture(fd);
      body->CreateFixture(fd3);
      body->CreateFixture(fd4);


      b2BodyDef bd2;
      bd2.position.Set(-3.9f, 28.5f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.lowerAngle = -0.1f; // -90 degrees
      jointDef.upperAngle = 0.1f; // 45 degrees
      //jointDef.enableLimit = true;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    }


    //The lower revolving bar
    ///<B><HR>The lower revolving bar</B>
    /*!
     * This is also a vertical bar which revolves about its centre which is <BR>
     * set into motion by the upper revoving bar,which inturn hits the <BR>
     * sphere which is placed adjacent to its lower end and sets it into motion.<BR>
     * Same as the above bar and positioned at (-1.8f, 23.0f) <BR>
     */
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 2.9f);
	
      b2BodyDef bd;
      bd.position.Set(-2.2f, 23.1f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.f;
      fd3->friction = 0.f;
      fd3->restitution = 1.0f;
      fd3->shape = new b2CircleShape;
      b2CircleShape bs3;
      bs3.m_radius=0.8f;
      bs3.m_p.Set(0.0f, 3.0f);
      fd3->shape = &bs3;
      b2FixtureDef *fd4 = new b2FixtureDef;
      fd4->density = 0.f;
      fd4->friction = 0.f;
      fd4->restitution = 1.0f;
      fd4->shape = new b2CircleShape;
      b2CircleShape bs4;
      bs4.m_radius=0.8f;
      bs4.m_p.Set(0.0f, -3.0f);
      fd4->shape = &bs4;
      body->CreateFixture(fd);
      body->CreateFixture(fd3);
      body->CreateFixture(fd4);

      b2BodyDef bd2;
      bd2.position.Set(-2.2f, 23.1f);
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
    ///<B><HR>The pulley system</B>
    /*!
     * This contains two ends .One is an open box and the other the bar.<BR>
     * When one side oves down , the other side moves up.<BR>
     */
    {

      //The open box
      ///<B><HR>The open box</B>
      /*!
       * This is the right end of the pulley system.The sphere which is <BR>
       * pushed by the topmost pendulum falls into this when the trapezium<BR>
       * platform moves.This moves down which makes the bar on the other side move up.<BR>
       * The box is made by joining 3 bars at the edges. Each bar has a density of 10 <BR>
	* Kg Per Metre Squared, Friction as 0.5 and restitutuion 0.<BR>
       */
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(25,15);
      bd->fixedRotation = true;
      
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
      ///<B><HR>The bar</B>
      /*!
       * This is the left end of the pulley. This is moved up when the sphere<BR>
       * falls into the open box which is the right end of pulley.This moves <BR>
       * to its topmost point and hits the revolving horizontal platform.<BR>
       * It is box shaped with very small thickness and high density. <BR>
       */
      bd->position.Set(10,15);	
      fd1->density = 34.0;	  
      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(fd1);

      // The pulley joint 
      ///<B><HR> The pulley joint </B>
      /*!
       * Pulley joint  connects the horizontal bar and the openbox to ground and to each other.<BR>
       * As one body goes up the goes down and total length of the pulley rope is conserved.<BR>
       * the ration is 1 . Therefore,length1 +length2=constant.<BR>
       */
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(25, 15); /// Anchor point on open box in world axis - (25, 15)
      b2Vec2 worldAnchorOnBody2(10, 15); /// Anchor point on bar in world axis - (10, 15)
      b2Vec2 worldAnchorGround1(25, 20); /// Anchor point for open box in world axis - (25, 20)
      b2Vec2 worldAnchorGround2(10, 20); /// Anchor point for bar in world axis - (10, 20)
      float32 ratio = 1.0f;
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);
    }


    //The revolving horizontal platform
    ///<B><HR>The revolving horizontal platform</B>
    /*!
     * This is a platform placed horizontally which is hinged at its centre and revolves about it.<BR>
     * This has a sphere placed on it which falls off this when the left end of the pulley <BR>
     * system hits the right end of platform.<BR>
     * It is created the same way as two vertical bars above.<BR>
     */
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
    ///<B><HR>The heavy sphere on the revolving platform</B>
    /*!
     * This is a movable object placed on a revolving horizontal platform. This falls off <BR>
     * the horizontal revolving surface into the jar below when the surface is hit by the <BR>
     * left end of the pulley.<BR>
     * Shape : Circle<BR>
     * Density : 50 Kg Per Metre Sq<BR>
     * Friction : 0<BR>
     * Restitution : 0<BR>
     */
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
      ballbd.position.Set(6.0f, 19.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }
	/*{

	b2BodyDef bd;
	b2Body* ground = m_world->CreateBody(&bd);

	float32 x1 = 0.0f;
	float32 y1 = 10.0f;

	for (float32 i = 0; i < b2_pi; i+=0.1f)
		{

		float32 x2 = sinf(i);
		float32 y2 = 10.0f + cosf(i);
	
		b2EdgeShape shape;
		shape.Set(b2Vec2(x1, y1), b2Vec2(x2, y2));
		ground->CreateFixture(&shape, 0.0f);

		x1 = x2;
		y1 = y2;

		}
	}*/
    //The right see-saw system
    ///<B><HR> The right see-saw system</B>
    /*!
     * This has a heavy open jar on its left side and therefore this end is grounded.<BR>
     */
    {
      //The triangle wedge
      ///<B><HR>The triangle wedge</B>
      /*!
       * This is a supporting non-movable hinge which helps see-saw's motion about it.<BR>
       * Static object, created by polygon shape with 3 vertices.<BR>
       * Density = 10.0<BR>
       * Friction = 0.0<BR>
       * Restitution = 0.0<BR>
       */
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[3];
      vertices[0].Set(-1,0);
      vertices[1].Set(1,0);
      vertices[2].Set(0,2.0);
      poly.Set(vertices, 3);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(-14.0f, 0.0f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);

      //The plank on top of the wedge
      ///<B><HR>The plank on top of the wedge</B>
      /*!
       * This is a revolving platform whose centre is at the upper edge of wedge. It supports the openbox and<BR>
       * helps in the sliding of the openbox which is intially on the left <BR>
       * end of the see-saw system.This rotates about the hinge which is <BR>
       * provied by the triangular wedge.<BR>
       * Friction is kept to be nil at this plank to ensure smooth sliding of the open box.<BR>
       */
      b2PolygonShape shape;
      shape.SetAsBox(7.0f, 0.2f);
      b2BodyDef bd2;
      bd2.position.Set(-14.0f, 2.2f);
      bd2.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd2);
      b2FixtureDef *fd0 = new b2FixtureDef;
      fd0->density = 0.5f;
      fd0->shape = new b2PolygonShape;
      fd0->shape = &shape;
      fd0->friction = 0.0f;
      fd0->restitution = 0.0f;
      body->CreateFixture(fd0);

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-14.0f, 2.2f);
      jd.Initialize(sbody, body, anchor);
      m_world->CreateJoint(&jd);

      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-20.0f, 2.5f);
      bd->fixedRotation = false;
      
      //The open box
      ///<B><HR>The open box</B>
      /*!
       * This is a movable box placed on the left end of the see-saw system.This can<BR>
       * slide on the plank of the see-saw system.Finally this box catches the sphere.<BR>
       * It is made in the same way by combining 3 vertical plates and slight modifications <BR>
       * in fixture properties.<BR>
       */
       
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 0.3;
      fd1->friction = 0.3;
      fd1->restitution = 0.0f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 0.3;
      fd2->friction = 1.0;
      fd2->restitution = 0.0f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.1;
      fd3->friction = 1.5;
      fd3->restitution = 0.0f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
      fd3->shape = &bs3;
       
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);
    }

    //The left see-saw system
    ///<B><HR>The left see-saw system</B>
    /*!
     * This is also the same as right see-saw, except for fixture and position.<BR>
     * It has a open box attached to it to its right, which catches the sphere moved by the dominos.<BR><HR><BR>
     */
    {
      //The triangle wedge
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[3];
      vertices[0].Set(-1,0);
      vertices[1].Set(1,0);
      vertices[2].Set(0,1.5);
      poly.Set(vertices, 3);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(-30.5f, 0.0f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);

      //The plank on top of the wedge
      b2PolygonShape shape;
      shape.SetAsBox(10.0f, 0.2f);
      b2BodyDef bd2;
      bd2.position.Set(-30.5f, 1.5f);
      bd2.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd2);
      b2FixtureDef *fd0 = new b2FixtureDef;
      fd0->density = 1.f;
      fd0->shape = new b2PolygonShape;
      fd0->shape = &shape;
      fd0->friction = 10.0f;
      body->CreateFixture(fd0);

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-30.5f, 1.5f);
      jd.Initialize(sbody, body, anchor);
      m_world->CreateJoint(&jd);

      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-37.5f, 2.5f);
      bd->fixedRotation = false;

      //The open box
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 0.1;
      fd1->friction = 1.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 0.1;
      fd2->friction = 1.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.1;
      fd3->friction = 1.5;
      fd3->restitution = 0.0f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
      fd3->shape = &bs3;
       
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);
    }
  }
  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}

