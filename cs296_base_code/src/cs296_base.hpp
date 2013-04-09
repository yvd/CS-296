
/* 
 * Base code for CS 296 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Parag Chaudhuri
 * Group : 26
 * Mridul Ravi Jain	(110040083)
 * Vamsidhar Yeddu      (110050051)
 * Sachin Chandra Bonagiri  (110050065) 
 */


#ifndef _CS296BASE_HPP_
#define _CS296BASE_HPP_

#include "render.hpp"
#include <Box2D/Box2D.h>
#include <cstdlib>

#define	RAND_LIMIT 32767

namespace cs296
{

  //! Creates a base_sim_t class & setting_t struct.
  //! In class data members and member functions are private by default. In structure members are public by default.
  class base_sim_t;
  struct settings_t;
  
  //! Typedef is used to create our own types based on existing data types
  typedef base_sim_t* sim_create_fcn(); 

  //! Simulation settings. Some can be controlled in the GUI.
  struct settings_t
  {
    //! Notice the initialization of the class members in the constructor
    //! Constructors can initialize their members by passing values as aruments in constructor definition

    /*! \brief settings_t

      This structure instantiates various default settings at the start of simulation such as setting up iteration count and frequncy of 
stepping.

      Deatails:
      
      view_center(0.0f, 20.0f)	\n Construct A 2D column vector named view_center using two float32 coordinates\n \n 
      hz(60.0f),		\n Set a time step of 1/60 seconds\n \n 
      velocity_iterations(8)	\n Set iteration count of 8 for velocity which computes the impulses necessary for the bodies to move 
				correctly\n \n 
      position_iterations(3) \n Set iteration count of 3 for position which adjusts the positions of the bodies to reduce overlap and 
				joint detachment\n \n 
      draw_shapes(1)		\n This enable the drawing of shapes\n \n 
      draw_joints(1)		\n This enables the drawing of joints\n \n 
      draw_AABBs(0)		\n Disable axis-aligned bounding box\n \n 
      draw_pairs(0)		\n Set No of collisuion pairs to 0\n \n 
      draw_contact_points(0)	\n This disables the drawing of contact points\n \n 
      draw_contact_normals(0)	\n This disables the drawing of contact normals\n \n 
      draw_contact_forces(0)	\n This disables action of contact forces\n \n 
      draw_friction_forces(0)	\n This disables action of friction points\n \n 
      draw_COMs(0)		\n This is to disable drawing of center of mass\n \n 
      enable_warm_starting(1)	\n Enable warm starting:using information generated in the previous simulation timestep use in the current timestep\n \n 
      enable_continuous(1)	\n Enable continuous simulation\n \n 
      enable_sub_stepping(0)	\n Disable step-wise(sub step) simulation\n \n 
      pause(0)			\n Disable Pause button\n \n 
      single_step(0)		\n Disable Single Step\n \n 
      */

    settings_t() :
      view_center(0.0f, 20.0f),
      hz(60.0f),
      velocity_iterations(8),
      position_iterations(3),
      draw_shapes(1),
      draw_joints(1),
      draw_AABBs(0),
      draw_pairs(0),
      draw_contact_points(0),
      draw_contact_normals(0),
      draw_contact_forces(0),
      draw_friction_forces(0),
      draw_COMs(0),
      draw_stats(0),
      draw_profile(0),
      enable_warm_starting(1),
      enable_continuous(1),
      enable_sub_stepping(0),
      pause(0),
      single_step(0)
    {}
    
    b2Vec2 view_center;
    float32 hz;
    int32 velocity_iterations;
    int32 position_iterations;
    int32 draw_shapes;
    int32 draw_joints;
    int32 draw_AABBs;
    int32 draw_pairs;
    int32 draw_contact_points;
    int32 draw_contact_normals;
    int32 draw_contact_forces;
    int32 draw_friction_forces;
    int32 draw_COMs;
    int32 draw_stats;
    int32 draw_profile;
    int32 enable_warm_starting;
    int32 enable_continuous;
    int32 enable_sub_stepping;
    int32 pause;
    int32 single_step;
  };
  
  struct sim_t
  {
      /*! \brief sim_t

      This structure contains a character pointer variable and a function to start the simulation.
      
      */

    const char *name;
    sim_create_fcn *create_fcn;

    sim_t(const char *_name, sim_create_fcn *_create_fcn): 
      name(_name), create_fcn(_create_fcn) {;}
  };
  
  extern sim_t *sim;
  
  
  const int32 k_max_contact_points = 2048;  
  struct contact_point_t
  {
      /*! \brief contact_point_t

      This structure holds the properties of a contact point t

      Details:
      
      b2Fixture* fixtureA 	\n Holds the fixture of first body\n \n 
      b2Fixture* fixtureB	\n Holds the fixture of second body\n \n 
      b2Vec2 normal		\n Normal vector at contact point \n \n 
      b2Vec2 position		\n Position vector at contact point\n \n 
      b2PointState state	\n Holds the state of a contact point\n \n 
      */
    b2Fixture* fixtureA;
    b2Fixture* fixtureB;
    b2Vec2 normal;
    b2Vec2 position;
    b2PointState state;
  };
  
  class base_sim_t : public b2ContactListener
  {      /*! \brief base_sim_t

      This class inhrits the b2ContactListener class and handles the user actions on the interface.

      Details:

      \n It contains virtual destructors which are called in order and helps in simulation. It also contains callbacks for derived classes. \n \n
      
    
      */
  public:
    
    base_sim_t();

    //! Virtual destructors are useful when you can delete an instance of a derived class through a pointer to base class.
    virtual ~base_sim_t();
    
    void set_text_line(int32 line) { m_text_line = line; }
    void draw_title(int x, int y, const char *string);
    
    virtual void step(settings_t* settings);

    virtual void keyboard(unsigned char key) { B2_NOT_USED(key); }
    virtual void keyboard_up(unsigned char key) { B2_NOT_USED(key); }

    void shift_mouse_down(const b2Vec2& p) { B2_NOT_USED(p); }
    virtual void mouse_down(const b2Vec2& p) { B2_NOT_USED(p); }
    virtual void mouse_up(const b2Vec2& p) { B2_NOT_USED(p); }
    void mouse_move(const b2Vec2& p) { B2_NOT_USED(p); }

    
    // Let derived tests know that a joint was destroyed.
    virtual void joint_destroyed(b2Joint* joint) { B2_NOT_USED(joint); }
    
    // Callbacks for derived classes.
    virtual void begin_contact(b2Contact* contact) { B2_NOT_USED(contact); }
    virtual void end_contact(b2Contact* contact) { B2_NOT_USED(contact); }
    virtual void pre_solve(b2Contact* contact, const b2Manifold* oldManifold);
    virtual void post_solve(const b2Contact* contact, const b2ContactImpulse* impulse)
    {
      B2_NOT_USED(contact);
      B2_NOT_USED(impulse);
    }

   //!Private members are only accessible within the class defining them.

//!Protected members are accessible in the class that defines them and in classes that inherit from that class.
//!A friend class can access the private and protected members of the class in which it is declared as a friend.
//!A friend function is a function that is not a member of a class but has access to the class's private and protected members. 
//!Friend functions are not considered class members; they are normal external functions that are given special access privileges.
//!A friend function is declared by the class that is granting access. 
//!The friend declaration can be placed anywhere in the class declaration. 
//!It is not affected by the access control keywords.
  protected:


    friend class contact_listener_t;
    
    b2Body* m_ground_body;
    b2AABB m_world_AABB;
    contact_point_t m_points[k_max_contact_points];
    int32 m_point_count;

    debug_draw_t m_debug_draw;
    int32 m_text_line;
    b2World* m_world;

    int32 m_step_count;
    
    b2Profile m_max_profile;
    b2Profile m_total_profile;
  };
}

#endif
