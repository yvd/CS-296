
/* 
 * Base code for CS 296 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Parag Chaudhuri
 */

#ifndef _DOMINOS_HPP_
#define _DOMINOS_HPP_

namespace cs296
{
  //! This is the class that sets up the Box2D simulation world
  class dominos_t : public base_sim_t
  {
  public:
    
    dominos_t();
    
    static base_sim_t* create()
    {
      return new dominos_t;
    }
  };
}
  
#endif
