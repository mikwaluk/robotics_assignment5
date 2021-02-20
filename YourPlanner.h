#ifndef _YOUR_PLANNER_H_
#define _YOUR_PLANNER_H_

#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

#include "RrtConConBase.h"
#include <rl/plan/PrmUtilityGuided.h>

using namespace ::rl::plan;

/**
*	The implementation of your planner.
*	modify any of the existing methods to improve planning performance.
*/
class YourPlanner : public PrmUtilityGuided
{
public:
  YourPlanner();

  virtual ~YourPlanner();

  virtual ::std::string getName() const;

  bool solve();

protected:
  //void choose(::rl::math::Vector& chosen,  const ::rl::math::Vector* current_goal, const double goal_bias);

  //PrmUtilityGuided::Vertex connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);

  //PrmUtilityGuided::Vertex extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);

private:

};

#endif // _YOUR_PLANNER_H_
