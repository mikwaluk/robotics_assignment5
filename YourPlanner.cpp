#include "YourPlanner.h"
#include <rl/plan/SimpleModel.h>
#include <iostream>

YourPlanner::YourPlanner() :
  RrtConConBase()
{
}

YourPlanner::~YourPlanner()
{
}

::std::string
YourPlanner::getName() const
{
  return "RrtConConBase with deactivating exhausted nodes min_taken=100 threshold=99percent";
}

void
YourPlanner::choose(::rl::math::Vector& chosen,  const ::rl::math::Vector* current_goal, const double goal_bias)
{

  this->model->getDof();
  RrtConConBase::choose(chosen);
  return;
  //your modifications here
  double r = static_cast<double>(rand()) / (RAND_MAX);
  if (r < goal_bias)
  {
    chosen = (*current_goal);
    return;
  }
  RrtConConBase::choose(chosen);
}

RrtConConBase::Vertex
YourPlanner::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  //your modifications here
  ::rl::math::Real distance = nearest.second;
  ::rl::math::Real step = distance;

  bool reached = false;

  if (step <= this->delta)
  {
    reached = true;
  }
  else
  {
    step = this->delta;
  }

  ::rl::plan::VectorPtr last = ::std::make_shared< ::rl::math::Vector >(this->model->getDof());
  if (static_cast<double>(tree[nearest.first].failed) / tree[nearest.first].taken > 0.99 && tree[nearest.first].taken > 100)
  {
    //std::cout << "XTaken: " << tree[nearest.first].taken << "Failed " << tree[nearest.first].failed << "\n";
    return NULL;
  }
  tree[nearest.first].taken++;
  //std::cout << "OKTaken: " << tree[nearest.first].taken << "Failed " << tree[nearest.first].failed << "\n";

  // move "last" along the line q<->chosen by distance "step / distance"
  this->model->interpolate(*tree[nearest.first].q, chosen, step / distance, *last);

  this->model->setPosition(*last);
  this->model->updateFrames();

  if (this->model->isColliding())
  {
    tree[nearest.first].failed++;
    return NULL;
  }

  ::rl::math::Vector next(this->model->getDof());

  while (!reached)
  {
    //Do further extend step

    distance = this->model->distance(*last, chosen);
    step = distance;

    if (step <= this->delta)
    {
      reached = true;
    }
    else
    {
      step = this->delta;
    }

    // move "next" along the line last<->chosen by distance "step / distance"
    this->model->interpolate(*last, chosen, step / distance, next);

    this->model->setPosition(next);
    this->model->updateFrames();

    if (this->model->isColliding())
    {
      break;
    }

    *last = next;
  }

  // "last" now points to the vertex where the connect step collided with the environment.
  // Add it to the tree
  Vertex connected = this->addVertex(tree, last);
  this->addEdge(nearest.first, connected, tree);
  return connected;
}

RrtConConBase::Vertex
YourPlanner::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  //your modifications here
  return RrtConConBase::extend(tree, nearest, chosen);
}

bool
YourPlanner::solve()
{
  this->time = ::std::chrono::steady_clock::now();
  // Define the roots of both trees
  this->begin[0] = this->addVertex(this->tree[0], ::std::make_shared< ::rl::math::Vector >(*this->start));
  this->begin[1] = this->addVertex(this->tree[1], ::std::make_shared< ::rl::math::Vector >(*this->goal));

  Tree* a = &this->tree[0];
  Tree* b = &this->tree[1];

  ::rl::math::Vector chosen(this->model->getDof());


  while ((::std::chrono::steady_clock::now() - this->time) < this->duration)
  {
    //First grow tree a and then try to connect b.
    //then swap roles: first grow tree b and connect to a.
    bool swapped = false;
    for (::std::size_t j = 0; j < 2; ++j)
    {
      const ::rl::math::Vector* current_goal;
      if (!swapped)
      {
        current_goal = this->goal;
      }
      else
      {
        current_goal = this->start;
      }
      //Sample a random configuration
      this->choose(chosen, current_goal, 0.0);

      //Find the nearest neighbour in the tree
      Neighbor aNearest = this->nearest(*a, chosen);

      //Do a CONNECT step from the nearest neighbour to the sample
      Vertex aConnected = this->connect(*a, aNearest, chosen);

      //If a new node was inserted tree a
      if (NULL != aConnected)
      {
        // Try a CONNECT step form the other tree to the sample
        Neighbor bNearest = this->nearest(*b, *(*a)[aConnected].q);
        Vertex bConnected = this->connect(*b, bNearest, *(*a)[aConnected].q);

        if (NULL != bConnected)
        {
          //Test if we could connect both trees with each other
          if (this->areEqual(*(*a)[aConnected].q, *(*b)[bConnected].q))
          {
            this->end[0] = &this->tree[0] == a ? aConnected : bConnected;
            this->end[1] = &this->tree[1] == b ? bConnected : aConnected;
            return true;
          }
        }
      }

      //Swap the roles of a and b
      using ::std::swap;
      swap(a, b);
      swapped = true;
    }

  }

  return false;
}

