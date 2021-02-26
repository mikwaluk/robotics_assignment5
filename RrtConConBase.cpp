//
// Copyright (c) 2009, Markus Rickert
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include "RrtConConBase.h"
#include <rl/plan/Sampler.h>
#include <rl/plan/SimpleModel.h>
#include <rl/plan/Verifier.h>
#include <rl/plan/Viewer.h>
#include <boost/make_shared.hpp>
#include <iostream>

RrtConConBase::RrtConConBase() :
  Planner(),
  delta(1.0f),
  epsilon(1.0e-3f),
  sampler(NULL),
  begin(2),
  end(2),
  tree(2)
{
}

RrtConConBase::~RrtConConBase()
{
}

RrtConConBase::Edge
RrtConConBase::addEdge(const Vertex& u, const Vertex& v, Tree& tree)
{
  Edge e = ::boost::add_edge(u, v, tree).first;

  if (NULL != this->viewer)
  {
    this->viewer->drawConfigurationEdge(*tree[u].q, *tree[v].q);
  }

  return e;
}
RrtConConBase::Vertex
RrtConConBase::addVertex(Tree& tree, const ::rl::plan::VectorPtr& q)
{
  Vertex v = ::boost::add_vertex(tree);
  tree[v].index = ::boost::num_vertices(tree) - 1;
  tree[v].q = q;
  tree[v].exploration_attempts = 0;
  tree[v].exploration_directions = {};
  tree[v].successful_directions = {};

  if (NULL != this->viewer)
  {
    this->viewer->drawConfigurationVertex(*tree[v].q);
  }

  return v;
}

bool
RrtConConBase::areEqual(const ::rl::math::Vector& lhs, const ::rl::math::Vector& rhs) const
{
  if (this->model->distance(lhs, rhs) > this->epsilon)
  {
    return false;
  }
  else
  {
    return true;
  }
}

void
RrtConConBase::choose(::rl::math::Vector& chosen)
{
  chosen = this->sampler->generate();
}

RrtConConBase::Vertex
RrtConConBase::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  //Do first extend step
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

  // move "last" along the line q<->chosen by distance "step / distance"
  this->model->interpolate(*tree[nearest.first].q, chosen, step / distance, *last);
  tree[nearest.first].exploration_attempts++;
  tree[nearest.first].exploration_directions.emplace_back(chosen - *tree[nearest.first].q);
  this->model->setPosition(*last);
  this->model->updateFrames();

  if (this->model->isColliding())
  {
    tree[nearest.first].successful_directions.push_back(false);
    return NULL;
  }
  tree[nearest.first].successful_directions.push_back(true);
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
RrtConConBase::connect(Tree& tree, const rl::math::Vector& direction, VertexBundle& chosen_bundle)
{
  //Do first extend step
  ::rl::math::Real distance = M_PI / 20;
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

  // move "last" along the line q<->chosen by distance "step / distance"
  this->model->interpolate(*(chosen_bundle.q), *(chosen_bundle.q) + direction, step / distance, *last);
  chosen_bundle.exploration_attempts++;
  chosen_bundle.exploration_directions.emplace_back(direction);
  this->model->setPosition(*last);
  this->model->updateFrames();

  if (this->model->isColliding())
  {
    chosen_bundle.successful_directions.push_back(false);
    return NULL;
  }
  chosen_bundle.successful_directions.push_back(true);
  ::rl::math::Vector next(this->model->getDof());

  while (!reached)
  {
    //Do further extend step

    distance = this->model->distance(*last, *chosen_bundle.q + direction);
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
    this->model->interpolate(*last, *chosen_bundle.q + direction, step / distance, next);

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
  this->addEdge(tree.vertex_by_property(chosen_bundle).value(), connected, tree);
  return connected;
}
RrtConConBase::Vertex
RrtConConBase::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  ::rl::math::Real distance = nearest.second;
  ::rl::math::Real step = (::std::min)(distance, this->delta);

  ::rl::plan::VectorPtr next = ::std::make_shared< ::rl::math::Vector >(this->model->getDof());

  this->model->interpolate(*tree[nearest.first].q, chosen, step / distance, *next);

  this->model->setPosition(*next);
  this->model->updateFrames();

  if (!this->model->isColliding())
  {
    Vertex extended = this->addVertex(tree, next);
    this->addEdge(nearest.first, extended, tree);
    return extended;
  }

  return NULL;
}

::std::string
RrtConConBase::getName() const
{
  return "RrtConConBase";
}

::std::size_t
RrtConConBase::getNumEdges() const
{
  ::std::size_t edges = 0;

  for (::std::size_t i = 0; i < this->tree.size(); ++i)
  {
    edges += ::boost::num_edges(this->tree[i]);
  }

  return edges;
}

::std::size_t
RrtConConBase::getNumVertices() const
{
  ::std::size_t vertices = 0;

  for (::std::size_t i = 0; i < this->tree.size(); ++i)
  {
    vertices += ::boost::num_vertices(this->tree[i]);
  }

  return vertices;
}

rl::plan::VectorList
RrtConConBase::getPath()
{

  rl::plan::VectorList path;
  Vertex i = this->end[0];

  while (i != this->begin[0])
  {
    path.push_front(*this->tree[0][i].q);
    i = ::boost::source(*::boost::in_edges(i, this->tree[0]).first, this->tree[0]);
  }

  path.push_front(*this->tree[0][i].q);

  i = ::boost::source(*::boost::in_edges(this->end[1], this->tree[1]).first, this->tree[1]);

  while (i != this->begin[1])
  {
    path.push_back(*this->tree[1][i].q);
    i = ::boost::source(*::boost::in_edges(i, this->tree[1]).first, this->tree[1]);
  }

  path.push_back(*this->tree[1][i].q);

  return path;
}

RrtConConBase::Neighbor
RrtConConBase::nearest(const Tree& tree, const ::rl::math::Vector& chosen)
{
  //create an empty pair <Vertex, distance> to return
  Neighbor p(Vertex(), (::std::numeric_limits< ::rl::math::Real >::max)());

  //Iterate through all vertices to find the nearest neighbour
  for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first)
  {
    ::rl::math::Real d = this->model->transformedDistance(chosen, *tree[*i.first].q);

    if (d < p.second)
    {
      p.first = *i.first;
      p.second = d;
    }
  }


  // Compute the square root of distance
  p.second = this->model->inverseOfTransformedDistance(p.second);

  return p;
}

void
RrtConConBase::reset()
{
  for (::std::size_t i = 0; i < this->tree.size(); ++i)
  {
    this->tree[i].clear();
    this->begin[i] = NULL;
    this->end[i] = NULL;
  }
}

bool
RrtConConBase::solve()
{

  this->time = ::std::chrono::steady_clock::now();
  // Define the roots of both trees
  this->begin[0] = this->addVertex(this->tree[0], ::std::make_shared< ::rl::math::Vector >(*this->start));
  this->begin[1] = this->addVertex(this->tree[1], ::std::make_shared< ::rl::math::Vector >(*this->goal));

  Tree* a = &this->tree[0];
  Tree* b = &this->tree[1];

  ::rl::math::Vector chosen(this->model->getDof());
  VertexBundle* chosen_bundle;


  while ((::std::chrono::steady_clock::now() - this->time) < this->duration)
  {
    //First grow tree a and then try to connect b.
    //then swap roles: first grow tree b and connect to a.
    for (::std::size_t j = 0; j < 2; ++j)
    {
      if (::boost::num_vertices(*a) < 1000)
      {
        //Sample a random configuration
        this->choose(chosen);
      }
      else
      {
        printf("EGG\n");
        int min_attempts = std::numeric_limits<int>::max();
        chosen_bundle = &(*a)[a->m_vertices.back()];
        // Pick a node with least exploration_attempts
        for (VertexIteratorPair i = ::boost::vertices(*a); i.first != i.second; ++i.first)
        {
          if ((*a)[*i.first].exploration_attempts < min_attempts)
          {
            min_attempts = (*a)[*i.first].exploration_attempts;
            *chosen_bundle = (*a)[*i.first];
          }
        }
        chosen = *(chosen_bundle->q);
        ::rl::math::Vector chosen_direction;
        rl::math::Vector new_exploration_direction(this->model->getDof());
        if (chosen_bundle->exploration_attempts != 0)
        {
          ::rl::math::Real best_direction_utility = std::numeric_limits<rl::math::Real>::min();

          // Choose the best direction from 10 randoms
          ::std::vector<::rl::math::Vector> random_directions {};
          for (int i = 0; i < 10; i++)
          {
            // TODO generate unit vectors
            random_directions.emplace_back(this->sampler->generate());
          }
          for (const auto random_direction : random_directions)
          {
            // Calculate utility for every direction
            rl::math::Real direction_utility = 0;

            for (int i = 0; i < chosen_bundle->exploration_attempts; i++)
            {
              rl::math::Real success = 1.0 ? chosen_bundle->successful_directions[i] : 0.5;
              direction_utility += -success * random_direction.dot(chosen_bundle->exploration_directions[i]);
            }
            if (direction_utility > best_direction_utility)
            {
              chosen_direction = random_direction;
              direction_utility = best_direction_utility;
            }
          }
        }
        else
        {
          new_exploration_direction = this->sampler->generate();
        }

        //std::cout << new_exploration_direction << "\n";
        *(chosen_bundle->q) = new_exploration_direction;
        std::cout << *(chosen_bundle->q) << "\n";
        chosen_bundle->exploration_directions.emplace_back(new_exploration_direction);
        chosen_bundle->exploration_attempts++;

        //Do a CONNECT step from the nearest neighbour to the sample
        Vertex aConnected = this->connect(*a, chosen_direction, *chosen_bundle);

        //If a new node was inserted tree a
        if (NULL != aConnected)
        {
          // Try a CONNECT step form the other tree to the sample
          Vertex bConnected = this->connect(*b, chosen_direction, *chosen_bundle);

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
        continue;
      }

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
    }

  }

  return false;

}
