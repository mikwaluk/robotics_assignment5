#include <chrono>
#include <rl/plan/SimpleModel.h>
#include "YourSampler.h"
#include <iostream>

namespace rl
{
namespace plan
{
YourSampler::YourSampler()
    :
    GaussianSampler(),
    randDistribution(0, 1),
    randEngine(::std::random_device()())
{
}

YourSampler::~YourSampler()
{
}

::rl::math::Vector
YourSampler::generate()
{
  while(true)
  {
    ::rl::math::Vector q1(this->model->getDof());

    // First sample - uniform
    for (::std::size_t i = 0; i < this->model->getDof(); ++i)
    {
      q1(i) = this->rand();
    }
    q1 = this->model->generatePositionUniform(q1);
    this->model->setPosition(q1);
    this->model->updateFrames();
    if (!this->model->isColliding())
    {
      continue;
    }

    // Second sample - gaussian (currently sigma is PI/20 for all samples)
    ::rl::math::Vector q2(this->model->getDof());
    for (::std::size_t i = 0; i < this->model->getDof(); ++i)
    {
      q2(i) = this->gauss() * (*this->sigma)(i) + q1(i);
    }
    this->model->setPosition(q2);
    this->model->updateFrames();
    if (!this->model->isColliding())
    {
      continue;
    }

    // Third sample - between the two
    ::rl::math::Vector q_mid(this->model->getDof());
    for (::std::size_t i = 0; i < this->model->getDof(); ++i)
    {
      q_mid(i) = (q1(i) + q2(i)) / 2;
    }
    this->model->setPosition(q_mid);
    this->model->updateFrames();
    if (!this->model->isColliding())
    {
      return q_mid;
    }
  }
}

::std::uniform_real_distribution<::rl::math::Real>::result_type
YourSampler::rand()
{
  return this->randDistribution(this->randEngine);
}

void
YourSampler::seed(const ::std::mt19937::result_type& value)
{
  this->randEngine.seed(value);
}
}
}

