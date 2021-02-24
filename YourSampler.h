#ifndef _YOURSAMPLER_H_
#define _YOURSAMPLER_H_


#include <rl/plan/GaussianSampler.h>
#include <random>

namespace rl
{
    namespace plan
    {
        /**
         * Uniform random sampling strategy.
         */
        class YourSampler : public GaussianSampler
        {
        public:
            YourSampler();

            virtual ~YourSampler();

            ::rl::math::Vector generate();

            virtual void seed(const ::std::mt19937::result_type& value);

        protected:
            ::std::uniform_real_distribution< ::rl::math::Real>::result_type rand();

            ::std::uniform_real_distribution< ::rl::math::Real> randDistribution;

            ::std::mt19937 randEngine;

        private:

        };
    }
}


#endif // _YOURSAMPLER_H_
