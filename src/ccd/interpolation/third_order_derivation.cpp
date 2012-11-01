#include "fcl/ccd/interpolation/third_order_derivation.h"

#include <cmath>

#include <boost/bind.hpp>
#include <boost/assert.hpp>

namespace fcl {

ThirdOrderDerivation::ThirdOrderDerivation(const InterpolationThirdOrder& interpolation) :
	interpolation_(interpolation),
	data_(interpolation.getData() ),
	derivation_functions_(std::vector<boost::function<FCL_REAL(FCL_REAL)> >(5) )
{
	init();
}

void ThirdOrderDerivation::init()
{
	initDerivationCalculation();
}

void ThirdOrderDerivation::initDerivationCalculation()
{
	derivation_functions_[0] = boost::bind(&ThirdOrderDerivation::getDerivation_t0_t0, this, _1);
	derivation_functions_[1] = boost::bind(&ThirdOrderDerivation::getDerivation_t0_t1, this, _1);
	derivation_functions_[2] = boost::bind(&ThirdOrderDerivation::getDerivation_t1_t2, this, _1);
	derivation_functions_[3] = boost::bind(&ThirdOrderDerivation::getDerivation_t2_t3, this, _1);
	derivation_functions_[4] = boost::bind(&ThirdOrderDerivation::getDerivation_t3_t4, this, _1);
}

FCL_REAL ThirdOrderDerivation::getDerivation_t0_t0(FCL_REAL time)
{
	return 0.0;
}

// first jerk
FCL_REAL ThirdOrderDerivation::getDerivation_t0_t1(FCL_REAL time)
{
	return data_->getMaxJerk() * 
		(3 * pow(time,2) - 6 * interpolation_.getTimePoint(0) * time + 
		3 * pow(interpolation_.getTimePoint(0),2) ) / 6;
}

// acceleration
FCL_REAL ThirdOrderDerivation::getDerivation_t1_t2(FCL_REAL time)
{
	return data_->getMaxAcceleration() * (time - interpolation_.getTimePoint(1) ) + 
		interpolation_.getVelocityPoint(1);
}

// second jerk
FCL_REAL ThirdOrderDerivation::getDerivation_t2_t3(FCL_REAL time)
{
	return data_->getMaxAcceleration() * (time - interpolation_.getTimePoint(1) ) - 
		data_->getMaxJerk() * (3 * time - 2 * interpolation_.getTimePoint(2) * time +
		pow(interpolation_.getTimePoint(2),2) ) / 3 + 
		data_->getMaxJerk() * pow(interpolation_.getTimePoint(1) - interpolation_.getTimePoint(0),2);
}

// max velocity
FCL_REAL ThirdOrderDerivation::getDerivation_t3_t4(FCL_REAL time)
{
	return data_->getMaxVelocity();
}

FCL_REAL ThirdOrderDerivation::getDerivation(FCL_REAL time) const
{
	BOOST_ASSERT_MSG(time >= 0 && time <= interpolation_.getEntireTime(), "Time is not in range.");

	FCL_REAL half_of_entire_time = interpolation_.getEntireTime() / 2;

	if ( time > half_of_entire_time )
	{
		time -= half_of_entire_time;
	}

	return derivation_functions_[interpolation_.getTimeUpperBound(time)](time);
}

FCL_REAL ThirdOrderDerivation::getAbsoluteMaxDerivative(FCL_REAL time) const
{
	FCL_REAL half_of_entire_time = interpolation_.getEntireTime() / 2;

	if (time <= half_of_entire_time)
	{
		return fabs(getDerivation(half_of_entire_time) );
	}

	return fabs(getDerivation(time) );
}

}