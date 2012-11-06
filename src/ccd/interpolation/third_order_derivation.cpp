#include "fcl/ccd/interpolation/third_order_derivation.h"

#include "fcl/ccd/interpolation/interpolation_data.h"
#include "fcl/ccd/interpolation/interpolation_third_order.h"
#include "fcl/ccd/interpolation/third_order_control_points.h"

#include <cmath>

#include <boost/bind.hpp>
#include <boost/assert.hpp>

namespace fcl {

ThirdOrderDerivation::ThirdOrderDerivation(
	const boost::shared_ptr<const InterpolationThirdOrderData>& data,
	const boost::shared_ptr<const ThirdOrderControlPoints>& control_points) :

	data_(data),
	points_(control_points),	
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

FCL_REAL ThirdOrderDerivation::getDerivation_t0_t0(FCL_REAL time) const
{
	return 0.0;
}

// first jerk
FCL_REAL ThirdOrderDerivation::getDerivation_t0_t1(FCL_REAL time) const
{
	/*return data_->getMaxJerk() * 
		(3.0 * pow(time,2) - 6.0 * points_->getTimePoint(0) * time + 
		3.0 * pow(points_->getTimePoint(0),2) ) / 6.0;*/

	return 0.5 * data_->getMaxJerk() * pow(time - points_->getTimePoint(0), 2) +
		points_->getVelocityPoint(0);
}

// acceleration
FCL_REAL ThirdOrderDerivation::getDerivation_t1_t2(FCL_REAL time) const
{
	return data_->getMaxAcceleration() * (time - points_->getTimePoint(1) ) + 
		points_->getVelocityPoint(1);
}

// second jerk
FCL_REAL ThirdOrderDerivation::getDerivation_t2_t3(FCL_REAL time) const
{
	/*return data_->getMaxAcceleration() * (time - points_->getTimePoint(1) ) - 
		data_->getMaxJerk() * (3 * time - 2 * points_->getTimePoint(2) * time +
		pow(points_->getTimePoint(2),2) ) / 3 + 
		data_->getMaxJerk() * pow(points_->getTimePoint(1) - points_->getTimePoint(0),2);*/

	return data_->getMaxAcceleration() * (time - points_->getTimePoint(1) ) - 
		0.5 * data_->getMaxJerk() * pow(time - points_->getTimePoint(2), 2) +
		points_->getVelocityPoint(1);
}

// max velocity
FCL_REAL ThirdOrderDerivation::getDerivation_t3_t4(FCL_REAL time) const
{
	return data_->getMaxVelocity();
}

FCL_REAL ThirdOrderDerivation::getDerivation(FCL_REAL time) const
{
	BOOST_ASSERT_MSG(time >= 0, "Time is negative value.");

	if (time >=  points_->getEntireTime() )
	{
		return 0.0;
	}

	// already slows the velocity till it stops
	if ( time > points_->getTimePoint(4) )
	{
		return getMirrorDerivation(time);
	}

	return derivation_functions_[points_->getTimeUpperBound(time)](time);
}

FCL_REAL ThirdOrderDerivation::getMirrorDerivation(FCL_REAL time) const
{
	time = points_->getEntireTime() - time;

	return -derivation_functions_[points_->getTimeUpperBound(time)](time);
}

FCL_REAL ThirdOrderDerivation::getAbsoluteMaxDerivation(FCL_REAL time) const
{
	FCL_REAL half_of_entire_time = points_->getEntireTime() / 2;

	if (time <= half_of_entire_time)
	{
		return fabs(getDerivation(half_of_entire_time) );
	}

	return fabs(getDerivation(time) );
}

}