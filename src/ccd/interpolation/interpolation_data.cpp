#include "fcl/ccd/interpolation/interpolation_data.h"

namespace fcl 
{

InterpolationData::InterpolationData()
{}


InterpolationLinearData::InterpolationLinearData()
{}

InterpolationType InterpolationLinearData::getType() const
{
	return LINEAR;
}

InterpolationThirdOrderData::InterpolationThirdOrderData(
	const FCL_REAL& velocity, const FCL_REAL& acceleration, const FCL_REAL& jerk) :
	velocity_(velocity),
	acceleration_(acceleration),
	jerk_(jerk)
{}

InterpolationType InterpolationThirdOrderData::getType() const
{
	return THIRD_ORDER;
}

void InterpolationThirdOrderData::setMaxVelocity(const FCL_REAL& velocity)
{
	velocity_ = velocity;
}

const FCL_REAL& InterpolationThirdOrderData::getMaxVelocity() const
{
	return velocity_;
}

void InterpolationThirdOrderData::setMaxAcceleration(const FCL_REAL& acceleration)
{
	acceleration_ = acceleration;
}

const FCL_REAL& InterpolationThirdOrderData::getMaxAcceleration() const
{
	return acceleration_;
}

void InterpolationThirdOrderData::setMaxJerk(const FCL_REAL& jerk)
{
	jerk_ = jerk;
}

const FCL_REAL& InterpolationThirdOrderData::getMaxJerk() const
{
	return jerk_;
}

}