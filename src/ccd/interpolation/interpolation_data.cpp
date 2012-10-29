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

const FCL_REAL& InterpolationThirdOrderData::getVelocity() const
{
	return velocity_;
}

const FCL_REAL& InterpolationThirdOrderData::getAcceleration() const
{
	return acceleration_;
}

const FCL_REAL& InterpolationThirdOrderData::getJerk() const
{
	return jerk_;
}

}