#ifndef INTERPOLATION_DATA_H
#define INTERPOLATION_DATA_H

#include "fcl/ccd/interpolation/interpolation.h"

#include "fcl/data_types.h"

namespace fcl 
{

class InterpolationData
{
public:
	InterpolationData();

	virtual ~InterpolationData() {}

	virtual InterpolationType getType() const = 0;
};

class InterpolationLinearData :
	public InterpolationData
{
public:
	InterpolationLinearData();

	virtual InterpolationType getType() const;
};

class InterpolationThirdOrderData :
	public InterpolationData
{
public:
	InterpolationThirdOrderData(
		const FCL_REAL& velocity, const FCL_REAL& acceleration, const FCL_REAL& jerk);

	virtual InterpolationType getType() const;

	void setMaxVelocity(const FCL_REAL& velocity);
	const FCL_REAL& getMaxVelocity() const;

	void setMaxAcceleration(const FCL_REAL& acceleration);
	const FCL_REAL& getMaxAcceleration() const;
	
	void setMaxJerk(const FCL_REAL& jerk);
	const FCL_REAL& getMaxJerk() const;

private:
	FCL_REAL velocity_;
	FCL_REAL acceleration_;
	FCL_REAL jerk_;
};

}

#endif