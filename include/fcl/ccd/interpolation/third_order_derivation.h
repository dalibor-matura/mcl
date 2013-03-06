#ifndef FCL_CCD_INTERPOLATION_THIRD_ORDER_DERIVATION_H
#define FCL_CCD_INTERPOLATION_THIRD_ORDER_DERIVATION_H

#include "fcl/data_types.h"

#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

namespace fcl {

class ThirdOrderControlPoints;
class InterpolationThirdOrderData;

class ThirdOrderDerivation
{
public:
	ThirdOrderDerivation(const boost::shared_ptr<const InterpolationThirdOrderData>& data,
		const boost::shared_ptr<const ThirdOrderControlPoints>& control_points);

	FCL_REAL getDerivation(FCL_REAL time) const;

	FCL_REAL getAbsoluteMaxDerivation(FCL_REAL start_time) const;
	FCL_REAL getAbsoluteMaxDerivation(FCL_REAL start_time, FCL_REAL end_time) const;
private:
	void init();

	std::size_t getTimeUpperBound(const FCL_REAL time) const;

	void initDerivationCalculation();


	FCL_REAL getDerivation_t0_t0(FCL_REAL time) const;
	// first jerk
	FCL_REAL getDerivation_t0_t1(FCL_REAL time) const;
	// acceleration
	FCL_REAL getDerivation_t1_t2(FCL_REAL time) const;
	// second jerk
	FCL_REAL getDerivation_t2_t3(FCL_REAL time) const;
	// max velocity
	FCL_REAL getDerivation_t3_t4(FCL_REAL time) const;

	FCL_REAL getMirrorDerivation(FCL_REAL time) const;

private:
	const boost::shared_ptr<const ThirdOrderControlPoints> points_;
	const boost::shared_ptr<const InterpolationThirdOrderData> data_;

	std::vector<boost::function<FCL_REAL(FCL_REAL)> > derivation_functions_;
};

}

#endif