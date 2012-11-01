#ifndef FCL_CCD_INTERPOLATION_THIRD_ORDER_DERIVATION_H
#define FCL_CCD_INTERPOLATION_THIRD_ORDER_DERIVATION_H

#include "fcl/data_types.h"
#include "fcl/ccd/interpolation/interpolation_data.h"
#include "fcl/ccd/interpolation/interpolation_third_order.h"

#include <vector>

#include <boost/function.hpp>

namespace fcl {

class ThirdOrderDerivation
{
public:
	ThirdOrderDerivation(const InterpolationThirdOrder& interpolation);

	FCL_REAL getDerivation(FCL_REAL time) const;

	FCL_REAL getAbsoluteMaxDerivative(FCL_REAL time) const;
private:
	void init();

	std::size_t getTimeUpperBound(const FCL_REAL time) const;

	void initDerivationCalculation();

	FCL_REAL getDerivation_t0_t0(FCL_REAL time);
	// first jerk
	FCL_REAL getDerivation_t0_t1(FCL_REAL time);
	// acceleration
	FCL_REAL getDerivation_t1_t2(FCL_REAL time);
	// second jerk
	FCL_REAL getDerivation_t2_t3(FCL_REAL time);
	// max velocity
	FCL_REAL getDerivation_t3_t4(FCL_REAL time);

private:
	const InterpolationThirdOrder& interpolation_;
	const boost::shared_ptr<const InterpolationThirdOrderData>& data_;

	std::vector<boost::function<FCL_REAL(FCL_REAL)> > derivation_functions_;
};

}

#endif