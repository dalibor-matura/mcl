#define BOOST_TEST_MODULE "FCL_MODEL_BOUND"
#include <boost/test/unit_test.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <cmath>

#include "fcl/ccd/interpolation/interpolation.h"
#include "fcl/ccd/interpolation/interpolation_data.h"
#include "fcl/ccd/interpolation/interpolation_linear.h"
#include "fcl/ccd/interpolation/interpolation_third_order.h"
#include "fcl/ccd/interpolation/third_order_control_points.h"
#include "fcl/ccd/interpolation/third_order_derivation.h"

#include "fcl/data_types.h"

namespace fcl {

class ThirdOrderInterpolationFixture
{
public:
	ThirdOrderInterpolationFixture()
	{
		start_value = 0;
		end_value = 80;

		absoulute_entire_distance_ = fabs(end_value - start_value);

		velocity_ = 10;
		acceleration_ = 10;
		jerk_ = 30;

		data_.reset(new InterpolationThirdOrderData(velocity_, acceleration_, jerk_) );		
		control_points_.reset(new ThirdOrderControlPoints(data_, absoulute_entire_distance_) );

		third_order_interpolation_.reset(new InterpolationThirdOrder(data_, start_value, end_value) );
		time_scale_ = third_order_interpolation_->getTimeScale();
		third_order_interpolation_->setMaxTimeScale(time_scale_);

		third_order_derivation_.reset(new ThirdOrderDerivation(data_, control_points_) );
	}

protected:
	FCL_REAL prescaleTime(FCL_REAL time)
	{
		BOOST_REQUIRE_LE(time / time_scale_, 1.0);

		return time / time_scale_;
	}

protected:
	boost::shared_ptr<ThirdOrderControlPoints> control_points_;
	boost::shared_ptr<InterpolationThirdOrderData> data_;
	boost::shared_ptr<InterpolationThirdOrder> third_order_interpolation_;
	boost::shared_ptr<ThirdOrderDerivation> third_order_derivation_;

	FCL_REAL velocity_;
	FCL_REAL acceleration_;
	FCL_REAL jerk_;

	FCL_REAL start_value;
	FCL_REAL end_value;

	FCL_REAL absoulute_entire_distance_;

	FCL_REAL time_scale_;
};

////////////////////////////////////////////////////////////////////////////////
FCL_REAL middle(FCL_REAL first, FCL_REAL second)
{
	return first + (second - first) / 2.0;
}

void equalWithEpsilon(FCL_REAL first, FCL_REAL second)
{
	static FCL_REAL epsilon = 0.000000001;

	FCL_REAL difference = fabs(second - first);
	BOOST_CHECK_LT(difference, epsilon);
}
////////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_SUITE(test_third_order_control_points)

BOOST_FIXTURE_TEST_CASE(test_get_time_upper_bound, ThirdOrderInterpolationFixture)
{
	FCL_REAL t0 = control_points_->getTimePoint(0);
	FCL_REAL t1 = control_points_->getTimePoint(1);
	FCL_REAL t2 = control_points_->getTimePoint(2);
	FCL_REAL t3 = control_points_->getTimePoint(3);
	FCL_REAL t4 = control_points_->getTimePoint(4);
	FCL_REAL t5 = control_points_->getTimePoint(5);
	FCL_REAL t6 = control_points_->getTimePoint(6);
	FCL_REAL t7 = control_points_->getTimePoint(7);

	BOOST_REQUIRE_LT(t0, t1);
	BOOST_REQUIRE_LT(t1, t2);
	BOOST_REQUIRE_LT(t2, t3);
	BOOST_REQUIRE_LT(t3, t4);
	BOOST_REQUIRE_LT(t4, t5);
	BOOST_REQUIRE_LT(t5, t6);
	BOOST_REQUIRE_LT(t6, t7);

	BOOST_CHECK_EQUAL(0, control_points_->getTimeUpperBound(t0) );

	BOOST_CHECK_EQUAL(1, control_points_->getTimeUpperBound(middle(t0, t1) ) );
	BOOST_CHECK_EQUAL(1, control_points_->getTimeUpperBound(t1) );

	BOOST_CHECK_EQUAL(2, control_points_->getTimeUpperBound(middle(t1, t2) ) );
	BOOST_CHECK_EQUAL(2, control_points_->getTimeUpperBound(t2) );

	BOOST_CHECK_EQUAL(3, control_points_->getTimeUpperBound(middle(t2, t3) ) );
	BOOST_CHECK_EQUAL(3, control_points_->getTimeUpperBound(t3) );

	BOOST_CHECK_EQUAL(4, control_points_->getTimeUpperBound(middle(t3, t4) ) );
	BOOST_CHECK_EQUAL(4, control_points_->getTimeUpperBound(t4) );

	BOOST_CHECK_EQUAL(5, control_points_->getTimeUpperBound(middle(t4, t5) ) );
	BOOST_CHECK_EQUAL(5, control_points_->getTimeUpperBound(t5) );

	BOOST_CHECK_EQUAL(6, control_points_->getTimeUpperBound(middle(t5, t6) ) );
	BOOST_CHECK_EQUAL(6, control_points_->getTimeUpperBound(t6) );

	BOOST_CHECK_EQUAL(7, control_points_->getTimeUpperBound(middle(t6, t7) ) );
	BOOST_CHECK_EQUAL(7, control_points_->getTimeUpperBound(t7) );

	BOOST_CHECK_EQUAL(7, control_points_->getTimeUpperBound(t7 + 1.0) );
}

BOOST_AUTO_TEST_SUITE_END()
////////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_SUITE(test_third_order_interpolation)

BOOST_FIXTURE_TEST_CASE(test_getValue, ThirdOrderInterpolationFixture)
{
	equalWithEpsilon(0.0000040774, third_order_interpolation_->getValue(prescaleTime(0.0093426760) ) );
	equalWithEpsilon(0.0200322916, third_order_interpolation_->getValue(prescaleTime(0.1588254922) ) );
	equalWithEpsilon(0.0716644733, third_order_interpolation_->getValue(prescaleTime(0.2429095762) ) );
	equalWithEpsilon(0.1465297097, third_order_interpolation_->getValue(prescaleTime(0.3083083083) ) );
	equalWithEpsilon(0.1748187468, third_order_interpolation_->getValue(prescaleTime(0.3269936603) ) );
	equalWithEpsilon(0.1902352803, third_order_interpolation_->getValue(prescaleTime(0.3363363363) ) );	
	equalWithEpsilon(0.2065233958, third_order_interpolation_->getValue(prescaleTime(0.3456790123) ) );
	equalWithEpsilon(0.3010568126, third_order_interpolation_->getValue(prescaleTime(0.3923923924) ) );
	equalWithEpsilon(0.3682511340, third_order_interpolation_->getValue(prescaleTime(0.4204204204) ) );
	equalWithEpsilon(0.3923949531, third_order_interpolation_->getValue(prescaleTime(0.4297630964) ) );
	equalWithEpsilon(0.4174116281, third_order_interpolation_->getValue(prescaleTime(0.4391057724) ) );
	equalWithEpsilon(0.4433011590, third_order_interpolation_->getValue(prescaleTime(0.4484484484) ) );
	equalWithEpsilon(1.8428945017, third_order_interpolation_->getValue(prescaleTime(0.7660994328) ) );
	equalWithEpsilon(3.4383504848, third_order_interpolation_->getValue(prescaleTime(0.9903236570) ) );
	equalWithEpsilon(3.5939957489, third_order_interpolation_->getValue(prescaleTime(1.0090090090) ) );
	equalWithEpsilon(4.4192410246, third_order_interpolation_->getValue(prescaleTime(1.1024357691) ) );
	equalWithEpsilon(6.5999347525, third_order_interpolation_->getValue(prescaleTime(1.3266599933) ) );
	equalWithEpsilon(6.6933600267, third_order_interpolation_->getValue(prescaleTime(1.3360026693) ) );
	equalWithEpsilon(6.9736403070, third_order_interpolation_->getValue(prescaleTime(1.3640306974) ) );
	equalWithEpsilon(26.6866866867, third_order_interpolation_->getValue(prescaleTime(3.3353353353) ) );
	equalWithEpsilon(73.3066399733, third_order_interpolation_->getValue(prescaleTime(7.9973306640) ) );
	equalWithEpsilon(73.4000652475, third_order_interpolation_->getValue(prescaleTime(8.0066733400) ) );
	equalWithEpsilon(74.4212067191, third_order_interpolation_->getValue(prescaleTime(8.1094427761) ) );
	equalWithEpsilon(76.4060042511, third_order_interpolation_->getValue(prescaleTime(8.3243243243) ) );
	equalWithEpsilon(76.4842614831, third_order_interpolation_->getValue(prescaleTime(8.3336670003) ) );
	equalWithEpsilon(76.7885764755, third_order_interpolation_->getValue(prescaleTime(8.3710377044) ) );
	equalWithEpsilon(79.1025955329, third_order_interpolation_->getValue(prescaleTime(8.7540874208) ) );
	equalWithEpsilon(79.8097647197, third_order_interpolation_->getValue(prescaleTime(8.9969969970) ) );
	equalWithEpsilon(79.8251812532, third_order_interpolation_->getValue(prescaleTime(9.0063396730) ) );
	equalWithEpsilon(79.9673807586, third_order_interpolation_->getValue(prescaleTime(9.1464798131) ) );
	equalWithEpsilon(79.9991192805, third_order_interpolation_->getValue(prescaleTime(9.2772772773) ) );
	equalWithEpsilon(79.9999959226, third_order_interpolation_->getValue(prescaleTime(9.3239906573) ) );
}

BOOST_FIXTURE_TEST_CASE(test_get_velocity_bound, ThirdOrderInterpolationFixture)
{
	equalWithEpsilon(10, third_order_interpolation_->getVelocityBound(prescaleTime(0) ) );

	equalWithEpsilon(10, third_order_interpolation_->getVelocityBound(prescaleTime(0.0093426760) ) );
	equalWithEpsilon(10, third_order_interpolation_->getVelocityBound(prescaleTime(0.0653987321) ) );
	equalWithEpsilon(10, third_order_interpolation_->getVelocityBound(prescaleTime(0.1214547881) ) );
	equalWithEpsilon(10, third_order_interpolation_->getVelocityBound(prescaleTime(0.3269936603) ) );
	equalWithEpsilon(10, third_order_interpolation_->getVelocityBound(prescaleTime(1.0183516850) ) );
	equalWithEpsilon(10, third_order_interpolation_->getVelocityBound(prescaleTime(1.3360026693) ) );
	equalWithEpsilon(10, third_order_interpolation_->getVelocityBound(prescaleTime(1.3453453453) ) );
	equalWithEpsilon(10, third_order_interpolation_->getVelocityBound(prescaleTime(4.3163163163) ) );
	equalWithEpsilon(10, third_order_interpolation_->getVelocityBound(prescaleTime(5) ) );

	equalWithEpsilon(9.9993319980, third_order_interpolation_->getVelocityBound(prescaleTime(8.0066733400) ) );
	equalWithEpsilon(9.9961523085, third_order_interpolation_->getVelocityBound(prescaleTime(8.0160160160) ) );
	equalWithEpsilon(8.4222059898, third_order_interpolation_->getVelocityBound(prescaleTime(8.3243243243) ) );
	equalWithEpsilon(8.3299966633, third_order_interpolation_->getVelocityBound(prescaleTime(8.3336670003) ) );
	equalWithEpsilon(6.0877544211, third_order_interpolation_->getVelocityBound(prescaleTime(8.5578912246) ) );
	equalWithEpsilon(1.6966966967, third_order_interpolation_->getVelocityBound(prescaleTime(8.9969969970) ) );
	equalWithEpsilon(1.6038728084, third_order_interpolation_->getVelocityBound(prescaleTime(9.0063396730) ) );
	equalWithEpsilon(1.5135322176, third_order_interpolation_->getVelocityBound(prescaleTime(9.0156823490) ) );
	equalWithEpsilon(0.4726514970, third_order_interpolation_->getVelocityBound(prescaleTime(9.1558224892) ) );
	equalWithEpsilon(0.0013092839, third_order_interpolation_->getVelocityBound(prescaleTime(9.3239906573) ) );

	equalWithEpsilon(0, third_order_interpolation_->getVelocityBound(prescaleTime(
		third_order_interpolation_->getTimeScale() ) ) );
}

BOOST_AUTO_TEST_SUITE_END()
////////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_SUITE(test_third_order_derivation)

BOOST_FIXTURE_TEST_CASE(test_get_derivation, ThirdOrderInterpolationFixture)
{
	equalWithEpsilon(0, third_order_derivation_->getDerivation(0) );

	equalWithEpsilon(0.0013092839, third_order_derivation_->getDerivation(0.0093426760) );
	equalWithEpsilon(0.0641549123, third_order_derivation_->getDerivation(0.0653987321) );
	equalWithEpsilon(0.2212689834, third_order_derivation_->getDerivation(0.1214547881) );
	equalWithEpsilon(1.6038728084, third_order_derivation_->getDerivation(0.3269936603) );
	equalWithEpsilon(1.6966966967, third_order_derivation_->getDerivation(0.3363363363) );
	equalWithEpsilon(1.7901234568, third_order_derivation_->getDerivation(0.3456790123) );
	equalWithEpsilon(2.3506840174, third_order_derivation_->getDerivation(0.4017350684) );
	equalWithEpsilon(8.3299966633, third_order_derivation_->getDerivation(0.9996663330) );
	equalWithEpsilon(8.4222059898, third_order_derivation_->getDerivation(1.0090090090) );
	equalWithEpsilon(8.5117984184, third_order_derivation_->getDerivation(1.0183516850) );
	equalWithEpsilon(9.5414567053, third_order_derivation_->getDerivation(1.1584918252) );
	equalWithEpsilon(9.9993319980, third_order_derivation_->getDerivation(1.3266599933) );
	equalWithEpsilon(10, third_order_derivation_->getDerivation(1.3360026693) );
	equalWithEpsilon(10, third_order_derivation_->getDerivation(1.3453453453) );
	equalWithEpsilon(10, third_order_derivation_->getDerivation(4.3163163163) );
	equalWithEpsilon(10, third_order_derivation_->getDerivation(7.9973306640) );
	equalWithEpsilon(-9.9993319980, third_order_derivation_->getDerivation(8.0066733400) );
	equalWithEpsilon(-9.9961523085, third_order_derivation_->getDerivation(8.0160160160) );
	equalWithEpsilon(-8.4222059898, third_order_derivation_->getDerivation(8.3243243243) );
	equalWithEpsilon(-8.3299966633, third_order_derivation_->getDerivation(8.3336670003) );
	equalWithEpsilon(-6.0877544211, third_order_derivation_->getDerivation(8.5578912246) );
	equalWithEpsilon(-1.6966966967, third_order_derivation_->getDerivation(8.9969969970) );
	equalWithEpsilon(-1.6038728084, third_order_derivation_->getDerivation(9.0063396730) );
	equalWithEpsilon(-1.5135322176, third_order_derivation_->getDerivation(9.0156823490) );
	equalWithEpsilon(-0.4726514970, third_order_derivation_->getDerivation(9.1558224892) );
	equalWithEpsilon(-0.0013092839, third_order_derivation_->getDerivation(9.3239906573) );

	equalWithEpsilon(0, third_order_derivation_->getDerivation(third_order_interpolation_->getTimeScale() ) );
}

BOOST_FIXTURE_TEST_CASE(test_get_absolute_max_derivation, ThirdOrderInterpolationFixture)
{
	equalWithEpsilon(10, third_order_derivation_->getAbsoluteMaxDerivation(0) );

	equalWithEpsilon(10, third_order_derivation_->getAbsoluteMaxDerivation(0.0093426760) );
	equalWithEpsilon(10, third_order_derivation_->getAbsoluteMaxDerivation(0.0653987321) );
	equalWithEpsilon(10, third_order_derivation_->getAbsoluteMaxDerivation(0.1214547881) );
	equalWithEpsilon(10, third_order_derivation_->getAbsoluteMaxDerivation(0.3269936603) );
	equalWithEpsilon(10, third_order_derivation_->getAbsoluteMaxDerivation(1.0183516850) );
	equalWithEpsilon(10, third_order_derivation_->getAbsoluteMaxDerivation(1.3360026693) );
	equalWithEpsilon(10, third_order_derivation_->getAbsoluteMaxDerivation(1.3453453453) );
	equalWithEpsilon(10, third_order_derivation_->getAbsoluteMaxDerivation(4.3163163163) );
	equalWithEpsilon(10, third_order_derivation_->getAbsoluteMaxDerivation(5) );

	equalWithEpsilon(9.9993319980, third_order_derivation_->getAbsoluteMaxDerivation(8.0066733400) );
	equalWithEpsilon(9.9961523085, third_order_derivation_->getAbsoluteMaxDerivation(8.0160160160) );
	equalWithEpsilon(8.4222059898, third_order_derivation_->getAbsoluteMaxDerivation(8.3243243243) );
	equalWithEpsilon(8.3299966633, third_order_derivation_->getAbsoluteMaxDerivation(8.3336670003) );
	equalWithEpsilon(6.0877544211, third_order_derivation_->getAbsoluteMaxDerivation(8.5578912246) );
	equalWithEpsilon(1.6966966967, third_order_derivation_->getAbsoluteMaxDerivation(8.9969969970) );
	equalWithEpsilon(1.6038728084, third_order_derivation_->getAbsoluteMaxDerivation(9.0063396730) );
	equalWithEpsilon(1.5135322176, third_order_derivation_->getAbsoluteMaxDerivation(9.0156823490) );
	equalWithEpsilon(0.4726514970, third_order_derivation_->getAbsoluteMaxDerivation(9.1558224892) );
	equalWithEpsilon(0.0013092839, third_order_derivation_->getAbsoluteMaxDerivation(9.3239906573) );

	equalWithEpsilon(0, third_order_derivation_->getAbsoluteMaxDerivation(
		third_order_interpolation_->getTimeScale() ) );
	equalWithEpsilon(0, third_order_derivation_->getAbsoluteMaxDerivation(
		third_order_interpolation_->getTimeScale() + 1 ) );
	equalWithEpsilon(0, third_order_derivation_->getAbsoluteMaxDerivation(
		third_order_interpolation_->getTimeScale() + 10 ) );
}

BOOST_AUTO_TEST_SUITE_END()
////////////////////////////////////////////////////////////////////////////////

}