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
#include "boost_auto_param_test_case.hpp"

namespace fcl { namespace test { namespace interpolation {

const FCL_REAL EPSILON_PERCENTAGE_MINI = 0.000001;
const FCL_REAL EPSILON_PERCENTAGE_SMALL = 0.00001;

class ThirdOrderInterpolationFixture
{
public:
	ThirdOrderInterpolationFixture()
	{
		init();
	}

protected:
	FCL_REAL prescaleTime(FCL_REAL time)
	{
		BOOST_REQUIRE_LE(time / getTimeScale(), 1.0);

		return time / getTimeScale();
	}

	FCL_REAL postscaleValue(FCL_REAL value)
	{
		return value * getTimeScale();
	}

private:
	void init()
	{
		// the order of init methods matters
		initInterpolationRange();
		initAbsoluteDistance();		
		initInterpolationData();		
		initControlPoints();	
		initInterpolation();
		initTimeScale();
		initDerivation();		
	}

	void initInterpolationRange()
	{
		start_value_ = 0;
		end_value_ = 80;
	}

	void initAbsoluteDistance()
	{
		absoulute_entire_distance_ = fabs(end_value_ - start_value_);
	}

	void initInterpolationData()
	{
		velocity_ = 10;
		acceleration_ = 10;
		jerk_ = 30;

		data_.reset(new InterpolationThirdOrderData(
			velocity_, acceleration_, jerk_) );		
	}

	void initControlPoints()
	{
		control_points_.reset(new ThirdOrderControlPoints(
			data_, absoulute_entire_distance_)
		);
	}

	void initInterpolation()
	{
		third_order_interpolation_.reset(new InterpolationThirdOrder(
			data_, start_value_, end_value_)
		);
		third_order_interpolation_->setMaxTimeScale(
			third_order_interpolation_->getTimeScale()
		);		
	}

	void initTimeScale()
	{
		setTimeScale(third_order_interpolation_->getTimeScale() );
	}

	void setTimeScale(FCL_REAL time_scale)
	{
		time_scale_ = time_scale;
	}

	FCL_REAL getTimeScale() const
	{
		return time_scale_;
	}

	void initDerivation()
	{
		third_order_derivation_.reset(
			new ThirdOrderDerivation(data_, control_points_)
		);
	}

protected:
	boost::shared_ptr<ThirdOrderControlPoints> control_points_;
	boost::shared_ptr<InterpolationThirdOrderData> data_;
	boost::shared_ptr<InterpolationThirdOrder> third_order_interpolation_;
	boost::shared_ptr<ThirdOrderDerivation> third_order_derivation_;

	FCL_REAL start_value_;
	FCL_REAL end_value_;

	FCL_REAL velocity_;
	FCL_REAL acceleration_;
	FCL_REAL jerk_;	

	FCL_REAL absoulute_entire_distance_;

	FCL_REAL time_scale_;
};

////////////////////////////////////////////////////////////////////////////////
FCL_REAL middle(FCL_REAL first, FCL_REAL second)
{
	return first + (second - first) / 2.0;
}

void equalWithEpsilon_9(FCL_REAL first, FCL_REAL second)
{
	static FCL_REAL epsilon = 0.000000001; // 9 decimal places

	BOOST_CHECK_SMALL(first - second, epsilon);
}

void equalWithEpsilon_8(FCL_REAL first, FCL_REAL second)
{
	static FCL_REAL epsilon = 0.00000001; // 8 decimal places

	BOOST_CHECK_SMALL(first - second, epsilon);
}
////////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_SUITE(test_third_order_control_points)

BOOST_FIXTURE_TEST_CASE(test_get_time_upper_bound, 
	ThirdOrderInterpolationFixture)
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

std::pair<FCL_REAL, FCL_REAL> time_value_pairs[] = 
{ 
	//std::make_pair(0.0093426760, 0.0000040774), // unnecessary inaccurate
	std::make_pair(0.1588254922, 0.0200322916),
	std::make_pair(0.2429095762, 0.0716644733),
	std::make_pair(0.3083083083, 0.1465297097),
	std::make_pair(0.3269936603, 0.1748187468),
	std::make_pair(0.3363363363, 0.1902352803),	
	std::make_pair(0.3456790123, 0.2065233958),
	std::make_pair(0.3923923924, 0.3010568126),
	std::make_pair(0.4204204204, 0.3682511340),
	std::make_pair(0.4297630964, 0.3923949531),
	std::make_pair(0.4391057724, 0.4174116281),
	std::make_pair(0.4484484484, 0.4433011590),
	std::make_pair(0.7660994328, 1.8428945017),
	std::make_pair(0.9903236570, 3.4383504848),
	std::make_pair(1.0090090090, 3.5939957489),
	std::make_pair(1.1024357691, 4.4192410246),
	std::make_pair(1.3266599933, 6.5999347525),
	std::make_pair(1.3360026693, 6.6933600267),
	std::make_pair(1.3640306974, 6.9736403070),
	std::make_pair(3.3353353353, 26.6866866867),
	std::make_pair(7.9973306640, 73.3066399733),
	std::make_pair(8.0066733400, 73.4000652475),
	std::make_pair(8.1094427761, 74.4212067191),
	std::make_pair(8.3243243243, 76.4060042511),
	std::make_pair(8.3336670003, 76.4842614831),
	std::make_pair(8.3710377044, 76.7885764755),
	std::make_pair(8.7540874208, 79.1025955329),
	std::make_pair(8.9969969970, 79.8097647197),
	std::make_pair(9.0063396730, 79.8251812532),
	std::make_pair(9.1464798131, 79.9673807586),
	std::make_pair(9.2772772773, 79.9991192805),
	std::make_pair(9.3239906573, 79.9999959226),
	std::make_pair(9.3333333333333339, 80.0)
};

BOOST_FIXTURE_PARAM_TEST_CASE(test_getValue, ThirdOrderInterpolationFixture, 
	std::begin(time_value_pairs), std::end(time_value_pairs) )
{
	FCL_REAL time = param.first;
	FCL_REAL value = param.second;

	BOOST_TEST_MESSAGE(
		"Testing <time, value>(" << time << ", " << value << ")"
	);

	BOOST_CHECK_CLOSE(
		value,
		third_order_interpolation_->getValue(prescaleTime(time) ),
		EPSILON_PERCENTAGE_MINI
	);
}

std::pair<FCL_REAL, FCL_REAL> time_velocity_bound_pairs[] = 
{ 
	std::make_pair(0.0, 10),

	std::make_pair(0.0093426760, 10),
	std::make_pair(0.0653987321, 10),
	std::make_pair(0.1214547881, 10),
	std::make_pair(0.3269936603, 10),
	std::make_pair(1.0183516850, 10),
	std::make_pair(1.3360026693, 10),
	std::make_pair(1.3453453453, 10),
	std::make_pair(4.3163163163, 10),
	std::make_pair(5, 10),

	std::make_pair(8.0066733400, 9.9993319980),
	std::make_pair(8.0160160160, 9.9961523085),
	std::make_pair(8.3243243243, 8.4222059898),
	std::make_pair(8.3336670003, 8.3299966633),
	std::make_pair(8.5578912246, 6.0877544211),
	std::make_pair(8.9969969970, 1.6966966967),
	std::make_pair(9.0063396730, 1.6038728084),
	std::make_pair(9.0156823490, 1.5135322176),
	std::make_pair(9.1558224892, 0.4726514970),
	std::make_pair(9.3239906573, 0.0013092839),

	std::make_pair(9.3333333333333339, 0)
};

BOOST_FIXTURE_PARAM_TEST_CASE(test_get_velocity_bound, 
	ThirdOrderInterpolationFixture, std::begin(time_velocity_bound_pairs), 
	std::end(time_velocity_bound_pairs) )
{
	FCL_REAL time = param.first;
	FCL_REAL velocity_bound = param.second;

	BOOST_TEST_MESSAGE(
		"Testing <time, velocity bound>("
		<< time << ", " << velocity_bound << ")"
	);

	BOOST_CHECK_CLOSE(
		postscaleValue(velocity_bound), 
		third_order_interpolation_->getVelocityBound(prescaleTime(time) ), 
		EPSILON_PERCENTAGE_SMALL
	);
}

BOOST_FIXTURE_PARAM_TEST_CASE(test_get_velocity_bound_on_time_interval, 
	ThirdOrderInterpolationFixture, std::begin(time_velocity_bound_pairs), 
	std::end(time_velocity_bound_pairs) )
{
	FCL_REAL time = param.first;
	FCL_REAL velocity_bound = param.second;

	BOOST_TEST_MESSAGE(
		"Testing <time, velocity bound>("
		<< time << ", " << velocity_bound << ")"
	);

	FCL_REAL whole_time_prescaled = prescaleTime(
		third_order_interpolation_->getTimeScale() );

	BOOST_CHECK_CLOSE(
		postscaleValue(velocity_bound), 
		third_order_interpolation_->getVelocityBound(
			prescaleTime(time), whole_time_prescaled), 
		EPSILON_PERCENTAGE_SMALL
	);
}

BOOST_FIXTURE_TEST_CASE(test_get_velocity_bound_on_time_interval_other_cases, 
	ThirdOrderInterpolationFixture)
{
	FCL_REAL whole_time_prescaled = 
		prescaleTime(third_order_interpolation_->getTimeScale() );

	equalWithEpsilon_8(
		postscaleValue(9.9993319980), 
		third_order_interpolation_->getVelocityBound(prescaleTime(0.0093426760), 
		whole_time_prescaled - prescaleTime(8.0066733400) ) 
	);
	equalWithEpsilon_8(postscaleValue(9.9961523085), 
		third_order_interpolation_->getVelocityBound(prescaleTime(0.0653987321),
		whole_time_prescaled - prescaleTime(8.0160160160) ) 
	);
	equalWithEpsilon_8(postscaleValue(8.4222059898), 
		third_order_interpolation_->getVelocityBound(prescaleTime(0.1214547881),
		whole_time_prescaled - prescaleTime(8.3243243243) ) 
	);
	equalWithEpsilon_8(postscaleValue(8.3299966633), 
		third_order_interpolation_->getVelocityBound(prescaleTime(0.3269936603),
		whole_time_prescaled - prescaleTime(8.3336670003) ) 
	);
	equalWithEpsilon_8(postscaleValue(9.9993319980), 
		third_order_interpolation_->getVelocityBound(prescaleTime(1.0183516850),
		whole_time_prescaled - prescaleTime(8.0066733400) ) 
	);
}

BOOST_AUTO_TEST_SUITE_END()
////////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_SUITE(test_third_order_derivation)

std::pair<FCL_REAL, FCL_REAL> time_derivation_pairs[] = 
{ 
	std::make_pair(0.0, 0.0),

	std::make_pair(0.0093426760, 0.0013092839),
	std::make_pair(0.0653987321, 0.0641549123),
	std::make_pair(0.1214547881, 0.2212689834),
	std::make_pair(0.3269936603, 1.6038728084),
	std::make_pair(0.3363363363, 1.6966966967),
	std::make_pair(0.3456790123, 1.7901234568),
	std::make_pair(0.4017350684, 2.3506840174),
	std::make_pair(0.9996663330, 8.3299966633),
	std::make_pair(1.0090090090, 8.4222059898),
	std::make_pair(1.0183516850, 8.5117984184),
	std::make_pair(1.1584918252, 9.5414567053),
	std::make_pair(1.3266599933, 9.9993319980),
	std::make_pair(1.3360026693, 10),
	std::make_pair(1.3453453453, 10),
	std::make_pair(4.3163163163, 10),
	std::make_pair(7.9973306640, 10),
	std::make_pair(8.0066733400, -9.9993319980),
	std::make_pair(8.0160160160, -9.9961523085),
	std::make_pair(8.3243243243, -8.4222059898),
	std::make_pair(8.3336670003, -8.3299966633),
	std::make_pair(8.5578912246, -6.0877544211),
	std::make_pair(8.9969969970, -1.6966966967),
	std::make_pair(9.0063396730, -1.6038728084),
	std::make_pair(9.0156823490, -1.5135322176),
	std::make_pair(9.1558224892, -0.4726514970),
	std::make_pair(9.3239906573, -0.0013092839),	

	std::make_pair(9.3333333333333339, 0)
};

BOOST_FIXTURE_PARAM_TEST_CASE(test_get_derivation, 
	ThirdOrderInterpolationFixture, std::begin(time_derivation_pairs), 
	std::end(time_derivation_pairs) )
{
	FCL_REAL time = param.first;
	FCL_REAL derivation = param.second;

	BOOST_TEST_MESSAGE(
		"Testing <time, derivation>(" << time << ", " << derivation << ")"
	);

	BOOST_CHECK_CLOSE(
		derivation, 
		third_order_derivation_->getDerivation(time), 
		EPSILON_PERCENTAGE_SMALL
	);
}

std::pair<FCL_REAL, FCL_REAL> time_absolute_max_derivation_pairs[] = 
{ 
	std::make_pair(0.0, 10.0),

	std::make_pair(0.0093426760, 10),
	std::make_pair(0.0653987321, 10),
	std::make_pair(0.1214547881, 10),
	std::make_pair(0.3269936603, 10),
	std::make_pair(1.0183516850, 10),
	std::make_pair(1.3360026693, 10),
	std::make_pair(1.3453453453, 10),
	std::make_pair(4.3163163163, 10),
	std::make_pair(5, 10),
	std::make_pair(8.0066733400, 9.9993319980),
	std::make_pair(8.0160160160, 9.9961523085),
	std::make_pair(8.3243243243, 8.4222059898),
	std::make_pair(8.3336670003, 8.3299966633),
	std::make_pair(8.5578912246, 6.0877544211),
	std::make_pair(8.9969969970, 1.6966966967),
	std::make_pair(9.0063396730, 1.6038728084),
	std::make_pair(9.0156823490, 1.5135322176),
	std::make_pair(9.1558224892, 0.4726514970),
	std::make_pair(9.3239906573, 0.0013092839),	

	std::make_pair(9.3333333333333339, 0)
};

BOOST_FIXTURE_PARAM_TEST_CASE(test_get_absolute_max_derivation, 
	ThirdOrderInterpolationFixture,
	std::begin(time_absolute_max_derivation_pairs), 
	std::end(time_absolute_max_derivation_pairs) )
{
	FCL_REAL time = param.first;
	FCL_REAL absolue_max_derivation = param.second;

	BOOST_TEST_MESSAGE(
		"Testing <time, absolue max derivation>(" 
		<< time << ", " << absolue_max_derivation << ")"
	);

	BOOST_CHECK_CLOSE(
		absolue_max_derivation, 
		third_order_derivation_->getAbsoluteMaxDerivation(time), 
		EPSILON_PERCENTAGE_SMALL
	);
}

BOOST_FIXTURE_PARAM_TEST_CASE(test_get_absolute_max_derivation_on_time_interval, 
	ThirdOrderInterpolationFixture, std::begin(time_absolute_max_derivation_pairs), 
	std::end(time_absolute_max_derivation_pairs) )
{
	FCL_REAL time = param.first;
	FCL_REAL absolute_max_derivation_on_time_interval = param.second;

	BOOST_TEST_MESSAGE(
		"Testing <time, absolute max derivation on time interval>("
		<< time << ", " << absolute_max_derivation_on_time_interval << ")"
	);

	FCL_REAL whole_time = third_order_interpolation_->getTimeScale();

	BOOST_CHECK_CLOSE(
		absolute_max_derivation_on_time_interval, 
		third_order_derivation_->getAbsoluteMaxDerivation(time, whole_time), 
		EPSILON_PERCENTAGE_SMALL
	);
}

BOOST_FIXTURE_TEST_CASE(
	test_get_absolute_max_derivation_on_time_interval_other_cases,
	ThirdOrderInterpolationFixture)
{
	FCL_REAL whole_time = third_order_interpolation_->getTimeScale();

	equalWithEpsilon_9(9.9993319980, third_order_derivation_->getAbsoluteMaxDerivation(0.0093426760, whole_time - 8.0066733400) );
	equalWithEpsilon_9(9.9961523085, third_order_derivation_->getAbsoluteMaxDerivation(0.0653987321, whole_time - 8.0160160160) );
	equalWithEpsilon_9(8.4222059898, third_order_derivation_->getAbsoluteMaxDerivation(0.1214547881, whole_time - 8.3243243243) );
	equalWithEpsilon_9(8.3299966633, third_order_derivation_->getAbsoluteMaxDerivation(0.3269936603, whole_time - 8.3336670003) );
	equalWithEpsilon_9(9.9993319980, third_order_derivation_->getAbsoluteMaxDerivation(1.0183516850, whole_time - 8.0066733400) );

	equalWithEpsilon_9(9.9993319980, third_order_derivation_->getAbsoluteMaxDerivation(8.0066733400, 9.0) );
	equalWithEpsilon_9(9.9961523085, third_order_derivation_->getAbsoluteMaxDerivation(8.0160160160, 9.0) );
	equalWithEpsilon_9(8.4222059898, third_order_derivation_->getAbsoluteMaxDerivation(8.3243243243, 9.0) );
	equalWithEpsilon_9(8.3299966633, third_order_derivation_->getAbsoluteMaxDerivation(8.3336670003, 9.0) );
	equalWithEpsilon_9(6.0877544211, third_order_derivation_->getAbsoluteMaxDerivation(8.5578912246, 9.0) );
	equalWithEpsilon_9(1.6966966967, third_order_derivation_->getAbsoluteMaxDerivation(8.9969969970, 9.0) );
	equalWithEpsilon_9(1.6038728084, third_order_derivation_->getAbsoluteMaxDerivation(9.0063396730, 9.5) );
	equalWithEpsilon_9(1.5135322176, third_order_derivation_->getAbsoluteMaxDerivation(9.0156823490, 9.5) );
	equalWithEpsilon_9(0.4726514970, third_order_derivation_->getAbsoluteMaxDerivation(9.1558224892, 9.5) );
	equalWithEpsilon_9(0.0013092839, third_order_derivation_->getAbsoluteMaxDerivation(9.3239906573, 9.5) );
}

BOOST_AUTO_TEST_SUITE_END()
////////////////////////////////////////////////////////////////////////////////

} } }