#define BOOST_TEST_MODULE "FCL_CONTINUOUS_COLLISION"
#include <boost/test/unit_test.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/filesystem.hpp>
#include <boost/math/constants/constants.hpp>

#include <cmath>

#include "fcl/ccd/conservative_advancement.h"
#include "fcl/ccd/motion.h"
#include "fcl/collision_node.h"
#include "fcl/collision.h"
#include "fcl/BV/BV.h"

#include "test_fcl_utility.h"
#include "fcl_resources/config.h"

#include "fcl/math/matrix_3f.h"
#include "fcl/collision_node.h"
#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"

namespace fcl {

class ContinuousCollisionFixture
{
public:
	ContinuousCollisionFixture() :
		models_min_distance_(1.0e-6)
	{
		init();
	}

	~ContinuousCollisionFixture()
	{

	}

	bool testCollision()
	{
		ContinuousCollisionData collision_data;	
		collision_data.request.num_max_contacts = 1;

		return defaultContinuousCollisionFunction(static_ccd_object_.get(), moving_ccd_object_.get(), &collision_data);
	}

	void setModelsMinDistance(FCL_REAL distance)
	{
		models_min_distance_ = distance;
		
		initInterpMotion();
		initContinuousCollisionObjects();
	}

	FCL_REAL getModelsMinDistance() const
	{
		return models_min_distance_;
	}

	void setCommonTolerance(FCL_REAL tolerance)
	{
		box_static_->setTolerance(tolerance);
		box_moving_->setTolerance(tolerance);
	}

	void setStaticBoxTolerance(FCL_REAL tolerance)
	{
		box_static_->setTolerance(tolerance);		
	}

	void setMovingBoxTolerance(FCL_REAL tolerance)
	{
		box_moving_->setTolerance(tolerance);
	}

private:
	void init()
	{
		initModels();
		initStaticMotion();
		initInterpMotion();		
		initContinuousCollisionObjects();
	}

	void initStaticMotion()
	{
		// no transformation
		tf_static_ = Transform3f(Matrix3f::getIdentity(), Vec3f(0, 0, 0) );
		
		// no movement, just stays on the place
		static_motion_.reset(new InterpMotion(tf_static_, tf_static_) );
	}

	void initInterpMotion()
	{
		// start + end transformation
		tf_interpolation_start_ = Transform3f(
			Matrix3f::getIdentity(), 
			Vec3f(-10, box_side_ + models_min_distance_, 0)
		);
		tf_interpolation_end_ = Transform3f(
			Matrix3f::getIdentity(),
			Vec3f(10, box_side_ + models_min_distance_, 0)
		);

		// move from the start to the end with linear interpolation
		interp_motion_.reset(new InterpMotion(tf_interpolation_start_, tf_interpolation_end_) );
	}

	void initModels()
	{
		std::vector<Vec3f> p;
		std::vector<Triangle> t;	
		boost::filesystem::path path(TEST_RESOURCES_DIR);

		loadOBJFile((path / "box.obj").string().c_str(), p, t);	
		box_side_ = 1.0;

		SplitMethodType split_method = SPLIT_METHOD_MEAN;

		// prepares box for static ccd object
		box_static_.reset(new BVHModel<RSS>() );
		box_static_->bv_splitter.reset(new BVSplitter<RSS>(split_method));

		box_static_->beginModel();
		box_static_->addSubModel(p, t);
		box_static_->endModel();

		// prepares box for moving ccd object
		box_moving_.reset(new BVHModel<RSS>() );
		box_moving_->bv_splitter.reset(new BVSplitter<RSS>(split_method));

		box_moving_->beginModel();
		box_moving_->addSubModel(p, t);
		box_moving_->endModel();
	}

	void initContinuousCollisionObjects()
	{
		static_ccd_object_.reset(new ContinuousCollisionObject(box_static_, static_motion_) );
		moving_ccd_object_.reset(new ContinuousCollisionObject(box_moving_, interp_motion_) );
	}

protected:
	Transform3f tf_static_;
	Transform3f tf_interpolation_start_;
	Transform3f tf_interpolation_end_;

	boost::shared_ptr<InterpMotion> static_motion_;
	boost::shared_ptr<InterpMotion> interp_motion_;

	boost::shared_ptr<BVHModel<RSS> > box_static_;
	boost::shared_ptr<BVHModel<RSS> > box_moving_;
	FCL_REAL box_side_;

	boost::shared_ptr<ContinuousCollisionObject> static_ccd_object_;
	boost::shared_ptr<ContinuousCollisionObject> moving_ccd_object_;

	FCL_REAL models_min_distance_;
};

BOOST_AUTO_TEST_SUITE(test_continuous_collision)

BOOST_FIXTURE_TEST_CASE(test_tolerance, ContinuousCollisionFixture)
{
	setModelsMinDistance(1.0);
	setCommonTolerance(0.4);	
	BOOST_CHECK_EQUAL(false, testCollision() );

	setCommonTolerance(0.5);	
	BOOST_CHECK_EQUAL(true, testCollision() );

	setCommonTolerance(0.6);	
	BOOST_CHECK_EQUAL(true, testCollision() );

	// tries much smaller distances
	setModelsMinDistance(1.0e-3);
	setCommonTolerance(0.4e-3);	
	BOOST_CHECK_EQUAL(false, testCollision() );

	setCommonTolerance(0.5e-3);	
	BOOST_CHECK_EQUAL(true, testCollision() );

	setCommonTolerance(0.6e-3);	
	BOOST_CHECK_EQUAL(true, testCollision() );
}

BOOST_FIXTURE_TEST_CASE(test_tolerance_separately, ContinuousCollisionFixture)
{
	setModelsMinDistance(1.0);
	setStaticBoxTolerance(0.1);	
	setMovingBoxTolerance(0.8);
	BOOST_CHECK_EQUAL(false, testCollision() );

	setStaticBoxTolerance(0.2);	
	setMovingBoxTolerance(0.9);
	BOOST_CHECK_EQUAL(true, testCollision() );

	setStaticBoxTolerance(0.8);	
	setMovingBoxTolerance(0.6);
	BOOST_CHECK_EQUAL(true, testCollision() );

	// tries much smaller distances
	setModelsMinDistance(1.0e-3);
	setStaticBoxTolerance(0.1e-3);	
	setMovingBoxTolerance(0.8e-3);
	BOOST_CHECK_EQUAL(false, testCollision() );

	setStaticBoxTolerance(0.2e-3);	
	setMovingBoxTolerance(0.9e-3);
	BOOST_CHECK_EQUAL(true, testCollision() );

	setStaticBoxTolerance(0.8e-3);	
	setMovingBoxTolerance(0.6e-3);
	BOOST_CHECK_EQUAL(true, testCollision() );
}

BOOST_AUTO_TEST_SUITE_END()

}