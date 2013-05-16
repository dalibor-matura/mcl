#define BOOST_TEST_MODULE "FCL_ARTICULATED_MODEL"
#include <boost/test/unit_test.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/filesystem.hpp>
#include <boost/math/constants/constants.hpp>

#include <cmath>

#include "fcl/ccd/conservative_advancement.h"
#include "fcl/articulated_model/model.h"
#include "fcl/articulated_model/model_config.h"
#include "fcl/ccd/motion.h"
#include "fcl/collision_node.h"
#include "fcl/collision.h"
#include "fcl/BV/BV.h"

#include "test_fcl_utility.h"
#include "fcl_resources/config.h"

namespace fcl {

class ConservativeAdvancementFixture
{
public:
	typedef ConservativeAdvancement<RSS, MeshDistanceTraversalNodeRSS, MeshCollisionTraversalNodeRSS> ConservativeAdvancementType;

public:
	ConservativeAdvancementFixture()
	{
		init();
	}

	~ConservativeAdvancementFixture() {}	

private:
	void init()
	{
		initModels();
		initStaticMotion();
		initInterpMotion();		
		initContinuousCollisionObjects();
		initConservativeAdvancement();
	}

	void initModels()
	{
		box_side_ = 1.0;

		std::vector<Vec3f> points;
		std::vector<Triangle> triangles;	
		boost::filesystem::path path(TEST_RESOURCES_DIR);

		loadOBJFile((path / "box.obj").string().c_str(), points, triangles);	
		
		box_static_ = createBox(points, triangles);
		box_moving_ = createBox(points, triangles);
	}

	boost::shared_ptr<BVHModel<RSS> > createBox(const std::vector<Vec3f>& points, 
		const std::vector<Triangle>& triangles)
	{
		boost::shared_ptr<BVHModel<RSS> > box(new BVHModel<RSS>() );
		box->bv_splitter.reset(new BVSplitter<RSS>(SPLIT_METHOD_MEAN) );

		box->beginModel();
		box->addSubModel(points, triangles);
		box->endModel();

		return box;
	}

	void initStaticMotion()
	{
		tf_static_ = Transform3f(Matrix3f::getIdentity(), Vec3f(0, 0, 0) );

		static_motion_.reset(new InterpMotion(tf_static_, tf_static_) );
	}

	void initInterpMotion()
	{
		// start + end transformation
		Transform3f tf_interpolation_start = Transform3f(
			Matrix3f::getIdentity(), 
			Vec3f(-10, 0, 0)
			);
		Transform3f tf_interpolation_end = Transform3f(
			Matrix3f::getIdentity(),
			Vec3f(10, 0, 0)
			);

		// move from the start to the end with linear interpolation
		interp_motion_collision.reset(new InterpMotion(tf_interpolation_start, tf_interpolation_end) );

		// start + end transformation
		tf_interpolation_start = Transform3f(
			Matrix3f::getIdentity(), 
			Vec3f(-10, box_side_ + 1.0, 0)
			);
		tf_interpolation_end = Transform3f(
			Matrix3f::getIdentity(),
			Vec3f(10, box_side_ + 1.0, 0)
			);

		// move from the start to the end with linear interpolation
		interp_motion_collision_free.reset(new InterpMotion(tf_interpolation_start, tf_interpolation_end) );
	}	

	void initContinuousCollisionObjects()
	{
		static_ccd_object_.reset(new ContinuousCollisionObject(box_static_, static_motion_) );
		collision_ccd_object_.reset(new ContinuousCollisionObject(box_moving_, interp_motion_collision) );
		collision_free_ccd_object_.reset(new ContinuousCollisionObject(box_moving_, interp_motion_collision_free) );
	}

	void initConservativeAdvancement()
	{
		advancement_collision_ = 
			boost::make_shared<ConservativeAdvancement<RSS, MeshDistanceTraversalNodeRSS, MeshCollisionTraversalNodeRSS> >
			(static_ccd_object_.get(), collision_ccd_object_.get() );
		advancement_collision_free_ =
			boost::make_shared<ConservativeAdvancement<RSS,MeshDistanceTraversalNodeRSS, MeshCollisionTraversalNodeRSS> >
			(static_ccd_object_.get(), collision_free_ccd_object_.get() );
	}

protected:
	Transform3f tf_static_;

	boost::shared_ptr<InterpMotion> static_motion_;
	boost::shared_ptr<InterpMotion> interp_motion_collision;
	boost::shared_ptr<InterpMotion> interp_motion_collision_free;

	boost::shared_ptr<BVHModel<RSS> > box_static_;
	boost::shared_ptr<BVHModel<RSS> > box_moving_;
	FCL_REAL box_side_;

	boost::shared_ptr<ContinuousCollisionObject> static_ccd_object_;
	boost::shared_ptr<ContinuousCollisionObject> collision_ccd_object_;
	boost::shared_ptr<ContinuousCollisionObject> collision_free_ccd_object_;

	boost::shared_ptr<ConservativeAdvancementType> 
		advancement_collision_;
	boost::shared_ptr<ConservativeAdvancementType> 
		advancement_collision_free_;
};

////////////////////////////////////////////////////////////////////////////////

bool operator==(const Vec3f& first, const Vec3f& second)
{
	return first[0] == second[0] &&
		first[1] == second[1] &&
		first[2] == second[2];
}

bool operator==(const Matrix3f& first, const Matrix3f& second)
{
	return first.getRow(0) == second.getRow(0) &&
		first.getRow(1) == second.getRow(1) &&
		first.getRow(2) == second.getRow(2);
}

bool operator==(const Quaternion3f& first, const Quaternion3f& second)
{
	return first.getW() == second.getW() &&
		first.getX() == second.getX() &&
		first.getY() == second.getY() &&
		first.getZ() == second.getZ();
}

bool operator==(const Transform3f& first, const Transform3f& second)
{
	bool are_equal = true;

	// there could be a small arithmetical error so check with epsilon
	static const FCL_REAL epsilon = 0.0000001;

	// Needs to use quaternions in order to prevent small arithmetical errors that could (would) cause unsuccessful comparison.
	const Vec3f& first_translation = first.getTranslation();
	const Vec3f& second_translation = second.getTranslation();

	for (int i = 0; i < 3; ++i)
	{
		FCL_REAL difference = std::fabs(first_translation[i] - second_translation[i]);

		are_equal &= difference <= epsilon;
	}	
	
	are_equal &= first.getQuatRotation() == second.getQuatRotation();

	return are_equal;
}

std::ostream& operator << (std::ostream& o, const Quaternion3f& q)
{
	o << "(" << q.getW() << ", " << q.getX() << ", " << q.getY() << ", " << q.getZ() << ")";
	return o;
}

std::ostream& operator << (std::ostream& o, const Transform3f& t)
{
	o << t.getTranslation() << ", " << t.getQuatRotation();
	return o;
}

std::ostream& operator << (std::ostream& o, const ModelConfig& c)
{
	o << "[";

	std::map<std::string, JointConfig> joint_cfgs_map = c.getJointCfgsMap();
	std::map<std::string, JointConfig>::const_iterator it;

	for (it = joint_cfgs_map.begin(); it != joint_cfgs_map.end(); ++it)
	{
		if (it != joint_cfgs_map.begin() )
		{
			o << ", ";
		}

		const JointConfig& joint_cfg = it->second;

		o << "(";

		for (size_t i = 0; i < joint_cfg.getDim(); ++i)
		{
			if (i != 0)
			{
				o << ", ";
			}

			o << joint_cfg[i];
		}

		o << ")";
	}

	o << "]";

	return o;
}

////////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_SUITE(test_conservative_advancement)

BOOST_FIXTURE_TEST_CASE(test_constructors, ConservativeAdvancementFixture)
{
	boost::shared_ptr<ConservativeAdvancementType> advancement = boost::make_shared<ConservativeAdvancementType>(
		static_ccd_object_.get(), collision_ccd_object_.get()
	);

	advancement = boost::make_shared<ConservativeAdvancementType>(
		box_static_.get(), static_motion_.get(),
		box_moving_.get(), interp_motion_collision.get()
	);
}

BOOST_FIXTURE_TEST_CASE(test_collide, ConservativeAdvancementFixture)
{
	ContinuousCollisionRequest request;
	ContinuousCollisionResult result;

	int collisions_number = 0;

	collisions_number = 
		advancement_collision_->collide(request, result);
	BOOST_CHECK_LT(0, collisions_number);

	result.clear();
	collisions_number =
		advancement_collision_free_->collide(request, result);
	BOOST_CHECK_EQUAL(0, collisions_number);
}

BOOST_FIXTURE_TEST_CASE(test_collide_boolean, ConservativeAdvancementFixture)
{
	ContinuousCollisionRequest request;
	ContinuousCollisionResult result;

	bool collide = true;

	collide = advancement_collision_->collideBoolean(request, result);
	BOOST_CHECK_EQUAL(true, collide);

	result.clear();
	collide = advancement_collision_free_->collideBoolean(request, result);
	BOOST_CHECK_EQUAL(false, collide);
}

BOOST_FIXTURE_TEST_CASE(test_backward_compability, ConservativeAdvancementFixture)
{
	CollisionRequest request;
	CollisionResult result;
	FCL_REAL toc;

	request.num_max_contacts = 2;

	int collisions_number = 0;

	collisions_number  = conservativeAdvancement<RSS,MeshDistanceTraversalNodeRSS, MeshCollisionTraversalNodeRSS>(
		box_static_.get(), static_motion_.get(),
		box_moving_.get(), interp_motion_collision.get(),
		request,
		result,
		toc
	);
	BOOST_CHECK_LT(0, collisions_number);

	collisions_number  = conservativeAdvancement<RSS,MeshDistanceTraversalNodeRSS, MeshCollisionTraversalNodeRSS>(
		box_static_.get(), static_motion_.get(),
		box_moving_.get(), interp_motion_collision_free.get(),
		request,
		result,
		toc
	);
	BOOST_CHECK_EQUAL(0, collisions_number);
}

BOOST_FIXTURE_TEST_CASE(test_discrete_detection_in_time, ConservativeAdvancementFixture)
{	
	CollisionResult result;

	int collisions_number = 0;

	result.clear();
	collisions_number = 
		advancement_collision_->discreteDetectionInTime(0.0, result);
	BOOST_CHECK_EQUAL(0, collisions_number);

	result.clear();
	collisions_number = 
		advancement_collision_->discreteDetectionInTime(0.5, result);
	BOOST_CHECK_LT(0, collisions_number); // should be in collision

	result.clear();
	collisions_number = 
		advancement_collision_->discreteDetectionInTime(1.0, result);
	BOOST_CHECK_EQUAL(0, collisions_number);

	result.clear();
	collisions_number =
		advancement_collision_free_->discreteDetectionInTime(0.0, result);
	BOOST_CHECK_EQUAL(0, collisions_number);

	result.clear();
	collisions_number =
		advancement_collision_free_->discreteDetectionInTime(1.0, result);
	BOOST_CHECK_EQUAL(0, collisions_number);
}

BOOST_AUTO_TEST_SUITE_END()
////////////////////////////////////////////////////////////////////////////////

}