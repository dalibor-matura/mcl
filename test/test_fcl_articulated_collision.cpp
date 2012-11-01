#define BOOST_TEST_MODULE "FCL_MODEL_BOUND"
#include <boost/test/unit_test.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/filesystem.hpp>

#include "fcl/articulated_model/link.h"
#include "fcl/articulated_model/joint.h"
#include "fcl/articulated_model/movement.h"
#include "fcl/articulated_model/model.h"
#include "fcl/articulated_model/model_config.h"
#include "fcl/articulated_model/link_bound.h"
#include "fcl/ccd/interpolation/interpolation_data.h"

#include "fcl/ccd/conservative_advancement.h"
#include "fcl/ccd/motion.h"
#include "fcl/collision_node.h"
#include "fcl/collision.h"
#include "fcl/BV/BV.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/narrowphase/narrowphase.h"
#include "test_fcl_utility.h"
#include "fcl_resources/config.h"

namespace fcl {

int num_max_contacts = std::numeric_limits<int>::max();
bool enable_contact = true;

std::vector<Contact> global_pairs;
std::vector<Contact> global_pairs_now;


class ArticularCollisionFixture
{
public:
	ArticularCollisionFixture()
	{
		initGeneral();
		initLinkBound();
		initMotions();
		initCollisionObjectsOnly();
	}

	~ArticularCollisionFixture()
	{

	}

	void initGeneral()
	{
		direction_ = Vec3f(0.0, 1.0, 0.0);
	}

	void initMotions()
	{
		initArticularMotion();
		initInterpMotion();
	}

	void initArticularMotion()
	{
		articular_motion_.reset(new ArticularMotion(link_bound_) );
	}

	void initInterpMotion()
	{
		// no transformation
		interp_motion_transform_ = Transform3f(Matrix3f::getIdentity(), Vec3f(0, 0, 0) );

		// no movement, just stays on the place
		interp_motion_.reset(new InterpMotion(interp_motion_transform_, interp_motion_transform_) );
	}

	void initLinkBound()
	{
		initModel();
		initConfigurationsOnly();
		initMovementOnly();

		link_bound_.reset(new LinkBound(model_, movement_, finger_) );
	}

	void initMovementOnly()
	{
		movement_.reset(new Movement(model_, cfg_start_, cfg_end_) );
	}	

	void initModel()
	{
		initLinksOnly();
		initJointsOnly();	

		boost::shared_ptr<const InterpolationData> interpolation_data(new InterpolationLinearData() );

		initModelOnly(interpolation_data);
	}

	void initModelOnly(boost::shared_ptr<const InterpolationData> interpolation_data)
	{
		model_.reset(new Model() );

		model_->addLink(body_);
		model_->addLink(arm_);
		model_->addLink(forearm_);
		model_->addLink(hand_);
		model_->addLink(finger_);

		model_->addJoint(shoulder_joint_, interpolation_data);
		model_->addJoint(elbow_joint_, interpolation_data);
		model_->addJoint(wrist_joint_, interpolation_data);
		model_->addJoint(finger_joint_, interpolation_data);

		model_->initTree();
	}

	void initLinksOnly()
	{
		body_name_ = "BODY";
		arm_name_ = "ARM";
		forearm_name_ = "FOREARM";
		hand_name_ = "HAND";
		finger_name_ = "FINGER";

		body_.reset(new Link(body_name_) );
		arm_.reset(new Link(arm_name_) );
		forearm_.reset(new Link(forearm_name_) );
		hand_.reset(new Link(hand_name_) );
		finger_.reset(new Link(finger_name_) );		
	}

	void initJointsOnly()
	{
		shoulder_joint_name_ = "SHOULDER";
		elbow_joint_name_ = "ELBOW";
		wrist_joint_name_ = "WRIST";
		finger_joint_name_ = "FINGER";

		shoulder_elbow_vector_ = Vec3f(0, 0, 0);
		elbow_wrist_vector_ = Vec3f(0, 0, 0);
		wrist_finger_vector_ = Vec3f(0, 0, 0);

		shoulder_elbow_matrix_ = Matrix3f::getIdentity();
		elbow_wrist_matrix_ = Matrix3f::getIdentity();
		wrist_finger_matrix_ = Matrix3f::getIdentity();

		Vec3f axis;
		Transform3f transform;

		// move in Z axis
		transform = Transform3f(Matrix3f::getIdentity(), Vec3f() );
		axis = Vec3f(0.0, 0.0, 1.0);
		shoulder_joint_.reset(new PrismaticJoint(body_, arm_, transform, shoulder_joint_name_, axis) );

		// rotate around Z axis
		transform = Transform3f(shoulder_elbow_matrix_, shoulder_elbow_vector_);
		axis = Vec3f(0.0, 0.0, 1.0);
		elbow_joint_.reset(new RevoluteJoint(arm_, forearm_, transform, elbow_joint_name_, axis) );

		// move in X axis
		transform = Transform3f(elbow_wrist_matrix_, elbow_wrist_vector_);
		axis = Vec3f(1.0, 0.0, 0.0);
		wrist_joint_.reset(new PrismaticJoint(forearm_, hand_, transform, wrist_joint_name_, axis) );

		// rotate around X axis
		transform = Transform3f(wrist_finger_matrix_, wrist_finger_vector_);
		axis = Vec3f(1.0, 0.0, 0.0);
		finger_joint_.reset(new RevoluteJoint(hand_, finger_, transform, finger_joint_name_, axis) );		
	}

	void initConfigurationsOnly()
	{
		cfg_start_.reset(new ModelConfig(model_) );
		cfg_end_.reset(new ModelConfig(model_) );

		cfg_end_->getJointConfig(shoulder_joint_)[0] = 6;
		cfg_end_->getJointConfig(elbow_joint_)[0] = boost::math::constants::pi<double>() / 2;
		cfg_end_->getJointConfig(wrist_joint_)[0] = 4;
		cfg_end_->getJointConfig(finger_joint_)[0] = 3;	
	}	

	void initCollisionObjectsOnly()
	{
		std::vector<Vec3f> p1, p2;
		std::vector<Triangle> t1, t2;
		boost::filesystem::path path(TEST_RESOURCES_DIR);

		loadOBJFile((path / "env.obj").string().c_str(), p1, t1);
		loadOBJFile((path / "rob.obj").string().c_str(), p2, t2);	

		SplitMethodType split_method = SPLIT_METHOD_MEAN;
		model_environment_.bv_splitter.reset(new BVSplitter<RSS>(split_method));
		model_robot_.bv_splitter.reset(new BVSplitter<RSS>(split_method));

		model_environment_.beginModel();
		model_environment_.addSubModel(p1, t1);
		model_environment_.endModel();

		model_robot_.beginModel();
		model_robot_.addSubModel(p2, t2);
		model_robot_.endModel();
	}

	void setNewConfigurations(boost::shared_ptr<ModelConfig>& cfg_start,
		boost::shared_ptr<ModelConfig>& cfg_end)
	{
		cfg_start_ = cfg_start;
		cfg_end_ = cfg_end;

		initMovementOnly();

		link_bound_.reset(new LinkBound(model_, movement_, finger_) );

		initArticularMotion();
	}

protected:
	void setConfigurations_1()
	{
		boost::shared_ptr<ModelConfig> cfg_start (new ModelConfig(model_) );
		boost::shared_ptr<ModelConfig> cfg_end (new ModelConfig(model_) );
		
		// Environment and Robot objects are already in collision
		cfg_start->getJointConfig(shoulder_joint_)[0] = 0;
		cfg_start->getJointConfig(elbow_joint_)[0] = 0;
		cfg_start->getJointConfig(wrist_joint_)[0] = 0;
		cfg_start->getJointConfig(finger_joint_)[0] = 0;

		cfg_end->getJointConfig(shoulder_joint_)[0] = 1000;
		cfg_end->getJointConfig(elbow_joint_)[0] = 0;
		cfg_end->getJointConfig(wrist_joint_)[0] = 0;
		cfg_end->getJointConfig(finger_joint_)[0] = 0;

		setNewConfigurations(cfg_start, cfg_end);
	}	

	void setConfigurations_2()
	{
		boost::shared_ptr<ModelConfig> cfg_start (new ModelConfig(model_) );
		boost::shared_ptr<ModelConfig> cfg_end (new ModelConfig(model_) );

		// Environment and Robot objects are NOT in collision
		cfg_start->getJointConfig(shoulder_joint_)[0] = -2000;
		cfg_start->getJointConfig(elbow_joint_)[0] = 0;
		cfg_start->getJointConfig(wrist_joint_)[0] = 0;
		cfg_start->getJointConfig(finger_joint_)[0] = 0;

		cfg_end->getJointConfig(shoulder_joint_)[0] = +2000;
		cfg_end->getJointConfig(elbow_joint_)[0] = 0;
		cfg_end->getJointConfig(wrist_joint_)[0] = 0;
		cfg_end->getJointConfig(finger_joint_)[0] = 0;

		// path contains collision
		setNewConfigurations(cfg_start, cfg_end);
	}

	void setConfigurations_3()
	{
		boost::shared_ptr<ModelConfig> cfg_start (new ModelConfig(model_) );
		boost::shared_ptr<ModelConfig> cfg_end (new ModelConfig(model_) );

		// Environment and Robot objects are NOT in collision
		cfg_start->getJointConfig(shoulder_joint_)[0] = 2000;
		cfg_start->getJointConfig(elbow_joint_)[0] = 0;
		cfg_start->getJointConfig(wrist_joint_)[0] = 0;
		cfg_start->getJointConfig(finger_joint_)[0] = 0;

		cfg_end->getJointConfig(shoulder_joint_)[0] = 4000;
		cfg_end->getJointConfig(elbow_joint_)[0] = 0;
		cfg_end->getJointConfig(wrist_joint_)[0] = 0;
		cfg_end->getJointConfig(finger_joint_)[0] = 0;

		// path is collision free
		setNewConfigurations(cfg_start, cfg_end);
	}

	void setConfigurations_4()
	{
		boost::shared_ptr<ModelConfig> cfg_start (new ModelConfig(model_) );
		boost::shared_ptr<ModelConfig> cfg_end (new ModelConfig(model_) );

		// Environment and Robot objects are NOT in collision
		cfg_start->getJointConfig(shoulder_joint_)[0] = 1525;
		cfg_start->getJointConfig(elbow_joint_)[0] = 0;
		cfg_start->getJointConfig(wrist_joint_)[0] = 0;
		cfg_start->getJointConfig(finger_joint_)[0] = 0;

		cfg_end->getJointConfig(shoulder_joint_)[0] = 1525;
		cfg_end->getJointConfig(elbow_joint_)[0] = 0;
		cfg_end->getJointConfig(wrist_joint_)[0] = 2000;
		cfg_end->getJointConfig(finger_joint_)[0] = 0;

		// path is collision free
		setNewConfigurations(cfg_start, cfg_end);
	}

	void setConfigurations_5()
	{
		boost::shared_ptr<ModelConfig> cfg_start (new ModelConfig(model_) );
		boost::shared_ptr<ModelConfig> cfg_end (new ModelConfig(model_) );

		// Environment and Robot objects are NOT in collision
		cfg_start->getJointConfig(shoulder_joint_)[0] = 1525;
		cfg_start->getJointConfig(elbow_joint_)[0] = 0;
		cfg_start->getJointConfig(wrist_joint_)[0] = 0;
		cfg_start->getJointConfig(finger_joint_)[0] = 0;

		cfg_end->getJointConfig(shoulder_joint_)[0] = 1525;
		cfg_end->getJointConfig(elbow_joint_)[0] = 0;
		cfg_end->getJointConfig(wrist_joint_)[0] = 2000;
		cfg_end->getJointConfig(finger_joint_)[0] = boost::math::constants::pi<double>() / 2;

		// path contains collision ; the piano is rotating through the window and the window is too small
		setNewConfigurations(cfg_start, cfg_end);
	}

	void setConfigurations_6()
	{
		boost::shared_ptr<ModelConfig> cfg_start (new ModelConfig(model_) );
		boost::shared_ptr<ModelConfig> cfg_end (new ModelConfig(model_) );

		// Environment and Robot objects are NOT in collision
		cfg_start->getJointConfig(shoulder_joint_)[0] = 1400;
		cfg_start->getJointConfig(elbow_joint_)[0] = 0;
		cfg_start->getJointConfig(wrist_joint_)[0] = 0;
		cfg_start->getJointConfig(finger_joint_)[0] = 0;

		cfg_end->getJointConfig(shoulder_joint_)[0] = 1530;
		cfg_end->getJointConfig(elbow_joint_)[0] = boost::math::constants::pi<double>() / 90;
		cfg_end->getJointConfig(wrist_joint_)[0] = 2000;
		cfg_end->getJointConfig(finger_joint_)[0] = 0;

		// path is collision free
		setNewConfigurations(cfg_start, cfg_end);
	}

	void setJointsInterpolation(boost::shared_ptr<const InterpolationData> interpolation_data)
	{
		initModelOnly(interpolation_data);
		initConfigurationsOnly();
		initMovementOnly();

		link_bound_.reset(new LinkBound(model_, movement_, finger_) );

		initArticularMotion();
	}

	int performContinuousCollision()
	{		
		CollisionRequest collision_request;
		CollisionResult collision_result;
		FCL_REAL toc = 0;
		int number_of_contacts = 0;

		number_of_contacts = 
			conservativeAdvancement<RSS, MeshConservativeAdvancementTraversalNodeRSS, MeshCollisionTraversalNodeRSS>(
			&model_robot_, articular_motion_.get(),
			&model_environment_, interp_motion_.get(),
			collision_request, collision_result, toc
			);

		return number_of_contacts;
	}

protected:
	boost::shared_ptr<ArticularMotion> articular_motion_;
	boost::shared_ptr<InterpMotion> interp_motion_;
	boost::shared_ptr<LinkBound> link_bound_;
	boost::shared_ptr<Model> model_;
	boost::shared_ptr<Movement> movement_;

	Transform3f interp_motion_transform_;

	boost::shared_ptr<ModelConfig> cfg_start_;
	boost::shared_ptr<ModelConfig> cfg_end_;

	boost::shared_ptr<Joint> shoulder_joint_;
	boost::shared_ptr<Joint> elbow_joint_;
	boost::shared_ptr<Joint> wrist_joint_;
	boost::shared_ptr<Joint> finger_joint_;

	std::string shoulder_joint_name_;
	std::string elbow_joint_name_;
	std::string wrist_joint_name_;
	std::string finger_joint_name_;	

	Vec3f shoulder_elbow_vector_;
	Vec3f elbow_wrist_vector_;
	Vec3f wrist_finger_vector_;

	Matrix3f shoulder_elbow_matrix_;
	Matrix3f elbow_wrist_matrix_;
	Matrix3f wrist_finger_matrix_;

	boost::shared_ptr<Link> body_;
	boost::shared_ptr<Link> arm_;
	boost::shared_ptr<Link> forearm_;
	boost::shared_ptr<Link> hand_;
	boost::shared_ptr<Link> finger_;

	std::string body_name_;
	std::string arm_name_;
	std::string forearm_name_;
	std::string hand_name_;
	std::string finger_name_;

	Vec3f direction_;

	BVHModel<RSS> model_environment_;
	BVHModel<RSS> model_robot_;
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
BOOST_AUTO_TEST_SUITE(test_model_bound)

BOOST_FIXTURE_TEST_CASE(test_articulated_collision_with_linear_interpolation, ArticularCollisionFixture)
{
	int number_of_contacts = 0;

	setJointsInterpolation(boost::make_shared<const InterpolationLinearData>() );

	setConfigurations_1();
	number_of_contacts = performContinuousCollision();
	BOOST_CHECK_EQUAL(1, number_of_contacts);

	setConfigurations_2();
	number_of_contacts = performContinuousCollision();
	BOOST_CHECK_EQUAL(1, number_of_contacts);

	setConfigurations_3();
	number_of_contacts = performContinuousCollision();
	BOOST_CHECK_EQUAL(0, number_of_contacts);

	setConfigurations_4();
	number_of_contacts = performContinuousCollision();
	BOOST_CHECK_EQUAL(0, number_of_contacts);

	setConfigurations_5();
	number_of_contacts = performContinuousCollision();
	BOOST_CHECK_EQUAL(1, number_of_contacts);

	setConfigurations_6();
	number_of_contacts = performContinuousCollision();
	BOOST_CHECK_EQUAL(0, number_of_contacts);
}

BOOST_FIXTURE_TEST_CASE(test_articulated_collision_with_third_order_interpolation, ArticularCollisionFixture)
{
	int number_of_contacts = 0;

	setJointsInterpolation(boost::make_shared<const InterpolationThirdOrderData>(10, 10, 10) );

	setConfigurations_1();
	number_of_contacts = performContinuousCollision();
	BOOST_CHECK_EQUAL(1, number_of_contacts);

	setConfigurations_2();
	number_of_contacts = performContinuousCollision();
	BOOST_CHECK_EQUAL(1, number_of_contacts);

	setConfigurations_3();
	number_of_contacts = performContinuousCollision();
	BOOST_CHECK_EQUAL(0, number_of_contacts);

	setConfigurations_4();
	number_of_contacts = performContinuousCollision();
	BOOST_CHECK_EQUAL(0, number_of_contacts);

	setConfigurations_5();
	number_of_contacts = performContinuousCollision();
	BOOST_CHECK_EQUAL(1, number_of_contacts);

	setConfigurations_6();
	number_of_contacts = performContinuousCollision();
	BOOST_CHECK_EQUAL(0, number_of_contacts);
}

BOOST_AUTO_TEST_SUITE_END()
////////////////////////////////////////////////////////////////////////////////

}