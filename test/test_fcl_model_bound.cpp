#define BOOST_TEST_MODULE "FCL_MODEL_BOUND"
#include <boost/test/unit_test.hpp>

#include <boost/shared_ptr.hpp>

#include "fcl/articulated_model/link.h"
#include "fcl/articulated_model/joint.h"
#include "fcl/articulated_model/joint_bound_info.h"
#include "fcl/articulated_model/model.h"
#include "fcl/articulated_model/model_config.h"
#include "fcl/articulated_model/model_bound.h"

namespace fcl {

class InitFixture
{
public:
	InitFixture()
	{
		initGeneral();
	}

	~InitFixture()
	{

	}

	void initGeneral()
	{
		direction_ = Vec3f(0.0, 1.0, 0.0);
	}

	void initModelBound()
	{
		initModel();
		initConfigurations();

		model_bound_.reset(new ModelBound(model_, cfg_start_, cfg_end_) );
	}

	void initModel()
	{
		model_.reset(new Model() );

		initLinks();

		model_->addLink(body_);
		model_->addLink(arm_);
		model_->addLink(forearm_);
		model_->addLink(hand_);
		model_->addLink(finger_);

		initJoints();	

		model_->addJoint(shoulder_joint_);
		model_->addJoint(elbow_joint_);
		model_->addJoint(wrist_joint_);
		model_->addJoint(finger_joint_);
	}

	void initLinks()
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

	void initJoints()
	{
		shoulder_joint_name_ = "SHOULDER";
		elbow_joint_name_ = "ELBOW";
		wrist_joint_name_ = "WRIST";
		finger_joint_name_ = "FINGER";

		shoulder_elbow_vector_ = Vec3f(20, 0, 0);
		elbow_wrist_vector_ = Vec3f(0, 4, 0);
		wrist_finger_vector_ = Vec3f(0, 0, 7);

		shoulder_elbow_matrix_ = Matrix3f(
			0.0, 1.0, 1.0,
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0
		);
		elbow_wrist_matrix_ = Matrix3f(
		);
		wrist_finger_matrix_ = Matrix3f(
			0.0, 0.0, 0.0,
			0.0, 1.0, 0.0,
			0.0, 1.0, 0.0
		);

		Vec3f axis;
		Transform3f transform;

		transform = Transform3f(Matrix3f(), Vec3f() );
		axis = Vec3f(1.0, 0.0, 0.0);
		shoulder_joint_.reset(new PrismaticJoint(body_, arm_, transform, shoulder_joint_name_, axis) );

		transform = Transform3f(shoulder_elbow_matrix_, shoulder_elbow_vector_);
		axis = Vec3f(1.0, 0.0, 0.0);
		elbow_joint_.reset(new RevoluteJoint(arm_, forearm_, transform, elbow_joint_name_, axis) );

		transform = Transform3f(elbow_wrist_matrix_, elbow_wrist_vector_);
		axis = Vec3f(0.0, 1.0, 0.0);
		wrist_joint_.reset(new PrismaticJoint(forearm_, hand_, transform, wrist_joint_name_, axis) );

		transform = Transform3f(wrist_finger_matrix_, wrist_finger_vector_);
		axis = Vec3f(0.0, 1.0, 0.0);
		finger_joint_.reset(new RevoluteJoint(hand_, finger_, transform, finger_joint_name_, axis) );		
	}

	void initConfigurations()
	{
		cfg_start_.reset(new ModelConfig(model_) );
		cfg_end_.reset(new ModelConfig(model_) );

		cfg_end_->getJointConfig(shoulder_joint_)[0] = 6;
		cfg_end_->getJointConfig(elbow_joint_)[0] = 5;
		cfg_end_->getJointConfig(wrist_joint_)[0] = 4;
		cfg_end_->getJointConfig(finger_joint_)[0] = 3;	
	}	

protected:
	boost::shared_ptr<ModelBound> model_bound_;
	boost::shared_ptr<Model> model_;
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
};

class LinkFixture :
	public InitFixture
{
public:
	LinkFixture() :
		InitFixture()
	{
		initLinks();
	}
};

class JointFixture :
	public InitFixture
{
public:
	JointFixture() :
		InitFixture()
	{
		initJoints();
	}
};

class ModelFixture :
	public InitFixture
{
public:
	ModelFixture() :
		InitFixture()
	{
		initModel();

		initTestLink();
		initTestJoint();
	}

	void initTestLink()
	{
		test_link_name_ = "TEST LINK";

		test_link_.reset(new Link(test_link_name_) );
	}

	void initTestJoint()
	{
		test_joint_name_ = "TEST JOINT";

		test_joint_vector_ = Vec3f(10, 0, 0);

		test_joint_matrix_ = Matrix3f(
			2.0, 0.0, 0.0,
			0.0, 2.0, 0.0,
			0.0, 0.0, 2.0
			);

		Vec3f axis;
		Transform3f transform;

		transform = Transform3f(test_joint_matrix_, test_joint_vector_);
		axis = Vec3f(0.0, 0.0, 1.0);
		shoulder_joint_.reset(new PrismaticJoint(body_, arm_, transform, test_joint_name_, axis) );
	}

protected:
	boost::shared_ptr<Link> test_link_;
	std::string test_link_name_;

	boost::shared_ptr<Joint> test_joint_;
	std::string test_joint_name_;
	Vec3f test_joint_vector_;
	Matrix3f test_joint_matrix_;
};

class JointBoundInfoFixture :
	public InitFixture
{
public:
	JointBoundInfoFixture() :
		InitFixture()
	{
		initModel();
		initConfigurations();

		joint_bound_info_.reset(new JointBoundInfo(model_, cfg_start_, cfg_end_) );
	}

protected:
	boost::shared_ptr<JointBoundInfo> joint_bound_info_;
};

class ModelBoundFixture :
	public InitFixture
{
public:
	ModelBoundFixture() :
		InitFixture()
	{
		initModelBound();
	}
};

bool operator==(const Vec3f& first, const Vec3f& second)
{
	return first[0] == second[0] &&
		first[1] == second[1] &&
		first[2] == second[2];
}

BOOST_AUTO_TEST_SUITE(test_model)

BOOST_FIXTURE_TEST_CASE(test_add_get_joint, ModelFixture)
{
	/*model_->addJoint(test_joint_);

	boost::shared_ptr<Joint> joint = model_->getJoint(test_joint_name_);

	BOOST_CHECK_EQUAL(test_joint_, joint);*/
}

BOOST_FIXTURE_TEST_CASE(test_add_get_link, ModelFixture)
{
	model_->addLink(test_link_);

	boost::shared_ptr<Link> link = model_->getLink(test_link_name_);

	BOOST_CHECK_EQUAL(test_link_, link);
}

BOOST_AUTO_TEST_SUITE_END()
BOOST_AUTO_TEST_SUITE(test_joint_bound_info)

BOOST_FIXTURE_TEST_CASE(test_set_get_current_time, JointBoundInfoFixture)
{
	FCL_REAL time = 33.7;

	joint_bound_info_->setCurrentTime(time);

	BOOST_CHECK_EQUAL(time, joint_bound_info_->getCurrentTime() );
}

BOOST_FIXTURE_TEST_CASE(test_get_linear_velocity_bound, JointBoundInfoFixture)
{
	Vec3f bound = -1.0;

	bound = joint_bound_info_->getLinearVelocityBound(boost::static_pointer_cast<const Joint>(elbow_joint_) );

	BOOST_CHECK_EQUAL(Vec3f(0.0, 0.0, 0.0), bound);
}

BOOST_FIXTURE_TEST_CASE(test_get_absolute_linear_velocity_bound, JointBoundInfoFixture)
{
	FCL_REAL bound = -1.0;

	bound = joint_bound_info_->getAbsoluteLinearVelocityBound(boost::static_pointer_cast<const Joint>(elbow_joint_) );

	BOOST_CHECK_EQUAL(0.0, bound);
}

BOOST_FIXTURE_TEST_CASE(test_get_angular_velocity_bound, JointBoundInfoFixture)
{
	Vec3f bound = -1.0;

	bound = joint_bound_info_->getAngularVelocityBound(boost::static_pointer_cast<const Joint>(shoulder_joint_) );

	BOOST_CHECK_EQUAL(Vec3f(0.0, 0.0, 0.0), bound);
}

BOOST_FIXTURE_TEST_CASE(test_get_absolute_angular_velocity_bound, JointBoundInfoFixture)
{
	FCL_REAL bound = -1.0;

	bound = joint_bound_info_->getAbsoluteAngularVelocityBound(boost::static_pointer_cast<const Joint>(shoulder_joint_) );

	BOOST_CHECK_EQUAL(0.0, bound);
}

BOOST_FIXTURE_TEST_CASE(test_get_vector_length_bound, JointBoundInfoFixture)
{
	FCL_REAL bound = -1.0;
	FCL_REAL time = 0.0;	

	bound = joint_bound_info_->getVectorLengthBound(boost::static_pointer_cast<const Joint>(shoulder_joint_), 
		boost::static_pointer_cast<const Joint>(elbow_joint_) );

	BOOST_CHECK_LE(shoulder_elbow_vector_.length(), bound);

	bound = joint_bound_info_->getVectorLengthBound(boost::static_pointer_cast<const Joint>(elbow_joint_), 
		boost::static_pointer_cast<const Joint>(wrist_joint_) );

	BOOST_CHECK_EQUAL(elbow_wrist_vector_.length(), bound);
}

BOOST_AUTO_TEST_SUITE_END()
BOOST_AUTO_TEST_SUITE(test_model_bound)

BOOST_FIXTURE_TEST_CASE(test_get_motion_bound, ModelBoundFixture)
{
	FCL_REAL bound = -1.0;
	FCL_REAL time = 0.0;	

	bound = model_bound_->getMotionBound(body_name_, time, direction_);

	BOOST_CHECK_EQUAL(0.0, bound);

	bound = model_bound_->getMotionBound(arm_name_, time, direction_);

	BOOST_CHECK_LE(0.0, bound);
}

BOOST_AUTO_TEST_SUITE_END()
BOOST_AUTO_TEST_SUITE(test_model)



BOOST_AUTO_TEST_SUITE_END()

}