#define BOOST_TEST_MODULE "FCL_MODEL_BOUND"
#include <boost/test/unit_test.hpp>

#include <boost/shared_ptr.hpp>

#include "fcl/articulated_model/link.h"
#include "fcl/articulated_model/joint.h"
#include "fcl/articulated_model/model.h"
#include "fcl/articulated_model/model_config.h"
#include "fcl/articulated_model/model_bound.h"

namespace fcl {

class ModelBoundFixture
{
public:
	ModelBoundFixture()
	{
		init();
	}

	~ModelBoundFixture()
	{

	}

	void init()
	{
		initModel();
		initConfigurations();
		initModelBound();

		direction_ = Vec3f(0.0, 1.0, 0.0);
	}

	void initModel()
	{
		model_.reset(new Model() );

		initLinks();
		initJoints();		
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

		model_->addLink(body_);
		model_->addLink(arm_);
		model_->addLink(forearm_);
		model_->addLink(hand_);
		model_->addLink(finger_);
	}

	void initJoints()
	{
		shoulder_joint_name_ = "SHOULDER";
		elbow_joint_name_ = "ELBOW";
		wrist_joint_name_ = "WRIST";
		finger_joint_name_ = "FINGER";

		Matrix3f matrix;
		Vec3f vector;
		Vec3f axis;
		Transform3f transform;

		matrix = Matrix3f();
		vector = Vec3f(24,0,0);
		transform = Transform3f(matrix, vector);
		axis = Vec3f(1.0, 0.0, 0.0);
		shoulder_joint_.reset(new PrismaticJoint(body_, arm_, transform, shoulder_joint_name_, axis) );

		matrix = Matrix3f(
			0.0, 1.0, 1.0,
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0
			);
		vector = Vec3f(0,0,0);
		transform = Transform3f(matrix, vector);
		axis = Vec3f(1.0, 0.0, 0.0);
		elbow_joint_.reset(new RevoluteJoint(arm_, forearm_, transform, elbow_joint_name_, axis) );

		matrix = Matrix3f();
		vector = Vec3f(0,6,0);
		transform = Transform3f(matrix, vector);
		axis = Vec3f(0.0, 1.0, 0.0);
		wrist_joint_.reset(new PrismaticJoint(forearm_, hand_, transform, wrist_joint_name_, axis) );

		matrix = Matrix3f(
			0.0, 0.0, 0.0,
			0.0, 1.0, 0.0,
			0.0, 1.0, 0.0
			);
		vector = Vec3f(0,0,0);
		transform = Transform3f(matrix, vector);
		axis = Vec3f(0.0, 1.0, 0.0);
		finger_joint_.reset(new RevoluteJoint(hand_, finger_, transform, finger_joint_name_, axis) );

		model_->addJoint(shoulder_joint_);
		model_->addJoint(elbow_joint_);
		model_->addJoint(wrist_joint_);
		model_->addJoint(finger_joint_);
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

	void initModelBound()
	{
		model_bound_.reset(new ModelBound(model_, cfg_start_, cfg_end_) );
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

BOOST_FIXTURE_TEST_CASE(motion_bound_test, ModelBoundFixture)
{
	FCL_REAL bound = -1.0;
	FCL_REAL time = 0.0;	

	bound = model_bound_->getMotionBound(body_name_, time, direction_);

	BOOST_CHECK_EQUAL(0.0, bound);

	bound = model_bound_->getMotionBound(arm_name_, time, direction_);

	BOOST_CHECK_LE(0.0, bound);
}

}