#define BOOST_TEST_MODULE "FCL_MODEL_BOUND"
#include <boost/test/unit_test.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/math/constants/constants.hpp>

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

		model_bound_.reset(new ModelBound(model_, cfg_start_, cfg_end_, finger_) );
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

		model_->initTree();
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
		initJoints();
	}
};

class JointFixture :
	public InitFixture
{
public:
	JointFixture() :
		InitFixture()
	{
		initLinks();
		initJoints();
	}
};

class JointConfigFixture :
	public InitFixture
{
public:
	JointConfigFixture() :
		InitFixture()
	{
		initJoints();
		initJointConfig();
	}

protected:
	void initJointConfig()
	{
		joint_cfg_value_ = 2;
		joint_cfg_value_min_ = -1;
		joint_cfg_value_max_ = 3;

		joint_config_.reset(
			new JointConfig(shoulder_joint_, joint_cfg_value_, joint_cfg_value_min_, joint_cfg_value_max_)
		);
	}

protected:
	boost::shared_ptr<JointConfig> joint_config_;

	FCL_REAL joint_cfg_value_;
	FCL_REAL joint_cfg_value_min_;
	FCL_REAL joint_cfg_value_max_;
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
		test_joint_.reset(new PrismaticJoint(body_, arm_, transform, test_joint_name_, axis) );
	}

protected:
	boost::shared_ptr<Link> test_link_;
	std::string test_link_name_;

	boost::shared_ptr<Joint> test_joint_;
	std::string test_joint_name_;
	Vec3f test_joint_vector_;
	Matrix3f test_joint_matrix_;
};

class ModelConfigFixture :
	public InitFixture
{
public:
	ModelConfigFixture() :
		InitFixture()
	{
		initModel();
		initConfigurations();		
	}

protected:
	
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

		distance_from_center_ = 6;
	}

protected:
	// bounded link = body_
	void setEndCfgLinkBody()
	{
		model_bound_.reset(new ModelBound(model_, cfg_start_, cfg_end_, body_) );		
	}

	// bounded link = arm_
	void setEndCfgLinkArm()
	{
		model_bound_.reset(new ModelBound(model_, cfg_start_, cfg_end_, arm_) );		
	}

	// one rotation
	void setEndCfg_1()
	{
		finger_joint_cfg_value_ = boost::math::constants::pi<double>() / 2;				

		cfg_end_->getJointConfig(shoulder_joint_)[0] = 0;
		cfg_end_->getJointConfig(elbow_joint_)[0] = 0;
		cfg_end_->getJointConfig(wrist_joint_)[0] = 0;
		cfg_end_->getJointConfig(finger_joint_)[0] = finger_joint_cfg_value_;	

		model_bound_.reset(new ModelBound(model_, cfg_start_, cfg_end_, finger_) );		
	}

	// two rotations
	void setEndCfg_2()
	{
		elbow_joint_cfg_value_ = boost::math::constants::pi<double>();
		finger_joint_cfg_value_ = boost::math::constants::pi<double>() / 2;		

		cfg_end_->getJointConfig(shoulder_joint_)[0] = 0;
		cfg_end_->getJointConfig(elbow_joint_)[0] = elbow_joint_cfg_value_;
		cfg_end_->getJointConfig(wrist_joint_)[0] = 0;
		cfg_end_->getJointConfig(finger_joint_)[0] = finger_joint_cfg_value_;	

		wrist_joint_->setTransformToParent(Transform3f() );
		finger_joint_->setTransformToParent(Transform3f() );

		model_bound_.reset(new ModelBound(model_, cfg_start_, cfg_end_, finger_) );		
	}

	// one linear movement
	void setEndCfg_3()
	{
		wrist_joint_cfg_value_ = 22.6;				

		cfg_end_->getJointConfig(shoulder_joint_)[0] = 0;
		cfg_end_->getJointConfig(elbow_joint_)[0] = 0;
		cfg_end_->getJointConfig(wrist_joint_)[0] = wrist_joint_cfg_value_;
		cfg_end_->getJointConfig(finger_joint_)[0] = 0;	

		model_bound_.reset(new ModelBound(model_, cfg_start_, cfg_end_, finger_) );		
	}

	// two linear movements
	void setEndCfg_4()
	{
		shoulder_joint_cfg_value_ = 100.0045;
		wrist_joint_cfg_value_ = 22.6;				

		cfg_end_->getJointConfig(shoulder_joint_)[0] = shoulder_joint_cfg_value_;
		cfg_end_->getJointConfig(elbow_joint_)[0] = 0;
		cfg_end_->getJointConfig(wrist_joint_)[0] = wrist_joint_cfg_value_;
		cfg_end_->getJointConfig(finger_joint_)[0] = 0;	

		model_bound_.reset(new ModelBound(model_, cfg_start_, cfg_end_, finger_) );		
	}

	// one rotation + one linear movement
	void setEndCfg_5()
	{
		wrist_joint_cfg_value_ = 22.6;				
		finger_joint_cfg_value_ = boost::math::constants::pi<double>() / 2;	

		cfg_end_->getJointConfig(shoulder_joint_)[0] = 0;
		cfg_end_->getJointConfig(elbow_joint_)[0] = 0;
		cfg_end_->getJointConfig(wrist_joint_)[0] = wrist_joint_cfg_value_;
		cfg_end_->getJointConfig(finger_joint_)[0] = finger_joint_cfg_value_;	

		model_bound_.reset(new ModelBound(model_, cfg_start_, cfg_end_, finger_) );		
	}

	// two rotations + transform to parent
	void setEndCfg_6()
	{
		elbow_joint_cfg_value_ = boost::math::constants::pi<double>();
		finger_joint_cfg_value_ = boost::math::constants::pi<double>() / 2;		

		finger_joint_transform_to_parent_ = Vec3f(4, 0, 0);

		cfg_end_->getJointConfig(shoulder_joint_)[0] = 0;
		cfg_end_->getJointConfig(elbow_joint_)[0] = elbow_joint_cfg_value_;
		cfg_end_->getJointConfig(wrist_joint_)[0] = 0;
		cfg_end_->getJointConfig(finger_joint_)[0] = finger_joint_cfg_value_;	

		wrist_joint_->setTransformToParent(Transform3f() );
		finger_joint_->setTransformToParent(Transform3f(Matrix3f(), finger_joint_transform_to_parent_) );

		model_bound_.reset(new ModelBound(model_, cfg_start_, cfg_end_, finger_) );		
	}

protected:
	FCL_REAL distance_from_center_;

	FCL_REAL shoulder_joint_cfg_value_;
	FCL_REAL elbow_joint_cfg_value_;
	FCL_REAL wrist_joint_cfg_value_;
	FCL_REAL finger_joint_cfg_value_;	

	Vec3f finger_joint_transform_to_parent_;
;
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

////////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_SUITE(test_link)

BOOST_FIXTURE_TEST_CASE(test_set_get_name, LinkFixture)
{
	std::string new_name = "new name";

	body_->setName(new_name);

	BOOST_CHECK_EQUAL(new_name, body_->getName() );
}

BOOST_FIXTURE_TEST_CASE(test_add_get_child_joints, LinkFixture)
{
	body_->addChildJoint(shoulder_joint_);
	body_->addChildJoint(elbow_joint_);
	body_->addChildJoint(wrist_joint_);

	std::vector<boost::shared_ptr<Joint> > childs = body_->getChildJoints();

	BOOST_CHECK(std::find(childs.begin(), childs.end(), shoulder_joint_) != childs.end() );
	BOOST_CHECK(std::find(childs.begin(), childs.end(), elbow_joint_) != childs.end() );
	BOOST_CHECK(std::find(childs.begin(), childs.end(), wrist_joint_) != childs.end() );
}

BOOST_FIXTURE_TEST_CASE(test_set_get_parent_joint, LinkFixture)
{
	body_->setParentJoint(finger_joint_);

	BOOST_CHECK_EQUAL(finger_joint_, body_->getParentJoint() );
}

BOOST_FIXTURE_TEST_CASE(test_add_object_and_get_num_objects, LinkFixture)
{
	BOOST_CHECK_EQUAL(0, body_->getNumObjects() );

	boost::shared_ptr<CollisionObject> object_1(new CollisionObject() );
	body_->addObject(object_1);
	BOOST_CHECK_EQUAL(1, body_->getNumObjects() );

	boost::shared_ptr<CollisionObject> object_2(new CollisionObject() );
	body_->addObject(object_2);
	BOOST_CHECK_EQUAL(2, body_->getNumObjects() );

	boost::shared_ptr<CollisionObject> object_3(new CollisionObject() );
	body_->addObject(object_3);
	BOOST_CHECK_EQUAL(3, body_->getNumObjects() );
}

BOOST_FIXTURE_TEST_CASE(test_get_num_child_joints, LinkFixture)
{
	BOOST_CHECK_EQUAL(0, body_->getNumChildJoints() );

	body_->addChildJoint(shoulder_joint_);
	BOOST_CHECK_EQUAL(1, body_->getNumChildJoints() );

	body_->addChildJoint(elbow_joint_);
	BOOST_CHECK_EQUAL(2, body_->getNumChildJoints() );

	body_->addChildJoint(wrist_joint_);
	BOOST_CHECK_EQUAL(3, body_->getNumChildJoints() );
}

BOOST_AUTO_TEST_SUITE_END()
////////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_SUITE(test_joint)

BOOST_FIXTURE_TEST_CASE(test_set_get_name, JointFixture)
{
	std::string new_name = "new name";

	shoulder_joint_->setName(new_name);

	BOOST_CHECK_EQUAL(new_name, shoulder_joint_->getName() );
}

BOOST_FIXTURE_TEST_CASE(test_get_local_transform_on_prismatic_joint, JointFixture)
{
	Matrix3f matrix = Matrix3f::getIdentity();
	Vec3f vector = Vec3f(2, 0 , 0);
	Transform3f transform = Transform3f(matrix, vector);
	Vec3f axis = Vec3f(1, 0, 0);

	boost::shared_ptr<Joint> joint(new PrismaticJoint(body_, arm_, transform, "joint", axis) );

	FCL_REAL joint_cfg_value = 3;
	JointConfig joint_cfg(joint, joint_cfg_value);
	Transform3f local_transform = joint->getLocalTransform(joint_cfg);

	Vec3f expected_vector = vector + (axis * joint_cfg_value); 
	Transform3f expected_transform = Transform3f(matrix, expected_vector);

	BOOST_CHECK_EQUAL(expected_transform, local_transform);

	// Rotation 90° about the axis Z 
	// correct visualization needs to consider direction of Z axis => then X axis take place of Y axis
	matrix = Matrix3f(
		0, -1, 0,
		1, 0, 0,
		0, 0, 1
	);
	transform = Transform3f(matrix, vector);

	boost::shared_ptr<Joint> joint_with_rotation(new PrismaticJoint(body_, arm_, transform, "joint with rotation", axis) );

	JointConfig joint_with_rotation_cfg(joint_with_rotation, joint_cfg_value);
	local_transform = joint_with_rotation->getLocalTransform(joint_with_rotation_cfg);

	expected_vector = vector + (Vec3f(0, 1, 0) * joint_cfg_value); 
	expected_transform = Transform3f(matrix, expected_vector);

	BOOST_CHECK_EQUAL(expected_transform, local_transform);
}

BOOST_FIXTURE_TEST_CASE(test_get_local_transform_on_revolute_joint, JointFixture)
{
	Matrix3f matrix = Matrix3f::getIdentity();
	Vec3f vector = Vec3f(2, 0 , 0);
	Transform3f transform = Transform3f(matrix, vector);
	Vec3f axis = Vec3f(0, 0, 1);

	boost::shared_ptr<Joint> joint(new RevoluteJoint(body_, arm_, transform, "joint", axis) );

	// 90°
	FCL_REAL joint_cfg_value = boost::math::constants::pi<double>() / 2;

	JointConfig joint_cfg(joint, joint_cfg_value);
	Transform3f local_transform = joint->getLocalTransform(joint_cfg);

	Vec3f expected_vector = vector; 
	// Rotation 90° about the axis Z 
	// correct visualization needs to consider direction of Z axis => then X axis take place of Y axis
	Matrix3f expected_matrix = Matrix3f(
		0, -1, 0,
		1, 0, 0,
		0, 0, 1
	);

	Transform3f expected_transform = Transform3f(expected_matrix, expected_vector);

	BOOST_CHECK_EQUAL(expected_transform, local_transform);
}

BOOST_FIXTURE_TEST_CASE(test_get_num_dofs, JointFixture)
{
	BOOST_CHECK_EQUAL(1, shoulder_joint_->getNumDofs() );
}

BOOST_FIXTURE_TEST_CASE(test_set_get_parent_link, JointFixture)
{
	BOOST_CHECK(body_ == shoulder_joint_->getParentLink() );

	shoulder_joint_->setParentLink(finger_);
	BOOST_CHECK(finger_ == shoulder_joint_->getParentLink() );
}

BOOST_FIXTURE_TEST_CASE(test_set_get_child_link, JointFixture)
{
	BOOST_CHECK(arm_ == shoulder_joint_->getChildLink() );

	shoulder_joint_->setChildLink(finger_);
	BOOST_CHECK(finger_ == shoulder_joint_->getChildLink() );
}

BOOST_FIXTURE_TEST_CASE(test_get_joint_type, JointFixture)
{
	BOOST_CHECK_EQUAL(JT_PRISMATIC, shoulder_joint_->getJointType() );
	BOOST_CHECK_EQUAL(JT_REVOLUTE, elbow_joint_->getJointType() );
}

BOOST_FIXTURE_TEST_CASE(test_set_get_transform_to_parent, JointFixture)
{
	Matrix3f matrix = Matrix3f(
		0.0, 0.0, 0.0,
		0.0, 1.0, 0.0,
		0.0, 1.0, 0.0
		);
	Vec3f vector = Vec3f(1, 0 , 0);
	Transform3f transform = Transform3f(matrix, vector);

	shoulder_joint_->setTransformToParent(transform);

	BOOST_CHECK_EQUAL(transform, shoulder_joint_->getTransformToParent() );
}

BOOST_FIXTURE_TEST_CASE(test_get_axis, JointFixture)
{
	Matrix3f matrix = Matrix3f(
		0.0, 0.0, 0.0,
		0.0, 1.0, 0.0,
		0.0, 1.0, 0.0
		);
	Vec3f vector = Vec3f(1, 0 , 0);
	Transform3f transform = Transform3f(matrix, vector);
	Vec3f axis = Vec3f(0, 1, 0);

	boost::shared_ptr<Joint> joint(new PrismaticJoint(body_, arm_, transform, "name", axis) );

	BOOST_CHECK_EQUAL(axis, joint->getAxis() );
}

BOOST_AUTO_TEST_SUITE_END()	
////////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_SUITE(test_joint_config)

BOOST_FIXTURE_TEST_CASE(test_get_dim, JointConfigFixture)
{
	BOOST_CHECK_EQUAL(shoulder_joint_->getNumDofs(), joint_config_->getDim() );
}

BOOST_FIXTURE_TEST_CASE(test_operator_square_brackets, JointConfigFixture)
{
	std::size_t size = joint_config_->getDim();

	for (std::size_t i = 0; i < size; ++i)
	{
		BOOST_CHECK_EQUAL(joint_cfg_value_, (*joint_config_)[i]);

		FCL_REAL new_value = 0.25;
		(*joint_config_)[i] = new_value;

		BOOST_CHECK_EQUAL(new_value, (*joint_config_)[i]);
	}
}

BOOST_FIXTURE_TEST_CASE(test_get_value, JointConfigFixture)
{
	std::size_t size = joint_config_->getDim();

	for (std::size_t i = 0; i < size; ++i)
	{
		BOOST_CHECK_EQUAL(joint_cfg_value_, joint_config_->getValue(i) );

		FCL_REAL new_value = 0.25;
		joint_config_->getValue(i) = new_value;

		BOOST_CHECK_EQUAL(new_value, joint_config_->getValue(i) );
	}
}

BOOST_FIXTURE_TEST_CASE(test_get_limit_min, JointConfigFixture)
{
	std::size_t size = joint_config_->getDim();

	for (std::size_t i = 0; i < size; ++i)
	{
		BOOST_CHECK_EQUAL(joint_cfg_value_min_, joint_config_->getLimitMin(i) );

		FCL_REAL new_value = 0.25;
		joint_config_->getLimitMin(i) = new_value;

		BOOST_CHECK_EQUAL(new_value, joint_config_->getLimitMin(i) );
	}
}

BOOST_FIXTURE_TEST_CASE(test_get_limit_max, JointConfigFixture)
{
	std::size_t size = joint_config_->getDim();

	for (std::size_t i = 0; i < size; ++i)
	{
		BOOST_CHECK_EQUAL(joint_cfg_value_max_, joint_config_->getLimitMax(i) );

		FCL_REAL new_value = 0.25;
		joint_config_->getLimitMax(i) = new_value;

		BOOST_CHECK_EQUAL(new_value, joint_config_->getLimitMax(i) );
	}
}

BOOST_FIXTURE_TEST_CASE(test_get_joint, JointConfigFixture)
{
	BOOST_CHECK_EQUAL(shoulder_joint_, joint_config_->getJoint() );
}

BOOST_AUTO_TEST_SUITE_END()
////////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_SUITE(test_model)

BOOST_FIXTURE_TEST_CASE(test_add_get_joint, ModelFixture)
{
	model_->addJoint(test_joint_);

	boost::shared_ptr<Joint> joint = model_->getJoint(test_joint_name_);

	BOOST_CHECK_EQUAL(test_joint_, joint);
}

BOOST_FIXTURE_TEST_CASE(test_add_get_link, ModelFixture)
{
	model_->addLink(test_link_);

	boost::shared_ptr<Link> link = model_->getLink(test_link_name_);

	BOOST_CHECK_EQUAL(test_link_, link);
}

BOOST_FIXTURE_TEST_CASE(test_get_joint_parent, ModelFixture)
{
	BOOST_CHECK_EQUAL(shoulder_joint_, model_->getJointParent(elbow_joint_));
	BOOST_CHECK_EQUAL(elbow_joint_, model_->getJointParent(wrist_joint_));
	BOOST_CHECK_EQUAL(wrist_joint_, model_->getJointParent(finger_joint_));
}

BOOST_FIXTURE_TEST_CASE(test_get_num_dofs, ModelFixture)
{
	BOOST_CHECK_EQUAL(4, model_->getNumDofs() );
}

BOOST_FIXTURE_TEST_CASE(test_get_num_links, ModelFixture)
{
	BOOST_CHECK_EQUAL(5, model_->getNumLinks() );
}

BOOST_FIXTURE_TEST_CASE(test_get_num_joints, ModelFixture)
{
	BOOST_CHECK_EQUAL(4, model_->getNumJoints() );
}

BOOST_FIXTURE_TEST_CASE(test_get_joint_interpolation_type, ModelFixture)
{
	BOOST_CHECK_EQUAL(LINEAR , model_->getJointInterpolationType(shoulder_joint_name_) );
}

BOOST_FIXTURE_TEST_CASE(test_get_links, ModelFixture)
{
	std::vector<boost::shared_ptr<Link> > links = model_->getLinks();

	BOOST_CHECK_EQUAL(5, links.size() );

	BOOST_CHECK(std::find(links.begin(), links.end(), body_) != links.end() );
	BOOST_CHECK(std::find(links.begin(), links.end(), arm_) != links.end() );
	BOOST_CHECK(std::find(links.begin(), links.end(), forearm_) != links.end() );
	BOOST_CHECK(std::find(links.begin(), links.end(), hand_) != links.end() );
	BOOST_CHECK(std::find(links.begin(), links.end(), finger_) != links.end() );
}

BOOST_FIXTURE_TEST_CASE(test_get_joints, ModelFixture)
{
	std::vector<boost::shared_ptr<Joint> > joints = model_->getJoints();

	BOOST_CHECK_EQUAL(4, joints.size() );

	BOOST_CHECK(std::find(joints.begin(), joints.end(), shoulder_joint_) != joints.end() );
	BOOST_CHECK(std::find(joints.begin(), joints.end(), elbow_joint_) != joints.end() );
	BOOST_CHECK(std::find(joints.begin(), joints.end(), wrist_joint_) != joints.end() );
	BOOST_CHECK(std::find(joints.begin(), joints.end(), finger_joint_) != joints.end() );
}

BOOST_FIXTURE_TEST_CASE(test_get_joints_map, ModelFixture)
{
	 std::map<std::string, boost::shared_ptr<Joint> > joints_map = model_->getJointsMap();

	 BOOST_CHECK_EQUAL(shoulder_joint_, joints_map[shoulder_joint_name_]);
	 BOOST_CHECK_EQUAL(elbow_joint_, joints_map[elbow_joint_name_]);
	 BOOST_CHECK_EQUAL(wrist_joint_, joints_map[wrist_joint_name_]);
	 BOOST_CHECK_EQUAL(finger_joint_, joints_map[finger_joint_name_]);
}

BOOST_AUTO_TEST_SUITE_END()	
////////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_SUITE(test_model_config)

BOOST_FIXTURE_TEST_CASE(test_get_joint_config, ModelConfigFixture)
{
	JointConfig cfg_1 = cfg_start_->getJointConfig(shoulder_joint_name_);
	JointConfig cfg_2 = cfg_start_->getJointConfig(shoulder_joint_);

	BOOST_CHECK(cfg_1 == cfg_2);

	JointConfig& cfg_ref_1 = cfg_start_->getJointConfig(shoulder_joint_name_);
	JointConfig& cfg_ref_2 = cfg_start_->getJointConfig(shoulder_joint_);

	BOOST_CHECK(cfg_ref_1 == cfg_ref_2);

	int cfg_value = 20;
	cfg_ref_1[0] = cfg_value;

	cfg_1 = cfg_start_->getJointConfig(shoulder_joint_name_);

	BOOST_CHECK_EQUAL(cfg_value, cfg_1[0]);
}

BOOST_FIXTURE_TEST_CASE(test_get_joint_cfgs_map, ModelConfigFixture)
{
	std::map<std::string, JointConfig> joint_cfgs_map = cfg_start_->getJointCfgsMap();

	BOOST_CHECK_EQUAL(4, joint_cfgs_map.size() );

	BOOST_CHECK(cfg_start_->getJointConfig(shoulder_joint_name_) == joint_cfgs_map[shoulder_joint_name_]);
	BOOST_CHECK(cfg_start_->getJointConfig(elbow_joint_name_) == joint_cfgs_map[elbow_joint_name_]);
	BOOST_CHECK(cfg_start_->getJointConfig(wrist_joint_name_) == joint_cfgs_map[wrist_joint_name_]);
	BOOST_CHECK(cfg_start_->getJointConfig(finger_joint_name_) == joint_cfgs_map[finger_joint_name_]);
}

BOOST_AUTO_TEST_SUITE_END()
////////////////////////////////////////////////////////////////////////////////
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
////////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_SUITE(test_model_bound)

BOOST_FIXTURE_TEST_CASE(test_get_motion_bound, ModelBoundFixture)
{
	FCL_REAL bound = -1.0;
	FCL_REAL time = 0.0;

	setEndCfgLinkBody();

	bound = model_bound_->getMotionBound(time, direction_);
	BOOST_CHECK_EQUAL(0.0, bound);

	setEndCfgLinkArm();

	bound = model_bound_->getMotionBound(time, direction_);
	BOOST_CHECK_LE(0.0, bound);
}

BOOST_FIXTURE_TEST_CASE(test_get_motion_bound_for_end_cfg_1, ModelBoundFixture)
{
	FCL_REAL bound = -1.0;
	FCL_REAL time = 0.5;	

	setEndCfg_1();

	FCL_REAL angular_velocity = finger_joint_cfg_value_ / 1.0;
	FCL_REAL expected_bound = angular_velocity * distance_from_center_;

	bound = model_bound_->getMotionBound(time, direction_, distance_from_center_);
	BOOST_CHECK_EQUAL(expected_bound, bound);
}

BOOST_FIXTURE_TEST_CASE(test_get_motion_bound_for_end_cfg_2, ModelBoundFixture)
{
	FCL_REAL bound = -1.0;
	FCL_REAL time = 0.5;	

	setEndCfg_2();

	FCL_REAL elbow_joint_angular_velocity = elbow_joint_cfg_value_ / 1.0;
	FCL_REAL finger_joint_angular_velocity = finger_joint_cfg_value_ / 1.0;
	FCL_REAL expected_bound = (elbow_joint_angular_velocity + finger_joint_angular_velocity) * distance_from_center_;

	bound = model_bound_->getMotionBound(time, direction_, distance_from_center_);
	BOOST_CHECK_EQUAL(expected_bound, bound);
}

BOOST_FIXTURE_TEST_CASE(test_get_motion_bound_for_end_cfg_3, ModelBoundFixture)
{
	FCL_REAL bound = -1.0;
	FCL_REAL time = 0.5;	

	setEndCfg_3();

	FCL_REAL expected_bound = wrist_joint_cfg_value_ / 1.0;

	bound = model_bound_->getMotionBound(time, direction_, distance_from_center_);
	BOOST_CHECK_EQUAL(expected_bound, bound);
}

BOOST_FIXTURE_TEST_CASE(test_get_motion_bound_for_end_cfg_4, ModelBoundFixture)
{
	FCL_REAL bound = -1.0;
	FCL_REAL time = 0.5;	

	setEndCfg_4();

	direction_ = shoulder_joint_->getAxis();

	FCL_REAL shoulder_joint_expected_bound = shoulder_joint_cfg_value_ / 1.0;
	FCL_REAL wrist_joint_expected_bound = wrist_joint_cfg_value_ / 1.0;
	FCL_REAL expected_bound = shoulder_joint_expected_bound + wrist_joint_cfg_value_;

	bound = model_bound_->getMotionBound(time, direction_, distance_from_center_);
	BOOST_CHECK_EQUAL(expected_bound, bound);
}

BOOST_FIXTURE_TEST_CASE(test_get_motion_bound_for_end_cfg_5, ModelBoundFixture)
{
	FCL_REAL bound = -1.0;
	FCL_REAL time = 0.5;	

	setEndCfg_5();

	FCL_REAL wrist_joint_expected_bound = wrist_joint_cfg_value_ / 1.0;
	FCL_REAL finger_joint_angular_velocity = finger_joint_cfg_value_ / 1.0;
	FCL_REAL finger_joint_expected_bound = finger_joint_angular_velocity * distance_from_center_;
	FCL_REAL expected_bound = wrist_joint_expected_bound + finger_joint_expected_bound;

	bound = model_bound_->getMotionBound(time, direction_, distance_from_center_);
	BOOST_CHECK_EQUAL(expected_bound, bound);
}

BOOST_FIXTURE_TEST_CASE(test_get_motion_bound_for_end_cfg_6, ModelBoundFixture)
{
	FCL_REAL bound = -1.0;
	FCL_REAL time = 0.5;	

	setEndCfg_6();

	FCL_REAL elbow_joint_angular_velocity = elbow_joint_cfg_value_ / 1.0;
	FCL_REAL finger_joint_angular_velocity = finger_joint_cfg_value_ / 1.0;

	FCL_REAL elbow_joint_expected_bound = elbow_joint_angular_velocity * finger_joint_transform_to_parent_.length();
	FCL_REAL finger_joint_expected_bound = (elbow_joint_angular_velocity + finger_joint_angular_velocity) * distance_from_center_;
	FCL_REAL expected_bound = elbow_joint_expected_bound + finger_joint_expected_bound;

	bound = model_bound_->getMotionBound(time, direction_, distance_from_center_);
	BOOST_CHECK_EQUAL(expected_bound, bound);
}

BOOST_AUTO_TEST_SUITE_END()
////////////////////////////////////////////////////////////////////////////////

}