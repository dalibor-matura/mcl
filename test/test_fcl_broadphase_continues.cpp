#define BOOST_TEST_MODULE "FCL_BROADPHASE_CONTINUES_COLLISION"
#include <boost/test/unit_test.hpp>

#include "fcl/broadphase/broadphase.h"
#include "fcl/broadphase/broadphase_bruteforce_continues.h"

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include <cmath>

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
#include "fcl/math/matrix_3f.h"
#include "fcl/collision_node.h"
#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"

namespace fcl {

bool equalWithEpsilon(FCL_REAL first, FCL_REAL second, FCL_REAL epsilon)
{
	FCL_REAL difference = fabs(second - first);

	return difference <= epsilon;
}

class CollisionResultInfo
{
public:
	CollisionResultInfo() :
		number_of_contacts_(0),
			time_of_contact_(0.0),
			error_margin_(0.0)
		{
		}

		CollisionResultInfo(int number_of_contacts, FCL_REAL time_of_contact, FCL_REAL error_margin = 0.0) :
		number_of_contacts_(number_of_contacts),
			time_of_contact_(time_of_contact),
			error_margin_(error_margin)
		{
		}

		void setNumberOfContacts(int number_of_contacts)
		{
			number_of_contacts_ = number_of_contacts;
		}

		int getNumberOfContact() const
		{
			return number_of_contacts_ ;
		}

		void setTimeOfContact(FCL_REAL time_of_contact)
		{
			time_of_contact_ = time_of_contact;
		}

		FCL_REAL getTimeOfContact() const
		{
			return time_of_contact_;
		}

		void setErrorMargin(FCL_REAL error_margin)
		{
			error_margin_ = error_margin;
		}

		FCL_REAL getErrorMargin() const
		{
			return error_margin_;
		}

private:
	int number_of_contacts_;
	FCL_REAL time_of_contact_;

	FCL_REAL error_margin_;
};

class ContinuesCollisionManagersList
{
public:
	ContinuesCollisionManagersList() {}
	
	void addManager(boost::shared_ptr<BroadPhaseContinuousCollisionManager> manager)
	{
		managers_.push_back(manager);
	}

	void addContinuousCollisionObject(
		boost::shared_ptr<ContinuousCollisionObject>& continues_collision_object)
	{
		continues_collision_objects_.push_back(continues_collision_object);
	}

	void initAllManagers()
	{
		BOOST_FOREACH(boost::shared_ptr<BroadPhaseContinuousCollisionManager>& manager, managers_)
		{
			initManager(manager);
			manager->setup();
		}
	}

	void collisionTest(int expected_contacts = 1)
	{
		BOOST_FOREACH(boost::shared_ptr<BroadPhaseContinuousCollisionManager>& manager, managers_)
		{
			collision(manager, expected_contacts);
		}
	}

	void clear()
	{
		managers_.clear();
		continues_collision_objects_.clear();
	}

private:
	void initManager(boost::shared_ptr<BroadPhaseContinuousCollisionManager>& manager)
	{
		registerObjects(manager);		
	}

	void registerObjects(boost::shared_ptr<BroadPhaseContinuousCollisionManager>& manager)
	{
		manager->clear();

		BOOST_FOREACH(boost::shared_ptr<ContinuousCollisionObject>& continues_collision_object, continues_collision_objects_)
		{
			manager->registerObject(continues_collision_object.get() );
		}					
	}

	void collision(boost::shared_ptr<BroadPhaseContinuousCollisionManager>& manager, int expected_contacts = 1)
	{
		CollisionData collision_data;	
		collision_data.request.num_max_contacts = 1;

		manager->collide(&collision_data, defaultContinuousCollisionFunction);

		int contacts_number = collision_data.result.numContacts();

		BOOST_CHECK_EQUAL(expected_contacts, contacts_number);
	}

private:
	std::vector<boost::shared_ptr<BroadPhaseContinuousCollisionManager> > managers_;
	std::vector<boost::shared_ptr<ContinuousCollisionObject> > continues_collision_objects_;
};

class ArticularCollisionFixture
{
public:
	ArticularCollisionFixture() :
	  model_environment_(new BVHModel<RSS>() ),
	  model_robot_(new BVHModel<RSS>() )
	{
		initGeneral();
		initLinkBound();
		initMotions();
		initCollisionObjectsOnly();
		initManagersList();
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

		FCL_REAL max_side = 1300;

		articular_motion_top_.reset(new ArticularMotion(link_bound_) );
		articular_motion_bottom_.reset(new ArticularMotion(link_bound_) );
		articular_motion_left_.reset(new ArticularMotion(link_bound_) );
		articular_motion_right_.reset(new ArticularMotion(link_bound_) );
		articular_motion_front_.reset(new ArticularMotion(link_bound_) );
		articular_motion_behind_.reset(new ArticularMotion(link_bound_) );

		// TODO: reference points must be rethought and implement somehow differently
		// doesn't work as wanted
		/*articular_motion_top_->setReferencePoint(Vec3f(0, max_side, 0) );
		articular_motion_bottom_->setReferencePoint(Vec3f(0, -max_side, 0) );
		articular_motion_left_->setReferencePoint(Vec3f(-max_side, 0, 0) );
		articular_motion_right_->setReferencePoint(Vec3f(max_side, 0, 0) );
		articular_motion_front_->setReferencePoint(Vec3f(0, 0, max_side) );
		articular_motion_behind_->setReferencePoint(Vec3f(0, 0, -max_side) );	*/	
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
		initCollisionObjectsHelper();

		model_environment_ = createModel("env.obj");
		model_robot_ = createModel("rob.obj");

		model_robot_top_ = createModel("rob_top.obj");
	}

	void initCollisionObjectsHelper()
	{
		std::vector<Vec3f> p1, p2;
		std::vector<Triangle> t1, t2;
		boost::filesystem::path path(TEST_RESOURCES_DIR);

		loadOBJFile((path / "env.obj").string().c_str(), p1, t1);
		loadOBJFile((path / "rob.obj").string().c_str(), p2, t2);	

		SplitMethodType split_method = SPLIT_METHOD_MEAN;
		model_environment_0_.bv_splitter.reset(new BVSplitter<RSS>(split_method));
		model_robot_0_.bv_splitter.reset(new BVSplitter<RSS>(split_method));
		model_environment_->bv_splitter.reset(new BVSplitter<RSS>(split_method));
		model_robot_->bv_splitter.reset(new BVSplitter<RSS>(split_method));

		model_environment_0_.beginModel();
		model_environment_0_.addSubModel(p1, t1);
		model_environment_0_.endModel();

		model_robot_0_.beginModel();
		model_robot_0_.addSubModel(p2, t2);
		model_robot_0_.endModel();
	}

	boost::shared_ptr<BVHModel<RSS> > createModel(std::string object_name)
	{
		boost::shared_ptr<BVHModel<RSS>> model(new BVHModel<RSS>() );

		std::vector<Vec3f> p;
		std::vector<Triangle> t;
		boost::filesystem::path path(TEST_RESOURCES_DIR);

		loadOBJFile((path / object_name).string().c_str(), p, t);

		SplitMethodType split_method = SPLIT_METHOD_MEAN;
		model->bv_splitter.reset(new BVSplitter<RSS>(split_method));

		model->beginModel();
		model->addSubModel(p, t);
		model->endModel();

		return model;
	}

	void initManagersList()
	{
		initContinuousCollisonObjectsOnly();

		managers_list_.clear();
		managers_list_.addManager(boost::make_shared<NaiveContinuesCollisionManager>() );

		managers_list_.addContinuousCollisionObject(continuous_collision_environment_);
		managers_list_.addContinuousCollisionObject(continuous_collision_robot_);

		managers_list_.addContinuousCollisionObject(continuous_collision_robot_top_);
		/*managers_list_.addContinuousCollisionObject(continuous_collision_robot_bottom_);
		managers_list_.addContinuousCollisionObject(continuous_collision_robot_left_);
		managers_list_.addContinuousCollisionObject(continuous_collision_robot_right_);
		managers_list_.addContinuousCollisionObject(continuous_collision_robot_front_);
		managers_list_.addContinuousCollisionObject(continuous_collision_robot_behind_);*/

		managers_list_.initAllManagers();	
	}

	void initContinuousCollisonObjectsOnly()
	{
		// Set mutually outer geometries
		model_environment_->addOuterGeometry(model_robot_.get() );
		model_robot_->addOuterGeometry(model_environment_.get() );	

		// There are 6 ContinuesCollisionObjects can collide with model_robot_
		model_robot_->addOuterGeometry(model_robot_top_.get() );	
		model_robot_top_->addOuterGeometry(model_robot_.get() );	

		continuous_collision_environment_.reset(
			new ContinuousCollisionObject(model_environment_, interp_motion_) );

		continuous_collision_robot_.reset(
			new ContinuousCollisionObject(model_robot_, articular_motion_) );

		continuous_collision_robot_top_.reset(
			new ContinuousCollisionObject(model_robot_top_, articular_motion_top_) );
		continuous_collision_robot_bottom_.reset(
			new ContinuousCollisionObject(model_robot_, articular_motion_bottom_) );
		continuous_collision_robot_left_.reset(
			new ContinuousCollisionObject(model_robot_, articular_motion_left_) );
		continuous_collision_robot_right_.reset(
			new ContinuousCollisionObject(model_robot_, articular_motion_right_) );
		continuous_collision_robot_front_.reset(
			new ContinuousCollisionObject(model_robot_, articular_motion_front_) );
		continuous_collision_robot_behind_.reset(
			new ContinuousCollisionObject(model_robot_, articular_motion_behind_) );
	}

	void setNewConfigurations(boost::shared_ptr<ModelConfig>& cfg_start,
		boost::shared_ptr<ModelConfig>& cfg_end)
	{
		cfg_start_ = cfg_start;
		cfg_end_ = cfg_end;

		initMovementOnly();

		link_bound_.reset(new LinkBound(model_, movement_, finger_) );

		initArticularMotion();
		initManagersList();
	}

	void BroadphaseContinuousCollisionManagerTest(int expected_contacts)
	{
		managers_list_.collisionTest(expected_contacts);
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

	void setConfigurationsForTimeOfContactTest(FCL_REAL variance = 1000)
	{
		boost::shared_ptr<ModelConfig> cfg_start (new ModelConfig(model_) );
		boost::shared_ptr<ModelConfig> cfg_end (new ModelConfig(model_) );

		const FCL_REAL environment_bottom_z = -14.5;
		const FCL_REAL robot_top_z = 425.0;

		const FCL_REAL robot_top_on_environment_bottom = environment_bottom_z - robot_top_z;

		// Environment and Robot objects are NOT in collision
		cfg_start->getJointConfig(shoulder_joint_)[0] = robot_top_on_environment_bottom - variance;
		cfg_start->getJointConfig(elbow_joint_)[0] = 0;
		cfg_start->getJointConfig(wrist_joint_)[0] = 0;
		cfg_start->getJointConfig(finger_joint_)[0] = 0;

		cfg_end->getJointConfig(shoulder_joint_)[0] = robot_top_on_environment_bottom + variance;
		cfg_end->getJointConfig(elbow_joint_)[0] = 0;
		cfg_end->getJointConfig(wrist_joint_)[0] = 0;
		cfg_end->getJointConfig(finger_joint_)[0] = 0;

		// path contains collision
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

	int performSimpleContinuousCollision()
	{		
		CollisionResultInfo collision_result_info = performContinuousCollision();

		return collision_result_info.getNumberOfContact();
	}

	int performSimpleDenseDiscreteCollision()
	{		
		CollisionResultInfo collision_result_info = performDenseDiscreteCollision();

		return collision_result_info.getNumberOfContact();
	}

	int performGuardedCollision(FCL_REAL density_ratio = 1.0)
	{
		CollisionResultInfo discrete_result_info = performDenseDiscreteCollision(density_ratio);
		CollisionResultInfo continues_result_info = performContinuousCollision();		

		FCL_REAL epsilon = 
			std::max(continues_result_info.getErrorMargin(), discrete_result_info.getErrorMargin() );

		std::cout << "Continues number of contacts: " << continues_result_info.getNumberOfContact() <<
			" Discrete number of contacts: " << discrete_result_info.getNumberOfContact() << std::endl;

		BOOST_CHECK_EQUAL(
			continues_result_info.getNumberOfContact(), discrete_result_info.getNumberOfContact() );

		if (continues_result_info.getNumberOfContact() > 0)
		{
			std::cout << "Continues toc: " << continues_result_info.getTimeOfContact() <<
				" Discrete toc: " << discrete_result_info.getTimeOfContact() << std::endl;

			BOOST_CHECK_LE(continues_result_info.getTimeOfContact(), 
				discrete_result_info.getTimeOfContact() );

			BOOST_CHECK(equalWithEpsilon(continues_result_info.getTimeOfContact(),
				discrete_result_info.getTimeOfContact(), epsilon) );
		}

		return continues_result_info.getNumberOfContact();
	}

	CollisionResultInfo performContinuousCollision()
	{		
		CollisionRequest collision_request;
		CollisionResult collision_result;
		FCL_REAL time_of_contact = 0.0;
		int number_of_contacts = 0;

		articular_motion_->integrate(0.0);
		interp_motion_->integrate(0.0);

		number_of_contacts = 
			conservativeAdvancement<RSS, MeshConservativeAdvancementTraversalNodeRSS, MeshCollisionTraversalNodeRSS>(
			//&model_robot_0_, articular_motion_.get(),
			//&model_environment_0_, interp_motion_.get(),
			model_robot_.get(), articular_motion_.get(),
			model_environment_.get(), interp_motion_.get(),
			collision_request, collision_result, time_of_contact
			);

		return CollisionResultInfo(number_of_contacts, time_of_contact, 
			conservative_advancement_error_margin);		
	}

	CollisionResultInfo performDenseDiscreteCollision(FCL_REAL density_ratio = 1.0) const
	{
		FCL_REAL time_step = getBestTimeStep(density_ratio);

		CollisionResultInfo collision_result_info;
		collision_result_info.setErrorMargin(time_step);

		for (FCL_REAL time = 0.0; time <= 1.0; time += time_step)
		{
			if (testCollision(time, collision_result_info) )
			{
				return collision_result_info;
			}
		}

		return collision_result_info;
	}

private:
	FCL_REAL getBestTimeStep(FCL_REAL density_ratio = 1.0) const
	{
		FCL_REAL max_movement = 0.0;

		const std::map<std::string, JointConfig>& 
			start_joints_cfg_map = cfg_start_->getJointCfgsMap();
		const std::map<std::string, JointConfig>& 
			end_joints_cfg_map = cfg_end_->getJointCfgsMap();

		std::map<std::string, JointConfig>::const_iterator start_cfg_it, end_cfg_it;

		for (start_cfg_it = start_joints_cfg_map.begin(), end_cfg_it = end_joints_cfg_map.begin();
			start_cfg_it != start_joints_cfg_map.end(); ++start_cfg_it, ++end_cfg_it)
		{
			assignBestTimeStep(max_movement, start_cfg_it->second, end_cfg_it->second);
		}

		if (max_movement == 0.0)
		{
			return 1.0;
		}

		return 1.0 / (max_movement * density_ratio / dense_discrete_collision_movement_step );
	}

	void assignBestTimeStep(FCL_REAL& max_movement, 
		const JointConfig& first, const JointConfig& second) const
	{
		const FCL_REAL abs_difference = fabs(second.getValue(0) - first.getValue(0) );

		if (abs_difference > max_movement)
		{
			max_movement = abs_difference;
		}
	}

	bool testCollision(FCL_REAL time, CollisionResultInfo& collision_result_info) const
	{
		articular_motion_->integrate(time);
		interp_motion_->integrate(time);

		Transform3f tf1, tf2;
		articular_motion_->getCurrentTransform(tf1);
		interp_motion_->getCurrentTransform(tf2);

		MeshCollisionTraversalNodeRSS cnode;
		CollisionRequest request;
		CollisionResult result;

		bool initialize_ok = 
			initialize(cnode, model_robot_0_, tf1, model_environment_0_, tf2, request, result);
		//	initialize<fcl::RSS>(cnode, *(model_robot_.get() ), tf1, *(model_environment_.get() ), tf2, request, result);

		BOOST_REQUIRE(initialize_ok);

		relativeTransform(tf1.getRotation(), tf1.getTranslation(), tf2.getRotation(), tf2.getTranslation(), cnode.R, cnode.T);

		cnode.enable_statistics = false;
		cnode.request = request;

		collide(&cnode);

		int number_of_contacts = result.numContacts();

		if (number_of_contacts > 0)
		{
			collision_result_info.setNumberOfContacts(number_of_contacts);
			collision_result_info.setTimeOfContact(time);

			return true;
		}

		return false;
	}

protected:
	boost::shared_ptr<ArticularMotion> articular_motion_;

	boost::shared_ptr<ArticularMotion> articular_motion_top_;
	boost::shared_ptr<ArticularMotion> articular_motion_bottom_;
	boost::shared_ptr<ArticularMotion> articular_motion_left_;
	boost::shared_ptr<ArticularMotion> articular_motion_right_;
	boost::shared_ptr<ArticularMotion> articular_motion_front_;
	boost::shared_ptr<ArticularMotion> articular_motion_behind_;
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

	BVHModel<RSS> model_environment_0_;
	BVHModel<RSS> model_robot_0_;

	boost::shared_ptr<BVHModel<RSS> > model_environment_;
	boost::shared_ptr<BVHModel<RSS> > model_robot_;

	boost::shared_ptr<BVHModel<RSS> > model_robot_top_;
	boost::shared_ptr<BVHModel<RSS> > model_robot_bottom_;
	boost::shared_ptr<BVHModel<RSS> > model_robot_left_;
	boost::shared_ptr<BVHModel<RSS> > model_robot_right_;
	boost::shared_ptr<BVHModel<RSS> > model_robot_front_;
	boost::shared_ptr<BVHModel<RSS> > model_robot_behind_;

	boost::shared_ptr<ContinuousCollisionObject> continuous_collision_environment_;
	boost::shared_ptr<ContinuousCollisionObject> continuous_collision_robot_;

	boost::shared_ptr<ContinuousCollisionObject> continuous_collision_robot_top_;
	boost::shared_ptr<ContinuousCollisionObject> continuous_collision_robot_bottom_;
	boost::shared_ptr<ContinuousCollisionObject> continuous_collision_robot_left_;
	boost::shared_ptr<ContinuousCollisionObject> continuous_collision_robot_right_;
	boost::shared_ptr<ContinuousCollisionObject> continuous_collision_robot_front_;
	boost::shared_ptr<ContinuousCollisionObject> continuous_collision_robot_behind_;

	ContinuesCollisionManagersList managers_list_;

private:
	static const FCL_REAL dense_discrete_collision_movement_step;
	static const FCL_REAL conservative_advancement_error_margin;
};

const FCL_REAL ArticularCollisionFixture::dense_discrete_collision_movement_step = 0.5;
const FCL_REAL ArticularCollisionFixture::conservative_advancement_error_margin = 0.00001;

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
BOOST_AUTO_TEST_SUITE(test_broad_phase_linear)

BOOST_FIXTURE_TEST_CASE(test_broadphase_managers_linear_interpolation, ArticularCollisionFixture)
{
	setJointsInterpolation(boost::make_shared<const InterpolationLinearData>() );

	std::cout << "CONFIGURATION 1" << std::endl;
	setConfigurations_1();
	BroadphaseContinuousCollisionManagerTest(1);

	std::cout << "CONFIGURATION 2" << std::endl;
	setConfigurations_2();
	BroadphaseContinuousCollisionManagerTest(1);

	std::cout << "CONFIGURATION 3" << std::endl;
	setConfigurations_3();
	BroadphaseContinuousCollisionManagerTest(0);

	std::cout << "CONFIGURATION 4" << std::endl;
	setConfigurations_4();
	BroadphaseContinuousCollisionManagerTest(0);

	std::cout << "CONFIGURATION 5" << std::endl;
	setConfigurations_5();
	BroadphaseContinuousCollisionManagerTest(1);

	std::cout << "CONFIGURATION 6" << std::endl;
	setConfigurations_6();
	BroadphaseContinuousCollisionManagerTest(0);
}

BOOST_FIXTURE_TEST_CASE(test_correctnes_linear_interpolation, ArticularCollisionFixture)
{
	int number_of_contacts = 0;

	setJointsInterpolation(boost::make_shared<const InterpolationLinearData>() );

	setConfigurations_1();
	number_of_contacts = performGuardedCollision();
	BOOST_CHECK_EQUAL(1, number_of_contacts);

	setConfigurations_2();
	number_of_contacts = performGuardedCollision();
	BOOST_CHECK_EQUAL(1, number_of_contacts);

	setConfigurations_3();
	number_of_contacts = performGuardedCollision();
	BOOST_CHECK_EQUAL(0, number_of_contacts);

	setConfigurations_4();
	number_of_contacts = performGuardedCollision();
	BOOST_CHECK_EQUAL(0, number_of_contacts);

	setConfigurations_5();
	number_of_contacts = performGuardedCollision();
	BOOST_CHECK_EQUAL(1, number_of_contacts);

	setConfigurations_6();
	number_of_contacts = performGuardedCollision();
	BOOST_CHECK_EQUAL(0, number_of_contacts);
}
	
BOOST_AUTO_TEST_SUITE_END()
////////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_SUITE(test_broad_phase_third_order)

BOOST_FIXTURE_TEST_CASE(test_broadphase_managers_third_order_interpolation, ArticularCollisionFixture)
{
	setJointsInterpolation(boost::make_shared<const InterpolationThirdOrderData>(10, 10, 10) );

	std::cout << "CONFIGURATION 1" << std::endl;
	setConfigurations_1();
	BroadphaseContinuousCollisionManagerTest(1);

	std::cout << "CONFIGURATION 2" << std::endl;
	setConfigurations_2();
	BroadphaseContinuousCollisionManagerTest(1);

	std::cout << "CONFIGURATION 3" << std::endl;
	setConfigurations_3();
	BroadphaseContinuousCollisionManagerTest(0);

	std::cout << "CONFIGURATION 4" << std::endl;
	setConfigurations_4();
	BroadphaseContinuousCollisionManagerTest(0);

	std::cout << "CONFIGURATION 5" << std::endl;
	setConfigurations_5();
	BroadphaseContinuousCollisionManagerTest(1);

	std::cout << "CONFIGURATION 6" << std::endl;
	setConfigurations_6();
	BroadphaseContinuousCollisionManagerTest(0);
}

BOOST_FIXTURE_TEST_CASE(test_correctnes_third_order_interpolation, ArticularCollisionFixture)
{
	int number_of_contacts = 0;

	setJointsInterpolation(boost::make_shared<const InterpolationThirdOrderData>(10, 10, 10) );

	setConfigurations_1();
	number_of_contacts = performGuardedCollision();
	BOOST_CHECK_EQUAL(1, number_of_contacts);

	setConfigurations_2();
	number_of_contacts = performGuardedCollision();
	BOOST_CHECK_EQUAL(1, number_of_contacts);

	setConfigurations_3();
	number_of_contacts = performGuardedCollision();
	BOOST_CHECK_EQUAL(0, number_of_contacts);

	setConfigurations_4();
	number_of_contacts = performGuardedCollision();
	BOOST_CHECK_EQUAL(0, number_of_contacts);

	setConfigurations_5();
	number_of_contacts = performGuardedCollision();
	BOOST_CHECK_EQUAL(1, number_of_contacts);

	setConfigurations_6();
	number_of_contacts = performGuardedCollision();
	BOOST_CHECK_EQUAL(0, number_of_contacts);
}

BOOST_AUTO_TEST_SUITE_END()
	////////////////////////////////////////////////////////////////////////////////

}