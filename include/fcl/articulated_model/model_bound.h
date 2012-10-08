#ifndef FCL_MODEL_BOUND_H
#define FCL_MODEL_BOUND_H

#include "fcl/data_types.h"
#include "fcl/math/vec_3f.h"

#include <boost/shared_ptr.hpp>

#include <string>
#include <vector>
#include <map>

namespace fcl 
{

class Model;
class ModelConfig;
class Joint;
class Interpolation;

class ModelBoundCache;

class ModelBound 
{
public:
	ModelBound(boost::shared_ptr<const Model> model, 
		boost::shared_ptr<const ModelConfig> cfg_start, boost::shared_ptr<const ModelConfig> cfg_end);

	// direction must be normalized
	FCL_REAL getMotionBound(const std::string& joint_name, const FCL_REAL time, 
		const Vec3f direction, const FCL_REAL body_max_distance_from_origin = 0);

	FCL_REAL getVectorLengthBoundToParent(const std::string& child_name) const;
	FCL_REAL getVectorLengthBoundToParent(boost::shared_ptr<Joint> child) const;

	Vec3f getLinearVelocityBoundGeneratedByJoint(const std::string& joint_name) const;
	Vec3f getAngularVelocityBoundGeneratedByJoint(const std::string& joint_name) const;

	FCL_REAL getAbsoluteLinearVelocityBoundGeneratedByJoint(const std::string& joint_name) const;
	FCL_REAL getAbsoluteAngularVelocityBoundGeneratedByJoint(const std::string& joint_name) const;	

private:
	void initJointsInterpolations();
	void initJointsParentTree();

	boost::shared_ptr<Interpolation> getInterpolation(const std::string& joint_name) const;	

	bool isJointRevolute(const std::string& joint_name) const;
	bool isJointTranslational(const std::string& joint_name) const;

	bool isJointRevolute(boost::shared_ptr<const Joint> joint) const;
	bool isJointTranslational(boost::shared_ptr<const Joint> joint) const;	

	// order of joints in vector is from last one to root joint
	std::vector<boost::shared_ptr<const Joint> >
		getJointsChainFromLastJoint(const std::string& last_joint_name) const;

	std::string getJointParentName(const std::string& joint_name) const;
	boost::shared_ptr<const Joint> getJointParent(const std::string& joint_name) const;

	boost::shared_ptr<const Joint> pop(std::vector<boost::shared_ptr<const Joint> >& vec);


	FCL_REAL getInterpolationVelocityBound(const std::string& joint_name) const;

	FCL_REAL getRootOneStepMotionBound(const std::string& from_joint_name, const std::string& to_joint_name);
	FCL_REAL getRootOneStepMotionBound(boost::shared_ptr<const Joint> from, boost::shared_ptr<const Joint> to);
	FCL_REAL getOneStepMotionBound(const std::string& from_joint_name, const std::string& to_joint_name);
	FCL_REAL getOneStepMotionBound(boost::shared_ptr<const Joint> from, boost::shared_ptr<const Joint> to);

	FCL_REAL getOneStepBodyMotionBound(const std::string& joint_name, const FCL_REAL body_max_distance_from_origin);
	FCL_REAL getOneStepBodyMotionBound(boost::shared_ptr<const Joint> joint, const FCL_REAL body_max_distance_from_origin);

	FCL_REAL getRootJointAccumulatedAngularBound(const std::string& joint_name);
	FCL_REAL getJointAccumulatedAngularBound(const std::string& joint_name);

	void setCurrentTime(const FCL_REAL time);
	FCL_REAL getCurrentTime() const;

	void setCurrentDirection(const Vec3f direction);
	Vec3f getCurrentDirection() const;

private:	
	FCL_REAL time_;
	Vec3f direction_;

	boost::shared_ptr<const Model> model_;

	boost::shared_ptr<const ModelConfig> cfg_start_;
	boost::shared_ptr<const ModelConfig> cfg_end_;

	std::map<std::string, boost::shared_ptr<Interpolation> > joints_interpolations_;
	std::map<std::string, std::string> joints_parent_tree_;



	std::vector<FCL_REAL> joints_accumulated_angular_bound_;

	//ModelBoundCache cache_;
};

//class ModelBoundCache
//{
//public:
//	ModelBoundCache();
//
//	
//private:
//};

}

#endif 