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
class Link;
class Joint;

class JointBoundInfo;

class ModelBound 
{
public:
	ModelBound(boost::shared_ptr<Model> model, 
		boost::shared_ptr<const ModelConfig> cfg_start, boost::shared_ptr<const ModelConfig> cfg_end);

	// direction must be normalized
	FCL_REAL getMotionBound(const std::string& link_name, const FCL_REAL& time, 
		const Vec3f& direction, const FCL_REAL max_distance_from_joint_center = 0);	

private:	
	void initJointsParentTree();

	void constructJointParentTree(
		std::map<std::string, std::string>& joint_parent_tree, boost::shared_ptr<const Link>& link);

	// order of joints in vector is from last one to root joint
	std::vector<boost::shared_ptr<const Joint> >
		getJointsChainFromLastJoint(const std::string& last_joint_name) const;	

	boost::shared_ptr<const Joint> getJointParent(const boost::shared_ptr<const Joint>& joint) const;
	std::string getJointParentName(const std::string& joint_name) const;	

	// joints are ordered from last joint to root joint
	FCL_REAL getJointsChainMotionBound(std::vector<boost::shared_ptr<const Joint> >& joints_chain) const;

	// returns motion bound of the joint in its parent's frame 
	FCL_REAL getMotionBoundInParentFrame(boost::shared_ptr<const Joint>& joint) const;
	FCL_REAL getDirectionalMotionBoundInParentFrame(boost::shared_ptr<const Joint>& joint) const;
	FCL_REAL getSimpleMotionBoundInParentFrame(boost::shared_ptr<const Joint>& joint) const;

	FCL_REAL getObjectMotionBoundInJointFrame(boost::shared_ptr<const Joint>& joint, 
		const FCL_REAL max_distance_from_joint_center);

	bool isRoot(const boost::shared_ptr<const Joint>& joint) const;	

	FCL_REAL getAccumulatedAngularBound(boost::shared_ptr<const Joint>& joint, bool is_directional = false) const;	

	boost::shared_ptr<const Joint> pop(std::vector<boost::shared_ptr<const Joint> >& vec) const;	

	void setCurrentDirection(const Vec3f& direction);
	Vec3f getCurrentDirection() const;

	void resetAngularBoundAccumulation();
	void addAngularBoundAccumulation(const FCL_REAL& accumulation) const;
	FCL_REAL getAngularBoundAccumulation() const;

private:		
	Vec3f direction_;
	FCL_REAL max_distance_; // distance from center of last joint to the furthermost point of the collision object
	boost::shared_ptr<JointBoundInfo> joint_bound_info_;

	boost::shared_ptr<Model> model_;
	boost::shared_ptr<const ModelConfig> cfg_start_;
	boost::shared_ptr<const ModelConfig> cfg_end_;

	std::map<std::string, std::string> joint_parent_tree_;

	// TODO: find out better solution then using of mutable
	// Maybe could be changed together with adding cache mechanism for all calculations
	mutable FCL_REAL accumulated_angular_bound_;
};

}

#endif 