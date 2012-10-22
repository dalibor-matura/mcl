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
	ModelBound(boost::shared_ptr<Model> model, boost::shared_ptr<const ModelConfig> cfg_start, 
		boost::shared_ptr<const ModelConfig> cfg_end, boost::shared_ptr<const Link> bounded_link);

	// direction must be normalized
	FCL_REAL getMotionBound(const FCL_REAL& time, const Vec3f& direction, 
		const FCL_REAL max_distance_from_joint_center = 0);	

private:	
	void initJointsChain();
	const std::vector<boost::shared_ptr<const Joint> >& getJointsChain() const;

	// get parent joint of bounded link
	boost::shared_ptr<Joint> getLastJoint() const;

	// order of joints in vector is from last one to root joint
	std::vector<boost::shared_ptr<const Joint> >
		getJointsChainFromLastJoint(const boost::shared_ptr<const Joint>& last_joint) const;				

	// joints are ordered from last joint to root joint
	FCL_REAL getJointsChainMotionBound() const;

	// returns motion bound of the joint in its parent's frame 
	FCL_REAL getMotionBoundInParentFrame(const boost::shared_ptr<const Joint>& joint) const;
	FCL_REAL getDirectionalMotionBoundInParentFrame(const boost::shared_ptr<const Joint>& joint) const;
	FCL_REAL getSimpleMotionBoundInParentFrame(const boost::shared_ptr<const Joint>& joint) const;

	FCL_REAL getObjectMotionBoundInLastJointFrame(const FCL_REAL max_distance_from_joint_center) const;

	bool isRoot(const boost::shared_ptr<const Joint>& joint) const;	

	FCL_REAL getAccumulatedAngularBound(const boost::shared_ptr<const Joint>& joint, bool is_directional = false) const;	

	void setCurrentDirection(const Vec3f& direction);
	Vec3f getCurrentDirection() const;

	void resetAngularBoundAccumulation();
	void addAngularBoundAccumulation(const FCL_REAL& accumulation) const;
	FCL_REAL getAngularBoundAccumulation() const;

private:		
	Vec3f direction_;
	FCL_REAL max_distance_; // distance from center of last joint to the furthermost point of the collision object	

	boost::shared_ptr<const Model> model_;
	boost::shared_ptr<const ModelConfig> cfg_start_;
	boost::shared_ptr<const ModelConfig> cfg_end_;
	boost::shared_ptr<const Link> bounded_link_;

	boost::shared_ptr<JointBoundInfo> joint_bound_info_;
	std::vector<boost::shared_ptr<const Joint> > joints_chain_;

	// TODO: find out better solution then using of mutable
	// Maybe could be changed together with adding cache mechanism for all calculations
	mutable FCL_REAL accumulated_angular_bound_;
};

}

#endif 