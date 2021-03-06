/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jia Pan */

#ifndef FCL_TRAVERSAL_NODE_BASE_H
#define FCL_TRAVERSAL_NODE_BASE_H

#include "fcl/data_types.h"
#include "fcl/math/transform.h"
#include "fcl/collision_data.h"

namespace fcl
{

/// @brief Node structure encoding the information required for traversal.
class TraversalNodeBase
{
public:
  virtual ~TraversalNodeBase();

  virtual void preprocess() {}
  
  virtual void postprocess() {}

  /// @brief Whether bv_node_id is a leaf node in the first BVH tree 
  virtual bool isFirstNodeLeaf(int bv_node_id) const;

  /// @brief Whether bv_node_id is a leaf node in the second BVH tree
  virtual bool isSecondNodeLeaf(int bv_node_id) const;

  /// @brief Traverse the subtree of the node in the first tree first
  virtual bool firstOverSecond(int bv_node1_id, int bv_node2_id) const;

  /// @brief Get the left child of the node bv_node_id in the first tree
  virtual int getFirstLeftChild(int bv_node_id) const;

  /// @brief Get the right child of the node bv_node_id in the first tree
  virtual int getFirstRightChild(int bv_node_id) const;

  /// @brief Get the left child of the node bv_node_id in the second tree
  virtual int getSecondLeftChild(int bv_node_id) const;

  /// @brief Get the right child of the node bv_node_id in the second tree
  virtual int getSecondRightChild(int bv_node_id) const;

  /// @brief Enable statistics (verbose mode)
  virtual void enableStatistics(bool enable) = 0;

  /// @brief configuation of first object
  Transform3f tf1;

  /// @brief configuration of second object
  Transform3f tf2;
};

/// @brief Node structure encoding the information required for collision traversal.
class CollisionTraversalNodeBase : public TraversalNodeBase
{
public:
  CollisionTraversalNodeBase() : result(NULL), enable_statistics(false) {}

  virtual ~CollisionTraversalNodeBase();

  /// @brief BV test between bv_node1_id and bv_node2_id
  virtual bool BVTesting(int bv_node1_id, int bv_node2_id) const;

  /// @brief Leaf test between node bv_node1_id and bv_node2_id, if they are both leafs
  virtual void leafTesting(int bv_node1_id, int bv_node2_id) const;

  /// @brief Check whether the traversal can stop
  virtual bool canStop() const;

  /// @brief Whether store some statistics information during traversal
  void enableStatistics(bool enable) { enable_statistics = enable; }

  /// @brief request setting for collision
  CollisionRequest request;

  /// @brief collision result kept during the traversal iteration
  CollisionResult* result;

  /// @brief Whether stores statistics 
  bool enable_statistics;
};

/// @brief Node structure encoding the information required for distance traversal.
class DistanceTraversalNodeBase : public TraversalNodeBase
{
public:
	DistanceTraversalNodeBase() : 
		result(NULL),
		enable_statistics(false),
		can_stop_distance_(std::numeric_limits<FCL_REAL>::max() ),
		whole_distance_(std::numeric_limits<FCL_REAL>::max() )
	{}

  virtual ~DistanceTraversalNodeBase();

  /// @brief BV test between bv_node1_id and bv_node2_id
  virtual FCL_REAL BVTesting(int bv_node1_id, int bv_node2_id) const;

  /// @brief Leaf test between node bv_node1_id and bv_node2_id, if they are both leafs
  virtual void leafTesting(int bv_node1_id, int bv_node2_id) const;

  /// @brief Check whether the traversal can stop
  virtual bool canStop(FCL_REAL c) const;

  /// @brief Whether store some statistics information during traversal
  void enableStatistics(bool enable) { enable_statistics = enable; }

  /// @brief request setting for distance
  DistanceRequest request;

  /// @brief distance result kept during the traversal iteration
  DistanceResult* result;

  /// @brief Whether stores statistics 
  bool enable_statistics;

  // Helpers that need refactoring
  FCL_REAL can_stop_distance_;
  FCL_REAL whole_distance_;

  /// @brief collision object 1
  const CollisionGeometry* o1;

  /// @brief collision object 2
  const CollisionGeometry* o2;
};

}

#endif
