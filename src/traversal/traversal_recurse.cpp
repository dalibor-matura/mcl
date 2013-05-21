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


#include "fcl/traversal/traversal_recurse.h"

namespace fcl
{
void collisionRecurse(CollisionTraversalNodeBase* node, int bv_node1_id, int bv_node2_id, BVHFrontList* front_list)
{
  bool is_first_node_leaf = node->isFirstNodeLeaf(bv_node1_id);
  bool is_second_node_leaf = node->isSecondNodeLeaf(bv_node2_id);

  if(is_first_node_leaf && is_second_node_leaf)
  {
    updateFrontList(front_list, bv_node1_id, bv_node2_id);

    if(node->BVTesting(bv_node1_id, bv_node2_id)) return;

    node->leafTesting(bv_node1_id, bv_node2_id);
    return;
  }

  if(node->BVTesting(bv_node1_id, bv_node2_id))
  {
    updateFrontList(front_list, bv_node1_id, bv_node2_id);
    return;
  }

  if(node->firstOverSecond(bv_node1_id, bv_node2_id))
  {
    int left_child = node->getFirstLeftChild(bv_node1_id);    
    collisionRecurse(node, left_child, bv_node2_id, front_list);

    // early stop is disabled if front_list is used
    if(node->canStop() && !front_list) return;

    int right_child = node->getFirstRightChild(bv_node1_id);
    collisionRecurse(node, right_child, bv_node2_id, front_list);
  }
  else
  {
    int left_child = node->getSecondLeftChild(bv_node2_id); 
    collisionRecurse(node, bv_node1_id, left_child, front_list);

    // early stop is disabled is front_list is used
    if(node->canStop() && !front_list) return;

    int right_child = node->getSecondRightChild(bv_node2_id);
    collisionRecurse(node, bv_node1_id, right_child, front_list);
  }
}

void collisionRecurse(MeshCollisionTraversalNodeOBB* node, int bv_node1_id, int bv_node2_id, 
  const Matrix3f& R, const Vec3f& T, BVHFrontList* front_list)
{
  bool is_first_node_leaf = node->isFirstNodeLeaf(bv_node1_id);
  bool is_second_node_leaf = node->isSecondNodeLeaf(bv_node2_id);

  if(is_first_node_leaf && is_second_node_leaf)
  {
    updateFrontList(front_list, bv_node1_id, bv_node2_id);

    if(node->BVTesting(bv_node1_id, bv_node2_id, R, T)) return;

    node->leafTesting(bv_node1_id, bv_node2_id, R, T);
    return;
  }

  if(node->BVTesting(bv_node1_id, bv_node2_id, R, T))
  {
    updateFrontList(front_list, bv_node1_id, bv_node2_id);
    return;
  }

  Vec3f temp;

  if(node->firstOverSecond(bv_node1_id, bv_node2_id))
  {
    int c1 = node->getFirstLeftChild(bv_node1_id);
    int c2 = node->getFirstRightChild(bv_node1_id);

    const OBB& bv1 = node->model1->getBV(c1).bv;

    Matrix3f Rc(R.transposeTimes(bv1.axis[0]), R.transposeTimes(bv1.axis[1]), R.transposeTimes(bv1.axis[2]));
    temp = T - bv1.To;
    Vec3f Tc(temp.dot(bv1.axis[0]), temp.dot(bv1.axis[1]), temp.dot(bv1.axis[2]));

    collisionRecurse(node, c1, bv_node2_id, Rc, Tc, front_list);

    // early stop is disabled is front_list is used
    if(node->canStop() && !front_list) return;

    const OBB& bv2 = node->model1->getBV(c2).bv;

    Rc = Matrix3f(R.transposeTimes(bv2.axis[0]), R.transposeTimes(bv2.axis[1]), R.transposeTimes(bv2.axis[2]));
    temp = T - bv2.To;
    Tc.setValue(temp.dot(bv2.axis[0]), temp.dot(bv2.axis[1]), temp.dot(bv2.axis[2]));

    collisionRecurse(node, c2, bv_node2_id, Rc, Tc, front_list);
  }
  else
  {
    int c1 = node->getSecondLeftChild(bv_node2_id);
    int c2 = node->getSecondRightChild(bv_node2_id);

    const OBB& bv1 = node->model2->getBV(c1).bv;
    Matrix3f Rc;
    temp = R * bv1.axis[0];
    Rc(0, 0) = temp[0]; Rc(1, 0) = temp[1]; Rc(2, 0) = temp[2];
    temp = R * bv1.axis[1];
    Rc(0, 1) = temp[0]; Rc(1, 1) = temp[1]; Rc(2, 1) = temp[2];
    temp = R * bv1.axis[2];
    Rc(0, 2) = temp[0]; Rc(1, 2) = temp[1]; Rc(2, 2) = temp[2];
    Vec3f Tc = R * bv1.To + T;

    collisionRecurse(node, bv_node1_id, c1, Rc, Tc, front_list);

    // early stop is disabled is front_list is used
    if(node->canStop() && !front_list) return;

    const OBB& bv2 = node->model2->getBV(c2).bv;
    temp = R * bv2.axis[0];
    Rc(0, 0) = temp[0]; Rc(1, 0) = temp[1]; Rc(2, 0) = temp[2];
    temp = R * bv2.axis[1];
    Rc(0, 1) = temp[0]; Rc(1, 1) = temp[1]; Rc(2, 1) = temp[2];
    temp = R * bv2.axis[2];
    Rc(0, 2) = temp[0]; Rc(1, 2) = temp[1]; Rc(2, 2) = temp[2];
    Tc = R * bv2.To + T;

    collisionRecurse(node, bv_node1_id, c2, Rc, Tc, front_list);
  }
}

void collisionRecurse(MeshCollisionTraversalNodeRSS* node, int bv_node1_id, int bv_node2_id, 
  const Matrix3f& R, const Vec3f& T, BVHFrontList* front_list)
{

}

/** Recurse function for self collision
 * Make sure node is set correctly so that the first and second tree are the same
 */
void selfCollisionRecurse(CollisionTraversalNodeBase* node, int bv_node_id, BVHFrontList* front_list)
{
  bool is_first_node_leaf = node->isFirstNodeLeaf(bv_node_id);

  if(is_first_node_leaf) return;

  int left_child = node->getFirstLeftChild(bv_node_id);
  int right_child = node->getFirstRightChild(bv_node_id);

  selfCollisionRecurse(node, left_child, front_list);
  if(node->canStop() && !front_list) return;

  selfCollisionRecurse(node, right_child, front_list);
  if(node->canStop() && !front_list) return;

  collisionRecurse(node, left_child, right_child, front_list);
}

void distanceRecurse(DistanceTraversalNodeBase* node, int bv_node1_id, int bv_node2_id, BVHFrontList* front_list)
{
  bool is_first_node_leaf = node->isFirstNodeLeaf(bv_node1_id);
  bool is_second_node_leaf = node->isSecondNodeLeaf(bv_node2_id);

  if(is_first_node_leaf && is_second_node_leaf)
  {
    updateFrontList(front_list, bv_node1_id, bv_node2_id);

    node->leafTesting(bv_node1_id, bv_node2_id);
    return;
  }

  // BVNodes distance pairs
  int a1, a2, c1, c2;

  if(node->firstOverSecond(bv_node1_id, bv_node2_id))
  {
    a1 = node->getFirstLeftChild(bv_node1_id);
    a2 = bv_node2_id;
    c1 = node->getFirstRightChild(bv_node1_id);
    c2 = bv_node2_id;
  }
  else
  {
    a1 = bv_node1_id;
    a2 = node->getSecondLeftChild(bv_node2_id);
    c1 = bv_node1_id;
    c2 = node->getSecondRightChild(bv_node2_id);
  }

  FCL_REAL distance_a = node->BVTesting(a1, a2);
  FCL_REAL distance_c = node->BVTesting(c1, c2);

  if(distance_c < distance_a)
  {
    if(!node->canStop(distance_c))
      distanceRecurse(node, c1, c2, front_list);
    else
      updateFrontList(front_list, c1, c2);

    if(!node->canStop(distance_a))
      distanceRecurse(node, a1, a2, front_list);
    else
      updateFrontList(front_list, a1, a2);
  }
  else
  {
    if(!node->canStop(distance_a))
      distanceRecurse(node, a1, a2, front_list);
    else
      updateFrontList(front_list, a1, a2);

    if(!node->canStop(distance_c))
      distanceRecurse(node, c1, c2, front_list);
    else
      updateFrontList(front_list, c1, c2);
  }
}


/** \brief Bounding volume test structure */
struct BVT
{
	BVT() :
		depth(0)
	{
	}

	/** \brief distance between bvs */
	FCL_REAL d;

	/** \brief bv indices for a pair of bvs in two models */
	int bv_node1_id;
	int bv_node2_id;

	int depth;
};

/** \brief Comparer between two BVT */
struct BVT_Comparer
{
  bool operator() (const BVT& lhs, const BVT& rhs) const
  {
    return lhs.d > rhs.d;
  }
};

struct BVTQ
{
  BVTQ() : qsize(2) {}

  bool empty() const
  {
    return pq.empty();
  }

  size_t size() const
  {
    return pq.size();
  }

  const BVT& top() const
  {
    return pq.top();
  }

  void push(const BVT& x)
  {
    pq.push(x);
  }

  void pop()
  {
    pq.pop();
  }

  bool full() const
  {
    return (pq.size() + 1 >= qsize);
  }

  std::priority_queue<BVT, std::vector<BVT>, BVT_Comparer> pq;

  /** \brief Queue size */
  int qsize;
};


void distanceQueueRecurse(DistanceTraversalNodeBase* node, int bv_node1_id, int bv_node2_id, BVHFrontList* front_list, int qsize)
{
  BVTQ bvtq;
  bvtq.qsize = qsize;

  BVT min_test;
  min_test.bv_node1_id = bv_node1_id;
  min_test.bv_node2_id = bv_node2_id;

  FCL_REAL whole_distance_fraction = node->whole_distance_ / 10.0;

  while(1)
  {
    bool l1 = node->isFirstNodeLeaf(min_test.bv_node1_id);
    bool l2 = node->isSecondNodeLeaf(min_test.bv_node2_id);

    if(l1 && l2)
    {
      updateFrontList(front_list, min_test.bv_node1_id, min_test.bv_node2_id);

      node->leafTesting(min_test.bv_node1_id, min_test.bv_node2_id);
    }
    else if(bvtq.full())
    {
      // queue should not get two more tests, recur

      distanceQueueRecurse(node, min_test.bv_node1_id, min_test.bv_node2_id, front_list, qsize);
    }
    else
    {
      // queue capacity is not full yet
      BVT bvt1, bvt2;

      if(node->firstOverSecond(min_test.bv_node1_id, min_test.bv_node2_id))
      {
        int c1 = node->getFirstLeftChild(min_test.bv_node1_id);
        int c2 = node->getFirstRightChild(min_test.bv_node1_id);

        bvt1.bv_node1_id = c1;
        bvt1.bv_node2_id = min_test.bv_node2_id;
        bvt1.d = node->BVTesting(bvt1.bv_node1_id, bvt1.bv_node2_id);

        bvt2.bv_node1_id = c2;
        bvt2.bv_node2_id = min_test.bv_node2_id;
        bvt2.d = node->BVTesting(bvt2.bv_node1_id, bvt2.bv_node2_id);
      }
      else
      {
        int c1 = node->getSecondLeftChild(min_test.bv_node2_id);
        int c2 = node->getSecondRightChild(min_test.bv_node2_id);

        bvt1.bv_node1_id = min_test.bv_node1_id;
        bvt1.bv_node2_id = c1;
        bvt1.d = node->BVTesting(bvt1.bv_node1_id, bvt1.bv_node2_id);

        bvt2.bv_node1_id = min_test.bv_node1_id;
        bvt2.bv_node2_id = c2;
        bvt2.d = node->BVTesting(bvt2.bv_node1_id, bvt2.bv_node2_id);
      }	  

	  bvt1.depth = min_test.depth + 1;
	  bvt2.depth = min_test.depth + 1;

      bvtq.push(bvt1);
      bvtq.push(bvt2);
    }

    if(bvtq.empty())
      break;
    else
    {
      min_test = bvtq.top();
      bvtq.pop();

      if (node->canStop(min_test.d) )
      {
        updateFrontList(front_list, min_test.bv_node1_id, min_test.bv_node2_id);
        break;
      }	  

      if (
		  min_test.d >= node->can_stop_distance_
		  || (min_test.depth <= 2 && min_test.d >= whole_distance_fraction)
		 )
      {
        node->result->update(min_test.d, node->o1, node->o2, min_test.bv_node1_id, min_test.bv_node2_id);

        updateFrontList(front_list, min_test.bv_node1_id, min_test.bv_node2_id);
        break;
      }
    }
  }
}

void propagateBVHFrontListCollisionRecurse(CollisionTraversalNodeBase* node, BVHFrontList* front_list)
{
  BVHFrontList::iterator front_iter;
  BVHFrontList append;
  for(front_iter = front_list->begin(); front_iter != front_list->end(); ++front_iter)
  {
    int bv_node1_id = front_iter->left;
    int bv_node2_id = front_iter->right;
    bool l1 = node->isFirstNodeLeaf(bv_node1_id);
    bool l2 = node->isSecondNodeLeaf(bv_node2_id);

    if(l1 & l2)
    {
      front_iter->valid = false; // the front node is no longer valid, in collideRecurse will add again.
      collisionRecurse(node, bv_node1_id, bv_node2_id, &append);
    }
    else
    {
      if(!node->BVTesting(bv_node1_id, bv_node2_id))
      {
        front_iter->valid = false;

        if(node->firstOverSecond(bv_node1_id, bv_node2_id))
        {
          int c1 = node->getFirstLeftChild(bv_node1_id);
          int c2 = node->getFirstRightChild(bv_node2_id);

          collisionRecurse(node, c1, bv_node2_id, front_list);
          collisionRecurse(node, c2, bv_node2_id, front_list);
        }
        else
        {
          int c1 = node->getSecondLeftChild(bv_node2_id);
          int c2 = node->getSecondRightChild(bv_node2_id);

          collisionRecurse(node, bv_node1_id, c1, front_list);
          collisionRecurse(node, bv_node1_id, c2, front_list);
        }
      }
    }
  }


  // clean the old front list (remove invalid node)
  for(front_iter = front_list->begin(); front_iter != front_list->end();)
  {
    if(!front_iter->valid)
      front_iter = front_list->erase(front_iter);
    else
      ++front_iter;
  }

  for(front_iter = append.begin(); front_iter != append.end(); ++front_iter)
  {
    front_list->push_back(*front_iter);
  }
}


}
