/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"
#include "math.h"

// Structure to represent node of kd tree

template <typename PointT>
struct Node
{
	PointT point;
	int id;
	Node *left;
	Node *right;

	Node(PointT arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}
};

template <typename PointT>
struct KdTree
{
	Node<PointT> *root;

	KdTree()
		: root(NULL)
	{
	}

	void insert_helper(Node<PointT> **curr, int level, PointT data, int id)
	{

		if ((*curr) == NULL)
			*curr = new Node<PointT>(data, id);
		else{
			float tgt_p [] {data.x, data.y, data.z};
			float curr_p [] {(*curr)->point.x, (*curr)->point.y, (*curr)->point.z};

			if (tgt_p[level%3] < curr_p[level%3])
				insert_helper(&(*curr)->left, level + 1, data, id);
			else
				insert_helper(&(*curr)->right, level + 1, data, id);
		}
	}

	void insert(PointT point, int id)
	{	
		insert_helper(&root, 0, point, id);
	}

	// return a list of point ids in the tree that are within distance of target

	void searchHelper(Node<PointT> *curr, int level, PointT target, float distanceTol, std::vector<int> &ids)
	{
		if (curr == NULL)
			return;

		float x, y, z, distance;
		float xP, yP, zP, xN, yN, zN;

		float tgt_p []{target.x, target.y, target.z};
		float curr_p [] {curr->point.x, curr->point.y, curr->point.z};

		xP = target.x + (distanceTol);
		yP = target.y + (distanceTol);
		zP = target.z + (distanceTol);

		xN = target.x - (distanceTol);
		yN = target.y - (distanceTol);
		zN = target.z - (distanceTol);
		
		x = curr->point.x;
		y = curr->point.y;
		z = curr->point.z;

		if (x <= xP && x >= xN && y <= yP && y >= yN && z <= zP && z >= zN)
		{
			distance = sqrt(pow(x - target.x, 2) + pow(y - target.y, 2) + + pow(z - target.z, 2));
			if (distance <= distanceTol)
				ids.push_back(curr->id);
		}
		if ((tgt_p[level%3] - distanceTol) < curr_p[level%3])
			searchHelper(curr->left, level + 1, target, distanceTol, ids);
		if ((tgt_p[level%3] + distanceTol) > curr_p[level%3])
			searchHelper(curr->right, level + 1, target, distanceTol, ids);
	}

	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(root, 0, target, distanceTol, ids);

		return ids;
	}
};
