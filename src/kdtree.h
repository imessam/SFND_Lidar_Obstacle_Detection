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

		int idx = level % 2 == 0 ? 0 : 1;
		float tgt = level % 2 == 0 ? data.x : data.y;
		float curr_p ;

		if ((*curr) == NULL)
			*curr = new Node<PointT>(data, id);
		else{
			curr_p = level % 2 == 0 ? (*curr)->point.x : (*curr)->point.y;
			if (tgt < curr_p)
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
		float x, y, distance;
		float xP, yP, xN, yN;

		xP = target.x + (distanceTol);
		yP = target.y + (distanceTol);
		xN = target.x - (distanceTol);
		yN = target.y - (distanceTol);

		int idx = level % 2 == 0 ? 0 : 1;
		float p = level % 2 == 0 ? target.x : target.y;
		float curr_p = level % 2 == 0 ? curr->point.x : curr->point.y;
		x = curr->point.x;
		y = curr->point.y;

		if (x <= xP && x >= xN && y <= yP && y >= yN)
		{
			distance = sqrt(pow(x - target.x, 2) + pow(y - target.y, 2));
			if (distance <= distanceTol)
				ids.push_back(curr->id);
		}
		if ((p - distanceTol) < curr_p)
			searchHelper(curr->left, level + 1, target, distanceTol, ids);
		if ((p + distanceTol) > curr_p)
			searchHelper(curr->right, level + 1, target, distanceTol, ids);
	}

	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(root, 0, target, distanceTol, ids);

		return ids;
	}
};
