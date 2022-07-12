/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include "math.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}
};

struct KdTree
{
	Node *root;

	KdTree()
		: root(NULL)
	{
	}

	void insert_helper(Node **curr, int level, std::vector<float> data, int id)
	{

		int idx = level % 2 == 0 ? 0 : 1;

		if ((*curr) == NULL)
			*curr = new Node(data, id);
		else if (data[idx] < (*curr)->point[idx])
			insert_helper(&(*curr)->left, level + 1, data, id);
		else
			insert_helper(&(*curr)->right, level + 1, data, id);
	}
	void insert(std::vector<float> point, int id)
	{
		insert_helper(&root, 0, point, id);
	}

	// return a list of point ids in the tree that are within distance of target

	void searchHelper(Node *curr, int level, std::vector<float> target, float distanceTol, std::vector<int> &ids)
	{
		if (curr == NULL)
			return;
		float x, y, distance;
		float xP, yP, xN, yN;

		xP = target[0] + (distanceTol);
		yP = target[1] + (distanceTol);
		xN = target[0] - (distanceTol);
		yN = target[1] - (distanceTol);

		int idx = level % 2 == 0 ? 0 : 1;
		x = curr->point[0];
		y = curr->point[1];

		if (x <= xP && x >= xN && y <= yP && y >= yN)
		{
			distance = sqrt(pow(x - target[0], 2) + pow(y - target[1], 2));
			if (distance <= distanceTol)
				ids.push_back(curr->id);
		}
		if ((target[idx] - distanceTol) < curr->point[idx])
			searchHelper(curr->left, level + 1, target, distanceTol, ids);
		if ((target[idx] + distanceTol) > curr->point[idx])
			searchHelper(curr->right, level + 1, target, distanceTol, ids);
	}

	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(root, 0, target, distanceTol, ids);

		return ids;
	}
};
