/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

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

	void insert(std::vector<float> point, int id)
	{
		// DONE: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(&root, 0, point, id);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, distanceTol, root, 0, ids);
		return ids;
	}

	void searchHelper(std::vector<float> target, float distanceTol, Node *node, uint depth, std::vector<int> &ids)
	{
		if (node != NULL)
		{
			float node_x = node->point[0];
			float node_y = node->point[1];
			int node_id = node->id;
			float target_x = target[0];
			float target_y = target[1];

			//check if node is within tolerance box
			if (node_x >= (target_x - distanceTol) && node_x <= (target_x + distanceTol) && node_y >= (target_y - distanceTol) && node_y <= (target_y + distanceTol))
			{
				//check the actual distance
				float distance = sqrt(pow(node_x - target_x, 2) + pow(node_y - target_y, 2));

				if (distance <= distanceTol)
				{
					ids.push_back(node_id);
				}
			}

			if ((target[depth % 2] - distanceTol) < node->point[depth % 2])
			{
				searchHelper(target, distanceTol, node->left, depth + 1, ids);
			}

			if ((target[depth % 2] + distanceTol) > node->point[depth % 2])
			{
				searchHelper(target, distanceTol, node->right, depth + 1, ids);
			}
		}
	}

	void insertHelper(Node **node, uint depth, std::vector<float> point, int id)
	{
		//if tree is empty, ie node is rooot
		if (*node == NULL)
		{
			*node = new Node(point, id);
		}

		else
		{
			//choose x or y to split based on depth
			int cd = depth % 2;

			//compare and insert
			if (point[cd] < ((*node)->point[cd]))
			{
				insertHelper(&((*node)->left), depth + 1, point, id);
			}

			else
			{
				insertHelper(&((*node)->right), depth + 1, point, id);
			}
		}
	}
};
