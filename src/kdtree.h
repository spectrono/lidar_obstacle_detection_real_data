/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id{0};
	Node* left{nullptr};
	Node* right{nullptr};

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(nullptr), right(nullptr)
	{}
};

struct KdTree
{
	Node* root{nullptr};

	KdTree()
	: root(nullptr)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertAtNode(root, 0, point, id);
	}

	void insertAtNode(Node*& node, const uint depth, std::vector<float> point, int id)
	{
		if (node == nullptr)
		{
			node = new Node(point, id);
		}
		else
		{
			// Get trees depth
			uint current_depth = depth % 2;

			if (point[current_depth] < (node->point[current_depth]))
			{
				insertAtNode(node->left, depth+1, point, id);
			}
			else
			{
				insertAtNode(node->right, depth+1, point, id);
			}
		}
	}

	void searchFromNode(Node*& node, std::vector<float> target, uint depth, float distanceTol, std::vector<int>& ids)
	{
		if (node != nullptr)
		{
			if (
				(node->point[0]>=(target[0]-distanceTol)) &&
			 	(node->point[0]<=(target[0]+distanceTol)) && 
			 	(node->point[1]>=(target[1]-distanceTol)) && 
			 	(node->point[1]<=(target[1]+distanceTol)))
			{
				const float diff_x = node->point[0] - target[0];				
				const float diff_y = node->point[1] - target[1];
				const float distance = sqrt((diff_x * diff_x) + (diff_y * diff_y));
				if (distance <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}
            
			if ((target[depth%2]-distanceTol) < node->point[depth%2])
			{
				searchFromNode(node->left, target, depth+1, distanceTol, ids);
			}

			if ((target[depth%2]+distanceTol) > node->point[depth%2])
			{
				searchFromNode(node->right, target, depth+1, distanceTol, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchFromNode(root, target, 0, distanceTol, ids);

		return ids;
	}
	

};




