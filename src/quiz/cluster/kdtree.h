/* \author Aaron Brown */
// Quiz on implementing kd tree
#include <iostream>
#include <vector>
#include <algorithm>
#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insertHelper(Node*& node, uint depth, const std::vector<float>& point, int id) {
        if (node == NULL) {
            node = new Node(point, id);
        } else {
            uint splitAxis = depth % 2;
            if (point[splitAxis] < node->point[splitAxis]) {
                insertHelper(node->left, depth + 1, point, id);
            } else {
                insertHelper(node->right, depth + 1, point, id);
            }
        }
    }

	void insert(std::vector<float> point, int id)
	{
		insertHelper(root, 0, point, id);
	}

	// check if the node point is within a boxed square which is 2 times the tolerance
    bool isInBox(std::vector<float> pointToCheck, std::vector<float> treePoint, float distanceTol)
    {
        if ((treePoint[0] < (pointToCheck[0] - distanceTol)) || (treePoint[0] > (pointToCheck[0] + distanceTol)) || (treePoint[1] < (pointToCheck[1] - distanceTol)) || (treePoint[1] > (pointToCheck[1] + distanceTol))) {
            return false;
        }
        return true;
    }

    // calculate if the distance between node point and point to check point is within tolerance
    bool isInDistanceTol(std::vector<float> pointToCheck, std::vector<float> treePoint, float distanceTol)
    {
        float d = sqrtf(pow((pointToCheck[0] - treePoint[0]), 2.0) + pow((pointToCheck[1] - treePoint[1]), 2.0));

        if (d < distanceTol) {
            return true;
        }
        return false;
    }

	void searchHelper(std::vector<float> pointToCheck, Node* node, uint depth, float distanceTol, std::vector<int>& ids)
    {
        if (node != NULL) {
            if (isInBox(pointToCheck, node->point, distanceTol)) {
                if (isInDistanceTol(pointToCheck, node->point, distanceTol)) {
                    // add point within distanceTol to the return list
                    ids.push_back(node->id);
                }
            }

            // branch to the next node depending on if the boxed square crosses over the divided x or y region
            if ((pointToCheck[depth % 2] - distanceTol) < node->point[depth % 2]) {
                searchHelper(pointToCheck, node->left, depth+1, distanceTol, ids);
            }
            if ((pointToCheck[depth % 2] + distanceTol) >= node->point[depth % 2]) {
                searchHelper(pointToCheck, node->right, depth+1, distanceTol, ids);
            }
        }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> pointToCheck, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(pointToCheck, root, 0, distanceTol, ids)
		return ids;
	}
	

};




