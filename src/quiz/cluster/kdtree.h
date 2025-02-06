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

	void insertHelper(Node*& node, uint depth, std::vector<float> point, int id) {
        if (node == NULL) {
            node = new Node(point, id);
            return;
        } else {
            uint splitAxis = depth % 3;
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


    float EuclideanDistance(std::vector<float> node, std::vector<float> pointToCheck)
	{
		float sum_squared = 0;
		for (size_t i = 0; i < node.size(); i++)
		{
			sum_squared += ((node[i] - pointToCheck[i]) * (node[i] - pointToCheck[i]));
		}
		double distance = sqrt(sum_squared);
		return distance;
	}


    std::vector<int> searchHelper(std::vector<float> pointToCheck, Node *&node, uint depth, float distanceTol, std::vector<int> &ids)
    {
        uint splitAxis = depth%2;

        if (node != NULL) 
        {
            if ((node->point[0] <= (pointToCheck[0] + distanceTol) && node->point[0] >= (pointToCheck[0] - distanceTol) &&
				node->point[1] <= (pointToCheck[1] + distanceTol) && node->point[1] >= (pointToCheck[1] - distanceTol) &&
				node->point[2] <= (pointToCheck[2] + distanceTol) && node->point[2] >= (pointToCheck[2] - distanceTol))) 
            {
				
                float distance = EuclideanDistance(node->point, pointToCheck);

				// Add node's ID only if node's distance is within distance tolerance
				if (distance <= distanceTol)
				{
					ids.push_back(node->id);
				}
            }
            // Intersection of the bounding box with the split line
			if (node->point[splitAxis] <= (pointToCheck[splitAxis] + distanceTol) && node->point[splitAxis] >= (pointToCheck[splitAxis] - distanceTol))
			{
				searchHelper(pointToCheck, node->left, distanceTol, depth + 1, ids);
				searchHelper(pointToCheck, node->right, distanceTol, depth + 1, ids);
			}
			// Recursive 2a - if node's cutting axis is NOT intersecting bounding box - min bound
			else if (node->point[splitAxis] < (pointToCheck[splitAxis] - distanceTol))
			{
				searchHelper(pointToCheck, node->right, distanceTol, depth + 1, ids);
			}
			// Recursive 2b - if node's cutting axis is NOT intersecting bounding box - max bound
			else
			{
				searchHelper(pointToCheck, node->left, distanceTol, depth + 1, ids);
			}
	    }

        return ids;
    }


	// // check if the node point is within a boxed square which is 2 times the tolerance
    // bool isInBox(std::vector<float> pointToCheck, std::vector<float> treePoint, float distanceTol)
    // {
    //     if ((treePoint[0] < (pointToCheck[0] - distanceTol)) || (treePoint[0] > (pointToCheck[0] + distanceTol)) || (treePoint[1] < (pointToCheck[1] - distanceTol)) || (treePoint[1] > (pointToCheck[1] + distanceTol))) {
    //         return false;
    //     }
    //     return true;
    // }

    // // calculate if the distance between node point and point to check point is within tolerance
    // bool isInDistanceTol(std::vector<float> pointToCheck, std::vector<float> treePoint, float distanceTol)
    // {
    //     float d = sqrtf(pow((pointToCheck[0] - treePoint[0]), 2.0) + pow((pointToCheck[1] - treePoint[1]), 2.0));

    //     if (d < distanceTol) {
    //         return true;
    //     }
    //     return false;
    // }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> pointToCheck, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(pointToCheck, root, 0, distanceTol, ids);
		return ids;
	}
	

};
