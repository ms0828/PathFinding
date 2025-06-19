#pragma once
#include "Define_Grid.h"
#include <Windows.h>
#include <vector>
#include <unordered_map>
#include "PathFinding_Renderer.h"

using namespace std;

class Node
{
public:
	Node(Node* _parent, pair<int,int> pos, double _g, double _h)
	{
		parent = _parent;
		x = pos.second;
		y = pos.first;
		g = _g;
		h = _h;
		f = g + h;
	}

public:
	Node* parent;
	int x;
	int y;
	double g;
	double h;
	double f;
};

// unordered_set<Node*> 해시 함수 정의
struct NodePtrHash
{
	std::size_t operator()(const Node* node) const
	{
		return std::hash<int>()(node->x) ^ (std::hash<int>()(node->y) << 1);
	}
};


// unordered_map, set <pair<int,int>> 해시 함수 정의
struct PositionHash
{
	std::size_t operator()(const pair<int, int> pos) const
	{
		return std::hash<int>()(pos.first) ^ (std::hash<int>()(pos.second) << 1);
	}
};

class CPathFinding
{

public:
	CPathFinding(CPathFinding_Renderer* _renderer, char(*_grid)[GRID_WIDTH], pair<int, int> _startPos)
	{
		renderer = _renderer;
		grid = _grid;
		startPos = _startPos;
		bHasGoal = false;
		bFirstStep = true;
		destNode = nullptr;
		bDistanceG_Manhattan = false;
		bDistanceH_Manhattan = true;
	}

	~CPathFinding()
	{
		std::vector<Node*> toDeleteOpenList;
		for (const auto& entry : openList)
			toDeleteOpenList.push_back(entry.second);
		openList.clear();
		for (Node* node : toDeleteOpenList)
			delete node;

		std::vector<Node*> toDeleteClosedList;
		for (const auto& entry : closedList)
			toDeleteClosedList.push_back(entry.second);
		closedList.clear();
		for (Node* node : toDeleteClosedList)
			delete node;

	}

	void Init();

	Node* FindMinNodeInOpenList();

	bool SetEndPos(pair<int, int> _endPos);
	
	void FindPath();
	
	bool FindPath_OneStep();

	double GetEuclideanDis(pair<int,int> _startPos, pair<int,int> _endPos);
	int GetManhattanDis(pair<int, int> _startPos, pair<int, int> _endPos);


public:

	CPathFinding_Renderer* renderer;

	pair<int, int> startPos;
	pair<int, int> endPos;

	char (*grid)[GRID_WIDTH];
	
	std::unordered_map<pair<int, int>, Node*, PositionHash> openList;
	std::unordered_map<pair<int, int>, Node*, PositionHash> closedList;


	bool bHasGoal;
	bool bFirstStep;
	Node* destNode;

	bool bDistanceG_Manhattan;
	bool bDistanceH_Manhattan;
};
