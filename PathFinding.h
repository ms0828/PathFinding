#pragma once
#include "Define_Grid.h"
#include <vector>
#include <unordered_map>
#include "PathFinding_Renderer.h"

using namespace std;

// 타일 렌더링을 위한 임시 변수
extern int g_gridSize;
extern int g_scrollOffsetX;
extern int g_scrollOffsetY;

enum EDir
{
	DEFAULT = 0,
	R,
	U,
	L,
	D,
	RU,
	RD,
	LD,
	LU
};

class Node
{
public:
	Node(Node* _parent, EDir _dir, pair<int,int> pos, double _g, double _h)
	{
		parent = _parent;
		dir = _dir;
		x = pos.second;
		y = pos.first;
		g = _g;
		h = _h;
		f = g + h;
	}

	
public:
	Node* parent;
	EDir dir;
	int x;
	int y;
	double g;
	double h;
	double f;
};


// unordered_map <pair<int,int>, Node*> 해시 함수 정의
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

		// renderer의 search Grid 초기화
		memset(renderer->searchGrid, RGB(0, 0, 0), sizeof(renderer->searchGrid));
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

		// searchGrid 색상 관련 (렌더링 용)
		// - colorBrushMap에 할당된 GDI 오브젝트(브러시) delete
		for (const auto& entry : colorBrushMap)
		{
			if (entry.second == NULL || entry.second == INVALID_HANDLE_VALUE)
				continue;
			DeleteObject(entry.second);			
		}
		colorBrushMap.clear();
	}

	void Init();

	bool SetEndPos(pair<int, int> _endPos);

	Node* FindMinNodeInOpenList();

	void CreatePath();

	double GetEuclideanDis(pair<int, int> _startPos, pair<int, int> _endPos);

	int GetManhattanDis(pair<int, int> _startPos, pair<int, int> _endPos);

	//-----------AStar-------------
	void FindPath_AStar();
	bool FindPath_AStar_OneStep();

	//-----------JPS-------------
	void FindPath_JPS();
	bool FindPath_JPS_OneStep();
	
	void SearchJumpPoint(Node* vNode, EDir searchDir);

	bool HasCorner_R(Node* vNode, int sy, int sx);
	bool HasCorner_U(Node* vNode, int sy, int sx);
	bool HasCorner_L(Node* vNode, int sy, int sx);
	bool HasCorner_D(Node* vNode, int sy, int sx);

	bool CheckObstacle(int y, int x);

	// searchGrid 색상 관련 (렌더링 용)
	bool IsWhiteColor(COLORREF c);
	COLORREF GenerateUniqueColor();
	COLORREF HSVtoRGB(double h, double s, double v);



	

public:

	CPathFinding_Renderer* renderer;

	pair<int, int> startPos;
	pair<int, int> endPos;

	char (*grid)[GRID_WIDTH];
	
	std::unordered_map<pair<int, int>, Node*, PositionHash> openList;
	std::unordered_map<pair<int, int>, Node*, PositionHash> closedList;
	std::vector<pair<int, int>> path;

	bool bHasGoal;
	bool bFirstStep;
	Node* destNode;

	bool bDistanceG_Manhattan;
	bool bDistanceH_Manhattan;


	// searchGrid 색상 관련 (렌더링 용)
	std::unordered_map<Node*, COLORREF> colorMap;
	std::unordered_map<COLORREF, HBRUSH> colorBrushMap;
	std::vector<int> huePool;
	int hueIndex = 0;

};
