#include "PathFinding.h"
#include <cmath>
#include <cfloat>

int dy[8] = { 0, -1, 0, 1, -1, 1, 1, -1 };
int dx[8] = { 1, 0, -1, 0, 1, 1, -1, -1 };


void CPathFinding::Init()
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

	bFirstStep = true;
	destNode = nullptr;
}


bool CPathFinding::SetEndPos(pair<int, int> _endPos)
{
	int ey = _endPos.first;
	int ex = _endPos.second;
	if (ex < 0 || ex >= GRID_WIDTH || ey < 0 || ey >= GRID_HEIGHT)
		return false;

	endPos = { ey,ex };
	bHasGoal = true;

	Init();
	return true;
}

Node* CPathFinding::FindMinNodeInOpenList()
{
	Node* minNode = nullptr;
	double minF = DBL_MAX;

	for (const auto& pair : openList)
	{
		Node* node = pair.second;
		if (node->f < minF)
		{
			minF = node->f;
			minNode = node;
		}
	}

	return minNode;
}
void CPathFinding::FindPath()
{
	while (1)
	{
		if (FindPath_OneStep())
			break;
	}
	return;
}

bool CPathFinding::FindPath_OneStep()
{
	if (bHasGoal == false || destNode != nullptr)
		return true;

	HBRUSH hOldBrush;
	int gridX;
	int gridY;

	// 첫 싸이클 예외 처리
	if (bFirstStep)
	{
		bFirstStep = false;
		int h;
		if (bDistanceH_Manhattan)
			h = GetManhattanDis(startPos, endPos);
		else
			h = GetEuclideanDis(startPos, endPos);
		Node* startNode = new Node(nullptr, startPos, 0, h);
		openList[startPos] = startNode;
	}
	
	// JPS
	if (!openList.empty())
	{
		// OpenList에서 가장 비용이 작은 노드를 추출
		Node* vNode = FindMinNodeInOpenList();
		openList.erase({ vNode->y, vNode->x });
		
		// 목적지 도달했다면 경로 구성 -> return
		if (vNode->y == endPos.first && vNode->x == endPos.second)
		{
			destNode = vNode;
			closedList[{vNode->y, vNode->x}] = vNode;
			renderer->RenderPath(vNode);
			return true;
		}

		// 방문 로직
		closedList[{vNode->y, vNode->x}] = vNode;

		// 다음 방문 노드 OpenList에 등록
		for (int i = 0; i < 8; i++)
		{
			int nx = vNode->x + dx[i];
			int ny = vNode->y + dy[i];
			if (nx < 0 || nx >= GRID_WIDTH || ny < 0 || ny >= GRID_HEIGHT)
				continue;

			// 장애물 처리
			if (grid[ny][nx] == 1)
				continue;
			// + 대각선 방향일 때 장애물 처리
			if (i == 4 && grid[vNode->y - 1][vNode->x] && grid[vNode->y][vNode->x + 1]) //↗
				continue;
			else if (i == 5 && grid[vNode->y][vNode->x + 1] && grid[vNode->y + 1][vNode->x]) //↘
				continue;
			else if (i == 6 && grid[vNode->y][vNode->x - 1] && grid[vNode->y + 1][vNode->x]) // ↙
				continue;
			else if (i == 7 && grid[vNode->y][vNode->x - 1] && grid[vNode->y - 1][vNode->x]) //↖
				continue;


			// 다음 방문할 위치의 F 값 계산
			double g;
			double h;
			if (bDistanceG_Manhattan)
				g = vNode->g + GetManhattanDis({ vNode->y, vNode->x }, { ny, nx });
			else
				g = vNode->g + GetEuclideanDis({ vNode->y, vNode->x }, { ny, nx });
			if (bDistanceH_Manhattan)
				h = GetManhattanDis({ ny, nx }, endPos);
			else
				h = GetEuclideanDis({ ny, nx }, endPos);

			// 이미 방문한 노드는 탐색 제외
			if (closedList.find({ ny,nx }) != closedList.end())
				continue;
			
			// 혹은 아직 오픈 리스트에 있으나, 새로 탐색한 경로가 이전의 경로보다 비용이 높으면 제외
			auto prev = openList.find({ ny,nx });
			if (prev != openList.end())
			{
				Node* prevNode = (*prev).second;
				if (g + h >= prevNode->f)
					continue;
			}

			// 새로운 위치이거나 혹은 기존 위치보다 더 좋은 경로의 위치는 노드 생성 후 방문 예약
			Node* nextVNode = new Node(vNode, { ny, nx }, g, h);
			openList[{ny, nx}] = nextVNode;
		}
		renderer->Rendering(this);
	}

	if (openList.empty())
		return true;

	return false;
}

double CPathFinding::GetEuclideanDis(pair<int, int> _startPos, pair<int, int> _endPos)
{
	int dx = _startPos.second - _endPos.second;
	int dy = _startPos.first - _endPos.first;
	double test = std::sqrt(dx * dx + dy * dy);
	return std::sqrt(dx * dx + dy * dy);
}

int CPathFinding::GetManhattanDis(pair<int, int> _startPos, pair<int, int> _endPos)
{
	int dx = _startPos.second - _endPos.second;
	int dy = _startPos.first - _endPos.first;
	return abs(dx) + abs(dy);
}

