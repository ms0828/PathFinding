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

	// ù ����Ŭ ���� ó��
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
		// OpenList���� ���� ����� ���� ��带 ����
		Node* vNode = FindMinNodeInOpenList();
		openList.erase({ vNode->y, vNode->x });
		
		// ������ �����ߴٸ� ��� ���� -> return
		if (vNode->y == endPos.first && vNode->x == endPos.second)
		{
			destNode = vNode;
			closedList[{vNode->y, vNode->x}] = vNode;
			renderer->RenderPath(vNode);
			return true;
		}

		// �湮 ����
		closedList[{vNode->y, vNode->x}] = vNode;

		// ���� �湮 ��� OpenList�� ���
		for (int i = 0; i < 8; i++)
		{
			int nx = vNode->x + dx[i];
			int ny = vNode->y + dy[i];
			if (nx < 0 || nx >= GRID_WIDTH || ny < 0 || ny >= GRID_HEIGHT)
				continue;

			// ��ֹ� ó��
			if (grid[ny][nx] == 1)
				continue;
			// + �밢�� ������ �� ��ֹ� ó��
			if (i == 4 && grid[vNode->y - 1][vNode->x] && grid[vNode->y][vNode->x + 1]) //��
				continue;
			else if (i == 5 && grid[vNode->y][vNode->x + 1] && grid[vNode->y + 1][vNode->x]) //��
				continue;
			else if (i == 6 && grid[vNode->y][vNode->x - 1] && grid[vNode->y + 1][vNode->x]) // ��
				continue;
			else if (i == 7 && grid[vNode->y][vNode->x - 1] && grid[vNode->y - 1][vNode->x]) //��
				continue;


			// ���� �湮�� ��ġ�� F �� ���
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

			// �̹� �湮�� ���� Ž�� ����
			if (closedList.find({ ny,nx }) != closedList.end())
				continue;
			
			// Ȥ�� ���� ���� ����Ʈ�� ������, ���� Ž���� ��ΰ� ������ ��κ��� ����� ������ ����
			auto prev = openList.find({ ny,nx });
			if (prev != openList.end())
			{
				Node* prevNode = (*prev).second;
				if (g + h >= prevNode->f)
					continue;
			}

			// ���ο� ��ġ�̰ų� Ȥ�� ���� ��ġ���� �� ���� ����� ��ġ�� ��� ���� �� �湮 ����
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

