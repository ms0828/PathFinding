#include "PathFinding.h"
#include <cmath>
#include <cfloat>
#include <random>

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

	path.clear();

	bFirstStep = true;
	destNode = nullptr;

	// searchNode 렌더링 관련 초기화
	memset(renderer->searchGrid, RGB(0, 0, 0), sizeof(renderer->searchGrid));
	
	// - colorBrushMap에 할당된 GDI 오브젝트(브러시) delete
	for (const auto& entry : colorBrushMap)
	{
		if (entry.second == NULL || entry.second == INVALID_HANDLE_VALUE)
			continue;
		DeleteObject(entry.second);
	}
	colorBrushMap.clear();
	colorMap.clear();
	huePool.clear();
}


bool CPathFinding::SetEndPos(pair<int, int> _endPos)
{
	int ey = _endPos.first;
	int ex = _endPos.second;
	if (ex < 0 || ex >= GRID_WIDTH || ey < 0 || ey >= GRID_HEIGHT)
		return false;

	endPos = { ey,ex };
	bHasGoal = true;
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

void CPathFinding::CreatePath()
{
	if (destNode == nullptr)
		return;
	path.clear();
	Node* current = destNode;
	while (current != nullptr)
	{
		path.push_back({ current->y, current->x });
		current = current->parent;
	}
	std::reverse(path.begin(), path.end());
}

void CPathFinding::FindPath_AStar()
{
	while (1)
	{
		if (FindPath_AStar_OneStep())
			break;
	}
	return;
}

bool CPathFinding::FindPath_AStar_OneStep()
{
	if (bHasGoal == false || destNode != nullptr)
		return true;

	// 첫 싸이클 예외 처리
	if (bFirstStep)
	{
		bFirstStep = false;
		int h;
		if (bDistanceH_Manhattan)
			h = GetManhattanDis(startPos, endPos);
		else
			h = GetEuclideanDis(startPos, endPos);
		Node* startNode = new Node(nullptr, EDir::DEFAULT, startPos, 0, h);
		openList[startPos] = startNode;
	}
	
	// AStar
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
			CreatePath();
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
			// - 비권장 (연산 오버헤드)
			// - 사용자가 장애물을 직각형으로 만드는게 더 낫다.
			// - 사용할꺼면 좌표에 대한 버퍼 오버런 예외 추가해야함 
			//if (i == 4 && grid[vNode->y - 1][vNode->x] && grid[vNode->y][vNode->x + 1]) //↗
			//	continue;
			//else if (i == 5 && grid[vNode->y][vNode->x + 1] && grid[vNode->y + 1][vNode->x]) //↘
			//	continue;
			//else if (i == 6 && grid[vNode->y][vNode->x - 1] && grid[vNode->y + 1][vNode->x]) // ↙
			//	continue;
			//else if (i == 7 && grid[vNode->y][vNode->x - 1] && grid[vNode->y - 1][vNode->x]) //↖
			//	continue;


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
			
			
			// 1. 해당 위치의 노드가 OpenList에 존재할 때
			//		- 더 좋은 경로로 해당 위치를 발견했다면 값을 갱신한다.
			// 2. 해당 위치의 노드가 OpenList에 없을 때
			//		- 새 노드를 생성 후, 새로 등록한다.
			auto prev = openList.find({ ny,nx });
			if (prev != openList.end())
			{
				Node* prevNode = (*prev).second;
				if (g + h >= prevNode->f)
					continue;
				prevNode->g = g;
				prevNode->f = g + h;
				prevNode->parent = vNode;
			}
			else
			{
				// 해당 위치를 처음 발견했다면 노드 생성 후 OpenList에 추가
				Node* nextVNode = new Node(vNode, EDir::DEFAULT, { ny, nx }, g, h);
				openList[{ny, nx}] = nextVNode;
			}
		}
		renderer->Rendering(this);
	}

	if (openList.empty())
		return true;

	return false;
}

bool CPathFinding::FindPath_JPS()
{
	int ret = 0;
	while (1)
	{
		ret = FindPath_JPS_OneStep();
		if (ret == 1)
			continue;
		else
			break;
	}
	if (ret == 0)
		return false;
	else
		return true;
}

int CPathFinding::FindPath_JPS_OneStep()
{
	if (bHasGoal == false || destNode != nullptr)
		return 0;

	// 첫 싸이클 예외 처리
	if (bFirstStep)
	{
		bFirstStep = false;
		int h;
		if (bDistanceH_Manhattan)
			h = GetManhattanDis(startPos, endPos);
		else
			h = GetEuclideanDis(startPos, endPos);
		Node* startNode = new Node(nullptr, DEFAULT, startPos, 0, h);
		openList[startPos] = startNode;

		//-------- searchGrid 랜더링 용 ---------
		// - colorMap에 해당 노드 포인터에 대한 color 등록
		// - 해당 color에 대한 브러시 gdi 오브젝트 등록
		COLORREF color = GenerateUniqueColor();
		colorMap[startNode] = color;
		colorBrushMap.insert({ color, CreateSolidBrush(color)});
	}

	
	// OpenList에서 가장 비용이 작은 노드를 추출
	if (!openList.empty())
	{
		Node* vNode = FindMinNodeInOpenList();
		openList.erase({ vNode->y, vNode->x });

		// 목적지 도달했다면 경로 구성 -> return
		if (vNode->y == endPos.first && vNode->x == endPos.second)
		{
			destNode = vNode;
			closedList[{vNode->y, vNode->x}] = vNode;
			CreatePath();
			renderer->Rendering(this);
			return 2;
		}

		// 방문 로직
		closedList[{vNode->y, vNode->x}] = vNode;

		// 현재 노드의 방향으로부터 특정 방향으로 새로 생성할 노드를 탐색
		EDir vNodeDir = vNode->dir;
		switch (vNodeDir)
		{
		case EDir::DEFAULT:
			SearchJumpPoint(vNode, EDir::R);
			SearchJumpPoint(vNode, EDir::U);
			SearchJumpPoint(vNode, EDir::L);
			SearchJumpPoint(vNode, EDir::D);
			SearchJumpPoint(vNode, EDir::RU);
			SearchJumpPoint(vNode, EDir::RD);
			SearchJumpPoint(vNode, EDir::LU);
			SearchJumpPoint(vNode, EDir::LD);
			break;
		case EDir::R:
			// 기본 방향
			SearchJumpPoint(vNode, EDir::R);
			// 위쪽 코너
			if (CheckObstacle(vNode->y - 1, vNode->x)) 
				SearchJumpPoint(vNode, EDir::RU);
			// 아래쪽 코너
			if (CheckObstacle(vNode->y + 1, vNode->x)) 
				SearchJumpPoint(vNode, EDir::RD);
			break;
		case EDir::U:
			// 기본 방향
			SearchJumpPoint(vNode, EDir::U);
			// 오른쪽 코너
			if (CheckObstacle(vNode->y, vNode->x + 1))
				SearchJumpPoint(vNode, EDir::RU);
			// 왼쪽 코너
			if (CheckObstacle(vNode->y, vNode->x - 1))
				SearchJumpPoint(vNode, EDir::LU);
			break;
		case EDir::L:
			SearchJumpPoint(vNode, EDir::L);
			// 위쪽 코너
			if (CheckObstacle(vNode->y - 1, vNode->x))
				SearchJumpPoint(vNode, EDir::LU);
			// 아래쪽 코너
			if (CheckObstacle(vNode->y + 1, vNode->x))
				SearchJumpPoint(vNode, EDir::LD);
			break;
		case EDir::D:
			// 기본 방향
			SearchJumpPoint(vNode, EDir::D);
			// 오른쪽 코너
			if (CheckObstacle(vNode->y, vNode->x + 1))
				SearchJumpPoint(vNode, EDir::RD);
			// 왼쪽 코너
			if (CheckObstacle(vNode->y, vNode->x - 1))
				SearchJumpPoint(vNode, EDir::LD);
			break;
		case EDir::RU:
			// 기본 방향
			SearchJumpPoint(vNode, EDir::RU);
			SearchJumpPoint(vNode, EDir::R);
			SearchJumpPoint(vNode, EDir::U);
			// 왼쪽 위 코너
			if(CheckObstacle(vNode->y, vNode->x - 1))
				SearchJumpPoint(vNode, EDir::LU);
			// 오른쪽 아래 코너
			if (CheckObstacle(vNode->y + 1, vNode->x))
				SearchJumpPoint(vNode, EDir::RD);
			break;
		case EDir::RD:
			// 기본 방향
			SearchJumpPoint(vNode, EDir::RD);
			SearchJumpPoint(vNode, EDir::R);
			SearchJumpPoint(vNode, EDir::D);
			// 오른쪽 위 코너
			if (CheckObstacle(vNode->y - 1, vNode->x))
				SearchJumpPoint(vNode, EDir::RU);
			// 왼쪽 아래 코너
			if (CheckObstacle(vNode->y, vNode->x - 1))
				SearchJumpPoint(vNode, EDir::LD);
			break;
		case EDir::LD:
			// 기본 방향
			SearchJumpPoint(vNode, EDir::LD);
			SearchJumpPoint(vNode, EDir::L);
			SearchJumpPoint(vNode, EDir::D);
			// 왼쪽 위 코너
			if (CheckObstacle(vNode->y - 1, vNode->x))
				SearchJumpPoint(vNode, EDir::LU);
			// 오른쪽 아래 코너
			if (CheckObstacle(vNode->y, vNode->x + 1))
				SearchJumpPoint(vNode, EDir::RD);
			break;
		case EDir::LU:
			// 기본 방향
			SearchJumpPoint(vNode, EDir::LU);
			SearchJumpPoint(vNode, EDir::L);
			SearchJumpPoint(vNode, EDir::U);
			// 오른쪽 위 코너
			if (CheckObstacle(vNode->y, vNode->x + 1))
				SearchJumpPoint(vNode, EDir::RU);
			// 왼쪽 아래 코너
			if (CheckObstacle(vNode->y + 1, vNode->x))
				SearchJumpPoint(vNode, EDir::LD);
			break;
		}
		
		renderer->Rendering(this);
	}

	if (openList.empty())
		return 0;

	return 1;
}

void CPathFinding::SearchJumpPoint(Node* vNode, EDir searchDir)
{
	int y, x;
	switch (searchDir)
	{
	case EDir::R:	y = vNode->y;	x = vNode->x + 1;	break;
	case EDir::U:	y = vNode->y - 1;	x = vNode->x;	break;
	case EDir::L:	y = vNode->y;	x = vNode->x - 1;	break;
	case EDir::D:	y = vNode->y + 1;	x = vNode->x;	break;
	case EDir::RU:	y = vNode->y - 1;	x = vNode->x + 1;	break;
	case EDir::RD:	y = vNode->y + 1;	x = vNode->x + 1;	break;
	case EDir::LD:	y = vNode->y + 1;	x = vNode->x - 1;	break;
	case EDir::LU:	y = vNode->y - 1;	x = vNode->x - 1;	break;
	}
	
	
	while (1)
	{
		if (x < 0 || x >= GRID_WIDTH || y < 0 || y >= GRID_HEIGHT)
			break;
		if (grid[y][x] == 1)
			break;

		// 코너 발견 또는 목적지 발견 확인 로직
		bool isJumpPoint = false;
		switch (searchDir)
		{
		case EDir::R:
			isJumpPoint |= (CheckObstacle(y + 1, x) && !CheckObstacle(y + 1, x + 1));
			isJumpPoint |= (CheckObstacle(y - 1, x) && !CheckObstacle(y - 1, x + 1));
			break;
		case EDir::L:
			isJumpPoint |= (CheckObstacle(y + 1, x) && !CheckObstacle(y + 1, x - 1));
			isJumpPoint |= (CheckObstacle(y - 1, x) && !CheckObstacle(y - 1, x - 1));
			break;
		case EDir::U:
			isJumpPoint |= (CheckObstacle(y, x + 1) && !CheckObstacle(y - 1, x + 1));
			isJumpPoint |= (CheckObstacle(y, x - 1) && !CheckObstacle(y - 1, x - 1));
			break;
		case EDir::D:
			isJumpPoint |= (CheckObstacle(y, x + 1) && !CheckObstacle(y + 1, x + 1));
			isJumpPoint |= (CheckObstacle(y, x - 1) && !CheckObstacle(y + 1, x - 1));
			break;
		case EDir::RU:
			isJumpPoint |= (CheckObstacle(y, x - 1) && !CheckObstacle(y - 1, x - 1));
			isJumpPoint |= (CheckObstacle(y + 1, x) && !CheckObstacle(y + 1, x + 1));
			isJumpPoint |= HasCorner(vNode, EDir::R, y, x);
			isJumpPoint |= HasCorner(vNode, EDir::U, y, x);
			break;
		case EDir::RD:
			isJumpPoint |= (CheckObstacle(y, x - 1) && !CheckObstacle(y + 1, x - 1));
			isJumpPoint |= (CheckObstacle(y - 1, x) && !CheckObstacle(y - 1, x + 1));
			isJumpPoint |= HasCorner(vNode, EDir::R, y, x);
			isJumpPoint |= HasCorner(vNode, EDir::D, y, x);
			break;
		case EDir::LU:
			isJumpPoint |= (CheckObstacle(y, x + 1) && !CheckObstacle(y - 1, x + 1));
			isJumpPoint |= (CheckObstacle(y + 1, x) && !CheckObstacle(y + 1, x - 1));
			isJumpPoint |= HasCorner(vNode, EDir::L, y, x);
			isJumpPoint |= HasCorner(vNode, EDir::U, y, x);
			break;
		case EDir::LD:
			isJumpPoint |= (CheckObstacle(y, x + 1) && !CheckObstacle(y + 1, x + 1));
			isJumpPoint |= (CheckObstacle(y - 1, x) && !CheckObstacle(y - 1, x - 1));
			isJumpPoint |= HasCorner(vNode, EDir::L, y, x);
			isJumpPoint |= HasCorner(vNode, EDir::D, y, x);
			break;
		}
		

		// 점프 포인트를 찾았을 때 점프 포인트 갱신 or 생성 후 OpenList 추가
		double g;
		double h;
		if (isJumpPoint || (y == endPos.first && x == endPos.second))
		{
			if (bDistanceG_Manhattan)
				g = vNode->g + GetManhattanDis({ vNode->y, vNode->x }, { y, x });
			else
				g = vNode->g + GetEuclideanDis({ vNode->y, vNode->x }, { y, x });
			if (bDistanceH_Manhattan)
				h = GetManhattanDis({ y, x }, endPos);
			else
				h = GetEuclideanDis({ y, x }, endPos);


			// 이미 방문한 노드는 탐색 제외 (최단 경로임을 보장)
			if (closedList.find({ y,x }) != closedList.end())
				break;


			// 1. 해당 위치의 노드가 OpenList에 존재할 때
			//		- 더 좋은 경로로 해당 위치를 발견했다면 값을 갱신한다.
			// 2. 해당 위치의 노드가 OpenList에 없을 때
			//		- 새 노드를 생성 후, 새로 등록한다.
			auto prev = openList.find({ y,x });
			if (prev != openList.end())
			{
				Node* prevNode = (*prev).second;
				if (g + h >= prevNode->f)
					break;
				prevNode->g = g;
				prevNode->f = g + h;
				prevNode->parent = vNode;
				prevNode->dir = searchDir;
			}
			else
			{
				// 해당 위치를 처음 발견했다면 노드 생성 후 OpenList에 추가
				Node* nextVNode = new Node(vNode, searchDir, { y, x }, g, h);
				openList[{y, x}] = nextVNode;

				// ----------- searchGrid 렌더링용 --------------
				// - colorMap에 해당 노드 포인터에 대한 color 등록
				// - 해당 color에 대한 브러시 gdi 오브젝트 등록
				COLORREF color = GenerateUniqueColor();
				colorMap[nextVNode] = color;

				if (colorBrushMap.find(color) == colorBrushMap.end())
				{
					HBRUSH hBrush = CreateSolidBrush(color);
					colorBrushMap[color] = hBrush;
				}

			}
			break;
		}

		// ----------- searchGrid 렌더링용 --------------
		// - 해당 위치에 renderer의 searchGrid에 색상 등록
		renderer->searchGrid[y][x] = colorMap[vNode];


		// 다음 탐색 좌표 업데이트
		switch (searchDir)
		{
		case EDir::R:	x++;	break;
		case EDir::U:	y--;	break;
		case EDir::L:	x--;	break;
		case EDir::D:	y++;	break;
		case EDir::RU:	x++;	y--;	break;
		case EDir::RD:	x++;	y++;	break;
		case EDir::LD:	x--;	y++;	break;
		case EDir::LU:	x--;	y--;	break;
		}
	}
}


bool CPathFinding::HasCorner(Node* vNode, EDir searchDir, int sy, int sx)
{
	int x;
	int y;
	switch (searchDir)
	{
	case EDir::R:	x = sx + 1;		y = sy;			break;
	case EDir::U:	x = sx;			y = sy - 1;		break;
	case EDir::L:	x = sx - 1;		y = sy;			break;
	case EDir::D:	x = sx;			y = sy + 1;		break;
	}

	while (1)
	{
		if (x < 0 || x >= GRID_WIDTH || y < 0 || y >= GRID_HEIGHT)
			break;
		if (grid[y][x] == 1)
			break;
		// 코너 발견 또는 목적지 발견
		bool bHasCorner = false;
		switch (searchDir)
		{
		case EDir::R:
			bHasCorner |= (CheckObstacle(y + 1, x) && !CheckObstacle(y + 1, x + 1));
			bHasCorner |= (CheckObstacle(y - 1, x) && !CheckObstacle(y - 1, x + 1));
			break;
		case EDir::U:
			bHasCorner |= (CheckObstacle(y, x + 1) && !CheckObstacle(y - 1, x + 1));
			bHasCorner |= (CheckObstacle(y, x - 1) && !CheckObstacle(y - 1, x - 1));
			break;
		case EDir::L:
			bHasCorner |= (CheckObstacle(y + 1, x) && !CheckObstacle(y + 1, x - 1));
			bHasCorner |= (CheckObstacle(y - 1, x) && !CheckObstacle(y - 1, x - 1));
			break;
		case EDir::D:
			bHasCorner |= (CheckObstacle(y, x + 1) && !CheckObstacle(y + 1, x + 1));
			bHasCorner |= (CheckObstacle(y, x - 1) && !CheckObstacle(y + 1, x - 1));
			break;
		}


		bool isGoal = (y == endPos.first && x == endPos.second);
		if (bHasCorner || isGoal)
		{
			// ----------- searchGrid 렌더링용 --------------
			// - 해당 위치에 renderer의 searchGrid에 색상 등록
			renderer->searchGrid[y][x] = colorMap[vNode];
			return true;
		}

		// ----------- searchGrid 렌더링용 --------------
		// - 해당 위치에 renderer의 searchGrid에 색상 등록
		renderer->searchGrid[y][x] = colorMap[vNode];
		

		// 다음 탐색 좌표 업데이트
		switch (searchDir)
		{
		case EDir::R:	x++;	break;
		case EDir::U:	y--;	break;
		case EDir::L:	x--;	break;
		case EDir::D:	y++;	break;
		}
	}

	return false;
}



bool CPathFinding::CheckObstacle(int y, int x)
{
	if (x < 0 || x >= GRID_WIDTH || y < 0 || y >= GRID_HEIGHT)
		return true;
	if (grid[y][x] == 1)
		return true;

	return false;
}

double CPathFinding::GetEuclideanDis(pair<int, int> _startPos, pair<int, int> _endPos)
{
	int dx = _startPos.second - _endPos.second;
	int dy = _startPos.first - _endPos.first;
	return std::sqrt(dx * dx + dy * dy);
}

int CPathFinding::GetManhattanDis(pair<int, int> _startPos, pair<int, int> _endPos)
{
	int dx = _startPos.second - _endPos.second;
	int dy = _startPos.first - _endPos.first;
	return abs(dx) + abs(dy);
}




//---------------------------------------
//-------- searchGrid 랜더링 용 ---------
//---------------------------------------
bool CPathFinding::IsWhiteColor(COLORREF c)
{
	int r = GetRValue(c);
	int g = GetGValue(c);
	int b = GetBValue(c);

	int avgRGB = (r + g + b) / 3;

	// 밝기가 너무 높다 → 거의 흰색  (max≥230)
	return (avgRGB >= 230);
}

COLORREF CPathFinding::GenerateUniqueColor()
{
	if (huePool.empty())
	{
		// 10도 간격으로 색상 추출 -> 36개의 색을 셔플 후 번갈아가며 사용할 것임
		for (int i = 0; i < 360; i += 10)
			huePool.push_back(i);
		std::shuffle(huePool.begin(), huePool.end(), std::mt19937{ std::random_device{}() });
	}

	COLORREF clr;

	do  // 회색 or 흰색이면 다시 뽑기
	{
		int hue = huePool[hueIndex % huePool.size()];
		++hueIndex;

		// 채도·명도 살짝 높여서 회색화 방지
		clr = HSVtoRGB(hue, 0.4, 0.9);

	} while (IsWhiteColor(clr));

	return clr;

}

COLORREF CPathFinding::HSVtoRGB(double h, double s, double v)
{
	double r, g, b;
	int i = static_cast<int>(h / 60.0) % 6;
	double f = (h / 60.0) - i;
	double p = v * (1.0 - s);
	double q = v * (1.0 - f * s);
	double t = v * (1.0 - (1.0 - f) * s);

	switch (i)
	{
	case 0:
		r = v; g = t; b = p;
		break;
	case 1:
		r = q; g = v; b = p;
		break;
	case 2:
		r = p; g = v; b = t;
		break;
	case 3:
		r = p; g = q; b = v;
		break;
	case 4:
		r = t; g = p; b = v;
		break;
	case 5:
		r = v; g = p; b = q;
		break;
	default:
		r = g = b = 0;
		break;
	}

	return RGB(static_cast<int>(r * 255), static_cast<int>(g * 255), static_cast<int>(b * 255));
}

