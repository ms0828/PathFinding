#include "PathFinding_Renderer.h"
#include "PathFinding.h"
#include <unordered_map>

void CPathFinding_Renderer::ReSize(HDC hdc)
{
	SelectObject(hMemDC, hMemDCBitMap_old);
	DeleteObject(hMemDC);
	DeleteObject(hMemDCBitMap);

	GetClientRect(hWnd, &memDCRect);
	hMemDCBitMap = CreateCompatibleBitmap(hdc, memDCRect.right, memDCRect.bottom);
	hMemDC = CreateCompatibleDC(hdc);

	hMemDCBitMap_old = (HBITMAP)SelectObject(hMemDC, hMemDCBitMap);
}


void CPathFinding_Renderer::Rendering(CPathFinding* pathFinding)
{
	// 메모리 DC 클리어 및 메모리 DC에 그림 그리기
	PatBlt(hMemDC, 0, 0, memDCRect.right, memDCRect.bottom, WHITENESS);
	RenderGrid();
	RenderObstacle();
	if (pathFinding != nullptr)
	{
		RenderOpenList(pathFinding);
		RenderClosedList(pathFinding);
		RenderStartPos(pathFinding);
		if (pathFinding->bHasGoal)
			RenderEndPos(pathFinding);

		if (pathFinding->destNode != nullptr)
			RenderPath(pathFinding->destNode);
		RenderParentNode(pathFinding);
	}
	RenderInfoPanel(pathFinding);
	
	// 메모리 DC에 렌더링이 끝나면 메모리 DC에서 윈도우 DC로 출력
	HDC hdc = GetDC(hWnd);
	BitBlt(hdc, 0, 0, memDCRect.right, memDCRect.bottom, hMemDC, 0, 0, SRCCOPY);
	ReleaseDC(hWnd, hdc);
}


void CPathFinding_Renderer::RenderGrid()
{
	int x = 0;
	int y = 0;
	HPEN hOldPen = (HPEN)SelectObject(hMemDC, hGridPen);
	for (int w = 0; w <= GRID_WIDTH; w++)
	{
		MoveToEx(hMemDC, x + g_scrollOffsetX, g_scrollOffsetY, NULL);
		LineTo(hMemDC, x + g_scrollOffsetX, GRID_HEIGHT * g_gridSize + g_scrollOffsetY);
		x += g_gridSize;
	}
	for (int h = 0; h <= GRID_HEIGHT; h++)
	{
		MoveToEx(hMemDC, g_scrollOffsetX, y + g_scrollOffsetY, NULL);
		LineTo(hMemDC, GRID_WIDTH * g_gridSize + g_scrollOffsetX, y + g_scrollOffsetY);
		y += g_gridSize;
	}
	SelectObject(hMemDC, hOldPen);
}


void CPathFinding_Renderer::RenderObstacle()
{
	HBRUSH hOldBrush = (HBRUSH)SelectObject(hMemDC, hObstacleBrush);
	SelectObject(hMemDC, GetStockObject(NULL_PEN));

	for (int w = 0; w < GRID_WIDTH; w++)
	{
		for (int h = 0; h < GRID_HEIGHT; h++)
		{
			if (grid[h][w] == 1)
			{
				int gridX = w * g_gridSize + g_scrollOffsetX;
				int gridY = h * g_gridSize + g_scrollOffsetY;
				Rectangle(hMemDC, gridX, gridY, gridX + g_gridSize + 2, gridY + g_gridSize + 2);
			}
		}
	}
	SelectObject(hMemDC, hOldBrush);
}

void CPathFinding_Renderer::RenderInfoPanel(CPathFinding* pathFinding)
{
	// 화면 크기 기준 위치 계산
	int panelWidth = memDCRect.right * 0.23;
	int panelHeight = memDCRect.bottom * 0.5;
	int panelX = memDCRect.right - panelWidth;
	int panelY = 0;

	RECT panelRect = { panelX, panelY, panelX + panelWidth, panelY + panelHeight };

	// 하얀색 배경
	HBRUSH hWhiteBrush = CreateSolidBrush(RGB(255, 255, 255));
	FillRect(hMemDC, &panelRect, hWhiteBrush);
	DeleteObject(hWhiteBrush);

	// 텍스트 출력
	SetBkMode(hMemDC, TRANSPARENT);  // 배경 투명 처리
	SetTextColor(hMemDC, RGB(0, 0, 0)); // 검정색 글씨

	const wchar_t* lines[] = { L"출발지 설정 - 휠 버튼", L"목적지 설정 - 우클릭", L"장애물 설정 - 좌클릭", L"'3번' 길찾기", L"'4번' 길 찾기 One_Step", L"'5번' G - 맨하튼 / 뉴클리드 선택", L"'6번' H - 맨하튼 / 뉴클리드 선택" };
	int lineHeight = 20;
	int lineCount = 7;
	for (int i = 0; i < lineCount; ++i)
	{
		TextOutW(hMemDC, panelX + 5, panelY + 5 + i * lineHeight, lines[i], lstrlenW(lines[i]));
	}

	// 거리 함수 정보 출력
	if (pathFinding != nullptr)
	{
		wchar_t distanceInfo[128];

		const wchar_t* gType = pathFinding->bDistanceG_Manhattan ? L"맨하튼" : L"뉴클리드";
		const wchar_t* hType = pathFinding->bDistanceH_Manhattan ? L"맨하튼" : L"뉴클리드";

		swprintf_s(distanceInfo, L"G - %s, H - %s", gType, hType);

		// 한 줄 더 아래 출력
		TextOutW(hMemDC, panelX + 5, panelY + 5 + (lineCount + 1) * lineHeight, distanceInfo, lstrlenW(distanceInfo));
	}
}

void CPathFinding_Renderer::RenderStartPos(CPathFinding* pathFinding)
{
	HBRUSH hOldBrush = (HBRUSH)SelectObject(hMemDC, hStartPosBrush);
	SelectObject(hMemDC, GetStockObject(NULL_PEN));

	int sy = pathFinding->startPos.first;
	int sx = pathFinding->startPos.second;
	int gridX = sx * g_gridSize + g_scrollOffsetX;
	int gridY = sy * g_gridSize + g_scrollOffsetY;

	Rectangle(hMemDC, gridX, gridY, gridX + g_gridSize + 2, gridY + g_gridSize + 2);
	SelectObject(hMemDC, hOldBrush);

}

void CPathFinding_Renderer::RenderEndPos(CPathFinding* pathFinding)
{
	HBRUSH hOldBrush = (HBRUSH)SelectObject(hMemDC, hEndPosBrush);
	SelectObject(hMemDC, GetStockObject(NULL_PEN));

	int sy = pathFinding->endPos.first;
	int sx = pathFinding->endPos.second;
	int gridX = sx * g_gridSize + g_scrollOffsetX;
	int gridY = sy * g_gridSize + g_scrollOffsetY;

	Rectangle(hMemDC, gridX, gridY, gridX + g_gridSize + 2, gridY + g_gridSize + 2);
	SelectObject(hMemDC, hOldBrush);
}

void CPathFinding_Renderer::RenderOpenList(CPathFinding* pathFinding)
{
	HBRUSH hOldBrush = (HBRUSH)SelectObject(hMemDC, hOpenBrush);
	std::unordered_map<pair<int, int>, Node*, PositionHash> openList = pathFinding->openList;
	for (const auto& entry : openList)
	{
		Node* vNode = entry.second;
		int gridX = vNode->x * g_gridSize + g_scrollOffsetX;
		int gridY = vNode->y * g_gridSize + g_scrollOffsetY;
		Rectangle(hMemDC, gridX, gridY, gridX + g_gridSize, gridY + g_gridSize);
		//RenderNodeInfo(vNode);
	}
		
	
	SelectObject(hMemDC, hOldBrush);
}

void CPathFinding_Renderer::RenderClosedList(CPathFinding* pathFinding)
{
	HBRUSH hOldBrush = (HBRUSH)SelectObject(hMemDC, hClosedBrush);

	std::unordered_map<pair<int, int>, Node*, PositionHash> closedList = pathFinding->closedList;
	for (const auto& entry : closedList)
	{
		Node* vNode = entry.second;
		if (vNode->y == pathFinding->endPos.first && vNode->x == pathFinding->endPos.second)
			continue;
		int gridX = vNode->x * g_gridSize + g_scrollOffsetX;
		int gridY = vNode->y * g_gridSize + g_scrollOffsetY;
		Rectangle(hMemDC, gridX, gridY, gridX + g_gridSize, gridY + g_gridSize);
		//RenderNodeInfo(vNode);
	}
	SelectObject(hMemDC, hOldBrush);
}


void CPathFinding_Renderer::RenderPath(Node* destNode)
{
	if (destNode == nullptr)
		return;

	// 빨간색 선을 위한 펜
	HPEN hRedPen = CreatePen(PS_SOLID, 2, RGB(255, 0, 0));
	HPEN hOldPen = (HPEN)SelectObject(hMemDC, hRedPen);

	Node* current = destNode;
	while (current->parent != nullptr)
	{
		// 현재 노드 좌표 중심
		int x1 = current->x * g_gridSize + g_gridSize / 2 + g_scrollOffsetX;
		int y1 = current->y * g_gridSize + g_gridSize / 2 + g_scrollOffsetY;

		// 부모 노드 좌표 중심
		int x2 = current->parent->x * g_gridSize + g_gridSize / 2 + g_scrollOffsetX;
		int y2 = current->parent->y * g_gridSize + g_gridSize / 2 + g_scrollOffsetY;

		// 선 그리기
		MoveToEx(hMemDC, x1, y1, NULL);
		LineTo(hMemDC, x2, y2);

		current = current->parent;
	}

	// 펜 정리
	SelectObject(hMemDC, hOldPen);
	DeleteObject(hRedPen);
}

void CPathFinding_Renderer::RenderParentNode(CPathFinding* pathFinding)
{
	if (pathFinding == nullptr)
		return;

	HPEN hRedPen = CreatePen(PS_SOLID, 1, RGB(255, 0, 0));
	HPEN hOldPen = (HPEN)SelectObject(hMemDC, hRedPen);

	// ----------- ClosedList 처리 -------------
	std::unordered_map<pair<int, int>, Node*, PositionHash> closedList = pathFinding->closedList;
	for (const auto& entry : closedList)
	{
		Node* vNode = entry.second;
		if (vNode->parent == NULL)
			continue;

		int dx = vNode->parent->x - vNode->x;
		int dy = vNode->parent->y - vNode->y;

		int cx = vNode->x * g_gridSize + g_gridSize / 2 + g_scrollOffsetX;
		int cy = vNode->y * g_gridSize + g_gridSize / 2 + g_scrollOffsetY;
		int tx = cx;
		int ty = cy;

		if (dx == 0 && dy == -1)         ty = cy - g_gridSize / 2;                   // ↑
		else if (dx == 0 && dy == 1)     ty = cy + g_gridSize / 2;                   // ↓
		else if (dx == -1 && dy == 0)    tx = cx - g_gridSize / 2;                   // ←
		else if (dx == 1 && dy == 0)     tx = cx + g_gridSize / 2;                   // →
		else if (dx == -1 && dy == -1) { tx = cx - g_gridSize / 2; ty = cy - g_gridSize / 2; } // ↖
		else if (dx == 1 && dy == -1) { tx = cx + g_gridSize / 2; ty = cy - g_gridSize / 2; } // ↗
		else if (dx == -1 && dy == 1) { tx = cx - g_gridSize / 2; ty = cy + g_gridSize / 2; } // ↙
		else if (dx == 1 && dy == 1) { tx = cx + g_gridSize / 2; ty = cy + g_gridSize / 2; } // ↘
		else continue;

		MoveToEx(hMemDC, cx, cy, NULL);
		LineTo(hMemDC, tx, ty);
	}



	// ----------- OpenList 처리 ---------------
	std::unordered_map<pair<int, int>, Node*, PositionHash> openList = pathFinding->openList;
	for (const auto& entry : openList)
	{
		Node* vNode = entry.second;
		if (vNode->parent == NULL)
			continue;

		int dx = vNode->parent->x - vNode->x;
		int dy = vNode->parent->y - vNode->y;

		int cx = vNode->x * g_gridSize + g_gridSize / 2 + g_scrollOffsetX;
		int cy = vNode->y * g_gridSize + g_gridSize / 2 + g_scrollOffsetY;
		int tx = cx;
		int ty = cy;

		if (dx == 0 && dy == -1)         ty = cy - g_gridSize / 2;
		else if (dx == 0 && dy == 1)     ty = cy + g_gridSize / 2;
		else if (dx == -1 && dy == 0)    tx = cx - g_gridSize / 2;
		else if (dx == 1 && dy == 0)     tx = cx + g_gridSize / 2;
		else if (dx == -1 && dy == -1) { tx = cx - g_gridSize / 2; ty = cy - g_gridSize / 2; }
		else if (dx == 1 && dy == -1) { tx = cx + g_gridSize / 2; ty = cy - g_gridSize / 2; }
		else if (dx == -1 && dy == 1) { tx = cx - g_gridSize / 2; ty = cy + g_gridSize / 2; }
		else if (dx == 1 && dy == 1) { tx = cx + g_gridSize / 2; ty = cy + g_gridSize / 2; }
		else continue;

		MoveToEx(hMemDC, cx, cy, NULL);
		LineTo(hMemDC, tx, ty);
	}

	// ----------- 복원 --------------
	SelectObject(hMemDC, hOldPen);
	DeleteObject(hRedPen);
}



void CPathFinding_Renderer::RenderNodeInfo(Node* node)
{
	int gridX = node->x * g_gridSize + g_scrollOffsetX;
	int girdY = node->y * g_gridSize + g_scrollOffsetY;

	// 글자 출력 사각형
	RECT rect;

	// 글자 색상 및 배경 투명 처리
	SetTextColor(hMemDC, RGB(0, 0, 0));        // 검은색
	SetBkMode(hMemDC, TRANSPARENT);            // 배경 투명

	// 글자 크기 조정
	int fontHeight = g_gridSize * 0.2f;

	HFONT hFont = CreateFontW(
		fontHeight, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE,
		HANGEUL_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
		DEFAULT_QUALITY, DEFAULT_PITCH | FF_DONTCARE,
		L"Consolas"
	);
	HFONT hOldFont = (HFONT)SelectObject(hMemDC, hFont);

	// 각 줄 출력
	wchar_t buf[32];

	// 줄 높이 (간격 포함)
	int lineHeight = g_gridSize / 3;

	// g
	swprintf_s(buf, L"g:%.2lf \n", node->g);
	rect = { gridX + 1, girdY + 1, gridX + g_gridSize - 1, girdY + lineHeight };
	DrawTextW(hMemDC, buf, -1, &rect, DT_LEFT | DT_TOP | DT_SINGLELINE | DT_NOPREFIX);

	// h
	swprintf_s(buf, L"h:%.2lf \n", node->h);
	rect.top += lineHeight;
	rect.bottom += lineHeight;
	DrawTextW(hMemDC, buf, -1, &rect, DT_LEFT | DT_TOP | DT_SINGLELINE | DT_NOPREFIX);

	// f
	swprintf_s(buf, L"f:%.2lf \n", node->f);
	rect.top += lineHeight;
	rect.bottom += lineHeight;
	DrawTextW(hMemDC, buf, -1, &rect, DT_LEFT | DT_TOP | DT_SINGLELINE | DT_NOPREFIX);

	// 정리
	SelectObject(hMemDC, hOldFont);
	DeleteObject(hFont);
}

