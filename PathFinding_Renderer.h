#pragma once

#include <Windows.h>
#include "Define_Grid.h"

class CPathFinding;
class Node;

extern int g_gridSize;
extern int g_scrollOffsetX;
extern int g_scrollOffsetY;


class CPathFinding_Renderer
{
public:
	CPathFinding_Renderer(HWND _hWnd, HDC hdc, char(*_grid)[GRID_WIDTH])
	{
		grid = _grid;
		hWnd = _hWnd;

		// 윈도우 생성 시 현 윈도우 크기와 동일한 메모리 DC 생성
		GetClientRect(hWnd, &memDCRect);
		hMemDCBitMap = CreateCompatibleBitmap(hdc, memDCRect.right, memDCRect.bottom);
		hMemDC = CreateCompatibleDC(hdc);
		hMemDCBitMap_old = (HBITMAP)SelectObject(hMemDC, hMemDCBitMap);

		// GDI 오브젝트 생성
		hGridPen = CreatePen(PS_SOLID, 1, RGB(200, 200, 200));
		hObstacleBrush = CreateSolidBrush(RGB(100, 100, 100)); // gray
		hOpenBrush = CreateSolidBrush(RGB(0, 0, 200)); // blue
		hClosedBrush = CreateSolidBrush(RGB(200, 200, 0)); // yellow
		hStartPosBrush = CreateSolidBrush(RGB(0, 200, 0)); // green
		hEndPosBrush = CreateSolidBrush(RGB(200, 0, 0)); // red

	}

	~CPathFinding_Renderer()
	{
		SelectObject(hMemDC, hMemDCBitMap_old);
		DeleteObject(hMemDC);
		DeleteObject(hMemDCBitMap);

		DeleteObject(hGridPen);
		DeleteObject(hObstacleBrush);
		DeleteObject(hOpenBrush);
		DeleteObject(hClosedBrush);
		DeleteObject(hStartPosBrush);
		DeleteObject(hEndPosBrush);
	}

	void ReSize(HDC hdc);

	void Rendering(CPathFinding* pathFinding);
	void RenderObstacle();
	void RenderInfoPanel(CPathFinding* pathFinding);
	void RenderGrid();
	void RenderStartPos(CPathFinding* pathFinding);
	void RenderEndPos(CPathFinding* pathFinding);
	void RenderOpenList(CPathFinding* pathFinding);
	void RenderClosedList(CPathFinding* pathFinding);
	void RenderParentNode(CPathFinding* pathFinding);
	void RenderPath(Node* destNode);
	void RenderNodeInfo(Node* node);

public:
	HWND hWnd;

	HBITMAP hMemDCBitMap;
	HBITMAP hMemDCBitMap_old;
	HDC hMemDC;
	RECT memDCRect;
	HBRUSH hObstacleBrush;
	HBRUSH hOpenBrush;
	HBRUSH hClosedBrush;
	HBRUSH hStartPosBrush;
	HBRUSH hEndPosBrush;
	HPEN hGridPen;

	char(*grid)[GRID_WIDTH];

};