#pragma once
#include "Define_Grid.h"

extern char g_grid[GRID_HEIGHT][GRID_WIDTH];
extern bool   g_autoRunning;
extern bool   g_testInFlight;

// 미로 길 찾기 자동화 수행 관련
#define WM_AUTOTEST   (WM_APP + 1)   // 다음 테스트 예약용 메시지
void StartPathFindingTest(HWND hWnd, CPathFinding* pathFinding, CPathFinding_Renderer* renderer);   // 한 싸이클 수행


// -----------미로 자동 생성 관련--------------
void GenerateMaze();
void carve(int y, int x);
void AddLoops(int loopCnt);


// -----------동굴 자동 생성 관련---------------
#define WALL 1
#define PATH 0

struct Point 
{
    int y;
    int x;
};

void Smooth(int iter);
int LabelRegions(Point reps[], int maxRegion);
void CarveLine(Point a, Point b);
void ConnectRegions();
void GenerateCave();

