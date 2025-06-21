#include <ctime>   // rand 초기화용
#include <cstdlib> // rand, srand
#include <queue>
#include "PathFinding.h"
#include "PathFinding_Renderer.h"
#include "PathFinding_TestProgram.h"


bool   g_autoRunning = false;      // true  → 계속 반복
bool   g_testInFlight = false;      // 현재 길찾기 수행 중?
int regionId[GRID_HEIGHT][GRID_WIDTH];


// -----------------------------------
// 미로 자동 생성 관련
// -----------------------------------
void AddLoops(int loopCnt)
{
    // 내부(테두리 제외)에서 벽 2×2 덩어리를 골라 없애기
    while (loopCnt--)
    {
        // 후보는 짝수 좌표(2,4,6…) – 각 2×2 방의 좌상단 기준
        int y = 2 + (rand() % ((GRID_HEIGHT - 4) / 2)) * 2;
        int x = 2 + (rand() % ((GRID_WIDTH - 4) / 2)) * 2;

        // 가로로 이웃한 방을 연결할지, 세로로 연결할지 무작위
        if (rand() & 1)  // 가로
        {
            // 중간 벽 2열(x+2, x+3)을 확인 후 제거
            if (x + 4 < GRID_WIDTH - 1 &&
                g_grid[y][x + 2] == 1 && g_grid[y][x + 3] == 1 &&
                g_grid[y + 1][x + 2] == 1 && g_grid[y + 1][x + 3] == 1)
            {
                g_grid[y][x + 2] = g_grid[y][x + 3] =
                    g_grid[y + 1][x + 2] = g_grid[y + 1][x + 3] = 0;
            }
        }
        else  // 세로
        {
            // 중간 벽 2행(y+2, y+3)을 확인 후 제거
            if (y + 4 < GRID_HEIGHT - 1 &&
                g_grid[y + 2][x] == 1 && g_grid[y + 3][x] == 1 &&
                g_grid[y + 2][x + 1] == 1 && g_grid[y + 3][x + 1] == 1)
            {
                g_grid[y + 2][x] = g_grid[y + 3][x] =
                    g_grid[y + 2][x + 1] = g_grid[y + 3][x + 1] = 0;
            }
        }
    }
}

void GenerateMaze()
{
    // (1) 전체를 벽으로 초기화
    for (int y = 0; y < GRID_HEIGHT; ++y)
        for (int x = 0; x < GRID_WIDTH; ++x)
            g_grid[y][x] = 1;

    // (2) DFS-백트래킹으로 ‘완전(사이클 없는)’ 2×2 미로 생성
    srand((unsigned)time(0));
    carve(2, 2);   // 반드시 짝수 시작점

    // (3) 일부 벽을 무작위로 제거해 루프(사이클) 추가
    int loopCount = (GRID_HEIGHT * GRID_WIDTH) / 100;  // 필요하면 비율 조절
    AddLoops(loopCount);
}

void carve(int y, int x)
{
    int dy[4] = { 0, 0, 2, -2 };
    int dx[4] = { 2, -2, 0, 0 };

    g_grid[y][x] = 0;
    g_grid[y][x + 1] = 0;
    g_grid[y + 1][x] = 0;
    g_grid[y + 1][x + 1] = 0;

    for (int i = 0; i < 4; ++i)
    {
        int j = rand() % 4;
        int temp;

        temp = dy[i]; dy[i] = dy[j]; dy[j] = temp;
        temp = dx[i]; dx[i] = dx[j]; dx[j] = temp;
    }

    for (int i = 0; i < 4; ++i)
    {
        int ny = y + dy[i] * 2;
        int nx = x + dx[i] * 2;

        if (ny < 2 || ny >= GRID_HEIGHT - 1 || nx < 2 || nx >= GRID_WIDTH - 1)
            continue;

        if (g_grid[ny][nx] == 1)
        {
            // 경로 연결 (중간 2칸 + 도착지점 2x2 확장)
            int my = y + dy[i];
            int mx = x + dx[i];

            g_grid[my][mx] = 0;
            g_grid[my + 1][mx] = 0;
            g_grid[my][mx + 1] = 0;
            g_grid[my + 1][mx + 1] = 0;

            carve(ny, nx);
        }
    }
}

// -----------------------------------
// 동굴 자동 생성 관련
// -----------------------------------

void Smooth(int iter)
{
    int y, x, i, j;
    static char temp[GRID_HEIGHT][GRID_WIDTH];

    while (iter--)
    {
        for (y = 1; y < GRID_HEIGHT - 1; ++y)
        {
            for (x = 1; x < GRID_WIDTH - 1; ++x)
            {
                int wallCnt = 0;
                for (i = -1; i <= 1; ++i)
                    for (j = -1; j <= 1; ++j)
                        if (g_grid[y + i][x + j] == WALL) ++wallCnt;

                temp[y][x] = (wallCnt >= 5) ? WALL : PATH;
            }
        }
        /* 바깥 테두리는 그대로 두고 안쪽만 복사 */
        for (y = 1; y < GRID_HEIGHT - 1; ++y)
            for (x = 1; x < GRID_WIDTH - 1; ++x)
                g_grid[y][x] = temp[y][x];
    }
}

// Flood-Fill 로 영역 라벨링, id 반환
int LabelRegions(Point reps[], int maxRegion)
{
    int dY[4] = { -1, 1, 0, 0 };
    int dX[4] = { 0, 0,-1, 1 };

    int y, x, i;
    int id = 0;
    for (y = 0; y < GRID_HEIGHT; ++y)
        for (x = 0; x < GRID_WIDTH; ++x)
            regionId[y][x] = -1;

    for (y = 0; y < GRID_HEIGHT; ++y)
    {
        for (x = 0; x < GRID_WIDTH; ++x)
        {
            if (g_grid[y][x] == PATH && regionId[y][x] == -1)
            {
                if (id >= maxRegion) return id;
                reps[id].y = y;  /* 대표 좌표 저장 */
                reps[id].x = x;

                std::queue<Point> q;
                q.push({ y, x });
                regionId[y][x] = id;

                while (!q.empty())
                {
                    Point p = q.front(); q.pop();
                    for (i = 0; i < 4; ++i)
                    {
                        int ny = p.y + dY[i];
                        int nx = p.x + dX[i];
                        if (ny < 0 && ny >= GRID_HEIGHT && nx < 0 && nx >= GRID_WIDTH)
                            continue;
                        if (g_grid[ny][nx] == PATH && regionId[ny][nx] == -1)
                        {
                            regionId[ny][nx] = id;
                            q.push({ ny, nx });
                        }
                    }
                }
                ++id;
            }
        }
    }
    return id;
}

// 두 점 사이를 직선(─, │)으로 파내어 연결
void CarveLine(Point a, Point b)
{
    int y = a.y, x = a.x;
    while (x != b.x)
    {
        g_grid[y][x] = PATH;
        x += (x < b.x) ? 1 : -1;
    }
    while (y != b.y)
    {
        g_grid[y][x] = PATH;
        y += (y < b.y) ? 1 : -1;
    }
    g_grid[b.y][b.x] = PATH;
}

// 모든 영역을 하나로 연결
void ConnectRegions()
{
    const int MAX_REG = 256;
    Point reps[MAX_REG];

    int regionCnt = LabelRegions(reps, MAX_REG);
    if (regionCnt <= 1)
        return;

    // 0번 영역을 기준으로 나머지를 차례로 연결
    int base = 0;
    while (regionCnt > 1)
    {
        int bestA = -1, bestB = -1, bestDist = 999999;

        // 가장 가까운 두 대표점을 찾음
        int a, b;
        for (a = 0; a < regionCnt; ++a)
        {
            for (b = a + 1; b < regionCnt; ++b)
            {
                int dy = reps[a].y - reps[b].y;
                int dx = reps[a].x - reps[b].x;
                int dist = dy * dy + dx * dx;
                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestA = a;
                    bestB = b;
                }
            }
        }

        // 두 영역 직선 연결 
        CarveLine(reps[bestA], reps[bestB]);

        // 다시 영역 재라벨링
        regionCnt = LabelRegions(reps, MAX_REG);
    }
}

void GenerateCave()
{
    int y, x;

    // 1) 초깃값 무작위
    srand((unsigned)time(0));
    for (y = 0; y < GRID_HEIGHT; ++y)
        for (x = 0; x < GRID_WIDTH; ++x)
            g_grid[y][x] = (rand() % 100 < 45) ? WALL : PATH;

    // 2) 셀룰러 오토마타 스무딩
    Smooth(4);

    // 3) 모든 영역 연결
    ConnectRegions();
}


// ------------- 길 찾기 자동화 관련 ----------------

void StartPathFindingTest(HWND hWnd, CPathFinding* pathFinding, CPathFinding_Renderer* renderer)
{
    g_testInFlight = true;                 // 실행 중 플래그

    // 1) 새 미로 생성 
    if (rand() % 2 == 0)
        GenerateMaze();
    else
        GenerateCave();

    // 2) 랜덤 출발·도착 좌표 확보
    auto RandomPoint = []() -> std::pair<int, int>
        {
            while (true)
            {
                int y = rand() % GRID_HEIGHT;
                int x = rand() % GRID_WIDTH;
                if (g_grid[y][x] == 0) return { y, x };
            }
        };

    std::pair<int, int> startPos, endPos;
    do { startPos = RandomPoint(); endPos = RandomPoint(); } while (startPos == endPos);

    if (pathFinding) 
    {
        delete pathFinding;
        pathFinding = nullptr;
    }
    pathFinding = new CPathFinding(renderer, g_grid, startPos);
    pathFinding->SetEndPos(endPos);

    // 4) 길찾기 수행
    bool found = pathFinding->FindPath_JPS();

    // 5) 결과 처리
    if (!found || pathFinding->path.empty())
    {
        MessageBox(hWnd, L"경로를 찾지 못했습니다.", L"PathFinding", MB_OK);
        g_autoRunning = false;            // 실패 → 반복 중단
    }

    g_testInFlight = false;              // 실행 완료

    // 7) 다음 테스트 예약
    if (g_autoRunning)                   // 자동 모드가 여전히 true 면
        PostMessage(hWnd, WM_AUTOTEST, 0, 0);

    Sleep(500);
}

