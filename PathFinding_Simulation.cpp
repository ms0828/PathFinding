
#include <Windows.h>
#include <windowsx.h>
#include "framework.h"
#include "PathFinding_Simulation.h"
#include "PathFinding.h"
#include "PathFinding_Renderer.h"
#include "Define_Grid.h"

#define MAX_LOADSTRING 100


// 전역 변수:
HINSTANCE hInst;                                // 현재 인스턴스입니다.
WCHAR szTitle[MAX_LOADSTRING];                  // 제목 표시줄 텍스트입니다.
WCHAR szWindowClass[MAX_LOADSTRING];            // 기본 창 클래스 이름입니다.


char g_grid[GRID_HEIGHT][GRID_WIDTH];
bool g_bErase = false;
bool g_bDrag = false;


int g_gridSize = GRID_SIZE;
int g_scrollOffsetX = 0;
int g_scrollOffsetY = 0;


// 함수 선언
LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);


CPathFinding* pathFinding = nullptr;
CPathFinding_Renderer* renderer;

int APIENTRY wWinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPWSTR lpCmdLine, _In_ int       nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);


    // 전역 문자열 초기화
    LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
    LoadStringW(hInstance, IDC_PATHFINDINGSIMULATION, szWindowClass, MAX_LOADSTRING);


    WNDCLASSEXW wcex;
    wcex.cbSize = sizeof(WNDCLASSEX);
    wcex.style = CS_HREDRAW | CS_VREDRAW;
    wcex.lpfnWndProc = WndProc;
    wcex.cbClsExtra = 0;
    wcex.cbWndExtra = 0;
    wcex.hInstance = hInstance;
    wcex.hIcon = LoadIcon(hInstance, MAKEINTRESOURCE(IDC_PATHFINDINGSIMULATION));
    wcex.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    wcex.lpszMenuName = MAKEINTRESOURCEW(IDC_PATHFINDINGSIMULATION);
    wcex.lpszClassName = szWindowClass;
    wcex.hIconSm = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));
    RegisterClassExW(&wcex);


    hInst = hInstance; // 인스턴스 핸들을 전역 변수에 저장
    HWND hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, nullptr, nullptr, hInstance, nullptr);
    if (!hWnd)
        return 0;
    ShowWindow(hWnd, nCmdShow);
    UpdateWindow(hWnd);


    MSG msg;

    // 기본 메시지 루프입니다:
    while (GetMessage(&msg, nullptr, 0, 0))
    {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }

    return (int)msg.wParam;
}


LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
    case WM_LBUTTONDOWN:
    {
        g_bDrag = true;
        int xPos = GET_X_LPARAM(lParam) - g_scrollOffsetX;
        int yPos = GET_Y_LPARAM(lParam) - g_scrollOffsetY;
        int xIdx = xPos / g_gridSize;
        int yIdx = yPos / g_gridSize;
        if (xIdx < 0 || xIdx >= GRID_WIDTH || yIdx < 0 || yIdx >= GRID_HEIGHT)
            break;
        if (g_grid[yIdx][xIdx] == 1)
            g_bErase = true;
        else
            g_bErase = false;
    }
    break;
    case WM_LBUTTONUP:
        g_bDrag = false;
        break;
    case WM_MBUTTONDOWN:
    {
        int xPos = GET_X_LPARAM(lParam) - g_scrollOffsetX;
        int yPos = GET_Y_LPARAM(lParam) - g_scrollOffsetY;
        int xIdx = xPos / g_gridSize;
        int yIdx = yPos / g_gridSize;
        if (xIdx < 0 || xIdx >= GRID_WIDTH || yIdx < 0 || yIdx >= GRID_HEIGHT)
            break;
        if (g_grid[yIdx][xIdx] == 1)
            break;

        if (pathFinding != nullptr)
            delete pathFinding;
        pathFinding = new CPathFinding(renderer, g_grid, { yIdx, xIdx });
        InvalidateRect(hWnd, NULL, false);
    }
    break;
    case WM_RBUTTONDOWN:
    {
        if (pathFinding != nullptr)
        {
            int xPos = GET_X_LPARAM(lParam) - g_scrollOffsetX;
            int yPos = GET_Y_LPARAM(lParam) - g_scrollOffsetY;
            int xIdx = xPos / g_gridSize;
            int yIdx = yPos / g_gridSize;
            if (xIdx < 0 || xIdx >= GRID_WIDTH || yIdx < 0 || yIdx >= GRID_HEIGHT)
                break;
            if (g_grid[yIdx][xIdx] == 1)
                break;
            pathFinding->SetEndPos({ yIdx, xIdx });
            pathFinding->Init();
            InvalidateRect(hWnd, NULL, false);
        }
    }
    break;
    case WM_MOUSEMOVE:
    {
        if (g_bDrag)
        {
            int xPos = GET_X_LPARAM(lParam) - g_scrollOffsetX;
            int yPos = GET_Y_LPARAM(lParam) - g_scrollOffsetY;
            int xIdx = xPos / g_gridSize;
            int yIdx = yPos / g_gridSize;
            if (xIdx < 0 || xIdx >= GRID_WIDTH || yIdx < 0 || yIdx >= GRID_HEIGHT)
                break;
            g_grid[yIdx][xIdx] = !g_bErase;
            InvalidateRect(hWnd, NULL, false);
        }
    }
    break;
    case WM_MOUSEWHEEL:
    {
        short delta = GET_WHEEL_DELTA_WPARAM(wParam);  // +120 또는 -120
        if (delta > 0)
        {
            if (g_gridSize < 100)
                g_gridSize += 2;  // 확대
        }
        else
        {
            if (g_gridSize > 4)
                g_gridSize -= 2;  // 축소
        }

        InvalidateRect(hWnd, NULL, FALSE);  // 다시 그리기
    }
    break;
    case WM_KEYDOWN:
    {
        switch (wParam)
        {
        case '3':
            if (pathFinding)
                pathFinding->FindPath_JPS();
            break;
        case '4':
            if (pathFinding)
                pathFinding->FindPath_JPS_OneStep();
            break;
        case '5':
            if (pathFinding)
                pathFinding->bDistanceG_Manhattan = !pathFinding->bDistanceG_Manhattan;
            break;
        case '6':
            if (pathFinding)
                pathFinding->bDistanceH_Manhattan = !pathFinding->bDistanceH_Manhattan;
            break;
        case VK_LEFT:
            g_scrollOffsetX += 20;
            break;
        case VK_RIGHT:
            g_scrollOffsetX -= 20;
            break;
        case VK_UP:
            g_scrollOffsetY += 20;
            break;
        case VK_DOWN:
            g_scrollOffsetY -= 20;
            break;
        }
        InvalidateRect(hWnd, NULL, FALSE);
    }
    break;
    case WM_CREATE:
    {
        HDC hdc = GetDC(hWnd);
        renderer = new CPathFinding_Renderer(hWnd, hdc, g_grid);
        ReleaseDC(hWnd, hdc);
    }
    break;
    case WM_COMMAND:
    {
        int wmId = LOWORD(wParam);
        switch (wmId)
        {
        case IDM_EXIT:
            DestroyWindow(hWnd);
            break;
        default:
            return DefWindowProc(hWnd, message, wParam, lParam);
        }
    }
    break;
    case WM_PAINT:
    {
        PAINTSTRUCT ps;
        HDC hdc = BeginPaint(hWnd, &ps);
        renderer->Rendering(pathFinding);
        EndPaint(hWnd, &ps);
    }
    break;
    case WM_DESTROY:
        delete renderer;
        if (pathFinding != nullptr)
            delete pathFinding;
        PostQuitMessage(0);
        break;
    case WM_SIZE:
    {
        HDC hdc = GetDC(hWnd);
        renderer->ReSize(hdc);
        ReleaseDC(hWnd, hdc);
    }
    break;
    default:
        return DefWindowProc(hWnd, message, wParam, lParam);
    }
    return 0;
}
