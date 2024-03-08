#include <iostream>
#include <vector>
#include <queue>
#include <climits>

using namespace std;

// Define directions: up, down, left, right
const int dx[] = {0, 0, -1, 1, 1, 1, -1, -1};
const int dy[] = {-1, 1, 0, 0, -1, 1, 1, -1};

// Function to generate distance map using BFS
vector<vector<int>> generateDistanceMap(vector<vector<int>>& grid) 
{
    int N = grid.size();
    vector<vector<int>> distanceMap(N, vector<int>(N, INT_MAX));
    queue<pair<int, int>> q;

    // Initialize the queue and distance map
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            if (grid[i][j] == 1) {
                distanceMap[i][j] = 0;
                q.push({i, j});
            }
        }
    }

    // Perform BFS
    while (!q.empty()) {
        auto [x, y] = q.front();
        q.pop();

        for (int k = 0; k < 8; ++k) {
            int nx = x + dx[k];
            int ny = y + dy[k];
            if (nx >= 0 && nx < N && ny >= 0 && ny < N && distanceMap[nx][ny] == INT_MAX) {
                distanceMap[nx][ny] = distanceMap[x][y] + 1;
                q.push({nx, ny});
            }
        }
    }
    return distanceMap;
}

int main() {
    // Define the grid
    // vector<vector<int>> grid = {
    //     {1, 1, 1, 1, 1, 1, 1},
    //     {1, 0, 0, 0, 0, 0, 1},
    //     {1, 0, 0, 0, 0, 0, 1},
    //     {1, 0, 0, 0, 0, 1, 1},
    //     {1, 0, 0, 0, 0, 1, 0},
    //     {0, 1, 0, 0, 1, 0, 0},
    //     {0, 0, 0, 0, 0, 0, 0},
    // };

    vector<vector<int>> grid = {
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0}
    };
    
    int N = grid.size();
    // Generate distance map
    vector<vector<int>> distanceMap = generateDistanceMap(grid);

    cout << "Original Map:\n";
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            cout << grid[i][j] << " ";
        }
        cout << endl;
    }
    // Output the distance map
    cout << "Distance Map:\n";
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            cout << distanceMap[i][j] << " ";
        }
        cout << endl;
    }

    return 0;
}
