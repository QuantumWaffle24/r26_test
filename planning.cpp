#include "planning.h"
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>

using namespace std;

Planner::Planner(const vector<vector<bool>> &grid) : grid(grid) {
  rows = (int)grid.size();
  cols = (int)grid[0].size();
}

bool Planner::isvalid(int x, int y) const {
  return (x >= 0 && x < rows && y >= 0 && y < cols && !grid[x][y]);
}

// Use Manhattan heuristic (admissible for 4-neighbour moves)
double Planner::heuristic(int x1, int y1, int x2, int y2) const {
  return std::abs(x1 - x2) + std::abs(y1 - y2);
}

vector<pair<int, int>> Planner::pathplanning(pair<int, int> start,
                                             pair<int, int> goal) {
  vector<pair<int, int>> path;

  if (!isvalid(start.first, start.second) || !isvalid(goal.first, goal.second)) {
    return path; // empty
  }

  // 4-neighbour moves (N, S, E, W)
  const int dx[4] = {-1, 1, 0, 0};
  const int dy[4] = {0, 0, -1, 1};

  // gScore (cost so far) initialized to +inf
  vector<vector<double>> g(rows, vector<double>(cols, numeric_limits<double>::infinity()));
  // parent pointers
  vector<vector<pair<int,int>>> parent(rows, vector<pair<int,int>>(cols, {-1, -1}));
  // min-heap on f = g + h
  using Node = pair<double, pair<int,int>>; // (fScore, (x,y))
  priority_queue<Node, vector<Node>, greater<Node>> pq;

  int sx = start.first, sy = start.second;
  int gx = goal.first,  gy = goal.second;

  g[sx][sy] = 0.0;
  pq.push({heuristic(sx, sy, gx, gy), {sx, sy}});

  vector<vector<bool>> inOpen(rows, vector<bool>(cols, false));
  inOpen[sx][sy] = true;

  while (!pq.empty()) {
    auto [fscore, cur] = pq.top(); pq.pop();
    int x = cur.first, y = cur.second;

    if (x == gx && y == gy) {
      // reconstruct path
      pair<int,int> p = {gx, gy};
      while (!(p.first == -1 && p.second == -1)) {
        path.push_back(p);
        p = parent[p.first][p.second];
      }
      reverse(path.begin(), path.end());
      return path;
    }

    for (int k = 0; k < 4; ++k) {
      int nx = x + dx[k], ny = y + dy[k];
      if (!isvalid(nx, ny)) continue;

      double tentative_g = g[x][y] + 1.0; // uniform cost for each step
      if (tentative_g < g[nx][ny]) {
        g[nx][ny] = tentative_g;
        parent[nx][ny] = {x, y};
        double f = tentative_g + heuristic(nx, ny, gx, gy);
        pq.push({f, {nx, ny}});
      }
    }
  }

  // no path
  return {};
}
