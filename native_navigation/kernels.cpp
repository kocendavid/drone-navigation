// Native search kernels. No ROS publishers or flight-control authority.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <tuple>
#include <vector>

#ifdef _WIN32
#define BK_API extern "C" __declspec(dllexport)
#else
#define BK_API extern "C"
#endif

BK_API int bk_abi_version() { return 1; }

BK_API const char *bk_source_hash() { return NAV_SOURCE_HASH; }

// Returns path length, -1 for no path, -2 for invalid input, -3 on allocation error.
BK_API int bk_astar(const uint8_t *mask, const double *weight, int h, int w,
                    int start, int goal, int *output) {
  try {
    if (!mask || !output || h <= 0 || w <= 0 ||
        int64_t(h) * w > std::numeric_limits<int>::max())
      return -2;
    const int count = h * w;
    if (h <= 0 || w <= 0 || start < 0 || goal < 0 || start >= count ||
        goal >= count || !mask[start] || !mask[goal])
      return -2;
    for (int i = 0; weight && i < count; ++i)
      if (mask[i] && (!std::isfinite(weight[i]) || weight[i] < 1.0))
        return -2;
    const auto heuristic = [w, goal](int cell) {
      return std::hypot(double(cell / w - goal / w),
                        double(cell % w - goal % w));
    };
    using Item = std::tuple<double, double, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
    std::vector<double> costs(count, std::numeric_limits<double>::infinity());
    std::vector<int> parent(count, -1);
    costs[start] = 0;
    queue.emplace(heuristic(start), 0.0, start);
    while (!queue.empty()) {
      const auto [f, g, cell] = queue.top();
      (void)f;
      queue.pop();
      if (g != costs[cell])
        continue;
      if (cell == goal) {
        int length = 0;
        for (int current = cell; current >= 0; current = parent[current])
          output[length++] = current;
        std::reverse(output, output + length);
        return length;
      }
      const int y = cell / w, x = cell % w;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if ((!dy && !dx) || y + dy < 0 || y + dy >= h || x + dx < 0 || x + dx >= w)
            continue;
          const int next = (y + dy) * w + x + dx;
          if (!mask[next] || (dy && dx && (!mask[(y + dy) * w + x] || !mask[y * w + x + dx])))
            continue;
          const double step = std::hypot(double(dy), double(dx));
          const double cost = g + step * (weight ? (weight[cell] + weight[next]) / 2 : 1.0);
          if (cost < costs[next]) {
            costs[next] = cost;
            parent[next] = cell;
            queue.emplace(cost + heuristic(next), cost, next);
          }
        }
      }
    }
    return -1;
  } catch (...) {
    return -3;
  }
}

// Output: complete, component cells, distinct unknown boundary cells, edge, valid start.
BK_API int bk_completion(const uint8_t *grid, int h, int w, int start, int *out) {
  try {
    if (!grid || !out || h <= 0 || w <= 0 ||
        int64_t(h) * w > std::numeric_limits<int>::max())
      return -1;
    std::fill(out, out + 5, 0);
    const auto known = [grid](int cell) { return grid[cell] == 1 || grid[cell] == 3; };
    if (h <= 0 || w <= 0 || start < 0 || start >= h * w || !known(start))
      return 0;
    out[4] = 1;
    std::vector<uint8_t> visited(h * w, 0), boundary(h * w, 0);
    std::vector<int> queue{start};
    visited[start] = 1;
    bool observed = false;
    constexpr int dy[] = {0, 0, 1, -1}, dx[] = {1, -1, 0, 0};
    for (size_t position = 0; position < queue.size(); ++position) {
      const int cell = queue[position], y = cell / w, x = cell % w;
      observed = observed || grid[cell] == 1;
      out[3] |= y == 0 || y == h - 1 || x == 0 || x == w - 1;
      for (int i = 0; i < 4; ++i) {
        if (y + dy[i] < 0 || y + dy[i] >= h || x + dx[i] < 0 || x + dx[i] >= w)
          continue;
        const int next = (y + dy[i]) * w + x + dx[i];
        if (grid[next] == 0) {
          if (!boundary[next]) {
            boundary[next] = 1;
            ++out[2];
          }
        } else if (known(next) && !visited[next]) {
          visited[next] = 1;
          queue.push_back(next);
        }
      }
    }
    out[1] = static_cast<int>(queue.size());
    out[0] = observed && out[2] == 0 && out[3] == 0;
    return 0;
  } catch (...) {
    return -1;
  }
}
