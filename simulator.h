#ifndef __SIMULATOR__
#define __SIMULATOR__

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <iostream>
#include <random>
#include <thread>

#include "fast_fixed.h"
#include "fixed.h"
#include "state.h"
#include "utils.h"

using namespace std;

template <class T>
struct is_fixed : std::false_type {};

template <size_t N, size_t K>
struct is_fixed<Fixed<N, K>> : std::true_type {};

template <class T>
struct is_fast_fixed : std::false_type {};

template <size_t N, size_t K>
struct is_fast_fixed<FastFixed<N, K>> : std::true_type {};

template <typename T>
class SafeQueue {
 public:
  void push(const T& value) {
    if (queue.size() >= limit) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex);
    queue.push(value);
    cond.notify_one();
  }

  bool pop(T& value) {
    std::unique_lock<std::mutex> lock(mutex);
    cond.wait(lock, [this] { return !queue.empty() || stop_value; });

    if (stop_value && queue.empty()) {
      return false;
    }

    value = queue.front();
    queue.pop();
    return true;
  }

  void stop() {
    std::lock_guard<std::mutex> lock(mutex);
    stop_value = true;
    cond.notify_all();
  }

 private:
  std::queue<T> queue;
  std::mutex mutex;
  std::condition_variable cond;
  size_t limit = 1024;
  bool stop_value = false;
};

template <class PressureType, class VelocityType, class VFlowType,
          size_t N = DEFAULT_N, size_t M = DEFAULT_M,
          bool SizesAreStatic = true>
class Simulator {
 public:
  Simulator(const SimulationState<PressureType, VelocityType, VFlowType, N, M>&
                state) {
    field = state.field;
    rho[' '] = PressureType(0.01);
    rho['.'] = PressureType(1000);
    if constexpr (N == dynamic_size || M == dynamic_size) {
      runtime_n = field.size();
      runtime_m = field[0].size() - 1;
      init_state(state);
    } else {
      runtime_n = N;
      runtime_m = M;
    }

    for (size_t x = 0; x < get_n(); ++x) {
      for (size_t y = 0; y < get_m(); ++y) {
        if (field[x][y] == '#') {
          continue;
        }
        for (auto [dx, dy] : deltas) {
          dirs[x][y] += (field[x + dx][y + dy] != '#');
        }
      }
    }
  }

  VelocityType random01() {
    return VelocityType(static_cast<double>(rnd() & ((1 << 16) - 1)) /
                        (1 << 16));
  }

  void init_state(const SimulationState<PressureType, VelocityType, VFlowType,
                                        N, M>& state) {
    field = state.field;
    p = state.p;
    old_p = state.old_p;
    last_use = state.last_use;
    UT = state.UT;
    dirs = state.dirs;
    velocity.resize(runtime_n, runtime_m);
    velocity_flow.resize(runtime_n, runtime_m);
  }

  void run() {
    const bool active = true;
    std::thread random_gen([this]() {
      while (active) {
        VelocityType val = VelocityType(
            static_cast<double>(rnd() & ((1 << 16) - 1)) / (1 << 16));

        this->safeQueue.push(val);
      }
    });

    // time stamps
    auto start = chrono::high_resolution_clock::now();

    PressureType g = PressureType(0.1);

    for (size_t i = 0; i < T; ++i) {
      PressureType total_delta_p = 0;

      // Apply external forces
      for (size_t x = 0; x < get_n(); ++x) {
        for (size_t y = 0; y < get_m(); ++y) {
          // << "3" << endl;
          if (field[x][y] == '#') continue;
          if (field[x + 1][y] != '#') velocity.add(x, y, 1, 0, VelocityType(g));
        }
      }

      // Apply forces from p
      old_p = p;
      for (size_t x = 0; x < get_n(); ++x) {
        for (size_t y = 0; y < get_m(); ++y) {
          if (field[x][y] == '#') continue;
          for (auto [dx, dy] : deltas) {
            int nx = x + dx, ny = y + dy;
            if (field[nx][ny] != '#' && old_p[nx][ny] < old_p[x][y]) {
              auto delta_p = old_p[x][y] - old_p[nx][ny];
              auto force = to_pressure(delta_p);
              auto& contr = velocity.get(nx, ny, -dx, -dy);
              if (to_pressure(contr) * rho[(int)field[nx][ny]] >= force) {
                contr -= to_velocity(force / rho[(int)field[nx][ny]]);
                continue;
              }
              force -= to_pressure(contr) * rho[(int)field[nx][ny]];
              contr = 0;
              velocity.add(x, y, dx, dy,
                           to_velocity(force / rho[(int)field[x][y]]));
              p[x][y] -= force / dirs[x][y];
              total_delta_p -= force / dirs[x][y];
            }
          }
        }
      }

      // Make flow from velocities
      velocity_flow = {};
      bool prop = false;
      do {
        UT += 2;
        prop = false;
        for (size_t x = 0; x < get_n(); ++x) {
          for (size_t y = 0; y < get_m(); ++y) {
            if (field[x][y] != '#' && last_use[x][y] != UT) {
              auto [t, local_prop, _] = propagate_flow(x, y, VelocityType(1));
              if (t > VelocityType(0)) prop = true;
            }
          }
        }
      } while (prop);

      // Recalculate p with kinetic energy
      for (size_t x = 0; x < get_n(); ++x) {
        for (size_t y = 0; y < get_m(); ++y) {
          if (field[x][y] == '#') {
            continue;
          }
          for (auto [dx, dy] : deltas) {
            auto old_v = velocity.get(x, y, dx, dy);
            auto new_v = velocity_flow.get(x, y, dx, dy);
            if (old_v > VelocityType(0)) {
              assert(new_v <= old_v);

              velocity.get(x, y, dx, dy) = new_v;
              auto force = to_pressure(old_v - new_v) * rho[(int)field[x][y]];
              if (field[x][y] == '.') force *= PressureType(0.8);
              if (field[x + dx][y + dy] == '#') {
                p[x][y] += force / dirs[x][y];
                total_delta_p += force / dirs[x][y];
              } else {
                p[x + dx][y + dy] += force / dirs[x + dx][y + dy];
                total_delta_p += force / dirs[x + dx][y + dy];
              }
            }
          }
        }
      }

      UT += 2;
      prop = false;
      for (size_t x = 0; x < get_n(); ++x) {
        for (size_t y = 0; y < get_m(); ++y) {
          if (field[x][y] != '#' && last_use[x][y] != UT) {
            VelocityType value;
            safeQueue.pop(value);
            if (value < move_prob(x, y)) {
              prop = true;
              propagate_move(x, y, true);
            } else {
              propagate_stop(x, y, true);
            }
          }
        }
      }

      if (prop) {
        cout << "Tick " << i << ":\n";
        for (size_t x = 0; x < get_n(); ++x) {
          field[x][get_m()] = '\0';
          cout << field[x].data() << "\n";
        }
      }
    }

    auto end = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(end - start);

    cout << "Time taken by run: " << duration.count() << " milliseconds"
         << endl;

    random_gen.join();
  }

 private:
  static constexpr size_t T = 200;

  SafeQueue<VelocityType> safeQueue;
  std::atomic<bool> done = std::atomic<bool>(false);

  mt19937 rnd{1337};

  static constexpr std::array<pair<int, int>, 4> deltas{
      {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}};

  size_t runtime_n, runtime_m;

  MatrixType<char, N, M + 1> field{};

  MatrixType<PressureType, N, M> p{};

  MatrixType<PressureType, N, M> old_p{};

  MatrixType<int, N, M> last_use{};

  int UT = 0;

  PressureType rho[256];

  struct VectorField {
    MatrixType<array<VelocityType, 4>, N, M> v{};

    VelocityType& get(int x, int y, int dx, int dy) {
      // size_t i = ranges::find(deltas, pair(dx, dy)) - deltas.begin();
      size_t i;
      if (dx == -1 && dy == 0) {
        i = 0;
      } else if (dx == 1 && dy == 0) {
        i = 1;
      } else if (dx == 0 && dy == -1) {
        i = 2;
      } else if (dx == 0 && dy == 1) {
        i = 3;
      } else {
        assert(false);
      }
      return v[x][y][i];
    }

    VelocityType& add(int x, int y, int dx, int dy, VelocityType dv) {
      return get(x, y, dx, dy) += dv;
    }
  };

  VectorField velocity{}, velocity_flow{};
  MatrixType<int, N, M> dirs{};

  struct ParticleParams {
    char type;
    PressureType cur_p;
    array<VelocityType, 4> v;

    void swap_with(Simulator* simulator, int x, int y) {
      swap(simulator->field[x][y], type);
      swap(simulator->p[x][y], cur_p);
      swap(simulator->velocity.v[x][y], v);
    }
  };

  template <typename T>
  inline PressureType to_pressure(T value) {
    if constexpr (std::is_same_v<T, PressureType>) {
      return value;
    } else {
      return PressureType(value);
    }
  }

  template <typename T>
  inline VelocityType to_velocity(T value) {
    if constexpr (std::is_same_v<T, VelocityType>) {
      return value;
    } else {
      return VelocityType(value);
    }
  }

  template <typename T>
  inline VFlowType to_flow(T value) {
    if constexpr (std::is_same_v<T, VFlowType>) {
      return value;
    } else {
      return VFlowType(value);
    }
  }

  size_t get_n() const {
    if constexpr (N == dynamic_size) {
      return runtime_n;
    } else {
      return N;
    }
  }

  size_t get_m() const {
    if constexpr (M == dynamic_size) {
      return runtime_m;
    } else {
      return M;
    }
  }

  tuple<VelocityType, bool, pair<int, int>> propagate_flow(int x, int y,
                                                           VelocityType lim) {
    last_use[x][y] = UT - 1;
    VelocityType ret = 0;
    for (auto [dx, dy] : deltas) {
      int nx = x + dx, ny = y + dy;
      if (field[nx][ny] != '#' && last_use[nx][ny] < UT) {
        auto cap = velocity.get(x, y, dx, dy);
        auto flow = velocity_flow.get(x, y, dx, dy);
        if (flow == cap) continue;

        auto vp = min(lim, cap - flow);
        if (last_use[nx][ny] == UT - 1) {
          velocity_flow.add(x, y, dx, dy, vp);
          last_use[x][y] = UT;
          return {vp, true, {nx, ny}};
        }
        auto [t, prop, end] = propagate_flow(nx, ny, vp);
        ret += t;
        if (prop) {
          velocity_flow.add(x, y, dx, dy, t);
          last_use[x][y] = UT;
          return {t, prop && end != pair(x, y), end};
        }
      }
    }
    last_use[x][y] = UT;
    return {ret, false, {0, 0}};
  }

  void propagate_stop(int x, int y, bool force = false) {
    if (!force) {
      bool stop = true;
      for (auto [dx, dy] : deltas) {
        int nx = x + dx, ny = y + dy;
        if (field[nx][ny] != '#' && last_use[nx][ny] < UT - 1 &&
            velocity.get(x, y, dx, dy) > VelocityType(0)) {
          stop = false;
          break;
        }
      }
      if (!stop) return;
    }
    last_use[x][y] = UT;
    for (auto [dx, dy] : deltas) {
      int nx = x + dx, ny = y + dy;
      if (field[nx][ny] == '#' || last_use[nx][ny] == UT ||
          velocity.get(x, y, dx, dy) > VelocityType(0))
        continue;
      propagate_stop(nx, ny);
    }
  }

  inline VelocityType move_prob(int x, int y) {
    VelocityType sum = 0;
    for (auto [dx, dy] : deltas) {
      int nx = x + dx, ny = y + dy;
      if (field[nx][ny] == '#' || last_use[nx][ny] == UT) continue;
      auto v = velocity.get(x, y, dx, dy);
      if (v < VelocityType(0)) continue;
      sum += v;
    }
    return sum;
  }

  bool propagate_move(int x, int y, bool is_first) {
    last_use[x][y] = UT - is_first;
    bool ret = false;
    int nx = -1, ny = -1;
    do {
      array<VelocityType, 4> tres;
      VelocityType sum = 0;
      for (size_t i = 0; i < deltas.size(); ++i) {
        auto [dx, dy] = deltas[i];
        int nx = x + dx, ny = y + dy;
        if (field[nx][ny] == '#' || last_use[nx][ny] == UT) {
          tres[i] = sum;
          continue;
        }
        auto v = velocity.get(x, y, dx, dy);
        if (v < VelocityType(0)) {
          tres[i] = sum;
          continue;
        }
        sum += v;
        tres[i] = sum;
      }

      if (sum == VelocityType(0)) break;

      VelocityType value;
      safeQueue.pop(value);

      VelocityType p = value * sum;
      size_t d = ranges::upper_bound(tres, p) - tres.begin();

      auto [dx, dy] = deltas[d];
      nx = x + dx;
      ny = y + dy;
      assert(velocity.get(x, y, dx, dy) > VelocityType(0) &&
             field[nx][ny] != '#' && last_use[nx][ny] < UT);

      ret = (last_use[nx][ny] == UT - 1 || propagate_move(nx, ny, false));
    } while (!ret);

    last_use[x][y] = UT;
    for (auto [dx, dy] : deltas) {
      int nx = x + dx, ny = y + dy;
      if (field[nx][ny] != '#' && last_use[nx][ny] < UT - 1 &&
          velocity.get(x, y, dx, dy) < VelocityType(0)) {
        propagate_stop(nx, ny);
      }
    }
    if (ret && !is_first) {
      ParticleParams pp{};
      pp.swap_with(this, x, y);
      pp.swap_with(this, nx, ny);
      pp.swap_with(this, x, y);
    }
    return ret;
  }
};

#endif  // __SIMULATOR__