#ifndef __STATE__
#define __STATE__

#include <cstdint>
#include <random>
#include <sstream>
#include <string>

#include "utils.h"

using namespace std;

template <typename NumericType, size_t N, size_t M>
struct VectorField {
  static constexpr std::array<pair<int, int>, 4> deltas{
      {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}};
  MatrixType<array<NumericType, 4>, N, M> v{};

  NumericType& add(int x, int y, int dx, int dy, NumericType dv) {
    return get(x, y, dx, dy) += dv;
  }

  NumericType& get(int x, int y, int dx, int dy) {
    size_t i = ranges::find(deltas, pair(dx, dy)) - deltas.begin();
    assert(i < deltas.size());
    return v[x][y][i];
  }
};

template <typename PressureType, typename VelocityType, typename VFlowType,
          size_t N = DEFAULT_N, size_t M = DEFAULT_M>
struct SimulationState {
  PressureType rho[256];
  MatrixType<PressureType, N, M> p{};
  MatrixType<PressureType, N, M> old_p{};

  MatrixType<char, N, M + 1> field{};
  MatrixType<int, N, M> last_use{};
  int UT{0};
  mt19937 rnd{1337};
  MatrixType<int, N, M> dirs{};

  VectorField<VelocityType, N, M> velocity{};
  VectorField<VFlowType, N, M> velocity_flow{};

  SimulationState(const std::vector<std::string>& initial_field = {}) {
    if constexpr (N == dynamic_size || M == dynamic_size) {
      if (!initial_field.empty()) {
        size_t n = initial_field.size();
        size_t m = initial_field[0].size();

        field.resize(n);
        for (auto& row : field) {
          row.resize(m + 1);
        }
        p.resize(n);
        for (auto& row : p) {
          row.resize(m);
        }
        old_p.resize(n);
        for (auto& row : old_p) {
          row.resize(m);
        }
        last_use.resize(n);
        for (auto& row : last_use) {
          row.resize(m);
        }
        dirs.resize(n);
        for (auto& row : dirs) {
          row.resize(m);
        }
        for (size_t i = 0; i < n; ++i) {
          for (size_t j = 0; j < m; ++j) {
            field[i][j] = initial_field[i][j];
          }
        }
      }
    } else {
      if (!initial_field.empty()) {
        if (initial_field.size() != N || initial_field[0].size() != M) {
          throw std::runtime_error("Field size mismatch");
        }
        for (size_t i = 0; i < N; ++i) {
          for (size_t j = 0; j < M; ++j) {
            field[i][j] = initial_field[i][j];
          }
        }
      }
    }
  }
};

#endif