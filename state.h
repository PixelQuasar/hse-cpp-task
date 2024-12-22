#ifndef __STATE__
#define __STATE__

#include <cstdint>
#include <random>
#include <sstream>
#include <string>

using namespace std;

constexpr size_t DEFAULT_N = 36, DEFAULT_M = 84;

template <typename NumericType, size_t N, size_t M>
struct VectorField {
    static constexpr std::array<pair<int, int>, 4> deltas{
            {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}};
    array<NumericType, 4> v[N][M];

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
    PressureType p[N][M]{};
    PressureType old_p[N][M]{};

    char field[N][M + 1];
    int last_use[N][M]{};
    int UT{0};
    mt19937 rnd{1337};
    int dirs[N][M]{};

    VectorField<VelocityType, N, M> velocity{};
    VectorField<VFlowType, N, M> velocity_flow{};

    SimulationState(const string& field_content) {
        stringstream ss(field_content);
        string line;
        size_t row = 0;
        while (getline(ss, line) && row < N) {
            size_t col = 0;
            while (col < M && col < line.length()) {
                field[row][col] = line[col];
                col++;
            }
            field[row][col] = '\0';
            row++;
        }
    }
};

#endif