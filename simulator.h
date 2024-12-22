#ifndef __SIMULATOR__
#define __SIMULATOR__

#include <algorithm>
#include <array>
#include <cassert>
#include <iostream>
#include <random>

#include "fast_fixed.h"
#include "fixed.h"
#include "state.h"

using namespace std;

template <class T>
struct is_fixed : std::false_type {};

template <size_t N, size_t K>
struct is_fixed<Fixed<N, K>> : std::true_type {};

template <class T>
struct is_fast_fixed : std::false_type {};

template <size_t N, size_t K>
struct is_fast_fixed<FastFixed<N, K>> : std::true_type {};

template <class PressureType, class VelocityType, class VFlowType,
        size_t N = DEFAULT_N, size_t M = DEFAULT_M,
        bool SizesAreStatic = true>
class Simulator {
public:
    Simulator(const SimulationState<PressureType, VelocityType, VFlowType, N, M>&
    state) {
        memcpy(field, state.field, sizeof(field));
        rho[' '] = PressureType(0.01);
        rho['.'] = PressureType(1000);

        for (size_t x = 0; x < N; ++x) {
            for (size_t y = 0; y < M; ++y) {
                if (field[x][y] == '#') {
                    continue;
                }
                for (auto [dx, dy] : deltas) {
                    dirs[x][y] += (field[x + dx][y + dy] != '#');
                }
            }
        }
    }

    void run() {
        PressureType g = PressureType(0.1);

        for (size_t i = 0; i < T; ++i) {
            PressureType total_delta_p = 0;

            // Apply external forces
            for (size_t x = 0; x < N; ++x) {
                for (size_t y = 0; y < M; ++y) {
                    if (field[x][y] == '#') continue;
                    if (field[x + 1][y] != '#') velocity.add(x, y, 1, 0, VelocityType(g));
                }
            }

            // Apply forces from p
            memcpy(old_p, p, sizeof(p));
            for (size_t x = 0; x < N; ++x) {
                for (size_t y = 0; y < M; ++y) {
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
                for (size_t x = 0; x < N; ++x) {
                    for (size_t y = 0; y < M; ++y) {
                        if (field[x][y] != '#' && last_use[x][y] != UT) {
                            auto [t, local_prop, _] = propagate_flow(x, y, VelocityType(1));
                            if (t > VelocityType(0)) prop = true;
                        }
                    }
                }
            } while (prop);

            // Recalculate p with kinetic energy
            for (size_t x = 0; x < N; ++x) {
                for (size_t y = 0; y < M; ++y) {
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
            for (size_t x = 0; x < N; ++x) {
                for (size_t y = 0; y < M; ++y) {
                    if (field[x][y] != '#' && last_use[x][y] != UT) {
                        if (random01() < move_prob(x, y)) {
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
                for (size_t x = 0; x < N; ++x) {
                    cout << field[x] << "\n";
                }
            }
        }
    }

private:
    template<class T>
    using MatrixType =
            std::conditional<SizesAreStatic, std::array<std::array<T, M + 1>, N>,
                    std::vector<std::vector<T>>>;

            static constexpr size_t T = 1'000'000;
    static constexpr std::array<pair<int, int>, 4> deltas{
            {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}};

    char field[N][M + 1];
    PressureType p[N][M]{}, old_p[N][M];
    int last_use[N][M]{};
    int UT = 0;
    mt19937 rnd{1337};
    PressureType rho[256];

    struct VectorField {
        array<VelocityType, 4> v[N][M]{};

        VelocityType& get(int x, int y, int dx, int dy) {
            //size_t i = ranges::find(deltas, pair(dx, dy)) - deltas.begin();
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
    int dirs[N][M]{};

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

    inline tuple<VelocityType, bool, pair<int, int>> propagate_flow(int x, int y,
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

    inline VelocityType random01() {
        return VelocityType(static_cast<double>(rnd() & ((1 << 16) - 1)) /
                            (1 << 16));
    }

    inline void propagate_stop(int x, int y, bool force = false) {
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

    inline bool propagate_move(int x, int y, bool is_first) {
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

            VelocityType p = random01() * sum;
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