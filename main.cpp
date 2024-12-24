#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <random>
#include <sstream>
#include <string_view>
#include <variant>
#include <vector>

#include "default_field.h"
#include "fast_fixed.h"
#include "fixed.h"
#include "simulator.h"
#include "state.h"
#include "utils.h"

#pragma optimize("O3")

using namespace std;

vector<string> get_field_from_file(const string& filename) {
  ifstream file(filename);
  if (!file.is_open()) {
    throw runtime_error("Failed to open file: " + filename);
  }

  vector<string> result;
  string line;
  while (getline(file, line)) {
    result.push_back(line);
  }
  return result;
}

#ifndef TYPES
#define TYPES                                                      \
  FLOAT, FIXED(31, 17), FAST_FIXED(25, 11), FIXED(32, 16), DOUBLE, \
      FAST_FIXED(32, 16)
#endif

#ifndef SIZES
#define SIZES S(36, 84), S(100, 100)
#endif

template <size_t N, size_t M>
struct SizeType {
  static constexpr size_t n = N;
  static constexpr size_t m = M;
};

struct Size {
  size_t n, m;
  constexpr Size(size_t n_, size_t m_) : n(n_), m(m_) {}
  constexpr Size() : n(0), m(0) {}
};

constexpr Size parse_size(const char* s) {
  s += 2;
  size_t n = 0;
  while (*s >= '0' && *s <= '9') {
    n = n * 10 + (*s - '0');
    s++;
  }
  s++;
  size_t m = 0;
  while (*s >= '0' && *s <= '9') {
    m = m * 10 + (*s - '0');
    s++;
  }
  return Size(n, m);
}

template <class... Sizes>
struct SizesList {
  static constexpr size_t size = sizeof...(Sizes);
  template <size_t I>
  using size_at = typename std::tuple_element<I, std::tuple<Sizes...>>::type;
};

template <class... Types>
struct TypesList {
  static constexpr size_t size = sizeof...(Types);
  template <size_t I>
  using type_at = typename std::tuple_element<I, std::tuple<Types...>>::type;
};

template <class L, size_t I = 0>
constexpr bool matches_size_impl(const Size& size) {
  if constexpr (I >= L::size) {
    return false;
  } else {
    return (L::template get<I>().n == size.n &&
            L::template get<I>().m == size.m) ||
           matches_size_impl<L, I + 1>(size);
  }
}

template <class L>
bool matches_size(const Size& size) {
  return matches_size_impl<L>(size);
}

template <class T>
struct NumericTraits {
  static T from_raw(int32_t x) { return T(x) / T(1 << 16); }
};

template <size_t N, size_t K>
struct NumericTraits<Fixed<N, K>> {
  static Fixed<N, K> from_raw(typename Fixed<N, K>::StorageType x) {
    return Fixed<N, K>::from_raw(x);
  }
};

template <typename P, typename V, typename VF,
          typename Size = SizeType<dynamic_size, dynamic_size>>
void run_simulation(size_t n = DEFAULT_N, size_t m = DEFAULT_M) {
  try {
    SimulationState<P, V, VF, Size::n, Size::m> state(INITIAL_FIELD);
    Simulator<P, V, VF, Size::n, Size::m> simulator(state);

    simulator.run();
  } catch (const std::exception& e) {
    throw std::runtime_error(e.what());
  }
}

template <class T>
std::string get_pretty_type_name() {
  if constexpr (std::is_same_v<T, float>) {
    return "float";
  } else if constexpr (std::is_same_v<T, double>) {
    return "double";
  } else if constexpr (is_fixed<T>::value) {
    return "Fixed<" + std::to_string(T::n) + "," + std::to_string(T::k) + ">";
  } else if constexpr (is_fast_fixed<T>::value) {
    return "FastFixed<" + std::to_string(T::n) + "," + std::to_string(T::k) +
           ">";
  } else {
    return "unknown";
  }
}

pair<size_t, size_t> parse_fixed_params(const string& type) {
  size_t start = type.find('(') + 1;
  size_t comma = type.find(',', start);
  size_t end = type.find(')', comma);

  size_t N = stoul(type.substr(start, comma - start));
  size_t K = stoul(type.substr(comma + 1, end - comma - 1));
  return {N, K};
}

template <typename T>
static bool matches_type(const string& type) {
  if constexpr (std::is_same_v<T, float>) {
    return type == "FLOAT";
  } else if constexpr (std::is_same_v<T, double>) {
    return type == "DOUBLE";
  } else if constexpr (is_fixed<T>::value) {
    if (!type.starts_with("FIXED(")) return false;
    auto [n, k] = parse_fixed_params(type);
    return n == T::n && k == T::k;
  } else if constexpr (is_fast_fixed<T>::value) {
    if (!type.starts_with("FAST_FIXED(")) return false;
    auto [n, k] = parse_fixed_params(type);
    return n == T::k && k == T::k;
  }
  return false;
}

template <class AllowedTypes, class AllowedSizes, class SelectedTypes>
struct TypeSelector {
  template <class... Selected>
  static bool try_combinations(const string& p_type, const string& v_type,
                               const string& v_flow_type, size_t n, size_t m) {
    return try_all_p_types<0>(p_type, v_type, v_flow_type, n, m);
  }

 private:
  template <size_t I>
  static bool try_all_p_types(const string& p_type, const string& v_type,
                              const string& v_flow_type, size_t n, size_t m) {
    if constexpr (I >= AllowedTypes::size) {
      return false;
    } else {
      using P = typename AllowedTypes::template type_at<I>;
      return try_with_p_type<P>(p_type, v_type, v_flow_type, n, m) ||
             try_all_p_types<I + 1>(p_type, v_type, v_flow_type, n, m);
    }
  }

  template <class P>
  static bool try_with_p_type(const string& p_type, const string& v_type,
                              const string& v_flow_type, size_t n, size_t m) {
    if (!matches_type<P>(p_type)) return false;
    return try_all_v_types<P, 0>(p_type, v_type, v_flow_type, n, m);
  }

  template <class P, size_t I>
  static bool try_all_v_types(const string& p_type, const string& v_type,
                              const string& v_flow_type, size_t n, size_t m) {
    if constexpr (I >= AllowedTypes::size) {
      return false;
    } else {
      using V = class AllowedTypes::template type_at<I>;
      return try_with_v_type<P, V>(p_type, v_type, v_flow_type, n, m) ||
             try_all_v_types<P, I + 1>(p_type, v_type, v_flow_type, n, m);
    }
  }

  template <class P, class V>
  static bool try_with_v_type(const string& p_type, const string& v_type,
                              const string& v_flow_type, size_t n, size_t m) {
    if (!matches_type<V>(v_type)) return false;
    return try_all_vf_types<P, V, 0>(p_type, v_type, v_flow_type, n, m);
  }

  template <class P, class V, size_t I>
  static bool try_all_vf_types(const string& p_type, const string& v_type,
                               const string& v_flow_type, size_t n, size_t m) {
    if constexpr (I >= AllowedTypes::size) {
      return false;
    } else {
      using VF = class AllowedTypes::template type_at<I>;
      return try_with_vf_type<P, V, VF>(p_type, v_type, v_flow_type, n, m) ||
             try_all_vf_types<P, V, I + 1>(p_type, v_type, v_flow_type, n, m);
    }
  }

  template <class P, class V, class VF>
  static bool try_with_vf_type(const string& p_type, const string& v_type,
                               const string& v_flow_type, size_t n, size_t m) {
    if (!matches_type<VF>(v_flow_type)) return false;

    if constexpr (AllowedSizes::size == 0) {
      run_simulation<P, V, VF>(n, m);
      return true;
    } else {
      return try_all_sizes<P, V, VF, 0>(p_type, v_type, v_flow_type, n, m);
    }
  }
  template <class P, class V, class VF, size_t I>
  static bool try_all_sizes(const string& p_type, const string& v_type,
                            const string& v_flow_type, size_t n, size_t m) {
    if constexpr (I >= AllowedSizes::size) {
      return false;
    } else {
      using CurrentSize = typename AllowedSizes::template size_at<I>;
      return try_with_size<P, V, VF, CurrentSize>(p_type, v_type, v_flow_type,
                                                  n, m) ||
             try_all_sizes<P, V, VF, I + 1>(p_type, v_type, v_flow_type, n, m);
    }
  }

  template <class P, class V, class VF, class CurrentSize>
  static bool try_with_size(const string& p_type, const string& v_type,
                            const string& v_flow_type, size_t n, size_t m) {
    if (CurrentSize::n != n || CurrentSize::m != m) {
      return false;
    }

    run_simulation<P, V, VF, CurrentSize>(CurrentSize::n, CurrentSize::m);
    return true;
  }
};

template <typename... Types>
bool try_all_type_combinations(const string& p_type, const string& v_type,
                               const string& v_flow_type, size_t n, size_t m) {
#define S(N, M) SizeType<N, M>
  return TypeSelector<TypesList<Types...>, SizesList<SIZES>,
                      TypesList<>>::try_combinations(p_type, v_type,
                                                     v_flow_type, n, m);
}

void create_and_run_simulation(const string& p_type, const string& v_type,
                               const string& v_flow_type, size_t n, size_t m) {
  try {
#define FLOAT float
#define DOUBLE double
#define FIXED(N, K) Fixed<N, K>
#define FAST_FIXED(N, K) FastFixed<N, K>
    if (!try_all_type_combinations<TYPES>(p_type, v_type, v_flow_type, n, m)) {
      throw runtime_error("No matching types found");
    }

  } catch (const std::exception& e) {
    throw runtime_error(e.what());
  }
}
#define DEFAULT_P_TYPE "FIXED(32,16)"
#define DEFAULT_V_TYPE "DOUBLE"
#define DEFAULT_V_FLOW_TYPE "FLOAT"
#define DEFALT_SIZES "S(36,84)"

int main(int argc, char** argv) {
  string p_type = has_arg("--p-type", argc, argv)
                      ? parse_arg("--p-type", argc, argv)
                      : DEFAULT_P_TYPE;
  string v_type = has_arg("--v-type", argc, argv)
                      ? parse_arg("--v-type", argc, argv)
                      : DEFAULT_V_TYPE;
  string v_flow_type = has_arg("--v-flow-type", argc, argv)
                           ? parse_arg("--v-flow-type", argc, argv)
                           : DEFAULT_V_FLOW_TYPE;
  Size size = parse_size((has_arg("--size", argc, argv)
                              ? parse_arg("--size", argc, argv)
                              : DEFALT_SIZES)
                             .c_str());

  vector<string> field;

  if (has_arg("--field", argc, argv)) {
    field = get_field_from_file(parse_arg("--field", argc, argv));
    size.m = field[0].length();
    size.n = field.size();
  } else {
    field = INITIAL_FIELD;
  }

  try {
    create_and_run_simulation(p_type, v_type, v_flow_type, size.n, size.m);
  } catch (const std::exception& e) {
    cout << e.what() << "\n";
    return 1;
  }

  return 0;
}
