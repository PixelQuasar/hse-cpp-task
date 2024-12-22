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

using namespace std;

string get_field_from_file(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        throw runtime_error("Failed to open file: " + filename);
    }

    string content, line;
    while (getline(file, line)) {
        content += line + '\n';
    }
    return content;
}

#ifndef TYPES
#define TYPES                                                      \
  FLOAT, FIXED(31, 17), FAST_FIXED(25, 11), FIXED(32, 16), DOUBLE, \
      FAST_FIXED(32, 16)
#endif

#ifndef SIZES
#define SIZES S(36, 84), S(100, 100)
#endif

struct Size {
    size_t n, m;
    constexpr Size(size_t n_, size_t m_) : n(n_), m(m_) {}
    constexpr Size() : n(0), m(0) {}
};

Size parse_size(const string& size_str) {
    size_t start = 2;
    size_t comma = size_str.find(',', start);
    size_t end = size_str.find(')', comma);
    size_t n = stoul(size_str.substr(start, comma - start));
    size_t m = stoul(size_str.substr(comma + 1, end - comma - 1));

    return Size(n, m);
}

template <Size... Sizes>
struct SizesList {
    static constexpr size_t size = sizeof...(Sizes);
    template <size_t I>
    static constexpr Size get() {
        constexpr Size arr[] = {Sizes...};
        return arr[I];
    }
};

template <typename L, size_t I = 0>
constexpr bool matches_size_impl(const Size& size) {
    if constexpr (I >= L::size) {
        return false;
    } else {
        return (L::template get<I>().n == size.n &&
                L::template get<I>().m == size.m) ||
               matches_size_impl<L, I + 1>(size);
    }
}

template <typename L>
bool matches_size(const Size& size) {
    return matches_size_impl<L>(size);
}

template <typename T>
struct NumericTraits {
    static T from_raw(int32_t x) { return T(x) / T(1 << 16); }
};

template <size_t N, size_t K>
struct NumericTraits<Fixed<N, K>> {
    static Fixed<N, K> from_raw(typename Fixed<N, K>::StorageType x) {
        return Fixed<N, K>::from_raw(x);
    }
};

template <typename P, typename V, typename VF>
void run_simulation(size_t n, size_t m, const string& field_content) {
    SimulationState<P, V, VF, DEFAULT_N, DEFAULT_M> state(field_content);
    Simulator<P, V, VF, DEFAULT_N, DEFAULT_M> simulator(state);
    simulator.run();
}

template <typename P, typename V, typename VF, size_t static_N, size_t static_M>
void run_simulation(size_t n, size_t m, const string& field_content) {
    SimulationState<P, V, VF, static_N, static_M> state(field_content);
    Simulator<P, V, VF, static_N, static_M> simulator(state);
    simulator.run();
}

template <typename T>
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

template <typename... Types>
struct TypesList {
    static constexpr size_t size = sizeof...(Types);
    template <size_t I>
    using type_at = typename std::tuple_element<I, std::tuple<Types...>>::type;
};

template <typename AllowedTypes, typename SelectedTypes>
struct TypeSelector {
    template <typename... Selected>
    static bool try_combinations(const string& p_type, const string& v_type,
                                 const string& v_flow_type, const Size& size,
                                 const string& field_content) {
        return try_all_p_types<0>(p_type, v_type, v_flow_type, size, field_content);
    }

private:
    template <size_t I>
    static bool try_all_p_types(const string& p_type, const string& v_type,
                                const string& v_flow_type, const Size& size,
                                const string& field_content) {
        if constexpr (I >= AllowedTypes::size) {
            return false;
        } else {
            using P = typename AllowedTypes::template type_at<I>;
            return try_with_p_type<P>(p_type, v_type, v_flow_type, size,
                                      field_content) ||
                   try_all_p_types<I + 1>(p_type, v_type, v_flow_type, size,
                                          field_content);
        }
    }

    template <typename P>
    static bool try_with_p_type(const string& p_type, const string& v_type,
                                const string& v_flow_type, const Size& size,
                                const string& field_content) {
        if (!matches_type<P>(p_type)) return false;
        return try_all_v_types<P, 0>(p_type, v_type, v_flow_type, size,
                                     field_content);
    }

    template <typename P, size_t I>
    static bool try_all_v_types(const string& p_type, const string& v_type,
                                const string& v_flow_type, const Size& size,
                                const string& field_content) {
        if constexpr (I >= AllowedTypes::size) {
            return false;
        } else {
            using V = typename AllowedTypes::template type_at<I>;
            return try_with_v_type<P, V>(p_type, v_type, v_flow_type, size,
                                         field_content) ||
                   try_all_v_types<P, I + 1>(p_type, v_type, v_flow_type, size,
                                             field_content);
        }
    }

    template <typename P, typename V>
    static bool try_with_v_type(const string& p_type, const string& v_type,
                                const string& v_flow_type, const Size& size,
                                const string& field_content) {
        if (!matches_type<V>(v_type)) return false;
        return try_all_vf_types<P, V, 0>(p_type, v_type, v_flow_type, size,
                                         field_content);
    }

    template <typename P, typename V, size_t I>
    static bool try_all_vf_types(const string& p_type, const string& v_type,
                                 const string& v_flow_type, const Size& size,
                                 const string& field_content) {
        if constexpr (I >= AllowedTypes::size) {
            return false;
        } else {
            using VF = typename AllowedTypes::template type_at<I>;
            return try_with_vf_type<P, V, VF>(p_type, v_type, v_flow_type, size,
                                              field_content) ||
                   try_all_vf_types<P, V, I + 1>(p_type, v_type, v_flow_type, size,
                                                 field_content);
        }
    }

    template <typename P, typename V, typename VF>
    static bool try_with_vf_type(const string& p_type, const string& v_type,
                                 const string& v_flow_type, const Size& size,
                                 const string& field_content) {
        if (!matches_type<VF>(v_flow_type)) return false;

        run_simulation<P, V, VF>(size.n, size.m, field_content);
        return true;
    }
};

template <typename... Types>
bool try_all_type_combinations(const string& p_type, const string& v_type,
                               const string& v_flow_type, const Size& size,
                               const string& field_content) {
    return TypeSelector<TypesList<Types...>, TypesList<>>::try_combinations(
            p_type, v_type, v_flow_type, size, field_content);
}

void create_and_run_simulation(const string& p_type, const string& v_type,
                               const string& v_flow_type, const Size& size,
                               const string& field_content) {
    try {
#define S(N, M) Size(N, M)
        using SizesListType = SizesList<SIZES>;
#undef S

#define FLOAT float
#define DOUBLE double
#define FIXED(N, K) Fixed<N, K>
#define FAST_FIXED(N, K) FastFixed<N, K>

        if (!try_all_type_combinations<TYPES>(p_type, v_type, v_flow_type, size,
                                              field_content)) {
            throw std::runtime_error("Error: No matching type combination found");
        }
    } catch (const std::exception& e) {
        throw e;
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

    string field;

    if (has_arg("--field", argc, argv)) {
        field = get_field_from_file(parse_arg("--field", argc, argv));
        size.m = field.find('\n');
        size.n = count(field.begin(), field.end(), '\n');
    } else {
        field = INITIAL_FIELD;
    }

    try {
        create_and_run_simulation(p_type, v_type, v_flow_type, size, field);
    } catch (const std::exception& e) {
        cerr << "Failed to create simulation:\n" << e.what() << endl;
        return 1;
    }

    return 0;
}