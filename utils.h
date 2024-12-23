#ifndef __UTILS__
#define __UTILS__

#include <stdexcept>
#include <string>
#include <string_view>

using namespace std;

constexpr size_t DEFAULT_N = 36, DEFAULT_M = 84;
constexpr size_t dynamic_size = std::numeric_limits<size_t>::max();

template<typename T, size_t N = dynamic_size, size_t M = dynamic_size>
using MatrixType = std::conditional_t<N == dynamic_size || M == dynamic_size,
        std::vector<std::vector<T>>,
        std::array<std::array<T, M + (std::is_same_v<T, char> ? 1 : 0)>, N>
>;


bool has_arg(string_view arg_name, int argc, char** argv) {
  for (int i = 1; i < argc; ++i) {
    if (argv[i] == arg_name) {
      return true;
    }
  }
  return false;
}

string parse_arg(string_view arg_name, int argc, char** argv) {
  for (int i = 1; i < argc - 1; ++i) {
    if (argv[i] == arg_name) {
      return argv[i + 1];
    }
  }
  throw runtime_error("Argument " + string(arg_name) + " not found");
}

bool is_valid_type(const string& type) {
  if (type == "FLOAT" || type == "DOUBLE") return true;

  if (type.rfind("FIXED(", 0) == 0 || type.rfind("FAST_FIXED(", 0) == 0) {
    size_t pos = type.find(',');
    if (pos == string::npos) return false;

    size_t end_pos = type.find(')', pos);
    if (end_pos == string::npos) return false;

    return true;
  }

  return false;
}

#endif  // __UTILS__
