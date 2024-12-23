#ifndef __FIXED__
#define __FIXED__

#include <cmath>
#include <cstdint>
#include <iostream>
#include <type_traits>

#include "fast_fixed.h"

template <size_t N, size_t K>
struct FastFixed;

template <size_t N, size_t K>
struct Fixed {
  using StorageType = typename std::conditional_t<
      (N <= 8), int8_t,
      std::conditional_t<
          (N <= 16), int16_t,
          std::conditional_t<
              (N <= 32), int32_t,
              std::conditional_t<(N <= 64), int64_t, __int128>>>>;

  constexpr Fixed() : v(0) {}

  constexpr Fixed(int v) : v(static_cast<StorageType>(v) << K) {}

  constexpr Fixed(float f) : v(f * (StorageType(1) << K)) {}

  constexpr Fixed(double f) : v(f * (StorageType(1) << K)) {}

  static constexpr Fixed from_raw(StorageType x) {
    Fixed ret;
    ret.v = x;
    return ret;
  }

  StorageType v;

  static constexpr size_t n = N;

  static constexpr size_t k = K;

  auto operator<=>(const Fixed&) const = default;

  bool operator==(const Fixed&) const = default;

  explicit operator float() const { return v / float(StorageType(1) << K); }

  explicit operator double() const { return v / double(StorageType(1) << K); }

  friend Fixed operator/(Fixed a, int b) { return Fixed::from_raw(a.v / b); }

  friend Fixed operator*(Fixed a, int b) { return Fixed::from_raw(a.v * b); }

  friend Fixed operator*(int a, Fixed b) { return b * a; }

  template <size_t N2, size_t K2>
  explicit operator Fixed<N2, K2>() const {
    if constexpr (K2 >= K) {
      return Fixed<N2, K2>::from_raw(
          static_cast<typename Fixed<N2, K2>::StorageType>(v) << (K2 - K));
    } else {
      constexpr size_t shift = K - K2;
      if constexpr (shift >= N2) {
        auto temp = v >> (shift - N2 + 1);
        return Fixed<N2, K2>::from_raw(
            static_cast<typename Fixed<N2, K2>::StorageType>(temp) >> 1);
      } else {
        return Fixed<N2, K2>::from_raw(
            static_cast<typename Fixed<N2, K2>::StorageType>(v) >> shift);
      }
    }
  }

  template <size_t N2, size_t K2>
  explicit operator FastFixed<N2, K2>() const {
    if constexpr (K2 >= K) {
      return FastFixed<N2, K2>::from_raw(
          static_cast<typename FastFixed<N2, K2>::StorageType>(v) << (K2 - K));
    } else {
      constexpr size_t shift = K - K2;
      if constexpr (shift >= N2) {
        auto temp = v >> (shift - N2 + 1);
        return FastFixed<N2, K2>::from_raw(
            static_cast<typename FastFixed<N2, K2>::StorageType>(temp) >> 1);
      } else {
        return FastFixed<N2, K2>::from_raw(
            static_cast<typename FastFixed<N2, K2>::StorageType>(v) >> shift);
      }
    }
  }
};

template <size_t N, size_t K>
Fixed<N, K> operator+(Fixed<N, K> a, Fixed<N, K> b) {
  return Fixed<N, K>::from_raw(a.v + b.v);
}

template <size_t N, size_t K>
Fixed<N, K> operator-(Fixed<N, K> a, Fixed<N, K> b) {
  return Fixed<N, K>::from_raw(a.v - b.v);
}

template <size_t N, size_t K>
Fixed<N, K> operator*(Fixed<N, K> a, Fixed<N, K> b) {
  using ST = typename Fixed<N, K>::StorageType;
  if constexpr (N <= 32) {
    return Fixed<N, K>::from_raw((static_cast<int64_t>(a.v) * b.v) >> K);
  } else {
    ST high = (a.v >> K) * b.v;
    ST low = (a.v & ((ST(1) << K) - 1)) * b.v >> K;
    return Fixed<N, K>::from_raw(high + low);
  }
}

template <size_t N, size_t K>
Fixed<N, K> operator/(Fixed<N, K> a, Fixed<N, K> b) {
  using ST = typename Fixed<N, K>::StorageType;
  if constexpr (N <= 32) {
    return Fixed<N, K>::from_raw((static_cast<int64_t>(a.v) << K) / b.v);
  } else {
    return Fixed<N, K>::from_raw((a.v << K) / b.v);
  }
}

template <size_t N, size_t K>
Fixed<N, K>& operator+=(Fixed<N, K>& a, Fixed<N, K> b) {
  return a = a + b;
}

template <size_t N, size_t K>
Fixed<N, K>& operator-=(Fixed<N, K>& a, Fixed<N, K> b) {
  return a = a - b;
}

template <size_t N, size_t K>
Fixed<N, K>& operator*=(Fixed<N, K>& a, Fixed<N, K> b) {
  return a = a * b;
}

template <size_t N, size_t K>
Fixed<N, K>& operator/=(Fixed<N, K>& a, Fixed<N, K> b) {
  return a = a / b;
}

template <size_t N, size_t K>
Fixed<N, K> operator-(Fixed<N, K> x) {
  return Fixed<N, K>::from_raw(-x.v);
}

template <size_t N, size_t K>
Fixed<N, K> abs(Fixed<N, K> x) {
  Fixed<N, K> ret = x;
  if (ret.v < 0) ret.v = -ret.v;
  return ret;
}

template <size_t N, size_t K>
std::ostream& operator<<(std::ostream& out, Fixed<N, K> x) {
  return out << static_cast<double>(x);
}

template <size_t N, size_t K>
Fixed<N, K> operator+(float a, Fixed<N, K> b) {
  return Fixed<N, K>(a) + b;
}

template <size_t N, size_t K>
Fixed<N, K> operator+(Fixed<N, K> a, float b) {
  return a + Fixed<N, K>(b);
}

template <size_t N, size_t K>
Fixed<N, K> operator-(float a, Fixed<N, K> b) {
  return Fixed<N, K>(a) - b;
}

template <size_t N, size_t K>
Fixed<N, K> operator-(Fixed<N, K> a, float b) {
  return a - Fixed<N, K>(b);
}

template <size_t N, size_t K>
Fixed<N, K> operator*(float a, Fixed<N, K> b) {
  return Fixed<N, K>(a) * b;
}

template <size_t N, size_t K>
Fixed<N, K> operator*(Fixed<N, K> a, float b) {
  return a * Fixed<N, K>(b);
}

template <size_t N, size_t K>
Fixed<N, K> operator/(float a, Fixed<N, K> b) {
  return Fixed<N, K>(a) / b;
}

template <size_t N, size_t K>
Fixed<N, K> operator/(Fixed<N, K> a, float b) {
  return a / Fixed<N, K>(b);
}

template <size_t N, size_t K>
Fixed<N, K>& operator+=(float& a, Fixed<N, K> b) {
  a = static_cast<float>(Fixed<N, K>(a) + b);
  return a;
}

template <size_t N, size_t K>
float& operator-=(float& a, Fixed<N, K> b) {
  a = static_cast<float>(Fixed<N, K>(a) - b);
  return a;
}

template <size_t N, size_t K>
Fixed<N, K> operator+(double a, Fixed<N, K> b) {
  return Fixed<N, K>(a) + b;
}

template <size_t N, size_t K>
Fixed<N, K>& operator-=(Fixed<N, K>& a, float b) {
  return a = a - Fixed<N, K>(b);
}

template <size_t N, size_t K>
Fixed<N, K>& operator+=(Fixed<N, K>& a, float b) {
  return a = a + Fixed<N, K>(b);
}

template <size_t N, size_t K>
Fixed<N, K>& operator*=(Fixed<N, K>& a, float b) {
  return a = a * Fixed<N, K>(b);
}

template <size_t N, size_t K>
Fixed<N, K> operator+(const Fixed<N, K>& a, const FastFixed<N, K>& b) {
  return a + static_cast<Fixed<N, K>>(b);
}

template <size_t N, size_t K>
Fixed<N, K> operator+(const FastFixed<N, K>& a, const Fixed<N, K>& b) {
  return static_cast<Fixed<N, K>>(a) + b;
}

template <size_t N, size_t K>
Fixed<N, K> operator-(const Fixed<N, K>& a, const FastFixed<N, K>& b) {
  return a - static_cast<Fixed<N, K>>(b);
}

template <size_t N, size_t K>
Fixed<N, K> operator-(const FastFixed<N, K>& a, const Fixed<N, K>& b) {
  return static_cast<Fixed<N, K>>(a) - b;
}

template <size_t N, size_t K>
Fixed<N, K> operator*(const Fixed<N, K>& a, const FastFixed<N, K>& b) {
  return a * static_cast<Fixed<N, K>>(b);
}

template <size_t N, size_t K>
Fixed<N, K> operator*(const FastFixed<N, K>& a, const Fixed<N, K>& b) {
  return static_cast<Fixed<N, K>>(a) * b;
}

template <size_t N, size_t K>
Fixed<N, K> operator/(const Fixed<N, K>& a, const FastFixed<N, K>& b) {
  return a / static_cast<Fixed<N, K>>(b);
}

template <size_t N, size_t K>
Fixed<N, K> operator/(const FastFixed<N, K>& a, const Fixed<N, K>& b) {
  return static_cast<Fixed<N, K>>(a) / b;
}

template <size_t N, size_t K>
bool operator<(const Fixed<N, K>& a, const FastFixed<N, K>& b) {
  return a < static_cast<Fixed<N, K>>(b);
}

template <size_t N, size_t K>
bool operator<(const FastFixed<N, K>& a, const Fixed<N, K>& b) {
  return static_cast<Fixed<N, K>>(a) < b;
}

template <size_t N, size_t K>
bool operator<=(const Fixed<N, K>& a, const FastFixed<N, K>& b) {
  return a <= static_cast<Fixed<N, K>>(b);
}

template <size_t N, size_t K>
bool operator<=(const FastFixed<N, K>& a, const Fixed<N, K>& b) {
  return static_cast<Fixed<N, K>>(a) <= b;
}

template <size_t N, size_t K>
bool operator>(const Fixed<N, K>& a, const FastFixed<N, K>& b) {
  return a > static_cast<Fixed<N, K>>(b);
}

template <size_t N, size_t K>
bool operator>(const FastFixed<N, K>& a, const Fixed<N, K>& b) {
  return static_cast<Fixed<N, K>>(a) > b;
}

template <size_t N, size_t K>
bool operator>=(const Fixed<N, K>& a, const FastFixed<N, K>& b) {
  return a >= static_cast<Fixed<N, K>>(b);
}

template <size_t N, size_t K>
bool operator>=(const FastFixed<N, K>& a, const Fixed<N, K>& b) {
  return static_cast<Fixed<N, K>>(a) >= b;
}

template <size_t N, size_t K>
bool operator==(const Fixed<N, K>& a, const FastFixed<N, K>& b) {
  return a == static_cast<Fixed<N, K>>(b);
}

template <size_t N, size_t K>
bool operator==(const FastFixed<N, K>& a, const Fixed<N, K>& b) {
  return static_cast<Fixed<N, K>>(a) == b;
}

template <size_t N, size_t K>
Fixed<N, K>& operator+=(Fixed<N, K>& a, const FastFixed<N, K>& b) {
  return a += static_cast<Fixed<N, K>>(b);
}

template <size_t N, size_t K>
Fixed<N, K>& operator-=(Fixed<N, K>& a, const FastFixed<N, K>& b) {
  return a -= static_cast<Fixed<N, K>>(b);
}

template <size_t N, size_t K>
Fixed<N, K>& operator*=(Fixed<N, K>& a, const FastFixed<N, K>& b) {
  return a *= static_cast<Fixed<N, K>>(b);
}

template <size_t N, size_t K>
Fixed<N, K>& operator/=(Fixed<N, K>& a, const FastFixed<N, K>& b) {
  return a /= static_cast<Fixed<N, K>>(b);
}

#endif  // FIXED