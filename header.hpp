#pragma GCC optimize("-O3,inline,unroll-all-loops,fast-math")
#pragma GCC target("tune=native")
#pragma GCC target("avx2")

#include <bits/stdc++.h>
 #include <sys/time.h>
using namespace std;

// Macros

using i8 = int8_t;
using u8 = uint8_t;
using i16 = int16_t;
using u16 = uint16_t;
using i32 = int32_t;
using u32 = uint32_t;
using i64 = int64_t;
using u64 = uint64_t;
using f32 = float;
using f64 = double;

#define ALWAYS_INLINE __attribute__((always_inline))
#define FORCE_INLINE ALWAYS_INLINE inline

#define AS_STRING_IMPL(x) #x
#define AS_STRING(x) AS_STRING_IMPL(x)
#define runtime_assert(x)                                                                                                                                                                                                                      \
  do {                                                                                                                                                                                                                                         \
    if (!(x)) { throw runtime_error(__FILE__ ":" AS_STRING(__LINE__) " Assertion failed: " #x); }                                                                                                                                              \
  } while (0)
#define impossible()                                                                                                                                                                                                                           \
  do { throw runtime_error(__FILE__ ":" AS_STRING(__LINE__) " impossible"); } while (0)
#define all(x) begin(x), end(x)
#define pb push_back
#define eb emplace_back

// Types

template <class T>
using min_queue = priority_queue<T, vector<T>, greater<T>>;
template <class T>
using max_queue = priority_queue<T>;

// Printing

template <class T>
void print_collection(ostream& out, T const& x);

template <class T, size_t... I>
void print_tuple(ostream& out, T const& a, index_sequence<I...>);

namespace std {

template <class... A>
ostream& operator<<(ostream& out, tuple<A...> const& x) {
  print_tuple(out, x, index_sequence_for<A...>{});
  return out;
}
template <class... A>
ostream& operator<<(ostream& out, pair<A...> const& x) {
  print_tuple(out, x, index_sequence_for<A...>{});
  return out;
}

template <class A, size_t N>
ostream& operator<<(ostream& out, array<A, N> const& x) {
  print_collection(out, x);
  return out;
}
template <class A>
ostream& operator<<(ostream& out, vector<A> const& x) {
  print_collection(out, x);
  return out;
}
template <class A>
ostream& operator<<(ostream& out, deque<A> const& x) {
  print_collection(out, x);
  return out;
}
template <class A>
ostream& operator<<(ostream& out, multiset<A> const& x) {
  print_collection(out, x);
  return out;
}
template <class A, class B>
ostream& operator<<(ostream& out, multimap<A, B> const& x) {
  print_collection(out, x);
  return out;
}
template <class A>
ostream& operator<<(ostream& out, set<A> const& x) {
  print_collection(out, x);
  return out;
}
template <class A, class B>
ostream& operator<<(ostream& out, map<A, B> const& x) {
  print_collection(out, x);
  return out;
}
template <class A, class B>
ostream& operator<<(ostream& out, unordered_set<A> const& x) {
  print_collection(out, x);
  return out;
}

} // namespace std

template <class T, size_t... I>
void print_tuple(ostream& out, T const& a, index_sequence<I...>) {
  using swallow = int[];
  out << '(';
  (void)swallow{0, (void(out << (I == 0 ? "" : ", ") << get<I>(a)), 0)...};
  out << ')';
}

template <class T>
void print_collection(ostream& out, T const& x) {
  int f = 0;
  out << '[';
  for (auto const& i : x) {
    out << (f++ ? "," : "");
    out << i;
  }
  out << "]";
}

// Random

struct RNG {
  uint64_t s[2]{};

  explicit RNG(u64 seed) { reset(seed); }

  RNG() { reset(time(0)); }

  using result_type = u32;
  static constexpr u32 min() { return numeric_limits<u32>::min(); }
  static constexpr u32 max() { return numeric_limits<u32>::max(); }
  u32 operator()() { return randomInt32(); }

  static FORCE_INLINE uint64_t rotl(const uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }

  inline void reset(u64 seed = time(nullptr)) {
    struct splitmix64_state {
      u64 s;

      u64 splitmix64() {
        u64 result = (s += 0x9E3779B97f4A7C15);
        result = (result ^ (result >> 30)) * 0xBF58476D1CE4E5B9;
        result = (result ^ (result >> 27)) * 0x94D049BB133111EB;
        return result ^ (result >> 31);
      }
    };

    splitmix64_state sm{seed};

    s[0] = sm.splitmix64();
    s[1] = sm.splitmix64();
  }

  uint64_t next() {
    const uint64_t s0 = s[0];
    uint64_t s1 = s[1];
    const uint64_t result = rotl(s0 * 5, 7) * 9;

    s1 ^= s0;
    s[0] = rotl(s0, 24) ^ s1 ^ (s1 << 16); // a, b
    s[1] = rotl(s1, 37);                   // c

    return result;
  }

  inline u32 randomInt32() { return next(); }

  inline u64 randomInt64() { return next(); }

  inline u32 random32(u32 r) { return (((u64)randomInt32()) * r) >> 32; }

  inline u64 random64(u64 r) { return randomInt64() % r; }

  inline u32 randomRange32(u32 l, u32 r) { return l + random32(r - l + 1); }

  inline u64 randomRange64(u64 l, u64 r) { return l + random64(r - l + 1); }

  inline double randomDouble() { return (double)randomInt32() / 4294967296.0; }

  inline float randomFloat() { return (float)randomInt32() / 4294967296.0; }

  inline double randomRangeDouble(double l, double r) { return l + randomDouble() * (r - l); }

  template <class T>
  void shuffle(vector<T>& v) {
    i32 sz = v.size();
    for (i32 i = sz; i > 1; i--) {
      i32 p = (i32)random32(i);
      swap(v[i - 1], v[p]);
    }
  }

  template <class T>
  void shuffle(T* fr, T* to) {
    i32 sz = distance(fr, to);
    for (int i = sz; i > 1; i--) {
      int p = (i32)random32(i);
      swap(fr[i - 1], fr[p]);
    }
  }

  template <class T>
  inline int sample_index(vector<T> const& v) {
    return random32(v.size());
  }

  template <class T>
  inline T sample(vector<T> const& v) {
    return v[sample_index(v)];
  }
} rng;

// Timer

struct timer {
  chrono::high_resolution_clock::time_point t_begin;

  timer() : t_begin(chrono::high_resolution_clock::now()) {}

  void reset() { t_begin = chrono::high_resolution_clock::now(); }

  [[nodiscard]] double elapsed() const { return chrono::duration<double>(chrono::high_resolution_clock::now() - t_begin).count(); }
};

// Util

template <class T>
T& smin(T& x, T const& y) {
  x = min(x, y);
  return x;
}

template <class T>
T& smax(T& x, T const& y) {
  x = max(x, y);
  return x;
}

template <typename T>
int sgn(T val) {
  if (val < 0) return -1;
  if (val > 0) return 1;
  return 0;
}

static inline string int_to_string(int val, int digits = 0) {
  string s = to_string(val);
  reverse(all(s));
  while ((int)s.size() < digits) s.pb('0');
  reverse(all(s));
  return s;
}

// Debug

static inline void debug_impl_seq() { cerr << "}"; }

template <class T, class... V>
void debug_impl_seq(T const& t, V const&... v) {
  cerr << t;
  if (sizeof...(v)) { cerr << ", "; }
  debug_impl_seq(v...);
}

#define debug(x...)                                                                                                                                                                                                                            \
  do {                                                                                                                                                                                                                                         \
    cerr << __FILE__ ":" AS_STRING(__LINE__) "  {" << #x << "} = {";                                                                                                                                                                           \
    debug_impl_seq(x);                                                                                                                                                                                                                         \
    cerr << endl << flush;                                                                                                                                                                                                                     \
  } while (0)

#define nl '\n'
using ll = long long;

const ll infll = (1LL << 62) - 1;
const int inf = (1 << 30) - 1;

inline bool has_bit_set(const i32 x, const i32 id) { return (x & (1 << id)) != 0; }
inline void set_bit(i32& x, const i32 id) { x |= (1 << id); }
inline void unset_bit(i32 &x, const i32 id){ x &= ~(1 << id); }

template <typename T1, typename T2>
istream& operator>>(istream& is, pair<T1, T2>& p) {
  is >> p.first >> p.second;
  return is;
}

template <typename T>
istream& operator>>(istream& is, vector<T>& v) {
  for (T& in : v) is >> in;
  return is;
}

template <typename T = ll>
vector<T> make_v(size_t a) {
  return vector<T>(a);
}

template <typename T, typename... Ts>
auto make_v(size_t a, Ts... ts) {
  return vector<decltype(make_v<T>(ts...))>(a, make_v<T>(ts...));
}

template <typename T, typename V>
typename enable_if<is_class<T>::value == 0>::type fill_v(T& t, const V& v) {
  t = v;
}

template <typename T, typename V>
typename enable_if<is_class<T>::value != 0>::type fill_v(T& t, const V& v) {
  for (auto& e : t) fill_v(e, v);
}

template <class T>
inline void remove_one(vector<T>& data, T x) {
  data.erase(find(data.begin(), data.end(), x));
}