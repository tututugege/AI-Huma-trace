#pragma once

#if __has_include("config.h")
#include "config.h"
#endif

#include <array>
#include <cstdint>
#include <string>
#include <type_traits>
#include <utility>

#ifndef AXI_KIT_HAS_PARENT_CONFIG
#if __has_include("config.h")
#define AXI_KIT_HAS_PARENT_CONFIG 1
#else
#define AXI_KIT_HAS_PARENT_CONFIG 0
#endif
#endif

#ifndef AXI_KIT_USE_PARENT_WIRE_REG
#ifdef SIMULATOR_HAS_PARENT_WIRE_REG
#define AXI_KIT_USE_PARENT_WIRE_REG 1
#else
#define AXI_KIT_USE_PARENT_WIRE_REG 0
#endif
#endif

namespace axi_compat {

template <typename T>
using remove_cvref_t = std::remove_cv_t<std::remove_reference_t<T>>;

template <typename T>
inline constexpr bool is_builtin_wire_v =
    std::is_integral_v<remove_cvref_t<T>> ||
    std::is_same_v<remove_cvref_t<T>, unsigned __int128> ||
    std::is_same_v<remove_cvref_t<T>, __int128>;

template <typename T, typename = void>
struct has_byte_array : std::false_type {};

template <typename T>
struct has_byte_array<
    T, std::void_t<decltype(std::declval<remove_cvref_t<T> &>().bytes[0])>>
    : std::true_type {};

template <typename T>
inline constexpr bool has_byte_array_v = has_byte_array<T>::value;

#if !AXI_KIT_USE_PARENT_WIRE_REG
template <int Bits>
struct wide_bits {
  static_assert(Bits > 0, "wire/reg bit width must be positive");

  static constexpr uint32_t kBits = Bits;
  static constexpr uint32_t kLaneCount = (Bits + 63) / 64;

  std::array<uint64_t, kLaneCount> lanes{};

  constexpr wide_bits() = default;

  template <typename T,
            typename = std::enable_if_t<is_builtin_wire_v<T>>>
  constexpr wide_bits(T value) {
    *this = value;
  }

  constexpr void clear() { lanes.fill(0); }

  constexpr void trim_unused_bits() {
    if constexpr ((Bits % 64) != 0) {
      lanes[kLaneCount - 1] &=
          (static_cast<uint64_t>(1u) << (Bits % 64)) - 1u;
    }
  }

  template <typename T,
            typename = std::enable_if_t<is_builtin_wire_v<T>>>
  constexpr wide_bits &operator=(T value) {
    clear();
    if constexpr (std::is_same_v<remove_cvref_t<T>, bool>) {
      lanes[0] = value ? 1u : 0u;
    } else if constexpr (sizeof(remove_cvref_t<T>) <= sizeof(uint64_t)) {
      lanes[0] = static_cast<uint64_t>(value);
    } else {
      const unsigned __int128 wide = static_cast<unsigned __int128>(value);
      lanes[0] = static_cast<uint64_t>(wide);
      if constexpr (kLaneCount > 1) {
        lanes[1] = static_cast<uint64_t>(wide >> 64u);
      }
    }
    trim_unused_bits();
    return *this;
  }

  constexpr wide_bits &operator|=(const wide_bits &other) {
    for (uint32_t lane = 0; lane < kLaneCount; ++lane) {
      lanes[lane] |= other.lanes[lane];
    }
    trim_unused_bits();
    return *this;
  }

  template <typename T,
            typename = std::enable_if_t<is_builtin_wire_v<T>>>
  constexpr wide_bits &operator|=(T value) {
    wide_bits other;
    other = value;
    return (*this |= other);
  }

  constexpr bool operator==(const wide_bits &other) const {
    return lanes == other.lanes;
  }

  template <typename T,
            typename = std::enable_if_t<is_builtin_wire_v<T>>>
  constexpr bool operator==(T value) const {
    wide_bits other;
    other = value;
    return *this == other;
  }
};

template <int Bits,
          typename T,
          typename = std::enable_if_t<is_builtin_wire_v<T>>>
constexpr bool operator==(T value, const wide_bits<Bits> &rhs) {
  return rhs == value;
}

template <int Bits>
constexpr wide_bits<Bits> operator|(wide_bits<Bits> lhs,
                                    const wide_bits<Bits> &rhs) {
  lhs |= rhs;
  return lhs;
}

template <int Bits,
          typename T,
          typename = std::enable_if_t<is_builtin_wire_v<T>>>
constexpr wide_bits<Bits> operator|(wide_bits<Bits> lhs, T value) {
  lhs |= value;
  return lhs;
}

template <typename T>
struct is_wide_bits : std::false_type {};

template <int Bits>
struct is_wide_bits<wide_bits<Bits>> : std::true_type {};
#else
template <typename T>
struct is_wide_bits : std::false_type {};
#endif

template <typename T>
inline constexpr bool is_wide_bits_v = is_wide_bits<remove_cvref_t<T>>::value;

template <typename T>
inline uint8_t get_byte(const T &value, uint32_t byte_idx) {
  if constexpr (is_wide_bits_v<T>) {
    using ValueT = remove_cvref_t<T>;
    const uint32_t lane_idx = byte_idx / 8u;
    if (lane_idx >= ValueT::kLaneCount) {
      return 0;
    }
    const uint32_t shift = (byte_idx % 8u) * 8u;
    return static_cast<uint8_t>((value.lanes[lane_idx] >> shift) & 0xFFu);
  } else if constexpr (has_byte_array_v<T>) {
    constexpr uint32_t kByteCount =
        sizeof(value.bytes) / sizeof(value.bytes[0]);
    if (byte_idx >= kByteCount) {
      return 0;
    }
    return static_cast<uint8_t>(value.bytes[byte_idx]);
  } else if constexpr (std::is_same_v<remove_cvref_t<T>, bool>) {
    return (byte_idx == 0 && value) ? 1u : 0u;
  } else if constexpr (sizeof(remove_cvref_t<T>) <= sizeof(uint64_t)) {
    if (byte_idx >= sizeof(remove_cvref_t<T>)) {
      return 0;
    }
    const uint64_t raw = static_cast<uint64_t>(value);
    return static_cast<uint8_t>((raw >> (byte_idx * 8u)) & 0xFFu);
  } else {
    if (byte_idx >= sizeof(remove_cvref_t<T>)) {
      return 0;
    }
    const unsigned __int128 raw = static_cast<unsigned __int128>(value);
    return static_cast<uint8_t>(
        (raw >> (byte_idx * 8u)) & static_cast<unsigned __int128>(0xFFu));
  }
}

template <typename T>
inline void set_byte(T &value, uint32_t byte_idx, uint8_t byte) {
  if constexpr (is_wide_bits_v<T>) {
    using ValueT = remove_cvref_t<T>;
    const uint32_t lane_idx = byte_idx / 8u;
    if (lane_idx >= ValueT::kLaneCount) {
      return;
    }
    const uint32_t shift = (byte_idx % 8u) * 8u;
    value.lanes[lane_idx] &= ~(static_cast<uint64_t>(0xFFu) << shift);
    value.lanes[lane_idx] |= static_cast<uint64_t>(byte) << shift;
    value.trim_unused_bits();
  } else if constexpr (has_byte_array_v<T>) {
    constexpr uint32_t kByteCount =
        sizeof(value.bytes) / sizeof(value.bytes[0]);
    if (byte_idx < kByteCount) {
      value.bytes[byte_idx] = byte;
    }
  } else if constexpr (std::is_same_v<remove_cvref_t<T>, bool>) {
    if (byte_idx == 0) {
      value = (byte & 0x1u) != 0u;
    }
  } else if constexpr (sizeof(remove_cvref_t<T>) <= sizeof(uint64_t)) {
    if (byte_idx >= sizeof(remove_cvref_t<T>)) {
      return;
    }
    uint64_t raw = static_cast<uint64_t>(value);
    const uint32_t shift = byte_idx * 8u;
    raw &= ~(static_cast<uint64_t>(0xFFu) << shift);
    raw |= static_cast<uint64_t>(byte) << shift;
    value = static_cast<remove_cvref_t<T>>(raw);
  } else {
    if (byte_idx >= sizeof(remove_cvref_t<T>)) {
      return;
    }
    unsigned __int128 raw = static_cast<unsigned __int128>(value);
    const uint32_t shift = byte_idx * 8u;
    const unsigned __int128 mask =
        static_cast<unsigned __int128>(0xFFu) << shift;
    raw = (raw & ~mask) |
          (static_cast<unsigned __int128>(byte) << shift);
    value = static_cast<remove_cvref_t<T>>(raw);
  }
}

template <typename T>
inline uint32_t get_u32(const T &value, uint32_t word_idx) {
  uint32_t out = 0;
  const uint32_t first_byte = word_idx * 4u;
  for (uint32_t byte = 0; byte < 4u; ++byte) {
    out |= static_cast<uint32_t>(get_byte(value, first_byte + byte))
           << (byte * 8u);
  }
  return out;
}

template <typename T>
inline void set_u32(T &value, uint32_t word_idx, uint32_t word) {
  const uint32_t first_byte = word_idx * 4u;
  for (uint32_t byte = 0; byte < 4u; ++byte) {
    set_byte(value, first_byte + byte,
             static_cast<uint8_t>((word >> (byte * 8u)) & 0xFFu));
  }
}

template <typename T>
inline bool test_bit(const T &value, uint32_t bit_idx) {
  const uint8_t byte = get_byte(value, bit_idx / 8u);
  return ((byte >> (bit_idx % 8u)) & 0x1u) != 0u;
}

template <typename T>
inline void set_bit(T &value, uint32_t bit_idx, bool enabled) {
  const uint32_t byte_idx = bit_idx / 8u;
  uint8_t byte = get_byte(value, byte_idx);
  const uint8_t mask = static_cast<uint8_t>(1u << (bit_idx % 8u));
  if (enabled) {
    byte |= mask;
  } else {
    byte &= static_cast<uint8_t>(~mask);
  }
  set_byte(value, byte_idx, byte);
}

template <typename T>
inline uint32_t low_u32(const T &value) {
  return get_u32(value, 0);
}

template <typename T>
inline uint64_t low_u64(const T &value) {
  uint64_t out = 0;
  for (uint32_t byte = 0; byte < 8u; ++byte) {
    out |= static_cast<uint64_t>(get_byte(value, byte)) << (byte * 8u);
  }
  return out;
}

template <typename T>
inline std::string hex_string(const T &value, uint32_t byte_count) {
  static constexpr char kHex[] = "0123456789abcdef";
  std::string out;
  out.reserve(2 + byte_count * 2u);
  out += "0x";
  for (int idx = static_cast<int>(byte_count) - 1; idx >= 0; --idx) {
    const uint8_t byte = get_byte(value, static_cast<uint32_t>(idx));
    out += kHex[byte >> 4];
    out += kHex[byte & 0x0Fu];
  }
  return out;
}

} // namespace axi_compat

#if !AXI_KIT_USE_PARENT_WIRE_REG
template <int Bits>
struct AutoTypeHelper {
  static_assert(Bits > 0, "wire/reg bit width must be positive");

  using type = std::conditional_t<
      Bits == 1, bool,
      std::conditional_t<
          (Bits <= 8), uint8_t,
          std::conditional_t<
              (Bits <= 16), uint16_t,
              std::conditional_t<
                  (Bits <= 32), uint32_t,
                  std::conditional_t<
                      (Bits <= 64), uint64_t,
                      std::conditional_t<(Bits <= 128), unsigned __int128,
                                         axi_compat::wide_bits<Bits>>>>>>>;
};

template <int Bits>
using AutoType = typename AutoTypeHelper<Bits>::type;
template <int Bits>
using wire = AutoType<Bits>;
template <int Bits>
using reg = AutoType<Bits>;

static_assert(std::is_same_v<wire<1>, bool>);
static_assert(std::is_same_v<wire<32>, uint32_t>);
static_assert(sizeof(wire<128>) == 16,
              "wire<128> must remain a real 128-bit carrier in standalone "
              "axi-interconnect-kit");
static_assert(sizeof(wire<256>) == 32,
              "wire<256> must remain a real 256-bit carrier in standalone "
              "axi-interconnect-kit");
#endif

#if AXI_KIT_HAS_PARENT_CONFIG
#ifndef CONFIG_SIM_DDR_LATENCY
#error "simulator config.h must define CONFIG_SIM_DDR_LATENCY for AXI kit integration"
#endif
#ifndef CONFIG_AXI_KIT_SIM_DDR_WRITE_RESP_LATENCY
#error "simulator config.h must define CONFIG_AXI_KIT_SIM_DDR_WRITE_RESP_LATENCY"
#endif
#ifndef CONFIG_AXI_KIT_SIM_DDR_BEAT_BYTES
#error "simulator config.h must define CONFIG_AXI_KIT_SIM_DDR_BEAT_BYTES"
#endif
#ifndef CONFIG_AXI_KIT_MAX_OUTSTANDING
#error "simulator config.h must define CONFIG_AXI_KIT_MAX_OUTSTANDING"
#endif
#ifndef CONFIG_AXI_KIT_MAX_READ_OUTSTANDING_PER_MASTER
#error "simulator config.h must define CONFIG_AXI_KIT_MAX_READ_OUTSTANDING_PER_MASTER"
#endif
#ifndef CONFIG_AXI_KIT_MAX_WRITE_OUTSTANDING
#error "simulator config.h must define CONFIG_AXI_KIT_MAX_WRITE_OUTSTANDING"
#endif
#ifndef CONFIG_AXI_KIT_MAX_WRITE_TRANSACTION_BYTES
#error "simulator config.h must define CONFIG_AXI_KIT_MAX_WRITE_TRANSACTION_BYTES"
#endif
#ifndef CONFIG_AXI_KIT_SIM_DDR_MAX_OUTSTANDING
#error "simulator config.h must define CONFIG_AXI_KIT_SIM_DDR_MAX_OUTSTANDING"
#endif
#ifndef CONFIG_AXI_KIT_SIM_DDR_WRITE_QUEUE_DEPTH
#error "simulator config.h must define CONFIG_AXI_KIT_SIM_DDR_WRITE_QUEUE_DEPTH"
#endif
#ifndef CONFIG_AXI_KIT_SIM_DDR_WRITE_ACCEPT_GAP
#error "simulator config.h must define CONFIG_AXI_KIT_SIM_DDR_WRITE_ACCEPT_GAP"
#endif
#ifndef CONFIG_AXI_KIT_SIM_DDR_WRITE_DATA_FIFO_DEPTH
#error "simulator config.h must define CONFIG_AXI_KIT_SIM_DDR_WRITE_DATA_FIFO_DEPTH"
#endif
#ifndef CONFIG_AXI_KIT_SIM_DDR_WRITE_DRAIN_GAP
#error "simulator config.h must define CONFIG_AXI_KIT_SIM_DDR_WRITE_DRAIN_GAP"
#endif
#ifndef CONFIG_AXI_KIT_SIM_DDR_READ_TO_WRITE_TURNAROUND
#error "simulator config.h must define CONFIG_AXI_KIT_SIM_DDR_READ_TO_WRITE_TURNAROUND"
#endif
#ifndef CONFIG_AXI_KIT_SIM_DDR_WRITE_TO_READ_TURNAROUND
#error "simulator config.h must define CONFIG_AXI_KIT_SIM_DDR_WRITE_TO_READ_TURNAROUND"
#endif
#ifndef CONFIG_AXI_KIT_AXI_ID_WIDTH
#error "simulator config.h must define CONFIG_AXI_KIT_AXI_ID_WIDTH"
#endif
#ifndef CONFIG_AXI_LLC_DCACHE_READ_MISS_NOALLOC
#error "simulator config.h must define CONFIG_AXI_LLC_DCACHE_READ_MISS_NOALLOC"
#endif
#ifndef CONFIG_AXI_KIT_DEBUG
#error "simulator config.h must define CONFIG_AXI_KIT_DEBUG"
#endif
#ifndef CONFIG_AXI_KIT_UART_BASE
#error "simulator config.h must define CONFIG_AXI_KIT_UART_BASE"
#endif
#ifndef CONFIG_AXI_KIT_MMIO_BASE
#error "simulator config.h must define CONFIG_AXI_KIT_MMIO_BASE"
#endif
#ifndef CONFIG_AXI_KIT_MMIO_SIZE
#error "simulator config.h must define CONFIG_AXI_KIT_MMIO_SIZE"
#endif
#endif

// Configurable defaults for standalone build (can be overridden via -D).
#ifndef CONFIG_SIM_DDR_LATENCY
#define CONFIG_SIM_DDR_LATENCY 50
#endif

#ifndef AXI_KIT_SIM_DDR_READ_LATENCY
#define AXI_KIT_SIM_DDR_READ_LATENCY CONFIG_SIM_DDR_LATENCY
#endif

#ifndef AXI_KIT_SIM_DDR_WRITE_RESP_LATENCY
#ifdef CONFIG_AXI_KIT_SIM_DDR_WRITE_RESP_LATENCY
#define AXI_KIT_SIM_DDR_WRITE_RESP_LATENCY \
  CONFIG_AXI_KIT_SIM_DDR_WRITE_RESP_LATENCY
#else
#define AXI_KIT_SIM_DDR_WRITE_RESP_LATENCY 1
#endif
#endif

#ifndef AXI_KIT_SIM_DDR_MAX_OUTSTANDING
#ifdef CONFIG_AXI_KIT_SIM_DDR_MAX_OUTSTANDING
#define AXI_KIT_SIM_DDR_MAX_OUTSTANDING CONFIG_AXI_KIT_SIM_DDR_MAX_OUTSTANDING
#else
#define AXI_KIT_SIM_DDR_MAX_OUTSTANDING 8
#endif
#endif

#ifndef AXI_KIT_SIM_DDR_WRITE_QUEUE_DEPTH
#ifdef CONFIG_AXI_KIT_SIM_DDR_WRITE_QUEUE_DEPTH
#define AXI_KIT_SIM_DDR_WRITE_QUEUE_DEPTH \
  CONFIG_AXI_KIT_SIM_DDR_WRITE_QUEUE_DEPTH
#else
#define AXI_KIT_SIM_DDR_WRITE_QUEUE_DEPTH AXI_KIT_SIM_DDR_MAX_OUTSTANDING
#endif
#endif

#ifndef AXI_KIT_SIM_DDR_WRITE_ACCEPT_GAP
#ifdef CONFIG_AXI_KIT_SIM_DDR_WRITE_ACCEPT_GAP
#define AXI_KIT_SIM_DDR_WRITE_ACCEPT_GAP \
  CONFIG_AXI_KIT_SIM_DDR_WRITE_ACCEPT_GAP
#else
#define AXI_KIT_SIM_DDR_WRITE_ACCEPT_GAP 0
#endif
#endif

#ifndef AXI_KIT_SIM_DDR_WRITE_DATA_FIFO_DEPTH
#ifdef CONFIG_AXI_KIT_SIM_DDR_WRITE_DATA_FIFO_DEPTH
#define AXI_KIT_SIM_DDR_WRITE_DATA_FIFO_DEPTH \
  CONFIG_AXI_KIT_SIM_DDR_WRITE_DATA_FIFO_DEPTH
#else
#define AXI_KIT_SIM_DDR_WRITE_DATA_FIFO_DEPTH 8
#endif
#endif

#ifndef AXI_KIT_SIM_DDR_WRITE_DRAIN_GAP
#ifdef CONFIG_AXI_KIT_SIM_DDR_WRITE_DRAIN_GAP
#define AXI_KIT_SIM_DDR_WRITE_DRAIN_GAP \
  CONFIG_AXI_KIT_SIM_DDR_WRITE_DRAIN_GAP
#else
#define AXI_KIT_SIM_DDR_WRITE_DRAIN_GAP 0
#endif
#endif

#ifndef AXI_KIT_SIM_DDR_READ_TO_WRITE_TURNAROUND
#ifdef CONFIG_AXI_KIT_SIM_DDR_READ_TO_WRITE_TURNAROUND
#define AXI_KIT_SIM_DDR_READ_TO_WRITE_TURNAROUND \
  CONFIG_AXI_KIT_SIM_DDR_READ_TO_WRITE_TURNAROUND
#else
#define AXI_KIT_SIM_DDR_READ_TO_WRITE_TURNAROUND 0
#endif
#endif

#ifndef AXI_KIT_SIM_DDR_WRITE_TO_READ_TURNAROUND
#ifdef CONFIG_AXI_KIT_SIM_DDR_WRITE_TO_READ_TURNAROUND
#define AXI_KIT_SIM_DDR_WRITE_TO_READ_TURNAROUND \
  CONFIG_AXI_KIT_SIM_DDR_WRITE_TO_READ_TURNAROUND
#else
#define AXI_KIT_SIM_DDR_WRITE_TO_READ_TURNAROUND 0
#endif
#endif

#ifndef AXI_KIT_SIM_DDR_WRITE_DRAIN_HIGH_WATERMARK
#ifdef CONFIG_AXI_KIT_SIM_DDR_WRITE_DRAIN_HIGH_WATERMARK
#define AXI_KIT_SIM_DDR_WRITE_DRAIN_HIGH_WATERMARK \
  CONFIG_AXI_KIT_SIM_DDR_WRITE_DRAIN_HIGH_WATERMARK
#else
#define AXI_KIT_SIM_DDR_WRITE_DRAIN_HIGH_WATERMARK \
  AXI_KIT_SIM_DDR_WRITE_DATA_FIFO_DEPTH
#endif
#endif

#ifndef AXI_KIT_SIM_DDR_WRITE_DRAIN_LOW_WATERMARK
#ifdef CONFIG_AXI_KIT_SIM_DDR_WRITE_DRAIN_LOW_WATERMARK
#define AXI_KIT_SIM_DDR_WRITE_DRAIN_LOW_WATERMARK \
  CONFIG_AXI_KIT_SIM_DDR_WRITE_DRAIN_LOW_WATERMARK
#else
#define AXI_KIT_SIM_DDR_WRITE_DRAIN_LOW_WATERMARK 0
#endif
#endif

#ifndef AXI_KIT_AXI_ID_WIDTH
#ifdef CONFIG_AXI_KIT_AXI_ID_WIDTH
#define AXI_KIT_AXI_ID_WIDTH CONFIG_AXI_KIT_AXI_ID_WIDTH
#else
#define AXI_KIT_AXI_ID_WIDTH 6
#endif
#endif

#ifndef AXI_KIT_LLC_DCACHE_READ_MISS_NOALLOC
#ifdef CONFIG_AXI_LLC_DCACHE_READ_MISS_NOALLOC
#define AXI_KIT_LLC_DCACHE_READ_MISS_NOALLOC \
  CONFIG_AXI_LLC_DCACHE_READ_MISS_NOALLOC
#else
#define AXI_KIT_LLC_DCACHE_READ_MISS_NOALLOC 0
#endif
#endif

#ifndef AXI_KIT_SIM_DDR_BEAT_BYTES
#ifdef CONFIG_AXI_KIT_SIM_DDR_BEAT_BYTES
#define AXI_KIT_SIM_DDR_BEAT_BYTES CONFIG_AXI_KIT_SIM_DDR_BEAT_BYTES
#else
#define AXI_KIT_SIM_DDR_BEAT_BYTES 4
#endif
#endif

#ifndef AXI_KIT_MAX_OUTSTANDING
#ifdef CONFIG_AXI_KIT_MAX_OUTSTANDING
#define AXI_KIT_MAX_OUTSTANDING CONFIG_AXI_KIT_MAX_OUTSTANDING
#else
#define AXI_KIT_MAX_OUTSTANDING 8
#endif
#endif

#ifndef AXI_KIT_MAX_READ_OUTSTANDING_PER_MASTER
#ifdef CONFIG_AXI_KIT_MAX_READ_OUTSTANDING_PER_MASTER
#define AXI_KIT_MAX_READ_OUTSTANDING_PER_MASTER \
  CONFIG_AXI_KIT_MAX_READ_OUTSTANDING_PER_MASTER
#else
#define AXI_KIT_MAX_READ_OUTSTANDING_PER_MASTER 4
#endif
#endif

#ifndef AXI_KIT_MAX_WRITE_OUTSTANDING
#ifdef CONFIG_AXI_KIT_MAX_WRITE_OUTSTANDING
#define AXI_KIT_MAX_WRITE_OUTSTANDING CONFIG_AXI_KIT_MAX_WRITE_OUTSTANDING
#else
#define AXI_KIT_MAX_WRITE_OUTSTANDING 8
#endif
#endif

#ifndef AXI_KIT_MAX_WRITE_TRANSACTION_BYTES
#ifdef CONFIG_AXI_KIT_MAX_WRITE_TRANSACTION_BYTES
#define AXI_KIT_MAX_WRITE_TRANSACTION_BYTES \
  CONFIG_AXI_KIT_MAX_WRITE_TRANSACTION_BYTES
#else
#define AXI_KIT_MAX_WRITE_TRANSACTION_BYTES 64
#endif
#endif

#ifndef AXI_KIT_DEBUG
#ifdef CONFIG_AXI_KIT_DEBUG
#define AXI_KIT_DEBUG CONFIG_AXI_KIT_DEBUG
#else
#define AXI_KIT_DEBUG 0
#endif
#endif

#ifndef AXI_KIT_DCACHE_LOG
#ifdef CONFIG_AXI_KIT_DCACHE_LOG
#define AXI_KIT_DCACHE_LOG CONFIG_AXI_KIT_DCACHE_LOG
#else
#define AXI_KIT_DCACHE_LOG 0
#endif
#endif

#ifndef AXI_KIT_UART_BASE
#ifdef CONFIG_AXI_KIT_UART_BASE
#define AXI_KIT_UART_BASE CONFIG_AXI_KIT_UART_BASE
#else
#define AXI_KIT_UART_BASE 0x10000000u
#endif
#endif

#ifndef AXI_KIT_MMIO_BASE
#ifdef CONFIG_AXI_KIT_MMIO_BASE
#define AXI_KIT_MMIO_BASE CONFIG_AXI_KIT_MMIO_BASE
#elif defined(MMIO_TEST_BASE)
#define AXI_KIT_MMIO_BASE MMIO_TEST_BASE
#else
#define AXI_KIT_MMIO_BASE AXI_KIT_UART_BASE
#endif
#endif

#ifdef MMIO_TEST_BASE
#undef AXI_KIT_MMIO_BASE
#define AXI_KIT_MMIO_BASE MMIO_TEST_BASE
#endif

#ifndef AXI_KIT_MMIO_SIZE
#ifdef CONFIG_AXI_KIT_MMIO_SIZE
#define AXI_KIT_MMIO_SIZE CONFIG_AXI_KIT_MMIO_SIZE
#elif defined(MMIO_TEST_SIZE)
#define AXI_KIT_MMIO_SIZE MMIO_TEST_SIZE
#else
#define AXI_KIT_MMIO_SIZE 0x00001000u
#endif
#endif

#ifdef MMIO_TEST_SIZE
#undef AXI_KIT_MMIO_SIZE
#define AXI_KIT_MMIO_SIZE MMIO_TEST_SIZE
#endif

static_assert(AXI_KIT_AXI_ID_WIDTH > 0,
              "AXI_KIT_AXI_ID_WIDTH must be positive");
static_assert(AXI_KIT_AXI_ID_WIDTH <= 7,
              "AXI_KIT_AXI_ID_WIDTH must stay <= 7; 0xFF is reserved as invalid ID");
static_assert(
    AXI_KIT_LLC_DCACHE_READ_MISS_NOALLOC == 0 ||
        AXI_KIT_LLC_DCACHE_READ_MISS_NOALLOC == 1,
    "AXI_KIT_LLC_DCACHE_READ_MISS_NOALLOC must be 0 or 1");
static_assert(AXI_KIT_MAX_OUTSTANDING <= (1u << AXI_KIT_AXI_ID_WIDTH),
              "AXI_KIT_MAX_OUTSTANDING exceeds available AXI IDs");
static_assert(AXI_KIT_SIM_DDR_WRITE_QUEUE_DEPTH > 0,
              "AXI_KIT_SIM_DDR_WRITE_QUEUE_DEPTH must be positive");
static_assert(AXI_KIT_SIM_DDR_WRITE_DATA_FIFO_DEPTH > 0,
              "AXI_KIT_SIM_DDR_WRITE_DATA_FIFO_DEPTH must be positive");
static_assert(AXI_KIT_SIM_DDR_WRITE_DRAIN_HIGH_WATERMARK > 0,
              "AXI_KIT_SIM_DDR_WRITE_DRAIN_HIGH_WATERMARK must be positive");
static_assert(AXI_KIT_SIM_DDR_WRITE_DRAIN_HIGH_WATERMARK <=
                  AXI_KIT_SIM_DDR_WRITE_DATA_FIFO_DEPTH,
              "AXI_KIT_SIM_DDR_WRITE_DRAIN_HIGH_WATERMARK exceeds the write data FIFO depth");
static_assert(AXI_KIT_SIM_DDR_WRITE_DRAIN_LOW_WATERMARK <
                  AXI_KIT_SIM_DDR_WRITE_DRAIN_HIGH_WATERMARK,
              "AXI_KIT_SIM_DDR_WRITE_DRAIN_LOW_WATERMARK must stay below the high watermark");
static_assert(AXI_KIT_SIM_DDR_READ_TO_WRITE_TURNAROUND >= 0,
              "AXI_KIT_SIM_DDR_READ_TO_WRITE_TURNAROUND must be non-negative");
static_assert(AXI_KIT_SIM_DDR_WRITE_TO_READ_TURNAROUND >= 0,
              "AXI_KIT_SIM_DDR_WRITE_TO_READ_TURNAROUND must be non-negative");
static_assert(AXI_KIT_MAX_READ_OUTSTANDING_PER_MASTER <= AXI_KIT_MAX_OUTSTANDING,
              "AXI_KIT_MAX_READ_OUTSTANDING_PER_MASTER exceeds total outstanding budget");
static_assert(AXI_KIT_MAX_WRITE_OUTSTANDING <= (1u << AXI_KIT_AXI_ID_WIDTH),
              "AXI_KIT_MAX_WRITE_OUTSTANDING exceeds available AXI IDs");

#if AXI_KIT_HAS_PARENT_CONFIG
static_assert(AXI_KIT_SIM_DDR_READ_LATENCY == CONFIG_SIM_DDR_LATENCY,
              "parent simulator DDR read latency must come from config.h");
static_assert(
    AXI_KIT_SIM_DDR_WRITE_RESP_LATENCY ==
        CONFIG_AXI_KIT_SIM_DDR_WRITE_RESP_LATENCY,
    "parent simulator DDR write response latency must come from config.h");
static_assert(AXI_KIT_SIM_DDR_BEAT_BYTES == CONFIG_AXI_KIT_SIM_DDR_BEAT_BYTES,
              "parent simulator DDR beat bytes must come from config.h");
static_assert(AXI_KIT_MAX_OUTSTANDING == CONFIG_AXI_KIT_MAX_OUTSTANDING,
              "parent simulator AXI outstanding cap must come from config.h");
static_assert(AXI_KIT_MAX_READ_OUTSTANDING_PER_MASTER ==
                  CONFIG_AXI_KIT_MAX_READ_OUTSTANDING_PER_MASTER,
              "parent simulator read outstanding cap must come from config.h");
static_assert(AXI_KIT_MAX_WRITE_OUTSTANDING ==
                  CONFIG_AXI_KIT_MAX_WRITE_OUTSTANDING,
              "parent simulator write outstanding cap must come from config.h");
static_assert(
    AXI_KIT_MAX_WRITE_TRANSACTION_BYTES ==
        CONFIG_AXI_KIT_MAX_WRITE_TRANSACTION_BYTES,
    "parent simulator AXI write payload width must come from config.h");
static_assert(AXI_KIT_SIM_DDR_MAX_OUTSTANDING ==
                  CONFIG_AXI_KIT_SIM_DDR_MAX_OUTSTANDING,
              "parent simulator SimDDR queue depth must come from config.h");
static_assert(AXI_KIT_SIM_DDR_WRITE_QUEUE_DEPTH ==
                  CONFIG_AXI_KIT_SIM_DDR_WRITE_QUEUE_DEPTH,
              "parent simulator SimDDR write queue depth must come from config.h");
static_assert(AXI_KIT_SIM_DDR_WRITE_ACCEPT_GAP ==
                  CONFIG_AXI_KIT_SIM_DDR_WRITE_ACCEPT_GAP,
              "parent simulator SimDDR write accept gap must come from config.h");
static_assert(AXI_KIT_SIM_DDR_WRITE_DATA_FIFO_DEPTH ==
                  CONFIG_AXI_KIT_SIM_DDR_WRITE_DATA_FIFO_DEPTH,
              "parent simulator SimDDR write data fifo depth must come from config.h");
static_assert(AXI_KIT_SIM_DDR_WRITE_DRAIN_GAP ==
                  CONFIG_AXI_KIT_SIM_DDR_WRITE_DRAIN_GAP,
              "parent simulator SimDDR write drain gap must come from config.h");
static_assert(
    AXI_KIT_SIM_DDR_READ_TO_WRITE_TURNAROUND ==
        CONFIG_AXI_KIT_SIM_DDR_READ_TO_WRITE_TURNAROUND,
    "parent simulator SimDDR read-to-write turnaround must come from config.h");
static_assert(
    AXI_KIT_SIM_DDR_WRITE_TO_READ_TURNAROUND ==
        CONFIG_AXI_KIT_SIM_DDR_WRITE_TO_READ_TURNAROUND,
    "parent simulator SimDDR write-to-read turnaround must come from config.h");
static_assert(
    AXI_KIT_SIM_DDR_WRITE_DRAIN_HIGH_WATERMARK ==
        CONFIG_AXI_KIT_SIM_DDR_WRITE_DRAIN_HIGH_WATERMARK,
    "parent simulator SimDDR write drain high watermark must come from config.h");
static_assert(
    AXI_KIT_SIM_DDR_WRITE_DRAIN_LOW_WATERMARK ==
        CONFIG_AXI_KIT_SIM_DDR_WRITE_DRAIN_LOW_WATERMARK,
    "parent simulator SimDDR write drain low watermark must come from config.h");
static_assert(AXI_KIT_AXI_ID_WIDTH == CONFIG_AXI_KIT_AXI_ID_WIDTH,
              "parent simulator AXI ID width must come from config.h");
static_assert(AXI_KIT_LLC_DCACHE_READ_MISS_NOALLOC ==
                  CONFIG_AXI_LLC_DCACHE_READ_MISS_NOALLOC,
              "parent simulator LLC dcache read miss allocation policy must "
              "come from config.h");
static_assert(AXI_KIT_UART_BASE == CONFIG_AXI_KIT_UART_BASE,
              "parent simulator UART base must come from config.h");
static_assert(AXI_KIT_MMIO_BASE == CONFIG_AXI_KIT_MMIO_BASE,
              "parent simulator MMIO base must come from config.h");
static_assert(AXI_KIT_MMIO_SIZE == CONFIG_AXI_KIT_MMIO_SIZE,
              "parent simulator MMIO size must come from config.h");
#endif

#ifndef DEBUG
#define DEBUG AXI_KIT_DEBUG
#endif

#ifndef DCACHE_LOG
#define DCACHE_LOG AXI_KIT_DCACHE_LOG
#endif

#ifndef UART_BASE
#define UART_BASE AXI_KIT_UART_BASE
#endif

#ifndef MMIO_BASE
#define MMIO_BASE AXI_KIT_MMIO_BASE
#endif

#ifndef MMIO_SIZE
#define MMIO_SIZE AXI_KIT_MMIO_SIZE
#endif
