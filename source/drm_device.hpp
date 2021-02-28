#pragma once

#include <cstdint>

namespace drm {
using Context = std::uint32_t;
using Buffer = std::uint32_t;
using BufferList = std::uint32_t;
using SubmitSequence = std::uint64_t;

enum class Generation {
  GFX6,
  GFX7,
  GFX8,
  GFX9,
  GFX10,
  GFX10_3,
};

enum class Domain : std::uint32_t {
  kNone = 0,
  kCpu = 1,
  kGtt = 2,
  kVram = 4,
  kGds = 8,
  kGws = 16,
  kOa = 32
};

Domain operator|(Domain a, Domain b) noexcept;
Domain operator&(Domain a, Domain b) noexcept;

enum class BufferFlags : std::uint32_t {
  kNone = 0,
  kCpuAccess = 1,
  kWriteCombine = 4,
  kClearedVram = 8
};

BufferFlags operator|(BufferFlags a, BufferFlags b) noexcept;
BufferFlags operator&(BufferFlags a, BufferFlags b) noexcept;

enum class MapFlags : std::uint32_t {
  kNone = 0,
  kRead = 1,
  kWrite = 2,
  kFixed = 4
};

MapFlags operator|(MapFlags a, MapFlags b) noexcept;
MapFlags operator&(MapFlags a, MapFlags b) noexcept;

enum class HwType : std::uint32_t { kGfx, kCompute, kDma };
struct BufferListEntry {
  Buffer buffer;
  std::uint32_t priority;
};

struct Fence {
  HwType hw_type;
  unsigned hw_instance;
  unsigned ring;
  Context context;
  SubmitSequence seq;
};

struct SubmitBuffer {
  uint64_t address = 0;
  uint64_t size = 0;
  bool is_ce = false;
  bool is_preamble = false;
};

struct SubmitArgs {
  Context context;
  HwType hw_type = HwType::kGfx;
  unsigned ring = 0;
  BufferList buffers = 0;
  const SubmitBuffer *command_buffers = nullptr;
  unsigned command_buffer_count = 0;

  const Fence *fences = nullptr;
  unsigned fence_count = 0;

  Buffer memory_fence_buffer = 0;
  uint32_t memory_fence_offset = 0;
};

struct DeviceInfo {
  std::uint64_t virtual_address_start;
  std::uint64_t virtual_address_end;
  double clock_freq;
};

class Device {
public:
  Device() noexcept;
  explicit Device(int fd) noexcept;
  Device(Device &&other) noexcept;
  Device(Device const &) = delete;
  Device &operator=(Device &&other) noexcept;
  Device &operator=(Device const &) = delete;
  ~Device() noexcept;

  bool IsValid() const noexcept;

  DeviceInfo GetInfo();
  Generation GetGeneration() const noexcept { return gen_; }

  Context CreateContext();
  void DestroyContext(Context ctx);

  Buffer CreateBuffer(std::uint64_t size, std::uint64_t alignment,
                      Domain domain, BufferFlags flags);
  void DestroyBuffer(Buffer b);

  void MapBuffer(Buffer buffer, std::uint64_t address, std::uint64_t size,
                 std::uint64_t offset);
  void UnmapBuffer(Buffer buffer, std::uint64_t address, std::uint64_t size,
                   std::uint64_t offset);

  BufferList CreateBufferList(const BufferListEntry *entries,
                              std::uint32_t count);
  void DestroyBufferList(BufferList list);

  void *MapBufferCpu(Buffer buffer, std::uint64_t size, MapFlags flags,
                     std::uint64_t map_address = 0);

  void Submit(const SubmitArgs &args, Fence *out_fence);

private:
  int fd_;
  Generation gen_;
};

inline Domain operator|(Domain a, Domain b) noexcept {
  return static_cast<Domain>(static_cast<std::uint32_t>(a) |
                             static_cast<std::uint32_t>(b));
}

inline Domain operator&(Domain a, Domain b) noexcept {
  return static_cast<Domain>(static_cast<std::uint32_t>(a) &
                             static_cast<std::uint32_t>(b));
}

inline BufferFlags operator|(BufferFlags a, BufferFlags b) noexcept {
  return static_cast<BufferFlags>(static_cast<std::uint32_t>(a) |
                                  static_cast<std::uint32_t>(b));
}

inline BufferFlags operator&(BufferFlags a, BufferFlags b) noexcept {
  return static_cast<BufferFlags>(static_cast<std::uint32_t>(a) &
                                  static_cast<std::uint32_t>(b));
}

inline MapFlags operator|(MapFlags a, MapFlags b) noexcept {
  return static_cast<MapFlags>(static_cast<std::uint32_t>(a) |
                               static_cast<std::uint32_t>(b));
}

inline MapFlags operator&(MapFlags a, MapFlags b) noexcept {
  return static_cast<MapFlags>(static_cast<std::uint32_t>(a) &
                               static_cast<std::uint32_t>(b));
}

} // namespace drm
