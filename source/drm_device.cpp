#include "drm_device.hpp"

#include <algorithm>
#include <memory>
#include <system_error>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <amdgpu_drm.h>
#include <xf86drm.h>

namespace drm {

static_assert(sizeof(BufferListEntry) == sizeof(drm_amdgpu_bo_list_entry),
              "BufferListEntry size mismatched");

namespace {
void WrappedIoctl(int fd, unsigned long request, void *arg) {
  int ret;

  do {
    ret = ioctl(fd, request, arg);
  } while (ret == -1 && (errno == EINTR || errno == EAGAIN));
  if (ret)
    throw std::system_error(errno, std::generic_category());
}

template <typename T>
void WriteIoctl(int fd, unsigned long command, T const &data) {
  auto request = DRM_IOC(DRM_IOC_WRITE, DRM_IOCTL_BASE,
                         DRM_COMMAND_BASE + command, sizeof(data));
  WrappedIoctl(fd, request, &const_cast<T &>(data));
}

template <typename T>
void WriteReadIoctl(int fd, unsigned long command, T const &data) {
  auto request = DRM_IOC(DRM_IOC_WRITE | DRM_IOC_READ, DRM_IOCTL_BASE,
                         DRM_COMMAND_BASE + command, sizeof(data));
  WrappedIoctl(fd, request, &const_cast<T &>(data));
}

template <typename T> T QueryInfo(int fd, unsigned kind) {
  T ret;
  drm_amdgpu_info request = {};
  request.return_pointer = reinterpret_cast<std::uintptr_t>(&ret);
  request.return_size = sizeof(T);
  request.query = kind;
  WriteIoctl(fd, DRM_AMDGPU_INFO, request);
  return ret;
}
}

Device::Device() noexcept : fd_{-1} {}

Device::Device(int fd) noexcept : fd_{fd} {}

Device::Device(Device &&other) noexcept : fd_{other.fd_} { other.fd_ = -1; }

Device &Device::operator=(Device &&other) noexcept {
  if (this != &other)
    std::swap(fd_, other.fd_);
  return *this;
}

Device::~Device() noexcept {
  if (fd_ >= 0)
    close(fd_);
}

bool Device::IsValid() const noexcept { return fd_ >= 0; }

DeviceInfo Device::GetInfo() {
  DeviceInfo info = {};
  auto amdgpu_device_info =
      QueryInfo<drm_amdgpu_info_device>(fd_, AMDGPU_INFO_DEV_INFO);

  info.virtual_address_start = amdgpu_device_info.virtual_address_offset;
  info.virtual_address_end = amdgpu_device_info.virtual_address_max;
  return info;
}

Context Device::CreateContext() {
  drm_amdgpu_ctx args = {};
  args.in.op = AMDGPU_CTX_OP_ALLOC_CTX;
  WriteReadIoctl(fd_, DRM_AMDGPU_CTX, args);
  return args.out.alloc.ctx_id;
}

void Device::DestroyContext(Context ctx) {
  drm_amdgpu_ctx args = {};
  args.in.op = AMDGPU_CTX_OP_FREE_CTX;
  args.in.ctx_id = ctx;
  WriteReadIoctl(fd_, DRM_AMDGPU_CTX, args);
}

Buffer Device::CreateBuffer(std::uint64_t size, std::uint64_t alignment,
                            Domain domain, BufferFlags flags) {
  drm_amdgpu_gem_create args = {};
  args.in.bo_size = size;
  args.in.alignment = alignment;
  args.in.domains = static_cast<std::uint32_t>(domain);
  args.in.domain_flags = static_cast<std::uint32_t>(flags);
  if ((flags & BufferFlags::kCpuAccess) == BufferFlags::kNone)
    args.in.domain_flags |= AMDGPU_GEM_CREATE_NO_CPU_ACCESS;

  WriteReadIoctl(fd_, DRM_AMDGPU_GEM_CREATE, args);
  return args.out.handle;
}

void Device::DestroyBuffer(Buffer b) {
  drm_gem_close args = {};

  args.handle = b;
  WrappedIoctl(fd_, DRM_IOCTL_GEM_CLOSE, &args);
}

void Device::MapBuffer(Buffer buffer, std::uint64_t address, std::uint64_t size,
                       std::uint64_t offset) {
  drm_amdgpu_gem_va args = {};
  args.handle = buffer;
  args.operation = AMDGPU_VA_OP_MAP;
  args.va_address = address;
  args.map_size = size;
  args.offset_in_bo = offset;
  args.flags = AMDGPU_VM_PAGE_READABLE | AMDGPU_VM_PAGE_WRITEABLE |
               AMDGPU_VM_PAGE_EXECUTABLE;
  WriteReadIoctl(fd_, DRM_AMDGPU_GEM_VA, args);
}

void Device::UnmapBuffer(Buffer buffer, std::uint64_t address,
                         std::uint64_t size, std::uint64_t offset) {
  drm_amdgpu_gem_va args = {};
  args.handle = buffer;
  args.operation = AMDGPU_VA_OP_UNMAP;
  args.va_address = address;
  args.map_size = size;
  args.offset_in_bo = offset;
  args.flags = AMDGPU_VM_PAGE_READABLE | AMDGPU_VM_PAGE_WRITEABLE |
               AMDGPU_VM_PAGE_EXECUTABLE;
  WriteReadIoctl(fd_, DRM_AMDGPU_GEM_VA, args);
}

BufferList Device::CreateBufferList(const BufferListEntry *entries,
                                    std::uint32_t count) {
  drm_amdgpu_bo_list args = {};
  args.in.operation = AMDGPU_BO_LIST_OP_CREATE;
  args.in.bo_number = count;
  args.in.bo_info_size = sizeof(BufferListEntry);
  args.in.bo_info_ptr = reinterpret_cast<std::uintptr_t>(entries);
  WriteReadIoctl(fd_, DRM_AMDGPU_BO_LIST, args);
  return args.out.list_handle;
}

void Device::DestroyBufferList(BufferList list) {
  drm_amdgpu_bo_list args = {};
  args.in.operation = AMDGPU_BO_LIST_OP_DESTROY;
  args.in.list_handle = list;
  WriteReadIoctl(fd_, DRM_AMDGPU_BO_LIST, args);
}

void *Device::MapBufferCpu(Buffer buffer, std::uint64_t size, MapFlags flags,
                           std::uint64_t map_address) {
  drm_amdgpu_gem_mmap args = {};
  args.in.handle = buffer;
  WriteReadIoctl(fd_, DRM_AMDGPU_GEM_MMAP, args);

  int prot = 0;
  int internal_flags = MAP_SHARED;
  if ((flags & MapFlags::kRead) != MapFlags::kNone)
    prot |= PROT_READ;
  if ((flags & MapFlags::kWrite) != MapFlags::kNone)
    prot |= PROT_WRITE;

  void *p = mmap(reinterpret_cast<void *>(map_address), size, prot,
                 internal_flags, fd_, args.out.addr_ptr);
  if (p == MAP_FAILED) {
    throw std::system_error(errno, std::generic_category());
  }
  return p;
}

void Device::Submit(const SubmitArgs &args, Fence *out_fence) {
  drm_amdgpu_cs cs_args = {};
  auto chunk_count = args.command_buffer_count + !!args.fence_count +
                     !!args.memory_fence_buffer;
  auto chunk_pointers = std::make_unique<std::uint64_t[]>(chunk_count);
  auto chunks = std::make_unique<drm_amdgpu_cs_chunk[]>(chunk_count);
  auto chunk = chunks.get();
  auto ib_chunks =
      std::make_unique<drm_amdgpu_cs_chunk_ib[]>(args.command_buffer_count);
  drm_amdgpu_cs_chunk_fence fence_chunk;

  for (unsigned i = 0; i < chunk_count; ++i)
    chunk_pointers[i] = reinterpret_cast<std::uintptr_t>(&chunks[i]);

  for (unsigned i = 0; i < args.command_buffer_count; ++i, ++chunk) {
    chunk->chunk_id = AMDGPU_CHUNK_ID_IB;
    chunk->length_dw = sizeof(ib_chunks[i]) / sizeof(std::uint32_t);
    chunk->chunk_data = reinterpret_cast<std::uintptr_t>(&ib_chunks[i]);
    ib_chunks[i].ip_type = static_cast<std::uint32_t>(args.hw_type);
    ib_chunks[i].ip_instance = 0;
    ib_chunks[i].ring = args.ring;
    ib_chunks[i].va_start = args.command_buffers[i].address;
    ib_chunks[i].ib_bytes = args.command_buffers[i].size;
    ib_chunks[i].flags = 0;
    if (args.command_buffers[i].is_ce) {
      ib_chunks[i].flags |= AMDGPU_IB_FLAG_CE;
      if (args.command_buffers[i].is_preamble)
        ib_chunks[i].flags |= AMDGPU_IB_FLAG_PREAMBLE;
    }
  }

  if (args.memory_fence_buffer) {
    chunk->chunk_id = AMDGPU_CHUNK_ID_FENCE;
    chunk->length_dw =
        sizeof(drm_amdgpu_cs_chunk_fence) / sizeof(std::uint32_t);
    chunk->chunk_data = reinterpret_cast<std::uintptr_t>(&fence_chunk);
    ++chunk;

    fence_chunk.handle = args.memory_fence_buffer;
    fence_chunk.offset = args.memory_fence_offset;
  }

  if (args.fence_count) {
    chunk->chunk_id = AMDGPU_CHUNK_ID_DEPENDENCIES;
    chunk->length_dw = args.fence_count * sizeof(Fence) / sizeof(std::uint32_t);
    chunk->chunk_data = reinterpret_cast<std::uintptr_t>(args.fences);
    ++chunk;
  }

  cs_args.in.ctx_id = args.context;
  cs_args.in.bo_list_handle = args.buffers;
  cs_args.in.num_chunks = chunk_count;
  cs_args.in.chunks = reinterpret_cast<std::uintptr_t>(chunk_pointers.get());

  WriteReadIoctl(fd_, DRM_AMDGPU_CS, cs_args);

  if (out_fence) {
    out_fence->hw_type = args.hw_type;
    out_fence->hw_instance = 0;
    out_fence->ring = args.ring;
    out_fence->context = args.context;
    out_fence->seq = cs_args.out.handle;
  }
}
}
