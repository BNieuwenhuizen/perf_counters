#include "native.hpp"

#include <cstring>
#include <memory>

static std::uint64_t AlignTo(std::uint64_t a, std::uint64_t b) {
  return (a + b - 1) / b * b;
}

NativeVirtualAllocation::NativeVirtualAllocation(
    boost::intrusive_ptr<NativeDevice> device, std::uint64_t size,
    std::uint64_t alignment)
    : device_{std::move(device)}, address_{0}, size_{size} {
  std::lock_guard<std::mutex> m{Device()->allocations_mutex_};
  size_ = AlignTo(size_, alignment);
  address_ = AlignTo(Device()->info_.virtual_address_start, alignment);
  auto eit = Device()->virtual_allocations_.end();
  for (auto it = Device()->virtual_allocations_.begin(); it != eit; ++it) {
    if (it->Address() - address_ >= size) {
      Device()->virtual_allocations_.insert(it, *this);
      return;
    }
    address_ = AlignTo(it->Address() + it->Size(), alignment);
  }
  if (Device()->info_.virtual_address_end - address_ < size_)
    std::terminate();
  Device()->virtual_allocations_.push_back(*this);
}

NativeVirtualAllocation::~NativeVirtualAllocation() {
  std::lock_guard<std::mutex> m{Device()->allocations_mutex_};
  Device()->virtual_allocations_.erase(
      Device()->virtual_allocations_.iterator_to(*this));
}

NativeBuffer::NativeBuffer(boost::intrusive_ptr<NativeDevice> device,
                           std::uint64_t size, std::uint64_t alignment,
                           drm::Domain domain, drm::BufferFlags flags)
    : NativeVirtualAllocation{std::move(device), size, alignment} {
  handle_ = Device()->drm_.CreateBuffer(Size(), alignment, domain, flags);
  Device()->drm_.MapBuffer(handle_, Address(), Size(), 0);
}

NativeBuffer::~NativeBuffer() {
  Device()->drm_.UnmapBuffer(handle_, Address(), Size(), 0);
  Device()->drm_.DestroyBuffer(handle_);
}

void *NativeBuffer::Map() {
  return Device()->drm_.MapBufferCpu(
      Handle(), Size(), drm::MapFlags::kRead | drm::MapFlags::kWrite);
}

NativeCommandBuffer::NativeCommandBuffer(
    boost::intrusive_ptr<NativeDevice> device, drm::HwType hw_type)
    : device_{std::move(device)}, hw_type_{hw_type} {}

void NativeCommandBuffer::StartRecording() {}

void NativeCommandBuffer::FinishRecording() {
  while (data_.empty() || (data_.size() & 7))
    data_.push_back(0xffff1000);
}

void NativeCommandBuffer::Reset() {
  data_.clear();
  accessed_buffers_.clear();
}

NativeDevice::NativeDevice(int fd) : drm_{fd} {
  info_ = drm_.GetInfo();
  context_ = drm_.CreateContext();
}

NativeDevice::~NativeDevice() { drm_.DestroyContext(context_); }

boost::intrusive_ptr<NativeDevice> NativeDevice::New(int fd) {
  return {new NativeDevice{fd}};
}

drm::Fence NativeDevice::Submit(NativeCommandBuffer *ncb) {
  NativeBuffer buffer{this, ncb->data_.size() * 4, 4096, drm::Domain::kGtt,
                      drm::BufferFlags::kCpuAccess};

  auto ptr = drm_.MapBufferCpu(buffer.Handle(), buffer.Size(),
                               drm::MapFlags::kRead | drm::MapFlags::kWrite);

  std::memcpy(ptr, ncb->data_.data(), ncb->data_.size() * 4);
  auto buffer_entries = std::make_unique<drm::BufferListEntry[]>(
      ncb->accessed_buffers_.size() + 1);
  auto buffer_entry = buffer_entries.get();

  for (auto e : ncb->accessed_buffers_) {
    buffer_entry->buffer = e.first;
    buffer_entry->priority = e.second;
    ++buffer_entry;
  }

  buffer_entry->buffer = buffer.Handle();
  buffer_entry->priority = 8;

  drm::SubmitArgs args;
  args.hw_type = ncb->hw_type_;
  args.ring = 0;
  args.context = context_;
  args.buffers = drm_.CreateBufferList(buffer_entries.get(),
                                       ncb->accessed_buffers_.size() + 1);

  drm::SubmitBuffer sb;
  sb.address = buffer.Address();
  sb.size = ncb->data_.size() * 4;

  args.command_buffers = &sb;
  args.command_buffer_count = 1;

  drm::Fence ret;
  drm_.Submit(args, &ret);
  drm_.DestroyBufferList(args.buffers);
  return ret;
}

double NativeDevice::ClockFrequency() const {
  return info_.clock_freq;
}
