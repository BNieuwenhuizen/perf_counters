#pragma once

#include <mutex>
#include <unordered_map>
#include <vector>

#include <boost/intrusive/list.hpp>
#include <boost/intrusive/list_hook.hpp>
#include <boost/smart_ptr/intrusive_ptr.hpp>
#include <boost/smart_ptr/intrusive_ref_counter.hpp>

#include "drm_device.hpp"

class NativeDevice;

class NativeVirtualAllocation : public boost::intrusive::list_base_hook<> {
public:
  NativeVirtualAllocation(boost::intrusive_ptr<NativeDevice> device,
                          std::uint64_t size, std::uint64_t alignment);
  NativeVirtualAllocation(NativeVirtualAllocation const &) = delete;
  NativeVirtualAllocation operator=(NativeVirtualAllocation const &) = delete;
  ~NativeVirtualAllocation() noexcept;

  NativeDevice *Device() noexcept { return device_.get(); }
  std::uint64_t Address() const noexcept { return address_; }
  std::uint64_t Size() const noexcept { return size_; }

private:
  boost::intrusive_ptr<NativeDevice> device_;
  std::uint64_t address_;
  std::uint64_t size_;
};

class NativeBuffer : public NativeVirtualAllocation {
public:
  NativeBuffer(boost::intrusive_ptr<NativeDevice> device, std::uint64_t size,
               std::uint64_t alignment, drm::Domain domain,
               drm::BufferFlags flags);
  NativeBuffer(NativeBuffer const &) = delete;
  NativeBuffer operator=(NativeBuffer const &) = delete;

  ~NativeBuffer() noexcept;

  drm::Buffer Handle() const noexcept { return handle_; }

  void *Map();

private:
  drm::Buffer handle_;
};

class NativeCommandBuffer {
public:
  NativeCommandBuffer(boost::intrusive_ptr<NativeDevice> device,
                      drm::HwType hw_type);

  void StartRecording();
  void FinishRecording();
  void Reset();

  void Emit(std::uint32_t v) { data_.push_back(v); }

  void AddBuffer(drm::Buffer b) { accessed_buffers_[b] = 8; }

private:
  boost::intrusive_ptr<NativeDevice> device_;
  drm::HwType hw_type_;
  std::vector<std::uint32_t> data_;
  std::unordered_map<drm::Buffer, unsigned> accessed_buffers_;

  friend class NativeDevice;
};

class NativeDevice final : public boost::intrusive_ref_counter<NativeDevice> {
public:
  ~NativeDevice() noexcept;

  static boost::intrusive_ptr<NativeDevice> New(int fd_);

  drm::Fence Submit(NativeCommandBuffer *ncb);

  double ClockFrequency() const;

private:
  explicit NativeDevice(int fd);
  drm::Device drm_;
  drm::DeviceInfo info_;
  drm::Context context_;

  boost::intrusive::list<NativeVirtualAllocation> virtual_allocations_;
  std::mutex allocations_mutex_;

  friend class NativeBuffer;
  friend class NativeVirtualAllocation;
};
