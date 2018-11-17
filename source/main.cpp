#include "drm_device.hpp"
#include "native.hpp"

#include <iostream>
#include <map>
#include <memory>
#include <set>

#include <fcntl.h>
#include <sys/stat.h>

#include "sid.h"

void EmitUConfigRegs(NativeCommandBuffer &cmd_buffer, std::uint32_t reg,
                     std::uint32_t count) {
  cmd_buffer.Emit(PKT3(PKT3_SET_UCONFIG_REG, count, 0));
  cmd_buffer.Emit((reg - CIK_UCONFIG_REG_OFFSET) >> 2);
}

struct SQPerfCounterInfo {
  unsigned selector;
};

std::map<std::string, SQPerfCounterInfo> const all_sq_perf_counters = {
    {"sq_cycles", {0x2}},
    {"sq_busy_cycles", {0x3}},
    {"sq_waves", {0x4}},
    {"sq_busy_cu_cycles", {0xd}},
    {"sq_items", {0xe}},
    {"sq_quads", {0xf}},
    {"sq_events", {0x10}},
    {"sq_surf_syncs", {0x11}},
    {"sq_insts", {0x19}},
    {"sq_insts_valu", {0x1a}},
    {"sq_insts_vmem_wr", {0x1b}},
    {"sq_insts_vmem_rd", {0x1c}},
    {"sq_insts_vmem", {0x1d}},
    {"sq_insts_salu", {0x1e}},
    {"sq_insts_smem", {0x1f}},
    {"sq_insts_flat", {0x20}},
    {"sq_insts_flat_lds_only", {0x21}},
    {"sq_insts_lds", {0x22}},
    {"sq_insts_gds", {0x23}},
    {"sq_insts_exp", {0x24}},
    {"sq_insts_exp_gds", {0x25}},
    {"sq_insts_branch", {0x26}},
    {"sq_insts_sendmsg", {0x27}},
    {"sq_insts_vskipped", {0x28}},
    {"sq_wait_cnt_vm", {0x30}},
    {"sq_wait_cnt_lgkm", {0x31}},
    {"sq_wait_cnt_exp", {0x32}},
    {"sq_wait_cnt_any", {0x33}},
    {"sq_wait_barrier", {0x34}},
    {"sq_wait_exp_alloc", {0x35}},
    {"sq_wait_sleep", {0x36}},
    {"sq_wait_other", {0x37}},
    {"sq_wait_any", {0x38}},
    {"sq_wait_ttrace", {0x39}},
    {"sq_wait_ifetch", {0x3a}},
    {"sq_wait_inst_vmem", {0x3b}},
    {"sq_wait_inst_sca", {0x3c}},
    {"sq_wait_inst_lds", {0x3d}},
    {"sq_wait_inst_valu", {0x3e}},
    {"sq_wait_inst_exp_gds", {0x3f}},
    {"thread_cycles_valu", {0x59}},
    {"thread_cycles_valu_max", {0x5a}},
    {"sq_valu_dep_stall", {0x67}},
    {"sq_valu_starve", {0x68}},
    {"sq_exp_req_fifo_full", {0x69}},
    {"sq_vmem_back2back_stall", {0x6d}},
    {"sq_vmem_ta_addr_fifo_full", {0x6e}},
    {"sq_vmem_ta_cmd_fifo_full", {0x6f}},
    {"sqc_tc_req", {0xae}},
    {"sqc_tc_data_read_req", {0xb0}},
    {"sqc_tc_stall", {0xb3}},
    {"sqc_tc_starve", {0xb4}},
    {"sq_dcache_req", {0xc7}},
    {"sq_dcache_hits", {0xc8}},

};

class CounterInterface {
public:
  virtual ~CounterInterface() {}
  virtual void Configure(NativeCommandBuffer& cb) const = 0;
  virtual void ExtractData(NativeCommandBuffer& cb, std::uint64_t va) const = 0;
  virtual std::size_t GetBufferSize() const = 0;
  virtual void PrintDifference(const void *old_data, const void* new_data, double time_delta, std::ostream& os) const = 0;
};

std::string HumanRate(double v) {
	if (v >= 1e18)
		return std::to_string(v / 1e18) + " E/s";
	if (v >= 1e15)
		return std::to_string(v / 1e15) + " P/s";
	if (v >= 1e12)
		return std::to_string(v / 1e12) + " T/s";
	if (v >= 1e9)
		return std::to_string(v / 1e9) + " G/s";
	else if (v >= 1e6)
		return std::to_string(v / 1e6) + " M/s";
	else if (v >= 1e3)
		return std::to_string(v / 1e3) + " K/s";
	else
		return std::to_string(v) + " 1/s";
}

class SQCounters  : public CounterInterface {
public:
  SQCounters(std::set<std::string>& counters) {
	  shader_mask = 0x7f;
	  for (auto e : all_sq_perf_counters) {
		  auto it = counters.find(e.first);
		  if (it != counters.end()) {
			  counters.erase(it);
			  counter_names.push_back(e.first);
			  counter_selectors.push_back(e.second);
		  }
	  }
  }

  void Configure(NativeCommandBuffer& cb) const override {
    if (counter_selectors.empty())
      return;

    EmitUConfigRegs(cb, R_036780_SQ_PERFCOUNTER_CTRL, 2);
    cb.Emit(shader_mask);
    cb.Emit(0xFFFFFFFFU);

    EmitUConfigRegs(cb, R_036700_SQ_PERFCOUNTER0_SELECT, counter_selectors.size());
    for (auto e : counter_selectors) {
      cb.Emit(S_036700_SQC_BANK_MASK(15) | S_036700_SQC_CLIENT_MASK(15) |
              S_036700_SIMD_MASK(15) |
              S_036700_PERF_SEL(e.selector));
    }
  }
  void ExtractData(NativeCommandBuffer& cb, std::uint64_t va) const override {
    for (int i = 0; i < 4; ++i) {
      EmitUConfigRegs(cb, R_030800_GRBM_GFX_INDEX, 1);
      cb.Emit(S_030800_SH_BROADCAST_WRITES(1) | S_030800_SE_INDEX(i) |
              S_030800_INSTANCE_BROADCAST_WRITES(1));
      for(unsigned j = 0; j < counter_names.size(); ++j) {
        auto dst_va = va + 32 * j + 8 * i;
        cb.Emit(PKT3(PKT3_COPY_DATA, 4, 0) | PKT3_SHADER_TYPE_S(1));
        cb.Emit(COPY_DATA_SRC_SEL(COPY_DATA_PERF) | COPY_DATA_DST_SEL(5));
        cb.Emit((R_034700_SQ_PERFCOUNTER0_LO + 8 * j) >> 2);
        cb.Emit(0); /* unused */
        cb.Emit(dst_va);
        cb.Emit(dst_va >> 32);

        cb.Emit(PKT3(PKT3_COPY_DATA, 4, 0) | PKT3_SHADER_TYPE_S(1));
        cb.Emit(COPY_DATA_SRC_SEL(COPY_DATA_PERF) | COPY_DATA_DST_SEL(5));
        cb.Emit((R_034700_SQ_PERFCOUNTER0_LO + 8 * j + 4) >> 2);
        cb.Emit(0); /* unused */
        cb.Emit((dst_va + 4));
        cb.Emit((dst_va + 4) >> 32);
      }
    }
  }
  std::size_t GetBufferSize() const override {
	  return 32 * counter_names.size();
  }
  void PrintDifference(const void *old_data, const void* new_data, double time_delta, std::ostream& os) const override {
    const std::uint64_t *old_counters = (const std::uint64_t*)old_data;
    const std::uint64_t *new_counters = (const std::uint64_t*)new_data;
    for(unsigned j = 0; j < counter_names.size(); ++j) {
      uint64_t cnt = 0;
      for (unsigned i = 0; i < 4; ++i) {
	      cnt += new_counters[4 * j + i] - old_counters[4 * j + i];
      }
      auto delta = cnt / time_delta;
      os << "   " << counter_names[j] << " : " << HumanRate(delta) << "\n";
    }
  }
private:
  uint32_t shader_mask;
  std::vector<std::string> counter_names;
  std::vector<SQPerfCounterInfo> counter_selectors;
};


static std::vector<std::pair<std::string, std::uint32_t>> all_spi_perf_counters = {
  {"spi_vs_window_valid", 0x0},
  {"spi_vs_busy", 0x1},
  {"spi_vs_first_wave", 0x2},
  {"spi_vs_last_wave", 0x3},
  {"spi_vs_lshs_dealloc", 0x4},
  {"spi_vs_pc_stall", 0x5},
  {"spi_vs_pos0_stall", 0x6},
  {"spi_vs_pos1_stall", 0x7},
  {"spi_vs_crawler_stall", 0x8},
  {"spi_vs_event_wave", 0x9},
  {"spi_vs_wave", 0xa},
  {"spi_vs_pers_upd_full0", 0xb},
  {"spi_vs_pers_upd_full1", 0xc},
  {"spi_vs_late_alloc_full", 0xd},
  {"spi_vs_first_subgrp", 0xe},
  {"spi_vs_last_subgrp", 0xf},
  {"spi_gs_window_valid", 0x10},
  {"spi_csg_window_valid", 0x39},
  {"spi_csg_busy", 0x3a},
  {"spi_csg_num_threadgroups", 0x3b},
  {"spi_csg_crawler_stall", 0x3c},
  {"spi_csg_event_wave", 0x3d},
  {"spi_csg_wave", 0x3e},
  {"spi_csn_window_valid", 0x3f},
  {"spi_csn_busy", 0x40},
  {"spi_csn_num_threadgroups", 0x41},
  {"spi_csn_crawler_stall", 0x42},
  {"spi_csn_event_wave", 0x43},
  {"spi_csn_wave", 0x44},
  {"spi_pix_alloc_scb_stall", 0x59},
  {"spi_pix_alloc_db0_stall", 0x5a},
  {"spi_pix_alloc_db1_stall", 0x5b},
  {"spi_pix_alloc_db2_stall", 0x5c},
  {"spi_pix_alloc_db3_stall", 0x5d},
};

class SPICounters  : public CounterInterface {
public:
  SPICounters(std::set<std::string>& counters) {
	  for (auto e : all_spi_perf_counters) {
		  auto it = counters.find(e.first);
		  if (it != counters.end()) {
			  counters.erase(it);
			  counter_names.push_back(e.first);
			  counter_selectors.push_back(e.second);
		  }
	  }
  }

  void Configure(NativeCommandBuffer& cb) const override {
    if (counter_selectors.empty())
      return;

    EmitUConfigRegs(cb, R_036600_SPI_PERFCOUNTER0_SELECT, counter_selectors.size());
    for (unsigned i = 0; i < 4 && i < counter_selectors.size(); ++i) {
      cb.Emit(counter_selectors[i]);
    }
    if (counter_selectors.size() > 4)
      EmitUConfigRegs(cb, R_036620_SPI_PERFCOUNTER4_SELECT, counter_selectors.size());
    for (unsigned i = 4; i < 6 && i < counter_selectors.size(); ++i) {
      cb.Emit(counter_selectors[i]);
    }
  }
  void ExtractData(NativeCommandBuffer& cb, std::uint64_t va) const override {
    if (counter_selectors.empty())
      return;
    for (int i = 0; i < 4; ++i) {
      EmitUConfigRegs(cb, R_030800_GRBM_GFX_INDEX, 1);
      cb.Emit(S_030800_SH_BROADCAST_WRITES(1) | S_030800_SE_INDEX(i) |
              S_030800_INSTANCE_BROADCAST_WRITES(1));
      for(unsigned j = 0; j < counter_names.size(); ++j) {
        auto dst_va = va + 32 * j + 8 * i;
        cb.Emit(PKT3(PKT3_COPY_DATA, 4, 0) | PKT3_SHADER_TYPE_S(1));
        cb.Emit(COPY_DATA_SRC_SEL(COPY_DATA_PERF) | COPY_DATA_DST_SEL(5));
        cb.Emit((R_034604_SPI_PERFCOUNTER0_LO + 8 * j) >> 2);
        cb.Emit(0); /* unused */
        cb.Emit(dst_va);
        cb.Emit(dst_va >> 32);

        cb.Emit(PKT3(PKT3_COPY_DATA, 4, 0) | PKT3_SHADER_TYPE_S(1));
        cb.Emit(COPY_DATA_SRC_SEL(COPY_DATA_PERF) | COPY_DATA_DST_SEL(5));
        cb.Emit((R_034600_SPI_PERFCOUNTER0_HI + 8 * j + 4) >> 2);
        cb.Emit(0); /* unused */
        cb.Emit((dst_va + 4));
        cb.Emit((dst_va + 4) >> 32);
      }
    }
  }
  std::size_t GetBufferSize() const override {
	  return 32 * counter_names.size();
  }
  void PrintDifference(const void *old_data, const void* new_data, double time_delta, std::ostream& os) const override {
    const std::uint64_t *old_counters = (const std::uint64_t*)old_data;
    const std::uint64_t *new_counters = (const std::uint64_t*)new_data;
    for(unsigned j = 0; j < counter_names.size(); ++j) {
      uint64_t cnt = 0;
      for (unsigned i = 0; i < 4; ++i) {
	      cnt += new_counters[4 * j + i] - old_counters[4 * j + i];
      }
      auto delta = cnt / time_delta;
      os << "   " << counter_names[j] << " : " << HumanRate(delta) << "\n";
    }
  }
private:
  std::vector<std::string> counter_names;
  std::vector<uint32_t> counter_selectors;
};

static std::vector<std::pair<std::string, std::uint32_t>> all_tcc_perf_counters = {
	{"tcc_cycle", {0x1}},
	{"tcc_busy", {0x2}},
	{"tcc_req", {0x3}},
	{"tcc_read", {0xf}},
	{"tcc_hit", {0x12}},
	{"tcc_miss", {0x13}},
	{"tcc_latency_fifo_full", {0x17}},
	{"tcc_src_fifo_full", {0x18}},
	{"tcc_hole_fifo_full", {0x19}},
	{"tcc_client0_req", {0x80}},
	{"tcc_client1_req", {0x81}},
	{"tcc_client127_req", {0xff}},
};


class TCCCounters  : public CounterInterface {
public:
  TCCCounters(std::set<std::string>& counters) {
	  for (auto e : all_tcc_perf_counters) {
		  auto it = counters.find(e.first);
		  if (it != counters.end()) {
			  counters.erase(it);
			  counter_names.push_back(e.first);
			  counter_selectors.push_back(e.second);
		  }
	  }
  }

  void Configure(NativeCommandBuffer& cb) const override {

    if (counter_selectors.empty())
      return;
    std::array<unsigned, 4> selectors = {
	    R_036E00_TCC_PERFCOUNTER0_SELECT,
	    R_036E08_TCC_PERFCOUNTER1_SELECT,
	    R_036E10_TCC_PERFCOUNTER2_SELECT,
	    R_036E14_TCC_PERFCOUNTER3_SELECT
    };

    for (unsigned j = 0; j < 16; ++j) {
      EmitUConfigRegs(cb, R_030800_GRBM_GFX_INDEX, 1);
      cb.Emit(S_030800_SH_BROADCAST_WRITES(1) | S_030800_INSTANCE_INDEX(j) |
              S_030800_SE_BROADCAST_WRITES(1));
      for (unsigned i = 0; i < counter_selectors.size(); ++i) {
        EmitUConfigRegs(cb, selectors[i], 1);
        cb.Emit(counter_selectors[i]);
      }
    }

    EmitUConfigRegs(cb, R_030800_GRBM_GFX_INDEX, 1);
    cb.Emit(S_030800_SH_BROADCAST_WRITES(1) | S_030800_SE_BROADCAST_WRITES(1) |
              S_030800_INSTANCE_BROADCAST_WRITES(1));

  }
  void ExtractData(NativeCommandBuffer& cb, std::uint64_t va) const override {
    if (counter_selectors.empty())
      return;

    for (int i = 0; i < 16; ++i) {
      EmitUConfigRegs(cb, R_030800_GRBM_GFX_INDEX, 1);
      cb.Emit(S_030800_SH_BROADCAST_WRITES(1) | S_030800_INSTANCE_INDEX(i) |
              S_030800_SE_BROADCAST_WRITES(1));
      for(unsigned j = 0; j < counter_names.size(); ++j) {
        auto dst_va = va + 128 * j + 8 * i;
        cb.Emit(PKT3(PKT3_COPY_DATA, 4, 0) | PKT3_SHADER_TYPE_S(1));
        cb.Emit(COPY_DATA_SRC_SEL(COPY_DATA_PERF) | COPY_DATA_DST_SEL(5));
        cb.Emit((R_034E00_TCC_PERFCOUNTER0_LO + 8 * j) >> 2);
        cb.Emit(0); /* unused */
        cb.Emit(dst_va);
        cb.Emit(dst_va >> 32);

        cb.Emit(PKT3(PKT3_COPY_DATA, 4, 0) | PKT3_SHADER_TYPE_S(1));
        cb.Emit(COPY_DATA_SRC_SEL(COPY_DATA_PERF) | COPY_DATA_DST_SEL(5));
        cb.Emit((R_034E04_TCC_PERFCOUNTER0_HI + 8 * j) >> 2);
        cb.Emit(0); /* unused */
        cb.Emit((dst_va + 4));
        cb.Emit((dst_va + 4) >> 32);
      }
    }
  }
  std::size_t GetBufferSize() const override {
	  return 128 * counter_names.size();
  }
  void PrintDifference(const void *old_data, const void* new_data, double time_delta, std::ostream& os) const override {
    const std::uint64_t *old_counters = (const std::uint64_t*)old_data;
    const std::uint64_t *new_counters = (const std::uint64_t*)new_data;
    for(unsigned j = 0; j < counter_names.size(); ++j) {
      double cnt = 0;
      for (unsigned i = 0; i < 16; ++i) {
	      cnt += new_counters[16 * j + i] - old_counters[16 * j + i];
      }
      auto delta = cnt / time_delta;
      os << "   " << counter_names[j] << " : " << HumanRate(delta) << "(";
      for (int i = 0; i < 16; ++i) {
	      if (i)
		      os << " ";
	      os << HumanRate((new_counters[16 * j + i] - old_counters[16 * j + i]) / time_delta);
      }
      os << ")\n";
    }
  }
private:
  std::vector<std::string> counter_names;
  std::vector<uint32_t> counter_selectors;
};

static std::vector<std::pair<std::string, std::uint32_t>> all_grbm_perf_counters = {
	{"grbm_count", {0x0}},
	{"grbm_gui_active", {0x2}},
	{"grbm_sc_busy", {0x9}},
	{"grbm_spi_busy", {0xb}},
	{"grbm_tc_busy", {0x1c}},
};


class GRBMCounters  : public CounterInterface {
public:
  GRBMCounters(std::set<std::string>& counters) {
	  for (auto e : all_grbm_perf_counters) {
		  auto it = counters.find(e.first);
		  if (it != counters.end()) {
			  counters.erase(it);
			  counter_names.push_back(e.first);
			  counter_selectors.push_back(e.second);
		  }
	  }
	  if (counter_selectors.size () > 2)
		  abort();
  }

  void Configure(NativeCommandBuffer& cb) const override {

    if (counter_selectors.empty())
      return;
    std::array<unsigned, 4> selectors = {
	    R_036100_GRBM_PERFCOUNTER0_SELECT,
	    R_036104_GRBM_PERFCOUNTER1_SELECT,
    };

    for (unsigned i = 0; i < counter_selectors.size(); ++i) {
      EmitUConfigRegs(cb, selectors[i], 1);
      cb.Emit(counter_selectors[i]);
    }

  }
  void ExtractData(NativeCommandBuffer& cb, std::uint64_t va) const override {
    if (counter_selectors.empty())
      return;

    const uint32_t regs[] = {
	    R_034100_GRBM_PERFCOUNTER0_LO,
	    R_03410C_GRBM_PERFCOUNTER1_LO
    };
    EmitUConfigRegs(cb, R_030800_GRBM_GFX_INDEX, 1);
    cb.Emit(S_030800_SH_BROADCAST_WRITES(1) | S_030800_INSTANCE_BROADCAST_WRITES(1) |
            S_030800_SE_BROADCAST_WRITES(1));
    for(unsigned j = 0; j < counter_names.size(); ++j) {
      auto dst_va = va + 8 * j;
      cb.Emit(PKT3(PKT3_COPY_DATA, 4, 0) | PKT3_SHADER_TYPE_S(1));
      cb.Emit(COPY_DATA_SRC_SEL(COPY_DATA_PERF) | COPY_DATA_DST_SEL(5));
      cb.Emit((regs[j]) >> 2);
      cb.Emit(0); /* unused */
      cb.Emit(dst_va);
      cb.Emit(dst_va >> 32);

      cb.Emit(PKT3(PKT3_COPY_DATA, 4, 0) | PKT3_SHADER_TYPE_S(1));
      cb.Emit(COPY_DATA_SRC_SEL(COPY_DATA_PERF) | COPY_DATA_DST_SEL(5));
      cb.Emit((regs[j] + 4) >> 2);
      cb.Emit(0); /* unused */
      cb.Emit((dst_va + 4));
      cb.Emit((dst_va + 4) >> 32);
    }
  }
  std::size_t GetBufferSize() const override {
	  return 8 * counter_names.size();
  }
  void PrintDifference(const void *old_data, const void* new_data, double time_delta, std::ostream& os) const override {
    const std::uint64_t *old_counters = (const std::uint64_t*)old_data;
    const std::uint64_t *new_counters = (const std::uint64_t*)new_data;
    for(unsigned j = 0; j < counter_names.size(); ++j) {
      double cnt = new_counters[j] - old_counters[j];
      auto delta = cnt / time_delta;
      os << "   " << counter_names[j] << " : " << HumanRate(delta) << "\n";
    }
  }
private:
  std::vector<std::string> counter_names;
  std::vector<uint32_t> counter_selectors;
};


struct CounterConfig {
  unsigned shader_mask = 0x7f;
  std::vector<std::string> counters;
};

#define EVENT_TYPE_SAMPLE_PIPELINESTAT 30
#define EVENT_TYPE_PERFCOUNTER_START 0x17
#define EVENT_TYPE_PERFCOUNTER_STOP 0x18
#define EVENT_TYPE_PERFCOUNTER_SAMPLE 0x1B

template <typename S, typename T>
std::size_t CountCounters(const S &counters, const T &set) {
  std::size_t ret = 0;
  for (auto &&e : counters)
    if (set.find(e) != set.end())
      ++ret;
  return ret;
}

template <typename D>
std::unique_ptr<NativeCommandBuffer>
BuildStartBuffer(D &&device, const CounterConfig &config, const std::vector<CounterInterface*>& counters) {
  auto ncb =
      std::make_unique<NativeCommandBuffer>(device, drm::HwType::kGfx);

  ncb->StartRecording();


  EmitUConfigRegs(*ncb, R_030800_GRBM_GFX_INDEX, 1);
  ncb->Emit(S_030800_SH_BROADCAST_WRITES(1) | S_030800_SE_BROADCAST_WRITES(1) |
            S_030800_INSTANCE_BROADCAST_WRITES(1));
  for(auto c : counters)
	  c->Configure(*ncb);

  EmitUConfigRegs(*ncb, R_036020_CP_PERFMON_CNTL, 1);
  ncb->Emit(S_036020_PERFMON_STATE(V_036020_DISABLE_AND_RESET));

  ncb->Emit(PKT3(PKT3_EVENT_WRITE, 0, 0));
  ncb->Emit(EVENT_TYPE(EVENT_TYPE_PERFCOUNTER_START) | EVENT_INDEX(0));


  EmitUConfigRegs(*ncb, R_036020_CP_PERFMON_CNTL, 1);
  ncb->Emit(S_036020_PERFMON_STATE(V_036020_START_COUNTING) |
            S_036020_PERFMON_SAMPLE_ENABLE(1));
 for(auto c : counters)
	  c->Configure(*ncb);
  ncb->FinishRecording();
  return ncb;
}

template <typename D>
std::unique_ptr<NativeCommandBuffer>
BuildSampleBuffer(D &&device, NativeBuffer &buffer,
                  const CounterConfig &config, const std::vector<CounterInterface*>& counters) {
  auto ncb =
      std::make_unique<NativeCommandBuffer>(device, drm::HwType::kGfx);
  ncb->StartRecording();

  EmitUConfigRegs(*ncb, R_036780_SQ_PERFCOUNTER_CTRL, 2);
  ncb->Emit(config.shader_mask);
  ncb->Emit(0xFFFFFFFFU);

  EmitUConfigRegs(*ncb, R_030800_GRBM_GFX_INDEX, 1);
  ncb->Emit(S_030800_SH_BROADCAST_WRITES(1) | S_030800_SE_BROADCAST_WRITES(1) |
            S_030800_INSTANCE_BROADCAST_WRITES(1));

  EmitUConfigRegs(*ncb, R_036020_CP_PERFMON_CNTL, 1);
  ncb->Emit(S_036020_PERFMON_STATE(V_036020_START_COUNTING) |
            S_036020_PERFMON_SAMPLE_ENABLE(1));

  ncb->Emit(PKT3(PKT3_EVENT_WRITE, 0, 0));
  ncb->Emit(EVENT_TYPE(EVENT_TYPE_PERFCOUNTER_SAMPLE) | EVENT_INDEX(0));

  ncb->AddBuffer(buffer.Handle());
  uint64_t va = buffer.Address();
  for(auto c : counters) {
	  c->ExtractData(*ncb, va);
	  va += c->GetBufferSize();
  }

  EmitUConfigRegs(*ncb, R_030800_GRBM_GFX_INDEX, 1);
  ncb->Emit(S_030800_SH_BROADCAST_WRITES(1) | S_030800_SE_BROADCAST_WRITES(1) |
            S_030800_INSTANCE_BROADCAST_WRITES(1));
  ncb->FinishRecording();
  return ncb;
}

int main(int argc, char **argv) {
  auto device =
      NativeDevice::New(open("/dev/dri/renderD129", O_RDWR | O_CLOEXEC));



  CounterConfig cfg;
  std::set<std::string> counter_args;
  for (int i = 1; i < argc; ++i) {
    cfg.counters.push_back(argv[i]);
    counter_args.insert(argv[i]);
  }
  SQCounters sq_counters(counter_args);
  SPICounters spi_counters(counter_args);
  TCCCounters tcc_counters(counter_args);
  GRBMCounters grbm_counters(counter_args);
  std::vector<CounterInterface*> counters;
  counters.push_back(&sq_counters);
  counters.push_back(&spi_counters);
  counters.push_back(&tcc_counters);
  counters.push_back(&grbm_counters);

  uint64_t size = 0;
  for(auto c : counters)
	  size += c->GetBufferSize();
  size = (size + 4095) & ~4095;
  if (size == 0)
	  abort();
  NativeBuffer buffer{device, size, 4096, drm::Domain::kGtt,
                      drm::BufferFlags::kCpuAccess};
  NativeBuffer buffer2{device, size, 4096, drm::Domain::kGtt,
                      drm::BufferFlags::kCpuAccess};

  auto start_cb = BuildStartBuffer(device, cfg, counters);
  auto sample_cb = BuildSampleBuffer(device, buffer, cfg, counters);
  auto sample_cb2 = BuildSampleBuffer(device, buffer2, cfg, counters);

  device->Submit(start_cb.get());
  std::uint64_t *p = reinterpret_cast<std::uint64_t *>(buffer.Map());
  std::uint64_t *p2 = reinterpret_cast<std::uint64_t *>(buffer2.Map());
  uint64_t sq_prev[16] = {};
  for (unsigned long long iter = 0;; ++iter) {
    auto fence = device->Submit((iter & 1) ? sample_cb2.get() : sample_cb.get());
    std::cout << "results:\n";

    sleep(1);
    auto new_p = (iter & 1) ? p2 : p;
    auto old_p = (iter & 1) ? p : p2;
    for (auto c : counters) {
	    c->PrintDifference(old_p, new_p, 1.0, std::cout);
	    new_p += c->GetBufferSize() / 8;
	    old_p += c->GetBufferSize() / 8;
    }

  }

  return 0;
}
