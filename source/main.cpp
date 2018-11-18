#include "drm_device.hpp"
#include "native.hpp"

#include <iostream>
#include <map>
#include <memory>
#include <set>

#include <fcntl.h>
#include <sys/stat.h>

#include "sid.h"


enum class Block {
	cpg,
	grbm,
	pa_su,
	sq,
	sx,
	ta,
	td,
	tcp,
	tcc,
};

enum BlockFlag {
	BLOCK_NONE = 0,
	BLOCK_ENGINE_INDEXED = 1,
	BLOCK_INSTANCE_INDEXED = 2,
};


enum class CounterType {
	rate,
	percentage,
};

const unsigned maxCountersPerBlock = 16;
struct BlockInfo {
	Block block;
	unsigned reg_count;
	unsigned flags;
	unsigned sel_regs[maxCountersPerBlock];
	unsigned result_regs[maxCountersPerBlock][2];
};

struct HWCounterEntry {
	Block block;
	unsigned idx;
	bool accum_prev;
};

bool operator<(const HWCounterEntry& a, const HWCounterEntry& b) {
	if (a.block != b.block)
		return static_cast<unsigned>(a.block) < static_cast<unsigned>(b.block);
	if (a.idx != b.idx)
		return a.idx < b.idx;
	return a.accum_prev < b.accum_prev;
}

const BlockInfo gfx9_blocks[] = {
	{Block::cpg, 2, BLOCK_NONE, {
		R_036008_CPG_PERFCOUNTER0_SELECT,
		R_036000_CPG_PERFCOUNTER1_SELECT,
	}, {
		{R_034008_CPG_PERFCOUNTER0_LO, R_03400C_CPG_PERFCOUNTER0_HI},
		{R_034000_CPG_PERFCOUNTER1_LO, R_034004_CPG_PERFCOUNTER1_HI},
	}},
	{Block::grbm, 2, BLOCK_NONE, {
		R_036100_GRBM_PERFCOUNTER0_SELECT,
		R_036104_GRBM_PERFCOUNTER1_SELECT,
	}, {
		{R_034100_GRBM_PERFCOUNTER0_LO, R_034104_GRBM_PERFCOUNTER0_HI},
		{R_03410C_GRBM_PERFCOUNTER1_LO, R_034110_GRBM_PERFCOUNTER1_HI},
	}},
	{Block::pa_su, 4, BLOCK_ENGINE_INDEXED, {
		R_036400_PA_SU_PERFCOUNTER0_SELECT,
		R_036408_PA_SU_PERFCOUNTER1_SELECT,
		R_036410_PA_SU_PERFCOUNTER2_SELECT,
		R_036414_PA_SU_PERFCOUNTER3_SELECT,
	}, {
		{R_034400_PA_SU_PERFCOUNTER0_LO, R_034404_PA_SU_PERFCOUNTER0_HI},
		{R_034408_PA_SU_PERFCOUNTER1_LO, R_03440C_PA_SU_PERFCOUNTER1_HI},
		{R_034410_PA_SU_PERFCOUNTER2_LO, R_034414_PA_SU_PERFCOUNTER2_HI},
		{R_034418_PA_SU_PERFCOUNTER3_LO, R_03441C_PA_SU_PERFCOUNTER3_HI},
	}},
	{Block::sq, 16, BLOCK_ENGINE_INDEXED, {
		R_036700_SQ_PERFCOUNTER0_SELECT,
		R_036704_SQ_PERFCOUNTER1_SELECT,
		R_036708_SQ_PERFCOUNTER2_SELECT,
		R_03670C_SQ_PERFCOUNTER3_SELECT,
		R_036710_SQ_PERFCOUNTER4_SELECT,
		R_036714_SQ_PERFCOUNTER5_SELECT,
		R_036718_SQ_PERFCOUNTER6_SELECT,
		R_03671C_SQ_PERFCOUNTER7_SELECT,
		R_036720_SQ_PERFCOUNTER8_SELECT,
		R_036724_SQ_PERFCOUNTER9_SELECT,
		R_036728_SQ_PERFCOUNTER10_SELECT,
		R_03672C_SQ_PERFCOUNTER11_SELECT,
		R_036730_SQ_PERFCOUNTER12_SELECT,
		R_036734_SQ_PERFCOUNTER13_SELECT,
		R_036738_SQ_PERFCOUNTER14_SELECT,
		R_03673C_SQ_PERFCOUNTER15_SELECT,
	}, {
		{R_034700_SQ_PERFCOUNTER0_LO, R_034704_SQ_PERFCOUNTER0_HI},
		{R_034708_SQ_PERFCOUNTER1_LO, R_03470C_SQ_PERFCOUNTER1_HI},
		{R_034710_SQ_PERFCOUNTER2_LO, R_034714_SQ_PERFCOUNTER2_HI},
		{R_034718_SQ_PERFCOUNTER3_LO, R_03471C_SQ_PERFCOUNTER3_HI},
		{R_034720_SQ_PERFCOUNTER4_LO, R_034724_SQ_PERFCOUNTER4_HI},
		{R_034728_SQ_PERFCOUNTER5_LO, R_03472C_SQ_PERFCOUNTER5_HI},
		{R_034730_SQ_PERFCOUNTER6_LO, R_034734_SQ_PERFCOUNTER6_HI},
		{R_034738_SQ_PERFCOUNTER7_LO, R_03473C_SQ_PERFCOUNTER7_HI},
		{R_034740_SQ_PERFCOUNTER8_LO, R_034744_SQ_PERFCOUNTER8_HI},
		{R_034748_SQ_PERFCOUNTER9_LO, R_03474C_SQ_PERFCOUNTER9_HI},
		{R_034750_SQ_PERFCOUNTER10_LO, R_034754_SQ_PERFCOUNTER10_HI},
		{R_034758_SQ_PERFCOUNTER11_LO, R_03475C_SQ_PERFCOUNTER11_HI},
		{R_034760_SQ_PERFCOUNTER12_LO, R_034764_SQ_PERFCOUNTER12_HI},
		{R_034768_SQ_PERFCOUNTER13_LO, R_03476C_SQ_PERFCOUNTER13_HI},
		{R_034770_SQ_PERFCOUNTER14_LO, R_034774_SQ_PERFCOUNTER14_HI},
		{R_034778_SQ_PERFCOUNTER15_LO, R_03477C_SQ_PERFCOUNTER15_HI},
	}},
	{Block::sx, 4, BLOCK_ENGINE_INDEXED, {
		R_036900_SX_PERFCOUNTER0_SELECT,
		R_036904_SX_PERFCOUNTER1_SELECT,
		R_036908_SX_PERFCOUNTER2_SELECT,
		R_03690C_SX_PERFCOUNTER3_SELECT,
	}, {
		{R_034900_SX_PERFCOUNTER0_LO, R_034904_SX_PERFCOUNTER0_HI},
		{R_034908_SX_PERFCOUNTER1_LO, R_03490C_SX_PERFCOUNTER1_HI},
		{R_034910_SX_PERFCOUNTER2_LO, R_034914_SX_PERFCOUNTER2_HI},
		{R_034918_SX_PERFCOUNTER3_LO, R_03491C_SX_PERFCOUNTER3_HI},
	}},
	{Block::ta, 2, BLOCK_ENGINE_INDEXED | BLOCK_INSTANCE_INDEXED, {
		R_036B00_TA_PERFCOUNTER0_SELECT,
		R_036B08_TA_PERFCOUNTER1_SELECT,
	}, {
		{R_034B00_TA_PERFCOUNTER0_LO, R_034B04_TA_PERFCOUNTER0_HI},
		{R_034B08_TA_PERFCOUNTER1_LO, R_034B0C_TA_PERFCOUNTER1_HI},
	}},
	{Block::tcp, 4, BLOCK_INSTANCE_INDEXED, {
		R_036D00_TCP_PERFCOUNTER0_SELECT,
		R_036D08_TCP_PERFCOUNTER1_SELECT,
		R_036D10_TCP_PERFCOUNTER2_SELECT,
		R_036D14_TCP_PERFCOUNTER3_SELECT,
	}, {
		{R_034D00_TCP_PERFCOUNTER0_LO, R_034D04_TCP_PERFCOUNTER0_HI},
		{R_034D08_TCP_PERFCOUNTER1_LO, R_034D0C_TCP_PERFCOUNTER1_HI},
		{R_034D10_TCP_PERFCOUNTER2_LO, R_034D14_TCP_PERFCOUNTER2_HI},
		{R_034D18_TCP_PERFCOUNTER3_LO, R_034D1C_TCP_PERFCOUNTER3_HI},
	}},
	{Block::tcc, 4, BLOCK_INSTANCE_INDEXED, {
		R_036E00_TCC_PERFCOUNTER0_SELECT,
		R_036E08_TCC_PERFCOUNTER1_SELECT,
		R_036E10_TCC_PERFCOUNTER2_SELECT,
		R_036E14_TCC_PERFCOUNTER3_SELECT,
	}, {
		{R_034E00_TCC_PERFCOUNTER0_LO, R_034E04_TCC_PERFCOUNTER0_HI},
		{R_034E08_TCC_PERFCOUNTER1_LO, R_034E0C_TCC_PERFCOUNTER1_HI},
		{R_034E10_TCC_PERFCOUNTER2_LO, R_034E14_TCC_PERFCOUNTER2_HI},
		{R_034E18_TCC_PERFCOUNTER3_LO, R_034E1C_TCC_PERFCOUNTER3_HI},
	}},
};

const BlockInfo& find_block(Block b)
{
	for(const auto& block : gfx9_blocks)
		if (block.block == b)
			return block;
}

void insert_config_reg(unsigned reg, std::uint32_t value, unsigned flags, 
		       std::map<std::uint32_t, std::map<unsigned, std::uint32_t>>& reg_config)
{
	for(unsigned se = 0; se < ((flags & BLOCK_ENGINE_INDEXED) ? 4 : 1); ++se) {
		for(unsigned instance = 0; instance < ((flags & BLOCK_INSTANCE_INDEXED) ? 16 : 1); ++instance) {
			unsigned grbm_config = S_030800_SH_BROADCAST_WRITES(1);
			grbm_config = (flags & BLOCK_ENGINE_INDEXED) ? S_030800_SE_INDEX(se) : S_030800_SE_BROADCAST_WRITES(1);
			grbm_config = (flags & BLOCK_INSTANCE_INDEXED) ? S_030800_INSTANCE_INDEX(instance) : S_030800_INSTANCE_BROADCAST_WRITES(1);
			reg_config[grbm_config][reg] = value;
		}
	}
}

void extract_sample_values(unsigned reg, std::uint32_t offset, unsigned flags,
			   std::map<std::uint32_t, std::map<unsigned, std::uint32_t>>& sample_map)
{
	for(unsigned se = 0; se < ((flags & BLOCK_ENGINE_INDEXED) ? 4 : 1); ++se) {
		for(unsigned instance = 0; instance < ((flags & BLOCK_INSTANCE_INDEXED) ? 16 : 1); ++instance) {
			unsigned grbm_config = S_030800_SH_BROADCAST_WRITES(1);
			grbm_config = (flags & BLOCK_ENGINE_INDEXED) ? S_030800_SE_INDEX(se) : S_030800_SE_BROADCAST_WRITES(1);
			grbm_config = (flags & BLOCK_INSTANCE_INDEXED) ? S_030800_INSTANCE_INDEX(instance) : S_030800_INSTANCE_BROADCAST_WRITES(1);
			sample_map[grbm_config][reg] = offset;
			offset += 8;
		}
	}
}

void ComputeHwConfiguration(const std::set<HWCounterEntry>& entries,
			    std::map<std::uint32_t, std::map<unsigned, std::uint32_t>>& reg_config,
			    std::map<std::uint32_t, std::map<unsigned, std::uint32_t>>& sample_map,
			    std::map<HWCounterEntry, std::pair<unsigned, unsigned>>& entry_map,
			    uint64_t& size)
{
	std::map<Block, unsigned> block_count;
	unsigned offset = 0;
	for(auto entry : entries) {
		const BlockInfo& info = find_block(entry.block);
		unsigned& block_idx = block_count[entry.block];
		unsigned value_count = ((info.flags & BLOCK_ENGINE_INDEXED) ? 4 : 1) * ((info.flags & BLOCK_INSTANCE_INDEXED) ? 16 : 1);

		unsigned extra_val = 0;
		if (entry.block == Block::sq) {
			extra_val = S_036700_SQC_BANK_MASK(15) | S_036700_SQC_CLIENT_MASK(15) |
				S_036700_SIMD_MASK(15);
		}
		

		if (block_idx >= info.reg_count)
			abort();
		insert_config_reg(info.sel_regs[block_idx], entry.idx | extra_val, info.flags, reg_config);
		if (entry.accum_prev) {
			++block_idx;
			if (block_idx >= info.reg_count)
				abort();
			insert_config_reg(info.sel_regs[block_idx], 1 | extra_val, info.flags, reg_config);
		}

		entry_map[entry] = {offset, value_count};

		extract_sample_values(info.result_regs[block_idx][0], offset * 8, info.flags, sample_map);
		extract_sample_values(info.result_regs[block_idx][1], offset * 8 + 4, info.flags, sample_map);
		++block_idx;
		offset += value_count;
	}

	size = offset * 8;
}

void EmitUConfigRegs(NativeCommandBuffer &cmd_buffer, std::uint32_t reg,
                     std::uint32_t count) {
  cmd_buffer.Emit(PKT3(PKT3_SET_UCONFIG_REG, count, 0));
  cmd_buffer.Emit((reg - CIK_UCONFIG_REG_OFFSET) >> 2);
}


#define EVENT_TYPE_SAMPLE_PIPELINESTAT 30
#define EVENT_TYPE_PERFCOUNTER_START 0x17
#define EVENT_TYPE_PERFCOUNTER_STOP 0x18
#define EVENT_TYPE_PERFCOUNTER_SAMPLE 0x1B


template <typename D>
std::unique_ptr<NativeCommandBuffer>
BuildStartBuffer(D &&device, const std::map<std::uint32_t, std::map<unsigned, std::uint32_t>>& reg_config) {
  auto ncb =
      std::make_unique<NativeCommandBuffer>(device, drm::HwType::kGfx);

  ncb->StartRecording();


  EmitUConfigRegs(*ncb, R_030800_GRBM_GFX_INDEX, 1);
  ncb->Emit(S_030800_SH_BROADCAST_WRITES(1) | S_030800_SE_BROADCAST_WRITES(1) |
            S_030800_INSTANCE_BROADCAST_WRITES(1));

  for (auto&& entry : reg_config) {
	EmitUConfigRegs(*ncb, R_030800_GRBM_GFX_INDEX, 1);
	ncb->Emit(entry.first);

	for (auto b = entry.second.begin(); b != entry.second.end(); ) {
		auto e = b;
		unsigned next_addr = b->first + 4;
		++e;
		while (e != entry.second.end() && e->first == next_addr) {
			++e;
			next_addr += 4;
		}

		EmitUConfigRegs(*ncb, b->first, std::distance(b, e));
		for (auto it = b; it != e; ++it)
			ncb->Emit(it->second);
		b = e;
	}
  }

  EmitUConfigRegs(*ncb, R_030800_GRBM_GFX_INDEX, 1);
  ncb->Emit(S_030800_SH_BROADCAST_WRITES(1) | S_030800_SE_BROADCAST_WRITES(1) |
            S_030800_INSTANCE_BROADCAST_WRITES(1));

  EmitUConfigRegs(*ncb, R_036020_CP_PERFMON_CNTL, 1);
  ncb->Emit(S_036020_PERFMON_STATE(V_036020_DISABLE_AND_RESET));

  ncb->Emit(PKT3(PKT3_EVENT_WRITE, 0, 0));
  ncb->Emit(EVENT_TYPE(EVENT_TYPE_PERFCOUNTER_START) | EVENT_INDEX(0));


  EmitUConfigRegs(*ncb, R_036020_CP_PERFMON_CNTL, 1);
  ncb->Emit(S_036020_PERFMON_STATE(V_036020_START_COUNTING) |
            S_036020_PERFMON_SAMPLE_ENABLE(1));

  for (auto&& entry : reg_config) {
	EmitUConfigRegs(*ncb, R_030800_GRBM_GFX_INDEX, 1);
	ncb->Emit(entry.first);

	for (auto b = entry.second.begin(); b != entry.second.end(); ) {
		auto e = b;
		unsigned next_addr = b->first + 4;
		++e;
		while (e != entry.second.end() && e->first == next_addr) {
			++e;
			next_addr += 4;
		}

		EmitUConfigRegs(*ncb, b->first, std::distance(b, e));
		for (auto it = b; it != e; ++it)
			ncb->Emit(it->second);
		b = e;
	}
  }

  EmitUConfigRegs(*ncb, R_030800_GRBM_GFX_INDEX, 1);
  ncb->Emit(S_030800_SH_BROADCAST_WRITES(1) | S_030800_SE_BROADCAST_WRITES(1) |
            S_030800_INSTANCE_BROADCAST_WRITES(1));

  ncb->FinishRecording();
  return ncb;
}

template <typename D>
std::unique_ptr<NativeCommandBuffer>
BuildSampleBuffer(D &&device, NativeBuffer &buffer,
                  const std::map<std::uint32_t, std::map<unsigned, std::uint32_t>>& sample_map) {
  auto ncb =
      std::make_unique<NativeCommandBuffer>(device, drm::HwType::kGfx);
  ncb->StartRecording();

  EmitUConfigRegs(*ncb, R_036780_SQ_PERFCOUNTER_CTRL, 2);
  ncb->Emit(0x7f);
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
  for(auto&& entry : sample_map) {
	EmitUConfigRegs(*ncb, R_030800_GRBM_GFX_INDEX, 1);
	ncb->Emit(entry.first);

	for (auto e : entry.second) {
		auto dst_va = va + e.second;
		ncb->Emit(PKT3(PKT3_COPY_DATA, 4, 0) | PKT3_SHADER_TYPE_S(1));
		ncb->Emit(COPY_DATA_SRC_SEL(COPY_DATA_PERF) | COPY_DATA_DST_SEL(5));
		ncb->Emit(e.first >> 2);
		ncb->Emit(0); /* unused */
		ncb->Emit(dst_va);
		ncb->Emit(dst_va >> 32);
	}
  }

  EmitUConfigRegs(*ncb, R_030800_GRBM_GFX_INDEX, 1);
  ncb->Emit(S_030800_SH_BROADCAST_WRITES(1) | S_030800_SE_BROADCAST_WRITES(1) |
            S_030800_INSTANCE_BROADCAST_WRITES(1));
  ncb->FinishRecording();
  return ncb;
}

class CounterInterface  {
public:
	CounterInterface(const std::string& name, const std::string& description, CounterType type) : name_{name}, description_{description}, type_{type} {}
	virtual ~CounterInterface();

	std::string const& name() const { return name_; }
	std::string const& description() const { return description_; }
	CounterType type() const { return type_; }

	virtual void register_counters(std::set<HWCounterEntry>& entries) const = 0;
	virtual double get_results(std::map<HWCounterEntry, std::pair<unsigned, unsigned>> const& entry_map, const uint64_t *old_p, const uint64_t *new_p) const = 0;
private:
	std::string name_;
	std::string description_;
	CounterType type_;
};

CounterInterface::~CounterInterface() {}

class SumCounter : public CounterInterface{
public:
	SumCounter(const std::string& name, const std::string& description, HWCounterEntry entry) : CounterInterface(name, description, CounterType::rate), entry_{entry} {}

	void register_counters(std::set<HWCounterEntry>& entries) const
	{
		entries.insert(entry_);
	}

	double get_results(std::map<HWCounterEntry, std::pair<unsigned, unsigned>> const& entry_map, const uint64_t *old_p, const uint64_t *new_p) const
	{
		auto it = entry_map.find(entry_);
		assert(it != entry_map.end());

		double v = 0;
		for(unsigned i = 0; i < it->second.second; ++i) {
			double v2 = new_p[it->second.first + i] - old_p[it->second.first + i];
			v += v2;
		}
		return v;
	}
private:
	HWCounterEntry entry_;
};

class MaxCounter : public CounterInterface{
public:
	MaxCounter(const std::string& name, const std::string& description, HWCounterEntry entry) : CounterInterface(name, description, CounterType::rate), entry_{entry} {}

	void register_counters(std::set<HWCounterEntry>& entries) const
	{
		entries.insert(entry_);
	}

	double get_results(std::map<HWCounterEntry, std::pair<unsigned, unsigned>> const& entry_map, const uint64_t *old_p, const uint64_t *new_p) const
	{
		auto it = entry_map.find(entry_);
		assert(it != entry_map.end());

		double v = 0;
		for(unsigned i = 0; i < it->second.second; ++i) {
			double v2 = new_p[it->second.first + i] - old_p[it->second.first + i];
			v = std::max(v, v2);
		}
		return v;
	}
private:
	HWCounterEntry entry_;
};

class DivMaxCounter : public CounterInterface{
public:
	DivMaxCounter(const std::string& name, const std::string& description, HWCounterEntry a, HWCounterEntry b) : CounterInterface(name, description, CounterType::percentage), a_{a}, b_{b} {}

	void register_counters(std::set<HWCounterEntry>& entries) const
	{
		entries.insert(a_);
		entries.insert(b_);
	}

	double get_results(std::map<HWCounterEntry, std::pair<unsigned, unsigned>> const& entry_map, const uint64_t *old_p, const uint64_t *new_p) const
	{
		auto it = entry_map.find(a_);
		assert(it != entry_map.end());

		double a = 0;
		for(unsigned i = 0; i < it->second.second; ++i) {
			double v2 = new_p[it->second.first + i] - old_p[it->second.first + i];
			a = std::max(a, v2);
		}

		it = entry_map.find(b_);
		assert(it != entry_map.end());

		double b = 0;
		for(unsigned i = 0; i < it->second.second; ++i) {
			double v2 = new_p[it->second.first + i] - old_p[it->second.first + i];
			b = std::max(b, v2);
		}
		return a / std::max(b, 1.0) * 100;
	}
private:
	HWCounterEntry a_, b_;
};

class DivSumCounter : public CounterInterface{
public:
	DivSumCounter(const std::string& name, const std::string& description, HWCounterEntry a, HWCounterEntry b, double extra_factor = 1.0) : CounterInterface(name, description, CounterType::percentage), a_{a}, b_{b}, extra_factor_{extra_factor} {}

	void register_counters(std::set<HWCounterEntry>& entries) const
	{
		entries.insert(a_);
		entries.insert(b_);
	}

	double get_results(std::map<HWCounterEntry, std::pair<unsigned, unsigned>> const& entry_map, const uint64_t *old_p, const uint64_t *new_p) const
	{
		auto it = entry_map.find(a_);
		assert(it != entry_map.end());

		double a = 0;
		for(unsigned i = 0; i < it->second.second; ++i) {
			double v2 = new_p[it->second.first + i] - old_p[it->second.first + i];
			a += v2;
		}

		it = entry_map.find(b_);
		assert(it != entry_map.end());

		double b = 0;
		for(unsigned i = 0; i < it->second.second; ++i) {
			double v2 = new_p[it->second.first + i] - old_p[it->second.first + i];
			b += v2;
		}
		return a / std::max(b, 1.0) * 100 * extra_factor_;
	}
private:
	HWCounterEntry a_, b_;
	double extra_factor_;
};


CounterInterface *counter_list[] = {
	new MaxCounter("gui_active", "Amount of cycles the GPU is busy.", {Block::grbm, 2, false}),
	new MaxCounter("sq_busy_cycles", "Amount of cycles the SQ unit is busy.", {Block::sq, 3, false}),
	new DivMaxCounter("mem_unit_busy", "The percentage of the time the memory unit is busy, including stalls.", {Block::ta, 0xf, false}, {Block::grbm, 2, false}),
	new DivMaxCounter("mem_unit_stalled", "The percentage of the time the memory unit is stalled.", {Block::tcp, 0x6, false}, {Block::grbm, 2, false}),
	new DivMaxCounter("pixel_export_stalled", "The percentage of the time pixel export are stalled on something downstream (late Z, color buffer, ...).", {Block::sx, 0xe, false}, {Block::grbm, 2, false}), // Hack, need 4 counters for full view
	new DivMaxCounter("primitive_assembly_stalled_on_rasterizer", "The percentage of the time primitive assembly is stalled on the rasterizer.", {Block::pa_su, 0x6d, false}, {Block::grbm, 2, false}),
	new DivMaxCounter("me_stalled_on_free_gfx_context", "The percentage of the time the ME is waiting for an available context for a context roll.", {Block::cpg, 0x19, false}, {Block::grbm, 2, false}),
	new DivMaxCounter("me_stalled_on_partial_flush", "The percentage of the time the Me is waiting for a partial flush.", {Block::cpg, 0x1c, false}, {Block::grbm, 2, false}),
	new DivSumCounter("cu_busy", "The percentage of the time the CUs are busy.", {Block::sq, 0xd, false}, {Block::grbm, 2, false}, 4.0/64.0),
	new DivSumCounter("valu_busy", "The percentage of CU-time the VALU is busy.", {Block::sq, 0x47, false}, {Block::sq, 0xd, false}),
	new DivSumCounter("exp_busy", "The percentage of CU-time the export units are busy", {Block::sq, 0x49, false}, {Block::sq, 0xd, false}),
	new DivSumCounter("lds_bank_conflict", "The percentage of CU-time there is a LDS bank conflict.", {Block::sq, 0x61, false}, {Block::sq, 0xd, false}),
	new DivSumCounter("lds_addr_conflict", "The percentage of CU-time there is a LDS address conflict.", {Block::sq, 0x62, false}, {Block::sq, 0xd, false}),
	new DivSumCounter("valu_starve", "The percentage of CU-time the VALU units are idle with waves running.", {Block::sq, 0x64, false}, {Block::sq, 0xd, false}),
	new DivSumCounter("salu_busy", "The percentage of CU-time the SALU units are busy.", {Block::sq, 0x54, false}, {Block::sq, 0xd, false}),
	new DivSumCounter("wave_ready", "The percentage of wave-time there is an instruction ready to issue.", {Block::sq, 0x2f, false}, {Block::sq, 0x2e, false}),
	new DivSumCounter("wave_wait_cnt_vmem", "The percentage of wave-time spent waiting on the VMEM counter.", {Block::sq, 0x30, false}, {Block::sq, 0x2e, false}),
	new DivSumCounter("wave_wait_cnt_any", "The percentage of wave-time spent waiting on any of the counters.", {Block::sq, 0x33, false}, {Block::sq, 0x2e, false}),
	new DivSumCounter("wave_wait_barrier", "The percentage of wave-time spent waiting on a barrier.", {Block::sq, 0x34, false}, {Block::sq, 0x2e, false}),
	new DivSumCounter("wave_wait_exp_alloc", "The percentage of wave-time waiting on space in the export FIFO to be available.", {Block::sq, 0x35, false}, {Block::sq, 0x2e, false}),
	new DivSumCounter("wave_wait_other", "The percentage of wave-time spent waiting on dependency stalls.", {Block::sq, 0x38, false}, {Block::sq, 0x2e, false}),
	new DivSumCounter("wave_wait_any", "The percentage of wave-time spent waiting.", {Block::sq, 0x39, false}, {Block::sq, 0x2e, false}),
	new DivSumCounter("wave_wait_ifetch", "The percentage of wave-time spent waiting on an instruction fetch.", {Block::sq, 0x3b, false}, {Block::sq, 0x2e, false}),
	new DivSumCounter("wave_wait_inst_any", "The percentage of wave-time spent waiting on an instruction issue slot.", {Block::sq, 0x3c, false}, {Block::sq, 0x2e, false}),
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


int main(int argc, char **argv) {
  auto device =
      NativeDevice::New(open("/dev/dri/renderD129", O_RDWR | O_CLOEXEC));

  std::vector<CounterInterface*> counters;
  for (int i = 1; i < argc; ++i) {
    for (auto e : counter_list)
	    if (e->name() == argv[i])
		    counters.push_back(e);
  }

  std::set<HWCounterEntry> entries;
  std::map<std::uint32_t, std::map<unsigned, std::uint32_t>> reg_config;
  std::map<std::uint32_t, std::map<unsigned, std::uint32_t>> sample_map;
  std::map<HWCounterEntry, std::pair<unsigned, unsigned>> entry_map;
  uint64_t size = 0;

  for(auto e : counters)
	  e->register_counters(entries);

  ComputeHwConfiguration(entries, reg_config, sample_map, entry_map, size);
  size = (size + 4095) & ~4095;
  if (size == 0) {
	  size = 4096;
  }
  NativeBuffer buffer{device, size, 4096, drm::Domain::kGtt,
                      drm::BufferFlags::kCpuAccess};
  NativeBuffer buffer2{device, size, 4096, drm::Domain::kGtt,
                      drm::BufferFlags::kCpuAccess};

  auto start_cb = BuildStartBuffer(device, reg_config);
  auto sample_cb = BuildSampleBuffer(device, buffer, sample_map);
  auto sample_cb2 = BuildSampleBuffer(device, buffer2, sample_map);

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
    for(auto e : counters) {
	    double result = e->get_results(entry_map, old_p, new_p);
	    std::cout << e->name() << ": ";
	    if (e->type() == CounterType::rate)
		    std::cout << HumanRate(result);
	    else if(e->type() == CounterType::percentage)
		    std::cout << result << " %";
	    std::cout << "\n";
    }

  }

  return 0;
}
