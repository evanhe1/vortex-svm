// Copyright © 2019-2023
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cstdint>
#include <vector>
#include <map>
#include <unordered_map>
#include <cstdint>
#include <stdexcept>
#include "../simx/constants.h"
#include "../../runtime/common/malloc.h"

namespace vortex {
enum VA_MODE
{
  BARE,
  SV64
};

enum ACCESS_TYPE
{
  LOAD,
  STORE,
  FETCH
};

class Page_Fault_Exception : public std::runtime_error /* or logic_error */
{
public:
    Page_Fault_Exception(const std::string& what = "") : std::runtime_error(what) {}
    uint64_t addr;
    ACCESS_TYPE type;
};

struct BadAddress {};
struct OutOfRange {};

class MemDevice {
public:
  virtual ~MemDevice() {}
  virtual uint64_t size() const = 0;
  virtual void read(void* data, uint64_t addr, uint64_t size) = 0;
  virtual void write(const void* data, uint64_t addr, uint64_t size) = 0;
};

///////////////////////////////////////////////////////////////////////////////

class RamMemDevice : public MemDevice {
public:
  RamMemDevice(uint64_t size, uint32_t wordSize);
  RamMemDevice(const char* filename, uint32_t wordSize);
  ~RamMemDevice() {}

  void read(void* data, uint64_t addr, uint64_t size) override;
  void write(const void* data, uint64_t addr, uint64_t size) override;

  virtual uint64_t size() const {
    return contents_.size();
  };

protected:
  std::vector<uint8_t> contents_;
  uint32_t wordSize_;
};

///////////////////////////////////////////////////////////////////////////////

class RomMemDevice : public RamMemDevice {
public:
  RomMemDevice(const char *filename, uint32_t wordSize)
    : RamMemDevice(filename, wordSize)
  {}

  RomMemDevice(uint64_t size, uint32_t wordSize)
    : RamMemDevice(size, wordSize)
  {}

  ~RomMemDevice();

  void write(const void* data, uint64_t addr, uint64_t size) override;
};

///////////////////////////////////////////////////////////////////////////////

class MemoryUnit {
public:

  struct PageFault {
    PageFault(uint64_t a, bool nf)
      : faultAddr(a)
      , notFound(nf)
    {}
    uint64_t  faultAddr;
    bool      notFound;
  };

  MemoryUnit(uint64_t pageSize = RAM_PAGE_SIZE);

  void attach(MemDevice &m, uint64_t start, uint64_t end);

  void read(void* data, uint64_t addr, uint64_t size, bool sup);
  void write(const void* data, uint64_t addr, uint64_t size, bool sup);

  void amo_reserve(uint64_t addr);
  bool amo_check(uint64_t addr);

  void tlbAdd(uint64_t virt, uint64_t phys, uint32_t flags, uint32_t page_size);
  void tlbRm(uint64_t vaddr);
  void tlbFlush() {
    tlb_.clear();
  }

  uint32_t get_satp();  
  void set_satp(uint32_t satp);
  void set_global_allocator(MemoryAllocator* alloc);

private:

  struct amo_reservation_t {
    uint64_t addr;
    bool     valid;
  };

  class ADecoder {
  public:
    ADecoder() {}

    void read(void* data, uint64_t addr, uint64_t size);
    void write(const void* data, uint64_t addr, uint64_t size);

    void map(uint64_t start, uint64_t end, MemDevice &md);

  private:

    struct mem_accessor_t {
      MemDevice*  md;
      uint64_t    addr;
    };

    struct entry_t {
      MemDevice*  md;
      uint64_t    start;
      uint64_t    end;
    };

    bool lookup(uint64_t addr, uint32_t wordSize, mem_accessor_t*);

    std::vector<entry_t> entries_;
  };

  struct TLBEntry {
    TLBEntry() {}
    TLBEntry(uint32_t pfn, uint32_t flags, uint32_t page_size)
      : pfn(pfn)
      , flags(flags)
      , page_size(page_size)
      , mru(0)
    {}
    uint32_t pfn;
    uint32_t flags;
    uint32_t page_size;
    bool mru;
  };

  TLBEntry tlbLookup(uint64_t vAddr, uint32_t flagMask);

  uint64_t toPhyAddr(uint64_t vAddr, uint32_t flagMask);
  std::pair<uint64_t, uint8_t> page_table_walk(uint64_t vAddr_bits, uint64_t* size_bits);
  
  std::unordered_map<uint64_t, TLBEntry> tlb_;
  uint64_t  pageSize_;
  ADecoder  decoder_;
  bool      enableVM_;

  uint32_t satp;
  VA_MODE mode;
  uint32_t ptbr;

  amo_reservation_t amo_reservation_;

  MemoryAllocator* global_mem_;
};

///////////////////////////////////////////////////////////////////////////////

class ACLManager {
public:

    void set(uint64_t addr, uint64_t size, int flags);

    bool check(uint64_t addr, uint64_t size, int flags) const;

private:

  struct acl_entry_t {
    uint64_t end;
    int32_t flags;
  };

  std::map<uint64_t, acl_entry_t> acl_map_;
};

///////////////////////////////////////////////////////////////////////////////

class RAM : public MemDevice {
public:

  RAM(uint64_t capacity, uint32_t page_size);
  RAM(uint64_t capacity) : RAM(capacity, capacity) {}
  ~RAM();

  void clear();

  uint64_t size() const override;

  void read(void* data, uint64_t addr, uint64_t size) override;
  void write(const void* data, uint64_t addr, uint64_t size) override;

  void loadBinImage(const char* filename, uint64_t destination);
  void loadHexImage(const char* filename);

  uint8_t& operator[](uint64_t address) {
    return *this->get(address);
  }

  const uint8_t& operator[](uint64_t address) const {
    return *this->get(address);
  }

  void set_acl(uint64_t addr, uint64_t size, int flags);

  void enable_acl(bool enable) {
    check_acl_ = enable;
  }

private:

  uint8_t *get(uint64_t address) const;

  uint64_t capacity_;
  uint32_t page_bits_;
  mutable std::unordered_map<uint64_t, uint8_t*> pages_;
  mutable uint8_t* last_page_;
  mutable uint64_t last_page_index_;
  ACLManager acl_mngr_;
  bool check_acl_;
};

class PTE_SV64_t 
{

  private:
    uint64_t address;
    uint64_t bits(uint64_t addr, uint8_t s_idx, uint8_t e_idx)
    {
        return (addr >> s_idx) & ((1 << (e_idx - s_idx + 1)) - 1);
    }
    bool bit(uint8_t idx)
    {
        return (address) & (1 << idx);
    }

  public:
    uint64_t ppn[2];
    uint32_t rsw;
    uint32_t flags;
    bool d, a, g, u, x, w, r, v;
    PTE_SV64_t(uint64_t address) : address(address)
    { 
      flags =  bits(address,0,7);
      rsw = bits(address,8,9);
      ppn[0] = bits(address,10,19);
      ppn[1] = bits(address,20,31);

      d = bit(7);
      a = bit(6);
      g = bit(5);
      u = bit(4);
      x = bit(3);
      w = bit(2);
      r = bit(1);
      v = bit(0);
    }
};

class vAddr_SV64_t 
{

  private:
    uint64_t address;
    uint64_t bits(uint64_t addr, uint8_t s_idx, uint8_t e_idx)
    {
        return (addr >> s_idx) & ((1 << (e_idx - s_idx + 1)) - 1);
    }
    bool bit(uint64_t addr, uint8_t idx)
    {
        return (addr) & (1 << idx);
    }

  public:
    uint64_t vpn[2];
    uint64_t pgoff;
    vAddr_SV64_t(uint64_t address) : address(address)
    {
      vpn[0] = bits(address,12,21);
      vpn[1] = bits(address,22,31);
      pgoff = bits(address,0,11);
    }
};

} // namespace vortex
