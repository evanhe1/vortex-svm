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

#include "mem.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <VX_config.h>
#include "util.h"

using namespace vortex;

#define CHECK_ERR(_expr, _cleanup)                                      \
    do                                                                  \
    {                                                                   \
        auto err = _expr;                                               \
        if (err == 0)                                                   \
            break;                                                      \
        printf("[VXDRV] Error: '%s' returned %d!\n", #_expr, (int)err); \
        _cleanup                                                        \
    } while (false)

RamMemDevice::RamMemDevice(const char *filename, uint32_t wordSize)
  : wordSize_(wordSize) {
  std::ifstream input(filename);

  if (!input) {
    std::cout << "Error reading file \"" << filename << "\" into RamMemDevice.\n";
    std::abort();
  }

  do {
    contents_.push_back(input.get());
  } while (input);

  while (contents_.size() & (wordSize-1)) {
    contents_.push_back(0x00);
  }
}

RamMemDevice::RamMemDevice(uint64_t size, uint32_t wordSize)
  : contents_(size)
  , wordSize_(wordSize)
{}

void RamMemDevice::read(void* data, uint64_t addr, uint64_t size) {
  auto addr_end = addr + size;
  if ((addr & (wordSize_-1))
   || (addr_end & (wordSize_-1))
   || (addr_end <= contents_.size())) {
    std::cout << "lookup of 0x" << std::hex << (addr_end-1) << " failed.\n";
    throw BadAddress();
  }

  const uint8_t *s = contents_.data() + addr;
  for (uint8_t *d = (uint8_t*)data, *de = d + size; d != de;) {
    *d++ = *s++;
  }
}

void RamMemDevice::write(const void* data, uint64_t addr, uint64_t size) {
  auto addr_end = addr + size;
  if ((addr & (wordSize_-1))
   || (addr_end & (wordSize_-1))
   || (addr_end <= contents_.size())) {
    std::cout << "lookup of 0x" << std::hex << (addr_end-1) << " failed.\n";
    throw BadAddress();
  }

  const uint8_t *s = (const uint8_t*)data;
  for (uint8_t *d = contents_.data() + addr, *de = d + size; d != de;) {
    *d++ = *s++;
  }
}

///////////////////////////////////////////////////////////////////////////////

void RomMemDevice::write(const void* /*data*/, uint64_t /*addr*/, uint64_t /*size*/) {
  std::cout << "attempt to write to ROM.\n";
  std::abort();
}

///////////////////////////////////////////////////////////////////////////////

bool MemoryUnit::ADecoder::lookup(uint64_t addr, uint32_t wordSize, mem_accessor_t* ma) {
  uint64_t end = addr + (wordSize - 1);
  assert(end >= addr);
  for (auto iter = entries_.rbegin(), iterE = entries_.rend(); iter != iterE; ++iter) {
    if (addr >= iter->start && end <= iter->end) {
      ma->md   = iter->md;
      ma->addr = addr - iter->start;
      return true;
    }
  }
  return false;
}

void MemoryUnit::ADecoder::map(uint64_t start, uint64_t end, MemDevice &md) {
  assert(end >= start);
  entry_t entry{&md, start, end};
  entries_.emplace_back(entry);
}

void MemoryUnit::ADecoder::read(void* data, uint64_t addr, uint64_t size) {
  mem_accessor_t ma;
  if (!this->lookup(addr, size, &ma)) {
    std::cout << "lookup of 0x" << std::hex << addr << " failed.\n";
    throw BadAddress();
  }
  ma.md->read(data, ma.addr, size);
}

void MemoryUnit::ADecoder::write(const void* data, uint64_t addr, uint64_t size) {
  mem_accessor_t ma;
  if (!this->lookup(addr, size, &ma)) {
    std::cout << "lookup of 0x" << std::hex << addr << " failed.\n";
    throw BadAddress();
  }
  ma.md->write(data, ma.addr, size);
}

///////////////////////////////////////////////////////////////////////////////

MemoryUnit::MemoryUnit(uint64_t pageSize)
  : pageSize_(pageSize)
  , enableVM_(pageSize != 0)
  , amo_reservation_({0x0, false}) {
  if (pageSize != 0) {
    tlb_[0] = TLBEntry(0, 077, RAM_PAGE_SIZE);
  }
}

void MemoryUnit::attach(MemDevice &m, uint64_t start, uint64_t end) {
  decoder_.map(start, end, m);
}

void MemoryUnit::set_global_allocator(MemoryAllocator* alloc) {
  global_mem_ = alloc;
}

MemoryUnit::TLBEntry MemoryUnit::tlbLookup(uint64_t vAddr, uint32_t flagMask) {
  auto iter = tlb_.find(vAddr / pageSize_);
  if (iter != tlb_.end()) {
    if (iter->second.flags & flagMask) {
      if (tlb_.size() == TLB_SIZE)
      {
        for (auto& entry : tlb_)
        {
          entry.second.mru = false;
        }
      }
      iter->second.mru = true;
      // std::cout << "TLB hit on vAddr " << vAddr << std::endl;
      return iter->second;
    } else {
     throw PageFault(vAddr, false); // protection fault
    }
  } else {
    // std::cout << "TLB miss on vAddr " << vAddr << std::endl;
    throw PageFault(vAddr, true); // TLB miss
  }
}

uint64_t MemoryUnit::toPhyAddr(uint64_t addr, uint32_t flagMask) {
  uint64_t pAddr;
  uint64_t pfn;
  uint64_t size_bits;
  // std::cout << "mem.cpp vAddr: " << addr << std::endl;
  if (enableVM_) {
    // TODO: Add TLB support
    try {
      TLBEntry t = this->tlbLookup(addr, flagMask);
      pfn = t.pfn;
      size_bits = t.page_size;
      //std::cout << "hit pfn: " << pfn << std::endl;
    } catch (PageFault e) {
      if (e.notFound == true) {
        std::pair<uint64_t, uint8_t> ptw_access = page_table_walk(addr, &size_bits);
        pfn = ptw_access.first;
        std::cout << "miss pfn " << pfn << std::endl;
        tlbAdd(addr, pfn << size_bits, flagMask, size_bits);
      } else {
        throw e;
      }
    }
    int offset = addr % (1 << size_bits);
    pAddr = (pfn << size_bits) + offset;
  } else {
    pAddr = addr;
  }
  return pAddr;
}

std::pair<uint64_t, uint8_t> MemoryUnit::page_table_walk(uint64_t vAddr_bits, uint64_t* size_bits)
{   
    uint64_t LEVELS = 2;
    vAddr_SV64_t vAddr(vAddr_bits);
    uint64_t pte_bytes;

    //Get base page table.
    uint64_t a = this->ptbr;
    int i = LEVELS - 1; 

    // std::cout << "vAddr: " << vAddr_bits << std::endl;
    uint64_t page_table_base_addr;
    while(true)
    {

      //Read PTE.
      // std::cout << std::hex << "a: " << a << std::endl;
      std::cout << std::hex << "vAddr.vpn[i] * PTE_SIZE: " << a + vAddr.vpn[i] * PTE_SIZE << std::endl;
      decoder_.read(&pte_bytes, a + vAddr.vpn[i] * PTE_SIZE, sizeof(uint64_t));
      page_table_base_addr = a + vAddr.vpn[i] * PTE_SIZE;
      PTE_SV64_t pte(pte_bytes);
      
      //Check if it has invalid flag bits.
      
      if ( (pte.v == 0) | ( (pte.r == 0) & (pte.w == 1) ) )
      {
        printf("ptbr on fault: %lx\n", ptbr);
        printf("Fault in mem.cpp\n");
        printf("Stack Base Addr: %lx\n", STACK_BASE_ADDR - RAM_PAGE_SIZE * 2);
        printf("Faulitng vAddr: %lx\n", vAddr_bits);
        printf("vpn level 1: %lx\n", vAddr.vpn[1]);
        printf("vpn level 0: %lx\n", vAddr.vpn[0]);
        printf("Faulting vpn: %lx\n", vAddr.vpn[i]);
        printf("Faulting ppn: %lx\n", pte.ppn[i]);
        printf("Faulting ppn flags: %x\n", pte.flags);
        throw Page_Fault_Exception("Page Fault : Attempted to access invalid entry.");
      }

      if ( (pte.r == 0) & (pte.w == 0) & (pte.x == 0))
      {
        //Not a leaf node as rwx == 000
        i--;
        if (i < 0)
        {
          throw Page_Fault_Exception("Page Fault : No leaf node found.");
        }
        else
        {
          //Continue on to next level.
          a = (pte_bytes >> 10 );
        }
      }
      else
      {
        //Leaf node found, finished walking.
        a = (pte_bytes >> 10 ) << 12;
        break;
      }
    }

    PTE_SV64_t pte(pte_bytes);

    // Check if page is absent and valid
    // TODO: fix
    if ((pte.a == 1) && (pte.v == 1)) {
        std::cout << "pte before: " << std::hex << pte_bytes << std::endl;
        uint64_t addr;
        CHECK_ERR((global_mem_->allocate(RAM_PAGE_SIZE, &addr)), {
            printf("%d\n", err);
        });
        uint64_t ppn = (addr >> 12) << 20;
        pte_bytes = ppn | 0x07; // add read write valid permission
        uint64_t listen;
        decoder_.write(&pte_bytes, page_table_base_addr, sizeof(uint64_t));
        decoder_.read(&listen, page_table_base_addr, sizeof(uint64_t));
        std::cout << "write: " << pte_bytes << " read: " << listen << std::endl;
        std::cout << "writing to: " << page_table_base_addr << std::endl;
        std::cout << "pte after: " << std::hex << pte_bytes << std::endl;
        PTE_SV64_t pte_new(pte_bytes);
        pte = pte_new;
        //std::cout << "pfn: " << pte.ppn[1] << std::endl;
        //std::cout << "i: " << i << std::endl;
        a = (pte_bytes >> 10 ) << 12;
    }
    // TODO: Clarify
    /*
    if ( (type == ACCESS_TYPE::FETCH) & ((pte.r == 0) | (pte.x == 0)) )
    {
      throw Page_Fault_Exception("Page Fault : TYPE FETCH, Incorrect permissions.");
    }
    else if ( (type == ACCESS_TYPE::LOAD) & (pte.r == 0) )
    {
      throw Page_Fault_Exception("Page Fault : TYPE LOAD, Incorrect permissions.");
    }
    else if ( (type == ACCESS_TYPE::STORE) & (pte.w == 0) )
    {
      throw Page_Fault_Exception("Page Fault : TYPE STORE, Incorrect permissions.");
    }*/

    uint64_t pfn;
    if (i > 0)
    {
      //It is a super page.
      if (pte.ppn[0] != 0)
      {
        //Misss aligned super page.
        throw Page_Fault_Exception("Page Fault : Miss Aligned Super Page.");

      }
      else
      {
        //Valid super page.
        pfn = pte.ppn[1];
        *size_bits = 22;
      }
    }
    else
    {
      //Regular page.
      *size_bits = 12;
      pfn = a >> 12;
    }
    std::cout << "translated vAddr 0x" << std::hex << vAddr_bits << " to pAddr 0x" << std::hex << pfn << "000" << std::endl;
    return std::make_pair(pfn, pte_bytes & 0xff);
}
 
void MemoryUnit::read(void* data, uint64_t addr, uint64_t size, bool sup) {
  uint64_t pAddr = this->toPhyAddr(addr, sup ? 8 : 1);
  return decoder_.read(data, pAddr, size);
}

void MemoryUnit::write(const void* data, uint64_t addr, uint64_t size, bool sup) {
  uint64_t pAddr;
  if (addr >= IO_BASE_ADDR) {
    pAddr = addr;
  }
  else {
    pAddr = this->toPhyAddr(addr, sup ? 16 : 1);
  }
  decoder_.write(data, pAddr, size);
  amo_reservation_.valid = false;
}

void MemoryUnit::amo_reserve(uint64_t addr) {
  uint64_t pAddr = this->toPhyAddr(addr, 1);
  amo_reservation_.addr = pAddr;
  amo_reservation_.valid = true;
}

bool MemoryUnit::amo_check(uint64_t addr) {
  uint64_t pAddr = this->toPhyAddr(addr, 1);
  return amo_reservation_.valid && (amo_reservation_.addr == pAddr);
}
void MemoryUnit::tlbAdd(uint64_t virt, uint64_t phys, uint32_t flags, uint32_t page_size) {
  if (TLB_SIZE == 1) {
    if (tlb_.size() == 1) {
      tlb_.erase(tlb_.begin());
    }
    tlb_[virt / pageSize_] = TLBEntry(phys / pageSize_, flags, page_size);
    return;
  }
  
  if (tlb_.size() == TLB_SIZE - 1)
  {
    for (auto& entry : tlb_)
    {
      entry.second.mru = false;
    }
    
  }
  else if (tlb_.size() == TLB_SIZE)
  {
    uint64_t del;
    for (auto entry : tlb_) // mru bit for pseudo-LRU replacement
    {
      if (!entry.second.mru)
      {
        del = entry.first;
        break;
      }
    }
    tlb_.erase(tlb_.find(del));
  }
  tlb_[virt / pageSize_] = TLBEntry(phys / pageSize_, flags, page_size);
}

void MemoryUnit::tlbRm(uint64_t va) {
  if (tlb_.find(va / pageSize_) != tlb_.end())
    tlb_.erase(tlb_.find(va / pageSize_));
}

///////////////////////////////////////////////////////////////////////////////

void ACLManager::set(uint64_t addr, uint64_t size, int flags) {
  if (size == 0)
    return;

  uint64_t end = addr + size;

  // get starting interval
  auto it = acl_map_.lower_bound(addr);
  if (it != acl_map_.begin() && (--it)->second.end < addr) {
    ++it;
  }

  // Remove existing entries that overlap or are within the new range
  while (it != acl_map_.end() && it->first < end) {
    auto current = it++;
    uint64_t current_end = current->second.end;
    if (current_end <= addr)
      continue; // No overlap, no need to adjust

    // Adjust the current interval or erase it depending on overlap and flags
    if (current->first < addr) {
      if (current_end > end) {
        acl_map_[end] = {current_end, current->second.flags};
      }
      current->second.end = addr;
    } else {
      if (current_end > end) {
        acl_map_[end] = {current_end, current->second.flags};
      }
      acl_map_.erase(current);
    }
  }

  // Insert new range if flags are not zero
  if (flags != 0) {
    it = acl_map_.emplace(addr, acl_entry_t{end, flags}).first;
    // Merge adjacent ranges with the same flags
    auto prev = it;
    if (it != acl_map_.begin() && (--prev)->second.end == addr && prev->second.flags == flags) {
      prev->second.end = it->second.end;
      acl_map_.erase(it);
      it = prev;
    }
    auto next = std::next(it);
    if (next != acl_map_.end() && it->second.end == next->first && it->second.flags == next->second.flags) {
      it->second.end = next->second.end;
      acl_map_.erase(next);
    }
  }
}

bool ACLManager::check(uint64_t addr, uint64_t size, int flags) const {
  uint64_t end = addr + size;

  auto it = acl_map_.lower_bound(addr);
  if (it != acl_map_.begin() && (--it)->second.end < addr) {
    ++it;
  }

  while (it != acl_map_.end() && it->first < end) {
    if (it->second.end > addr) {
      if ((it->second.flags & flags) != flags) {
        std::cout << "Memory access violation from 0x" << std::hex << addr << " to 0x" << end << ", current flags=" << it->second.flags << ", access flags=" << flags << std::endl;
        return false; // Overlapping entry is missing at least one required flag bit
      }
      addr = it->second.end; // Move to the end of the current matching range
    }
    ++it;
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////

RAM::RAM(uint64_t capacity, uint32_t page_size)
  : capacity_(capacity)
  , page_bits_(log2ceil(page_size))
  , last_page_(nullptr)
  , last_page_index_(0)
  , check_acl_(false) {
  assert(ispow2(page_size));
  if (capacity != 0) {
    assert(ispow2(capacity));
    assert(page_size <= capacity);
    assert(0 == (capacity % page_size));
  }
}

RAM::~RAM() {
  this->clear();
}

void RAM::clear() {
  for (auto& page : pages_) {
    delete[] page.second;
  }
}

uint64_t RAM::size() const {
  return uint64_t(pages_.size()) << page_bits_;
}

uint8_t *RAM::get(uint64_t address) const {
  if (capacity_ != 0 && address >= capacity_) {
    throw OutOfRange();
  }
  uint32_t page_size   = 1 << page_bits_;
  uint32_t page_offset = address & (page_size - 1);
  uint64_t page_index  = address >> page_bits_;

  uint8_t* page;
  if (last_page_ && last_page_index_ == page_index) {
    page = last_page_;
  } else {
    auto it = pages_.find(page_index);
    if (it != pages_.end()) {
      page = it->second;
    } else {
      uint8_t *ptr = new uint8_t[page_size];
      // set uninitialized data to "baadf00d"
      for (uint32_t i = 0; i < page_size; ++i) {
        ptr[i] = (0xbaadf00d >> ((i & 0x3) * 8)) & 0xff;
      }
      pages_.emplace(page_index, ptr);
      page = ptr;
    }
    last_page_ = page;
    last_page_index_ = page_index;
  }

  return page + page_offset;
}

void RAM::read(void* data, uint64_t addr, uint64_t size) {
  if (check_acl_ && acl_mngr_.check(addr, size, 0x1) == false) {
    std::cout << "RAM::read failed" << std::endl; 
    throw BadAddress();
  }
  uint8_t* d = (uint8_t*)data;
  for (uint64_t i = 0; i < size; i++) {
    d[i] = *this->get(addr + i);
  }
}

void RAM::write(const void* data, uint64_t addr, uint64_t size) {
  if (check_acl_ && acl_mngr_.check(addr, size, 0x2) == false) {
    std::cout << "RAM::write failed" << std::endl; 
    throw BadAddress();
  }
  const uint8_t* d = (const uint8_t*)data;
  for (uint64_t i = 0; i < size; i++) {
    *this->get(addr + i) = d[i];
  }
}

void RAM::set_acl(uint64_t addr, uint64_t size, int flags) {
  if (capacity_ != 0 && (addr + size)> capacity_) {
    throw OutOfRange();
  }
  acl_mngr_.set(addr, size, flags);
}

void RAM::loadBinImage(const char* filename, uint64_t destination) {
  std::ifstream ifs(filename);
  if (!ifs) {
    std::cout << "error: " << filename << " not found" << std::endl;
    std::abort();
  }

  ifs.seekg(0, ifs.end);
  size_t size = ifs.tellg();
  std::vector<uint8_t> content(size);
  ifs.seekg(0, ifs.beg);
  ifs.read((char*)content.data(), size);

  this->clear();
  this->write(content.data(), destination, size);
}

void RAM::loadHexImage(const char* filename) {
  auto hti = [&](char c)->uint32_t {
    if (c >= 'A' && c <= 'F')
      return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
      return c - 'a' + 10;
    return c - '0';
  };

  auto hToI = [&](const char *c, uint32_t size)->uint32_t {
    uint32_t value = 0;
    for (uint32_t i = 0; i < size; i++) {
      value += hti(c[i]) << ((size - i - 1) * 4);
    }
    return value;
  };

  std::ifstream ifs(filename);
  if (!ifs) {
    std::cout << "error: " << filename << " not found" << std::endl;
    std::abort();
  }

  ifs.seekg(0, ifs.end);
  size_t size = ifs.tellg();
  std::vector<char> content(size);
  ifs.seekg(0, ifs.beg);
  ifs.read(content.data(), size);

  uint32_t offset = 0;
  char *line = content.data();

  this->clear();

  while (true) {
    if (line[0] == ':') {
      uint32_t byteCount = hToI(line + 1, 2);
      uint32_t nextAddr = hToI(line + 3, 4) + offset;
      uint32_t key = hToI(line + 7, 2);
      switch (key) {
      case 0:
        for (uint32_t i = 0; i < byteCount; i++) {
          uint32_t addr  = nextAddr + i;
          uint32_t value = hToI(line + 9 + i * 2, 2);
          *this->get(addr) = value;
        }
        break;
      case 2:
        offset = hToI(line + 9, 4) << 4;
        break;
      case 4:
        offset = hToI(line + 9, 4) << 16;
        break;
      default:
        break;
      }
    }
    while (*line != '\n' && size != 0) {
      ++line;
      --size;
    }
    if (size <= 1)
      break;
    ++line;
    --size;
  }
}

uint32_t MemoryUnit::get_satp()
{
  return satp;
}  
void MemoryUnit::set_satp(uint32_t satp)
{
  this->satp = satp;
  this->ptbr = satp & 0x003fffff;
  this->mode = VA_MODE::SV64;
}
