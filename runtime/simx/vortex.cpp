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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <iostream>
#include <future>
#include <chrono>

#include <vortex.h>
#include <utils.h>
#include <malloc.h>

#include <VX_config.h>
#include <VX_types.h>

#include <util.h>

#include <processor.h>
#include <arch.h>
#include <mem.h>
#include <constants.h>

using namespace vortex;

#ifndef NDEBUG
#define DBGPRINT(format, ...)                        \
    do                                               \
    {                                                \
        printf("[VXDRV] " format "", ##__VA_ARGS__); \
    } while (0)
#else
#define DBGPRINT(format, ...) ((void)0)
#endif

#define CHECK_ERR(_expr, _cleanup)                                      \
    do                                                                  \
    {                                                                   \
        auto err = _expr;                                               \
        if (err == 0)                                                   \
            break;                                                      \
        printf("[VXDRV] Error: '%s' returned %d!\n", #_expr, (int)err); \
        _cleanup                                                        \
    } while (false)

uint64_t bits(uint64_t addr, uint8_t s_idx, uint8_t e_idx)
{
    return (addr >> s_idx) & ((1 << (e_idx - s_idx + 1)) - 1);
}
bool bit(uint64_t addr, uint8_t idx)
{
    return (addr) & (1 << idx);
}

///////////////////////////////////////////////////////////////////////////////

class vx_device
{
public:
    vx_device()
        : arch_(NUM_THREADS, NUM_WARPS, NUM_CORES), ram_(0, RAM_PAGE_SIZE), processor_(arch_), global_mem_(ALLOC_BASE_ADDR, GLOBAL_MEM_SIZE - ALLOC_BASE_ADDR, RAM_PAGE_SIZE, CACHE_BLOCK_SIZE), virtual_allocator_(ALLOC_BASE_ADDR, GLOBAL_MEM_SIZE - ALLOC_BASE_ADDR, RAM_PAGE_SIZE, CACHE_BLOCK_SIZE)
    {
        // attach memory module
        processor_.attach_ram(&ram_);
        // attach allocator
        processor_.set_global_allocator(&global_mem_);

        // Sets more
        set_processor_satp(VM_ADDR_MODE);

        // reservation to avoid tlb collision with amo_reservation entry
        virtual_allocator_.reserve(ALLOC_BASE_ADDR, 3 * RAM_PAGE_SIZE);
    }

    ~vx_device()
    {
        if (future_.valid())
        {
            future_.wait();
        }
        profiling_remove(profiling_id_);
    }

    int init()
    {
        CHECK_ERR(dcr_initialize(this), {
            return err;
        });
        profiling_id_ = profiling_add(this);
        return 0;
    }

    int get_caps(uint32_t caps_id, uint64_t *value)
    {
        uint64_t _value;
        switch (caps_id)
        {
        case VX_CAPS_VERSION:
            _value = IMPLEMENTATION_ID;
            break;
        case VX_CAPS_NUM_THREADS:
            _value = NUM_THREADS;
            break;
        case VX_CAPS_NUM_WARPS:
            _value = NUM_WARPS;
            break;
        case VX_CAPS_NUM_CORES:
            _value = NUM_CORES * NUM_CLUSTERS;
            break;
        case VX_CAPS_NUM_BARRIERS:
            _value = NUM_BARRIERS;
            break;
        case VX_CAPS_CACHE_LINE_SIZE:
            _value = CACHE_BLOCK_SIZE;
            break;
        case VX_CAPS_GLOBAL_MEM_SIZE:
            _value = GLOBAL_MEM_SIZE;
            break;
        case VX_CAPS_LOCAL_MEM_SIZE:
            _value = (1 << LMEM_LOG_SIZE);
            break;
        case VX_CAPS_LOCAL_MEM_ADDR:
            _value = LMEM_BASE_ADDR;
            break;
        case VX_CAPS_ISA_FLAGS:
            _value = ((uint64_t(MISA_EXT)) << 32) | ((log2floor(XLEN) - 4) << 30) | MISA_STD;
            break;
        default:
            std::cout << "invalid caps id: " << caps_id << std::endl;
            std::abort();
            return -1;
        }
        *value = _value;
        return 0;
    }

    int map_local_mem(uint64_t size, uint64_t *dev_maddr)
    {
        bool skip = false;
        if (*dev_maddr == STARTUP_ADDR || *dev_maddr == 0x7FFFF000)
        {
            skip = true;
        }

        if (get_mode() == VA_MODE::BARE)
        {
            return 0;
        }

        uint64_t ppn = *dev_maddr >> 12; // 4 KB pages
        uint64_t init_pAddr = *dev_maddr;

        // uint64_t init_vAddr = *dev_maddr + 0xf0000000; // vpn will change, but we want to return the vpn of the beginning of the virtual allocation
        // init_vAddr = (init_vAddr >> 12) << 12;         // Shift off any page offset bits
        uint64_t init_vAddr;
        uint64_t vpn;
        bool init_addr_set = false;

        // dev_maddr can be of size greater than a page, but we have to map and update
        // page tables on a page table granularity. So divide the allocation into pages.
        for (ppn = (*dev_maddr) >> 12; ppn < ((*dev_maddr) >> 12) + (size / RAM_PAGE_SIZE) + 1; ppn++)
        {
            // Currently a 1-1 mapping is used, this can be changed here to support different
            // mapping schemes
            if (!skip) {
                uint64_t vAddr;
                std::cout << "alloc size: " << size << std::endl;
                virtual_allocator_.allocate(std::min((uint64_t)RAM_PAGE_SIZE, size % RAM_PAGE_SIZE), &vAddr);
                std::cout << "alloced vpn: " << (vAddr >> 12) << std::endl;
                vpn = vAddr >> 12;
                if (!init_addr_set) {
                    init_vAddr = vpn << 12;
                    init_addr_set = true;
                }
            } else {
                vpn = ppn;
            }

            // vpn = skip ? ppn : ppn + 0xf0000;
            // vpn = ppn;

            // If ppn to vpn mapping doesnt exist.
            if (addr_mapping.find(vpn) == addr_mapping.end())
            {
                std::cout << "creating mapping vpn " << vpn << " to ppn " << ppn << std::endl;
                // Create mapping.
                update_page_table(ppn, vpn);
                addr_mapping[vpn] = ppn;
            }
        }

        uint64_t size_bits;
        if (skip)
        {
            return 0;
        }

        *dev_maddr = init_vAddr; // commit vpn to be returned to host

        return 0;
    }

    // For when virutal address is known and need to reverse physical mapping
    int map_virtual_physical(uint64_t size, uint64_t *virt_addr)
    {
        bool skip = false;
        if (*virt_addr == STARTUP_ADDR || *virt_addr == 0x7FFFF000)
        {
            skip = true;
        }

        uint64_t vpn = *virt_addr >> 12; // 4 KB pages
        uint64_t init_vAddr = *virt_addr;
        uint64_t init_pAddr = *virt_addr - 0xf0000000; // vpn will change, but we want to return the vpn of the beginning of the virtual allocation
        init_pAddr = (init_pAddr >> 12) << 12;         // Shift off any page offset bits
        uint64_t ppn;

        // virt_addr can be of size greater than a page, but we have to map and update
        // page tables on a page table granularity. So divide the allocation into pages.
        for (vpn = (*virt_addr) >> 12; vpn < ((*virt_addr) >> 12) + (size / RAM_PAGE_SIZE) + 1; vpn++)
        {
            // Currently a 1-1 mapping is used, this can be changed here to support different
            // mapping schemes
            ppn = skip ? vpn : vpn - 0xf0000;
            // vpn = ppn;

            // If ppn to vpn mapping doesnt exist.
            if (addr_mapping.find(vpn) == addr_mapping.end())
            {
                // Create mapping.
                update_page_table(ppn, vpn);
                addr_mapping[vpn] = ppn;
            }
        }

        uint64_t size_bits;
        if (skip)
        {
            return 0;
        }
        *virt_addr = init_pAddr; // commit vpn to be returned to host

        return 0;
    }

    int mem_alloc(uint64_t size, int flags, uint64_t *dev_addr)
    {
        uint64_t addr;
        CHECK_ERR(global_mem_.allocate(size, &addr), {
            return err;
        });
        printf("physical: %x\n", addr);
        uint64_t offset = addr % RAM_PAGE_SIZE;
        uint64_t phys_addr = addr;
        CHECK_ERR(map_local_mem(size, &addr), {
            return err;
        });
        printf("virtual: %x\n", addr);
        printf("Page Offset: %x\n", offset);
        printf("Size: %d\n", size);
        std::cout << "mem alloc flags: " << flags << std::endl;
        std::cout << "mem alloc size: " << size << std::endl;
        CHECK_ERR(this->mem_access(phys_addr, size, flags), {
            global_mem_.release(phys_addr);
            return err;
        });
        *dev_addr = addr + offset;
        return 0;
    }

    int mem_reserve(uint64_t dev_addr, uint64_t size, int flags)
    {
        std::cout << std::hex << "address: " << dev_addr << " size: " << size <<  std::endl;
        CHECK_ERR(virtual_allocator_.reserve(dev_addr, size), {
            return err;
        });
        for (uint64_t vpn = dev_addr >> 12; vpn < (dev_addr >> 12) + (size / RAM_PAGE_SIZE) + 1; vpn++) {
            std::cout << "setting flags for vpn: " << std::hex << vpn << std::endl;
            CHECK_ERR(this->set_pte_flag((vpn << 12) | 0x000, flags), {
                virtual_allocator_.release(vpn);
                return err;
            });
        }
        return 0;
    }

    int mem_free(uint64_t dev_addr)
    {
        printf("freeing: %lx\n", dev_addr);
        return global_mem_.release(dev_addr);
    }

    int mem_access(uint64_t dev_addr, uint64_t size, int flags)
    {
        std::cout << "mem_access setting addr: " << dev_addr << std::endl;
        uint64_t asize = aligned_size(size, CACHE_BLOCK_SIZE);
        if (dev_addr + asize > GLOBAL_MEM_SIZE)
            return -1;

        ram_.set_acl(dev_addr, size, flags);
        return 0;
    }

    int mem_info(uint64_t *mem_free, uint64_t *mem_used) const
    {
        if (mem_free)
            *mem_free = global_mem_.free();
        if (mem_used)
            *mem_used = global_mem_.allocated();
        return 0;
    }

    int upload(uint64_t dest_addr, const void *src, uint64_t size)
    {
        uint64_t asize = aligned_size(size, CACHE_BLOCK_SIZE);
        uint64_t pAddr = dest_addr; // map_local_mem overwrites the provided dest_addr, so store away physical destination address
        if (dest_addr + asize > GLOBAL_MEM_SIZE)
            return -1;

        if (dest_addr >= STARTUP_ADDR)
        {
            map_local_mem(asize, &dest_addr);
        }
        else if (dest_addr >= 0x7fff0000)
        {
            map_local_mem(asize, &dest_addr);
        }

        ram_.enable_acl(false);
        ram_.write((const uint8_t *)src, pAddr, size);
        ram_.enable_acl(true);

        uint32_t v; 
        ram_.read(&v, pAddr, sizeof(uint32_t));

        /*DBGPRINT("upload %ld bytes to 0x%lx\n", size, dest_addr);
        for (uint64_t i = 0; i < size && i < 1024; i += 4) {
            DBGPRINT("  0x%lx <- 0x%x\n", dest_addr + i, *(uint32_t*)((uint8_t*)src + i));
        }*/

        return 0;
    }

    int download(void *dest, uint64_t src_addr, uint64_t size)
    {
        uint64_t asize = aligned_size(size, CACHE_BLOCK_SIZE);
        if (src_addr + asize > GLOBAL_MEM_SIZE)
            return -1;

        ram_.enable_acl(false);
        ram_.read((uint8_t *)dest, src_addr, size);
        ram_.enable_acl(true);

        /*DBGPRINT("download %ld bytes from 0x%lx\n", size, src_addr);
        for (uint64_t i = 0; i < size && i < 1024; i += 4) {
            DBGPRINT("  0x%lx -> 0x%x\n", src_addr + i, *(uint32_t*)((uint8_t*)dest + i));
        }*/

        return 0;
    }

    int start(uint64_t krnl_addr, uint64_t args_addr)
    {
        // ensure prior run completed
        if (future_.valid())
        {
            future_.wait();
        }
        for (auto i = addr_mapping.begin(); i != addr_mapping.end(); i++)
        {
            std::cout << "virtual: " << std::hex << i->first << " to physical: " << std::hex << i->second << std::endl;
        }
        // set kernel info
        // TODO: new DCR for SATP
        this->dcr_write(VX_DCR_BASE_STARTUP_ADDR0, krnl_addr & 0xffffffff);
        this->dcr_write(VX_DCR_BASE_STARTUP_ADDR1, krnl_addr >> 32);
        this->dcr_write(VX_DCR_BASE_STARTUP_ARG0, args_addr & 0xffffffff);
        this->dcr_write(VX_DCR_BASE_STARTUP_ARG1, args_addr >> 32);
        this->dcr_write(VX_DCR_BASE_SATP_ADDR0, this->satp & 0xffffffff);
        this->dcr_write(VX_DCR_BASE_SATP_ADDR1, this->satp >> 32);


        profiling_begin(profiling_id_);

        // start new run
        future_ = std::async(std::launch::async, [&]
                             { processor_.run(); });

        // clear mpm cache
        mpm_cache_.clear();

        return 0;
    }

    int ready_wait(uint64_t timeout)
    {
        if (!future_.valid())
            return 0;
        uint64_t timeout_sec = timeout / 1000;
        std::chrono::seconds wait_time(1);
        for (;;)
        {
            // wait for 1 sec and check status
            auto status = future_.wait_for(wait_time);
            if (status == std::future_status::ready)
                break;
            if (0 == timeout_sec--)
                return -1;
        }
        profiling_end(profiling_id_);
        return 0;
    }

    int dcr_write(uint32_t addr, uint32_t value)
    {
        if (future_.valid())
        {
            future_.wait(); // ensure prior run completed
        }
        processor_.dcr_write(addr, value);
        dcrs_.write(addr, value);
        return 0;
    }

    int dcr_read(uint32_t addr, uint32_t *value) const
    {
        return dcrs_.read(addr, value);
    }

    int mpm_query(uint32_t addr, uint32_t core_id, uint64_t *value)
    {
        uint32_t offset = addr - VX_CSR_MPM_BASE;
        if (offset > 31)
            return -1;
        if (mpm_cache_.count(core_id) == 0)
        {
            uint64_t mpm_mem_addr = IO_MPM_ADDR + core_id * 32 * sizeof(uint64_t);
            CHECK_ERR(this->download(mpm_cache_[core_id].data(), mpm_mem_addr, 32 * sizeof(uint64_t)), {
                return err;
            });
        }
        *value = mpm_cache_.at(core_id).at(offset);
        return 0;
    }

    void set_processor_satp(VA_MODE mode)
    {
        if (mode == VA_MODE::BARE)
            this->satp = 0;
        else if (mode == VA_MODE::SV64)
        {
            this->satp = alloc_page_table();
        }
        processor_.set_satp(this->satp);
    }

    uint32_t get_ptbr()
    {

        return processor_.get_satp();
    }

    VA_MODE get_mode()
    {
        return VA_MODE::SV64;
    }

    void update_page_table(uint64_t pAddr, uint64_t vAddr)
    {
        std::cout << "mapping PPN 0x" << std::hex << pAddr << " to VPN 0x" << std::hex << vAddr << ":" << std::endl;
        std::cout << "\t";
        vAddr = vAddr << 12;
        // Updating page table with the following mapping of (vAddr) to (pAddr).
        uint64_t ppn_1, pte_addr, pte_bytes;
        uint64_t vpn_1 = bits(vAddr, 22, 31);
        uint64_t vpn_0 = bits(vAddr, 12, 21);

        // Read first level PTE.
        pte_addr = get_ptbr() + vpn_1 * PTE_SIZE;
        pte_bytes = read_pte(pte_addr);

        if (bit(pte_bytes, 0) && ((pte_bytes & 0xFFFFFFFF) != 0xbaadf00d))
        {
            // If valid bit set, proceed to next level using new ppn form PTE.
            ppn_1 = (pte_bytes >> 10);
            std::cout << pte_bytes;
        }
        else
        {
            // If valid bit not set, allocate a second level page table
            //  in device memory and store ppn in PTE. Set rwx = 000 in PTE
            // to indicate this is a pointer to the next level of the page table.

            // HW: pte not initialized here
            ppn_1 = alloc_page_table();
            pte_bytes = ((ppn_1 << 10) | 0b0000000001);
            std::cout << "pte entry: " << std::hex << pte_bytes;
            std::cout << std::hex << " vpn1: " << vpn_1;
            write_pte(pte_addr, pte_bytes);
        }
        std::cout << " --> " << std::endl
                  << "\t\t";

        // Read second level PTE.
        pte_addr = ppn_1 + vpn_0 * PTE_SIZE;
        pte_bytes = read_pte(pte_addr);

        if (bit(pte_bytes, 0) && ((pte_bytes & 0xFFFFFFFF) != 0xbaadf00d))
        {
            std::cout << "ERROR, shouldn't be here" << std::endl;
            // If valid bit is set, then the page is already allocated.
            // Should not reach this point, a sanity check.
        }
        else
        {
            // If valid bit not set, write ppn of pAddr in PTE. Set rwx = 111 in PTE
            // to indicate this is a leaf PTE and has the stated permissions.
            pte_bytes = ((pAddr << 10) | 0b0000001111);
            std::cout << "pte entry: " << std::hex << pte_bytes;
            std::cout << std::hex << " vpn0: " << vpn_0;
            write_pte(pte_addr, pte_bytes);

            // If super paging is enabled.
            if (SUPER_PAGING)
            {
                // Check if this second level Page Table can be promoted to a super page. Brute force
                // method is used to iterate over all PTE entries of the table and check if they have
                // their valid bit set.
                bool superpage = true;
                for (int i = 0; i < 1024; i++)
                {
                    pte_addr = (ppn_1 << 12) + i;
                    pte_bytes = read_pte(pte_addr);

                    if (!bit(pte_bytes, 0) && ((pte_bytes & 0xFFFFFFFF) != 0xbaadf00d))
                    {
                        superpage = false;
                        break;
                    }
                }
                if (superpage)
                {
                    // This can be promoted to a super page. Set root PTE to the first PTE of the
                    // second level. This is because the first PTE of the second level already has the
                    // correct PPN1, PPN0 set to zero and correct access bits.
                    pte_addr = (ppn_1 << 12);
                    pte_bytes = read_pte(pte_addr);
                    pte_addr = get_ptbr() + vpn_1 * PTE_SIZE;
                    write_pte(pte_addr, pte_bytes);
                }
            }
        }
        std::cout << " --> " << std::endl
                  << "\t\t\t0x" << std::hex << pAddr << std::endl;
    }

    std::pair<uint64_t, uint8_t> page_table_walk(uint64_t vAddr_bits, uint64_t *size_bits)
    {
        uint64_t LEVELS = 2;
        vAddr_SV64_t vAddr(vAddr_bits);
        uint64_t pte_bytes;

        // Get base page table.
        uint64_t a = get_ptbr();
        int i = LEVELS - 1;

        std::cout << std::hex << "page table walk for: " << vAddr_bits << std::endl;
        while (true)
        {
            // Read PTE.
            //  HW: ram read from 1st layer page table
            std::cout << std::hex << "a: " << a << std::endl;
            std::cout << std::hex << "vAddr.vpn[i] * PTE_SIZE: " << vAddr.vpn[i] * PTE_SIZE << std::endl;
            ram_.read(&pte_bytes, a + vAddr.vpn[i] * PTE_SIZE, sizeof(uint64_t));

            // pte_bytes &= 0x00000000FFFFFFFF;
            PTE_SV64_t pte(pte_bytes);

            // Check if it has invalid flag bits.
            if ((pte.v == 0) | ((pte.r == 0) & (pte.w == 1)))
            {
                printf("Fault in vortex.cpp\n");
                printf("Faulitng vAddr: %lx\n", vAddr_bits);
                printf("Faulting ppn: %lx\n", pte.ppn[0]);
                printf("Faulting ppn: %lx\n", pte.ppn[1]);
                printf("Faulting ppn flags: %x\n", pte.flags);
                throw Page_Fault_Exception("Page Fault : Attempted to access invalid entry. Entry: 0x");
            }
            // HW: this line tells if we're still looking for metadata or the final PTE
            if ((pte.r == 0) & (pte.w == 0) & (pte.x == 0))
            {
                // Not a leaf node as rwx == 000
                i--;
                if (i < 0)
                {
                    throw Page_Fault_Exception("Page Fault : No leaf node found.");
                }
                else
                {
                    // Continue on to next level.
                    //  shift off bottom 10 bits (status, offset, etc)
                    a = (pte_bytes >> 10);
                }
            }
            else
            {
                // Leaf node found, finished walking.
                //  actual physical addr found
                a = (pte_bytes >> 10) << 12;
                break;
            }
        }

        PTE_SV64_t pte(pte_bytes);

        // Check RWX permissions according to access type.
        if (pte.r == 0)
        {
            throw Page_Fault_Exception("Page Fault : TYPE LOAD, Incorrect permissions.");
        }

        uint64_t pfn;
        if (i > 0)
        {
            // It is a super page.
            if (pte.ppn[0] != 0)
            {
                // Misss aligned super page.
                throw Page_Fault_Exception("Page Fault : Miss Aligned Super Page.");
            }
            else
            {
                // Valid super page.
                pfn = pte.ppn[1];
                *size_bits = 22;
            }
        }

        else
        {
            // Regular page.
            *size_bits = 12;
            pfn = a >> 12;
        }
        std::cout << "translated vAddr 0x" << std::hex << vAddr_bits << " to pAddr 0x" << std::hex << pfn << "000" << std::endl;

        return std::make_pair(pfn, pte_bytes & 0xff);
    }

    // Find page table entry for a given virtual address and update flag value
    int set_pte_flag(uint64_t vAddr_bits, uint8_t flag_mask) {

        uint64_t LEVELS = 2;
        vAddr_SV64_t vAddr(vAddr_bits);
        uint64_t pte_bytes;
        uint64_t pte_addr;

        // Get base page table.
        uint64_t a = get_ptbr();
        int i = LEVELS - 1;

        while (true)
        {
            // Read PTE.
            pte_addr = a + vAddr.vpn[i] * PTE_SIZE;
            pte_bytes = read_pte(pte_addr);
            // pte_bytes &= 0x00000000FFFFFFFF;
            PTE_SV64_t pte(pte_bytes);

            // HW: this line tells if we're still looking for metadata or the final PTE
            if (i != 0 & ((pte.r == 0) & (pte.w == 0) & (pte.x == 0)))
            {
                // Not a leaf node as rwx == 000
                i--;
                // Need to allocate second level page table
                if (pte.v == 0) {
                    std::cout << "allocating unmapped pte" << std::endl;
                    uint64_t ppn_1 = alloc_page_table();
                    pte_bytes = ((ppn_1 << 10) | 0b0000000001);
                    write_pte(pte_addr, pte_bytes);
                }
                // Continue on to next level.
                //  shift off bottom 10 bits (status, offset, etc)
                a = (pte_bytes >> 10);
            }
            else
            {
                // Leaf node found, finished walking.
                //  actual physical addr found
                a = (pte_bytes >> 10) << 12;
                break;
            }
        }


        pte_bytes = (pte_bytes >> 8) << 8; // shift off status flags
        pte_bytes |= (flag_mask & 0xff); // Set new flag values
        std::cout << "old pte: " << std::hex << read_pte(pte_addr) << std::endl; 
        write_pte(pte_addr, pte_bytes);
        std::cout << "new pte: " << std::hex << read_pte(pte_addr) << std::endl; 
        return 0;
    }

    uint64_t alloc_page_table()
    {
        uint64_t addr;
        global_mem_.allocate(RAM_PAGE_SIZE * 2, &addr);
        init_page_table(addr);
        return addr;
    }

    void init_page_table(uint64_t addr)
    {
        uint64_t asize = aligned_size(RAM_PAGE_SIZE * 2, CACHE_BLOCK_SIZE);
        uint8_t *src = new uint8_t[RAM_PAGE_SIZE * 2];
        for (uint64_t i = 0; i < RAM_PAGE_SIZE * 2; ++i)
        {
            src[i] = (0x00000000 >> ((i & 0x3) * 8)) & 0xff;
        }
        ram_.write((const uint8_t *)src, addr, asize);
    }

    void read_page_table(uint64_t addr)
    {
        uint8_t *dest = new uint8_t[RAM_PAGE_SIZE * 2];
        download(dest, addr, RAM_PAGE_SIZE * 2);
        printf("VXDRV: download %d bytes from 0x%x\n", RAM_PAGE_SIZE * 2, addr);
        for (int i = 0; i < RAM_PAGE_SIZE * 2; i += 4)
        {
            printf("mem-read: 0x%x -> 0x%x\n", addr + i, *(uint64_t *)((uint8_t *)dest + i));
        }
    }

    void write_pte(uint64_t addr, uint64_t value = 0xbaadf00d)
    {
        uint8_t *src = new uint8_t[PTE_SIZE];
        for (uint64_t i = 0; i < PTE_SIZE; ++i)
        {
            //(value >> ((i & 0x3) * 8)) & 0xff;
            src[i] = (value >> (i * 8)) & 0xff;
        }
        std::cout << "pte write dest: " << addr << std::endl;
        ram_.enable_acl(false);
        ram_.write((const uint8_t *)src, addr, PTE_SIZE);
        ram_.enable_acl(true);
    }

    uint64_t read_pte(uint64_t addr)
    {
        uint8_t *dest = new uint8_t[PTE_SIZE];
        ram_.read((uint8_t *)dest, addr, PTE_SIZE);
        return *(uint64_t *)((uint8_t *)dest);
    }

private:
    Arch arch_;
    RAM ram_;
    Processor processor_;
    MemoryAllocator global_mem_;
    MemoryAllocator virtual_allocator_;
    DeviceConfig dcrs_;
    std::future<void> future_;
    std::unordered_map<uint64_t, uint64_t> addr_mapping;
    std::unordered_map<uint32_t, std::array<uint64_t, 32>> mpm_cache_;
    int profiling_id_;
    uint64_t satp;
};

struct vx_buffer
{
    vx_device *device;
    uint64_t addr;
    uint64_t size;
};

///////////////////////////////////////////////////////////////////////////////

extern int vx_dev_open(vx_device_h *hdevice)
{
    if (nullptr == hdevice)
        return -1;

    auto device = new vx_device();
    if (device == nullptr)
        return -1;

    CHECK_ERR(device->init(), {
        delete device;
        return err;
    });

    DBGPRINT("DEV_OPEN: hdevice=%p\n", (void *)device);

    *hdevice = device;

    return 0;
}

extern int vx_dev_close(vx_device_h hdevice)
{
    if (nullptr == hdevice)
        return -1;

    DBGPRINT("DEV_CLOSE: hdevice=%p\n", hdevice);

    auto device = ((vx_device *)hdevice);

    delete device;

    return 0;
}

extern int vx_dev_caps(vx_device_h hdevice, uint32_t caps_id, uint64_t *value)
{
    if (nullptr == hdevice)
        return -1;

    vx_device *device = ((vx_device *)hdevice);

    uint64_t _value;

    CHECK_ERR(device->get_caps(caps_id, &_value), {
        return err;
    });

    DBGPRINT("DEV_CAPS: hdevice=%p, caps_id=%d, value=%ld\n", hdevice, caps_id, _value);

    *value = _value;

    return 0;
}

extern int vx_mem_alloc(vx_device_h hdevice, uint64_t size, int flags, vx_buffer_h *hbuffer)
{
    if (nullptr == hdevice || nullptr == hbuffer || 0 == size)
        return -1;

    auto device = ((vx_device *)hdevice);

    uint64_t dev_addr;
    CHECK_ERR(device->mem_alloc(size, flags, &dev_addr), {
        return err;
    });

    auto buffer = new vx_buffer{device, dev_addr, size};
    if (nullptr == buffer)
    {
        device->mem_free(dev_addr);
        return -1;
    }

    DBGPRINT("MEM_ALLOC: hdevice=%p, size=%ld, flags=0x%d, hbuffer=%p\n", hdevice, size, flags, (void *)buffer);

    *hbuffer = buffer;
    std::cout << "address: " << std::hex << buffer->addr << std::endl;
    return 0;
}

extern int vx_mem_reserve(vx_device_h hdevice, uint64_t address, uint64_t size, int flags, vx_buffer_h *hbuffer)
{
    if (nullptr == hdevice || nullptr == hbuffer || 0 == size)
        return -1;

    auto device = ((vx_device *)hdevice);

    CHECK_ERR(device->mem_reserve(address, size, flags), {
        return err;
    });

    auto buffer = new vx_buffer{device, address, size};
    if (nullptr == buffer)
    {
        device->mem_free(address);
        return -1;
    }

    DBGPRINT("MEM_RESERVE: hdevice=%p, address=0x%lx, size=%ld, flags=0x%d, hbuffer=%p\n", hdevice, address, size, flags, (void *)buffer);

    *hbuffer = buffer;

    return 0;
}

extern int vx_mem_free(vx_buffer_h hbuffer)
{
    if (nullptr == hbuffer)
        return 0;

    DBGPRINT("MEM_FREE: hbuffer=%p\n", hbuffer);

    auto buffer = ((vx_buffer *)hbuffer);
    auto device = ((vx_device *)buffer->device);

    std::cout << "addr: " << buffer->addr << std::endl;

    uint64_t offset = buffer->addr % RAM_PAGE_SIZE;
    uint64_t size_bits;
    printf("pre walk\n");
    std::pair<uint64_t, uint8_t> ptw_access = device->page_table_walk(buffer->addr, &size_bits);
    printf("post walk\n");
    uint64_t pfn = ptw_access.first;
    buffer->addr = (pfn << 12) + offset;

    if (0 == buffer->addr)
        return 0;
        
    int err = device->mem_free(buffer->addr);

    delete buffer;

    return err;
}

extern int vx_mem_access(vx_buffer_h hbuffer, uint64_t offset, uint64_t size, int flags)
{
    if (nullptr == hbuffer)
        return -1;

    auto buffer = ((vx_buffer *)hbuffer);
    auto device = ((vx_device *)buffer->device);

    if ((offset + size) > buffer->size)
        return -1;

    DBGPRINT("MEM_ACCESS: hbuffer=%p, offset=%ld, size=%ld, flags=%d\n", hbuffer, offset, size, flags);
    printf("Mem Address: %lx\n", buffer->addr);
    return device->mem_access(buffer->addr + offset, size, flags);
}

extern int vx_mem_address(vx_buffer_h hbuffer, uint64_t *address)
{
    if (nullptr == hbuffer)
        return -1;

    auto buffer = ((vx_buffer *)hbuffer);

    DBGPRINT("MEM_ADDRESS: hbuffer=%p, address=0x%lx\n", hbuffer, buffer->addr);

    *address = buffer->addr;

    return 0;
}

extern int vx_mem_info(vx_device_h hdevice, uint64_t *mem_free, uint64_t *mem_used)
{
    if (nullptr == hdevice)
        return -1;

    auto device = ((vx_device *)hdevice);

    uint64_t _mem_free, _mem_used;

    CHECK_ERR(device->mem_info(&_mem_free, &_mem_used), {
        return err;
    });

    DBGPRINT("MEM_INFO: hdevice=%p, mem_free=%ld, mem_used=%ld\n", hdevice, _mem_free, _mem_used);

    if (mem_free)
    {
        *mem_free = _mem_free;
    }

    if (mem_used)
    {
        *mem_used = _mem_used;
    }

    return 0;
}

extern int vx_copy_to_dev(vx_buffer_h hbuffer, const void *host_ptr, uint64_t dst_offset, uint64_t size)
{
    if (nullptr == hbuffer || nullptr == host_ptr)
        return -1;

    auto buffer = ((vx_buffer *)hbuffer);
    auto device = ((vx_device *)buffer->device);

    if ((dst_offset + size) > buffer->size)
        return -1;

    if (!(buffer->addr + dst_offset == 0x7FFFF000))
    {
        uint64_t offset = buffer->addr % RAM_PAGE_SIZE;
        uint64_t size_bits;
        std::pair<uint64_t, uint8_t> ptw_access = device->page_table_walk(buffer->addr + dst_offset, &size_bits);
        uint64_t pfn = ptw_access.first;
        uint64_t phys_addr = (pfn << 12) + offset;
        DBGPRINT("COPY_TO_DEV: hbuffer=%p, host_addr=%p, dst_offset=%ld, size=%ld\n", hbuffer, host_ptr, dst_offset, size);
        return device->upload(phys_addr, host_ptr, size);
    }

    DBGPRINT("COPY_TO_DEV: hbuffer=%p, host_addr=%p, dst_offset=%ld, size=%ld\n", hbuffer, host_ptr, dst_offset, size);
    return device->upload(buffer->addr, host_ptr, size);
}

extern int vx_copy_from_dev(void *host_ptr, vx_buffer_h hbuffer, uint64_t src_offset, uint64_t size)
{
    if (nullptr == hbuffer || nullptr == host_ptr)
        return -1;

    auto buffer = ((vx_buffer *)hbuffer);
    auto device = ((vx_device *)buffer->device);

    if ((src_offset + size) > buffer->size)
        return -1;

    DBGPRINT("COPY_FROM_DEV: hbuffer=%p, host_addr=%p, src_offset=%ld, size=%ld\n", hbuffer, host_ptr, src_offset, size);

    std::cout << "copy from virtual " << buffer->addr << std::endl;
    uint64_t offset = buffer->addr % RAM_PAGE_SIZE;
    uint64_t size_bits;
    std::pair<uint64_t, uint8_t> ptw_access = device->page_table_walk(buffer->addr + src_offset, &size_bits);
    uint64_t pfn = ptw_access.first;
    std::cout << "copy from physical " << (pfn << 12) + offset << std::endl;
    return device->download(host_ptr, (pfn << 12) + offset, size);
}

extern int vx_start(vx_device_h hdevice, vx_buffer_h hkernel, vx_buffer_h harguments)
{
    if (nullptr == hdevice || nullptr == hkernel || nullptr == harguments)
        return -1;

    DBGPRINT("START: hdevice=%p, hkernel=%p, harguments=%p\n", hdevice, hkernel, harguments);

    auto device = ((vx_device *)hdevice);
    auto kernel = ((vx_buffer *)hkernel);
    auto arguments = ((vx_buffer *)harguments);

    vx_stack_alloc(hdevice);
    return device->start(kernel->addr, arguments->addr);
}

extern int vx_stack_alloc(vx_device_h hdevice) {
    std::cout << "stack allocation: " << std::endl;
    vx_buffer_h stack_buff = nullptr;
    uint32_t total_threads    = NUM_CORES * NUM_WARPS * NUM_THREADS;
    uint64_t total_stack_size = STACK_SIZE * total_threads;
    uint64_t stack_end        = STACK_BASE_ADDR - total_stack_size;
    // Reserve Stack Pages
    CHECK_ERR(vx_mem_reserve(hdevice, stack_end, total_stack_size, VX_PAGE_VALID_ABSENT, &stack_buff), {
        return err;
    });
    return 0;
}

extern int vx_ready_wait(vx_device_h hdevice, uint64_t timeout)
{
    if (nullptr == hdevice)
        return -1;

    DBGPRINT("READY_WAIT: hdevice=%p, timeout=%ld\n", hdevice, timeout);

    auto device = ((vx_device *)hdevice);

    return device->ready_wait(timeout);
}

extern int vx_dcr_read(vx_device_h hdevice, uint32_t addr, uint32_t *value)
{
    if (nullptr == hdevice || NULL == value)
        return -1;

    auto device = ((vx_device *)hdevice);

    uint32_t _value;

    CHECK_ERR(device->dcr_read(addr, &_value), {
        return err;
    });

    DBGPRINT("DCR_READ: hdevice=%p, addr=0x%x, value=0x%x\n", hdevice, addr, _value);

    *value = _value;

    return 0;
}

extern int vx_dcr_write(vx_device_h hdevice, uint32_t addr, uint32_t value)
{
    if (nullptr == hdevice)
        return -1;

    DBGPRINT("DCR_WRITE: hdevice=%p, addr=0x%x, value=0x%x\n", hdevice, addr, value);

    auto device = ((vx_device *)hdevice);

    return device->dcr_write(addr, value);
}

extern int vx_mpm_query(vx_device_h hdevice, uint32_t addr, uint32_t core_id, uint64_t *value)
{
    if (nullptr == hdevice)
        return -1;

    auto device = ((vx_device *)hdevice);

    uint64_t _value;

    CHECK_ERR(device->mpm_query(addr, core_id, &_value), {
        return err;
    });

    DBGPRINT("MPM_QUERY: hdevice=%p, addr=0x%x, core_id=%d, value=0x%lx\n", hdevice, addr, core_id, _value);

    *value = _value;

    return 0;
}