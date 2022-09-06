/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2020 
 *
 * Authors:
 *   Pamenas Murimi <mpamenas@gmail.com>
 */

#include <sbi/riscv_asm.h>
#include <sbi/riscv_barrier.h>
#include <sbi/riscv_encoding.h>
#include <sbi/riscv_atomic.h>
#include <sbi/sbi_bitops.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_domain.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_ecall_interface.h>
#include <sbi/sbi_ecall.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_hartmask.h>
#include <sbi/sbi_hsm.h>
#include <sbi/sbi_init.h>
#include <sbi/sbi_ipi.h>
#include <sbi/sbi_scratch.h>
#include <sbi/sbi_system.h>
#include <sbi/sbi_timer.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_trap.h>

#define VMP_CONFIG_REG_COUNT 16

#define VMP_CFG_BASE_ADDRESS    0x378
#define VMP_ADDR_BASE_ADDRESS   0x380

#define ALIGN8(value)  (value &=  ~(0x007))
#define ALIGN16(value) (value &=  ~(0X0ff))
#define ALIGN64(value) (value &= ~(0x0fff))
#define ALIGN4K(value) (value &=  ~(04095))

#define VMP_MAX_REGION_ENTRIES(value) ( ((value-2)/2) + 2 )
#define VMP_MAX_REGIONS_COUNT VMP_MAX_REGION_ENTRIES(VMP_CONFIG_REG_COUNT)
struct vmp_config_reg {
    union {
        struct {
            uint8_t r:1;/*read allowed*/
            uint8_t w:1;/*write allowed*/
            uint8_t x:1;/*execute allowed*/
            uint8_t a:2;/*address mode, 00 off, 01 TOR, 10 NA4, 11 NAPOT*/
            uint8_t rsv:2;
            uint8_t l:1;
        };
        uint8_t byte;
    };
};

#define VMP_CFG_OFF         0b000
#define VMP_CFG_TOR         0b001
#define VMP_CFG_NA4         0b010
#define VMP_CFG_NAPOT       0b011
#define VMP_CFG_PERM_MASK   0x07
#define VMP_CFG_PERM(byte) ((0xFF & byte & VMP_CFG_PERM_MASK))
#define VMP_CFG_PERMISSIVE 0x07
#define VMP_CFG_NON_PERMISSIVE 0x00
#define VMP_CFG_IS_PERMISSIVE(byte) (VMP_CFG_PERMISSIVE == VMP_CFG_PERM(byte))

typedef struct vmp_config_registers
{
    struct vmp_config_reg vmp_reg[VMP_CONFIG_REG_COUNT];
} vmp_config_reg_t;

typedef struct vmp_address_registers
{
    unsigned long vmp_address_reg[VMP_CONFIG_REG_COUNT];
} vmp_address_reg_t;

enum sbi_vmp_command {
    SBI_VMP_CLEAR_REGION,
    SBI_VMP_CREATE_REGION,
    SBI_VMP_MODIFY_REGION,
    SBI_VMP_RESTORE_CONTEXT,
    SBI_VMP_SAVE_CLEAR_CONTEXT
};

int sbi_ecall_restore_context(unsigned long cluster, long regs, long cfg);
int sbi_ecall_save_clear_context(unsigned long cluster, long reg, long cfg);
int sbi_ecall_set_region(long region, uint8_t perms);
int sbi_ecall_vmp_create_region(unsigned long start, unsigned long end, uint8_t perms);

bool vmp_permission_matches(struct vmp_config_reg reg, uint8_t perm)
{
    uint8_t val = (reg.r | reg.w | reg.x) ;
    if (val == (VMP_CFG_PERM_MASK & perm))
        return true;
    return false;
}

int address_perm_exists(unsigned start, unsigned end, uint8_t perm)
{
    vmp_address_reg_t *addr_regs = (vmp_address_reg_t*)(VMP_ADDR_BASE_ADDRESS);
    vmp_config_reg_t *cfg_regs = (vmp_config_reg_t*)(VMP_CFG_BASE_ADDRESS);
    vmp_address_reg_t found={0};
    unsigned long lower=0;
    uint8_t val=0;
    int count = 0;
    
    for (int i=0; i < VMP_CONFIG_REG_COUNT; i+=2) {

        if ( cfg_regs->vmp_reg[i].a == VMP_CFG_OFF )
            continue;

        if ( lower <= start && 
             cfg_regs->vmp_reg[i].a == VMP_CFG_TOR && 
             addr_regs->vmp_address_reg[i]  <= end )
        {
            count += 1;
            found.vmp_address_reg[i] = 1;
        }
        
        lower = addr_regs->vmp_address_reg[i];

    }
    if (count != 1) return 0;

    for (int i=0; i<VMP_CONFIG_REG_COUNT; i++) {
        if (found.vmp_address_reg[i] == 1) {
            if ( vmp_permission_matches( cfg_regs->vmp_reg[i], perm) )
                return 1;
        }
    }

    return 0;
}

bool addresss_exists(unsigned start, unsigned end)
{
    vmp_address_reg_t *addr_regs = (vmp_address_reg_t*)(VMP_ADDR_BASE_ADDRESS);
    vmp_config_reg_t *cfg_regs = (vmp_config_reg_t*)(VMP_CFG_BASE_ADDRESS);
    unsigned long lower=0, upper;
    int i = 1;

   /* address matching starts from lowest entry to 
    * highest entry. If a match is found, we terminate
    * */ 
    do {
        upper =  addr_regs->vmp_address_reg[i];
        if ( (lower <= start && cfg_regs->vmp_reg[i-1].a == VMP_CFG_TOR) &&
             (end <= upper) && cfg_regs->vmp_reg[i].a == VMP_CFG_TOR) {
                 return true;
        }
        i = i + 1;
        lower = upper; 
    } while (i < VMP_CONFIG_REG_COUNT);

    return false;
}

bool address_spans_multiple_ranges(unsigned start, unsigned end )
{
    vmp_address_reg_t *addr_regs = (vmp_address_reg_t*)(VMP_ADDR_BASE_ADDRESS);
    vmp_config_reg_t *cfg_regs = (vmp_config_reg_t*)(VMP_CFG_BASE_ADDRESS);
    unsigned long lower=0, upper;
    int i = 0;

    do {
        upper = addr_regs->vmp_address_reg[i];
        if (cfg_regs->vmp_reg[i].a ==  VMP_CFG_OFF)
            break;

        if ((start < lower && end > lower ) ||
            (start >= lower && end > upper)) {
            return true;
        }
        i = i + 1;
        lower = addr_regs->vmp_address_reg[i-1];
    } while (i < VMP_CONFIG_REG_COUNT);

    return false;
}

int address_partially_spans_range( unsigned start, unsigned end)
{
    vmp_address_reg_t *addr_regs = (vmp_address_reg_t*)(VMP_ADDR_BASE_ADDRESS);
    vmp_config_reg_t *cfg_regs = (vmp_config_reg_t*)(VMP_CFG_BASE_ADDRESS);
    unsigned long lower=0, upper;
    int i = 0;

    do {
        upper = addr_regs->vmp_address_reg[i];
        if (cfg_regs->vmp_reg[i].a ==  VMP_CFG_OFF)
            break;

        if ((start > lower && end <= upper) ||
            (start >= lower && end < upper)) {
            return true;
        }
        i = i + 1;
        lower = addr_regs->vmp_address_reg[i-1];
    } while (i < VMP_CONFIG_REG_COUNT);

    return false;
}

int vmp_get_active_entries()
{
    vmp_config_reg_t *cfg_regs = (vmp_config_reg_t*)(VMP_CFG_BASE_ADDRESS);
    int count = 0;

    for (int i = 0; i < VMP_CONFIG_REG_COUNT; i++) {
        if ( cfg_regs->vmp_reg[i].a == VMP_CFG_TOR )
            count += 1;
    }
    return count;
}

int vmp_get_highest_used_entry()
{
    vmp_config_reg_t *cfg_regs = (vmp_config_reg_t*)(VMP_CFG_BASE_ADDRESS);
    int index = -1;

    for (int i=0; i < VMP_CONFIG_REG_COUNT; i++) {
        if (cfg_regs->vmp_reg[i].a == VMP_CFG_TOR)
            index = i;
    }

    return index;
}

int vmp_get_region_by_range(unsigned start, unsigned end)
{
    vmp_address_reg_t *addr_regs = (vmp_address_reg_t*)(VMP_ADDR_BASE_ADDRESS);
    vmp_config_reg_t *cfg_regs = (vmp_config_reg_t*)(VMP_CFG_BASE_ADDRESS);
    unsigned lower = 0, upper;
    bool upper_valid, lower_valid = true;

    for (int i=0; i < VMP_CONFIG_REG_COUNT; i++) {
        upper = addr_regs->vmp_address_reg[i];
        upper_valid = cfg_regs->vmp_reg[i].a ==  VMP_CFG_TOR ? true: false;
        if ( start >= lower && end <= upper ) {
            if (upper_valid && lower_valid)
                return i;
        }
        lower = upper;
    }
    return -1;
}

uint8_t vmp_get_range_permissions(unsigned start, unsigned end)
{
    vmp_address_reg_t *addr_regs = (vmp_address_reg_t*)(VMP_ADDR_BASE_ADDRESS);
    vmp_config_reg_t *cfg_regs = (vmp_config_reg_t*)(VMP_CFG_BASE_ADDRESS);
    unsigned lower = 0;

    for(int i=0; i < VMP_CONFIG_REG_COUNT; i++) {

       if( lower <= start && 
            (end <= addr_regs->vmp_address_reg[i]  && cfg_regs->vmp_reg[i].a == VMP_CFG_TOR)) 
                return (cfg_regs->vmp_reg[i].byte & VMP_CFG_PERM_MASK);
    }
    return 0;
}

uint8_t vmp_get_region_permission(int region)
{
    vmp_address_reg_t *addr_regs = (vmp_address_reg_t*)(VMP_ADDR_BASE_ADDRESS);
    vmp_config_reg_t *cfg_regs = (vmp_config_reg_t*)(VMP_CFG_BASE_ADDRESS);

    if(region >= VMP_MAX_REGIONS_COUNT)
        return 0;

    return  cfg_regs->vmp_reg[region].a ? (cfg_regs->vmp_reg[region].byte & VMP_CFG_PERM_MASK) : 0;
}

bool vmp_region_is_permissive(int region)
{
    vmp_address_reg_t *addr_regs = (vmp_address_reg_t*)(VMP_ADDR_BASE_ADDRESS);
    vmp_config_reg_t *cfg_regs = (vmp_config_reg_t*)(VMP_CFG_BASE_ADDRESS);
    
    if (region > (VMP_MAX_REGIONS_COUNT-1)) 
        return false;
    if (cfg_regs->vmp_reg[region].a == VMP_CFG_TOR && 
            VMP_CFG_IS_PERMISSIVE(cfg_regs->vmp_reg[region].byte) )
        return true;
    return false;
}

int vmp_get_lowest_index_region(unsigned long start)
{
    vmp_address_reg_t *addr_regs = (vmp_address_reg_t*)(VMP_ADDR_BASE_ADDRESS);
    vmp_config_reg_t *cfg_regs = (vmp_config_reg_t*)(VMP_CFG_BASE_ADDRESS);
    unsigned lower = 0;

    for (int i=0; i < VMP_CONFIG_REG_COUNT; i++) {
        if ( (start >= lower) && (start < addr_regs->vmp_address_reg[i]) && 
                (cfg_regs->vmp_reg[i].a == VMP_CFG_TOR) )
            return i;
        lower = addr_regs->vmp_address_reg[i];
    }
    return 0;
}

int sbi_ecall_vmp_create_region(unsigned long start, unsigned long size, uint8_t perms)
{
    vmp_address_reg_t *addr_regs = (vmp_address_reg_t*)(VMP_ADDR_BASE_ADDRESS);
    vmp_config_reg_t *cfg_regs = (vmp_config_reg_t*)(VMP_CFG_BASE_ADDRESS);
    unsigned end = start + size;
    int lowest_index; 
    int region = -1;
    int count = 1;
    int active_entries = 0;

    ALIGN8(end);

    if ( (active_entries = vmp_get_active_entries()) >= VMP_MAX_REGIONS_COUNT ) return -1; /*not possible*/

    if ( address_spans_multiple_ranges(start, end) ) return -1; /*exists*/
    /*check if the address spans any existing range*/

    if (addresss_exists(start,end)) {
        region = vmp_get_region_by_range(start, end);
        if ( vmp_region_is_permissive(region) == false ) return -1; 
    }   
    
    /*all conditions are fulfilled*/
    if (region == -1) region = vmp_get_region_by_range(start, end);

    if (region >=0 ) lowest_index = region;
    else lowest_index = vmp_get_lowest_index_region(start);

    if (lowest_index > VMP_MAX_REGIONS_COUNT) return -1; 

    if ((lowest_index == 0) || (lowest_index == VMP_MAX_REGIONS_COUNT)) {
        addr_regs->vmp_address_reg[lowest_index] = end;
        cfg_regs->vmp_reg[lowest_index].a = VMP_CFG_TOR;
        return 0;
    }

    /*two registers are needed to split the region*/
    unsigned range = addr_regs->vmp_address_reg[ lowest_index ];
    range = range - addr_regs->vmp_address_reg[ lowest_index + 1 ];
     
    if (range < size) return -1;

    /*
     * ensure that lower end is aligned
     * ensure that upper end is aligned
     * */
    if (addr_regs->vmp_address_reg[lowest_index] != start  || 
            addr_regs->vmp_address_reg[lowest_index+1] != end) {
        count += 1;
    }
    /*need an extra register*/ 
    if (count == 2 && (active_entries <= (VMP_MAX_REGIONS_COUNT-2)) ) {
        /*push upwards*/
    }
    /*assume registers are available 
    addr_regs->vmp_address_reg[lowest_index+1];
    addr_regs->vmp_address_reg[lowest_index+1];*/
}

int sbi_ecall_restore_context(unsigned long cluster, long regs, long cfg)
{
    if( cluster < 0 || regs <= 0 || cfg <= 0 ) return 0;
    return 1;
}

int sbi_ecall_save_clear_context(unsigned long cluster, long reg, long cfg)
{
    if( cluster < 0 || reg <= 0 || cfg <= 0 ) return 0;
    return 1;
}

int sbi_ecall_set_region(long region, uint8_t perms)
{
    if( region <= 0 || perms <= 0 ) return 0;
    return 1;
}
static int sbi_ecall_vmp_handler(unsigned long extid, unsigned long funcid,
				 const struct sbi_trap_regs *regs,
				 unsigned long *out_val,
				 struct sbi_trap_info *out_trap)
{
    int ret = 0;
//	struct sbi_scratch *scratch = sbi_scratch_thishart_ptr();
//	ulong smode = (csr_read(CSR_MSTATUS) & MSTATUS_MPP) >>
//	MSTATUS_MPP_SHIFT;
//  vmp_address_reg_t ex*var_ptr = 0;
//  vmp_config_reg_t  *vcf_ptr = 0;
    long cluster = regs->a1;
    switch (funcid) {
        case SBI_VMP_CREATE_REGION:
            ret = sbi_ecall_vmp_create_region(regs->a0, regs->a1, regs->a2);
            break;
        case SBI_VMP_MODIFY_REGION:
            ret = sbi_ecall_set_region(cluster, (0xFF & regs->a1));
            break;
        case SBI_VMP_SAVE_CLEAR_CONTEXT:
            ret = sbi_ecall_save_clear_context(cluster, regs->a1, regs->a2);
            break;
       case SBI_VMP_RESTORE_CONTEXT:
            ret = sbi_ecall_restore_context(cluster, regs->a1,(0xFF&regs->a2));
            break;
       default:
            ret = SBI_ENOTSUPP;
    }
    return ret;
}

struct sbi_ecall_extension ecall_vmp = {
    .extid_start    = SBI_EXT_VMP,
    .extid_end      = SBI_EXT_VMP,
    .handle         = sbi_ecall_vmp_handler,
};




struct vmp_region {
    unsigned start;
    unsigned end;
    uint8_t perm;
    bool active;

    struct vmp_region *next;
};
