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

#define uint8_t unsigned char

enum sbi_vmp_command {
    SBI_VMP_DESTROY_REGION,
    SBI_VMP_CREATE_REGION,
    SBI_VMP_MODIFY_REGION,
    SBI_VMP_RESTORE_CONTEXT,
    SBI_VMP_SAVE_CLEAR_CONTEXT
};

#ifndef NULL
#define NULL (0)
#endif

#define VMP_CONFIG_REG_COUNT 16
#define VMP_CFG_BASE_ADDRESS    0x378
#define VMP_ADDR_BASE_ADDRESS   0x380

#define VMP_MAX_REGION_ENTRIES(value) ((value/2) + 1)
#define VMP_MAX_REGIONS_COUNT VMP_MAX_REGION_ENTRIES(VMP_CONFIG_REG_COUNT)

#define ALIGN8(value)  (value &=  ~(0x007))
#define ALIGN16(value) (value &=  ~(0X0ff))
#define ALIGN64(value) (value &= ~(0x0fff))
#define ALIGN4K(value) (value &=  ~(04095))

#define VMP_CFG_OFF         0b000
#define VMP_CFG_TOR         0b001
#define VMP_CFG_NA4         0b010
#define VMP_CFG_NAPOT       0b011
#define VMP_CFG_PERM_MASK   0x07
#define VMP_CFG_PERM(byte) ((0xFF & byte & VMP_CFG_PERM_MASK))
#define VMP_CFG_PERMISSIVE 0x07
#define VMP_CFG_NON_PERMISSIVE 0x00
#define VMP_CFG_IS_PERMISSIVE(byte) (VMP_CFG_PERMISSIVE == VMP_CFG_PERM(byte))

#define R 0x01
#define W 0x02
#define X 0x04
#define PERM(r,w,x) (r|x|w)
#define MB ((unsigned long)(1024*1024*1024))

struct vmp_region {
    unsigned long   start;
    unsigned long   end;
    uint8_t         perm;
    bool            active;
    struct vmp_region *next;
};

struct region {
    unsigned long   start;
    unsigned long   size;
    uint8_t         perm;
};

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

typedef struct vmp_config_registers {
    struct vmp_config_reg vmp_reg[VMP_CONFIG_REG_COUNT];
} vmp_config_reg_t;

typedef struct vmp_address_registers {
    unsigned long vmp_address_reg[VMP_CONFIG_REG_COUNT];
} vmp_address_reg_t;

/*
 *  static vmp_address_reg_t system_address_registers = {{0}};
 *  static vmp_config_reg_t  system_configuration_registers = {0};
 *  #define VMP_CFG_BASE_ADDRESS    (&system_configuration_registers)
 *  #define VMP_ADDR_BASE_ADDRESS   (&system_address_registers)
 * */

int sbi_ecall_vmp_restore_context(unsigned long buffer, unsigned long size);
int sbi_ecall_vmp_save_clear_context(unsigned long buffer, unsigned long size);
int sbi_ecall_vmp_create_region(unsigned long start, unsigned long end, uint8_t perms);
int sbi_ecall_vmp_set_region(unsigned long start, unsigned long size, uint8_t perms);
int sbi_ecall_vmp_destroy_region(unsigned long start, unsigned long end);

void vmp_reg_to_struct_region(struct vmp_region *ptr, int count)
{
    vmp_address_reg_t *addr_regs = (vmp_address_reg_t*)(VMP_ADDR_BASE_ADDRESS);
    vmp_config_reg_t *cfg_regs = (vmp_config_reg_t*)(VMP_CFG_BASE_ADDRESS);
    unsigned long start = 0;

    int end = (count < VMP_CONFIG_REG_COUNT) ? count : VMP_CONFIG_REG_COUNT;

    for(int i=0; i < end && ptr; i++) {
        ptr->perm   = cfg_regs->vmp_reg[i].byte & VMP_CFG_PERM_MASK;
        if (cfg_regs->vmp_reg[i].a != 0) {
            ptr->active = true;
        }
        ptr->end    = addr_regs->vmp_address_reg[i];
        if (ptr->end == 0) start = 0;
        ptr->start  = start;
        start       = ptr->end;
        ptr = ptr->next;
    }
}

/*assume the regions are ordered linearly from 0 */
void vmp_struct_region_to_reg(struct vmp_region *ptr, int count)
{
    vmp_address_reg_t *addr_regs = (vmp_address_reg_t*)(VMP_ADDR_BASE_ADDRESS);
    vmp_config_reg_t *cfg_regs   = (vmp_config_reg_t*)(VMP_CFG_BASE_ADDRESS);

    int end = (count < VMP_CONFIG_REG_COUNT) ? count : VMP_CONFIG_REG_COUNT;
    struct vmp_config_reg reg = {0};

    for (int i=0; ((i < end) && ptr); i++) {
    //    if (!ptr->active && i != 0) break;

        addr_regs->vmp_address_reg[i] = ptr->end;
        reg.byte = (ptr->perm & VMP_CFG_PERM_MASK);
        if (ptr->active)
            reg.a = VMP_CFG_TOR;
        else
            reg.a = 0x0;
        cfg_regs->vmp_reg[i] = reg;
        ptr = ptr->next;
    }
}

void vmp_initialize_struct_region_array(struct vmp_region *region, int count)
{
    if (count == 0 || !region) return;

    for(int i=0; i < count-1; i++) {
        region[i].start = 0;
        region[i].end   = 0;
        region[i].perm  = 0;
        region[i].active = false;
        region[i].next = &region[i+1];
    }
    region[count-1].start   = 0;
    region[count-1].end     = 0;
    region[count-1].perm    = 0;
    region[count-1].active  = false;
    region[count-1].next = 0;
}

void vmp_shift_regions_up(struct vmp_region *base)
{
    struct vmp_region *tmp = base;
    struct vmp_region to_move = *base;
    struct vmp_region to_store;

    if (!tmp->next) return;

    base->active = 0;
    base->start = 0;
    base->perm = 0;
    base->end = 0;

    while (tmp->next) {
        to_store = *tmp->next;
        tmp->next->active = to_move.active;
        tmp->next->start  = to_move.start;
        tmp->next->perm   = to_move.perm;
        tmp->next->end    = to_move.end;
        to_move = to_store;
        tmp = tmp->next;
    }

}

void vmp_shift_regions_down(struct vmp_region *base)
{
    struct vmp_region *tmp = base;
    while (tmp->next) {
        tmp->active = tmp->next->active;
        tmp->start  = tmp->next->start;
        tmp->perm   = tmp->next->perm;
        tmp->end    = tmp->next->end;
        tmp = tmp->next;
    }
    tmp->start   = 0;
    tmp->end     = 0;
    tmp->perm    = 0;
    tmp->active  = false;
}

void vmp_reset_struct_region(struct vmp_region *region)
{
    if (!region) return;
    region->start   = 0;
    region->end     = 0;
    region->perm    = 0;
    region->active  = false;
}

struct vmp_region* vmp_split_region(struct vmp_region *base, unsigned long start, unsigned long end, int count)
{
    /*check how many shifts may be needed*/
    struct vmp_region *target;
    struct vmp_region *t=base;
    struct vmp_region original;
    int free_positions = 0;
    int needed = 0;

    if (end <= start || count == 0) return NULL;
    if (start == base->start && end == base->end) return base;
    for (int i=0;  ((i < VMP_MAX_REGIONS_COUNT) && (i < count) && t); i++, t=t->next) {
        if (start >= t->start && end <= t->end) {
            target = t;
        }
        if (t->active == true)
            free_positions++;
    }
    free_positions = VMP_MAX_REGIONS_COUNT - free_positions;

    if (!target) return false;

    needed = 1;
    if (start != target->start && end != target->end)
        needed = 2;

    original = *target;
    for (int i=0; i<needed; i++) {
        vmp_shift_regions_up(target);
    }

    if (needed == 1 ) {
        /*when only a single shift is needed, it is because;
         * 1. starts are same
         * 2. ends are same
         * */
        if (original.start == start) {
            target->start = start;
            target->end = end;
            target->next->start = end;
        } else if(original.end == end) {
            target->start = original.start;
            target->end = start;
            target->next->start = start;
            target = target->next;
        }
    } else if (needed == 2) {
        target->end = start;
        target->next->start = start;
        target->next->end = end;
        target->next->next->start = end;
        target = target->next;
    }

    return target;
}

void vmp_print_regions()
{
    struct vmp_region regions[VMP_MAX_REGIONS_COUNT];
    vmp_initialize_struct_region_array(regions, VMP_MAX_REGIONS_COUNT);
    vmp_reg_to_struct_region(regions, VMP_MAX_REGIONS_COUNT);

    struct vmp_region *tmp = regions;

    while (tmp) {
        //printf("0x%x %ld-%ld\n", tmp->perm, tmp->start/(MB), tmp->end/(MB));
        tmp = tmp->next;
    }
}

int vmp_fit_region(unsigned long start, unsigned long end)
{
    vmp_address_reg_t *addr_regs = (vmp_address_reg_t*) VMP_ADDR_BASE_ADDRESS;
    vmp_config_reg_t   *cfg_regs = (vmp_config_reg_t*)  VMP_CFG_BASE_ADDRESS;
    struct vmp_config_reg *cfg_reg = 0;
    unsigned long lower=0;
    unsigned long upper=0;

    upper = addr_regs->vmp_address_reg[0];

    int max_index = 0;
    int tmp = 0;
    for(int i=0; i<VMP_MAX_REGIONS_COUNT; i++) {
        if (addr_regs->vmp_address_reg[i] || cfg_regs->vmp_reg[i].a != 0) {
            max_index = i;
        }
    }

    while (max_index > 0) {
        if (start >= addr_regs->vmp_address_reg[max_index-1] &&
                end <= addr_regs->vmp_address_reg[max_index]) {
            tmp = max_index;
        }
        max_index -= 1;
    }

    if (start >= 0 && end <= addr_regs->vmp_address_reg[1])
        tmp = 0;

    if(max_index > 0) return tmp;

    for(int i=0; i<VMP_MAX_REGIONS_COUNT; i++) {
        upper = addr_regs->vmp_address_reg[i];
        cfg_reg = &cfg_regs->vmp_reg[i];
        if (cfg_reg->a == VMP_CFG_OFF && (addr_regs->vmp_address_reg[i] == 0x0)) {
            return  (i == 0 && start != 0x0 )? 1: i;
        }
        if (lower == 0 && upper == 0 && cfg_reg->a != VMP_CFG_TOR) {
            return  (i == 0)? 1: i;
        }
        if (lower <= start && end <= upper)
            return i;
        lower = upper;
    }
    return -1;
}

int vmp_create_region(unsigned long start, unsigned long end, uint8_t perm)
{
    struct vmp_region regions[VMP_MAX_REGIONS_COUNT];
    struct vmp_region *tmp = 0;
    struct vmp_region *new = 0;
    int index = 0;

    vmp_initialize_struct_region_array(regions, VMP_MAX_REGIONS_COUNT);
    vmp_reg_to_struct_region(regions, VMP_MAX_REGIONS_COUNT);
    index = vmp_fit_region(start, end);

    if (index < 0 || index >= VMP_MAX_REGIONS_COUNT) return -1;

    tmp = &regions[index];

    if (! ((start >= tmp->start && end <= tmp->end) ||
                (tmp->start == 0 && tmp->end == 0)) ) return -1;

    if ( ( tmp->perm != 0x7 && tmp->active)) return -1;

    if ( (tmp->active == true) && (tmp->perm == 0x00)) return -1;

    if (start >= tmp->start && end <= tmp->end ) {
        new = vmp_split_region(tmp, start, end, VMP_MAX_REGIONS_COUNT - index);
        if (!new) return -1;
    } else if(start >= tmp->start && tmp->end == 0 ) {
        new = tmp;
    }
    new->start = start;
    new->end = end;
    new->perm = perm & VMP_CFG_PERM_MASK;
    new->active = true;

    if (index == 1) {
        if (regions[0].end != start && regions[0].end == 0 && regions[0].active == false) {
            regions[0].end = start;
        }
    }

    vmp_struct_region_to_reg(regions, VMP_MAX_REGIONS_COUNT);
    vmp_print_regions();
    //printf("------------------------------------------------------------\n");
    return 0;
}

int vmp_region_from_range(struct vmp_region *regions, unsigned long start, unsigned long end)
{
    int index = 0;
    if (!regions) return -1;

    struct vmp_region *tmp = regions;

    while(tmp) {
        if (start >= tmp->start && end <= tmp->end)
            return index;
        tmp = tmp->next;
        index += 1;
    }
    return -1;
}

int vmp_destroy_region(unsigned long start, unsigned long end)
{
    struct vmp_region regions[VMP_MAX_REGIONS_COUNT];
    struct vmp_region *region = 0;
    int index = 0;

    vmp_initialize_struct_region_array(regions, VMP_MAX_REGIONS_COUNT);
    vmp_reg_to_struct_region(regions, VMP_MAX_REGIONS_COUNT);

    index = vmp_region_from_range(regions, start, end);
    if (index < 0) return -1;

    region =  &regions[index];

    if (index == 0 || index == (VMP_MAX_REGIONS_COUNT)){
    } else {
        /*match lower and upper*/
        if ( regions[index-1].perm == 0  && regions[index+1].perm == 0) {
            regions[index-1].end = regions[index+1].end;
            vmp_shift_regions_down(region);
            vmp_shift_regions_down(region);
        } else if ( regions[index-1].perm == 0 ) {
            regions[index-1].end = regions[index+1].start;
            vmp_shift_regions_down(region);
        }
    }
    vmp_struct_region_to_reg(regions, VMP_MAX_REGIONS_COUNT);
    return 0;
}

void vmp_clear_all_regions()
{
    struct vmp_region regions[VMP_MAX_REGIONS_COUNT];

    vmp_initialize_struct_region_array(regions, VMP_MAX_REGIONS_COUNT);
    vmp_struct_region_to_reg(regions, VMP_MAX_REGIONS_COUNT);
}

void *l_memcpy(void *dest, const void *src, size_t n)
{
    if (!dest || !src) return 0;

    for ( size_t i=0; i < n; i++)
        ((char*)dest)[i] = ((char*)src)[i];
    return dest;
}

int vmp_save_and_clear_context(void *buffer, unsigned long size)
{
    vmp_address_reg_t *addr_regs = (vmp_address_reg_t*)(VMP_ADDR_BASE_ADDRESS);
    vmp_config_reg_t *cfg_regs = (vmp_config_reg_t*)(VMP_CFG_BASE_ADDRESS);
    unsigned char *ptr = buffer;

    unsigned long local_size = sizeof(vmp_config_reg_t) + sizeof(vmp_address_reg_t);

    if (size < local_size) return -1;

    l_memcpy(ptr, addr_regs, sizeof(vmp_address_reg_t));
    l_memcpy(&ptr[sizeof(vmp_address_reg_t)], cfg_regs, sizeof(vmp_config_reg_t));

    for( int i=0; i < sizeof(vmp_address_reg_t); i++)
        ((unsigned char*)addr_regs)[i] = 0;

    for( int i=0; i < sizeof(vmp_config_reg_t); i++)
        ((unsigned char*)cfg_regs)[i] = 0;

    return local_size;
}

int vmp_restore_context( void *buffer, unsigned long size)
{
    vmp_address_reg_t *addr_regs = (vmp_address_reg_t*)(VMP_ADDR_BASE_ADDRESS);
    vmp_config_reg_t *cfg_regs = (vmp_config_reg_t*)(VMP_CFG_BASE_ADDRESS);

    unsigned char *dst = (unsigned char*) addr_regs;
    unsigned char *src = (unsigned char*) buffer;

    unsigned long local_size = sizeof(vmp_config_reg_t) + sizeof(vmp_address_reg_t);

    if (size < local_size) return -1;

    l_memcpy( dst, src, sizeof(vmp_address_reg_t) );
    dst = (unsigned char *)cfg_regs;
    l_memcpy( dst, &src[sizeof(vmp_address_reg_t)], sizeof(vmp_config_reg_t));
    return 0;
}

bool vmp_regions_is_zero( struct vmp_region *array)
{
    if (!array) return false;

    for( struct vmp_region *t = array; t; t=t->next) {
        if ( !( t->start == 0 && t->end == 0 && t->perm == 0 && t->active == 0))
            return false;
    }
    return true;
}

bool vmp_regions_compare( struct vmp_region *first, struct vmp_region *second, int count)
{
    int i;
    if (!first|| !second ) return false;

    for( i=0; first && second  && i < count; first=first->next, second=second->next, i++) {
        if (first->start  != second->start &&
            first->end    != second->end &&
            first->active != second->active &&
            first->perm   != second->perm )
            return false;
    }
    if (i!=count) return false;
    return true;
}

int sbi_ecall_vmp_restore_context(unsigned long buffer, unsigned long size)
{
    if(!buffer) return 0;
    return 1;
}

int sbi_ecall_vmp_save_clear_context(unsigned long buffer, unsigned long size)
{
    if(!buffer) return 0;
    return 1;
}

int sbi_ecall_vmp_set_region(unsigned long start, unsigned long size, uint8_t perms)
{
    return 1;
}

static int sbi_ecall_vmp_handler(unsigned long extid, unsigned long funcid,
				const struct sbi_trap_regs *regs,
				unsigned long *out_val,
				struct sbi_trap_info *out_trap)
{
    int ret = 0;
    switch (funcid) {
        case SBI_VMP_CREATE_REGION:
            ret = sbi_ecall_vmp_create_region(regs->a0, regs->a1, regs->a2);
            break;
        case SBI_VMP_MODIFY_REGION:
            ret = sbi_ecall_vmp_set_region(regs->a0, regs->a1, (0xFF & regs->a2));
            break;
        case SBI_VMP_SAVE_CLEAR_CONTEXT:
            ret = sbi_ecall_vmp_save_clear_context(regs->a0, regs->a1);
            break;
       case SBI_VMP_RESTORE_CONTEXT:
            ret = sbi_ecall_vmp_restore_context(regs->a0, regs->a1);
            break;
       case SBI_VMP_DESTROY_REGION:
            ret = sbi_ecall_vmp_destroy_region(regs->a0, regs->a1);
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
