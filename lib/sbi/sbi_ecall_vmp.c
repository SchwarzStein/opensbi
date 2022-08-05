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
#include <sbi/sbi_hart.h>
#include <sbi/sbi_hartmask.h>
#include <sbi/sbi_hsm.h>
#include <sbi/sbi_init.h>
#include <sbi/sbi_ipi.h>
#include <sbi/sbi_scratch.h>
#include <sbi/sbi_system.h>
#include <sbi/sbi_timer.h>
#include <sbi/sbi_console.h>

#define VMP_CONFIG_REG_COUNT 16

struct vmp_config_reg {
    uint8_t r:1;
    uint8_t w:1;
    uint8_t x:1;
    uint8_t a:2;
    uint8_t rsv:2;
    uint8_t l:1;
};

typedef struct vmp_config_registers
{
    struct vmp_config_reg vmp_reg[VMP_CONFIG_REG_COUNT];
} vmp_config_reg_t;


typedef struct vmp_address_registers
{
    unsigned long vmp_address_reg[VMP_CONFIG_REG_COUNT];
} vmp_address_reg_t;

enum {
    SBI_EXT_VMP_CREATE_REGION,
    SBI_EXT_VMP_SET_REGION,
    SBI_EXT_VMP_SAVE_CLEAR_CONTEXT,
    SBI_EXT_VMP_RESTORE_CONTEXT,
    SBI_EXT_VMP_CREATE_REGION
};

int sbi_ecall_restore_context(unsigned long cluster);
int sbi_ecall_save_clear_context(unsigned long cluster);
int sbi_ecall_set_region(int region, uint8_t perms);
int sbi_ecall_vmp_create_region(unsigned long start, unsigned long end, uint8_t perms);

static int sbi_ecall_vmp_handler(unsigned long extid, unsigned long funcid,
				 const struct sbi_trap_regs *regs,
				 unsigned long *out_val,
				 struct sbi_trap_info *out_trap)
{
    int ret = 0;
	struct sbi_scratch *scratch = sbi_scratch_thishart_ptr();
	ulong smode = (csr_read(CSR_MSTATUS) & MSTATUS_MPP) >>
			MSTATUS_MPP_SHIFT;
    vmp_address_reg_t *var_ptr = 0;
    vmp_config_reg_t  *vcf_ptr = 0;

    switch (funcid) {
        case SBI_EXT_VMP_CREATE_REGION:
            ret = 0;//sbi_ecall_vmp_create_region();
            break;
        case SBI_EXT_VMP_SET_REGION:
            ret = 0;//sbi_ecall_set_region();
            break;
        case SBI_EXT_VMP_SAVE_CLEAR_CONTEXT:
            ret = 0;//sbi_ecall_save_clear_context();
            break;
       case SBI_EXT_VMP_RESTORE_CONTEXT:
            ret = 0;//sbi_ecall_restore_context();
            break;
       default:
            ret = SBI_ENOTSUPP;
    }

}

