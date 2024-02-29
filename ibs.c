// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Author: Karim Manaouil <k.manaouil@gmail.com>
 *
 *  Copyright (C) 2024
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/smp.h>
#include <asm/cpuid.h>
#include <asm/msr.h>
#include <asm/nmi.h>
#include <asm/apic.h>

#define	CPUID_IBS_MASK		(1 << 10)
#define IBS_LVT_MSR		0xC001103A
#define IBS_LVT_OFFMASK		0xf

#define IBS_OP_CTRL_REG		0xC0011033
#define IBS_OP_RIP_ADDR_REG	0xC0011034
#define IBS_OP_DATA_REG		0xC0011035	/* Branch */
#define IBS_OP_DATA2_REG	0xC0011036	/* DRAM and MMIO */
#define IBS_OP_DATA3_REG	0xC0011037	/* Cache info */
#define IBS_OP_LADDR_REG	0xC0011038
#define IBS_OP_PADDR_REG	0xC0011039
#define IBS_OP_BR_ADDR_REG	0xC001103B

#define IBS_OP_L3MISSONLY	BIT_ULL_MASK(16)
#define IBS_OP_ENABLE		BIT_ULL_MASK(17)
#define IBS_OP_VALID		BIT_ULL_MASK(18)
#define IBS_OP_CNT_CTL		BIT_ULL_MASK(19) /* 1: count dispatched ops */

#define IBS_DATA3_CACHE_MISS	BIT_ULL_MASK(7)
#define IBS_DATA3_LADDR_VALID	BIT_ULL_MASK(17)
#define IBS_DATA3_PADDR_VALID	BIT_ULL_MASK(18)
#define IBS_DATA3_MISS_LAT	GENMASK_ULL(47, 32)

#define IBS_DATA3_LD_OP		0
#define IBS_DATA3_ST_OP		1

enum ibs_pcp_state {
	IBS_PCP_STATE_FAILED,
	IBS_PCP_STATE_IDLE,
	IBS_PCP_STATE_RUNNING
};

struct ibs_pcp {
	enum ibs_pcp_state state;
	unsigned long long op_ctl;	/* scratch register */
};

struct ibs_raw_sample {
	unsigned long long data;
	unsigned long long data2;
	unsigned long long data3;
	unsigned long long linaddr;
	unsigned long long physaddr;
};

struct ibs_sample {
	struct ibs_raw_sample raw;
	struct task_struct *owner;
	int cpu;
	unsigned long virt;
	unsigned long physaddr;
	int ldst;	/* 0 for load, 1 for store */
	int cache_miss;
	int miss_lat; /* Only for reads */
};

DEFINE_PER_CPU(struct ibs_pcp, ibs_cpus);

/* Make sure no preemption is allowed */
static void read_ibs_sample(struct ibs_raw_sample *s)
{
	rdmsrl(IBS_OP_DATA_REG, s->data);
	rdmsrl(IBS_OP_DATA2_REG, s->data2);
	rdmsrl(IBS_OP_DATA3_REG, s->data3);
	rdmsrl(IBS_OP_LADDR_REG, s->linaddr);
	rdmsrl(IBS_OP_PADDR_REG, s->physaddr);
}

static inline int ibs_ldst(struct ibs_raw_sample *s)
{
	if (s->data3 & IBS_DATA3_LD_OP)
		return 0;
	if (s->data3 & IBS_DATA3_ST_OP)
	       return 1;
	return -1;
}

static void process_ibs_sample(struct ibs_raw_sample *s, struct ibs_sample *n)
{
	memcpy(&n->raw, s, sizeof(*s));

	n->ldst = ibs_ldst(s);

	n->virt = -1;
	if (s->data3 & IBS_DATA3_LADDR_VALID)
		n->virt = s->linaddr;

	n->physaddr = -1;
	if (s->data3 & IBS_DATA3_PADDR_VALID)
		n->physaddr = s->physaddr;

	if (s->data3 & IBS_DATA3_CACHE_MISS)
		n->cache_miss = 1;

	if (n->ldst == 0)
		n->miss_lat = (s->data3 & IBS_DATA3_MISS_LAT) >> 32;
}

static int get_ibs_lvtoff(void)
{
	unsigned long long msr_val;
	int offset;

	rdmsrl(IBS_LVT_MSR, msr_val);
	offset = msr_val & IBS_LVT_OFFMASK;
	return offset;
}

static int setup_ibs_lvt(void)
{
	int ibs_lvtoff;
	int retval;

	ibs_lvtoff = get_ibs_lvtoff();
	if (ibs_lvtoff < 0)
		return ibs_lvtoff;

	preempt_disable();
	retval = setup_APIC_eilvt(ibs_lvtoff, 0, APIC_EILVT_MSG_NMI, 0);
	preempt_enable();

	return retval;
}

static void clear_ibs_lvt(void)
{
	int ibs_lvtoff;

	ibs_lvtoff = get_ibs_lvtoff();

	preempt_disable();
	setup_APIC_eilvt(ibs_lvtoff, 0, APIC_EILVT_MSG_FIX, 1);
	preempt_enable();
}

static void disable_ibs(void)
{
	unsigned long long ibs_ctl;

	rdmsrl(IBS_OP_CTRL_REG, ibs_ctl);
	ibs_ctl &= ~IBS_OP_ENABLE;
	wrmsrl(IBS_OP_CTRL_REG, ibs_ctl);
}

static void disable_clear_ibs(void *nil)
{
	disable_ibs();
	clear_ibs_lvt();
}

static void start_ibs(void *nil)
{
	int cpu = raw_smp_processor_id();
	struct ibs_pcp	*ibs_cpu = per_cpu_ptr(&ibs_cpus, cpu);
	unsigned long long ibs_ctl;
	int retval;

	retval = setup_ibs_lvt();
	if (retval) {
		ibs_cpu->state = IBS_PCP_STATE_FAILED;
		pr_err("ibs lvt setup failed (%d) on cpu %d\n", retval, cpu);
		return;
	}

	rdmsrl(IBS_OP_CTRL_REG, ibs_ctl);
	ibs_ctl &= ~IBS_OP_VALID;
	ibs_ctl |= IBS_OP_ENABLE | IBS_OP_CNT_CTL;
	ibs_ctl |= 1 << 10;
	pr_info("%s: ibs_ctl=%llx\n", __func__, ibs_ctl);
	wrmsrl(IBS_OP_CTRL_REG, ibs_ctl);

	ibs_cpu->state = IBS_PCP_STATE_RUNNING;
}

static int ibs_nmi_handler(unsigned int val, struct pt_regs *regs)
{
	unsigned long long ibs_ctl;
	struct ibs_pcp *ibs_cpu;
	struct ibs_raw_sample s;
	struct ibs_sample n;
	int cpu = raw_smp_processor_id();

	rdmsrl(IBS_OP_CTRL_REG, ibs_ctl);
	pr_info("%s: ibs_ctl=%llx\n", __func__, ibs_ctl);
	if (!(ibs_ctl & IBS_OP_VALID))
		return NMI_HANDLED;

	read_ibs_sample(&s);
	n.cpu = cpu;

	if (ibs_ldst(&s))
		process_ibs_sample(&s, &n);

	pr_info("cpu=%d, ldst=%d, virt=%lx, phys=%lx, cache_miss=%d, miss_lat=%d\n",
			cpu, n.ldst, n.virt, n.physaddr, n.cache_miss, n.miss_lat);

	disable_ibs();
	ibs_cpu = per_cpu_ptr(&ibs_cpus, cpu);
	ibs_cpu->state = IBS_PCP_STATE_IDLE;

	return NMI_HANDLED;
}

static int register_ibs_nmi_handler(void)
{
	return register_nmi_handler(NMI_LOCAL, ibs_nmi_handler, 0, "ibs");
}

static int check_ibs_exists(void)
{
	unsigned int cpuid_ibs;

	cpuid_ibs = cpuid_ecx(0x80000001);
	if (cpuid_ibs & CPUID_IBS_MASK)
		return 1;
	return 0;
}

static int __init ibs_init(void)
{
	int cpu;
	int retval;
	struct ibs_pcp *ibs_cpu;

	if (!check_ibs_exists())
		return -1;

	pr_info("Initializing amd ibs\n");

	retval = register_ibs_nmi_handler();
	if (retval) {
		pr_err("Could not register IBS NMI handler %d\n", retval);
		return retval;
	}

	on_each_cpu_cond_mask(NULL, start_ibs, NULL, 1, cpu_online_mask);

	for_each_present_cpu(cpu) {
		ibs_cpu = per_cpu_ptr(&ibs_cpus, cpu);
		if (ibs_cpu->state == IBS_PCP_STATE_FAILED) {
			pr_err("ibs init failed on cpu %d\n", cpu);
			return -1;
		}
	}
	return 0;
}

static void __exit ibs_exit(void)
{
	unregister_nmi_handler(NMI_LOCAL, "ibs");
	on_each_cpu_cond_mask(NULL, disable_clear_ibs, NULL, 0, cpu_online_mask);
}

module_init(ibs_init);
module_exit(ibs_exit);

MODULE_LICENSE("GPL");
