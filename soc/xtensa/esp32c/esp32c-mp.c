/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Include esp-idf headers first to avoid redefining BIT() macro */
#include <soc.h>
#include <soc/rtc_cntl_reg.h>

#include <zephyr.h>
#include <spinlock.h>
#include <kernel_structs.h>

#define Z_REG(base, off) (*(volatile u32_t *)((base) + (off)))

#define RTC_CNTL_BASE             0x3ff48000
#define RTC_CNTL_OPTIONS0     Z_REG(RTC_CNTL_BASE, 0x0)
#define RTC_CNTL_SW_CPU_STALL Z_REG(RTC_CNTL_BASE, 0xac)

#define DPORT_BASE                 0x3ff00000
#define DPORT_APPCPU_CTRL_A    Z_REG(DPORT_BASE, 0x02C)
#define DPORT_APPCPU_CTRL_B    Z_REG(DPORT_BASE, 0x030)
#define DPORT_APPCPU_CTRL_C    Z_REG(DPORT_BASE, 0x034)

struct cpustart_rec {
	int cpu;
	arch_cpustart_t fn;
	char *stack_top;
	void *arg;
	int vecbase;
	volatile int *alive;
};

volatile struct cpustart_rec *start_rec;
static void *appcpu_top;

static struct k_spinlock loglock;

/* Note that the logging done here is ACTUALLY REQUIRED FOR RELIABLE
 * OPERATION!  At least one particular board will experience spurious
 * hangs during initialization (usually the APPCPU fails to start at
 * all) without these calls present.  It's not just time -- careful
 * use of k_busy_wait() (and even hand-crafted timer loops using the
 * Xtensa timer SRs directly) that duplicates the timing exactly still
 * sees hangs.  Something is happening inside the ROM UART code that
 * magically makes the startup sequence reliable.
 *
 * Leave this in place until the sequence is understood better.
 *
 * (Note that the use of the spinlock is cosmetic only -- if you take
 * it out the messages will interleave across the two CPUs but startup
 * will still be reliable.)
 */
void smp_log(const char *msg)
{
	k_spinlock_key_t key = k_spin_lock(&loglock);

	while (*msg) {
		esp32c_rom_uart_tx_one_char(*msg++);
	}
	esp32c_rom_uart_tx_one_char('\r');
	esp32c_rom_uart_tx_one_char('\n');

	k_spin_unlock(&loglock, key);
}

static void appcpu_entry2(void)
{
	volatile int ps, ie;
	smp_log("ESP32C: APPCPU running");

	/* Copy over VECBASE from the main CPU for an initial value
	 * (will need to revisit this if we ever allow a user API to
	 * change interrupt vectors at runtime).  Make sure interrupts
	 * are locally disabled, then synthesize a PS value that will
	 * enable them for the user code to pass to irq_unlock()
	 * later.
	 */
	__asm__ volatile("rsr.PS %0" : "=r"(ps));
	ps &= ~(PS_EXCM_MASK | PS_INTLEVEL_MASK);
	__asm__ volatile("wsr.PS %0" : : "r"(ps));

	ie = 0;
	__asm__ volatile("wsr.INTENABLE %0" : : "r"(ie));
	__asm__ volatile("wsr.VECBASE %0" : : "r"(start_rec->vecbase));
	__asm__ volatile("rsync");

	/* Set up the CPU pointer.  Really this should be xtensa arch
	 * code, not in the ESP-32 layer
	 */
	_cpu_t *cpu = &_kernel.cpus[1];

	__asm__ volatile("wsr.MISC0 %0" : : "r"(cpu));

	*start_rec->alive = 1;
	start_rec->fn(start_rec->arg);
}

/* Defines a locally callable "function" named _stack-switch().  The
 * first argument (in register a2 post-ENTRY) is the new stack pointer
 * to go into register a1.  The second (a3) is the entry point.
 * Because this never returns, a0 is used as a scratch register then
 * set to zero for the called function (a null return value is the
 * signal for "top of stack" to the debugger).
 */
void z_appcpu_stack_switch(void *stack, void *entry);
__asm__("\n"
	".align 4"		"\n"
	"z_appcpu_stack_switch:"	"\n\t"

	"entry a1, 16"		"\n\t"

	/* Subtle: we want the stack to be 16 bytes higher than the
	 * top on entry to the called function, because the ABI forces
	 * it to assume that those bytes are for its caller's A0-A3
	 * spill area.  (In fact ENTRY instructions with stack
	 * adjustments less than 16 are a warning condition in the
	 * assembler). But we aren't a caller, have no bit set in
	 * WINDOWSTART and will never be asked to spill anything.
	 * Those 16 bytes would otherwise be wasted on the stack, so
	 * adjust
	 */
	"addi a1, a2, 16"	"\n\t"

	/* Clear WINDOWSTART so called functions never try to spill
	 * our callers' registers into the now-garbage stack pointers
	 * they contain.  No need to set the bit corresponding to
	 * WINDOWBASE, our C callee will do that when it does an
	 * ENTRY.
	 */
	"movi a0, 0"		"\n\t"
	"wsr.WINDOWSTART a0"	"\n\t"

	/* Clear CALLINC field of PS (you would think it would, but
	 * our ENTRY doesn't actually do that) so the callee's ENTRY
	 * doesn't shift the registers
	 */
	"rsr.PS a0"		"\n\t"
	"movi a2, 0xfffcffff"	"\n\t"
	"and a0, a0, a2"	"\n\t"
	"wsr.PS a0"		"\n\t"

	"rsync"			"\n\t"
	"movi a0, 0"		"\n\t"

	"jx a3"			"\n\t");

/* Carefully constructed to use no stack beyond compiler-generated ABI
 * instructions.  WE DO NOT KNOW WHERE THE STACK FOR THIS FUNCTION IS.
 * The ROM library just picks a spot on its own with no input from our
 * app linkage and tells us nothing about it until we're already
 * running.
 */
static void appcpu_entry1(void)
{
	z_appcpu_stack_switch(appcpu_top, appcpu_entry2);
}
/* Functions to set page attributes for Region Protection option in the CPU.
 * See Xtensa ISA Reference manual for explanation of arguments (section 4.6.3.2).
 */

static inline void cpu_write_dtlb(uint32_t vpn, unsigned attr)
{
    __asm__ __volatile__  ("wdtlb  %1, %0; dsync\n" :: "r" (vpn), "r" (attr));

}


static inline void cpu_write_itlb(unsigned vpn, unsigned attr)
{
    __asm__ __volatile__  ("witlb  %1, %0; isync\n" :: "r" (vpn), "r" (attr));
}

/**
 * @brief Configure memory region protection
 *
 * Make page 0 access raise an exception.
 * Also protect some other unused pages so we can catch weirdness.
 * Useful attribute values:
 * 0 — cached, RW
 * 2 — bypass cache, RWX (default value after CPU reset)
 * 15 — no access, raise exception
 */

static inline void cpu_configure_region_protection()
{
    const uint32_t pages_to_protect[] = {0x00000000, 0x80000000, 0xa0000000, 0xc0000000, 0xe0000000};
    for (int i = 0; i < sizeof(pages_to_protect)/sizeof(pages_to_protect[0]); ++i) {
        cpu_write_dtlb(pages_to_protect[i], 0xf);
        cpu_write_itlb(pages_to_protect[i], 0xf);
    }
    cpu_write_dtlb(0x20000000, 0);
    cpu_write_itlb(0x20000000, 0);
}

/* The calls and sequencing here were extracted from the ESP-32
 * FreeRTOS integration with just a tiny bit of cleanup.  None of the
 * calls or registers shown are documented, so treat this code with
 * extreme caution.
 */
static void appcpu_start(void)
{
	smp_log("ESP32C: starting APPCPU");
	/* Configure the mode of instruction cache : cache size, cache line size. */
	extern void rom_config_instruction_cache_mode(uint32_t cfg_cache_size, uint8_t cfg_cache_ways, uint8_t cfg_cache_line_size);
    rom_config_instruction_cache_mode(0x4000, 4, 16);
	/* If we need use SPIRAM, we should use data cache.
       Configure the mode of data : cache size, cache line size.*/
       extern uint32_t Cache_Suspend_DCache(void);
    Cache_Suspend_DCache();
extern void rom_config_data_cache_mode(uint32_t cfg_cache_size, uint8_t cfg_cache_ways, uint8_t cfg_cache_line_size);
    rom_config_data_cache_mode(0x4000, 4, 16);
	extern void Cache_Resume_DCache(uint32_t autoload);
    Cache_Resume_DCache(0);
	cpu_configure_region_protection();

	CLEAR_PERI_REG_MASK(RTC_CNTL_SW_CPU_STALL_REG, RTC_CNTL_SW_STALL_APPCPU_C1_M);
    CLEAR_PERI_REG_MASK(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_SW_STALL_APPCPU_C0_M);
	if (!REG_GET_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_CLKGATE_EN)) {
            REG_SET_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_CLKGATE_EN);
            REG_CLR_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_RUNSTALL);
            REG_SET_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_RESETING);
            REG_CLR_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_RESETING);
    }
	/* Seems weird that you set the boot address AFTER starting
	 * the CPU, but this is how they do it...
	 */
	esp32c_rom_ets_set_appcpu_boot_addr((void *)appcpu_entry1);

	smp_log("ESP32: APPCPU start sequence complete");
}

void arch_start_cpu(int cpu_num, k_thread_stack_t *stack, int sz,
		    arch_cpustart_t fn, void *arg)
{
	volatile struct cpustart_rec sr;
	int vb;
	volatile int alive_flag;

	__ASSERT(cpu_num == 1, "ESP-32C supports only two CPUs");

	__asm__ volatile("rsr.VECBASE %0\n\t" : "=r"(vb));

	alive_flag = 0;

	sr.cpu = cpu_num;
	sr.fn = fn;
	sr.stack_top = Z_THREAD_STACK_BUFFER(stack) + sz;
	sr.arg = arg;
	sr.vecbase = vb;
	sr.alive = &alive_flag;

	appcpu_top = Z_THREAD_STACK_BUFFER(stack) + sz;

	start_rec = &sr;

	appcpu_start();

	while (!alive_flag) {
	}

	smp_log("ESP32C: APPCPU initialized");
}
