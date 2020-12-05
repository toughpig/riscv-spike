// See LICENSE for license details.

#include "mtrap.h"
#include "atomic.h"
#include "vm.h"
#include "fp_emulation.h"
#include "fdt.h"
#include "uart.h"
#include "uart16550.h"
#include "finisher.h"
#include "disabled_hart_mask.h"
#include "htif.h"
#include <string.h>
#include <limits.h>

pte_t* root_page_table;
uintptr_t mem_size;
volatile uint64_t* mtime;
volatile uint32_t* plic_priorities;
size_t plic_ndevs;
void* kernel_start;
void* kernel_end;

static void mstatus_init()
{
  uintptr_t mstatus = 0;

  // Enable FPU
  if (supports_extension('F'))
    mstatus |= MSTATUS_FS;

  // Enable vector extension
  if (supports_extension('V'))
    mstatus |= MSTATUS_VS;

  write_csr(mstatus, mstatus);

  // Enable user/supervisor use of perf counters
//  if (supports_extension('S'))
//    write_csr(scounteren, -1);
//  if (supports_extension('U'))
//    write_csr(mcounteren, -1);

  write_csr(mucounteren, -1);
  write_csr(mscounteren, -1);
  // Enable software interrupts
  write_csr(mie, MIP_MSIP);

  // Disable paging
  if (supports_extension('S'))
    write_csr(sptbr, 0);
}

// send S-mode interrupts and most exceptions straight to S-mode
static void delegate_traps()
{
  if (!supports_extension('S'))
    return;

  uintptr_t interrupts = MIP_SSIP | MIP_STIP | MIP_SEIP;
//  uintptr_t exceptions =
//    (1U << CAUSE_MISALIGNED_FETCH) |
//    (1U << CAUSE_FETCH_PAGE_FAULT) |
//    (1U << CAUSE_BREAKPOINT) |
//    (1U << CAUSE_LOAD_PAGE_FAULT) |
//    (1U << CAUSE_STORE_PAGE_FAULT) |
//    (1U << CAUSE_USER_ECALL);


#define CAUSE_MISALIGNED_FETCH 0x0
#define CAUSE_FAULT_FETCH 0x1
#define CAUSE_ILLEGAL_INSTRUCTION 0x2
#define CAUSE_BREAKPOINT 0x3
#define CAUSE_MISALIGNED_LOAD 0x4
#define CAUSE_FAULT_LOAD 0x5
#define CAUSE_MISALIGNED_STORE 0x6
#define CAUSE_FAULT_STORE 0x7
#define CAUSE_USER_ECALL 0x8
#define CAUSE_SUPERVISOR_ECALL 0x9
#define CAUSE_HYPERVISOR_ECALL 0xa
#define CAUSE_MACHINE_ECALL 0xb

  uintptr_t exceptions =
    (1U << CAUSE_MISALIGNED_FETCH) |
    (1U << CAUSE_FAULT_FETCH) |
    (1U << CAUSE_BREAKPOINT) |
    (1U << CAUSE_FAULT_LOAD) |
    (1U << CAUSE_FAULT_STORE) |
    (1U << CAUSE_BREAKPOINT) |
    (1U << CAUSE_USER_ECALL);
#undef CAUSE_MISALIGNED_FETCH
#undef CAUSE_FAULT_FETCH
#undef CAUSE_ILLEGAL_INSTRUCTION
#undef CAUSE_BREAKPOINT
#undef CAUSE_MISALIGNED_LOAD
#undef CAUSE_FAULT_LOAD
#undef CAUSE_MISALIGNED_STORE
#undef CAUSE_FAULT_STORE
#undef CAUSE_USER_ECALL
#undef CAUSE_SUPERVISOR_ECALL
#undef CAUSE_HYPERVISOR_ECALL
#undef CAUSE_MACHINE_ECALL
  printm("interrupts:%x exceptions:%x\n",interrupts,exceptions);
  //exceptions=0x1ab;
  write_csr(mideleg, interrupts);
  write_csr(medeleg, exceptions);
  assert(read_csr(mideleg) == interrupts);
  assert(read_csr(medeleg) == exceptions);
}

static void fp_init()
{
  if (!supports_extension('F'))
    return;

  assert(read_csr(mstatus) & MSTATUS_FS);

#ifdef __riscv_flen
  for (int i = 0; i < 32; i++)
    init_fp_reg(i);
  write_csr(fcsr, 0);

# if __riscv_flen == 32
  uintptr_t d_mask = 1 << ('D' - 'A');
  clear_csr(misa, d_mask);
  assert(!(read_csr(misa) & d_mask));
# endif

#else
  uintptr_t fd_mask = (1 << ('F' - 'A')) | (1 << ('D' - 'A'));
  clear_csr(misa, fd_mask);
  assert(!(read_csr(misa) & fd_mask));
#endif
}

hls_t* hls_init(uintptr_t id)
{
  hls_t* hls = OTHER_HLS(id);printm("hls -> addr: %llx\r\n",hls);
  memset(hls, 0, sizeof(*hls));
  return hls;
}

static void memory_init()
{
  mem_size = mem_size / MEGAPAGE_SIZE * MEGAPAGE_SIZE;
}

static void hart_init()
{
  mstatus_init();
  fp_init();
#ifndef BBL_BOOT_MACHINE
  delegate_traps();
#endif /* BBL_BOOT_MACHINE */
  //setup_pmp();
}

static void plic_init()
{
  for (size_t i = 1; i <= plic_ndevs; i++)
    plic_priorities[i] = 1;
}

static void prci_test()
{
  assert(!(read_csr(mip) & MIP_MSIP));
  *HLS()->ipi = 1;
  assert(read_csr(mip) & MIP_MSIP);
  *HLS()->ipi = 0;

  assert(!(read_csr(mip) & MIP_MTIP));
  *HLS()->timecmp = 0;
  assert(read_csr(mip) & MIP_MTIP);
  *HLS()->timecmp = -1ULL;
}

static void hart_plic_init()
{
printm("HLS()->ipi %x HLS()->timecmp %x\n",HLS()->ipi,HLS()->timecmp);
  // clear pending interrupts
  *HLS()->ipi = 0;
  *HLS()->timecmp = -1ULL;
  write_csr(mip, 0);

  if (!plic_ndevs)
    return;

  size_t ie_words = (plic_ndevs + 8 * sizeof(*HLS()->plic_s_ie) - 1) /
		(8 * sizeof(*HLS()->plic_s_ie));
  for (size_t i = 0; i < ie_words; i++) {
     if (HLS()->plic_s_ie) {
        // Supervisor not always present
        HLS()->plic_s_ie[i] = __UINT32_MAX__;
     }
  }
  *HLS()->plic_m_thresh = 1;
  if (HLS()->plic_s_thresh) {
      // Supervisor not always present
      *HLS()->plic_s_thresh = 0;
  }
}

static void wake_harts()
{

//    for (int hart = 0; hart < MAX_HARTS; ++hart)
//    if ((((~disabled_hart_mask & hart_mask) >> hart) & 1))printm("%x\n",OTHER_HLS(hart)->ipi);
  for (int hart = 0; hart < MAX_HARTS; ++hart)
    if ((((~disabled_hart_mask & hart_mask) >> hart) & 1))
      *OTHER_HLS(hart)->ipi = 1; // wakeup the hart
}
void init_first_hart(uintptr_t hartid, uintptr_t dtb)
{
extern char _dtb_start,_payload_start;
    dtb=(uintptr_t)&_dtb_start;
  // Confirm console as early as possible
  //query_uart(dtb);
  //query_uart16550(dtb);
  query_htif(dtb);printm("after query_htif\r\n");


  hart_init();

  hls_init(0); // this might get called again from parse_config_string

  // Find the power button early as well so die() works
  query_finisher(dtb);

  query_mem(dtb);

  query_harts(dtb);

  query_clint(dtb);

/*
printm("HLS()->ipi %llx HLS()->mipi_pending %llx\n\r\nHLS()->timecmp %llx HLS()->plic_m_thresh %llx\n\r\n HLS()->plic_m_ie %llx HLS()->plic_s_thresh %llx\n\r\nHLS()->plic_s_ie:%llx\r\n",\
  HLS()->ipi,HLS()->mipi_pending,HLS()->timecmp,HLS()->plic_m_thresh,HLS()->plic_m_ie,HLS()->plic_s_thresh,HLS()->plic_s_ie);
*/

  query_plic(dtb);

// olatile uint32_t* ipi;
//   volatile int mipi_pending;

//   volatile uint64_t* timecmp;

//   volatile uint32_t* plic_m_thresh;
//   volatile uint32_t* plic_m_ie;
//   volatile uint32_t* plic_s_thresh;
//   volatile uint32_t* plic_s_ie;

/*
  printm("HLS()->ipi %llx HLS()->mipi_pending %llx\n\r\nHLS()->timecmp %llx HLS()->plic_m_thresh %llx\n\r\n HLS()->plic_m_ie %llx HLS()->plic_s_thresh %llx\n\r\nHLS()->plic_s_ie:%llx\r\n",\
  HLS()->ipi,HLS()->mipi_pending,HLS()->timecmp,HLS()->plic_m_thresh,HLS()->plic_m_ie,HLS()->plic_s_thresh,HLS()->plic_s_ie);
 */
  
  query_chosen(dtb);
  wake_harts();
  plic_init();
  hart_plic_init();
  //prci_test();
  memory_init();

  boot_loader(dtb);
}

void init_other_hart(uintptr_t hartid, uintptr_t dtb)
{
extern char _dtb_start,_payload_start;
dtb=(uintptr_t)&_dtb_start;
printm("init_other_hart\r\n");
  hart_init();
  hart_plic_init();
  boot_other_hart(dtb);
}

void setup_pmp(void)
{
  // Set up a PMP to permit access to all of memory.
  // Ignore the illegal-instruction trap if PMPs aren't supported.
  uintptr_t pmpc = PMP_NAPOT | PMP_R | PMP_W | PMP_X;
  asm volatile ("la t0, 1f\n\t"
                "csrrw t0, mtvec, t0\n\t"
                "csrw pmpaddr0, %1\n\t"
                "csrw pmpcfg0, %0\n\t"
                ".align 2\n\t"
                "1: csrw mtvec, t0"
                : : "r" (pmpc), "r" (-1UL) : "t0");
}

void enter_supervisor_mode(void (*fn)(uintptr_t), uintptr_t arg0, uintptr_t arg1)
{
  printm("enter_supervisor_mode\r\n");
  uintptr_t mstatus = read_csr(mstatus);
  mstatus = INSERT_FIELD(mstatus, MSTATUS_MPP, PRV_S);
  mstatus = INSERT_FIELD(mstatus, MSTATUS_MPIE, 0);
  write_csr(mstatus, mstatus);
  write_csr(mscratch, MACHINE_STACK_TOP() - MENTRY_FRAME_SIZE);
#ifndef __riscv_flen
  uintptr_t *p_fcsr = MACHINE_STACK_TOP() - MENTRY_FRAME_SIZE; // the x0's save slot
  *p_fcsr = 0;
#endif
  write_csr(mepc, fn);
    printm("enter_supervisor_mode fn=0x%x fdt=0x%x\n",fn,arg1);
    uintptr_t hartid = read_csr(mhartid);
  register uintptr_t a0 asm ("a0") = arg0;
  register uintptr_t a1 asm ("a1") = arg1;
  register uintptr_t a3 asm ("a3") = hartid;
  
  
  asm volatile ("mret" : : "r" (a0), "r" (a1), "r" (a3));
  __builtin_unreachable();
}

void enter_machine_mode(void (*fn)(uintptr_t, uintptr_t), uintptr_t arg0, uintptr_t arg1)
{
  uintptr_t mstatus = read_csr(mstatus);
  mstatus = INSERT_FIELD(mstatus, MSTATUS_MPIE, 0);
  write_csr(mstatus, mstatus);
  write_csr(mscratch, MACHINE_STACK_TOP() - MENTRY_FRAME_SIZE);

  /* Jump to the payload's entry point */
  fn(arg0, arg1);

  __builtin_unreachable();
}
