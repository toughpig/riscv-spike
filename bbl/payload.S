// See LICENSE for license details.

#include "config.h"
#include "encoding.h"

  .section ".payload","a",@progbits

#if RELAXED_ALIGNMENT
  /* align payload minimally */
  .align 3
#else
  /* align payload to megapage */
  .align RISCV_PGSHIFT + RISCV_PGLEVEL_BITS
#endif

  .globl _payload_start, _payload_end, _dtb_start, _dtb_end
_payload_start:
  .incbin BBL_PAYLOAD
_payload_end:
   .align 4
_dtb_start:
   .incbin "../spike.dtb"
_dtb_end:
