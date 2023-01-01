#include "daplink.h"

struct daplink daplink;
volatile uint8_t daplink_xfer_abort;

static const char dap_fw_ver [] = DAP_FW_VER;


void dap_setup(void) {
  daplink.debug_port = 0U;
  daplink.nominal_clock = DAP_DEFAULT_SWJ_CLOCK;
  daplink.transfer.idle_cycles = 0U;
  daplink.transfer.retry_count = 100U;
  daplink.transfer.match_retry = 0U;
  daplink.transfer.match_mask  = 0x00000000U;
#if (DAP_SWD != 0)
  daplink.swd_conf.turnaround  = 1U;
  daplink.swd_conf.data_phase  = 0U;
#endif
#if (DAP_JTAG != 0)
  daplink.jtag_dev.count = 0U;
#endif

  // Sets dap_data.fast_clock and dap_data.clock_delay.
  Set_DAP_Clock_Delay(DAP_DEFAULT_SWJ_CLOCK);

  DAP_SETUP();  // Device specific setup
}
