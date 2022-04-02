/**
 * Copyright (c) 2021 sekigon-gonnoc
 *                    Ha Thach (thach@tinyusb.org)
 */

#include "pio_usb_hw.h"

pio_hw_endpoint_t pio_hw_ep_pool[PIO_USB_EP_POOL_CNT];
pio_hw_root_port_t pio_hw_root_port[PIO_USB_ROOT_PORT_CNT];

port_pin_status_t __no_inline_not_in_flash_func(pio_hw_get_line_state)(pio_hw_root_port_t* hw_root)
{
  uint8_t dp = gpio_get(hw_root->pin_dp) ? 1 : 0;
  uint8_t dm = gpio_get(hw_root->pin_dm) ? 1 : 0;

  return (dp << 1) | dm;
}
