/**
 * Copyright (c) 2021 sekigon-gonnoc
 *                    Ha Thach (thach@tinyusb.org)
 */

#include "pio_usb_hw.h"
#include "hardware/timer.h"

pio_hw_endpoint_t pio_hw_ep_pool[PIO_USB_EP_POOL_CNT];
pio_hw_root_port_t pio_hw_root_port[PIO_USB_ROOT_PORT_CNT];

port_pin_status_t __no_inline_not_in_flash_func(pio_hw_get_line_state)(pio_hw_root_port_t* hw_root)
{
  uint8_t dp = gpio_get(hw_root->pin_dp) ? 1 : 0;
  uint8_t dm = gpio_get(hw_root->pin_dm) ? 1 : 0;

  return (dp << 1) | dm;
}

void pio_usb_hw_port_reset_start(uint8_t root_idx)
{
  pio_hw_root_port_t *root = PIO_USB_HW_RPORT(root_idx);
  pio_port_t *pp = PIO_USB_HW_PIO(0);

  // bus is not operating while in reset
  root->suspended = true;

  // Force line state to SE0
  pio_sm_set_pins_with_mask(pp->pio_usb_tx, pp->sm_tx, (0b00 << root->pin_dp),
                            (0b11u << root->pin_dp));
  pio_sm_set_pindirs_with_mask(pp->pio_usb_tx, pp->sm_tx, (0b11u << root->pin_dp),
                               (0b11u << root->pin_dp));
}

void pio_usb_hw_port_reset_end(uint8_t root_idx)
{
  pio_hw_root_port_t *root = PIO_USB_HW_RPORT(root_idx);
  pio_port_t *pp = PIO_USB_HW_PIO(0);

  // line state to input
  pio_sm_set_pindirs_with_mask(pp->pio_usb_tx, pp->sm_tx, (0b00u << root->pin_dp),
                               (0b11u << root->pin_dp));

  busy_wait_us(100); // TODO check if this is neccessary

  // bus back to operating
  root->suspended = false;
}
