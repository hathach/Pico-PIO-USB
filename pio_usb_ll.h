/**
 * Copyright (c) 2021 sekigon-gonnoc
 *                    Ha Thach (thach@tinyusb.org)
 */

#pragma once

#include "usb_definitions.h"
#include "pio_usb_configuration.h"

enum {
  PIO_USB_INTS_CONNECT_POS = 0,
  PIO_USB_INTS_DISCONNECT_POS,
  PIO_USB_INTS_RESET_END_POS,
  PIO_USB_INTS_SETUP_REQ_POS,
  PIO_USB_INTS_SOF_POS,

  PIO_USB_INTS_ENDPOINT_COMPLETE_POS,
  PIO_USB_INTS_ENDPOINT_ERROR_POS,
  PIO_USB_INTS_ENDPOINT_STALLED_POS,
};

#define PIO_USB_INTS_CONNECT_BITS              (1u << PIO_USB_INTS_CONNECT_POS)
#define PIO_USB_INTS_DISCONNECT_BITS           (1u << PIO_USB_INTS_DISCONNECT_POS)
#define PIO_USB_INTS_RESET_END_BITS            (1u << PIO_USB_INTS_RESET_END_POS)
#define PIO_USB_INTS_SETUP_REQ_BITS            (1u << PIO_USB_INTS_SETUP_REQ_POS)

#define PIO_USB_INTS_SOF_BITS                  (1u << PIO_USB_INTS_SOF_POS)

#define PIO_USB_INTS_ENDPOINT_COMPLETE_BITS    (1u << PIO_USB_INTS_ENDPOINT_COMPLETE_POS)
#define PIO_USB_INTS_ENDPOINT_ERROR_BITS       (1u << PIO_USB_INTS_ENDPOINT_ERROR_POS)
#define PIO_USB_INTS_ENDPOINT_STALLED_BITS     (1u << PIO_USB_INTS_ENDPOINT_STALLED_POS)


//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

enum {
  PIO_USB_MODE_INVALID = 0,
  PIO_USB_MODE_DEVICE,
  PIO_USB_MODE_HOST,
};

extern pio_hw_root_port_t pio_usb_root_port[PIO_USB_ROOT_PORT_CNT];
#define PIO_USB_HW_RPORT(_idx)    (pio_usb_root_port + (_idx))

extern endpoint_t pio_usb_ep_pool[PIO_USB_EP_POOL_CNT];
#define PIO_USB_HW_EP(_idx)     (pio_usb_ep_pool + (_idx))

extern pio_port_t pio_port[1];
#define PIO_USB_HW_PIO(_idx)    (pio_port + (_idx))

void pio_usb_ll_init(pio_port_t *pp, const pio_usb_configuration_t *c, pio_hw_root_port_t* hw_root);

static __always_inline port_pin_status_t pio_usb_ll_get_line_state(pio_hw_root_port_t* hw_root)
{
  uint8_t dp = gpio_get(hw_root->pin_dp) ? 1 : 0;
  uint8_t dm = gpio_get(hw_root->pin_dm) ? 1 : 0;

  return (dm << 1) | dp;
}


void pio_usb_ll_endpoint_configure(endpoint_t * ep, uint8_t const* desc_endpoint);
bool pio_usb_ll_endpoint_transfer(endpoint_t * ep, uint8_t* buffer, uint16_t buflen);

static inline __force_inline void pio_usb_endpoint_finish_transfer(endpoint_t * ep, uint32_t flag) {
  pio_hw_root_port_t *hw_root = PIO_USB_HW_RPORT(ep->root_idx);
  uint32_t const ep_mask = (1u << (ep-pio_usb_ep_pool));

  hw_root->ints |= flag;

  if (flag == PIO_USB_INTS_ENDPOINT_COMPLETE_BITS) {
    hw_root->ep_complete |= ep_mask;
  }else if (flag == PIO_USB_INTS_ENDPOINT_ERROR_BITS) {
    hw_root->ep_error |= ep_mask;
  }else if (flag == PIO_USB_INTS_ENDPOINT_STALLED_BITS) {
    hw_root->ep_stalled |= ep_mask;
  }else {
    // something wrong
  }

  ep->has_transfer = false;
}

static inline __force_inline uint16_t pio_usb_endpoint_transaction_len(endpoint_t * ep)
{
  uint16_t remaining = ep->total_len - ep->actual_len;
  return (remaining < ep->size) ? remaining : ep->size;
}

