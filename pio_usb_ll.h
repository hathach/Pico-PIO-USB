/**
 * Copyright (c) 2021 sekigon-gonnoc
 *                    Ha Thach (thach@tinyusb.org)
 */

#pragma once

#include "usb_definitions.h"
#include "pio_usb_configuration.h"
#include "hardware/pio.h"

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

typedef enum{
  PORT_PIN_SE0 = 0b00,
  PORT_PIN_FS_IDLE = 0b01,
  PORT_PIN_LS_IDLE = 0b10,
  PORT_PIN_SE1 = 0b11,
} port_pin_status_t;

typedef struct {
  uint16_t div_int;
  uint8_t div_frac;
} pio_clk_div_t;

typedef struct {
  PIO pio_usb_tx;  // could not set to volatile
  uint sm_tx;
  uint offset_tx;
  uint tx_ch;

  PIO pio_usb_rx;  // could not set to volatile
  uint sm_rx;
  uint offset_rx;
  uint sm_eop;
  uint offset_eop;
  uint rx_reset_instr;
  uint device_rx_irq_num;

  int8_t debug_pin_rx;
  int8_t debug_pin_eop;

  pio_clk_div_t clk_div_fs_tx;
  pio_clk_div_t clk_div_fs_rx;
  pio_clk_div_t clk_div_ls_tx;
  pio_clk_div_t clk_div_ls_rx;

  bool need_pre;

  uint8_t usb_rx_buffer[128];
} pio_port_t;


//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

enum {
  PIO_USB_MODE_INVALID = 0,
  PIO_USB_MODE_DEVICE,
  PIO_USB_MODE_HOST,
};

extern root_port_t pio_usb_root_port[PIO_USB_ROOT_PORT_CNT];
#define PIO_USB_ROOT_PORT(_idx)    (pio_usb_root_port + (_idx))

extern endpoint_t pio_usb_ep_pool[PIO_USB_EP_POOL_CNT];
#define PIO_USB_ENDPOINT(_idx)     (pio_usb_ep_pool + (_idx))

extern pio_port_t pio_port[1];
#define PIO_USB_PIO_PORT(_idx)    (pio_port + (_idx))


//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

void pio_usb_ll_init(pio_port_t *pp, const pio_usb_configuration_t *c, root_port_t* hw_root);

static __always_inline port_pin_status_t pio_usb_ll_get_line_state(root_port_t* hw_root)
{
  uint8_t dp = gpio_get(hw_root->pin_dp) ? 1 : 0;
  uint8_t dm = gpio_get(hw_root->pin_dm) ? 1 : 0;

  return (dm << 1) | dp;
}


void pio_usb_ll_endpoint_configure(endpoint_t * ep, uint8_t const* desc_endpoint);
bool pio_usb_ll_endpoint_transfer(endpoint_t * ep, uint8_t* buffer, uint16_t buflen);

static inline __force_inline void pio_usb_ll_endpoint_complete(endpoint_t * ep, uint32_t flag) {
  root_port_t *rport = PIO_USB_ROOT_PORT(ep->root_idx);
  uint32_t const ep_mask = (1u << (ep-pio_usb_ep_pool));

  rport->ints |= flag;

  if (flag == PIO_USB_INTS_ENDPOINT_COMPLETE_BITS) {
    rport->ep_complete |= ep_mask;
  }else if (flag == PIO_USB_INTS_ENDPOINT_ERROR_BITS) {
    rport->ep_error |= ep_mask;
  }else if (flag == PIO_USB_INTS_ENDPOINT_STALLED_BITS) {
    rport->ep_stalled |= ep_mask;
  }else {
    // something wrong
  }

  ep->has_transfer = false;
}

static inline __force_inline uint16_t pio_usb_ll_endpoint_transaction_len(endpoint_t * ep)
{
  uint16_t remaining = ep->total_len - ep->actual_len;
  return (remaining < ep->size) ? remaining : ep->size;
}

