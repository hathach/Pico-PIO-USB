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

typedef struct {
  volatile bool initialized;
  volatile bool is_fullspeed;
  volatile bool connected;
  volatile bool suspended;

  uint8_t mode;

  volatile uint pin_dp;
  volatile uint pin_dm;

  // register interface
  volatile uint32_t ints; // interrupt status
  volatile uint32_t ep_complete;
  volatile uint32_t ep_error;
  volatile uint32_t ep_stalled;

  // device only
  uint8_t dev_addr;
  //uint8_t setup_packet[8];
  uint8_t* setup_packet;
} pio_hw_root_port_t;

typedef struct {
  volatile uint8_t root_idx;
  volatile uint8_t dev_addr;
           bool    need_pre;
           bool    is_tx;

  volatile uint8_t ep_num;
  volatile uint16_t size;
  volatile uint8_t attr;
  volatile uint8_t interval;
  volatile uint8_t interval_counter;
  volatile uint8_t data_id;  // data0 or data1

  volatile bool stalled;
  volatile bool has_transfer;
  uint8_t* bufptr;
  uint16_t total_len;
  uint16_t actual_len;

  uint8_t buffer[64 + 4];
  uint8_t packet_len;
} pio_hw_endpoint_t;

extern pio_hw_root_port_t pio_hw_root_port[PIO_USB_ROOT_PORT_CNT];
#define PIO_USB_HW_RPORT(_idx)    (pio_hw_root_port + (_idx))

extern pio_hw_endpoint_t pio_hw_ep_pool[PIO_USB_EP_POOL_CNT];
#define PIO_USB_HW_EP(_idx)     (pio_hw_ep_pool + (_idx))

extern pio_port_t pio_port[1];
#define PIO_USB_HW_PIO(_idx)    (pio_port + (_idx))

port_pin_status_t pio_hw_get_line_state(pio_hw_root_port_t* hw_root);

void pio_usb_endpoint_configure(pio_hw_endpoint_t * ep, uint8_t const* desc_endpoint);
bool pio_usb_endpoint_transfer(pio_hw_endpoint_t * ep, uint8_t* buffer, uint16_t buflen);

static inline __force_inline void pio_usb_endpoint_finish_transfer(pio_hw_endpoint_t * ep, uint32_t flag) {
  pio_hw_root_port_t *hw_root = PIO_USB_HW_RPORT(ep->root_idx);
  uint32_t const ep_mask = (1u << (ep-pio_hw_ep_pool));

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

static inline __force_inline uint16_t pio_usb_endpoint_transaction_len(pio_hw_endpoint_t * ep)
{
  uint16_t remaining = ep->total_len - ep->actual_len;
  return (remaining < ep->size) ? remaining : ep->size;
}

