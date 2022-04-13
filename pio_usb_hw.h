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

typedef struct {
  volatile bool initialized;
  volatile bool is_fullspeed;
  volatile bool connected;
  volatile bool suspended;

  volatile uint pin_dp;
  volatile uint pin_dm;

  // register interface
  volatile uint32_t ints; // interrupt status
  volatile uint32_t ep_complete;
  volatile uint32_t ep_error;
  volatile uint32_t ep_stalled;

  // device only
  uint8_t setup_packet[8];
} pio_hw_root_port_t;

typedef struct {
  volatile uint8_t root_idx;
  volatile uint8_t dev_addr;
           bool    need_pre;

  volatile uint8_t ep_num;
  volatile uint8_t size;
  volatile uint8_t attr;
  volatile uint8_t interval;
  volatile uint8_t interval_counter;
  volatile uint8_t data_id;  // data0 or data1

  volatile bool has_transfer;
  uint8_t* bufptr;
  uint16_t total_len;
  uint16_t actual_len;
  uint8_t crc16[2];

           uint8_t buffer[64 + 4];
           uint8_t packet_len;
} pio_hw_endpoint_t;

extern pio_hw_root_port_t pio_hw_root_port[PIO_USB_ROOT_PORT_CNT];
#define PIO_USB_HW_RPORT(_idx)    (pio_hw_root_port + _idx)

extern pio_hw_endpoint_t pio_hw_ep_pool[PIO_USB_EP_POOL_CNT];
#define PIO_USB_HW_EP(_idx)     (pio_hw_ep_pool+_idx)

extern pio_port_t pio_port[1];
#define PIO_USB_HW_PIO(_idx)    (pio_port + _idx)

port_pin_status_t pio_hw_get_line_state(pio_hw_root_port_t* hw_root);

void pio_usb_hw_port_reset_start(uint8_t root_idx);
void pio_usb_hw_port_reset_end(uint8_t root_idx);

bool pio_usb_endpoint_open(uint8_t root_idx, uint8_t device_address, uint8_t const *desc_endpoint, bool need_pre);
bool pio_usb_endpoint_send_setup(uint8_t root_idx, uint8_t device_address, uint8_t const setup_packet[8]);
bool pio_usb_endpoint_transfer(uint8_t root_idx, uint8_t device_address, uint8_t ep_address, uint8_t* buffer, uint16_t buflen);

uint16_t pio_usb_endpoint_tx_prepare_buf(pio_hw_endpoint_t * ep, uint8_t* buf, uint16_t bufsize);



