/**
 * Copyright (c) 2021 sekigon-gonnoc
 *                    Ha Thach (thach@tinyusb.org)
 */

#pragma once

#include "usb_definitions.h"
#include "pio_usb_configuration.h"

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

typedef struct {
  volatile bool initialized;
  volatile bool connected;
  volatile bool is_fullspeed;
  volatile uint pin_dp;
  volatile uint pin_dm;

  // register interface
  volatile uint32_t ints; // interrupt status
  volatile uint32_t ep_complete;
  volatile uint32_t ep_error;
  volatile uint32_t ep_stalled;
} pio_hw_root_port_t;

typedef struct {
  volatile uint8_t root_idx;
  volatile uint8_t dev_addr;
  volatile uint8_t ep_num;
  volatile uint8_t size;
  volatile uint8_t attr;
  volatile uint8_t interval;
  volatile uint8_t interval_counter;
  volatile uint8_t data_id;  // data0 or data1

  volatile bool active;
  uint8_t* bufptr;
  uint16_t total_len;
  uint16_t actual_len;
  uint8_t crc16[2];
} pio_hw_endpoint_t;

extern pio_hw_root_port_t pio_hw_root_port[PIO_USB_ROOT_PORT_CNT];
#define PIO_USB_HW_RPORT(_idx)    (pio_hw_root_port + _idx)

extern pio_hw_endpoint_t pio_hw_ep_pool[PIO_USB_EP_POOL_CNT];
#define PIO_USB_HW_EP(_idx)     (pio_hw_ep_pool+_idx)

port_pin_status_t pio_hw_get_line_state(pio_hw_root_port_t* hw_root);

