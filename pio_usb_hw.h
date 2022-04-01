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
  volatile uint8_t root_idx;
  volatile uint8_t dev_addr;
  volatile uint8_t ep_num;
  volatile uint8_t size;
  volatile uint8_t attr;
  volatile uint8_t interval;
  volatile uint8_t interval_counter;
  volatile uint8_t data_id;  // data0 or data1

  volatile bool new_data_flag;
  uint8_t* bufptr;
  uint16_t total_len;
  uint16_t actual_len;
  uint8_t crc16[2];
} pio_hw_endpoint_t;

extern root_port_t root_port[PIO_USB_ROOT_PORT_CNT];
#define PIO_USB(_root_idx)    (root_port + _root_idx)

extern pio_hw_endpoint_t pio_hw_ep_pool[PIO_USB_EP_POOL_CNT];
#define PIO_USB_HW_EP(_idx)     (pio_hw_ep_pool+_idx)
