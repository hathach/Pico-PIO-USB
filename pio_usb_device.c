/**
 * Copyright (c) 2021 sekigon-gonnoc
 *                    Ha Thach (thach@tinyusb.org)
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "pio_usb.h"
#include "usb_crc.h"
#include "usb_tx.pio.h"
#include "usb_rx.pio.h"

#include "tusb.h"

bool pio_usb_device_endpoint_open(uint8_t root_idx, uint8_t const *desc_endpoint)
{
  const endpoint_descriptor_t *d = (const endpoint_descriptor_t *) desc_endpoint;
  pio_hw_endpoint_t *ep = pio_usb_device_get_ep(d->epaddr);

  pio_usb_endpoint_configure(ep, desc_endpoint);
  ep->root_idx = root_idx;
  ep->dev_addr = 0; // not used
  ep->need_pre = 0;
  ep->is_tx = (d->epaddr & 0x80) ? true : false; // device: endpoint in is tx

  return true;
}

bool pio_usb_device_endpoint_transfer(uint8_t root_idx, uint8_t ep_address, uint8_t* buffer, uint16_t buflen)
{
  (void) root_idx;
  pio_hw_endpoint_t *ep = pio_usb_device_get_ep(ep_address);
  pio_usb_endpoint_transfer(ep, buffer, buflen);
  return true;
}
