/**
 * Copyright (c) 2021 sekigon-gonnoc
 *                    Ha Thach (thach@tinyusb.org)
 */

#include <stdio.h>
#include "pio_usb_hw.h"
#include "hardware/timer.h"
#include "usb_crc.h"

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

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

pio_hw_endpoint_t* _get_ep(uint8_t root_idx, uint8_t device_address, uint8_t ep_address) {
  for (int ep_pool_idx = 0; ep_pool_idx < PIO_USB_EP_POOL_CNT; ep_pool_idx++) {
    pio_hw_endpoint_t *ep = PIO_USB_HW_EP(ep_pool_idx);
    // note 0x00 and 0x80 are matched as control endpoint of opposite direction
    if ( (ep->root_idx == root_idx) && (ep->dev_addr == device_address) && ep->size &&
         ((ep->ep_num == ep_address) || (((ep_address & 0x7f) == 0) && ((ep->ep_num & 0x7f) == 0)) ) ) {
      return ep;
    }
  }

  return NULL;
}

bool __no_inline_not_in_flash_func(pio_usb_endpoint_open)(uint8_t root_idx, uint8_t device_address, uint8_t const* desc_endpoint, bool need_pre) {
  const endpoint_descriptor_t *d = (const endpoint_descriptor_t *) desc_endpoint;

  for (int ep_pool_idx = 0; ep_pool_idx < PIO_USB_EP_POOL_CNT; ep_pool_idx++) {
    pio_hw_endpoint_t *ep = PIO_USB_HW_EP(ep_pool_idx);
    // ep size is used as valid indicator
    if (PIO_USB_HW_EP(ep_pool_idx)->size == 0) {
      // TODO size declared as 8-bit, can cause overflow warning
      ep->size             = d->max_size[0] | (d->max_size[1] << 8);
      ep->root_idx         = root_idx;
      ep->dev_addr         = device_address;
      ep->need_pre         = need_pre;
      ep->ep_num           = d->epaddr;
      ep->attr             = d->attr;
      ep->interval_counter = 0;
      ep->data_id          = 0;

      return true;
    }
  }

  return false;
}

bool pio_usb_endpoint_send_setup(uint8_t root_idx, uint8_t device_address, uint8_t const setup_packet[8]) {
  pio_hw_endpoint_t *ep = _get_ep(root_idx, device_address, 0);
  if (!ep) return false;

  ep->ep_num = 0; // setup is is OUT
  ep->data_id = USB_PID_SETUP;

  ep->bufptr = (uint8_t*) setup_packet;
  ep->total_len = 8;
  ep->actual_len = 0;

  uint16_t crc16 = calc_usb_crc16(setup_packet, 8);
  ep->crc16[0] = crc16 & 0xff;
  ep->crc16[1] = crc16 >> 8;

  ep->has_transfer = true;

  return true;
}

#include "tusb.h" // logging
bool pio_usb_endpoint_transfer(uint8_t root_idx, uint8_t device_address, uint8_t ep_address, uint8_t* buffer, uint16_t buflen) {
  pio_hw_endpoint_t *ep = _get_ep(root_idx, device_address, ep_address);
  if (!ep) {
    printf("no endpoint 0x%02X\r\n", ep_address);
    return false;
  }

  // control endpoint switch direction when switch stage
  if (ep->ep_num != ep_address) {
    ep->ep_num = ep_address;
    ep->data_id = 1; // data and status always start with DATA1
  }

  ep->bufptr = buffer;
  ep->total_len = buflen;
  ep->actual_len = 0;

  uint16_t const xact_len = MIN(buflen, ep->size);
  uint16_t crc16 = calc_usb_crc16(buffer, xact_len);

  ep->crc16[0] = crc16 & 0xff;
  ep->crc16[1] = crc16 >> 8;

  ep->buffer[0] = USB_SYNC;
  ep->buffer[1] = ep->data_id ? USB_PID_DATA1 : USB_PID_DATA0;
  memcpy(ep->buffer+2, buffer, xact_len);
  ep->buffer[2+xact_len] = ep->crc16[0];
  ep->buffer[2+xact_len+1] = ep->crc16[1];

  ep->packet_len = xact_len + 4;

  ep->has_transfer = true;

  return true;
}

uint16_t __no_inline_not_in_flash_func(pio_usb_endpoint_tx_prepare_buf)(pio_hw_endpoint_t * ep, uint8_t* buf, uint16_t bufsize) {
  uint16_t xact_len = ep->total_len-ep->actual_len;
  xact_len = MIN(xact_len, (uint16_t) ep->size);

  // buffer is not enough
  if (bufsize < xact_len + 4) return 0;

  buf[0] = USB_SYNC;
  buf[1] = ep->data_id ? USB_PID_DATA1 : USB_PID_DATA0;

  memcpy(buf+2, ep->bufptr+ep->actual_len, xact_len);

  // crc16 is already pre-computed
  buf[2+xact_len] = ep->crc16[0];
  buf[2+xact_len+1] = ep->crc16[1];

  return xact_len;
}
