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

#define IRQ_TX_EOP_MASK (1 << usb_tx_fs_IRQ_EOP)
#define IRQ_TX_COMP_MASK (1 << usb_tx_fs_IRQ_COMP)
#define IRQ_TX_ALL_MASK (IRQ_TX_EOP_MASK | IRQ_TX_COMP_MASK)
#define IRQ_RX_COMP_MASK (1 << IRQ_RX_EOP)
#define IRQ_RX_ALL_MASK ((1 << IRQ_RX_EOP) | (1 << IRQ_RX_BS_ERR) | (1 << IRQ_RX_START))

extern void __no_inline_not_in_flash_func(wait_handshake)(pio_port_t* pp);
extern void __no_inline_not_in_flash_func(start_receive)(const pio_port_t *pp);
extern void __no_inline_not_in_flash_func(prepare_receive)(const pio_port_t *pp);
extern void __no_inline_not_in_flash_func(data_transfer)(const pio_port_t *pp,
                                                  uint8_t *tx_data_address,
                                                  uint8_t tx_data_len);
extern void __not_in_flash_func(usb_transfer)(const pio_port_t *pp,
                                              uint8_t *data, uint16_t len);
extern void __no_inline_not_in_flash_func(send_out_token)(const pio_port_t *pp,
                                                   uint8_t addr,
                                                   uint8_t ep_num);
extern int __no_inline_not_in_flash_func(receive_packet_and_ack)(pio_port_t* pp);
extern void  __no_inline_not_in_flash_func(calc_in_token)(uint8_t * packet, uint8_t addr, uint8_t ep_num);

port_pin_status_t __no_inline_not_in_flash_func(get_port_pin_status)(
    root_port_t *port) {
  bool dp = gpio_get(port->pin_dp);
  bool dm = gpio_get(port->pin_dm);

  if (dp == false && dm == false) {
    return PORT_PIN_SE0;
  } else if (dp == true && dm == false) {
    return PORT_PIN_FS_IDLE;
  } else if (dp == false && dm == true) {
    return PORT_PIN_LS_IDLE;
  }

  return PORT_PIN_SE1;
}

endpoint_t* _get_ep(uint8_t root_idx, uint8_t device_address, uint8_t ep_address) {
  for (int ep_pool_idx = 0; ep_pool_idx < PIO_USB_EP_POOL_CNT; ep_pool_idx++)
  {
    endpoint_t *ep = &ep_pool[ep_pool_idx];
    if ( (ep->root_idx == root_idx) && (ep->dev_addr == device_address) &&
         ((ep->ep_num == ep_address) || (((ep_address & 0x7f) == 0) && ((ep->ep_num & 0x7f) == 0)) ) ) {
      return ep;
    }
  }

  return NULL;
}

bool pio_usb_endpoint_open(uint8_t root_idx, uint8_t device_address, uint8_t const* desc_endpoint) {
  const endpoint_descriptor_t *d = (const endpoint_descriptor_t *) desc_endpoint;

  endpoint_t *ep = NULL;

  if (device_address == 0) {
    // dedicate first endpoint for address0
    ep = &ep_pool[0];
  }else {
    for (int ep_pool_idx = 1; ep_pool_idx < PIO_USB_EP_POOL_CNT; ep_pool_idx++) {
      // ep size is used as valid indicator
      if (ep_pool[ep_pool_idx].size == 0) {
        ep = &ep_pool[ep_pool_idx];
        break;
      }
    }
  }

  if (ep == NULL) {
    return false;
  }

  // TODO size declared as 8-bit, can cause overflow warning
  ep->size = d->max_size[0] | (d->max_size[1] << 8);
  ep->root_idx = root_idx;
  ep->dev_addr = device_address;
  ep->ep_num = d->epaddr;
  ep->attr = d->attr;
  ep->interval_counter = 0;
  ep->data_id = 0;

  return true;
}

bool pio_usb_endpoint_send_setup(uint8_t root_idx, uint8_t device_address, uint8_t const setup_packet[8]) {
  endpoint_t *ep = _get_ep(root_idx, device_address, 0);
  if (!ep) return false;

  ep->ep_num = 0; // setup is is OUT
  ep->data_id = USB_PID_SETUP;

  ep->bufptr = (uint8_t*) setup_packet;
  ep->total_len = 8;
  ep->actual_len = 0;

  uint16_t crc16 = calc_usb_crc16(setup_packet, 8);
  ep->crc16[0] = crc16 & 0xff;
  ep->crc16[1] = crc16 >> 8;

  ep->new_data_flag = true;

  return true;
}

bool pio_usb_endpoint_transfer(uint8_t root_idx, uint8_t device_address, uint8_t ep_address, uint8_t* buffer, uint16_t buflen) {
  endpoint_t *ep = _get_ep(root_idx, device_address, ep_address);
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

  ep->new_data_flag = true;

  return true;
}

/*static*/ void __no_inline_not_in_flash_func(endpoint_transfer_finish)(endpoint_t * ep, uint32_t flag) {
  root_port_t *active_root = &root_port[ep->root_idx];
  uint32_t const ep_mask = (1u << (ep-ep_pool));

  active_root->ints |= flag;

  if (flag == PIO_USB_INTS_ENDPOINT_COMPLETE_BITS) {
    active_root->ep_complete |= ep_mask;
  }else if (flag == PIO_USB_INTS_ENDPOINT_ERROR_BITS) {
    active_root->ep_error |= ep_mask;
  }else if (flag == PIO_USB_INTS_ENDPOINT_STALLED_BITS) {
    active_root->ep_stalled |= ep_mask;
  }else {
    // something wrong
  }

  ep->new_data_flag = false;
}

/*static*/ int __no_inline_not_in_flash_func(endpoint_in_transaction)(pio_port_t* pp, endpoint_t * ep) {
  int res = -1;

  uint8_t expect_token = ep->data_id == 0 ? USB_PID_DATA0 : USB_PID_DATA1;
  uint8_t packet[4];
  calc_in_token(packet, ep->dev_addr, ep->ep_num & 0x0f);
  data_transfer(pp, packet, sizeof(packet));

  int receive_len = receive_packet_and_ack(pp);
  uint8_t const receive_token = pp->usb_rx_buffer[1];

  if (receive_len >= 0) {
    if (receive_token == expect_token) {
      memcpy(ep->bufptr+ep->actual_len, &pp->usb_rx_buffer[2], receive_len);
      ep->actual_len += receive_len;
      ep->data_id ^= 1;

      // complete if all bytes transferred or short packet
      if ( (receive_len < ep->size) || (ep->actual_len >= ep->total_len) ) {
        endpoint_transfer_finish(ep, PIO_USB_INTS_ENDPOINT_COMPLETE_BITS);
      }
    }else {
      // DATA0/1 mismatched
    }
    res = 0;
  } else if (receive_token == USB_PID_NAK) {
    res = 0;
    // NAK try again next frame
  } else if (receive_token == USB_PID_STALL) {
    endpoint_transfer_finish(ep, PIO_USB_INTS_ENDPOINT_STALLED_BITS);
    res = 0;
  }else {
    res = -1;
    if ((pp->pio_usb_rx->irq & IRQ_RX_COMP_MASK) == 0) {
      res = -2;
    }
    endpoint_transfer_finish(ep, PIO_USB_INTS_ENDPOINT_ERROR_BITS);
  }

  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, false);
  pp->usb_rx_buffer[0] = 0;
  pp->usb_rx_buffer[1] = 0;

  return res;
}

/*static*/ uint8_t __no_inline_not_in_flash_func(endpoint_out_prepare_buf)(endpoint_t * ep, uint8_t* buf) {
  buf[0] = USB_SYNC;
  buf[1] = ep->data_id ? USB_PID_DATA1 : USB_PID_DATA0;

  uint16_t xact_len = ep->total_len-ep->actual_len;
  xact_len = MIN(xact_len, (uint16_t) ep->size);

  memcpy(buf+2, ep->bufptr+ep->actual_len, xact_len);

  // crc16 is already pre-computed
  buf[2+xact_len] = ep->crc16[0];
  buf[2+xact_len+1] = ep->crc16[1];

  return (uint8_t) xact_len;
}

/*static*/ int __no_inline_not_in_flash_func(endpoint_out_transaction)(pio_port_t* pp, endpoint_t * ep) {
  int res = -1;

  uint8_t out_buf[64+4];
  uint8_t const xact_len = endpoint_out_prepare_buf(ep, out_buf);

  prepare_receive(pp);
  send_out_token(pp, ep->dev_addr, ep->ep_num & 0xf);
  // ensure previous tx complete
  while ((pp->pio_usb_tx->irq & IRQ_TX_COMP_MASK) == 0) {
    continue;
  }

  usb_transfer(pp, out_buf, xact_len+4);
  start_receive(pp);

  wait_handshake(pp);

  uint8_t const receive_token = pp->usb_rx_buffer[1];

  if (receive_token == USB_PID_ACK) {
    res = 0;
    ep->actual_len += xact_len;
    ep->data_id ^= 1;

    // complete if all bytes transferred or short packet
    if ( (xact_len < ep->size) || (ep->actual_len >= ep->total_len) ) {
      endpoint_transfer_finish(ep, PIO_USB_INTS_ENDPOINT_COMPLETE_BITS);
    }
  } else if (receive_token == USB_PID_NAK) {
    res = 0;
    // NAK try again next frame
  } else if (receive_token == USB_PID_STALL) {
    res = 0;
    endpoint_transfer_finish(ep, PIO_USB_INTS_ENDPOINT_STALLED_BITS);
  }else {
    res = -1;
    endpoint_transfer_finish(ep, PIO_USB_INTS_ENDPOINT_ERROR_BITS);
  }

  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, false);
  pp->usb_rx_buffer[0] = 0;
  pp->usb_rx_buffer[1] = 0;

  return res;
}


