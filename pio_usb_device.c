/**
 * Copyright (c) 2021 sekigon-gonnoc
 *                    Ha Thach (thach@tinyusb.org)
 */

#pragma GCC push_options
#pragma GCC optimize("-O3")

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "pio_usb.h"
#include "pio_usb_ll.h"
#include "usb_crc.h"
#include "usb_tx.pio.h"
#include "usb_rx.pio.h"

#include "hardware/irq.h"
#include "hardware/dma.h"

#include "tusb.h"

static uint8_t new_devaddr = 0;
static uint8_t ep0_crc5_lut[16];

static void __no_inline_not_in_flash_func(usb_device_packet_handler)(void);

/*static*/ void __no_inline_not_in_flash_func(update_ep0_crc5_lut)(uint8_t addr) {
  uint16_t dat;
  uint8_t crc;

  for (int epnum = 0; epnum < 16; epnum++) {
    dat = (addr) | (epnum << 7);
    crc = calc_usb_crc5(dat);
    ep0_crc5_lut[epnum] = (crc << 3) | ((epnum >> 1) & 0x07);
  }
}

static __always_inline void restart_usb_reveiver(pio_port_t *pp) {
  pio_sm_exec(pp->pio_usb_rx, pp->sm_rx, pp->rx_reset_instr);
  pio_sm_restart(pp->pio_usb_rx, pp->sm_rx);
  pio_sm_restart(pp->pio_usb_rx, pp->sm_eop);
  pp->pio_usb_rx->irq = IRQ_RX_ALL_MASK;
}

bool pio_usb_device_endpoint_open(uint8_t root_idx, uint8_t const *desc_endpoint)
{
  const endpoint_descriptor_t *d = (const endpoint_descriptor_t *) desc_endpoint;
  endpoint_t *ep = pio_usb_device_get_endpoint_by_address(d->epaddr);

  pio_usb_ll_endpoint_configure(ep, desc_endpoint);
  ep->root_idx = root_idx;
  ep->dev_addr = 0; // not used
  ep->need_pre = 0;
  ep->is_tx = (d->epaddr & 0x80) ? true : false; // device: endpoint in is tx

  return true;
}

usb_device_t *pio_usb_device_init(const pio_usb_configuration_t *c,
                                  const usb_descriptor_buffers_t *buffers) {
  pio_port_t *pp = PIO_USB_PIO_PORT(0);
  root_port_t* rport = PIO_USB_ROOT_PORT(0);

  pio_usb_bus_init(pp, c, rport);
  rport->mode = PIO_USB_MODE_DEVICE;

  update_ep0_crc5_lut(rport->dev_addr);

  float const cpu_freq = (float) clock_get_hz(clk_sys);

  pio_calculate_clkdiv_from_float(cpu_freq / 48000000,
                                  &pp->clk_div_fs_tx.div_int,
                                  &pp->clk_div_fs_tx.div_frac);
  pio_calculate_clkdiv_from_float(cpu_freq / 96000000,
                                  &pp->clk_div_fs_rx.div_int,
                                  &pp->clk_div_fs_rx.div_frac);

  pio_sm_set_enabled(pp->pio_usb_tx, pp->sm_tx, true);
  pio_usb_bus_prepare_receive(pp);
  pp->pio_usb_rx->ctrl |= (1 << pp->sm_rx);
  pp->pio_usb_rx->irq |= IRQ_RX_ALL_MASK;

  // configure PIOx_IRQ_0 to detect packet receive start
  pio_set_irqn_source_enabled(pp->pio_usb_rx, 0, pis_interrupt0 + IRQ_RX_START,
                              true);
  pp->device_rx_irq_num = (pp->pio_usb_rx == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0;
  irq_set_exclusive_handler(pp->device_rx_irq_num, usb_device_packet_handler);
  irq_set_enabled(pp->device_rx_irq_num, true);

  return NULL;
}

void pio_usb_device_set_address(uint8_t root_idx, uint8_t dev_addr) {
  (void) root_idx;
  new_devaddr = dev_addr;
}

static __always_inline int8_t device_receive_token(uint8_t *buffer,
                                                        uint8_t dev_addr) {
  pio_port_t *pp = PIO_USB_PIO_PORT(0);
  uint8_t idx = 0;
  uint8_t addr;
  uint8_t ep;
  bool match = false;

  static uint8_t eplut[2][8] = {{0, 2, 4, 6, 8, 10, 12, 14},
                                {1, 3, 5, 7, 9, 11, 13, 15}};
  uint8_t *current_lut;

  if ((pp->pio_usb_rx->irq & IRQ_RX_COMP_MASK) == 0) {
    while ((pp->pio_usb_rx->irq & IRQ_RX_COMP_MASK) == 0) {
      if (pio_sm_get_rx_fifo_level(pp->pio_usb_rx, pp->sm_rx)) {
        buffer[idx++] = pio_sm_get(pp->pio_usb_rx, pp->sm_rx) >> 24;
        if ( (idx == 3) && (buffer[1] != USB_PID_SOF) ) {
          addr = buffer[2] & 0x7f;
          current_lut = &eplut[buffer[2] >> 7][0];
          match = dev_addr == addr ? true : false;
        }
      }
    }
  } else {
    // host is probably timeout. Ignore this packets.
    pio_sm_clear_fifos(pp->pio_usb_rx, pp->sm_rx);
  }

  restart_usb_reveiver(pp);

  if (match) {
    ep = current_lut[buffer[3] & 0x07];
    if (ep0_crc5_lut[ep] == buffer[3]) {
      return ep;
    } else {
      return -1;
    }
  }

  return -1;
}

static void __no_inline_not_in_flash_func(usb_device_packet_handler)(void) {
  static uint8_t token_buf[64];
  pio_port_t *pp = PIO_USB_PIO_PORT(0);
  root_port_t *rport = PIO_USB_ROOT_PORT(0);

  //
  // time critical start
  //
  int8_t ep_num = device_receive_token(token_buf, rport->dev_addr);

  if (token_buf[1] == USB_PID_IN) {
    if (ep_num < 0) {
      return;
    }
    bool wait_ack = false;
    endpoint_t* ep = PIO_USB_ENDPOINT((ep_num << 1) | 0x01);

    pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, false);

    if (ep->stalled) {
      pio_usb_bus_send_handshake(pp, USB_PID_STALL);
    }else if (ep->has_transfer) {
      uint16_t const xact_len = pio_usb_ll_endpoint_transaction_len(ep);
      pio_usb_bus_usb_transfer(pp, ep->bufptr+ep->actual_len, xact_len+4);
      wait_ack = true;
    } else {
      pio_usb_bus_send_handshake(pp, USB_PID_NAK);
    }

//    pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, true);
//    prepare_receive(pp);

//    pp->pio_usb_rx->irq = IRQ_RX_ALL_MASK;
//    irq_clear(pp->device_rx_irq_num);
//    start_receive(pp);

    //
    // time critical end
    //

    if (wait_ack) {
      pp->pio_usb_rx->irq = IRQ_RX_ALL_MASK;
      irq_clear(pp->device_rx_irq_num);
      pio_usb_bus_start_receive(pp);
      pio_usb_bus_wait_handshake(pp);

      wait_ack = false;

//      pio_sm_clear_fifos(pp->pio_usb_rx, pp->sm_rx);
//      restart_usb_reveiver(pp);
//      irq_clear(pp->device_rx_irq_num);

      pp->pio_usb_rx->irq = IRQ_RX_ALL_MASK;
      irq_clear(pp->device_rx_irq_num);
      pio_usb_bus_start_receive(pp);

      if (ep->ep_num == 0x80 && new_devaddr > 0) {
        rport->dev_addr = new_devaddr;
        new_devaddr = 0;
        update_ep0_crc5_lut(rport->dev_addr);
      }

      uint16_t const xact_len = pio_usb_ll_endpoint_transaction_len(ep);

      ep->actual_len += xact_len;
      ep->data_id ^= 1;

      // complete if all bytes transferred
      if (ep->actual_len >= ep->total_len) {
        pio_usb_ll_endpoint_complete(ep, PIO_USB_INTS_ENDPOINT_COMPLETE_BITS);
      }
    }else
    {
      pp->pio_usb_rx->irq = IRQ_RX_ALL_MASK;
      irq_clear(pp->device_rx_irq_num);
      pio_usb_bus_start_receive(pp);
    }
  } else if (token_buf[1] == USB_PID_OUT) {
    if (ep_num < 0) {
      return;
    }
    endpoint_t* ep = PIO_USB_ENDPOINT(ep_num << 1);

    uint8_t hanshake = ep->stalled ? USB_PID_STALL : (ep->has_transfer ? USB_PID_ACK : USB_PID_NAK);
    int res = pio_usb_bus_receive_packet_and_handshake(pp, hanshake);
    pio_sm_clear_fifos(pp->pio_usb_rx, pp->sm_rx);
    restart_usb_reveiver(pp);
    irq_clear(pp->device_rx_irq_num);

    if (ep->has_transfer) {
      if (res >= 0) {
        memcpy(ep->bufptr+ep->actual_len, pp->usb_rx_buffer, res);
        ep->actual_len += res;
        ep->data_id ^= 1;

        // complete if all bytes transferred or short packet
        if ( (res < ep->size) || (ep->actual_len >= ep->total_len) ) {
          pio_usb_ll_endpoint_complete(ep, PIO_USB_INTS_ENDPOINT_COMPLETE_BITS);
        }
      }
    }
  } else if (token_buf[1] == USB_PID_SETUP) {
    if (ep_num < 0) {
      return;
    }
    int res = pio_usb_bus_receive_packet_and_handshake(pp, USB_PID_ACK);
    pio_sm_clear_fifos(pp->pio_usb_rx, pp->sm_rx);
    restart_usb_reveiver(pp);
    irq_clear(pp->device_rx_irq_num);

    //memcpy(rport->setup_packet, pp->usb_rx_buffer+2, 8);
    rport->setup_packet = pp->usb_rx_buffer+2;
    rport->ints |= PIO_USB_INTS_SETUP_REQ_BITS;

    // DATA1 for both data and status stage
    // PIO_USB_ENDPOINT(0)->has_transfer = PIO_USB_ENDPOINT(1)->has_transfer = false;
    PIO_USB_ENDPOINT(0)->data_id = PIO_USB_ENDPOINT(1)->data_id = 1;
    PIO_USB_ENDPOINT(0)->stalled = PIO_USB_ENDPOINT(1)->stalled = false;
  } else if (token_buf[1] == USB_PID_SOF) {
    // SOF interrupt
  }

  if (token_buf[0] == 0 && pio_usb_bus_get_line_state(rport) == PORT_PIN_SE0) {
    // Reset detected
    memset(pio_usb_ep_pool, 0, sizeof(pio_usb_ep_pool));
    rport->dev_addr = 0;
    update_ep0_crc5_lut(rport->dev_addr);

    // init endpoint control in/out
    PIO_USB_ENDPOINT(0)->size = 64;
    PIO_USB_ENDPOINT(0)->ep_num = 0;
    PIO_USB_ENDPOINT(0)->is_tx = false;

    PIO_USB_ENDPOINT(1)->size = 64;
    PIO_USB_ENDPOINT(1)->ep_num = 0x80;
    PIO_USB_ENDPOINT(1)->is_tx = true;

    // TODO should be reset end, this is reset start only
    rport->ep_complete = rport->ep_stalled = rport->ep_error = 0;
    rport->ints = PIO_USB_INTS_RESET_END_BITS;
  }

  token_buf[0] = 0; // clear received token
  token_buf[1] = 0;

  if (rport->ints) {
    pio_usb_device_irq_handler(0);
  }
}

#pragma GCC pop_options

