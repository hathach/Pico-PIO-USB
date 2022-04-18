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

#include "hardware/irq.h"

#include "tusb.h"

#define IRQ_TX_EOP_MASK (1 << usb_tx_fs_IRQ_EOP)
#define IRQ_TX_COMP_MASK (1 << usb_tx_fs_IRQ_COMP)
#define IRQ_TX_ALL_MASK (IRQ_TX_EOP_MASK | IRQ_TX_COMP_MASK)
#define IRQ_RX_COMP_MASK (1 << IRQ_RX_EOP)
#define IRQ_RX_ALL_MASK ((1 << IRQ_RX_EOP) | (1 << IRQ_RX_BS_ERR) | (1 << IRQ_RX_START))

extern void __no_inline_not_in_flash_func(start_receive)(const pio_port_t *pp);
extern void __no_inline_not_in_flash_func(prepare_receive)(const pio_port_t *pp);
extern int __no_inline_not_in_flash_func(receive_packet_and_ack)(pio_port_t* pp, bool ack_handshake);
extern void __not_in_flash_func(usb_transfer)(const pio_port_t *pp,
                                              uint8_t *data, uint16_t len);

extern void __no_inline_not_in_flash_func(wait_handshake)(pio_port_t* pp);
extern void __no_inline_not_in_flash_func(send_handshake)(const pio_port_t *pp, uint8_t pid);
extern void  __no_inline_not_in_flash_func(send_token)(const pio_port_t *pp, uint8_t token, uint8_t addr, uint8_t ep_num);

static uint8_t new_devaddr = 0;
static uint8_t ep0_crc5_lut[16];

static void usb_device_packet_handler(void);

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
  pio_hw_endpoint_t *ep = pio_usb_device_get_ep(d->epaddr);

  pio_usb_endpoint_configure(ep, desc_endpoint);
  ep->root_idx = root_idx;
  ep->dev_addr = 0; // not used
  ep->need_pre = 0;
  ep->is_tx = (d->epaddr & 0x80) ? true : false; // device: endpoint in is tx

  return true;
}

usb_device_t *pio_usb_device_init(const pio_usb_configuration_t *c,
                                  const usb_descriptor_buffers_t *buffers) {
  pio_port_t *pp = &pio_port[0];
  pio_hw_root_port_t* hw_root = PIO_USB_HW_RPORT(0);

  pio_usb_ll_init(pp, c, hw_root);
  hw_root->mode = PIO_USB_MODE_DEVICE;

  update_ep0_crc5_lut(hw_root->dev_addr);

  float const cpu_freq = (float) clock_get_hz(clk_sys);

  pio_calculate_clkdiv_from_float(cpu_freq / 48000000,
                                  &pp->clk_div_fs_tx.div_int,
                                  &pp->clk_div_fs_tx.div_frac);
  pio_calculate_clkdiv_from_float(cpu_freq / 96000000,
                                  &pp->clk_div_fs_rx.div_int,
                                  &pp->clk_div_fs_rx.div_frac);

  pio_sm_set_enabled(pp->pio_usb_tx, pp->sm_tx, true);
  prepare_receive(pp);
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
  pio_port_t *pp = &pio_port[0];
  uint8_t idx = 0;
  int8_t addr = -1;
  int8_t ep = -1;
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
  static volatile bool wait_ack = false;
  static pio_hw_endpoint_t* wait_ep = NULL;

  pio_port_t *pp = &pio_port[0];
  pio_hw_root_port_t *hw_root = PIO_USB_HW_RPORT(0);

  //
  // time critical start
  //
  int8_t ep_num = device_receive_token(token_buf, hw_root->dev_addr);

  if (token_buf[1] == USB_PID_IN) {
    if (ep_num < 0) {
      return;
    }
    pio_hw_endpoint_t* ep = PIO_USB_HW_EP((ep_num << 1) | 0x01);

    pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, false);

    if (ep->stalled) {
      send_handshake(pp, USB_PID_STALL);
      wait_ep = NULL;
      wait_ack = false;
    }else if (ep->has_transfer) {
      uint16_t const xact_len = pio_usb_endpoint_transaction_len(ep);
      usb_transfer(pp, ep->bufptr+ep->actual_len, xact_len+4);
      wait_ep = ep;
      wait_ack = true;
    } else {
      send_handshake(pp, USB_PID_NAK);
      wait_ep = NULL;
      wait_ack = false;
    }

    pp->pio_usb_rx->irq = IRQ_RX_ALL_MASK;
    irq_clear(pp->device_rx_irq_num);
    start_receive(pp);

    //
    // time critical end
    //
  } else if (token_buf[1] == USB_PID_OUT) {
    wait_ack = false;
    wait_ep = NULL;
    if (ep_num < 0) {
      return;
    }
    pio_hw_endpoint_t* ep = PIO_USB_HW_EP(ep_num << 1);

    int res = receive_packet_and_ack(pp, ep->has_transfer);
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
          pio_usb_endpoint_finish_transfer(ep, PIO_USB_INTS_ENDPOINT_COMPLETE_BITS);
        }
      }
    }
  } else if (token_buf[1] == USB_PID_SETUP) {
    wait_ack = false;
    wait_ep = NULL;
    if (ep_num < 0) {
      return;
    }
    //active_ep = ep;
    int res = receive_packet_and_ack(pp, true);
    pio_sm_clear_fifos(pp->pio_usb_rx, pp->sm_rx);
    restart_usb_reveiver(pp);
    irq_clear(pp->device_rx_irq_num);

    //memcpy(hw_root->setup_packet, pp->usb_rx_buffer+2, 8);
    hw_root->setup_packet = pp->usb_rx_buffer+2;
    hw_root->ints |= PIO_USB_INTS_SETUP_REQ_BITS;

    // DATA1 for both data and status stage
    //PIO_USB_HW_EP(0)->has_transfer = PIO_USB_HW_EP(1)->has_transfer = false;
    PIO_USB_HW_EP(0)->data_id = PIO_USB_HW_EP(1)->data_id = 1;
    PIO_USB_HW_EP(0)->stalled = PIO_USB_HW_EP(1)->stalled = false;
  } else if (token_buf[1] == USB_PID_ACK && wait_ack) {
    pio_hw_endpoint_t* ep = wait_ep;
    wait_ack = false;
    wait_ep = NULL;
    if (ep) {
      if (ep->ep_num == 0x80 && new_devaddr > 0) {
        hw_root->dev_addr = new_devaddr;
        new_devaddr = 0;
        update_ep0_crc5_lut(hw_root->dev_addr);
      }

      uint16_t const xact_len = pio_usb_endpoint_transaction_len(ep);

      ep->actual_len += xact_len;
      ep->data_id ^= 1;

      // complete if all bytes transferred
      if (ep->actual_len >= ep->total_len) {
        pio_usb_endpoint_finish_transfer(ep, PIO_USB_INTS_ENDPOINT_COMPLETE_BITS);
      }
    }
  } else {
    wait_ack = false;
  }

  if (token_buf[0] == 0 && pio_hw_get_line_state(hw_root) == PORT_PIN_SE0) {
    // Reset detected
    memset(pio_hw_ep_pool, 0, sizeof(pio_hw_ep_pool));
    hw_root->dev_addr = 0;
    update_ep0_crc5_lut(hw_root->dev_addr);

    // init endpoint control in/out
    PIO_USB_HW_EP(0)->size = 64;
    PIO_USB_HW_EP(0)->ep_num = 0;
    PIO_USB_HW_EP(0)->is_tx = false;

    PIO_USB_HW_EP(1)->size = 64;
    PIO_USB_HW_EP(1)->ep_num = 0x80;
    PIO_USB_HW_EP(1)->is_tx = true;

    // TODO should be reset end, this is reset start only
    hw_root->ep_complete = hw_root->ep_stalled = hw_root->ep_error = 0;
    hw_root->ints = PIO_USB_INTS_RESET_END_BITS;
  }

  token_buf[0] = 0; // clear received token
  token_buf[1] = 0;

  if (hw_root->ints) {
    pio_usb_device_irq_handler(0);
  }
}

