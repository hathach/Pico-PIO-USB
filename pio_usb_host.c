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

#define IRQ_TX_EOP_MASK (1 << usb_tx_fs_IRQ_EOP)
#define IRQ_TX_COMP_MASK (1 << usb_tx_fs_IRQ_COMP)
#define IRQ_TX_ALL_MASK (IRQ_TX_EOP_MASK | IRQ_TX_COMP_MASK)
#define IRQ_RX_COMP_MASK (1 << IRQ_RX_EOP)
#define IRQ_RX_ALL_MASK ((1 << IRQ_RX_EOP) | (1 << IRQ_RX_BS_ERR) | (1 << IRQ_RX_START))

static alarm_pool_t* _alarm_pool = NULL;
static repeating_timer_t _sof_rt;

bool sof_timer(repeating_timer_t *_rt);
extern void pio_usb_endpoint_finish_transfer(pio_hw_endpoint_t * ep, uint32_t flag);
extern uint32_t endpoint_setup_transaction( pio_port_t *pp,  pio_hw_endpoint_t *ep);
extern uint32_t endpoint_in_transaction(pio_port_t* pp, pio_hw_endpoint_t * ep);
extern uint32_t endpoint_out_transaction(pio_port_t* pp, pio_hw_endpoint_t * ep);

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

extern pio_port_t pio_port[1];
extern root_port_t root_port[PIO_USB_ROOT_PORT_CNT];

extern void __no_inline_not_in_flash_func(start_receive)(const pio_port_t *pp);
extern void __no_inline_not_in_flash_func(prepare_receive)(const pio_port_t *pp);
extern int __no_inline_not_in_flash_func(receive_packet_and_ack)(pio_port_t* pp, bool ack_handshake);
extern void __not_in_flash_func(usb_transfer)(const pio_port_t *pp,
                                              uint8_t *data, uint16_t len);

extern void __no_inline_not_in_flash_func(send_out_token)(const pio_port_t *pp,
                                                   uint8_t addr,
                                                   uint8_t ep_num);


extern void configure_tx_channel(uint8_t ch, PIO pio, uint sm);
extern void apply_config(pio_port_t *pp, const pio_usb_configuration_t *c, root_port_t *port);
extern void initialize_host_programs( pio_port_t *pp, const pio_usb_configuration_t *c, root_port_t *port);
extern void port_pin_drive_setting(const root_port_t *port);

#define SM_SET_CLKDIV(pio, sm, div) pio_sm_set_clkdiv_int_frac(pio, sm, div.div_int, div.div_frac)

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

void pio_usb_host_controller_init(const pio_usb_configuration_t *c)
{
  _alarm_pool = alarm_pool_create(2, 1);

  pio_port_t *pp = PIO_USB_HW_PIO(0);
  pio_hw_root_port_t* hw_root = PIO_USB_HW_RPORT(0);

  pp->pio_usb_tx = c->pio_tx_num == 0 ? pio0 : pio1;
  configure_tx_channel(c->tx_ch, pp->pio_usb_tx, c->sm_tx);

  apply_config(pp, c, &root_port[0]);
  initialize_host_programs(pp, c, &root_port[0]);
  port_pin_drive_setting(&root_port[0]);
  root_port[0].initialized = true;

  hw_root->initialized = true;
  hw_root->mode = PIO_USB_MODE_HOST;
  hw_root->pin_dp = c->pin_dp;
  hw_root->pin_dm = c->pin_dp+1;

  pio_calculate_clkdiv_from_float((float)clock_get_hz(clk_sys) / 48000000,
                                  &pp->clk_div_fs_tx.div_int,
                                  &pp->clk_div_fs_tx.div_frac);
  pio_calculate_clkdiv_from_float((float)clock_get_hz(clk_sys) / 6000000,
                                  &pp->clk_div_ls_tx.div_int,
                                  &pp->clk_div_ls_tx.div_frac);

  pio_calculate_clkdiv_from_float((float)clock_get_hz(clk_sys) / 96000000,
                                  &pp->clk_div_fs_rx.div_int,
                                  &pp->clk_div_fs_rx.div_frac);
  pio_calculate_clkdiv_from_float((float)clock_get_hz(clk_sys) / 12000000,
                                  &pp->clk_div_ls_rx.div_int,
                                  &pp->clk_div_ls_rx.div_frac);

  alarm_pool_add_repeating_timer_us(_alarm_pool, -1000, sof_timer, NULL,
                                      &_sof_rt);
}

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

/*static*/ void __no_inline_not_in_flash_func(override_pio_program)(PIO pio, const pio_program_t* program, uint offset) {
    for (uint i = 0; i < program->length; ++i) {
      uint16_t instr = program->instructions[i];
      pio->instr_mem[offset + i] =
          pio_instr_bits_jmp != _pio_major_instr_bits(instr) ? instr
                                                             : instr + offset;
    }
}

/*static*/ __always_inline void override_pio_rx_program(PIO pio,
                                             const pio_program_t *program,
                                             const pio_program_t *debug_program,
                                             uint offset, int debug_pin) {
  if (debug_pin < 0) {
    override_pio_program(pio, program, offset);
  } else {
    override_pio_program(pio, debug_program, offset);
  }
}

/*static*/ void __no_inline_not_in_flash_func(configure_fullspeed_host)(
    pio_port_t const *pp, pio_hw_root_port_t *port) {
  override_pio_program(pp->pio_usb_tx, &usb_tx_fs_program, pp->offset_tx);
  SM_SET_CLKDIV(pp->pio_usb_tx, pp->sm_tx, pp->clk_div_fs_tx);

  override_pio_rx_program(pp->pio_usb_rx, &usb_rx_fs_program,
                          &usb_rx_fs_debug_program, pp->offset_rx,
                          pp->debug_pin_rx);
  SM_SET_CLKDIV(pp->pio_usb_rx, pp->sm_rx, pp->clk_div_fs_rx);

  override_pio_rx_program(pp->pio_usb_rx, &eop_detect_fs_program,
                          &eop_detect_fs_debug_program, pp->offset_eop,
                          pp->debug_pin_eop);
  SM_SET_CLKDIV(pp->pio_usb_rx, pp->sm_eop, pp->clk_div_fs_rx);

  usb_tx_configure_pins(pp->pio_usb_tx, pp->sm_tx, port->pin_dp);
  usb_rx_configure_pins(pp->pio_usb_rx, pp->sm_eop, port->pin_dp);
  usb_rx_configure_pins(pp->pio_usb_rx, pp->sm_rx, port->pin_dp);
}

/*static*/ void __no_inline_not_in_flash_func(configure_lowspeed_host)(
    pio_port_t const *pp, pio_hw_root_port_t *port) {
  override_pio_program(pp->pio_usb_tx, &usb_tx_ls_program, pp->offset_tx);
  SM_SET_CLKDIV(pp->pio_usb_tx, pp->sm_tx, pp->clk_div_ls_tx);

  override_pio_rx_program(pp->pio_usb_rx, &usb_rx_ls_program,
                          &usb_rx_ls_debug_program, pp->offset_rx,
                          pp->debug_pin_rx);
  SM_SET_CLKDIV(pp->pio_usb_rx, pp->sm_rx, pp->clk_div_ls_rx);

  override_pio_rx_program(pp->pio_usb_rx, &eop_detect_ls_program,
                          &eop_detect_ls_debug_program, pp->offset_eop,
                          pp->debug_pin_eop);
  SM_SET_CLKDIV(pp->pio_usb_rx, pp->sm_eop, pp->clk_div_ls_rx);

  usb_tx_configure_pins(pp->pio_usb_tx, pp->sm_tx, port->pin_dp);
  usb_rx_configure_pins(pp->pio_usb_rx, pp->sm_eop, port->pin_dp);
  usb_rx_configure_pins(pp->pio_usb_rx, pp->sm_rx, port->pin_dm);
}

/*static*/ void __no_inline_not_in_flash_func(configure_root_port)(
    pio_port_t *pp, pio_hw_root_port_t *root) {
  if (root->is_fullspeed) {
    configure_fullspeed_host(pp, root);
  } else {
    configure_lowspeed_host(pp, root);
  }
}

/*static*/ void __no_inline_not_in_flash_func(restore_fs_bus)(const pio_port_t *pp) {
  // change bus speed to full-speed
  pio_sm_set_enabled(pp->pio_usb_tx, pp->sm_tx, false);
  SM_SET_CLKDIV(pp->pio_usb_tx, pp->sm_tx, pp->clk_div_fs_tx);

  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, false);
  SM_SET_CLKDIV(pp->pio_usb_rx, pp->sm_rx, pp->clk_div_fs_rx);
  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, true);

  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_eop, false);
  SM_SET_CLKDIV(pp->pio_usb_rx, pp->sm_eop, pp->clk_div_fs_rx);
  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_eop, true);
}

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

/*static*/ bool __no_inline_not_in_flash_func(connection_check)(pio_hw_root_port_t *port) {
  if (pio_hw_get_line_state(port) == PORT_PIN_SE0) {
    busy_wait_us_32(1);

    if (pio_hw_get_line_state(port) == PORT_PIN_SE0) {
      busy_wait_us_32(1);
      // device disconnect
      port->connected = false;
      port->suspended = true;
      port->ints |= PIO_USB_INTS_DISCONNECT_BITS;
      return false;
    }
  }

  return true;
}

/*static*/ bool __no_inline_not_in_flash_func(sof_timer)(repeating_timer_t *_rt) {
  static uint8_t sof_packet[4] = {USB_SYNC, USB_PID_SOF, 0x00, 0x10};
  static uint8_t sof_count = 0;
  (void) _rt;

  pio_port_t *pp = &pio_port[0];

  // Send SOF
  for (int root_idx = 0; root_idx < PIO_USB_ROOT_PORT_CNT; root_idx++) {
    pio_hw_root_port_t *hw_root = PIO_USB_HW_RPORT(root_idx);
    if (!(hw_root->initialized && hw_root->connected &&
         !hw_root->suspended && connection_check(hw_root))) {
      continue;
    }
    configure_root_port(pp, hw_root);
    usb_transfer(pp, sof_packet, 4);
  }

  // Carry out all queued endpoint transaction
  for (int root_idx = 0; root_idx < PIO_USB_ROOT_PORT_CNT; root_idx++) {
    pio_hw_root_port_t *hw_root = PIO_USB_HW_RPORT(root_idx);
    if (!(hw_root->initialized && hw_root->connected && !hw_root->suspended)) {
      continue;
    }

    configure_root_port(pp, hw_root);

    for (int ep_pool_idx = 0; ep_pool_idx < PIO_USB_EP_POOL_CNT; ep_pool_idx++) {
      pio_hw_endpoint_t *ep = PIO_USB_HW_EP(ep_pool_idx);

      if (ep->root_idx == root_idx && ep->size && ep->has_transfer) {
        uint32_t result = 0;

        if (ep->need_pre) {
          pp->need_pre = true;
        }

        if (ep->ep_num == 0 && ep->data_id == USB_PID_SETUP) {
          result = endpoint_setup_transaction(pp, ep);
        }else {
          if ( ep->ep_num & 0x80 ) {
            result = endpoint_in_transaction(pp, ep);
          }else{
            result = endpoint_out_transaction(pp, ep);
          }
        }

        if (ep->need_pre) {
          pp->need_pre = false;
          restore_fs_bus(pp);
        }

        if (result) {
          pio_usb_endpoint_finish_transfer(ep, result);
        }
      }
    }
  }

  // check for new connection to root hub
  for (int root_idx = 0; root_idx < PIO_USB_ROOT_PORT_CNT; root_idx++) {
    pio_hw_root_port_t *hw_root = PIO_USB_HW_RPORT(root_idx);
    if ( hw_root->initialized && !hw_root->connected) {
      port_pin_status_t const line_state = pio_hw_get_line_state(hw_root);
      if ( line_state == PORT_PIN_FS_IDLE || line_state == PORT_PIN_LS_IDLE) {
        hw_root->is_fullspeed = (line_state == PORT_PIN_FS_IDLE);
        hw_root->connected = true;
        hw_root->suspended = true; // need a bus reset before operating
        hw_root->ints |= PIO_USB_INTS_CONNECT_BITS;
      }
    }
  }

  // Invoke IRQHandler if interrupt status is set
  for (uint8_t root_idx = 0; root_idx < PIO_USB_ROOT_PORT_CNT; root_idx++) {
    if (PIO_USB_HW_RPORT(root_idx)->ints) {
      pio_usb_host_irq_handler(root_idx);
    }
  }

  sof_count = (sof_count + 1) & 0x1f;
  sof_packet[2] = sof_count & 0xff;
  sof_packet[3] = (calc_usb_crc5(sof_count) << 3) | (sof_count >> 8);

  return true;
}

void pio_usb_host_close_device(uint8_t root_idx, uint8_t device_address)
{
  for (int ep_pool_idx = 0; ep_pool_idx < PIO_USB_EP_POOL_CNT; ep_pool_idx++) {
    pio_hw_endpoint_t *ep = PIO_USB_HW_EP(ep_pool_idx);
    if ( (ep->root_idx == root_idx) && (ep->dev_addr == device_address) && ep->size){
      ep->size = 0;
      ep->has_transfer = false;
    }
  }
}

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

/*static*/ void __no_inline_not_in_flash_func(wait_handshake)(pio_port_t* pp) {
  int16_t t = 240;
  int16_t idx = 0;

  while (t--) {
    if (pio_sm_get_rx_fifo_level(pp->pio_usb_rx, pp->sm_rx)) {
      uint8_t data = pio_sm_get(pp->pio_usb_rx, pp->sm_rx) >> 24;
      pp->usb_rx_buffer[idx++] = data;
      if (idx == 2) {
        break;
      }
    }
  }

  if (t > 0) {
    while ((pp->pio_usb_rx->irq & IRQ_RX_COMP_MASK) == 0) {
      continue;
    }
  }

  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, false);
}

void  __no_inline_not_in_flash_func(send_token)(const pio_port_t *pp, uint8_t token, uint8_t addr, uint8_t ep_num) {

  uint8_t packet[4] = { USB_SYNC, token, 0, 0 };
  uint16_t dat = ((uint16_t)(ep_num & 0xf) << 7) | (addr & 0x7f);
  uint8_t crc = calc_usb_crc5(dat);
  packet[2] = dat & 0xff;
  packet[3] = (crc << 3) | ((dat >> 8) & 0x1f);

  usb_transfer(pp, packet, sizeof(packet));
}

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+
pio_hw_endpoint_t* _find_ep(uint8_t root_idx, uint8_t device_address, uint8_t ep_address) {
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

bool pio_usb_host_endpoint_open(uint8_t root_idx, uint8_t device_address, uint8_t const* desc_endpoint, bool need_pre) {
  const endpoint_descriptor_t *d = (const endpoint_descriptor_t *) desc_endpoint;

  for (int ep_pool_idx = 0; ep_pool_idx < PIO_USB_EP_POOL_CNT; ep_pool_idx++) {
    pio_hw_endpoint_t *ep = PIO_USB_HW_EP(ep_pool_idx);
    // ep size is used as valid indicator
    if (PIO_USB_HW_EP(ep_pool_idx)->size == 0) {
      pio_usb_endpoint_configure(ep, desc_endpoint);
      ep->root_idx = root_idx;
      ep->dev_addr = device_address;
      ep->need_pre = need_pre;
      ep->is_tx = (d->epaddr & 0x80) ? false : true; // host endpoint out is tx
      return true;
    }
  }

  return false;
}

bool pio_usb_host_send_setup(uint8_t root_idx, uint8_t device_address, uint8_t const setup_packet[8]) {
  pio_hw_endpoint_t *ep = _find_ep(root_idx, device_address, 0);
  if (!ep) return false;

  ep->ep_num = 0; // setup is is OUT
  ep->data_id = USB_PID_SETUP;
  ep->is_tx = true;

  pio_usb_endpoint_transfer(ep, (uint8_t*) setup_packet, 8);

  return true;
}

bool pio_usb_host_endpoint_transfer(uint8_t root_idx, uint8_t device_address, uint8_t ep_address, uint8_t* buffer, uint16_t buflen) {
  pio_hw_endpoint_t *ep = _find_ep(root_idx, device_address, ep_address);
  if (!ep) {
    printf("no endpoint 0x%02X\r\n", ep_address);
    return false;
  }

  // control endpoint switch direction when switch stage
  if (ep->ep_num != ep_address) {
    ep->ep_num = ep_address;
    ep->data_id = 1; // data and status always start with DATA1
    ep->is_tx = (ep_address == 0) ? true : false;
  }

  pio_usb_endpoint_transfer(ep, buffer, buflen);

  return true;
}

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

/*static*/ uint32_t __no_inline_not_in_flash_func(endpoint_in_transaction)(pio_port_t* pp, pio_hw_endpoint_t * ep) {
  uint32_t res = 0;

  uint8_t expect_token = (ep->data_id == 1) ? USB_PID_DATA1 : USB_PID_DATA0;

  prepare_receive(pp);
  send_token(pp, USB_PID_IN, ep->dev_addr, ep->ep_num);
  start_receive(pp);

  int receive_len = receive_packet_and_ack(pp, true);
  uint8_t const receive_token = pp->usb_rx_buffer[1];

  if (receive_len >= 0) {
    if (receive_token == expect_token) {
      // skip crc16 check
      memcpy(ep->bufptr+ep->actual_len, &pp->usb_rx_buffer[2], receive_len);
      ep->actual_len += receive_len;
      ep->data_id ^= 1;

      // complete if all bytes transferred or short packet
      if ( (receive_len < ep->size) || (ep->actual_len >= ep->total_len) ) {
        res = PIO_USB_INTS_ENDPOINT_COMPLETE_BITS;
      }
    }else {
      // DATA0/1 mismatched, 0 for re-try next frame
    }
  } else if (receive_token == USB_PID_NAK) {
    // NAK try again next frame
  } else if (receive_token == USB_PID_STALL) {
    res = PIO_USB_INTS_ENDPOINT_STALLED_BITS;
  }else {
    res = PIO_USB_INTS_ENDPOINT_ERROR_BITS;
  }

  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, false);
  pp->usb_rx_buffer[0] = 0;
  pp->usb_rx_buffer[1] = 0;

  return res;
}

/*static*/ uint32_t __no_inline_not_in_flash_func(endpoint_out_transaction)(pio_port_t* pp, pio_hw_endpoint_t * ep) {
  uint32_t res = 0;

  uint16_t const xact_len = pio_usb_endpoint_transaction_len(ep);

  prepare_receive(pp);
  send_token(pp, USB_PID_OUT, ep->dev_addr, ep->ep_num);
  // ensure previous tx complete
  while ((pp->pio_usb_tx->irq & IRQ_TX_COMP_MASK) == 0) {
    continue;
  }

  usb_transfer(pp, ep->bufptr+ep->actual_len, xact_len+4);
  start_receive(pp);

  wait_handshake(pp);

  uint8_t const receive_token = pp->usb_rx_buffer[1];

  if (receive_token == USB_PID_ACK) {
    ep->actual_len += xact_len;
    ep->data_id ^= 1;

    // complete if all bytes are transferred
    if ( ep->actual_len >= ep->total_len ) {
      res = PIO_USB_INTS_ENDPOINT_COMPLETE_BITS;
    }
  } else if (receive_token == USB_PID_NAK) {
    // NAK try again next frame
  } else if (receive_token == USB_PID_STALL) {
    res = PIO_USB_INTS_ENDPOINT_STALLED_BITS;
  }else {
    res = PIO_USB_INTS_ENDPOINT_ERROR_BITS;
  }

  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, false);
  pp->usb_rx_buffer[0] = 0;
  pp->usb_rx_buffer[1] = 0;

  return res;
}

uint32_t __no_inline_not_in_flash_func(endpoint_setup_transaction)(
    pio_port_t *pp,  pio_hw_endpoint_t *ep) {

  uint32_t res = 0;

  // Setup token
  prepare_receive(pp);

  send_token(pp, USB_PID_SETUP, ep->dev_addr, 0);
  // ensure previous tx complete
  while ((pp->pio_usb_tx->irq & IRQ_TX_COMP_MASK) == 0) {
    continue;
  }

  // Data
  ep->data_id = 0; // set to DATA0
  usb_transfer(pp, ep->bufptr, 12);

  // Handshake
  start_receive(pp);
  wait_handshake(pp);

  ep->actual_len = 8;

  if (pp->usb_rx_buffer[0] == USB_SYNC && pp->usb_rx_buffer[1] == USB_PID_ACK) {
    res = PIO_USB_INTS_ENDPOINT_COMPLETE_BITS;
  }else{
    res = PIO_USB_INTS_ENDPOINT_ERROR_BITS;
  }

  pp->usb_rx_buffer[1] = 0;  // reset buffer

  return res;
}
