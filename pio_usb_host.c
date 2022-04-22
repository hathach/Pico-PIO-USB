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
static repeating_timer_t sof_rt;
static bool timer_active;

static bool sof_timer(repeating_timer_t *_rt);

static uint32_t endpoint_setup_transaction( pio_port_t *pp,  endpoint_t *ep);
static uint32_t endpoint_in_transaction(pio_port_t* pp, endpoint_t * ep);
static uint32_t endpoint_out_transaction(pio_port_t* pp, endpoint_t * ep);

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

extern pio_port_t pio_port[1];

extern void __no_inline_not_in_flash_func(start_receive)(const pio_port_t *pp);
extern void __no_inline_not_in_flash_func(prepare_receive)(const pio_port_t *pp);
extern int __no_inline_not_in_flash_func(receive_packet_and_ack)(pio_port_t* pp, bool ack_handshake);
extern void __not_in_flash_func(usb_transfer)(const pio_port_t *pp,
                                              uint8_t *data, uint16_t len);

extern uint8_t __no_inline_not_in_flash_func(wait_handshake)(pio_port_t* pp);
extern void  __no_inline_not_in_flash_func(send_token)(const pio_port_t *pp, uint8_t token, uint8_t addr, uint8_t ep_num);

extern void port_pin_drive_setting(const root_port_t *port);

#define SM_SET_CLKDIV(pio, sm, div) pio_sm_set_clkdiv_int_frac(pio, sm, div.div_int, div.div_frac)

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

static void start_timer(alarm_pool_t *alarm_pool) {
  if (timer_active) {
    return;
  }

  if (alarm_pool != NULL) {
    alarm_pool_add_repeating_timer_us(alarm_pool, -1000, sof_timer, NULL,
                                      &sof_rt);
  } else {
    add_repeating_timer_us(-1000, sof_timer, NULL, &sof_rt);
  }

  timer_active = true;
}


static void stop_timer(void) {
  cancel_repeating_timer(&sof_rt);
  timer_active = false;
}

bool pio_usb_host_init(const pio_usb_configuration_t *c)
{
  pio_port_t *pp = PIO_USB_PIO_PORT(0);
  root_port_t* rport = PIO_USB_ROOT_PORT(0);

  pio_usb_ll_init(pp, c, rport);
  rport->mode = PIO_USB_MODE_HOST;

  float const cpu_freq = (float) clock_get_hz(clk_sys);
  pio_calculate_clkdiv_from_float(cpu_freq / 48000000,
                                  &pp->clk_div_fs_tx.div_int,
                                  &pp->clk_div_fs_tx.div_frac);
  pio_calculate_clkdiv_from_float(cpu_freq / 6000000,
                                  &pp->clk_div_ls_tx.div_int,
                                  &pp->clk_div_ls_tx.div_frac);

  pio_calculate_clkdiv_from_float(cpu_freq / 96000000,
                                  &pp->clk_div_fs_rx.div_int,
                                  &pp->clk_div_fs_rx.div_frac);
  pio_calculate_clkdiv_from_float(cpu_freq / 12000000,
                                  &pp->clk_div_ls_rx.div_int,
                                  &pp->clk_div_ls_rx.div_frac);

  _alarm_pool = alarm_pool_create(2, 1);
  start_timer(_alarm_pool);

  return true;
}

static volatile bool cancel_timer_flag;
static volatile bool start_timer_flag;
static uint32_t int_stat;

void pio_usb_host_stop(void) {
  cancel_timer_flag = true;
  while (cancel_timer_flag) {
    continue;
  }
}

void pio_usb_host_restart(void) {
  start_timer_flag = true;
  while (start_timer_flag) {
    continue;
  }
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
    pio_port_t const *pp, root_port_t *port) {
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
    pio_port_t const *pp, root_port_t *port) {
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
    pio_port_t *pp, root_port_t *root) {
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

/*static*/ bool __no_inline_not_in_flash_func(connection_check)(root_port_t *port) {
  if (pio_usb_ll_get_line_state(port) == PORT_PIN_SE0) {
    busy_wait_us_32(1);

    if (pio_usb_ll_get_line_state(port) == PORT_PIN_SE0) {
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

static bool __no_inline_not_in_flash_func(sof_timer)(repeating_timer_t *_rt) {
  static uint8_t sof_packet[4] = {USB_SYNC, USB_PID_SOF, 0x00, 0x10};
  static uint8_t sof_count = 0;
  (void) _rt;

  pio_port_t *pp = &pio_port[0];

  // Send SOF
  for (int root_idx = 0; root_idx < PIO_USB_ROOT_PORT_CNT; root_idx++) {
    root_port_t *rport = PIO_USB_ROOT_PORT(root_idx);
    if (!(rport->initialized && rport->connected &&
         !rport->suspended && connection_check(rport))) {
      continue;
    }
    configure_root_port(pp, rport);
    usb_transfer(pp, sof_packet, 4);
  }

  // Carry out all queued endpoint transaction
  for (int root_idx = 0; root_idx < PIO_USB_ROOT_PORT_CNT; root_idx++) {
    root_port_t *rport = PIO_USB_ROOT_PORT(root_idx);
    if (!(rport->initialized && rport->connected && !rport->suspended)) {
      continue;
    }

    configure_root_port(pp, rport);

    for (int ep_pool_idx = 0; ep_pool_idx < PIO_USB_EP_POOL_CNT; ep_pool_idx++) {
      endpoint_t *ep = PIO_USB_ENDPOINT(ep_pool_idx);

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
          pio_usb_ll_endpoint_complete(ep, result);
        }
      }
    }
  }

  // check for new connection to root hub
  for (int root_idx = 0; root_idx < PIO_USB_ROOT_PORT_CNT; root_idx++) {
    root_port_t *rport = PIO_USB_ROOT_PORT(root_idx);
    if ( rport->initialized && !rport->connected) {
      port_pin_status_t const line_state = pio_usb_ll_get_line_state(rport);
      if ( line_state == PORT_PIN_FS_IDLE || line_state == PORT_PIN_LS_IDLE) {
        rport->is_fullspeed = (line_state == PORT_PIN_FS_IDLE);
        rport->connected = true;
        rport->suspended = true; // need a bus reset before operating
        rport->ints |= PIO_USB_INTS_CONNECT_BITS;
      }
    }
  }

  // Invoke IRQHandler if interrupt status is set
  for (uint8_t root_idx = 0; root_idx < PIO_USB_ROOT_PORT_CNT; root_idx++) {
    if (PIO_USB_ROOT_PORT(root_idx)->ints) {
      pio_usb_host_irq_handler(root_idx);
    }
  }

  sof_count = (sof_count + 1) & 0x1f;
  sof_packet[2] = sof_count & 0xff;
  sof_packet[3] = (calc_usb_crc5(sof_count) << 3) | (sof_count >> 8);

  return true;
}

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

void pio_usb_host_port_reset_start(uint8_t root_idx)
{
  root_port_t *root = PIO_USB_ROOT_PORT(root_idx);
  pio_port_t *pp = PIO_USB_PIO_PORT(0);

  // bus is not operating while in reset
  root->suspended = true;

  // Force line state to SE0
  pio_sm_set_pins_with_mask(pp->pio_usb_tx, pp->sm_tx, (0b00 << root->pin_dp),
                            (0b11u << root->pin_dp));
  pio_sm_set_pindirs_with_mask(pp->pio_usb_tx, pp->sm_tx, (0b11u << root->pin_dp),
                               (0b11u << root->pin_dp));
}

void pio_usb_host_port_reset_end(uint8_t root_idx)
{
  root_port_t *root = PIO_USB_ROOT_PORT(root_idx);
  pio_port_t *pp = PIO_USB_PIO_PORT(0);

  // line state to input
  pio_sm_set_pindirs_with_mask(pp->pio_usb_tx, pp->sm_tx, (0b00u << root->pin_dp),
                               (0b11u << root->pin_dp));

  busy_wait_us(100); // TODO check if this is neccessary

  // bus back to operating
  root->suspended = false;
}

void pio_usb_host_close_device(uint8_t root_idx, uint8_t device_address)
{
  for (int ep_pool_idx = 0; ep_pool_idx < PIO_USB_EP_POOL_CNT; ep_pool_idx++) {
    endpoint_t *ep = PIO_USB_ENDPOINT(ep_pool_idx);
    if ( (ep->root_idx == root_idx) && (ep->dev_addr == device_address) && ep->size){
      ep->size = 0;
      ep->has_transfer = false;
    }
  }
}

endpoint_t* _find_ep(uint8_t root_idx, uint8_t device_address, uint8_t ep_address) {
  for (int ep_pool_idx = 0; ep_pool_idx < PIO_USB_EP_POOL_CNT; ep_pool_idx++) {
    endpoint_t *ep = PIO_USB_ENDPOINT(ep_pool_idx);
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
    endpoint_t *ep = PIO_USB_ENDPOINT(ep_pool_idx);
    // ep size is used as valid indicator
    if (PIO_USB_ENDPOINT(ep_pool_idx)->size == 0) {
      pio_usb_ll_endpoint_configure(ep, desc_endpoint);
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
  endpoint_t *ep = _find_ep(root_idx, device_address, 0);
  if (!ep) return false;

  ep->ep_num = 0; // setup is is OUT
  ep->data_id = USB_PID_SETUP;
  ep->is_tx = true;

  pio_usb_ll_endpoint_transfer(ep, (uint8_t*) setup_packet, 8);

  return true;
}

bool pio_usb_host_endpoint_transfer(uint8_t root_idx, uint8_t device_address, uint8_t ep_address, uint8_t* buffer, uint16_t buflen) {
  endpoint_t *ep = _find_ep(root_idx, device_address, ep_address);
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

  pio_usb_ll_endpoint_transfer(ep, buffer, buflen);

  return true;
}

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

static uint32_t __no_inline_not_in_flash_func(endpoint_in_transaction)(pio_port_t* pp, endpoint_t * ep) {
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

static uint32_t __no_inline_not_in_flash_func(endpoint_out_transaction)(pio_port_t* pp, endpoint_t * ep) {
  uint32_t res = 0;

  uint16_t const xact_len = pio_usb_ll_endpoint_transaction_len(ep);

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

static uint32_t __no_inline_not_in_flash_func(endpoint_setup_transaction)(
    pio_port_t *pp,  endpoint_t *ep) {

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
