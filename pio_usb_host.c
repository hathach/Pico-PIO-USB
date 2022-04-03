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

static alarm_pool_t* _alarm_pool = NULL;
static repeating_timer_t _sof_rt;

bool sof_timer(repeating_timer_t *_rt);
extern void endpoint_transfer_finish(pio_hw_endpoint_t * ep, uint32_t flag);
extern uint8_t endpoint_out_prepare_buf(pio_hw_endpoint_t * ep, uint8_t* buf);
extern void endpoint_setup_transaction( pio_port_t *pp,  pio_hw_endpoint_t *ep);
extern int endpoint_in_transaction(pio_port_t* pp, pio_hw_endpoint_t * ep);
extern int endpoint_out_transaction(pio_port_t* pp, pio_hw_endpoint_t * ep);

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

extern pio_port_t pio_port[1];
extern root_port_t root_port[PIO_USB_ROOT_PORT_CNT];

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
  pp->pio_usb_tx = c->pio_tx_num == 0 ? pio0 : pio1;
  configure_tx_channel(c->tx_ch, pp->pio_usb_tx, c->sm_tx);

  apply_config(pp, c, &root_port[0]);
  initialize_host_programs(pp, c, &root_port[0]);
  port_pin_drive_setting(&root_port[0]);
  root_port[0].initialized = true;

  PIO_USB_HW_RPORT(0)->initialized = true;
  PIO_USB_HW_RPORT(0)->pin_dp = c->pin_dp;
  PIO_USB_HW_RPORT(0)->pin_dm = c->pin_dp+1;

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
    pio_port_t *pp, pio_hw_root_port_t *port) {
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
    pio_port_t *pp, pio_hw_root_port_t *port) {
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

      if (ep->root_idx == root_idx && ep->size && ep->active) {
        if (ep->ep_num == 0 && ep->data_id == USB_PID_SETUP) {
          endpoint_setup_transaction(pp, ep);
        }else {
          if ( ep->ep_num & 0x80 ) {
            endpoint_in_transaction(pp, ep);
          }else{
            endpoint_out_transaction(pp, ep);
          }
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

bool pio_usb_endpoint_open(uint8_t root_idx, uint8_t device_address, uint8_t const* desc_endpoint) {
  const endpoint_descriptor_t *d = (const endpoint_descriptor_t *) desc_endpoint;

  pio_hw_endpoint_t *ep = NULL;

  if (device_address == 0) {
    // dedicate first endpoint for address0
    ep = PIO_USB_HW_EP(0);
  }else {
    for (int ep_pool_idx = 1; ep_pool_idx < PIO_USB_EP_POOL_CNT; ep_pool_idx++) {
      // ep size is used as valid indicator
      if (PIO_USB_HW_EP(ep_pool_idx)->size == 0) {
        ep = PIO_USB_HW_EP(ep_pool_idx);
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

void pio_usb_host_close_device(uint8_t root_idx, uint8_t device_address)
{
  for (int ep_pool_idx = 0; ep_pool_idx < PIO_USB_EP_POOL_CNT; ep_pool_idx++) {
    pio_hw_endpoint_t *ep = PIO_USB_HW_EP(ep_pool_idx);
    if ( (ep->root_idx == root_idx) && (ep->dev_addr == device_address) && ep->size){
      ep->size = 0;
    }
  }
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

  ep->active = true;

  return true;
}

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

  ep->active = true;

  return true;
}

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

/*static*/ void __no_inline_not_in_flash_func(endpoint_transfer_finish)(pio_hw_endpoint_t * ep, uint32_t flag) {
  pio_hw_root_port_t *hw_root = PIO_USB_HW_RPORT(ep->root_idx);
  uint32_t const ep_mask = (1u << (ep-pio_hw_ep_pool));

  hw_root->ints |= flag;

  if (flag == PIO_USB_INTS_ENDPOINT_COMPLETE_BITS) {
    hw_root->ep_complete |= ep_mask;
  }else if (flag == PIO_USB_INTS_ENDPOINT_ERROR_BITS) {
    hw_root->ep_error |= ep_mask;
  }else if (flag == PIO_USB_INTS_ENDPOINT_STALLED_BITS) {
    hw_root->ep_stalled |= ep_mask;
  }else {
    // something wrong
  }

  ep->active = false;
}

/*static*/ int __no_inline_not_in_flash_func(endpoint_in_transaction)(pio_port_t* pp, pio_hw_endpoint_t * ep) {
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

/*static*/ uint8_t __no_inline_not_in_flash_func(endpoint_out_prepare_buf)(pio_hw_endpoint_t * ep, uint8_t* buf) {
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

/*static*/ int __no_inline_not_in_flash_func(endpoint_out_transaction)(pio_port_t* pp, pio_hw_endpoint_t * ep) {
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

void __no_inline_not_in_flash_func(send_setup_token)(const pio_port_t *pp,
                                                      uint8_t addr,
                                                      uint8_t ep_num) {
  uint8_t packet[] = {USB_SYNC, USB_PID_SETUP, 0, 0};
  uint16_t dat = ((uint16_t)(ep_num & 0xf) << 7) | (addr & 0x7f);
  uint8_t crc = calc_usb_crc5(dat);
  packet[2] = dat & 0xff;
  packet[3] = (crc << 3) | ((dat >> 8) & 0x1f);

  usb_transfer(pp, packet, sizeof(packet));
}

void __no_inline_not_in_flash_func(endpoint_setup_transaction)(
    pio_port_t *pp,  pio_hw_endpoint_t *ep) {

  ep->data_id = 0; // DATA0
  uint8_t setup_buf[8+4];
  endpoint_out_prepare_buf(ep, setup_buf);

  // Setup token
  prepare_receive(pp);

  send_setup_token(pp, ep->dev_addr, 0);
  // ensure previous tx complete
  while ((pp->pio_usb_tx->irq & IRQ_TX_COMP_MASK) == 0) {
    continue;
  }

  // Data
  usb_transfer(pp, setup_buf, sizeof(setup_buf));

  // Handshake
  start_receive(pp);
  wait_handshake(pp);

  ep->actual_len = ep->total_len; // should be 8

  if (pp->usb_rx_buffer[0] == USB_SYNC && pp->usb_rx_buffer[1] == USB_PID_ACK) {
    endpoint_transfer_finish(ep, PIO_USB_INTS_ENDPOINT_COMPLETE_BITS);
  }else{
    endpoint_transfer_finish(ep, PIO_USB_INTS_ENDPOINT_ERROR_BITS);
  }

  pp->usb_rx_buffer[1] = 0;  // reset buffer
}
