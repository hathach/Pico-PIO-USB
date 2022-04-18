/**
 * Copyright (c) 2021 sekigon-gonnoc
 */

#pragma GCC push_options
#pragma GCC optimize("-O3")

#include <stdio.h>
#include <stdint.h>
#include <string.h> // memcpy

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "hardware/irq.h"
#include "pico/bootrom.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "pio_usb.h"
#include "usb_definitions.h"
#include "usb_crc.h"
#include "usb_tx.pio.h"
#include "usb_rx.pio.h"

#include "tusb.h" // for logging

#define UNUSED_PARAMETER(x) (void)x

#define IRQ_TX_EOP_MASK (1 << usb_tx_fs_IRQ_EOP)
#define IRQ_TX_COMP_MASK (1 << usb_tx_fs_IRQ_COMP)
#define IRQ_TX_ALL_MASK (IRQ_TX_EOP_MASK | IRQ_TX_COMP_MASK)
#define IRQ_RX_COMP_MASK (1 << IRQ_RX_EOP)
#define IRQ_RX_ALL_MASK ((1 << IRQ_RX_EOP) | (1 << IRQ_RX_BS_ERR) | (1 << IRQ_RX_START))

//typedef struct {
//  uint16_t div_int;
//  uint8_t div_frac;
//} pio_clk_div_t;

//typedef struct {
//  PIO pio_usb_tx;  // colud not set to volatile
//  uint sm_tx;
//  uint offset_tx;
//  uint tx_ch;
//
//  PIO pio_usb_rx;  // colud not set to volatile
//  uint sm_rx;
//  uint offset_rx;
//  uint sm_eop;
//  uint offset_eop;
//  uint rx_reset_instr;
//  uint device_rx_irq_num;
//
//  pio_clk_div_t clk_div_fs_tx;
//  pio_clk_div_t clk_div_fs_rx;
//  pio_clk_div_t clk_div_ls_tx;
//  pio_clk_div_t clk_div_ls_rx;
//
//  bool need_pre;
//
//  uint8_t usb_rx_buffer[128];
//} pio_port_t;

/*static*/ usb_device_t usb_device[PIO_USB_DEVICE_CNT];
/*static*/ pio_port_t pio_port[1];
/*static*/ root_port_t root_port[PIO_USB_ROOT_PORT_CNT];
/*static*/ endpoint_t ep_pool[PIO_USB_EP_POOL_CNT];

/*static*/ pio_usb_configuration_t current_config;

#define SM_SET_CLKDIV(pio, sm, div) pio_sm_set_clkdiv_int_frac(pio, sm, div.div_int, div.div_frac)

/*static*/ void __no_inline_not_in_flash_func(send_pre)(const pio_port_t *pp) {
  uint8_t data[] = {USB_SYNC, USB_PID_PRE};

  // send PRE token in full-speed
  pio_sm_set_enabled(pp->pio_usb_tx, pp->sm_tx, false);
  for (uint i = 0; i < USB_TX_EOP_DISABLER_LEN; ++i) {
    uint16_t instr = usb_tx_fs_pre_program.instructions[i + USB_TX_EOP_OFFSET];
    pp->pio_usb_tx->instr_mem[pp->offset_tx + i + USB_TX_EOP_OFFSET] = instr;
  }

  SM_SET_CLKDIV(pp->pio_usb_tx, pp->sm_tx, pp->clk_div_fs_tx);

  dma_channel_transfer_from_buffer_now(pp->tx_ch, data, 2);

  pio_sm_set_enabled(pp->pio_usb_tx, pp->sm_tx, true);
  pp->pio_usb_tx->irq |= IRQ_TX_ALL_MASK;  // clear complete flag
  pp->pio_usb_tx->irq_force |= IRQ_TX_EOP_MASK;  // disable eop

  while ((pp->pio_usb_tx->irq & IRQ_TX_COMP_MASK) == 0) {
    continue;
  }

  // change bus speed to low-speed
  pio_sm_set_enabled(pp->pio_usb_tx, pp->sm_tx, false);
  for (uint i = 0; i < USB_TX_EOP_DISABLER_LEN; ++i) {
    uint16_t instr = usb_tx_fs_program.instructions[i + USB_TX_EOP_OFFSET];
    pp->pio_usb_tx->instr_mem[pp->offset_tx + i + USB_TX_EOP_OFFSET] = instr;
  }
  SM_SET_CLKDIV(pp->pio_usb_tx, pp->sm_tx, pp->clk_div_ls_tx);

  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, false);
  SM_SET_CLKDIV(pp->pio_usb_rx, pp->sm_rx, pp->clk_div_ls_rx);

  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_eop, false);
  SM_SET_CLKDIV(pp->pio_usb_rx, pp->sm_eop, pp->clk_div_ls_rx);
  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_eop, true);
}

/*static*/ void __not_in_flash_func(usb_transfer)(const pio_port_t *pp,
                                              uint8_t *data, uint16_t len) {
  if (pp->need_pre) {
    send_pre(pp);
  }

  dma_channel_transfer_from_buffer_now(pp->tx_ch, data, len);

  pio_sm_set_enabled(pp->pio_usb_tx, pp->sm_tx, true);
  pp->pio_usb_tx->irq |= IRQ_TX_ALL_MASK;  // clear complete flag

  while ((pp->pio_usb_tx->irq & IRQ_TX_ALL_MASK) == 0) {
    continue;
  }
}

/*static*/ void __no_inline_not_in_flash_func(send_ack)(const pio_port_t *pp) {
  uint8_t data[] = {USB_SYNC, USB_PID_ACK};
  usb_transfer(pp, data, sizeof(data));
}

void __no_inline_not_in_flash_func(send_nak)(const pio_port_t *pp) {
  uint8_t data[] = {USB_SYNC, USB_PID_NAK};
  usb_transfer(pp, data, sizeof(data));
}

void __no_inline_not_in_flash_func(send_handshake)(const pio_port_t *pp, uint8_t pid) {
  uint8_t data[] = {USB_SYNC, pid};
  usb_transfer(pp, data, sizeof(data));
}

/*static*/ void __no_inline_not_in_flash_func(prepare_receive)(const pio_port_t *pp) {
  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, false);
  pio_sm_clear_fifos(pp->pio_usb_rx, pp->sm_rx);
  pio_sm_restart(pp->pio_usb_rx, pp->sm_rx);
  pio_sm_exec(pp->pio_usb_rx, pp->sm_rx, pp->rx_reset_instr);
}

/*static*/ void __no_inline_not_in_flash_func(start_receive)(const pio_port_t *pp) {
  pp->pio_usb_rx->ctrl |= (1 << pp->sm_rx);
  pp->pio_usb_rx->irq |= IRQ_RX_ALL_MASK;
}

/*static*/ int __no_inline_not_in_flash_func(receive_packet_and_ack)(pio_port_t* pp, bool ack_response) {
  uint16_t crc = 0xffff;
  uint16_t crc_prev = 0xffff;
  uint16_t crc_prev2 = 0xffff;
  uint16_t crc_receive = 0xffff;
  bool  crc_match = false;
  int16_t t = 240;
  uint16_t idx = 0;

  while (t--) {
    if (pio_sm_get_rx_fifo_level(pp->pio_usb_rx, pp->sm_rx)) {
      uint8_t data = pio_sm_get(pp->pio_usb_rx, pp->sm_rx) >> 24;
      pp->usb_rx_buffer[idx++] = data;
      if (idx == 2) {
        break;
      }
    }
  }

  // timing critical start
  if (t > 0) {
    while ((pp->pio_usb_rx->irq & IRQ_RX_COMP_MASK) == 0) {
      if (pio_sm_get_rx_fifo_level(pp->pio_usb_rx, pp->sm_rx)) {
        uint8_t data = pio_sm_get(pp->pio_usb_rx, pp->sm_rx) >> 24;
        if (ack_response)
        {
          crc_prev2 = crc_prev;
          crc_prev = crc;
          crc = update_usb_crc16(crc, data);
          pp->usb_rx_buffer[idx++] = data;
          crc_receive = (crc_receive >> 8) | (data << 8);
          crc_match = ((crc_receive ^ 0xffff) == crc_prev2);
        }
      }
    }
  }

  if (ack_response) {
    if (idx >= 4 && crc_match) {
      send_ack(pp);
      // timing critical end
      return idx - 4;
    }
  }else {
    send_nak(pp);
  }

  return -1;
}

//typedef enum{
//  PORT_PIN_SE0,
//  PORT_PIN_FS_IDLE,
//  PORT_PIN_LS_IDLE,
//  PORT_PIN_SE1,
//} port_pin_status_t;

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


/*static*/ __always_inline void add_pio_host_rx_program(PIO pio,
                                             const pio_program_t *program,
                                             const pio_program_t *debug_program,
                                             uint *offset, int debug_pin) {
  if (debug_pin < 0) {
    *offset = pio_add_program(pio, program);
  } else {
    *offset = pio_add_program(pio, debug_program);
  }
}

/*static*/ void __no_inline_not_in_flash_func(initialize_host_programs)(
    pio_port_t *pp, const pio_usb_configuration_t *c, root_port_t *port) {
  pp->offset_tx = pio_add_program(pp->pio_usb_tx, &usb_tx_fs_program);
  usb_tx_fs_program_init(pp->pio_usb_tx, pp->sm_tx, pp->offset_tx,
                         port->pin_dp);

  add_pio_host_rx_program(pp->pio_usb_rx, &usb_rx_fs_program,
                          &usb_rx_fs_debug_program, &pp->offset_rx,
                          c->debug_pin_rx);
  usb_rx_fs_program_init(pp->pio_usb_rx, pp->sm_rx, pp->offset_rx, port->pin_dp,
                         c->debug_pin_rx);
  pp->rx_reset_instr = pio_encode_jmp(pp->offset_rx);

  add_pio_host_rx_program(pp->pio_usb_rx, &eop_detect_fs_program,
                          &eop_detect_fs_debug_program, &pp->offset_eop,
                          c->debug_pin_eop);
  eop_detect_fs_program_init(pp->pio_usb_rx, c->sm_eop, pp->offset_eop,
                             port->pin_dp, true, c->debug_pin_eop);

  usb_tx_configure_pins(pp->pio_usb_tx, pp->sm_tx, port->pin_dp);
  usb_rx_configure_pins(pp->pio_usb_rx, pp->sm_eop, port->pin_dp);
  usb_rx_configure_pins(pp->pio_usb_rx, pp->sm_rx, port->pin_dp);
}

/*static*/ void update_packet_crc16(usb_setup_packet_t * packet) {
  uint16_t crc16 = calc_usb_crc16(&packet->request_type,
                                  sizeof(*packet) - 4);
  packet->crc16[0] = crc16 & 0xff;
  packet->crc16[1] = crc16 >> 8;
}

int __no_inline_not_in_flash_func(pio_usb_get_in_data)(endpoint_t *ep,
                                                       uint8_t *buffer,
                                                       uint8_t len) {
  if ((ep->ep_num & EP_IN) && ep->new_data_flag) {
    len = len < ep->packet_len ? len : ep->packet_len;
    memcpy(buffer, (void *)ep->buffer, len);

    ep->new_data_flag = false;

    return len;
  }

  return -1;
}

int __no_inline_not_in_flash_func(pio_usb_set_out_data)(endpoint_t *ep,
                                                          const uint8_t *buffer,
                                                          uint8_t len) {
  if (ep->new_data_flag || (ep->ep_num & EP_IN)) {
    return -1;
  }

  ep->buffer[0] = USB_SYNC;
  ep->buffer[1] = ep->data_id == 0 ? USB_PID_DATA0 : USB_PID_DATA1;
  memcpy((uint8_t *)&ep->buffer[2], buffer, len);
  uint16_t crc = calc_usb_crc16(buffer, len);
  ep->buffer[2 + len] = crc & 0xff;
  ep->buffer[2 + len + 1] = (crc >> 8) & 0xff;
  ep->packet_len = len + 4;

  ep->new_data_flag = true;



  return 0;
}

/*static*/ void configure_tx_channel(uint8_t ch, PIO pio, uint sm) {
  dma_channel_config conf = dma_channel_get_default_config(ch);

  channel_config_set_read_increment(&conf, true);
  channel_config_set_write_increment(&conf, false);
  channel_config_set_transfer_data_size(&conf, DMA_SIZE_8);
  channel_config_set_dreq(&conf, pio_get_dreq(pio, sm, true));

  dma_channel_set_config(ch, &conf, false);
  dma_channel_set_write_addr(ch, &pio->txf[sm], false);
}

/*static*/ void apply_config(pio_port_t *pp, const pio_usb_configuration_t *c,
                         root_port_t *port) {
  pp->pio_usb_tx = c->pio_tx_num == 0 ? pio0 : pio1;
  pp->sm_tx = c->sm_tx;
  pp->tx_ch = c->tx_ch;
  pp->pio_usb_rx = c->pio_rx_num == 0 ? pio0 : pio1;
  pp->sm_rx = c->sm_rx;
  pp->sm_eop = c->sm_eop;
  port->pin_dp = c->pin_dp;
  port->pin_dm = c->pin_dp + 1;

  pp->debug_pin_rx = c->debug_pin_rx;
  pp->debug_pin_eop = c->debug_pin_eop;
}

/*static*/ repeating_timer_t sof_rt;
/*static*/ bool timer_active;

/*static*/ void start_timer(alarm_pool_t *alarm_pool) {
  if (timer_active) {
    return;
  }

//  if (alarm_pool != NULL) {
//    alarm_pool_add_repeating_timer_us(alarm_pool, -1000, sof_timer, NULL,
//                                      &sof_rt);
//  } else {
//    add_repeating_timer_us(-1000, sof_timer, NULL, &sof_rt);
//  }

  timer_active = true;
}

/*static*/ void stop_timer(void) {
  cancel_repeating_timer(&sof_rt);
  timer_active = false;
}

/*static*/ void port_pin_drive_setting(const root_port_t *port) {
  gpio_set_slew_rate(port->pin_dp, GPIO_SLEW_RATE_FAST);
  gpio_set_slew_rate(port->pin_dm, GPIO_SLEW_RATE_FAST);
  gpio_set_drive_strength(port->pin_dp, GPIO_DRIVE_STRENGTH_12MA);
  gpio_set_drive_strength(port->pin_dm, GPIO_DRIVE_STRENGTH_12MA);
}

endpoint_t *pio_usb_get_endpoint(usb_device_t *device, uint8_t idx) {
  uint8_t ep_id = device->endpoint_id[idx];
  if (ep_id == 0) {
    return NULL;
  } else if (ep_id >= 1) {
    return &ep_pool[ep_id - 1];
  }
  return NULL;
}

usb_device_t *pio_usb_host_init(const pio_usb_configuration_t *c) {
  pio_port_t *pp = &pio_port[0];
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

  start_timer(c->alarm_pool);

  current_config = *c;

  return &usb_device[0];
}

int pio_usb_host_add_port(uint8_t pin_dp) {
  for (int idx = 0; idx < PIO_USB_ROOT_PORT_CNT; idx++) {
    if (!root_port[idx].initialized) {
      root_port[idx].pin_dp = pin_dp;
      root_port[idx].pin_dm = pin_dp + 1;

      PIO_USB_HW_RPORT(idx)->pin_dp = pin_dp;
      PIO_USB_HW_RPORT(idx)->pin_dm = pin_dp+1;

      gpio_pull_down(pin_dp);
      gpio_pull_down(pin_dp + 1);
      pio_gpio_init(pio_port[0].pio_usb_tx, pin_dp);
      pio_gpio_init(pio_port[0].pio_usb_tx, pin_dp + 1);
      pio_sm_set_pindirs_with_mask(pio_port[0].pio_usb_tx, pio_port[0].sm_tx, 0,
                                   (0b11 << pin_dp));
      port_pin_drive_setting(&root_port[idx]);
      root_port[idx].initialized = true;

      return 0;
    }
  }

  return -1;
}

/*static*/ volatile bool cancel_timer_flag;
/*static*/ volatile bool start_timer_flag;
/*static*/ uint32_t int_stat;

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

//
// Device implementation
//

static uint8_t new_devaddr = 0;
static uint8_t ep0_crc5_lut[16];

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

void pio_usb_device_set_address(uint8_t root_idx, uint8_t dev_addr) {
  (void) root_idx;
  new_devaddr = dev_addr;
}

/*static*/ void __no_inline_not_in_flash_func(usb_device_packet_handler)(void) {
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

  if (token_buf[0] == 0 && get_port_pin_status(&root_port[0]) == PORT_PIN_SE0) {
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

usb_device_t *pio_usb_device_init(const pio_usb_configuration_t *c,
                                  const usb_descriptor_buffers_t *buffers) {
  pio_port_t *pp = &pio_port[0];
  usb_device_t *dev = &usb_device[0];
  pio_hw_root_port_t* hw_root = PIO_USB_HW_RPORT(0);

  pp->pio_usb_tx = c->pio_tx_num == 0 ? pio0 : pio1;
  configure_tx_channel(c->tx_ch, pp->pio_usb_tx, c->sm_tx);


  apply_config(pp, c, &root_port[0]);
  initialize_host_programs(pp, c, &root_port[0]);
  port_pin_drive_setting(&root_port[0]);
  root_port[0].initialized = true;

  hw_root->initialized = true;
  hw_root->mode = PIO_USB_MODE_DEVICE;
  hw_root->pin_dp = c->pin_dp;
  hw_root->pin_dm = c->pin_dp+1;
  hw_root->dev_addr = 0;

  memset(dev, 0, sizeof(*dev));
  for (int i = 0; i < PIO_USB_DEV_EP_CNT; i++) {
    dev->endpoint_id[i] = i + 1;
  }
  update_ep0_crc5_lut(hw_root->dev_addr);

  pio_calculate_clkdiv_from_float((float)clock_get_hz(clk_sys) / 48000000,
                                  &pp->clk_div_fs_tx.div_int,
                                  &pp->clk_div_fs_tx.div_frac);
  pio_calculate_clkdiv_from_float((float)clock_get_hz(clk_sys) / 96000000,
                                  &pp->clk_div_fs_rx.div_int,
                                  &pp->clk_div_fs_rx.div_frac);

  current_config = *c;

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

  return dev;
}

#pragma GCC pop_options
