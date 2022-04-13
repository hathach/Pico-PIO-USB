/**
 * Copyright (c) 2021 sekigon-gonnoc
 *                    Ha Thach (thach@tinyusb.org)
 */

#pragma once

#include "pio_usb_configuration.h"
#include "usb_definitions.h"

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

// IRQ Handler
__attribute__((weak)) void pio_usb_host_irq_handler(uint8_t root_idx);

void pio_usb_host_controller_init(const pio_usb_configuration_t *c);

void pio_usb_host_close_device(uint8_t root_idx, uint8_t device_address);

