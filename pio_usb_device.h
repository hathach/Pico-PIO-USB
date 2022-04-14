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
__attribute__((weak)) void pio_usb_device_irq_handler(uint8_t root_idx);

// void pio_usb_device_controller_init(const pio_usb_configuration_t *c);
//void pio_usb_device_set_address(uint8_t root_idx, uint8_t dev_addr);

void pio_usb_device_set_address(uint8_t root_idx, uint8_t dev_addr);
bool pio_usb_device_endpoint_open(uint8_t root_idx, uint8_t const *desc_endpoint);
bool pio_usb_device_endpoint_transfer(uint8_t root_idx, uint8_t ep_address, uint8_t* buffer, uint16_t buflen);
