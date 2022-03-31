/**
 * Copyright (c) 2021 sekigon-gonnoc
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "pio_usb.h"

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
