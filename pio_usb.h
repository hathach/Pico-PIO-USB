
#pragma once

#include "pio_usb_configuration.h"
#include "usb_definitions.h"

// Host functions
usb_device_t *pio_usb_host_init(const pio_usb_configuration_t *c);
int pio_usb_host_add_port(uint8_t pin_dp);
void pio_usb_host_task(void);
void pio_usb_host_stop(void);
void pio_usb_host_restart(void);

// Device functions
usb_device_t *pio_usb_device_init(const pio_usb_configuration_t *c,
                                  const usb_descriptor_buffers_t *buffers);
void pio_usb_device_task(void);

// Common functions
endpoint_t *pio_usb_get_endpoint(usb_device_t *device, uint8_t idx);
int pio_usb_get_in_data(endpoint_t *ep, uint8_t *buffer, uint8_t len);
int pio_usb_set_out_data(endpoint_t *ep, const uint8_t *buffer, uint8_t len);

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+
extern root_port_t root_port[PIO_USB_ROOT_PORT_CNT];
#define PIO_USB(_root_idx)    (root_port + _root_idx)

extern endpoint_t ep_pool[PIO_USB_EP_POOL_CNT];
#define PIO_USB_EP(_ep_idx)     (ep_pool+_ep_idx)

__attribute__((weak)) void pio_usb_host_irq_handler(uint8_t root_idx);

//pio_usb_port_get_pin_status()
port_pin_status_t get_port_pin_status(root_port_t *port);

static inline void pio_usb_port_reset_start(root_port_t *root, pio_port_t *pp)
{
  // Force line state to SE0
  pio_sm_set_pins_with_mask(pp->pio_usb_tx, pp->sm_tx, (0b00 << root->pin_dp),
                            (0b11u << root->pin_dp));
  pio_sm_set_pindirs_with_mask(pp->pio_usb_tx, pp->sm_tx, (0b11u << root->pin_dp),
                               (0b11u << root->pin_dp));
}

static inline void pio_usb_port_reset_end(root_port_t *root, pio_port_t *pp)
{
  // line state to input
  pio_sm_set_pindirs_with_mask(pp->pio_usb_tx, pp->sm_tx, (0b00u << root->pin_dp),
                               (0b11u << root->pin_dp));
}


bool pio_usb_endpoint_open(uint8_t root_idx, uint8_t device_address, uint8_t const *desc_endpoint);
bool pio_usb_endpoint_send_setup(uint8_t root_idx, uint8_t device_address, uint8_t const setup_packet[8]);
bool pio_usb_endpoint_transfer(uint8_t root_idx, uint8_t device_address, uint8_t ep_address, uint8_t* buffer, uint32_t buflen);


