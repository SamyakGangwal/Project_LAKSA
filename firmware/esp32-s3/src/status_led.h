#pragma once
/* status_led.c/.h were never committed (handoff tree 4787d05). The bridge only
 * uses the LED to show micro-ROS connection state, so this build has no LED
 * indication: set_mode is a no-op. */
typedef enum {
    STATUS_LED_WAITING_FOR_MICRO_ROS = 0,
    STATUS_LED_MICRO_ROS_CONNECTED = 1,
} status_led_mode_t;

static inline void status_led_set_mode(status_led_mode_t mode) { (void)mode; }
