#define DT_DRV_COMPAT zmk_input_processor_kinetic_xy

#include "zephyr/dt-bindings/input/input-event-codes.h"
#include <drivers/input_processor.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(input_processor_kinetic_xy, CONFIG_ZMK_LOG_LEVEL);

// From ZMK `behavior_input_two_axis.c`
#if CONFIG_MINIMAL_LIBC
static float powf(float base, float exponent) {
  float power = 1.0f;
  for (; exponent >= 1.0f; exponent--) {
    power = power * base;
  }
  return power;
}
#else
#include <math.h>
#endif

enum DecayMode {
  Linear = 0,
  Squared = 1,
  Exponential = 2,
};

struct input_processor_kinetic_xy_config {
  uint8_t slot;
  uint32_t event_interval;

  enum DecayMode decay_mode;
  int32_t decay_rate;
  int32_t decay_factor;

  int32_t clamp_threshold;
  int32_t trigger_threshold;
};

struct input_processor_kinetic_xy_data {
  struct k_work_delayable tick_work;
  const struct device *device;
};

static bool kinetic_xy_toggle_slots[CONFIG_ZMK_KINETIC_XY_TOGGLE_STATES];

static void kinetic_xy_handle_work(struct k_work *work) {
  // TODO: implement
}

static int kinetic_xy_init(const struct device *device) {
  struct input_processor_kinetic_xy_data *state =
      (struct input_processor_kinetic_xy_data *)device->data;

  state->device = device;
  // NOTE: fill as `input_processor_kinetic_xy_data` grows

  k_work_init_delayable(&state->tick_work, kinetic_xy_handle_work);
  return 0;
}

static int kinetic_xy_handle_event(const struct device *device,
                                   struct input_event *event,
                                   const uint32_t param1, const uint32_t param2,
                                   struct zmk_input_processor_state *_state) {
  ARG_UNUSED(param1);
  ARG_UNUSED(param2);
  ARG_UNUSED(_state);

  // just ergonomics
  struct input_processor_kinetic_xy_data *state =
      (struct input_processor_kinetic_xy_data *)device->state;
  const struct input_processor_kinetic_xy_config *config = device->config;
  const uint8_t event_type = event->type;
  const uint16_t event_code = event->code;
  const int32_t event_value = event->value;

  // if (not sync) or (not relative), ignore and continue
  if (!event->sync) {
    return ZMK_INPUT_PROC_CONTINUE;
  }

  if (event_type == INPUT_EV_REL) {
    // fat guard
  } else if (event_type == INPUT_EV_KEY) {
    if (event_code == INPUT_BTN_TOUCH && event_value == 0) {
      // finger lifted, start
      // TODO: implement

    } else
      return ZMK_INPUT_PROC_CONTINUE;
  } else
    return ZMK_INPUT_PROC_CONTINUE;

  // TODO: implement
  if (event_code == INPUT_REL_X) {
  }
  if (event_code == INPUT_REL_Y) {
  }

  return ZMK_INPUT_PROC_CONTINUE;
}

static const struct zmk_input_processor_driver_api kinetic_xy_driver_api = {
    .handle_event = kinetic_xy_handle_event,
};
