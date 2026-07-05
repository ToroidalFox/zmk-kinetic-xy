#define DT_DRV_COMPAT zmk_input_processor_kinetic_xy

#include <drivers/input_processor.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/time_units.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys_clock.h>
#include <zephyr/toolchain.h>

LOG_MODULE_REGISTER(input_processor_kinetic_xy, CONFIG_ZMK_LOG_LEVEL);

int64_t i64_sat_mul(int64_t a, int64_t b) {
  int64_t result;
  if (__builtin_mul_overflow(a, b, &result)) {
    return (a < 0) == (b < 0) ? INT64_MAX : INT64_MIN;
  }
  return result;
}

struct input_processor_kinetic_xy_config {
  uint8_t slot;
  uint32_t event_interval;

  int32_t decay_rate;

  int32_t clamp_threshold;
  int32_t trigger_threshold;
};

enum Unit {
  Displacement,
  Velocity,
};

struct axis {
  int64_t value;
  enum Unit unit; // unit when next event comes in rather than right now
  int64_t time;
};

struct input_processor_kinetic_xy_data {
  struct k_work_delayable tick_work;
  const struct device *device;

  struct axis x;
  struct axis y;
  int64_t event_time;
};

static bool kinetic_xy_toggle_slots[CONFIG_ZMK_KINETIC_XY_TOGGLE_STATES];

static void input_processor_kinetic_xy_toggle(uint8_t slot) {
  kinetic_xy_toggle_slots[slot] = !kinetic_xy_toggle_slots[slot];
}

static bool is_above_trigger_threshold(
    const struct input_processor_kinetic_xy_config *config,
    const struct input_processor_kinetic_xy_data *data) {
  int32_t threshold = config->trigger_threshold;
  int32_t x_vel = data->x.value;
  int32_t y_vel = data->y.value;
  return threshold * threshold <= x_vel * x_vel + y_vel * y_vel;
}
static bool
is_above_clamp_threshold(const struct input_processor_kinetic_xy_config *config,
                         const struct input_processor_kinetic_xy_data *data) {
  int32_t threshold = config->clamp_threshold;
  int32_t x_vel = data->x.value;
  int32_t y_vel = data->y.value;
  return threshold * threshold <= x_vel * x_vel + y_vel * y_vel;
}

static void kinetic_xy_handle_work(struct k_work *work) {
  // TODO: implement
}

static int kinetic_xy_init(const struct device *device) {
  struct input_processor_kinetic_xy_data *data = device->data;

  int64_t now = k_uptime_ticks();

  data->device = device;
  data->x = (struct axis){.value = 0, .unit = Displacement, .time = now};
  data->y = (struct axis){.value = 0, .unit = Displacement, .time = now};
  // TODO: fill as `input_processor_kinetic_xy_data` grows

  k_work_init_delayable(&data->tick_work, kinetic_xy_handle_work);
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
  struct input_processor_kinetic_xy_data *data =
      (struct input_processor_kinetic_xy_data *)device->data;
  const struct input_processor_kinetic_xy_config *config = device->config;
  const uint8_t event_type = event->type;
  const uint16_t event_code = event->code;
  const int32_t event_value = event->value;
  int64_t now = k_uptime_ticks();

  if (event_type == INPUT_EV_REL) {
    goto marker_relative_input;
  } else if (event_type == INPUT_EV_KEY) {
    if (event_code == INPUT_BTN_TOUCH && event_value == 0) {
      if ((data->x.unit == Velocity || data->y.unit == Velocity) &&
          is_above_trigger_threshold(config, data)) {
        data->event_time = now;
        k_work_reschedule(&data->tick_work, K_MSEC(config->event_interval));
      }
      data->x.unit = data->y.unit = Displacement;
    }
  }
  return ZMK_INPUT_PROC_CONTINUE;

marker_relative_input:
  k_work_cancel_delayable(&data->tick_work);
  if (event_code == INPUT_REL_X) {
    int64_t delta_ticks = now - data->x.time;
    data->x.time = now;
    if (data->x.unit == Displacement) {
      data->x.value = 0;
      data->x.unit = Velocity;
    } else {
      int64_t delta_ns =
          CLAMP(k_ticks_to_ns_near64(delta_ticks), 0, NSEC_PER_SEC);

      if (delta_ns == 0) {
        LOG_DBG("delta_ns is 0");
        return ZMK_INPUT_PROC_CONTINUE;
      }
      data->x.value = i64_sat_mul(event_value, NSEC_PER_SEC) / delta_ns;
    }
  }
  if (event_code == INPUT_REL_Y) {
    int64_t delta_ticks = now - data->y.time;
    data->y.time = now;
    if (data->y.unit == Displacement) {
      data->y.value = 0;
      data->y.unit = Velocity;
    } else {
      int64_t delta_ns =
          CLAMP(k_ticks_to_ns_near64(delta_ticks), 0, NSEC_PER_SEC);

      if (delta_ns == 0) {
        LOG_DBG("delta_ns is 0");
        return ZMK_INPUT_PROC_CONTINUE;
      }
      data->y.value = i64_sat_mul(event_value, NSEC_PER_SEC) / delta_ns;
    }
  }

  return ZMK_INPUT_PROC_CONTINUE;
}

static const struct zmk_input_processor_driver_api kinetic_xy_driver_api = {
    .handle_event = kinetic_xy_handle_event,
};
