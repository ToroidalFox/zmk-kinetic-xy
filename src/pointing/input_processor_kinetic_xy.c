#define DT_DRV_COMPAT zmk_input_processor_kinetic_xy

#include <drivers/input_processor.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/time_units.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys_clock.h>

LOG_MODULE_REGISTER(input_processor_kinetic_xy, CONFIG_ZMK_LOG_LEVEL);

int32_t i32_sat_mul(int32_t a, int32_t b) {
  int32_t result;
  if (__builtin_mul_overflow(a, b, &result)) {
    return (a < 0) == (b < 0) ? INT32_MAX : INT32_MIN;
  }
  return result;
}
typedef int32_t i22f10;
#define I22F10_SHIFT 10
#define I22F10_ONE (1 << 10)
#define I22F10_HALF (1 << 9)
static inline i22f10 i22f10_from(int32_t val) { return val << I22F10_SHIFT; }
static inline int32_t i32_from(i22f10 val) { return val / I22F10_ONE; }
static inline i22f10 vel_from_dpdt(int32_t dp, int32_t dt_us) {
  return CLAMP(((int64_t)dp << I22F10_SHIFT) * USEC_PER_SEC / (int64_t)dt_us,
               INT32_MIN, INT32_MAX);
}

struct input_processor_kinetic_xy_config {
  uint8_t toggle_slot;
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
  i22f10 value;
  i22f10 rem;
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

static bool KINETIC_XY_TOGGLE_SLOTS[CONFIG_ZMK_KINETIC_XY_TOGGLE_STATES];

void input_processor_kinetic_xy_toggle(uint8_t slot) {
  KINETIC_XY_TOGGLE_SLOTS[slot] = !KINETIC_XY_TOGGLE_SLOTS[slot];
}

static inline int32_t delta_ticks_to_us(int64_t t) {
  return CLAMP(k_ticks_to_us_near64(t), 0, USEC_PER_SEC);
}

static bool is_above_trigger_threshold(
    const struct input_processor_kinetic_xy_config *config,
    const struct input_processor_kinetic_xy_data *data) {
  int32_t threshold = config->trigger_threshold;
  int32_t x_vel = i32_from(data->x.value);
  int32_t y_vel = i32_from(data->y.value);
  return threshold * threshold <= x_vel * x_vel + y_vel * y_vel;
}
static bool
is_above_clamp_threshold(const struct input_processor_kinetic_xy_config *config,
                         const struct input_processor_kinetic_xy_data *data) {
  int32_t threshold = config->clamp_threshold;
  int32_t x_vel = i32_from(data->x.value);
  int32_t y_vel = i32_from(data->y.value);
  return threshold * threshold <= x_vel * x_vel + y_vel * y_vel;
}

static void kinetic_xy_handle_work(struct k_work *work) {
  struct k_work_delayable *_work = k_work_delayable_from_work(work);
  struct input_processor_kinetic_xy_data *data =
      CONTAINER_OF(_work, struct input_processor_kinetic_xy_data, tick_work);
  const struct device *device = data->device;
  const struct input_processor_kinetic_xy_config *config = device->config;

  if (!KINETIC_XY_TOGGLE_SLOTS[config->toggle_slot]) {
    data->x = (struct axis){.value = 0, .rem = 0};
    data->y = (struct axis){.value = 0, .rem = 0};
    return;
  }

  data->x.value = i32_sat_mul(data->x.value, 1000 - config->decay_rate) / 1000;
  data->y.value = i32_sat_mul(data->y.value, 1000 - config->decay_rate) / 1000;

  if (is_above_clamp_threshold(config, data)) {
    i22f10 dx =
        i32_sat_mul(data->x.value, config->event_interval) / 1000 + data->x.rem;
    i22f10 dy =
        i32_sat_mul(data->y.value, config->event_interval) / 1000 + data->y.rem;
    int32_t dx_int = i32_from(dx);
    int32_t dy_int = i32_from(dy);
    data->x.rem = (dx - (i22f10_from(dx_int)));
    data->y.rem = (dy - (i22f10_from(dy_int)));
    if (dx_int != 0)
      input_report_rel(device, INPUT_REL_X, dx_int, dy_int == 0, K_NO_WAIT);
    if (dy_int != 0)
      input_report_rel(device, INPUT_REL_Y, dy_int, true, K_NO_WAIT);

    k_work_schedule(&data->tick_work, K_MSEC(config->event_interval));
  } else {
    data->x = (struct axis){.value = 0, .rem = 0};
    data->y = (struct axis){.value = 0, .rem = 0};
  }
}

static int input_processor_kinetic_xy_init(const struct device *device) {
  struct input_processor_kinetic_xy_data *data = device->data;

  int64_t now = k_uptime_ticks();

  data->device = device;
  data->x =
      (struct axis){.value = 0, .rem = 0, .unit = Displacement, .time = now};
  data->y =
      (struct axis){.value = 0, .rem = 0, .unit = Displacement, .time = now};

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
        if (KINETIC_XY_TOGGLE_SLOTS[config->toggle_slot])
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
      data->x.rem = 0;
      data->x.unit = Velocity;
    } else {
      int64_t delta_us = delta_ticks_to_us(delta_ticks);

      if (delta_us == 0) {
        LOG_DBG("delta_us is 0");
        return ZMK_INPUT_PROC_CONTINUE;
      }
      data->x.value = vel_from_dpdt(event_value, delta_us);
    }
  }
  if (event_code == INPUT_REL_Y) {
    int64_t delta_ticks = now - data->y.time;
    data->y.time = now;
    if (data->y.unit == Displacement) {
      data->y.value = 0;
      data->y.rem = 0;
      data->y.unit = Velocity;
    } else {
      int64_t delta_us = delta_ticks_to_us(delta_ticks);

      if (delta_us == 0) {
        LOG_DBG("delta_us is 0");
        return ZMK_INPUT_PROC_CONTINUE;
      }
      data->y.value = vel_from_dpdt(event_value, delta_us);
    }
  }

  return ZMK_INPUT_PROC_CONTINUE;
}

static const struct zmk_input_processor_driver_api
    input_processor_kinetic_xy_driver_api = {
        .handle_event = kinetic_xy_handle_event,
};

#define KINETIC_XY_INST(n)                                                     \
  static struct input_processor_kinetic_xy_data_##n = {};                      \
  static struct input_processor_kinetic_xy_config_##n = {                      \
      .toggle_slot = DT_INST_PROP(n, toggle_slot),                             \
      .event_interval = DT_INST_PROP(n, event_interval),                       \
      .decay_rate = DT_INST_PROP(n, decay_rate),                               \
      .clamp_threshold = DT_INST_PROP(n, clamp_threshold),                     \
      .trigger_threshold = DT_INST_PROP(n, trigger_threshold),                 \
  };                                                                           \
  DEVICE_DT_INST_DEFINE(n, input_processor_kinetic_xy_init, NULL,              \
                        &input_processor_kinetic_xy_data_##n,                  \
                        &input_processor_kinetic_xy_config_##n, POST_KERNEL,   \
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                   \
                        &input_processor_kinetic_xy_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KINETIC_XY_INST)
