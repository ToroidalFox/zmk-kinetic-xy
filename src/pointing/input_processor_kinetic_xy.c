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

static inline int32_t i32_sat_mul(int32_t a, int32_t b) {
  int32_t result;
  if (__builtin_mul_overflow(a, b, &result)) {
    return (a < 0) == (b < 0) ? INT32_MAX : INT32_MIN;
  }
  return result;
}
typedef int32_t fp;
#define FP_SHIFT 10
#define FP_ONE (1 << FP_SHIFT)
#define FP_HALF (1 << (FP_SHIFT - 1))
static inline fp fp_from(int32_t val) { return val << FP_SHIFT; }
static inline int32_t i32_from(fp val) { return val / FP_ONE; }
static inline fp vel_from_dpdt(int32_t dp, int64_t dt_us) {
  return CLAMP(((int64_t)dp << FP_SHIFT) * USEC_PER_SEC / dt_us, INT32_MIN,
               INT32_MAX);
}

struct input_processor_kinetic_xy_config {
  uint8_t toggle_slot;

  uint32_t event_delay;
  uint32_t event_interval;
  int32_t decay_rate;

  int32_t clamp_threshold;
  int32_t trigger_threshold;
};

struct axis {
  fp value;
  fp rem;
  int64_t time;
};

struct input_processor_kinetic_xy_data {
  struct k_work_delayable tick_work;
  const struct device *device;

  struct axis x;
  struct axis y;
};

static bool KINETIC_XY_TOGGLE_SLOTS[CONFIG_ZMK_KINETIC_XY_TOGGLE_STATES] = {
    [0 ... CONFIG_ZMK_KINETIC_XY_TOGGLE_STATES - 1] = true};
static inline bool
is_enabled(const struct input_processor_kinetic_xy_config *config) {
  return KINETIC_XY_TOGGLE_SLOTS[config->toggle_slot];
}

void input_processor_kinetic_xy_toggle(uint8_t slot) {
  KINETIC_XY_TOGGLE_SLOTS[slot] = !KINETIC_XY_TOGGLE_SLOTS[slot];
}

static inline int64_t delta_ticks_to_us(int64_t t) {
  return MAX(k_ticks_to_us_near64(t), 0);
}

static bool
is_above_threshold(const int32_t threshold,
                   const struct input_processor_kinetic_xy_data *data) {
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
  int64_t now = k_uptime_ticks();

  if (!is_enabled(config)) {
    data->x = (struct axis){.value = 0, .rem = 0, .time = now};
    data->y = (struct axis){.value = 0, .rem = 0, .time = now};
    return;
  }

  data->x.value = i32_sat_mul(data->x.value, 1000 - config->decay_rate) / 1000;
  data->y.value = i32_sat_mul(data->y.value, 1000 - config->decay_rate) / 1000;
  data->x.time = now;
  data->y.time = now;

  if (is_above_threshold(config->clamp_threshold, data)) {
    fp dx =
        i32_sat_mul(data->x.value, config->event_interval) / 1000 + data->x.rem;
    fp dy =
        i32_sat_mul(data->y.value, config->event_interval) / 1000 + data->y.rem;
    int32_t dx_int = i32_from(dx);
    int32_t dy_int = i32_from(dy);
    data->x.rem = (dx - (fp_from(dx_int)));
    data->y.rem = (dy - (fp_from(dy_int)));
    if (dx_int != 0)
      input_report_rel(device, INPUT_REL_X, dx_int, dy_int == 0, K_NO_WAIT);
    if (dy_int != 0)
      input_report_rel(device, INPUT_REL_Y, dy_int, true, K_NO_WAIT);

    k_work_reschedule(&data->tick_work, K_MSEC(config->event_interval));
  } else {
    LOG_DBG("kinetic movement is slower than clamp threshold");
    data->x = (struct axis){.value = 0, .rem = 0};
    data->y = (struct axis){.value = 0, .rem = 0};
  }
}

static int input_processor_kinetic_xy_init(const struct device *device) {
  struct input_processor_kinetic_xy_data *data = device->data;
  const struct input_processor_kinetic_xy_config *config = device->config;
  LOG_DBG("initializing kinetic-xy, enabled: %s",
          is_enabled(config) ? "true" : "false");

  int64_t now = k_uptime_ticks();

  data->device = device;
  data->x = (struct axis){.value = 0, .rem = 0, .time = now};
  data->y = (struct axis){.value = 0, .rem = 0, .time = now};

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

  if (event_type != INPUT_EV_REL) {
    return ZMK_INPUT_PROC_CONTINUE;
  }

  if (event_code == INPUT_REL_X) {
    int64_t delta_ticks = now - data->x.time;
    data->x.time = now;
    int64_t delta_us = delta_ticks_to_us(delta_ticks);

    if (delta_us == 0) {
      return ZMK_INPUT_PROC_CONTINUE;
    }
    fp vel = vel_from_dpdt(event_value, delta_us);
    LOG_DBG("current x vel: %d", i32_from(vel));
    data->x.value = vel;
  }
  if (event_code == INPUT_REL_Y) {
    int64_t delta_ticks = now - data->y.time;
    data->y.time = now;
    int64_t delta_us = delta_ticks_to_us(delta_ticks);

    if (delta_us == 0) {
      return ZMK_INPUT_PROC_CONTINUE;
    }
    fp vel = vel_from_dpdt(event_value, delta_us);
    LOG_DBG("current y vel: %d", i32_from(vel));
    data->y.value = vel;
  }
  if (event->sync) {
    if (is_above_threshold(config->trigger_threshold, data)) {
      k_work_reschedule(&data->tick_work, K_MSEC(config->event_delay));
    } else {
      k_work_cancel_delayable(&data->tick_work);
    }
  }

  return ZMK_INPUT_PROC_CONTINUE;
}

static const struct zmk_input_processor_driver_api
    input_processor_kinetic_xy_driver_api = {
        .handle_event = kinetic_xy_handle_event,
};

#define KINETIC_XY_INST(n)                                                     \
  static struct input_processor_kinetic_xy_data                                \
      input_processor_kinetic_xy_data_##n = {};                                \
  static const struct input_processor_kinetic_xy_config                        \
      input_processor_kinetic_xy_config_##n = {                                \
          .toggle_slot = DT_INST_PROP(n, toggle_slot),                         \
          .event_delay = DT_INST_PROP(n, event_delay),                         \
          .event_interval = DT_INST_PROP(n, event_interval),                   \
          .decay_rate = DT_INST_PROP(n, decay_rate),                           \
          .clamp_threshold = DT_INST_PROP(n, clamp_threshold),                 \
          .trigger_threshold = DT_INST_PROP(n, trigger_threshold),             \
  };                                                                           \
  DEVICE_DT_INST_DEFINE(n, input_processor_kinetic_xy_init, NULL,              \
                        &input_processor_kinetic_xy_data_##n,                  \
                        &input_processor_kinetic_xy_config_##n, POST_KERNEL,   \
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                   \
                        &input_processor_kinetic_xy_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KINETIC_XY_INST)
