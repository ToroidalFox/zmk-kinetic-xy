#define DT_DRV_COMPAT zmk_behavior_kinetic_xy_toggle

#include <zephyr/device.h>
#include <zmk/behavior.h>
#include <zmk_kinetic_xy/kinetic_xy.h>

struct behavior_kinetic_xy_toggle_config {
  uint8_t slot;
};

static int
behavior_kinetic_xy_toggle_on_pressed(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
  const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
  const struct behavior_kinetic_xy_toggle_config *config = dev->config;
  input_processor_kinetic_xy_toggle(config->slot);
  return ZMK_BEHAVIOR_OPAQUE;
}

static int behavior_kinetic_xy_toggle_on_released(
    struct zmk_behavior_binding *binding,
    struct zmk_behavior_binding_event event) {
  return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_kinetic_xy_toggle_driver_api =
    {
        .binding_pressed = behavior_kinetic_xy_toggle_on_pressed,
        .binding_released = behavior_kinetic_xy_toggle_on_released,
};

#define KINETIC_XY_TOGGLE_INST(n)                                              \
  static struct behavior_kinetic_xy_toggle_config_##n = {                      \
      .slot = DT_INST_PROP(n, slot),                                           \
  };                                                                           \
  DEVICE_DT_INST_DEFINE(n, NULL, NULL, NULL,                                   \
                        &behavior_kinetic_xy_toggle_config_##n, POST_KERNEL,   \
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                   \
                        &behavior_kinetic_xy_toggle_driver_api);

// `n`, no init, no pm pt, no data pt, _config, POST_KERNEL, default prio, _api
// DEVICE_DT_INST_DEFINE // uncomment and goto definition for documentation

DT_INST_FOREACH_STATUS_OKAY(KINETIC_XY_TOGGLE_INST)
