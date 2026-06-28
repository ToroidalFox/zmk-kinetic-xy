#define DT_DRV_COMPAT zmk_behavior_kinetic_xy_toggle

#include <zmk/behavior.h>
#include <zmk/input/kinetic_xy.h>

struct behavior_kinetic_xy_toggle_config {
  uint8_t slot;
};

static int
behavior_kinetic_xy_toggle_on_pressed(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
  const struct device *dev = device_get_binding(binding->behavior_dev);
  const struct behavior_kinetic_xy_toggle_config *config =
      (const struct behavior_kinetic_xy_toggle_config *)dev->config;

  // TODO: toggle kinetic_xy

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

DT_INST_FOREACH_STATUS_OKAY(KINETIC_XY_TOGGLE_INST)
