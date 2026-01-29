/*
 * Copyright (c) 2026 matchey
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define DT_DRV_COMPAT zmk_input_processor_axis_constrain

#include <drivers/input_processor.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(axis_constrain, CONFIG_ZMK_LOG_LEVEL);

enum axis_state {
  AXIS_NONE = 0,
  AXIS_X,
  AXIS_Y,
};

struct axis_constrain_config {
  int  threshold;
  bool sticky;
  int  release_after_ms;
  bool track_remainders;
};

struct axis_constrain_data {
  const struct device    *dev;
  enum axis_state         locked_axis;
  int32_t                 accum_x;
  int32_t                 accum_y;
  int32_t                 remainder_x;
  int32_t                 remainder_y;
  struct k_work_delayable release_work;
};

static void release_work_handler(struct k_work *work) {
  struct k_work_delayable    *dwork = k_work_delayable_from_work(work);
  struct axis_constrain_data *data  = CONTAINER_OF(dwork, struct axis_constrain_data, release_work);

  LOG_DBG("Releasing axis lock");
  data->locked_axis = AXIS_NONE;
  data->accum_x     = 0;
  data->accum_y     = 0;
  data->remainder_x = 0;
  data->remainder_y = 0;
}

static int axis_constrain_handle_event(const struct device *dev, struct input_event *event,
                                       uint32_t param1, uint32_t param2,
                                       struct zmk_input_processor_state *state) {
  const struct axis_constrain_config *config = dev->config;
  struct axis_constrain_data         *data   = dev->data;

  /* Only process REL_X and REL_Y events */
  if (event->type != INPUT_EV_REL) {
    return 0;
  }

  if (event->code != INPUT_REL_X && event->code != INPUT_REL_Y) {
    return 0;
  }

  /* Cancel and reschedule release timer if sticky */
  if (config->sticky) {
    k_work_cancel_delayable(&data->release_work);
    k_work_schedule(&data->release_work, K_MSEC(config->release_after_ms));
  }

  int32_t value = event->value;
  bool    is_x  = (event->code == INPUT_REL_X);

  /* Accumulate movement */
  if (is_x) {
    data->accum_x += (value > 0) ? value : -value;
  } else {
    data->accum_y += (value > 0) ? value : -value;
  }

  /* Determine axis if not yet locked */
  if (data->locked_axis == AXIS_NONE) {
    if (data->accum_x >= config->threshold && data->accum_x > data->accum_y) {
      data->locked_axis = AXIS_X;
      LOG_DBG("Locked to X axis");
    } else if (data->accum_y >= config->threshold && data->accum_y > data->accum_x) {
      data->locked_axis = AXIS_Y;
      LOG_DBG("Locked to Y axis");
    }
  }

  /* Apply axis constraint */
  if (data->locked_axis == AXIS_X && !is_x) {
    if (config->track_remainders) {
      data->remainder_y += value;
    }
    event->value = 0;
    LOG_DBG("Suppressed Y: %d", value);
  } else if (data->locked_axis == AXIS_Y && is_x) {
    if (config->track_remainders) {
      data->remainder_x += value;
    }
    event->value = 0;
    LOG_DBG("Suppressed X: %d", value);
  }

  return 0;
}

static struct zmk_input_processor_driver_api axis_constrain_api = {
    .handle_event = axis_constrain_handle_event,
};

static int axis_constrain_init(const struct device *dev) {
  struct axis_constrain_data *data = dev->data;

  data->dev         = dev;
  data->locked_axis = AXIS_NONE;
  data->accum_x     = 0;
  data->accum_y     = 0;
  data->remainder_x = 0;
  data->remainder_y = 0;

  k_work_init_delayable(&data->release_work, release_work_handler);

  LOG_DBG("Axis constrain processor initialized");
  return 0;
}

#define AC_INST(n)                                                              \
  static struct axis_constrain_data         axis_constrain_data_##n   = {};     \
  static const struct axis_constrain_config axis_constrain_config_##n = {       \
      .threshold        = DT_INST_PROP(n, threshold),                           \
      .sticky           = DT_INST_PROP(n, sticky),                              \
      .release_after_ms = DT_INST_PROP(n, release_after_ms),                    \
      .track_remainders = DT_INST_PROP(n, track_remainders),                    \
  };                                                                            \
  DEVICE_DT_INST_DEFINE(n, axis_constrain_init, NULL, &axis_constrain_data_##n, \
                        &axis_constrain_config_##n, POST_KERNEL,                \
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &axis_constrain_api);

DT_INST_FOREACH_STATUS_OKAY(AC_INST)
