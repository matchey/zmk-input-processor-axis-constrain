/*
 * Copyright (c) 2026 matchey
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define DT_DRV_COMPAT zmk_input_processor_axis_constrain

#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/input_processor.h>

LOG_MODULE_REGISTER(axis_constrain, CONFIG_ZMK_LOG_LEVEL);

/* Prevent overflow during addition */
#define MAX_ACCUM (INT32_MAX / 2)

enum axis_state {
  AXIS_NONE = 0,
  AXIS_X,
  AXIS_Y,
};

struct axis_constrain_config {
  int  threshold;
  bool sticky;
  int  release_after_ms;
};

struct axis_constrain_data {
  const struct device    *dev;
  enum axis_state         locked_axis;
  int32_t                 accum_x;
  int32_t                 accum_y;
  int32_t                 abs_accum_x;
  int32_t                 abs_accum_y;
  struct k_spinlock       lock;
  struct k_work_delayable release_work;
};

static inline void reset_state_locked(struct axis_constrain_data *data) {
  data->locked_axis = AXIS_NONE;
  data->accum_x     = 0;
  data->accum_y     = 0;
  data->abs_accum_x = 0;
  data->abs_accum_y = 0;
}

/* abs(INT32_MIN) is undefined behavior */
static inline int32_t safe_abs(int32_t value) {
  if (value == INT32_MIN) {
    return INT32_MAX;
  }
  return abs(value);
}

static inline int32_t safe_accum_add(int32_t current, int32_t delta) {
  int64_t result = (int64_t)current + (int64_t)delta;
  if (result > MAX_ACCUM) {
    return MAX_ACCUM;
  }
  if (result < -MAX_ACCUM) {
    return -MAX_ACCUM;
  }
  return (int32_t)result;
}

static inline void update_accum(struct axis_constrain_data *data, bool is_x, int32_t delta) {
  if (is_x) {
    data->accum_x     = safe_accum_add(data->accum_x, delta);
    data->abs_accum_x = safe_abs(data->accum_x);
  } else {
    data->accum_y     = safe_accum_add(data->accum_y, delta);
    data->abs_accum_y = safe_abs(data->accum_y);
  }
}

static enum axis_state determine_dominant_axis(struct axis_constrain_data *data, int threshold) {
  if (data->abs_accum_x >= threshold && data->abs_accum_x > data->abs_accum_y) {
    return AXIS_X;
  }
  if (data->abs_accum_y >= threshold && data->abs_accum_y > data->abs_accum_x) {
    return AXIS_Y;
  }
  /* Prefer X when equal for deterministic behavior */
  if (data->abs_accum_x >= threshold && data->abs_accum_x == data->abs_accum_y) {
    return AXIS_X;
  }
  return AXIS_NONE;
}

static void release_work_handler(struct k_work *work) {
  struct k_work_delayable    *dwork = k_work_delayable_from_work(work);
  struct axis_constrain_data *data  = CONTAINER_OF(dwork, struct axis_constrain_data, release_work);

  k_spinlock_key_t key = k_spin_lock(&data->lock);

  LOG_DBG("Releasing axis lock (was: %s)",
          data->locked_axis == AXIS_X ? "X" : (data->locked_axis == AXIS_Y ? "Y" : "NONE"));

  reset_state_locked(data);

  k_spin_unlock(&data->lock, key);
}

#if defined(CONFIG_LOG)
static inline const char *axis_name(enum axis_state axis) {
  switch (axis) {
    case AXIS_X:
      return "X";
    case AXIS_Y:
      return "Y";
    default:
      return "NONE";
  }
}
#endif

static void handle_sticky_mode(struct axis_constrain_data         *data,
                               const struct axis_constrain_config *config,
                               struct input_event *event, bool is_x) {
  if (data->locked_axis == AXIS_NONE) {
    data->locked_axis = determine_dominant_axis(data, config->threshold);

    if (data->locked_axis != AXIS_NONE) {
      LOG_DBG("Locked to %s axis (abs_accum_x=%d, abs_accum_y=%d)", axis_name(data->locked_axis),
              data->abs_accum_x, data->abs_accum_y);
    }
  }

  if (data->locked_axis == AXIS_NONE) {
    LOG_DBG("Below threshold, suppressed %s: %d (abs_accum_x=%d, abs_accum_y=%d)", is_x ? "X" : "Y",
            event->value, data->abs_accum_x, data->abs_accum_y);
    event->value = 0;
    return;
  }

  bool is_locked_axis =
      (data->locked_axis == AXIS_X && is_x) || (data->locked_axis == AXIS_Y && !is_x);

  if (!is_locked_axis) {
    LOG_DBG("Suppressed %s: %d (locked: %s)", is_x ? "X" : "Y", event->value,
            axis_name(data->locked_axis));
    event->value = 0;
  }
}

static void handle_non_sticky_mode(struct axis_constrain_data         *data,
                                   const struct axis_constrain_config *config,
                                   struct input_event *event, bool is_x) {
  enum axis_state dominant = determine_dominant_axis(data, config->threshold);

  if (dominant == AXIS_NONE) {
    LOG_DBG("Below threshold, suppressed %s: %d (abs_accum_x=%d, abs_accum_y=%d)", is_x ? "X" : "Y",
            event->value, data->abs_accum_x, data->abs_accum_y);
    event->value = 0;
    return;
  }

  bool is_dominant = (dominant == AXIS_X && is_x) || (dominant == AXIS_Y && !is_x);

  if (!is_dominant) {
    LOG_DBG("Suppressed %s: %d (dominant: %s)", is_x ? "X" : "Y", event->value,
            axis_name(dominant));
    event->value = 0;
  } else {
    /*
     * Reset suppressed axis accumulator to allow quick direction switching.
     * Clamp dominant axis to threshold to prevent indefinite growth, making it
     * easier to switch directions after sustained movement.
     */
    if (dominant == AXIS_X) {
      data->accum_y     = 0;
      data->abs_accum_y = 0;
      if (data->abs_accum_x > config->threshold) {
        data->accum_x     = (data->accum_x > 0) ? config->threshold : -config->threshold;
        data->abs_accum_x = config->threshold;
      }
    } else {
      data->accum_x     = 0;
      data->abs_accum_x = 0;
      if (data->abs_accum_y > config->threshold) {
        data->accum_y     = (data->accum_y > 0) ? config->threshold : -config->threshold;
        data->abs_accum_y = config->threshold;
      }
    }
  }
}

static int axis_constrain_handle_event(const struct device *dev, struct input_event *event,
                                       uint32_t param1, uint32_t param2,
                                       struct zmk_input_processor_state *state) {
  const struct axis_constrain_config *config = dev->config;
  struct axis_constrain_data         *data   = dev->data;

  if (event->type != INPUT_EV_REL) {
    return 0;
  }

  if (event->code != INPUT_REL_X && event->code != INPUT_REL_Y) {
    return 0;
  }

  bool is_x = (event->code == INPUT_REL_X);

  k_spinlock_key_t key = k_spin_lock(&data->lock);

  update_accum(data, is_x, event->value);

  if (config->sticky) {
    k_work_reschedule(&data->release_work, K_MSEC(config->release_after_ms));
    handle_sticky_mode(data, config, event, is_x);
  } else {
    handle_non_sticky_mode(data, config, event, is_x);
  }

  k_spin_unlock(&data->lock, key);

  return 0;
}

static struct zmk_input_processor_driver_api axis_constrain_api = {
    .handle_event = axis_constrain_handle_event,
};

static int axis_constrain_init(const struct device *dev) {
  struct axis_constrain_data         *data   = dev->data;
  const struct axis_constrain_config *config = dev->config;

  data->dev = dev;
  reset_state_locked(data);
  k_work_init_delayable(&data->release_work, release_work_handler);

  LOG_DBG("Initialized (threshold=%d, sticky=%s, release_after_ms=%d)", config->threshold,
          config->sticky ? "true" : "false", config->release_after_ms);

  return 0;
}

#define AC_INST(n)                                                                  \
  BUILD_ASSERT(DT_INST_PROP(n, threshold) > 0, "threshold must be greater than 0"); \
  BUILD_ASSERT(!DT_INST_PROP(n, sticky) || DT_INST_PROP(n, release_after_ms) > 0,   \
               "release_after_ms must be > 0 when sticky mode is enabled");         \
                                                                                    \
  static struct axis_constrain_data axis_constrain_data_##n = {                     \
      .lock = {},                                                                   \
  };                                                                                \
                                                                                    \
  static const struct axis_constrain_config axis_constrain_config_##n = {           \
      .threshold        = DT_INST_PROP(n, threshold),                               \
      .sticky           = DT_INST_PROP(n, sticky),                                  \
      .release_after_ms = DT_INST_PROP(n, release_after_ms),                        \
  };                                                                                \
                                                                                    \
  DEVICE_DT_INST_DEFINE(n, axis_constrain_init, NULL, &axis_constrain_data_##n,     \
                        &axis_constrain_config_##n, POST_KERNEL,                    \
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &axis_constrain_api);

DT_INST_FOREACH_STATUS_OKAY(AC_INST)
