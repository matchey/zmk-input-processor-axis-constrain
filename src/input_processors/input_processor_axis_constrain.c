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
  int axis_lock_threshold;
  int release_after_ms;
  int axis_tolerance_deg;
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

/*
 * Lookup table for tan(angle) * 1000, where angle is 0-45 degrees.
 * Used to avoid floating-point operations in the kernel.
 * tan(0°)=0, tan(15°)≈0.268, tan(30°)≈0.577, tan(45°)=1.0
 */
static const int32_t tan_table_x1000[46] = {
    0,   17,  35,  52,  70,  87,  105, 123, 141, 158, 176, 194, 213, 231,  249, 268,
    287, 306, 325, 344, 364, 384, 404, 424, 445, 466, 488, 510, 532, 554,  577, 601,
    625, 649, 675, 700, 727, 754, 781, 810, 839, 869, 900, 933, 966, 1000,
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

/*
 * Check if the movement vector is within ±tolerance_deg from the given axis.
 * For X-axis: angle from horizontal must be <= tolerance_deg
 * For Y-axis: angle from vertical must be <= tolerance_deg
 *
 * Uses the formula: |minor| / |major| <= tan(tolerance_deg)
 * Rearranged to avoid division: |minor| * 1000 <= |major| * tan_table_x1000[tolerance_deg]
 */
static bool is_within_axis_tolerance(int32_t abs_x, int32_t abs_y, enum axis_state axis,
                                     int tolerance_deg) {
  if (tolerance_deg >= 45) {
    return true; /* Accept all directions */
  }

  int32_t tan_threshold = tan_table_x1000[tolerance_deg];
  int64_t minor_scaled;
  int64_t major_scaled;

  if (axis == AXIS_X) {
    /* For X-axis dominance, check if |y|/|x| <= tan(tolerance) */
    minor_scaled = (int64_t)abs_y * 1000;
    major_scaled = (int64_t)abs_x * tan_threshold;
  } else {
    /* For Y-axis dominance, check if |x|/|y| <= tan(tolerance) */
    minor_scaled = (int64_t)abs_x * 1000;
    major_scaled = (int64_t)abs_y * tan_threshold;
  }

  return minor_scaled <= major_scaled;
}

static enum axis_state determine_dominant_axis(struct axis_constrain_data         *data,
                                               const struct axis_constrain_config *config) {
  int     axis_lock_threshold = config->axis_lock_threshold;
  int     tolerance_deg       = config->axis_tolerance_deg;
  int32_t abs_x               = data->abs_accum_x;
  int32_t abs_y               = data->abs_accum_y;

  enum axis_state candidate = AXIS_NONE;

  if (abs_x >= axis_lock_threshold && abs_x > abs_y) {
    candidate = AXIS_X;
  } else if (abs_y >= axis_lock_threshold && abs_y > abs_x) {
    candidate = AXIS_Y;
  } else if (abs_x >= axis_lock_threshold && abs_x == abs_y) {
    /* Prefer X when equal for deterministic behavior */
    candidate = AXIS_X;
  }

  if (candidate == AXIS_NONE) {
    return AXIS_NONE;
  }

  /* Check if within angular tolerance */
  if (!is_within_axis_tolerance(abs_x, abs_y, candidate, tolerance_deg)) {
    LOG_DBG("Outside tolerance: candidate=%s, abs_x=%d, abs_y=%d, tolerance=%d°",
            candidate == AXIS_X ? "X" : "Y", abs_x, abs_y, tolerance_deg);
    return AXIS_NONE;
  }

  return candidate;
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
    data->locked_axis = determine_dominant_axis(data, config);

    if (data->locked_axis != AXIS_NONE) {
      LOG_DBG("Locked to %s axis (abs_accum_x=%d, abs_accum_y=%d)", axis_name(data->locked_axis),
              data->abs_accum_x, data->abs_accum_y);
    }
  }

  if (data->locked_axis == AXIS_NONE) {
    LOG_DBG(
        "Below threshold or outside tolerance, suppressed %s: %d (abs_accum_x=%d, abs_accum_y=%d)",
        is_x ? "X" : "Y", event->value, data->abs_accum_x, data->abs_accum_y);
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
  enum axis_state dominant = determine_dominant_axis(data, config);

  if (dominant == AXIS_NONE) {
    LOG_DBG(
        "Below threshold or outside tolerance, suppressed %s: %d (abs_accum_x=%d, abs_accum_y=%d)",
        is_x ? "X" : "Y", event->value, data->abs_accum_x, data->abs_accum_y);
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
      if (data->abs_accum_x > config->axis_lock_threshold) {
        data->accum_x =
            (data->accum_x > 0) ? config->axis_lock_threshold : -config->axis_lock_threshold;
        data->abs_accum_x = config->axis_lock_threshold;
      }
    } else {
      data->accum_x     = 0;
      data->abs_accum_x = 0;
      if (data->abs_accum_y > config->axis_lock_threshold) {
        data->accum_y =
            (data->accum_y > 0) ? config->axis_lock_threshold : -config->axis_lock_threshold;
        data->abs_accum_y = config->axis_lock_threshold;
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

  const bool sticky_mode = (config->release_after_ms > 0);
  if (sticky_mode) {
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

  LOG_DBG("Initialized (axis_lock_threshold=%d, release_after_ms=%d, axis_tolerance_deg=%d)",
          config->axis_lock_threshold, config->release_after_ms, config->axis_tolerance_deg);

  return 0;
}

#define AC_INST(n)                                                              \
  BUILD_ASSERT(DT_INST_PROP(n, axis_lock_threshold) > 0,                        \
               "axis_lock_threshold must be greater than 0");                   \
  BUILD_ASSERT(0 < DT_INST_PROP(n, axis_tolerance_deg) &&                       \
                   DT_INST_PROP(n, axis_tolerance_deg) <= 45,                   \
               "axis-tolerance-deg must be in the range (0, 45]");              \
                                                                                \
  static struct axis_constrain_data axis_constrain_data_##n = {                 \
      .lock = {},                                                               \
  };                                                                            \
                                                                                \
  static const struct axis_constrain_config axis_constrain_config_##n = {       \
      .axis_lock_threshold = DT_INST_PROP(n, axis_lock_threshold),              \
      .release_after_ms    = DT_INST_PROP(n, release_after_ms),                 \
      .axis_tolerance_deg  = DT_INST_PROP(n, axis_tolerance_deg),               \
  };                                                                            \
                                                                                \
  DEVICE_DT_INST_DEFINE(n, axis_constrain_init, NULL, &axis_constrain_data_##n, \
                        &axis_constrain_config_##n, POST_KERNEL,                \
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &axis_constrain_api);

DT_INST_FOREACH_STATUS_OKAY(AC_INST)
