#include <stdlib.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT zmk_axis_clamper
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct zip_axis_clamper_config {
    const uint32_t history_size; // in discrete sensor events
    const uint32_t threshold, hysteresis; // in percents
};

struct zip_axis_clamper_data {
    const struct device *dev;
    bool initialized;
    uint32_t* history[2];
    uint16_t x_index;
    uint16_t y_index;
    uint16_t x_captured;
    uint16_t y_captured;
    bool clamped_to_x;
    bool clamped_to_y;
    struct k_work_delayable history_ttl_work;
    int64_t last_event_timestamp;
};

static void data_init(const struct device *dev);

static void clear_history(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct zip_axis_clamper_data *data = CONTAINER_OF(dwork, struct zip_axis_clamper_data, history_ttl_work);
    const struct zip_axis_clamper_config *config = data->dev->config;

    memset(data->history[0], 0, config->history_size * sizeof(uint32_t));
    memset(data->history[1], 0, config->history_size * sizeof(uint32_t));

    data->x_index = 0;
    data->x_captured = 0;
    data->y_index = 0;
    data->y_captured = 0;
    data->clamped_to_x = false;
    data->clamped_to_y = false;
}

// ReSharper disable once CppParameterMayBeConstPtrOrRef
static int sy_handle_event(const struct device *dev, struct input_event *event, const uint32_t p1,
                           const uint32_t p2, struct zmk_input_processor_state *s) {
    struct zip_axis_clamper_data *data = dev->data;
    const struct zip_axis_clamper_config *config = dev->config;

    if (!data->initialized) {
        data_init(dev);
    }

    k_work_reschedule(&data->history_ttl_work, K_MSEC(CONFIG_ZMK_AXIS_CLAMPER_HISTORY_TTL));
    data->last_event_timestamp = k_uptime_get();

    if (event->code == INPUT_REL_X) {
        data->history[0][data->x_index] = abs(event->value);
        data->x_index = (data->x_index + 1) % config->history_size;
        data->x_captured = CLAMP(data->x_captured + 1, 0, config->history_size);
    } else if (event->code == INPUT_REL_Y) {
        data->history[1][data->y_index] = abs(event->value);
        data->y_index = (data->y_index + 1) % config->history_size;
        data->y_captured = CLAMP(data->y_captured + 1, 0, config->history_size);
    } else {
        return 0;
    }

    uint32_t x_sum = 0;
    uint32_t y_sum = 0;
    for (uint16_t i = 0; i < config->history_size; i++) {
        x_sum += data->history[0][i];
        y_sum += data->history[1][i];
    }

    if (data->x_captured < config->history_size / 2 || data->y_captured < config->history_size / 2) {
        return 0;
    }

    const uint32_t x_avg = x_sum / data->x_captured;
    const uint32_t y_avg = y_sum / data->y_captured;
    const uint32_t total = x_avg + y_avg;
    if (total == 0) {
        return 0;
    }

    const uint32_t x_percent = (x_avg * 100) / total;
    const uint32_t y_percent = (y_avg * 100) / total;
    if (!data->clamped_to_x && !data->clamped_to_y) {
        if (x_percent >= config->threshold) {
            data->clamped_to_x = true;
        } else if (y_percent >= config->threshold) {
            data->clamped_to_y = true;
        }
    } else if (data->clamped_to_x) {
        if (x_percent < (config->threshold - config->hysteresis)) {
            data->clamped_to_x = false;
        }
    } else if (data->clamped_to_y) {
        if (y_percent < config->threshold - config->hysteresis) {
            data->clamped_to_y = false;
        }
    }

    if ((data->clamped_to_x && event->code == INPUT_REL_Y) ||
        (data->clamped_to_y && event->code == INPUT_REL_X)) {
        event->value = 0;
        event->sync = false;
    }

    return 0;
}

static void data_init(const struct device *dev) {
    struct zip_axis_clamper_data *data = dev->data;
    const struct zip_axis_clamper_config *config = dev->config;

    data->history[0] = malloc(config->history_size * sizeof(uint32_t));
    data->history[1] = malloc(config->history_size * sizeof(uint32_t));

    if (data->history[0] == NULL || data->history[1] == NULL) {
        LOG_ERR("Failed to allocate memory for axis history");
        return;
    }

    memset(data->history[0], 0, config->history_size * sizeof(uint32_t));
    memset(data->history[1], 0, config->history_size * sizeof(uint32_t));

    data->x_index = 0;
    data->y_index = 0;
    data->clamped_to_x = false;
    data->clamped_to_y = false;
    data->last_event_timestamp = 0;
    data->initialized = true;

    k_work_init_delayable(&data->history_ttl_work, clear_history);
}

static int sy_init(const struct device *dev) {
    struct zip_axis_clamper_data *data = dev->data;
    data->dev = dev;
    return 0;
}

static struct zmk_input_processor_driver_api sy_driver_api = {
    .handle_event = sy_handle_event,
};

static struct zip_axis_clamper_data data = {
    .history = { 0 },
};

static const struct zip_axis_clamper_config config = {
    .history_size = DT_INST_PROP_OR(0, history_size, 24),
    .hysteresis = DT_INST_PROP_OR(0, hysteresis, 5),
    .threshold = DT_INST_PROP_OR(0, threshold, 35),
};

DEVICE_DT_INST_DEFINE(0, &sy_init, NULL, &data, &config, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &sy_driver_api);
