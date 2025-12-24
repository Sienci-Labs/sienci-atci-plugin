#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "grbl/hal.h"
#include "driver.h"
#include "grbl/nvs_buffer.h"
#include "grbl/gcode.h"
#include "grbl/system.h"
#include "grbl/motion_control.h"
#include "grbl/settings.h"
#include "grbl/plugins.h"
#include "grbl/task.h"

extern system_t sys;

typedef enum {
    SOURCE_STARTUP,
    SOURCE_RACK,
    SOURCE_COMMAND,
    SOURCE_MACRO
} keepout_source_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t plugin_enabled        :1;
        uint8_t monitor_rack_presence :1;
        uint8_t monitor_tc_macro      :1;
        uint8_t unused                :5;
    } bits;
} config_flags_t;

typedef struct {
    float x_min;
    float y_min;
    float x_max;
    float y_max;
    config_flags_t flags;
} atci_config_t;

/* Runtime-only state (NOT persisted) */
typedef struct {
    float x_min;
    float y_min;
    float x_max;
    float y_max;
    bool enabled;
    keepout_source_t source;
    bool last_pin_state;
} atci_rt_t;

static atci_config_t config;
static atci_rt_t atci;
static nvs_address_t nvs_addr;

static user_mcode_ptrs_t user_mcode = {0};
static on_report_options_ptr on_report_options = NULL;
static on_realtime_report_ptr on_realtime_report = NULL;
static on_report_ngc_parameters_ptr on_report_ngc_parameters = NULL;
static on_tool_selected_ptr prev_on_tool_selected = NULL;
static on_tool_changed_ptr prev_on_tool_changed = NULL;
static bool tc_macro_running = false;

/* Additional sensor states and inside zone marker */
static bool drawbar_state = false;
static bool tool_sensor_state = false;
static bool pressure_sensor_state = false;
static bool inside_keepout_zone = false;

/* Function pointer backups */
typedef bool (*travel_limits_ptr)(float *target, axes_signals_t axes, bool is_cartesian, work_envelope_t *envelope);
typedef void (*apply_travel_limits_ptr)(float *target, float *position, work_envelope_t *envelope);

static travel_limits_ptr prev_check_travel_limits = NULL;
static apply_travel_limits_ptr prev_apply_travel_limits = NULL;

/* Setting IDs - choose a free range if you have other plugins */
#define SETTING_PLUGIN_ENABLE         683
#define SETTING_X_MIN                 684
#define SETTING_Y_MIN                 685
#define SETTING_X_MAX                 686
#define SETTING_Y_MAX                 687

/* --- Helpers --- */

/* Build runtime cached min/max and mirror flags from persistent config */
static void keepout_set(void)
{
    atci.x_min = fminf(config.x_min, config.x_max);
    atci.x_max = fmaxf(config.x_min, config.x_max);
    atci.y_min = fminf(config.y_min, config.y_max);
    atci.y_max = fmaxf(config.y_min, config.y_max);
}

/* is the keepout enforcement currently active?
   requires both the plugin to be persistently enabled and the runtime keepout flag */
static bool is_keepout_active(void)
{
    return (config.flags.bits.plugin_enabled && atci.enabled);
}

static void set_keepout_state(bool new_state, keepout_source_t event_source)
{
    if (atci.enabled != new_state || atci.source != event_source) {
        atci.enabled = new_state;
        atci.source = event_source;
    }
}

/* --- Tool change & rack monitoring --- */
static void keepout_tool_selected(tool_data_t *tool)
{
    if (config.flags.bits.monitor_tc_macro) {
        tc_macro_running = true;
        set_keepout_state(false, SOURCE_MACRO); /* during TC macro disable keepout */
    }
    if (prev_on_tool_selected)
        prev_on_tool_selected(tool);
}

static void keepout_tool_changed(tool_data_t *tool)
{
    if (config.flags.bits.monitor_tc_macro) {
        tc_macro_running = false;
        bool rack_is_installed = !DIGITAL_IN(AUXINPUT7_PORT, AUXINPUT7_PIN);
        set_keepout_state(rack_is_installed, SOURCE_RACK);
    }
    if (prev_on_tool_changed)
        prev_on_tool_changed(tool);
}

/* --- Sensor polling with inside zone tracking --- */
static void poll_rack_sensor(void *data)
{
    if (config.flags.bits.monitor_rack_presence) {
        bool current_pin_is_low = !DIGITAL_IN(AUXINPUT7_PORT, AUXINPUT7_PIN);
        if (current_pin_is_low != atci.last_pin_state) {
            atci.last_pin_state = current_pin_is_low;
            set_keepout_state(current_pin_is_low, SOURCE_RACK);
        }
    }

    /* Additional sensors (optional) */
    drawbar_state         = !DIGITAL_IN(AUXINPUT0_PORT, AUXINPUT0_PIN);
    tool_sensor_state     = !DIGITAL_IN(AUXINPUT1_PORT, AUXINPUT1_PIN);
    pressure_sensor_state = !DIGITAL_IN(AUXINPUT2_PORT, AUXINPUT2_PIN);

    /* Track if we are inside keepout zone (based on planner position) */
    float *pos = plan_get_position();
    if (pos) {
        inside_keepout_zone =
          (pos[X_AXIS] > atci.x_min && pos[X_AXIS] < atci.x_max &&
           pos[Y_AXIS] > atci.y_min && pos[Y_AXIS] < atci.y_max);
    } else {
        inside_keepout_zone = false;
    }

    task_add_delayed(poll_rack_sensor, NULL, 100); /* 100ms polling */
}

/* --- Geometry --- */
static bool line_intersects_keepout(float x0, float y0, float x1, float y1)
{
    float dx = x1 - x0;
    float dy = y1 - y0;
    float t0 = 0.0f, t1 = 1.0f;
    float p[4] = { -dx, dx, -dy, dy };
    float q[4] = {
        x0 - atci.x_min, atci.x_max - x0,
        y0 - atci.y_min, atci.y_max - y0
    };
    for (int i = 0; i < 4; i++) {
        if (p[i] == 0) {
            if (q[i] < 0) return false;
        } else {
            float t = q[i] / p[i];
            if (p[i] < 0) {
                if (t > t1) return false;
                if (t > t0) t0 = t;
            } else {
                if (t < t0) return false;
                if (t < t1) t1 = t;
            }
        }
    }
    /* CHANGED: Strict inequality allows touching (t0==t1) without flagging as intersection */
    return t0 < t1;
}

/* --- Travel & jog protection --- */
static bool travel_limits_check(float *target, axes_signals_t axes, bool is_cartesian, work_envelope_t *envelope)
{
    if (!is_keepout_active())
        return prev_check_travel_limits ? prev_check_travel_limits(target, axes, is_cartesian, envelope) : true;

    float xt = target[X_AXIS];
    float yt = target[Y_AXIS];
    float *pos = plan_get_position();
    float x0 = pos ? pos[X_AXIS] : 0.0f;
    float y0 = pos ? pos[Y_AXIS] : 0.0f;

    /* CHANGED: Use Strict Inequality (> <).
       (10, 20) is NOT strictly inside, so we don't auto-block. */
    bool strictly_inside = (x0 > atci.x_min && x0 < atci.x_max && y0 > atci.y_min && y0 < atci.y_max);

    if (xt >= atci.x_min && xt <= atci.x_max && yt >= atci.y_min && yt <= atci.y_max) {
        if (strictly_inside)
            report_message("ATCI: You are currently inside the keepout zone", Message_Warning);
        else
            report_message("ATCI: Target inside region", Message_Warning);
        return false;
    }

    if (line_intersects_keepout(x0, y0, xt, yt)) {
        if (strictly_inside)
            report_message("ATCI: You are currently inside the keepout zone", Message_Warning);
        else
            report_message("ATCI: Move crosses keepout zone", Message_Warning);
        return false;
    }

    return prev_check_travel_limits ? prev_check_travel_limits(target, axes, is_cartesian, envelope) : true;
}

/* Liang-Barsky style clipping to safe boundary + margin */
static bool calculate_clipped_point(const float *start, const float *end, float *clipped_point)
{
    const float x0 = start[X_AXIS];
    const float y0 = start[Y_AXIS];
    const float x1 = end[X_AXIS];
    const float y1 = end[Y_AXIS];

    const float safe_boundary_xmin = atci.x_min;
    const float safe_boundary_xmax = atci.x_max;
    const float safe_boundary_ymin = atci.y_min;
    const float safe_boundary_ymax = atci.y_max;

    float t0 = 0.0f, t1 = 1.0f;
    const float dx = x1 - x0;
    const float dy = y1 - y0;

    const float p[4] = { -dx, dx, -dy, dy };
    const float q[4] = {
        x0 - safe_boundary_xmin,
        safe_boundary_xmax - x0,
        y0 - safe_boundary_ymin,
        safe_boundary_ymax - y0
    };

    for (int i = 0; i < 4; i++) {
        if (p[i] == 0) {
            if (q[i] < 0) return false; /* Parallel and outside, no intersection */
        } else {
            float t = q[i] / p[i];
            if (p[i] < 0) { /* Line is entering across this edge */
                if (t > t1) return false;
                if (t > t0) t0 = t;
            } else { /* Line is leaving across this edge */
                if (t < t0) return false;
                if (t < t1) t1 = t;
            }
        }
    }

    if (t0 > 0.0f) {
        memcpy(clipped_point, end, sizeof(float) * N_AXIS); /* preserve other axes */
        clipped_point[X_AXIS] = x0 + t0 * dx;
        clipped_point[Y_AXIS] = y0 + t0 * dy;
        return true;
    }

    return false;
}

static void keepout_apply_travel_limits(float *target, float *current_position, work_envelope_t *envelope)
{
    if (!is_keepout_active()) {
        if (prev_apply_travel_limits)
            prev_apply_travel_limits(target, current_position, envelope);
        return;
    }

    float x0 = current_position[X_AXIS];
    float y0 = current_position[Y_AXIS];
    float xt = target[X_AXIS];
    float yt = target[Y_AXIS];

    /* CHANGED: Strict Inequality. */
    bool strictly_inside = (x0 > atci.x_min && x0 < atci.x_max && y0 > atci.y_min && y0 < atci.y_max);

    /* Only block if we are DEEP inside. If on boundary, allow through to intersection check. */
    if (strictly_inside) {
        report_message("ATCI: You are currently inside the keepout zone. Disable keepout before Jogging to safety", Message_Warning);
        memcpy(target, current_position, sizeof(float) * N_AXIS); /* Block move */
        return;
    }

    /* Standard clipping logic proceeds here...
       If on boundary and moving OUT: line_intersects_keepout returns false -> Allowed.
       If on boundary and moving IN: line_intersects_keepout returns true -> Clipped.
    */
    bool in_box = (xt >= atci.x_min && xt <= atci.x_max && yt >= atci.y_min && yt <= atci.y_max);
    bool intersects = line_intersects_keepout(x0, y0, xt, yt);

    if (in_box || intersects) {
        float clipped_target[N_AXIS];
        if (calculate_clipped_point(current_position, target, clipped_target)) {
            report_message("ATCI: Jog move clipped at keepout boundary.", Message_Warning);
            memcpy(target, clipped_target, sizeof(float) * N_AXIS);
        } else {
            report_message("ATCI: Jog move blocked at keepout boundary.", Message_Warning);
            memcpy(target, current_position, sizeof(float) * N_AXIS);
        }
        return;
    }

    if (prev_apply_travel_limits)
        prev_apply_travel_limits(target, current_position, envelope);
}

/* --- M810 handlers --- */
/* M810 toggles runtime-only keepout state (atci.enabled) */

static user_mcode_type_t mcode_check(user_mcode_t mcode)
{
    /* Recognize M810 always so it can be used to toggle runtime state,
       but only advertise as normal if plugin persistent setting is enabled
       to keep UX similar to original — however we still accept it. */
    if (mcode == 810)
        return UserMCode_Normal;
    return user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported;
}

static status_code_t mcode_validate(parser_block_t *gc_block)
{
    status_code_t state = Status_Unhandled;
    if (gc_block->user_mcode == 810) {
        state = Status_OK;
        if (gc_block->words.p) {
            if (gc_block->values.p != 0.0f && gc_block->values.p != 1.0f)
                state = Status_GcodeValueOutOfRange;
            gc_block->words.p = 0; /* consume */
        }
    }
    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block) : state;
}

static void mcode_execute(uint_fast16_t state, parser_block_t *gc_block)
{
    if (gc_block->user_mcode != 810) {
        if (user_mcode.execute)
            user_mcode.execute(state, gc_block);
        return;
    }

    if (state == STATE_CHECK_MODE)
        return;

    /* Toggle runtime-only keepout state */
    if (gc_block->words.p) {
        set_keepout_state(gc_block->values.p == 1.0f, SOURCE_COMMAND);
    } else {
        report_message("Use M810 P1 to enable Sienci ATCi Keepout, M810 P0 to disable.", Message_Info);
    }
}

/* --- Settings registration (persistent only) --- */

static const setting_detail_t plugin_settings[] = {
    /* persistent flags: plugin_enabled, monitor rack, monitor tc macro */
    { SETTING_PLUGIN_ENABLE,         Group_Limits, "ATCi Plugin",  NULL, Format_XBitfield, "Enable,Monitor Rack Presence,Monitor TC Macro", NULL, NULL, Setting_NonCore, &config.flags.value },
    { SETTING_X_MIN,                 Group_Limits, "ATCi Keepout X Min", "mm", Format_Decimal, "-####0.00", "-10000", "10000", Setting_NonCore, &config.x_min },
    { SETTING_Y_MIN,                 Group_Limits, "ATCi Keepout Y Min", "mm", Format_Decimal, "-####0.00", "-10000", "10000", Setting_NonCore, &config.y_min },
    { SETTING_X_MAX,                 Group_Limits, "ATCi Keepout X Max", "mm", Format_Decimal, "-####0.00", "-10000", "10000", Setting_NonCore, &config.x_max },
    { SETTING_Y_MAX,                 Group_Limits, "ATCi Keepout Y Max", "mm", Format_Decimal, "-####0.00", "-10000", "10000", Setting_NonCore, &config.y_max },
};

static void atci_save(void)
{
    keepout_set();
    hal.nvs.memcpy_to_nvs(nvs_addr, (uint8_t *)&config, sizeof(config), true);
}

/* --- Restore & load --- */
static void atci_restore(void)
{
    config.x_min = 10.0f;
    config.y_min = 10.0f;
    config.x_max = 50.0f;
    config.y_max = 50.0f;

    /* defaults: plugin persisted OFF (user must enable), monitor flags OFF */
    config.flags.value = 0;
    config.flags.bits.plugin_enabled = 0;
    config.flags.bits.monitor_rack_presence = 0;
    config.flags.bits.monitor_tc_macro = 0;

    atci.enabled = true;
    atci.source = SOURCE_STARTUP;
    atci.last_pin_state = false;

    hal.nvs.memcpy_to_nvs(nvs_addr, (uint8_t *)&config, sizeof(config), true);
}

static void atci_load(void)
{
    if (hal.nvs.memcpy_from_nvs((uint8_t *)&config, nvs_addr, sizeof(config), true) != NVS_TransferResult_OK)
        atci_restore();

    keepout_set();

    set_keepout_state(true, SOURCE_STARTUP);
    tc_macro_running = false;

    task_add_delayed(poll_rack_sensor, NULL, 1000); /* start polling after 1s */
}

/* --- Report options & NGC params --- */
static void onReportOptions(bool newopt)
{
    if (on_report_options)
        on_report_options(newopt);
    if (!newopt)
        report_plugin("SIENCI ATCi plugin", "0.3.0");
}

static void onReportNgcParameters(void)
{
    char buf[100];

    /* Report keepout bounds as part of $# output */
    snprintf(buf, sizeof(buf),
             "[ATCI:%.2f,%.2f,%.2f,%.2f]" ASCII_EOL,
             atci.x_max,
             atci.x_min,
             atci.y_max,
             atci.y_min);

    hal.stream.write(buf);
    if (on_report_ngc_parameters)
        on_report_ngc_parameters();
}

/* --- Realtime report --- */
static void onRealtimeReport(stream_write_ptr stream_write, report_tracking_flags_t report)
{
    char buf[256];
    char flags[16];
    size_t f = 0;

    switch (atci.source) {
        case SOURCE_RACK:    flags[f++] = 'R'; break;
        case SOURCE_COMMAND: flags[f++] = 'M'; break;
        case SOURCE_MACRO:   flags[f++] = 'T'; break;
        case SOURCE_STARTUP: flags[f++] = 'S'; break;
        default: break;
    }

    /* indicate runtime keepout state (enabled/disabled) */
    if (atci.enabled)
        flags[f++] = 'E';

    if (config.flags.bits.monitor_rack_presence && atci.last_pin_state)
        flags[f++] = 'I';
    if (drawbar_state)
        flags[f++] = 'B';
    if (tool_sensor_state)
        flags[f++] = 'L';
    if (pressure_sensor_state)
        flags[f++] = 'P';
    if (inside_keepout_zone)
        flags[f++] = 'Z';

    flags[f] = '\0';

    snprintf(buf, sizeof(buf),
             "|ATCI:%s",
             flags);

    stream_write(buf);
    if (on_realtime_report)
        on_realtime_report(stream_write, report);
}

/* --- Init --- */
void atci_init(void)
{
    static setting_details_t settings = {
        .settings = plugin_settings,
        .n_settings = sizeof(plugin_settings) / sizeof(setting_detail_t),
        .load = atci_load,
        .save = atci_save,
        .restore = atci_restore
    };

    if ((nvs_addr = nvs_alloc(sizeof(config)))) {
        /* Load config first */
        atci_load();

        /* Hook handlers regardless of persistent plugin_enabled so we can reflect runtime state
           and let the system dynamically enable/disable enforcement at runtime. This mirrors the
           original behavior where the plugin could be present but keepout is managed dynamically. */
        prev_check_travel_limits = grbl.check_travel_limits;
        grbl.check_travel_limits = travel_limits_check;
        prev_apply_travel_limits = grbl.apply_travel_limits;
        grbl.apply_travel_limits = keepout_apply_travel_limits;

        memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));
        grbl.user_mcode.check = mcode_check;
        grbl.user_mcode.validate = mcode_validate;
        grbl.user_mcode.execute = mcode_execute;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        on_realtime_report = grbl.on_realtime_report;
        grbl.on_realtime_report = onRealtimeReport;

        on_report_ngc_parameters = grbl.on_report_ngc_parameters;
        grbl.on_report_ngc_parameters = onReportNgcParameters;

        prev_on_tool_selected = grbl.on_tool_selected;
        grbl.on_tool_selected = keepout_tool_selected;
        prev_on_tool_changed = grbl.on_tool_changed;
        grbl.on_tool_changed = keepout_tool_changed;

        settings_register(&settings);
        task_add_delayed(poll_rack_sensor, NULL, 1000); /* start polling after 1s */

        report_message("Sienci ATCi plugin v0.3.0 initialized", Message_Info);
    }
}
