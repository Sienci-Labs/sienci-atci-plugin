#include <stdio.h>
#include <string.h>
#include <stdbool.h>

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

typedef struct {
    float x_min;
    float y_min;
    float x_max;
    float y_max;
    bool enabled;
    keepout_source_t source;
    bool plugin_enabled;
    bool monitor_rack_presence;
    bool monitor_tc_macro;
    bool last_pin_state;
} keepout_config_t;

static keepout_config_t config;
static nvs_address_t nvs_addr;

static user_mcode_ptrs_t user_mcode = {0};
static on_report_options_ptr on_report_options = NULL;
static on_realtime_report_ptr on_realtime_report = NULL;
static on_tool_selected_ptr prev_on_tool_selected = NULL;
static on_tool_changed_ptr prev_on_tool_changed = NULL;
static bool p_word_present = false;
static bool tc_macro_running = false;

// --- New state variables ---
static bool drawbar_state = false;
static bool tool_sensor_state = false;
static bool pressure_sensor_state = false;
static bool inside_keepout_zone = false;

typedef bool (*travel_limits_ptr)(float *target, axes_signals_t axes, bool is_cartesian, work_envelope_t *envelope);
typedef void (*apply_travel_limits_ptr)(float *target, float *position, work_envelope_t *envelope);

static travel_limits_ptr prev_check_travel_limits = NULL;
static apply_travel_limits_ptr prev_apply_travel_limits = NULL;

// --- Settings IDs ---
#define SETTING_PLUGIN_ENABLE         Setting_UserDefined_0
#define SETTING_MONITOR_RACK_PRESENCE Setting_UserDefined_1
#define SETTING_MONITOR_TC_MACRO      Setting_UserDefined_2
#define SETTING_X_MIN                 Setting_UserDefined_3
#define SETTING_Y_MIN                 Setting_UserDefined_4
#define SETTING_X_MAX                 Setting_UserDefined_5
#define SETTING_Y_MAX                 Setting_UserDefined_6

// --- Helpers ---
static inline float keepout_xmin(void) { return config.x_min < config.x_max ? config.x_min : config.x_max; }
static inline float keepout_xmax(void) { return config.x_min < config.x_max ? config.x_max : config.x_min; }
static inline float keepout_ymin(void) { return config.y_min < config.y_max ? config.y_min : config.y_max; }
static inline float keepout_ymax(void) { return config.y_min < config.y_max ? config.y_max : config.y_min; }

static void set_keepout_state(bool new_state, keepout_source_t event_source)
{
    if (config.enabled != new_state || config.source != event_source) {
        config.enabled = new_state;
        config.source = event_source;
    }
}

// --- Tool change monitoring ---
static void keepout_tool_selected(tool_data_t *tool)
{
    if (config.plugin_enabled && config.monitor_tc_macro) {
        tc_macro_running = true;
        set_keepout_state(false, SOURCE_MACRO);
    }
    if (prev_on_tool_selected)
        prev_on_tool_selected(tool);
}

static void keepout_tool_changed(tool_data_t *tool)
{
    if (config.plugin_enabled && config.monitor_tc_macro) {
        tc_macro_running = false;
        bool rack_is_installed = !DIGITAL_IN(AUXINPUT7_PORT, AUXINPUT7_PIN);
        set_keepout_state(rack_is_installed, SOURCE_RACK);
    }
    if (prev_on_tool_changed)
        prev_on_tool_changed(tool);
}

// --- Sensor polling with inside zone tracking ---
static void poll_rack_sensor(void *data)
{
    if (config.plugin_enabled && config.monitor_rack_presence) {
        bool current_pin_is_low = !DIGITAL_IN(AUXINPUT7_PORT, AUXINPUT7_PIN);
        if (current_pin_is_low != config.last_pin_state) {
            config.last_pin_state = current_pin_is_low;
            set_keepout_state(current_pin_is_low, SOURCE_RACK);
        }
    }

    // Additional sensors
    drawbar_state         = !DIGITAL_IN(AUXINPUT0_PORT, AUXINPUT0_PIN);
    tool_sensor_state     = !DIGITAL_IN(AUXINPUT1_PORT, AUXINPUT1_PIN);
    pressure_sensor_state = !DIGITAL_IN(AUXINPUT2_PORT, AUXINPUT2_PIN);

    // Track if we are inside keepout zone (based on machine position)
    float pos[N_AXIS];
    system_convert_array_steps_to_mpos(pos, sys.position);
    inside_keepout_zone =
        (pos[X_AXIS] >= keepout_xmin() && pos[X_AXIS] <= keepout_xmax() &&
         pos[Y_AXIS] >= keepout_ymin() && pos[Y_AXIS] <= keepout_ymax());

    task_add_delayed(poll_rack_sensor, NULL, 100); // 100ms polling
}

// --- Geometry ---
static bool line_intersects_keepout(float x0, float y0, float x1, float y1)
{
    float dx = x1 - x0;
    float dy = y1 - y0;
    float t0 = 0.0f, t1 = 1.0f;
    float p[4] = { -dx, dx, -dy, dy };
    float q[4] = {
        x0 - keepout_xmin(), keepout_xmax() - x0,
        y0 - keepout_ymin(), keepout_ymax() - y0
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
    return true;
}

static bool is_keepout_active(void)
{
    return config.plugin_enabled && config.enabled;
}

// --- Travel & jog protection ---
static bool travel_limits_check(float *target, axes_signals_t axes, bool is_cartesian, work_envelope_t *envelope)
{
    if (!is_keepout_active())
        return prev_check_travel_limits ? prev_check_travel_limits(target, axes, is_cartesian, envelope) : true;

    float xt = target[X_AXIS];
    float yt = target[Y_AXIS];
    float pos[N_AXIS];
    system_convert_array_steps_to_mpos(pos, sys.position);
    float x0 = pos[X_AXIS];
    float y0 = pos[Y_AXIS];

    if (xt >= keepout_xmin() && xt <= keepout_xmax() && yt >= keepout_ymin() && yt <= keepout_ymax()) {
        if (inside_keepout_zone)
            report_message("ATCI: You are currently inside the keepout zone", Message_Warning);
        else
            report_message("ATCI: Target inside region", Message_Warning);
        return false;
    }

    if (line_intersects_keepout(x0, y0, xt, yt)) {
        if (inside_keepout_zone)
            report_message("ATCI: You are currently inside the keepout zone", Message_Warning);
        else
            report_message("ATCI: Move crosses keepout zone", Message_Warning);
        return false;
    }

    return prev_check_travel_limits ? prev_check_travel_limits(target, axes, is_cartesian, envelope) : true;
}

static void keepout_apply_travel_limits(float *target, float *current_position, work_envelope_t *envelope)
{
    if (!is_keepout_active()) {
        if (prev_apply_travel_limits)
            prev_apply_travel_limits(target, current_position, envelope);
        return;
    }

    float xt = target[X_AXIS];
    float yt = target[Y_AXIS];
    float x0 = current_position[X_AXIS];
    float y0 = current_position[Y_AXIS];

    bool in_box = (xt >= keepout_xmin() && xt <= keepout_xmax() && yt >= keepout_ymin() && yt <= keepout_ymax());
    bool intersects = line_intersects_keepout(x0, y0, xt, yt);

    if (in_box || intersects) {
        if (inside_keepout_zone)
            report_message("ATCI: You are currently inside the keepout zone. Disable keepout before Jogging to safety", Message_Warning);
        else
            report_message("ATCI: Jog move blocked", Message_Warning);
        memcpy(target, current_position, sizeof(float) * N_AXIS);
        return;
    }

    if (prev_apply_travel_limits)
        prev_apply_travel_limits(target, current_position, envelope);
}

// --- M810 handlers ---
static user_mcode_type_t mcode_check(user_mcode_t mcode)
{
    if (config.plugin_enabled && mcode == 810)
        return UserMCode_Normal;
    return user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported;
}

static status_code_t mcode_validate(parser_block_t *gc_block)
{
    status_code_t state = Status_Unhandled;
    if (gc_block->user_mcode == 810) {
        state = Status_OK;
        p_word_present = false;
        if (gc_block->words.p) {
            p_word_present = true;
            if (gc_block->values.p != 0.0f && gc_block->values.p != 1.0f)
                state = Status_GcodeValueOutOfRange;
            gc_block->words.p = 0;
        }
        if (gc_block->words.value)
            state = Status_GcodeUnusedWords;
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

    if (!config.plugin_enabled || state == STATE_CHECK_MODE)
        return;

    if (p_word_present)
        set_keepout_state(gc_block->values.p == 1.0f, SOURCE_COMMAND);
    else
        report_message("Use M810 P1 to enable Sienci ATCi Keepout, M810 P0 to disable.", Message_Info);
}

// --- Settings & persistence ---
static status_code_t set_bool_setting(setting_id_t id, uint_fast16_t value) {
    bool val = (value != 0);
    switch(id) {
        case SETTING_PLUGIN_ENABLE:         config.plugin_enabled = val; break;
        case SETTING_MONITOR_RACK_PRESENCE: config.monitor_rack_presence = val; break;
        case SETTING_MONITOR_TC_MACRO:      config.monitor_tc_macro = val; break;
        default: return Status_Unhandled;
    }
    hal.nvs.memcpy_to_nvs(nvs_addr, (uint8_t *)&config, sizeof(config), true);
    return Status_OK;
}

static uint_fast16_t get_bool_setting(setting_id_t id) {
    switch(id) {
        case SETTING_PLUGIN_ENABLE:         return config.plugin_enabled;
        case SETTING_MONITOR_RACK_PRESENCE: return config.monitor_rack_presence;
        case SETTING_MONITOR_TC_MACRO:      return config.monitor_tc_macro;
        default: return 0;
    }
}

static status_code_t set_float_setting(setting_id_t id, float value) {
    switch(id) {
        case SETTING_X_MIN: config.x_min = value; break;
        case SETTING_Y_MIN: config.y_min = value; break;
        case SETTING_X_MAX: config.x_max = value; break;
        case SETTING_Y_MAX: config.y_max = value; break;
        default: return Status_Unhandled;
    }
    hal.nvs.memcpy_to_nvs(nvs_addr, (uint8_t *)&config, sizeof(config), true);
    return Status_OK;
}

static float get_float_setting(setting_id_t id) {
    switch(id) {
        case SETTING_X_MIN: return config.x_min;
        case SETTING_Y_MIN: return config.y_min;
        case SETTING_X_MAX: return config.x_max;
        case SETTING_Y_MAX: return config.y_max;
        default: return 0.0f;
    }
}

static const setting_detail_t plugin_settings[] = {
    { SETTING_PLUGIN_ENABLE,         Group_UserSettings, "ATCi Plugin Enabled",        NULL, Format_Bool,    NULL, NULL, NULL, Setting_IsLegacyFn, (void *)set_bool_setting, (void *)get_bool_setting },
    { SETTING_MONITOR_RACK_PRESENCE, Group_UserSettings, "ATCi Monitor Rack Presence", NULL, Format_Bool,    NULL, NULL, NULL, Setting_IsLegacyFn, (void *)set_bool_setting, (void *)get_bool_setting },
    { SETTING_MONITOR_TC_MACRO,      Group_UserSettings, "ATCi Monitor TC Macro",      NULL, Format_Bool,    NULL, NULL, NULL, Setting_IsLegacyFn, (void *)set_bool_setting, (void *)get_bool_setting },
    { SETTING_X_MIN,                 Group_UserSettings, "ATCi Keepout X Min", "mm", Format_Decimal, "-####0.00", "-10000", "10000", Setting_IsLegacyFn, set_float_setting, get_float_setting },
    { SETTING_Y_MIN,                 Group_UserSettings, "ATCi Keepout Y Min", "mm", Format_Decimal, "-####0.00", "-10000", "10000", Setting_IsLegacyFn, set_float_setting, get_float_setting },
    { SETTING_X_MAX,                 Group_UserSettings, "ATCi Keepout X Max", "mm", Format_Decimal, "-####0.00", "-10000", "10000", Setting_IsLegacyFn, set_float_setting, get_float_setting },
    { SETTING_Y_MAX,                 Group_UserSettings, "ATCi Keepout Y Max", "mm", Format_Decimal, "-####0.00", "-10000", "10000", Setting_IsLegacyFn, set_float_setting, get_float_setting },
};

// --- Restore & load ---
static void keepout_restore(void)
{
    config.x_min = 10.0f;
    config.y_min = 10.0f;
    config.x_max = 50.0f;
    config.y_max = 50.0f;
    config.enabled  = true;
    config.plugin_enabled = false;
    config.monitor_rack_presence = false;
    config.monitor_tc_macro = false;
    config.last_pin_state = false;
    hal.nvs.memcpy_to_nvs(nvs_addr, (uint8_t *)&config, sizeof(config), true);
}

static void keepout_load(void)
{
    if (hal.nvs.memcpy_from_nvs((uint8_t *)&config, nvs_addr, sizeof(config), true) != NVS_TransferResult_OK)
        keepout_restore();
}

// --- Report options ---
static void onReportOptions(bool newopt)
{
    if (on_report_options)
        on_report_options(newopt);
    if (!newopt)
        report_plugin("Sienci ATCi plugin", "0.3.0");
}

// --- Realtime report ---
static void onRealtimeReport (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    char buf[256];
    char flags[16];
    size_t f = 0;

    if (config.enabled)
        flags[f++] = 'E';

    switch (config.source) {
        case SOURCE_RACK:    flags[f++] = 'R'; break;
        case SOURCE_COMMAND: flags[f++] = 'M'; break;
        case SOURCE_MACRO:   flags[f++] = 'T'; break;
        case SOURCE_STARTUP: flags[f++] = 'S'; break;
        default: break;
    }

    if (config.monitor_rack_presence && config.last_pin_state)
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
             "|ATCI:%.2f,%.2f,%.2f,%.2f,%s",
             keepout_xmax(),
             keepout_xmin(),
             keepout_ymax(),
             keepout_ymin(),
             flags);

    stream_write(buf);
    if (on_realtime_report)
        on_realtime_report(stream_write, report);
}

// --- Init ---
void keepout_init(void)
{
    static setting_details_t settings = {
        .settings = plugin_settings,
        .n_settings = sizeof(plugin_settings) / sizeof(setting_detail_t),
        .load = keepout_load,
        .restore = keepout_restore
    };

    if ((nvs_addr = nvs_alloc(sizeof(config)))) {
        keepout_load();

        tc_macro_running = false;
        if (config.monitor_rack_presence) {
            config.last_pin_state = !DIGITAL_IN(AUXINPUT7_PORT, AUXINPUT7_PIN);
            set_keepout_state(config.last_pin_state, SOURCE_RACK);
        } else {
            set_keepout_state(true, SOURCE_STARTUP);
        }

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

        prev_on_tool_selected = grbl.on_tool_selected;
        grbl.on_tool_selected = keepout_tool_selected;
        prev_on_tool_changed = grbl.on_tool_changed;
        grbl.on_tool_changed = keepout_tool_changed;

        settings_register(&settings);
        task_add_delayed(poll_rack_sensor, NULL, 1000); // start polling after 1s

        report_message("Sienci ATCi plugin v0.3.0 initialized", Message_Info);
    }
}
