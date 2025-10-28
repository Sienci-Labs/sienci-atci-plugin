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

typedef union {
    uint8_t value;
    struct {
        uint8_t enabled :1,
        monitor_rack_presence :1,
        monitor_tc_macro :1,
        unused :4;
    };
} config_flags_t;

typedef struct {
    float x_min;
    float y_min;
    float x_max;
    float y_max;
    config_flags_t flags;
} keepout_config_t;

typedef struct {
    float x_min;
    float y_min;
    float x_max;
    float y_max;
    config_flags_t flags;
    keepout_source_t source;
    bool last_pin_state;
} keepout_rt_t;

keepout_rt_t keepout;

static keepout_config_t config;
static nvs_address_t nvs_addr;

static user_mcode_ptrs_t user_mcode = {0};
static on_report_options_ptr on_report_options = NULL;
static on_realtime_report_ptr on_realtime_report = NULL;
static on_report_ngc_parameters_ptr on_report_ngc_parameters;
static on_tool_selected_ptr prev_on_tool_selected = NULL;
static on_tool_changed_ptr prev_on_tool_changed = NULL;
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
#define SETTING_PLUGIN_ENABLE         683
#define SETTING_X_MIN                 684
#define SETTING_Y_MIN                 685
#define SETTING_X_MAX                 686
#define SETTING_Y_MAX                 687

static void keepout_set (void)
{
    keepout.x_min = min(config.x_min, config.x_max);
    keepout.x_max = max(config.x_min, config.x_max);
    keepout.y_min = min(config.y_min, config.y_max);
    keepout.y_max = max(config.y_min, config.y_max);
    keepout.flags.value = config.flags.value;
}

static void set_keepout_state(bool new_state, keepout_source_t event_source)
{
    if (keepout.flags.enabled != new_state || keepout.source != event_source) {
        keepout.flags.enabled = new_state;
        keepout.source = event_source;
    }
}

// --- Tool change monitoring ---
static void keepout_tool_selected(tool_data_t *tool)
{
    if (keepout.flags.enabled && keepout.flags.monitor_tc_macro) {
        tc_macro_running = true;
        set_keepout_state(false, SOURCE_MACRO);
    }
    if (prev_on_tool_selected)
        prev_on_tool_selected(tool);
}

static void keepout_tool_changed(tool_data_t *tool)
{
    if (keepout.flags.enabled && keepout.flags.monitor_tc_macro) {
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
    if (keepout.flags.enabled && keepout.flags.monitor_rack_presence) {
        bool current_pin_is_low = !DIGITAL_IN(AUXINPUT7_PORT, AUXINPUT7_PIN);
        if (current_pin_is_low != keepout.last_pin_state) {
            keepout.last_pin_state = current_pin_is_low;
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
        (pos[X_AXIS] >= keepout.x_min && pos[X_AXIS] <= keepout.x_max &&
         pos[Y_AXIS] >= keepout.y_min && pos[Y_AXIS] <= keepout.y_max);

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
        x0 - keepout.x_min, keepout.x_max - x0,
        y0 - keepout.y_min, keepout.y_max - y0
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

static bool is_keepout_active(void) // remove
{
    return config.flags.enabled && keepout.flags.enabled;
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

    if (xt >= keepout.x_min && xt <= keepout.x_max && yt >= keepout.y_min && yt <= keepout.y_max) {
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
    work_envelope_t new;

    if (!is_keepout_active()) {
        if (prev_apply_travel_limits)
            prev_apply_travel_limits(target, current_position, envelope);
        return;
    }

    float xt = target[X_AXIS];
    float yt = target[Y_AXIS];
    float x0 = current_position[X_AXIS];
    float y0 = current_position[Y_AXIS];

    bool in_box = (xt >= keepout.x_min && xt <= keepout.x_max && yt >= keepout.y_min && yt <= keepout.y_max);

    if (in_box) {
        if (inside_keepout_zone)
            report_message("ATCI: You are currently inside the keepout zone. Disable keepout before Jogging to safety", Message_Warning);
        memcpy(target, current_position, sizeof(float) * N_AXIS);
        return;
    }

    if(line_intersects_keepout(x0, y0, xt, yt)) {

        memcpy(&new, envelope, sizeof(work_envelope_t));

        if(target[X_AXIS] != current_position[X_AXIS]) {
            if(current_position[X_AXIS] <= keepout.x_min && target[X_AXIS] > keepout.x_min)
                new.max.x = keepout.x_min;
            if(current_position[X_AXIS] >= keepout.x_max && target[X_AXIS] < keepout.x_max)
                new.min.x = keepout.x_max;
        }

        if(target[Y_AXIS] != current_position[Y_AXIS]) {
            if(current_position[Y_AXIS] <= keepout.y_min && target[Y_AXIS] > keepout.y_min)
                new.max.y = keepout.y_min;
            if(current_position[Y_AXIS] >= keepout.y_max && target[Y_AXIS] < keepout.y_max)
                new.min.y = keepout.y_max;
        }

        envelope = &new;
    }

    prev_apply_travel_limits(target, current_position, envelope);
}

// --- M810 handlers ---
static user_mcode_type_t mcode_check(user_mcode_t mcode)
{
    if (keepout.flags.enabled && mcode == 810)
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
            gc_block->words.p = 0;
        }
 //       if (gc_block->words.value) there may be other commands in the block so leave this check to the parser
 //           state = Status_GcodeUnusedWords;
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

    if (!keepout.flags.enabled || state == STATE_CHECK_MODE)
        return;

    if (gc_block->words.p) // FYI gc_block->words.p gets reinstated on execute
        set_keepout_state(gc_block->values.p == 1.0f, SOURCE_COMMAND);
    else
        report_message("Use M810 P1 to enable Sienci ATCi Keepout, M810 P0 to disable.", Message_Info);
}

static const setting_detail_t plugin_settings[] = {
    { SETTING_PLUGIN_ENABLE,         Group_Limits, "ATCi Plugin",  NULL, Format_XBitfield, "Enable,Monitor Rack Presence,Monitor TC Macro", NULL, NULL, Setting_NonCore, &config.flags.value },
    { SETTING_X_MIN,                 Group_Limits, "ATCi Keepout X Min", "mm", Format_Decimal, "-####0.00", "-10000", "10000", Setting_NonCore, &config.x_min },
    { SETTING_Y_MIN,                 Group_Limits, "ATCi Keepout Y Min", "mm", Format_Decimal, "-####0.00", "-10000", "10000", Setting_NonCore, &config.y_min },
    { SETTING_X_MAX,                 Group_Limits, "ATCi Keepout X Max", "mm", Format_Decimal, "-####0.00", "-10000", "10000", Setting_NonCore, &config.x_max },
    { SETTING_Y_MAX,                 Group_Limits, "ATCi Keepout Y Max", "mm", Format_Decimal, "-####0.00", "-10000", "10000", Setting_NonCore, &config.x_max },
};

static void keepout_save (void)
{
    keepout_set();
    hal.nvs.memcpy_to_nvs(nvs_addr, (uint8_t *)&config, sizeof(config), true);
}

// --- Restore & load ---
static void keepout_restore(void)
{
    config.x_min = 10.0f;
    config.y_min = 10.0f;
    config.x_max = 50.0f;
    config.y_max = 50.0f;
    config.flags.enabled  = true;
    config.flags.monitor_rack_presence = false;
    config.flags.monitor_tc_macro = false;
    hal.nvs.memcpy_to_nvs(nvs_addr, (uint8_t *)&config, sizeof(config), true);
}

static void keepout_load(void)
{
    if (hal.nvs.memcpy_from_nvs((uint8_t *)&config, nvs_addr, sizeof(config), true) != NVS_TransferResult_OK)
        keepout_restore();
    else
        keepout_set();

    tc_macro_running = false;
    if (keepout.flags.monitor_rack_presence) {
        keepout.last_pin_state = !DIGITAL_IN(AUXINPUT7_PORT, AUXINPUT7_PIN);
        set_keepout_state(keepout.last_pin_state, SOURCE_RACK);
    } else {
        set_keepout_state(true, SOURCE_STARTUP);
    }

    task_add_delayed(poll_rack_sensor, NULL, 1000); // start polling after 1s
}

// --- Report options ---
static void onReportOptions(bool newopt)
{
    if (on_report_options)
        on_report_options(newopt);
    if (!newopt)
        report_plugin("Sienci ATCi plugin", "0.3.0");
}

static void onReportNgcParameters (void)
{
    char buf[100]; //

    // move to $# report
    snprintf(buf, sizeof(buf),
             "[ATCI:%.2f,%.2f,%.2f,%.2f]" ASCII_EOL,
             keepout.x_max,
             keepout.x_min,
             keepout.y_max,
             keepout.y_min);

    hal.stream.write(buf);
    if (on_report_ngc_parameters)
        on_report_ngc_parameters();
}

// --- Realtime report ---
static void onRealtimeReport (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    char buf[256];
    char flags[16];
    size_t f = 0;

    if (config.flags.enabled)
        flags[f++] = 'E';

    switch (keepout.source) {
        case SOURCE_RACK:    flags[f++] = 'R'; break;
        case SOURCE_COMMAND: flags[f++] = 'M'; break;
        case SOURCE_MACRO:   flags[f++] = 'T'; break;
        case SOURCE_STARTUP: flags[f++] = 'S'; break;
        default: break;
    }

    if (config.flags.monitor_rack_presence && keepout.last_pin_state)
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

    // move to $# report, see above
    snprintf(buf, sizeof(buf),
             "|ATCI:%.2f,%.2f,%.2f,%.2f,%s",
             keepout.x_max,
             keepout.x_min,
             keepout.y_max,
             keepout.y_min,
             flags);

    stream_write(buf);
    if (on_realtime_report)
        on_realtime_report(stream_write, report);
}

// --- Init ---
void my_plugin_init(void)
{
    static setting_details_t settings = {
        .settings = plugin_settings,
        .n_settings = sizeof(plugin_settings) / sizeof(setting_detail_t),
        .load = keepout_load,
        .save = keepout_save,
        .restore = keepout_restore
    };

    if ((nvs_addr = nvs_alloc(sizeof(config)))) {

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

        report_message("Sienci ATCi plugin v0.3.0 initialized", Message_Info);
    }
}
