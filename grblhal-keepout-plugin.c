#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "grbl/hal.h"
#include "driver.h" // Required for DIGITAL_IN and pin macros
#include "grbl/nvs_buffer.h"
#include "grbl/gcode.h"
#include "grbl/system.h"
#include "grbl/motion_control.h"
#include "grbl/settings.h"
#include "grbl/plugins.h"



extern system_t sys;

// #define KEEP_DEBUG 1

typedef struct {
    float x_min;
    float y_min;
    float x_max;
    float y_max;
    bool enabled;             // Current enabled/disabled state, controlled by M-code or pin
    bool plugin_enabled;      // Master on/off switch for the plugin from settings
    bool monitor_rack_presence; // Setting to enable/disable pin monitoring
    bool monitor_tc_macro; // Setting to enable/disable toolchange macro monitoring
    bool manual_override;     // Flag to indicate M810 has been used
    bool last_pin_state;      // The last seen state of the input pin (true = LOW/present)
} keepout_config_t;

static keepout_config_t config;
static nvs_address_t nvs_addr;

static user_mcode_ptrs_t user_mcode = {0};
static on_report_options_ptr on_report_options = NULL;
static bool p_word_present = false; // Flag to communicate between validate and execute steps

// Typedefs for the original limit checking functions we are overriding
typedef bool (*travel_limits_ptr)(float *target, axes_signals_t axes, bool is_cartesian, work_envelope_t *envelope);
typedef void (*apply_travel_limits_ptr)(float *target, float *position, work_envelope_t *envelope);
typedef bool (*arc_limits_ptr)(coord_data_t *target, coord_data_t *position, point_2d_t center, float radius, plane_t plane, int32_t turns, work_envelope_t *envelope);

static travel_limits_ptr prev_check_travel_limits = NULL;
static apply_travel_limits_ptr prev_apply_travel_limits = NULL;
static arc_limits_ptr prev_check_arc_limits = NULL;

static on_tool_selected_ptr on_tool_selected;
static on_tool_changed_ptr on_tool_changed;

// Settings Order
#define SETTING_PLUGIN_ENABLE         Setting_UserDefined_0
#define SETTING_MONITOR_RACK_PRESENCE Setting_UserDefined_1
#define SETTING_MONITOR_TC_MACRO      Setting_UserDefined_2
#define SETTING_X_MIN                 Setting_UserDefined_3
#define SETTING_Y_MIN                 Setting_UserDefined_4
#define SETTING_X_MAX                 Setting_UserDefined_5
#define SETTING_Y_MAX                 Setting_UserDefined_6

// Forward declarations for our separate getter/setter functions
static status_code_t set_bool_setting(setting_id_t id, uint_fast16_t value);
static uint_fast16_t get_bool_setting(setting_id_t id);
static status_code_t set_float_setting(setting_id_t id, float value);
static float get_float_setting(setting_id_t id);

static inline float keepout_xmin(void) { return config.x_min < config.x_max ? config.x_min : config.x_max; }
static inline float keepout_xmax(void) { return config.x_min < config.x_max ? config.x_max : config.x_min; }
static inline float keepout_ymin(void) { return config.y_min < config.y_max ? config.y_min : config.y_max; }
static inline float keepout_ymax(void) { return config.y_min < config.y_max ? config.y_max : config.y_min; }

static void report_keepout_status(void); // Forward declaration

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
    if (!config.plugin_enabled) {
        return false;
    }

    if (!config.monitor_rack_presence) {
        return config.enabled;
    }

    bool current_pin_is_low = !DIGITAL_IN(AUXINPUT7_PORT, AUXINPUT7_PIN);

    if (current_pin_is_low != config.last_pin_state) {
        config.last_pin_state = current_pin_is_low;
        config.enabled = current_pin_is_low;
        config.manual_override = false;
        report_keepout_status(); // Run the full status report on state change
    }

    return config.enabled;
}

static bool travel_limits_check(float *target, axes_signals_t axes, bool is_cartesian, work_envelope_t *envelope)
{
    if (!is_keepout_active()) {
        return prev_check_travel_limits ? prev_check_travel_limits(target, axes, is_cartesian, envelope) : true;
    }

    float xt = target[X_AXIS];
    float yt = target[Y_AXIS];

    float pos[N_AXIS];
    system_convert_array_steps_to_mpos(pos, sys.position);
    float x0 = pos[X_AXIS];
    float y0 = pos[Y_AXIS];

    if (xt >= keepout_xmin() && xt <= keepout_xmax() &&
        yt >= keepout_ymin() && yt <= keepout_ymax()) {
        report_message("Keepout: Target inside region", Message_Warning);
        return false;
    }

    if (line_intersects_keepout(x0, y0, xt, yt)) {
        report_message("Keepout: Move crosses keepout zone", Message_Warning);
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

    bool in_box = (xt >= keepout_xmin() && xt <= keepout_xmax() &&
                   yt >= keepout_ymin() && yt <= keepout_ymax());
    bool intersects = line_intersects_keepout(x0, y0, xt, yt);

    if (in_box || intersects) {
        report_message("Keepout: Jog move blocked", Message_Warning);
        memcpy(target, current_position, sizeof(float) * N_AXIS);
        return;
    }

    if (prev_apply_travel_limits)
        prev_apply_travel_limits(target, current_position, envelope);
}

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
        state = Status_OK; // We are handling this M-code
        p_word_present = false; // Reset flag for this block

        if (gc_block->words.p) {
            p_word_present = true; // Set flag for execute step
            // Check for a valid P value (must be 0 or 1)
            if (gc_block->values.p != 0.0f && gc_block->values.p != 1.0f) {
                state = Status_GcodeValueOutOfRange;
            }
            gc_block->words.p = 0; // "Consume" the P-word so the parser doesn't complain
        }

        // Check if any other value words (X, Y, etc.) were used inappropriately
        if (gc_block->words.value) {
          state = Status_GcodeUnusedWords;
        }
    }

    // This is the proper way to chain plugin hooks
    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block) : state;
}

static void keepout_tool_selected (tool_data_t *tool)
{

  config.enabled = false;
  report_keepout_status()
  if(on_tool_selected)
      on_tool_selected(tool);

}

static void keepout_tool_changed (tool_data_t *tool)
{

  if(on_tool_changed)
      on_tool_changed(tool);

      config.enabled = true;
      report_keepout_status()
}

static void report_keepout_status(void)
{
    char report_buf[220]; // Increased buffer size for safety
    const char *rack_status_str;

    if(config.monitor_rack_presence) {
        rack_status_str = !DIGITAL_IN(AUXINPUT7_PORT, AUXINPUT7_PIN) ? "INSTALLED" : "REMOVED";
    } else {
        rack_status_str = "N/A";
    }

    snprintf(report_buf, sizeof(report_buf),
             "[SKO:%d|STATUS:%s|OVERRIDE:%s|XMIN:%.2f|XMAX:%.2f|YMIN:%.2f|YMAX:%.2f|SOFTLIM:%s|RACK:%s]",
             config.plugin_enabled,
             config.enabled ? "ENABLED" : "DISABLED",
             config.manual_override ? "TRUE" : "FALSE",
             keepout_xmin(),
             keepout_xmax(),
             keepout_ymin(),
             keepout_ymax(),
             (settings.limits.soft_enabled.mask != 0) ? "TRUE" : "FALSE",
             rack_status_str);

    hal.stream.write(report_buf);
    hal.stream.write(ASCII_EOL);
}

static void mcode_execute(uint_fast16_t state, parser_block_t *gc_block)
{
    if (state == STATE_CHECK_MODE || gc_block->user_mcode != 810)
        return;

    // Check our static flag that was set during the validate step
    if (p_word_present) {
        if (gc_block->values.p == 1.0f) {
            // M810 P1 enables the keepout
            config.enabled = true;
            config.manual_override = true;
        } else if (gc_block->values.p == 0.0f) {
            // M810 P0 disables the keepout
            config.enabled = false;
            config.manual_override = true;
        }
    } else {
        // M810 without P-word reports status and help
        report_keepout_status();
        report_message("Use M810 P1 to enable keepout, M810 P0 to disable.", Message_Info);
        return; // Exit before the final status report
    }

    report_keepout_status();
}


static status_code_t set_bool_setting(setting_id_t id, uint_fast16_t value) {
    bool val = (value != 0);
    switch(id) {
        case SETTING_PLUGIN_ENABLE:           config.plugin_enabled = val; break;
        case SETTING_MONITOR_RACK_PRESENCE:   config.monitor_rack_presence = val; break;
        case SETTING_MONITOR_TC_MACRO:   config.monitor_tc_macro = val; break;
        default: return Status_Unhandled;
    }
    hal.nvs.memcpy_to_nvs(nvs_addr, (uint8_t *)&config, sizeof(config), true);
    return Status_OK;
}

static uint_fast16_t get_bool_setting(setting_id_t id) {
    switch(id) {
        case SETTING_PLUGIN_ENABLE:           return config.plugin_enabled;
        case SETTING_MONITOR_RACK_PRESENCE:   return config.monitor_rack_presence;
        case SETTING_MONITOR_TC_MACRO: return config.monitor_tc_macro;
        default: return 0;
    }
}

static status_code_t set_float_setting(setting_id_t id, float value) {
    switch(id) {
        case SETTING_X_MIN:                   config.x_min = value; break;
        case SETTING_Y_MIN:                   config.y_min = value; break;
        case SETTING_X_MAX:                   config.x_max  = value; break;
        case SETTING_Y_MAX:                   config.y_max = value; break;
        default: return Status_Unhandled;
    }
    hal.nvs.memcpy_to_nvs(nvs_addr, (uint8_t *)&config, sizeof(config), true);
    return Status_OK;
}

static float get_float_setting(setting_id_t id) {
    switch(id) {
        case SETTING_X_MIN:                   return config.x_min;
        case SETTING_Y_MIN:                   return config.y_min;
        case SETTING_X_MAX:                   return config.x_max;
        case SETTING_Y_MAX:                   return config.y_max;
        default: return 0.0f;
    }
}


static const setting_detail_t plugin_settings[] = {
    { SETTING_PLUGIN_ENABLE,         Group_UserSettings, "Keepout Plugin Enabled",        NULL, Format_Bool,    NULL,       NULL,     NULL,    Setting_IsLegacyFn, (void *)set_bool_setting, (void *)get_bool_setting },
    { SETTING_MONITOR_RACK_PRESENCE, Group_UserSettings, "Keepout Monitor Rack Presence", NULL, Format_Bool,    NULL,       NULL,     NULL,    Setting_IsLegacyFn, (void *)set_bool_setting, (void *)get_bool_setting },
    { SETTING_MONITOR_TC_MACRO,      Group_UserSettings, "Keepout Monitor TC Macro",      NULL, Format_Bool,    NULL,       NULL,     NULL,    Setting_IsLegacyFn, (void *)set_bool_setting, (void *)get_bool_setting },
    { SETTING_X_MIN,                 Group_UserSettings, "Keepout X Min",                 "mm",  Format_Decimal, "-####0.00", "-10000", "10000", Setting_IsLegacyFn, set_float_setting, get_float_setting },
    { SETTING_Y_MIN,                 Group_UserSettings, "Keepout Y Min",                 "mm", Format_Decimal, "-####0.00", "-10000", "10000", Setting_IsLegacyFn, set_float_setting, get_float_setting },
    { SETTING_X_MAX,                 Group_UserSettings, "Keepout X Max",                 "mm", Format_Decimal, "-####0.00", "-10000", "10000", Setting_IsLegacyFn, set_float_setting, get_float_setting },
    { SETTING_Y_MAX,                 Group_UserSettings, "Keepout Y Max",                 "mm", Format_Decimal, "-####0.00", "-10000", "10000", Setting_IsLegacyFn, set_float_setting, get_float_setting },
};

static void keepout_restore(void)
{
    config.x_min = 10.0f;
    config.y_min = 10.0f;
    config.x_max = 50.0f;
    config.y_max = 50.0f;
    config.enabled  = true;
    config.plugin_enabled = false;
    config.monitor_rack_presence = false;
    config.manual_override = false;
    config.last_pin_state = false;

    hal.nvs.memcpy_to_nvs(nvs_addr, (uint8_t *)&config, sizeof(config), true);
}

static void keepout_load(void)
{
    if (hal.nvs.memcpy_from_nvs((uint8_t *)&config, nvs_addr, sizeof(config), true) != NVS_TransferResult_OK)
        keepout_restore();
}

static void onReportOptions(bool newopt)
{
    if(on_report_options)
        on_report_options(newopt);

    if(!newopt)
        report_plugin("SIENCI Keepout Plugin", "2.2"); // Version bump for rack status
}

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

        if(config.plugin_enabled && config.monitor_rack_presence) {
            config.last_pin_state = !DIGITAL_IN(AUXINPUT7_PORT, AUXINPUT7_PIN);
            config.enabled = config.last_pin_state;
            config.manual_override = false;
        }

        prev_check_travel_limits = grbl.check_travel_limits;
        grbl.check_travel_limits = travel_limits_check;

        prev_apply_travel_limits = grbl.apply_travel_limits;
        grbl.apply_travel_limits = keepout_apply_travel_limits;

        prev_check_arc_limits = grbl.check_arc_travel_limits;

        memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));
        grbl.user_mcode.check = mcode_check;
        grbl.user_mcode.validate = mcode_validate;
        grbl.user_mcode.execute = mcode_execute;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        on_tool_selected = grbl.on_tool_selected;
        grbl.on_tool_selected = keepout_tool_selected;

        on_tool_changed = grbl.on_tool_changed;
        grbl.on_tool_changed = keepout_tool_changed;

        settings_register(&settings);
        report_message("Keepout plugin initialized", Message_Info);
    }
}
