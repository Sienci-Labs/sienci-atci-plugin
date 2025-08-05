#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "grbl/hal.h"
#include "grbl/nvs_buffer.h"
#include "grbl/gcode.h"


// todo:
// - Obtain correct $$ options from Terje
// - Address alarms to prevent lockout of jog (and drops into Critical Event needing reset - to fix)
// - Improve reporting on enable/disable
// - better way to handle enabled/disabled status for UI
//


// Keepout config stored in NVS
typedef struct {
    float x_offset;
    float y_offset;
    float x_width;
    float y_height;
    bool enabled;
} keepout_config_t;


static keepout_config_t config;
static nvs_address_t nvs_addr;

static user_mcode_ptrs_t user_mcode = {0};
static on_report_options_ptr on_report_options = NULL;

static travel_limits_ptr prev_check_travel_limits = NULL;
static apply_travel_limits_ptr prev_apply_travel_limits = NULL;

#define SETTING_X_OFFSET   Setting_UserDefined_0
#define SETTING_Y_OFFSET   Setting_UserDefined_1
#define SETTING_X_WIDTH    Setting_UserDefined_2
#define SETTING_Y_HEIGHT   Setting_UserDefined_3

// Setters for plugin settings
static status_code_t set_setting(setting_id_t id, float value)
{
    switch(id) {
        case SETTING_X_OFFSET:  config.x_offset = value; break;
        case SETTING_Y_OFFSET:  config.y_offset = value; break;
        case SETTING_X_WIDTH:   config.x_width  = value; break;
        case SETTING_Y_HEIGHT:  config.y_height = value; break;
        default: return Status_Unhandled;
    }

    hal.nvs.memcpy_to_nvs(nvs_addr, (uint8_t *)&config, sizeof(config), true);
    return Status_OK;
}

// Getters for plugin settings
static float get_setting(setting_id_t id)
{
    switch(id) {
        case SETTING_X_OFFSET:  return config.x_offset;
        case SETTING_Y_OFFSET:  return config.y_offset;
        case SETTING_X_WIDTH:   return config.x_width;
        case SETTING_Y_HEIGHT:  return config.y_height;
        default: return 0.0f;
    }
}

// Plugin settings definitions
static const setting_detail_t plugin_settings[] = {
    { SETTING_X_OFFSET, Group_UserSettings, "Keepout X Offset", "mm", Format_Decimal, "#0.00", NULL, NULL, Setting_IsLegacyFn, set_setting, get_setting },
    { SETTING_Y_OFFSET, Group_UserSettings, "Keepout Y Offset", "mm", Format_Decimal, "#0.00", NULL, NULL, Setting_IsLegacyFn, set_setting, get_setting },
    { SETTING_X_WIDTH,  Group_UserSettings, "Keepout X Width",  "mm", Format_Decimal, "#0.00", NULL, NULL, Setting_IsLegacyFn, set_setting, get_setting },
    { SETTING_Y_HEIGHT, Group_UserSettings, "Keepout Y Height", "mm", Format_Decimal, "#0.00", NULL, NULL, Setting_IsLegacyFn, set_setting, get_setting },
};

// travel limits filter — checks if target is inside the keepout zone
static bool travel_limits_check(float *target, axes_signals_t axes, bool is_cartesian)
{
    float x = target[X_AXIS];
    float y = target[Y_AXIS];

    if (config.enabled) {
        float xmin = -config.x_offset - config.x_width;
        float xmax = -config.x_offset;
        float ymin = -config.y_offset - config.y_height;
        float ymax = -config.y_offset;

        if (x >= xmin && x <= xmax && y >= ymin && y <= ymax) {
            report_message("Keepout zone: move blocked", Message_Warning); // in contrast with Jog move blocked (apply_travel_limits) this checks other moves
            system_raise_alarm(Alarm_SoftLimit);
            return false;
        }
    }

    // Chain to original travel limits checker if any
    return prev_check_travel_limits ? prev_check_travel_limits(target, axes, is_cartesian) : true;
}

// The apply_travel_limits filter for jog moves (void return)
static void keepout_apply_travel_limits(float *target, float *current_position)
{
    float x = target[X_AXIS];
    float y = target[Y_AXIS];

    if (config.enabled) {
        float xmin = -config.x_offset - config.x_width;
        float xmax = -config.x_offset;
        float ymin = -config.y_offset - config.y_height;
        float ymax = -config.y_offset;

        if (x >= xmin && x <= xmax && y >= ymin && y <= ymax) {
            report_message("Keepout zone: Jog move blocked", Message_Warning);

            // Set fault to soft limit error, stop jog immediately
            system_raise_alarm(Alarm_SoftLimit);
            return;
        }
    }

    // Chain to original apply_travel_limits if any
    if (prev_apply_travel_limits)
        prev_apply_travel_limits(target, current_position);
}

// M-code check to identify which M-codes this plugin handles
static user_mcode_type_t mcode_check(user_mcode_t mcode)
{
    if(mcode == 810 || mcode == 811)
        return UserMCode_Normal;
    return user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported;
}

// M-code validation — accept all for now
static status_code_t mcode_validate(parser_block_t *gc_block)
{
    (void)gc_block;
    return Status_OK;
}

// M-code execution — enable or disable keepout zone
static void mcode_execute(uint_fast16_t state, parser_block_t *gc_block)
{
    if(state == STATE_CHECK_MODE)
        return;

    if(gc_block->user_mcode == 810) {
        config.enabled = true;
        report_message("Keepout ENABLED", Message_Info);

        char region[100];
        float xmin = -config.x_offset - config.x_width;
        float xmax = -config.x_offset;
        float ymin = -config.y_offset - config.y_height;
        float ymax = -config.y_offset;
        snprintf(region, sizeof(region), "Keepout Region: X[%.2f..%.2f] Y[%.2f..%.2f]", xmin, xmax, ymin, ymax);
        report_message(region, Message_Info);
    }
    else if(gc_block->user_mcode == 811) {
        config.enabled = false;
        report_message("Keepout DISABLED", Message_Info);
    }
}

// Restore defaults if NVS is empty or invalid
static void keepout_restore(void)
{
    config.x_offset = 50.0f;
    config.y_offset = 50.0f;
    config.x_width  = 40.0f;
    config.y_height = 40.0f;
    config.enabled  = true;

    hal.nvs.memcpy_to_nvs(nvs_addr, (uint8_t *)&config, sizeof(config), true);
}

// Load settings from NVS or restore defaults
static void keepout_load(void)
{
    if (hal.nvs.memcpy_from_nvs((uint8_t *)&config, nvs_addr, sizeof(config), true) != NVS_TransferResult_OK)
        keepout_restore();
}

// Hook into grbl's report options to show plugin info
static void onReportOptions(bool newopt)
{
    if(on_report_options)
        on_report_options(newopt);

    if(!newopt)
        report_plugin("SIENCI Keepout Plugin", "0.2");

}

// Plugin initialization function
void keepout_init(void)
{
    static setting_details_t settings = {
        .settings = plugin_settings,
        .n_settings = sizeof(plugin_settings) / sizeof(setting_detail_t),
        .load = keepout_load,
        .restore = keepout_restore
    };

    if((nvs_addr = nvs_alloc(sizeof(config)))) {
        keepout_load();

        // Hook travel limits check
        prev_check_travel_limits = grbl.check_travel_limits;
        grbl.check_travel_limits = travel_limits_check;

        // Hook apply travel limits for jog moves
        prev_apply_travel_limits = grbl.apply_travel_limits;
        grbl.apply_travel_limits = keepout_apply_travel_limits;

        // Hook M-code handlers
        memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));
        grbl.user_mcode.check = mcode_check;
        grbl.user_mcode.validate = mcode_validate;
        grbl.user_mcode.execute = mcode_execute;

        // Hook report options callback
        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        settings_register(&settings);

        report_message("Keepout plugin initialized", Message_Info);
    }
}
