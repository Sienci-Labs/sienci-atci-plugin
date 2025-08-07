#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "grbl/hal.h"
#include "grbl/nvs_buffer.h"
#include "grbl/gcode.h"
#include "grbl/system.h"
#include "grbl/motion_control.h"

extern system_t sys;

// #define KEEP_DEBUG 1

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
static arc_limits_ptr prev_check_arc_limits = NULL;

#define SETTING_X_OFFSET   Setting_UserDefined_0
#define SETTING_Y_OFFSET   Setting_UserDefined_1
#define SETTING_X_WIDTH    Setting_UserDefined_2
#define SETTING_Y_HEIGHT   Setting_UserDefined_3

static inline float keepout_xmin(void) { return -config.x_offset - config.x_width; }
static inline float keepout_xmax(void) { return -config.x_offset; }
static inline float keepout_ymin(void) { return -config.y_offset - config.y_height; }
static inline float keepout_ymax(void) { return -config.y_offset; }

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

static bool travel_limits_check(float *target, axes_signals_t axes, bool is_cartesian)
{
    if (!config.enabled)
        return prev_check_travel_limits ? prev_check_travel_limits(target, axes, is_cartesian) : true;

    float xt = target[X_AXIS];
    float yt = target[Y_AXIS];

    float pos[N_AXIS];
    system_convert_array_steps_to_mpos(pos, sys.position);
    float x0 = pos[X_AXIS];
    float y0 = pos[Y_AXIS];

    #ifdef KEEP_DEBUG
        char dbg[120];
        snprintf(dbg, sizeof(dbg), "[KEEPOUT_DBG] Move from (%.3f, %.3f) -> (%.3f, %.3f)", x0, y0, xt, yt);
        report_message(dbg, Message_Debug);
    #endif

    if (xt >= keepout_xmin() && xt <= keepout_xmax() &&
        yt >= keepout_ymin() && yt <= keepout_ymax()) {
        report_message("Keepout: Target inside region", Message_Warning);
        return false;
    }

    if (line_intersects_keepout(x0, y0, xt, yt)) {
        report_message("Keepout: Move crosses keepout zone", Message_Warning);
        return false;
    }

    return prev_check_travel_limits ? prev_check_travel_limits(target, axes, is_cartesian) : true;
}


static void keepout_apply_travel_limits(float *target, float *current_position)
{
    if (!config.enabled) {
        if (prev_apply_travel_limits)
            prev_apply_travel_limits(target, current_position);
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
    #ifdef KEEP_DEBUG
        report_message("[KEEPOUT_DBG] Jog blocked due to keepout", Message_Debug);
    #endif
        memcpy(target, current_position, sizeof(float) * N_AXIS);
        return;
    }

    if (prev_apply_travel_limits)
        prev_apply_travel_limits(target, current_position);
}


static user_mcode_type_t mcode_check(user_mcode_t mcode)
{
    return (mcode == 810 || mcode == 811) ? UserMCode_Normal :
           user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported;
}

static status_code_t mcode_validate(parser_block_t *gc_block)
{
    return Status_OK;
}

static void mcode_execute(uint_fast16_t state, parser_block_t *gc_block)
{
    if (state == STATE_CHECK_MODE)
        return;

    if (gc_block->user_mcode == 810) {
        config.enabled = true;
        report_message("Keepout ENABLED", Message_Info);
        prev_check_arc_limits = grbl.check_arc_travel_limits;
        grbl.check_arc_travel_limits = NULL;

        char region[100];
        snprintf(region, sizeof(region), "Keepout: X[%.2f..%.2f] Y[%.2f..%.2f]",
            keepout_xmin(), keepout_xmax(), keepout_ymin(), keepout_ymax());
        report_message(region, Message_Info);

    } else if (gc_block->user_mcode == 811) {
        config.enabled = false;
        report_message("Keepout DISABLED", Message_Info);
        if (prev_check_arc_limits)
            grbl.check_arc_travel_limits = prev_check_arc_limits;
    }
}

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

static const setting_detail_t plugin_settings[] = {
    { SETTING_X_OFFSET, Group_UserSettings, "Keepout X Offset", "mm", Format_Decimal, "#0.00", NULL, NULL, Setting_IsLegacyFn, set_setting, get_setting },
    { SETTING_Y_OFFSET, Group_UserSettings, "Keepout Y Offset", "mm", Format_Decimal, "#0.00", NULL, NULL, Setting_IsLegacyFn, set_setting, get_setting },
    { SETTING_X_WIDTH,  Group_UserSettings, "Keepout X Width",  "mm", Format_Decimal, "#0.00", NULL, NULL, Setting_IsLegacyFn, set_setting, get_setting },
    { SETTING_Y_HEIGHT, Group_UserSettings, "Keepout Y Height", "mm", Format_Decimal, "#0.00", NULL, NULL, Setting_IsLegacyFn, set_setting, get_setting },
};

static void keepout_restore(void)
{
    config.x_offset = 50.0f;
    config.y_offset = 50.0f;
    config.x_width  = 40.0f;
    config.y_height = 40.0f;
    config.enabled  = true;

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
        report_plugin("SIENCI Keepout Plugin", "0.5");
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

        settings_register(&settings);
        report_message("Keepout plugin initialized", Message_Info);
    }
}
