/*

  LDM.c - plugin for for interfacing with laserline LDM

  Part of grblHAL

  Copyright (c) 2025 Mitchell Grams

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.

*/

#include "driver.h"

#if FANS_ENABLE

// #if FANS_ENABLE
// #error "FANS PLUGIN CANNOT BE ENABLED WITH CUSTOM LDM PLUGIN"
// #endif

#define SIGNALS 4

#include <string.h>
#include <math.h>

#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"

typedef struct {
    uint8_t port[4];
} ldm_settings_t;

static const char *signal_names[] = {
    "Laser Pilot",
    "Laser Shutter",
    "Laser Threshold",
    "Powder Select"
};

typedef enum {
    LaserPilot = 0,
    LaserShutter = 1,
    LaserThreshold = 2,
    PowderSelect = 3
} ldm_signals_t;

typedef enum {
    LaserPilot_On = 510,
    LaserPilot_Off = 511,
    LaserShutter_On = 512,
    LaserShutter_Off = 513,
    LaserThreshold_On = 514,
    LaserThreshold_Off = 515,
    PowderSelectHopper1 = 520,
    PowderSelectHopper2 = 522
} ldm_mcode_t;

static uint32_t n_signals = 0, signals_on = 0;
static user_mcode_ptrs_t user_mcode;
static ldm_settings_t ldm_setting, signals;
static io_port_cfg_t d_out;
static nvs_address_t nvs_address;

static on_report_options_ptr on_report_options;
static on_realtime_report_ptr on_realtime_report;
static on_program_completed_ptr on_program_completed;
static on_unknown_accessory_override_ptr on_unknown_accessory_override;
static driver_reset_ptr driver_reset;

bool ldm_get_state (uint8_t signal);
void ldm_set_state (uint8_t signal, bool on);

static user_mcode_type_t userMCodeCheck (user_mcode_t mcode)
{
    return ((ldm_mcode_t) mcode == LaserPilot_On || (ldm_mcode_t) mcode == LaserPilot_Off ||
            (ldm_mcode_t) mcode == LaserShutter_On || (ldm_mcode_t) mcode == LaserShutter_Off ||
            (ldm_mcode_t) mcode == LaserThreshold_On || (ldm_mcode_t) mcode == LaserThreshold_Off ||
            (ldm_mcode_t) mcode == PowderSelectHopper1 || (ldm_mcode_t) mcode == PowderSelectHopper2
            )
                     ? UserMCode_Normal //  Handled by us. Set to UserMCode_NoValueWords if there are any parameter words (letters) without an accompanying value.
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);	// If another handler present then call it or return ignore.
}

static status_code_t userMCodeValidate (parser_block_t *gc_block)
{
    status_code_t state = Status_OK;

    switch((ldm_mcode_t) gc_block->user_mcode) {

        case LaserPilot_On:
            break;
        case LaserPilot_Off:
            break;
        case LaserShutter_On:
            break;
        case LaserShutter_Off:
            break;
        case LaserThreshold_On:
            break;
        case LaserThreshold_Off:
            break;
        case PowderSelectHopper1:
            break;
        case PowderSelectHopper2:
            break;

        default:
            state = Status_Unhandled;
            break;
    }

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block) : state;
}

static void userMCodeExecute (uint_fast16_t state, parser_block_t *gc_block)
{
    bool handled = true;

    if (state != STATE_CHECK_MODE)
      switch((ldm_mcode_t) gc_block->user_mcode) {

        case LaserPilot_On:
            ldm_set_state(LaserPilot, On);
            break;
        case LaserPilot_Off:
            ldm_set_state(LaserPilot, Off);
            break;
        case LaserShutter_On:
            ldm_set_state(LaserShutter, On);
            break;
        case LaserShutter_Off:
            ldm_set_state(LaserShutter, Off);
            break;
        case LaserThreshold_On:
            ldm_set_state(LaserThreshold, On);
            break;
        case LaserThreshold_Off:
            ldm_set_state(LaserThreshold, Off);
            break;
        // case LaserErrorReset_On:
        //     ldm_set_state(LaserReset, On);
        //     break;
        case PowderSelectHopper1:
            ldm_set_state(PowderSelect, On);
            break;
        case PowderSelectHopper2:
            ldm_set_state(PowderSelect, Off);
            break;

        default:
            handled = false;
            break;
    }

    if(!handled && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static void driverReset (void)
{
    driver_reset();

    uint32_t idx = SIGNALS;
    do {
        ldm_set_state(--idx, Off);
    } while(idx);
}

static void onProgramCompleted (program_flow_t program_flow, bool check_mode)
{

    if(on_program_completed)
        on_program_completed(program_flow, check_mode);
}

static void onRealtimeReport (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    if(report.fan) {
        stream_write("|LDM:");
        stream_write(uitoa(signals_on));
    }

    if(on_realtime_report)
        on_realtime_report(stream_write, report);
}

static void onAccessoryOverride (uint8_t cmd)
{
    if(cmd == CMD_OVERRIDE_FAN0_TOGGLE && signals.port[0] != 0xFF)
        ldm_set_state(0, !ldm_get_state(0));
    else if(on_unknown_accessory_override)
        on_unknown_accessory_override(cmd);
}

bool ldm_get_state (uint8_t signal)
{
    return signals.port[signal] != 0xFF && !!(signals_on & (1 << signal));
}

void ldm_set_state (uint8_t signal, bool on)
{
    if(signals.port[signal] != 0xFF) {

        if(on)
            bit_true(signals_on, bit(signal));
        else {
            bit_false(signals_on, bit(signal));
        }

        sys.report.fan = On;
        ioport_digital_out(signals.port[signal], on);
    }
}


static void ldm_setup (void)
{
    memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

    grbl.user_mcode.check = userMCodeCheck;
    grbl.user_mcode.validate = userMCodeValidate;
    grbl.user_mcode.execute = userMCodeExecute;

    driver_reset = hal.driver_reset;
    hal.driver_reset = driverReset;

    on_realtime_report = grbl.on_realtime_report;
    grbl.on_realtime_report = onRealtimeReport;

    on_unknown_accessory_override = grbl.on_unknown_accessory_override;
    grbl.on_unknown_accessory_override = onAccessoryOverride;

    on_program_completed = grbl.on_program_completed;
    grbl.on_program_completed = onProgramCompleted;
}

static bool is_setting_available (const setting_detail_t *setting, uint_fast16_t offset)
{
    return d_out.n_ports >= setting->id - Setting_FanPort0;
}

static status_code_t set_float (setting_id_t setting, float value)
{
    return d_out.set_value(&d_out, &ldm_setting.port[setting - Setting_FanPort0], (pin_cap_t){}, value);
}

static float get_float (setting_id_t setting)
{
    return d_out.get_value(&d_out, ldm_setting.port[setting - Setting_FanPort0]);
}

static const setting_detail_t ldm_settings[] = {
    { Setting_FanPort0, Group_AuxPorts, "Laser Pilot port", NULL, Format_Decimal, "-#0", "-1", d_out.port_maxs, Setting_NonCoreFn, set_float, get_float, is_setting_available, { .reboot_required = On } },
    { Setting_FanPort1, Group_AuxPorts, "Laser Shutter port", NULL, Format_Decimal, "-#0", "-1", d_out.port_maxs, Setting_NonCoreFn, set_float, get_float, is_setting_available, { .reboot_required = On } },
    { Setting_FanPort2, Group_AuxPorts, "Laser Threshold port", NULL, Format_Decimal, "-#0", "-1", d_out.port_maxs, Setting_NonCoreFn, set_float, get_float, is_setting_available, { .reboot_required = On } },
    { Setting_FanPort3, Group_AuxPorts, "Powder Select port", NULL, Format_Decimal, "-#0", "-1", d_out.port_maxs, Setting_NonCoreFn, set_float, get_float, is_setting_available, { .reboot_required = On } },
};

static const setting_descr_t ldm_settings_descr[] = {
    { Setting_FanPort0, "Aux output port number to use for laser pilot control. Set to -1 to disable." },
    { Setting_FanPort1, "Aux output port number to use for laser shutter control. Set to -1 to disable." },
    { Setting_FanPort2, "Aux output port number to use for laser threshold control. Set to -1 to disable." },
    { Setting_FanPort3, "Aux output port number to use for powder select control. Set to -1 to disable." },
};

// Write settings to non volatile storage (NVS).
static void ldm_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&ldm_setting, sizeof(ldm_settings_t), true);
}

// Restore default settings and write to non volatile storage (NVS).
// Default is highest numbered free port.
static void ldm_settings_restore (void)
{
    uint32_t idx = SIGNALS;

    do {
        idx--;
        ldm_setting.port[idx] = d_out.get_next(&d_out, idx == SIGNALS - 1 ? IOPORT_UNASSIGNED : ldm_setting.port[idx + 1], signal_names[idx], (pin_cap_t){});
    } while(idx);

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&ldm_setting, sizeof(ldm_settings_t), true);
}

static void ldm_settings_load (void)
{
    uint_fast8_t failed = 0;
    uint_fast8_t idx = SIGNALS;

    if(hal.nvs.memcpy_from_nvs((uint8_t *)&ldm_setting, nvs_address, sizeof(ldm_settings_t), true) != NVS_TransferResult_OK)
        ldm_settings_restore();

    do {
        if(--idx != 0) {

            if((signals.port[idx] = ldm_setting.port[idx]) != IOPORT_UNASSIGNED && d_out.claim(&d_out, &signals.port[idx], signal_names[idx], (pin_cap_t){}))
                n_signals++;
            else {
                failed++;
                signals.port[idx] = IOPORT_UNASSIGNED;
            }
        }
    } while(idx);
  
    if(n_signals)
        ldm_setup();

    if(failed)
        task_run_on_startup(report_warning, "LDM plugin: configured port number(s) not available");
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt) {
        report_plugin("LDM-Fans", "0.02");
        hal.stream.write("[LDM:");
        hal.stream.write(uitoa(n_signals));
        hal.stream.write("]" ASCII_EOL);
    }
}

void fans_init (void)
{
    static setting_details_t setting_details = {
        .settings = ldm_settings,
        .n_settings = sizeof(ldm_settings) / sizeof(setting_detail_t),
        .descriptions = ldm_settings_descr,
        .n_descriptions = sizeof(ldm_settings_descr) / sizeof(setting_descr_t),
        .save = ldm_settings_save,
        .load = ldm_settings_load,
        .restore = ldm_settings_restore
    };

    if(ioports_cfg(&d_out, Port_Digital, Port_Output)->n_ports && (nvs_address = nvs_alloc(sizeof(ldm_settings_t)))) {

        settings_register(&setting_details);

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

    } else
        task_run_on_startup(report_warning, "LDM plugin failed to initialize!");
}

#endif //FANS_ENABLE
