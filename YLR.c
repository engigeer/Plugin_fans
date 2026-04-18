/*

  YLR.c - plugin for for interfacing with IPG YLR series laser

  Part of grblHAL

  Copyright (c) 2026 Mitchell Grams

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

/*TODO
- move all code to laser plugin
- add linearization of powder feed rate (add settings?)
- confirm default settings behaviour?
- reload rpm to 0 when changing settings
- is dout_get_next working as expected?
*/

#include "driver.h"

#if FANS_ENABLE == 2  // YLR

#include "fans.h"

#define SIGNALS 4

#define CMD_GUIDE_TOGGLE            0xBA //!< Realtime command to toggle Pilot on/off
#define CMD_MAINS_TOGGLE            0xBB //!< Realtime command to toggle Shutter on/off
//#define CMD_POWDER_TOGGLE           0xBC //!< Realtime command to toggle Powder on/off

#include <string.h>
#include <math.h>
#include <stdio.h>

#include "grbl/hal.h"
#include "grbl/override.h"
#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"

typedef struct {
    float offset;
    float slope;
} pwm_mxb_t;

typedef struct {
    uint8_t port[SIGNALS];
    uint8_t powder_feedrate_port;
    uint8_t nozzlegas_flowrate_port;
    pwm_mxb_t blc_pwm[2];
} ylr_settings_t;

static const char *signal_names[] = {
    "Laser RemoteKey",
    "Laser Mains",
    "Laser Guide",
    "Laser Error Reset",
};

typedef enum {
    LaserRemoteKey = 0,
    LaserMains = 1,
    LaserGuide = 2,
    LaserErrorReset = 3,
} ylr_signals_t;

typedef enum {
    LaserErrorReset_Mom = 510,
    LaserGuide_On = 511,
    LaserGuide_Off = 512,
    LaserRemoteKey_On = 513,
    LaserRemoteKey_Off = 514,
    LaserMains_Mom = 515,
    PowderFeedRate = 530,          //R#.#
    NozzleGasFlowRate = 550,       //R#.#
} ylr_mcode_t;

static const float ngas_maxval = 50.0f;
static const float pfr_maxval = 12.0f;

static uint8_t powder_feedrate_port;
static uint8_t nozzlegas_flowrate_port;

static uint32_t n_signals = 0, signals_on = 0;
static user_mcode_ptrs_t user_mcode;
static ylr_settings_t ylr_setting, signals;
static io_port_cfg_t d_out, a_out;
static nvs_address_t nvs_address;

static on_report_options_ptr on_report_options;
static on_realtime_report_ptr on_realtime_report;
static on_program_completed_ptr on_program_completed;
static on_unknown_realtime_cmd_ptr on_unknown_realtime_cmd;
static driver_reset_ptr driver_reset;

bool ylr_get_state (uint8_t signal);
void ylr_set_state (uint8_t signal, bool on);
void set_powder_feedrate (float value);
void set_nozzlegas_flowrate (float value);

float pfr_value = 0.0f;
float ngas_value = 0.0f;

static user_mcode_type_t userMCodeCheck (user_mcode_t mcode)
{
    return ((ylr_mcode_t) mcode == LaserGuide_On || (ylr_mcode_t) mcode == LaserGuide_Off ||
            (ylr_mcode_t) mcode == LaserRemoteKey_On || (ylr_mcode_t) mcode == LaserRemoteKey_Off ||
            (ylr_mcode_t) mcode == LaserMains_Mom || (ylr_mcode_t) mcode == LaserErrorReset_Mom ||
            (ylr_mcode_t) mcode == PowderFeedRate || (ylr_mcode_t) mcode == NozzleGasFlowRate
            )
                     ? UserMCode_Normal //  Handled by us. Set to UserMCode_NoValueWords if there are any parameter words (letters) without an accompanying value.
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);	// If another handler present then call it or return ignore.
}

static status_code_t userMCodeValidate (parser_block_t *gc_block)
{
    status_code_t state = Status_OK;

    switch((ylr_mcode_t) gc_block->user_mcode) {

        case LaserGuide_On:
            break;
        case LaserGuide_Off:
            break;
        case LaserRemoteKey_On:
            break;
        case LaserRemoteKey_Off:
            break;
        case LaserMains_Mom:
            break;
        case LaserErrorReset_Mom:
            break;
        case PowderFeedRate:
            if(!gc_block->words.r)
                state = Status_GcodeValueWordMissing;
            else if(gc_block->values.r < 0.0f || gc_block->values.r > pfr_maxval){
                state = Status_GcodeValueOutOfRange;
                report_add_realtime(Report_All);
            }
            gc_block->words.r = Off;
            break;
        case NozzleGasFlowRate:
            if(!gc_block->words.r)
                state = Status_GcodeValueWordMissing;
            else if(gc_block->values.r < 0.0f || gc_block->values.r > ngas_maxval){
                state = Status_GcodeValueOutOfRange;
                report_add_realtime(Report_All);
            }
            gc_block->words.r = Off;
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
      switch((ylr_mcode_t) gc_block->user_mcode) {

        case LaserGuide_On:
            ylr_set_state(LaserGuide, On);
            break;
        case LaserGuide_Off:
            ylr_set_state(LaserGuide, Off);
            break;
        case LaserRemoteKey_On:
            ylr_set_state(LaserRemoteKey, On);
            break;
        case LaserRemoteKey_Off:
            ylr_set_state(LaserRemoteKey, Off);
            break;
        case LaserMains_Mom:
            ylr_set_state(LaserMains, On);
            delay_sec(0.5f, DelayMode_Dwell);
            ylr_set_state(LaserMains, Off);
            break;
        case LaserErrorReset_Mom:
            ylr_set_state(LaserErrorReset, On);
            delay_sec(0.5f, DelayMode_Dwell);
            ylr_set_state(LaserErrorReset, Off);
            break;
        case PowderFeedRate:
            pfr_value = floorf((float)gc_block->values.r * 100)/100;
            set_powder_feedrate(pfr_value);
            break;
        case NozzleGasFlowRate:
            ngas_value = floorf((float)gc_block->values.r);
            set_nozzlegas_flowrate(ngas_value);
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
}

static void onProgramCompleted (program_flow_t program_flow, bool check_mode)
{

    if(on_program_completed)
        on_program_completed(program_flow, check_mode);
}

static void onRealtimeReport (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    static float pfr_prev = 0.0f;
    static float ngas_prev = 0.0f;

    char buf[60] = "";

    if(report.fan) {
        strcat(buf, "|YLR:");
        strcat(buf, uitoa(signals_on));
    }

    if(pfr_prev != pfr_value || report.all) {
        strcat(buf, "|PFR:");
        strcat(buf, ftoa(pfr_value, 2));
        pfr_prev = pfr_value;
    }

    if(ngas_prev != ngas_value || report.all) {
        strcat(buf, "|NGAS:");
        strcat(buf, ftoa(ngas_value, 0));
        ngas_prev = ngas_value;
    }

    if(*buf != '\0')
        stream_write(buf);

    if(on_realtime_report)
        on_realtime_report(stream_write, report);
}

bool ylr_get_state (uint8_t signal)
{
    return signals.port[signal] != 0xFF && !!(signals_on & (1 << signal));
}

void ylr_set_state (uint8_t signal, bool on)
{
    if(signals.port[signal] != 0xFF) {

        if(on)
            bit_true(signals_on, bit(signal));
        else {
            bit_false(signals_on, bit(signal));
        }

        report_add_realtime(Report_Fan);
        ioport_digital_out(signals.port[signal], on);
    }
}

void set_powder_feedrate(float rpm)
{
    float lin_rpm, pwm_value = 0.0f;

    lin_rpm = ylr_setting.blc_pwm[0].offset + rpm * ylr_setting.blc_pwm[0].slope;

    pwm_value= min(100.0f/pfr_maxval * lin_rpm, 100.0f);

    ioport_analog_out(powder_feedrate_port, pwm_value);
}

void set_nozzlegas_flowrate(float rpm)
{
    float lin_rpm, pwm_value = 0.0f;

    lin_rpm = ylr_setting.blc_pwm[1].offset + rpm * ylr_setting.blc_pwm[1].slope;

    pwm_value= min(100.0f/ngas_maxval * lin_rpm, 100.0f);

    ioport_analog_out(nozzlegas_flowrate_port, pwm_value);
}

static bool onRealtimeCmd (char c)
{
    if(c == CMD_GUIDE_TOGGLE && signals.port[LaserGuide] != 0xFF) {
        ylr_set_state(LaserGuide, !ylr_get_state(LaserGuide));
        return true;
    }
    else if(c == CMD_MAINS_TOGGLE && signals.port[LaserMains] != 0xFF) {
        ylr_set_state(LaserMains, !ylr_get_state(LaserMains));
        return true;
    }
    return on_unknown_realtime_cmd == NULL || on_unknown_realtime_cmd(c);
}

static void ylr_setup (void)
{
    memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

    grbl.user_mcode.check = userMCodeCheck;
    grbl.user_mcode.validate = userMCodeValidate;
    grbl.user_mcode.execute = userMCodeExecute;

    driver_reset = hal.driver_reset;
    hal.driver_reset = driverReset;

    on_realtime_report = grbl.on_realtime_report;
    grbl.on_realtime_report = onRealtimeReport;

    on_unknown_realtime_cmd = grbl.on_unknown_realtime_cmd;
    grbl.on_unknown_realtime_cmd = onRealtimeCmd;

    on_program_completed = grbl.on_program_completed;
    grbl.on_program_completed = onProgramCompleted;
}

PROGMEM static const setting_group_detail_t plasma_groups[] = {
    { Group_Root, Group_Plasma, "Laser Integration" },
};

static bool is_setting_available (const setting_detail_t *setting, uint_fast16_t offset)
{
    switch(setting->id) {

        case Setting_UserDefined_0:
        case Setting_UserDefined_1:
        case Setting_UserDefined_2:
        case Setting_UserDefined_3:
            return d_out.n_ports >= setting->id - Setting_UserDefined_0;

        case Setting_UserDefined_7:
        case Setting_UserDefined_8:
            return a_out.n_ports > 0;

        default: break;
    }

    return false;
}

static status_code_t set_float (setting_id_t setting, float value)
{
    status_code_t status = Status_SettingDisabled;

    switch(setting) {

        case Setting_UserDefined_0:
        case Setting_UserDefined_1:
        case Setting_UserDefined_2:
        case Setting_UserDefined_3:
            status = d_out.set_value(&d_out, &ylr_setting.port[setting - Setting_UserDefined_0], (pin_cap_t){}, value);
            break;

        case Setting_UserDefined_7:
            status = a_out.set_value(&a_out, &ylr_setting.powder_feedrate_port, (pin_cap_t){}, value);
            break;

        case Setting_UserDefined_8:
            status = a_out.set_value(&a_out, &ylr_setting.nozzlegas_flowrate_port, (pin_cap_t){}, value);
            break;

        default: break;
    }
    return status;
}

static float get_float (setting_id_t setting)
{
    float value = -1.0f;

    switch(setting) {

        case Setting_UserDefined_0:
        case Setting_UserDefined_1:
        case Setting_UserDefined_2:
        case Setting_UserDefined_3:
            value = d_out.get_value(&d_out, ylr_setting.port[setting - Setting_UserDefined_0]);
            break;

        case Setting_UserDefined_7:
            value = a_out.get_value(&a_out, ylr_setting.powder_feedrate_port);
            break;

        case Setting_UserDefined_8:
            value = a_out.get_value(&a_out, ylr_setting.nozzlegas_flowrate_port);
            break;

        default: break;
    }
    return value;
}

static status_code_t set_linear_piece (setting_id_t id, char *svalue)
{
    uint32_t idx = id - Setting_LinearSpindle1Piece1;
    float offset, slope;

    if(*svalue == '\0' || (svalue[0] == '0' && svalue[1] == '\0')) {
        ylr_setting.blc_pwm[idx].offset = 0.0f;
        ylr_setting.blc_pwm[idx].slope = 1.0f;
    } else if(sscanf(svalue, "%f,%f", &offset, &slope) == 2) {
        ylr_setting.blc_pwm[idx].offset = offset;
        ylr_setting.blc_pwm[idx].slope = slope;
    } else
        return Status_SettingValueOutOfRange;

    return Status_OK;
}

static char *get_linear_piece (setting_id_t id)
{
    static char buf[40];

    uint32_t idx = id - Setting_LinearSpindle1Piece1;

    if(isnan(ylr_setting.blc_pwm[idx].offset))
        *buf = '\0';
    else
        snprintf(buf, sizeof(buf), "%g,%g", ylr_setting.blc_pwm[idx].offset, ylr_setting.blc_pwm[idx].slope);

    return buf;
}

static const setting_detail_t ylr_settings[] = {
    { Setting_UserDefined_0, Group_Plasma, "Laser Remote Key port", NULL, Format_Decimal, "-#0", "-1", d_out.port_maxs, Setting_NonCoreFn, set_float, get_float, is_setting_available, { .reboot_required = On } },
    { Setting_UserDefined_1, Group_Plasma, "Laser Mains port", NULL, Format_Decimal, "-#0", "-1", d_out.port_maxs, Setting_NonCoreFn, set_float, get_float, is_setting_available, { .reboot_required = On } },
    { Setting_UserDefined_2, Group_Plasma, "Laser Guide port", NULL, Format_Decimal, "-#0", "-1", d_out.port_maxs, Setting_NonCoreFn, set_float, get_float, is_setting_available, { .reboot_required = On } },
    { Setting_UserDefined_3, Group_Plasma, "Laser Error Reset port", NULL, Format_Decimal, "-#0", "-1", d_out.port_maxs, Setting_NonCoreFn, set_float, get_float, is_setting_available, { .reboot_required = On } },
    { Setting_UserDefined_7, Group_Plasma, "Powder Feedrate port", NULL, Format_Decimal, "-#0", "-1", a_out.port_maxs, Setting_NonCoreFn, set_float, get_float, is_setting_available, { .reboot_required = On } },
    { Setting_UserDefined_8, Group_Plasma, "Nozzle Gas Flowrate port", NULL, Format_Decimal, "-#0", "-1", a_out.port_maxs, Setting_NonCoreFn, set_float, get_float, is_setting_available, { .reboot_required = On } },
    { Setting_LinearSpindle1Piece1, Group_Plasma, "PWM linear, Powder Feedrate", NULL, Format_String, "x(39)", NULL, "39", Setting_NonCoreFn, set_linear_piece, get_linear_piece, NULL },
    { Setting_LinearSpindle1Piece2, Group_Plasma, "PWM linear, Nozzle Gas Flowrate", NULL, Format_String, "x(39)", NULL, "39", Setting_NonCoreFn, set_linear_piece, get_linear_piece, NULL }
};

static const setting_descr_t ylr_settings_descr[] = {
    { Setting_UserDefined_0, "Aux output port number to use for laser remote keyswitch control. Set to -1 to disable." },
    { Setting_UserDefined_1, "Aux output port number to use for laser mains control. Set to -1 to disable." },
    { Setting_UserDefined_2, "Aux output port number to use for laser guide control. Set to -1 to disable." },
    { Setting_UserDefined_3, "Aux output port number to use for laser error reset. Set to -1 to disable." },
    { Setting_UserDefined_7, "Aux output port number to use for powder feedrate." },
    { Setting_UserDefined_8, "Aux output port number to use for nozzle gas flowrate." },
    { Setting_LinearSpindle1Piece1, "Comma separated list of values: OFFSET, SLOPE, set to blank to disable." },
    { Setting_LinearSpindle1Piece2, "Comma separated list of values: OFFSET, SLOPE, set to blank to disable." },
};

// Write settings to non volatile storage (NVS).
static void ylr_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&ylr_setting, sizeof(ylr_settings_t), true);
}

// Restore default settings and write to non volatile storage (NVS).
// Default is highest numbered free port.
static void ylr_settings_restore (void)
{
    uint32_t idx = SIGNALS;

    do {
        idx--;
        ylr_setting.port[idx] = d_out.get_next(&d_out, ylr_setting.port[idx], signal_names[idx], (pin_cap_t){});
    } while(idx);

    //ylr_setting.powder_feedrate_port = -1;
    //ylr_setting.nozzlegas_flowrate_port = -1;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&ylr_setting, sizeof(ylr_settings_t), true);
}

static void ylr_settings_load (void)
{
    bool ok = false;
    xbar_t *pin = NULL;
    uint_fast8_t failed = 0;
    uint_fast8_t idx = SIGNALS;

    pwm_config_t config = {
        .freq_hz = 5000.0f,
        .min = 0.0f,
        .max = 100.0f,
        .off_value = 0.0f,
        .min_value = 0.0f,
        .max_value = 100.0f,
        .invert = Off
    };

    if(hal.nvs.memcpy_from_nvs((uint8_t *)&ylr_setting, nvs_address, sizeof(ylr_settings_t), true) != NVS_TransferResult_OK)
        ylr_settings_restore();

    powder_feedrate_port = ylr_setting.powder_feedrate_port;
    nozzlegas_flowrate_port = ylr_setting.nozzlegas_flowrate_port;

    if(powder_feedrate_port == IOPORT_UNASSIGNED) {
        ok = true;

    } else if(pin = a_out.claim(&a_out, &powder_feedrate_port, "Powder feedrate", (pin_cap_t){})) {
        // if (pin->pin == 1 || pin->pin == 2){
        //     config.invert = On;
        //     pin->config(pin, &config, false);
        // } else
        if (pin->config){
            config.invert = Off;
            pin->config(pin, &config, false);
        }
    }
    else
        failed++;

    if(nozzlegas_flowrate_port == IOPORT_UNASSIGNED) {
        ok = true;

    } else if(pin = a_out.claim(&a_out, &nozzlegas_flowrate_port, "Nozzle gas flowrate", (pin_cap_t){})) {
        ok = true;
        // if (pin->pin == 1 || pin->pin == 2){
        //     config.invert = On;
        //     pin->config(pin, &config, false);
        // } else
        if (pin->config){
            config.invert = Off;
            pin->config(pin, &config, false);
        }
    }
    else
        failed++;

    do {
        if(--idx >= 0) {

            if((signals.port[idx] = ylr_setting.port[idx]) != IOPORT_UNASSIGNED && d_out.claim(&d_out, &signals.port[idx], signal_names[idx], (pin_cap_t){}))
                n_signals++;
            else {
                failed++;
                signals.port[idx] = IOPORT_UNASSIGNED;
            }
        }
    } while(idx);
  
    if(ok || n_signals) {
        ylr_setup();

        ylr_setting.blc_pwm[0] = (pwm_mxb_t){ .offset = 0.0f, .slope = 1.0f }; //PFR
        ylr_setting.blc_pwm[1] = (pwm_mxb_t){ .offset = 0.0f, .slope = 1.0f }; //NGas
    }
    if(failed)
        task_run_on_startup(report_warning, "YLR plugin: configured port number(s) not available");
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt) {
        report_plugin("YLR-Fans", "0.02");
        hal.stream.write("[YLR:");
        hal.stream.write(uitoa(n_signals));
        hal.stream.write("]" ASCII_EOL);
    }
}

void ylr_init (void)
{
    static setting_details_t setting_details = {
        .groups = plasma_groups,
        .n_groups = sizeof(plasma_groups) / sizeof(setting_group_detail_t),
        .settings = ylr_settings,
        .n_settings = sizeof(ylr_settings) / sizeof(setting_detail_t),
        .descriptions = ylr_settings_descr,
        .n_descriptions = sizeof(ylr_settings_descr) / sizeof(setting_descr_t),
        .save = ylr_settings_save,
        .load = ylr_settings_load,
        .restore = ylr_settings_restore
    };

    if(ioports_cfg(&d_out, Port_Digital, Port_Output)->n_ports && (nvs_address = nvs_alloc(sizeof(ylr_settings_t)))) {

        ioports_cfg(&a_out, Port_Analog, Port_Output);

        settings_register(&setting_details);

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

    } else
        task_run_on_startup(report_warning, "YLR plugin failed to initialize!");
}

#endif //FANS_ENABLE == 2  // YLR
