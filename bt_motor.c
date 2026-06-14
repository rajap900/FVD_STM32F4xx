/**
  ******************************************************************************
  * @file    bt_motor.c
  * @brief   Bluetooth command + telemetry bridge (HC-05 on UART1).
  *
  *  This is the user's original bridge with two additions required by the HMI,
  *  both backward compatible with the original line protocol:
  *    1. the telemetry frame now also carries the measured I/V/temp/rpm/torque
  *       (printed as integer deci-units, so no %f / printf-float dependency);
  *    2. new commands TORQUE= / POLES= / HZSYS= / MODE= and an optional 7th
  *       NAMEPLATE field (pole count).
  *  Nothing else changed: START/STOP/FWD/REV/CLR_FLT/FREQ=/RAMP= behave exactly
  *  as before, and bt_send still uses HAL_MAX_DELAY for whole frames.
  ******************************************************************************
  */
#include "bt_motor.h"
#include <string.h>
#include <stdio.h>

/* single instance pointer so the parameterless Motor_ callbacks can reach it */
static BtMotor_t *g_bt = 0;

/* ------------------------------------------------------------------ */
/*  Low-level TX                                                       */
/* ------------------------------------------------------------------ */
static void bt_send(BtMotor_t *bt, const char *s)
{
    if (!bt || !bt->huart || !s) return;
    /* IMPORTANT: use HAL_MAX_DELAY. A short timeout can abort mid-frame at low
       baud (a ~104-byte frame needs ~108 ms at 9600), truncating the line and
       dropping its terminating CR/LF -> the UI sees glued, unparsable JSON. */
    HAL_UART_Transmit(bt->huart, (uint8_t *)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

static void bt_send_ok(BtMotor_t *bt, const char *cmd)
{
    char buf[BT_LINE_MAX + 8];
    snprintf(buf, sizeof(buf), "OK:%s\r\n", cmd);
    bt_send(bt, buf);
}

/* ------------------------------------------------------------------ */
/*  Motor event callbacks -> LOG lines for the UI                     */
/* ------------------------------------------------------------------ */
static void cb_start(void)              { if (g_bt) bt_send(g_bt, "LOG:MOTOR_START=ON\r\n"); }
static void cb_stop(void)               { if (g_bt) bt_send(g_bt, "LOG:MOTOR_STOP=ON\r\n"); }
static void cb_dir(bool reverse)        { if (g_bt) bt_send(g_bt, reverse ? "LOG:DIR=REV\r\n" : "LOG:DIR=FWD\r\n"); }
static void cb_fault(Motor_Fault_t f)
{
    const char *s;
    switch (f) {
        case MOTOR_FAULT_OVERCURRENT:  s = "LOG:FAULT OVERCURRENT (BKIN)\r\n"; break;
        case MOTOR_FAULT_OVERTEMP:     s = "LOG:FAULT OVERTEMP\r\n";           break;
        case MOTOR_FAULT_UNDERVOLTAGE: s = "LOG:FAULT UNDERVOLTAGE\r\n";       break;
        case MOTOR_FAULT_OVERVOLTAGE:  s = "LOG:FAULT OVERVOLTAGE\r\n";        break;
        case MOTOR_FAULT_BOOTSTRAP:    s = "LOG:FAULT BOOTSTRAP\r\n";          break;
        case MOTOR_FAULT_ESTOP:        s = "LOG:FAULT ESTOP\r\n";             break;
        default:                       s = "LOG:FAULT UNKNOWN\r\n";            break;
    }
    if (g_bt) bt_send(g_bt, s);
}

/* ------------------------------------------------------------------ */
/*  Small parse helpers (no float printf/scanf dependency)            */
/* ------------------------------------------------------------------ */
static int starts_with(const char *l, const char *p)
{
    return strncmp(l, p, strlen(p)) == 0;
}

/* parse a decimal "x" or "x.y" -> value*10 (one fractional digit) */
static int parse_x10(const char *s)
{
    int sign = 1; long ip = 0, fp = 0; int fdig = 0;
    while (*s == ' ') s++;
    if (*s == '-') { sign = -1; s++; }
    while (*s >= '0' && *s <= '9') { ip = ip * 10 + (*s - '0'); s++; }
    if (*s == '.') { s++; if (*s >= '0' && *s <= '9') { fp = (*s - '0'); fdig = 1; } }
    return (int)(sign * (ip * 10 + (fdig ? fp : 0)));
}

static int parse_int(const char *s)
{
    int sign = 1; long v = 0;
    while (*s == ' ') s++;
    if (*s == '-') { sign = -1; s++; }
    while (*s >= '0' && *s <= '9') { v = v * 10 + (*s - '0'); s++; }
    return (int)(sign * v);
}

static int clampi(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* helpers to split a float into signed integer + 1 decimal digit */
static int whole10(int x10) { return x10 / 10; }
static int frac10 (int x10) { return (x10 < 0 ? -x10 : x10) % 10; }

/* ------------------------------------------------------------------ */
/*  Telemetry frame                                                   */
/* ------------------------------------------------------------------ */
static void bt_send_telemetry(BtMotor_t *bt)
{
    Motor_Handle_t *m = bt->motor;
    Motor_State_t st = m->state;

    int cur_x10  = m->current_freq_hz_x10;
    int tgt_x10  = m->target_freq_hz_x10;
    int ramp_x10 = m->ramp_rate_hz_per_sec_x10;

    int pwr = (st != MOTOR_STATE_IDLE && st != MOTOR_STATE_FAULT) ? 1 : 0;
    int flt = (st == MOTOR_STATE_FAULT) ? 1 : 0;
    int dir = pwr ? (m->is_reverse ? -1 : 1) : 0;

    unsigned int duty = 0;
    if (m->pwm_arr > 0) duty = (unsigned int)(((uint32_t)m->amplitude * 100u) / m->pwm_arr);

    const char *sta = (st == MOTOR_STATE_FAULT) ? "FLT"
                    : (st == MOTOR_STATE_IDLE)  ? "STP" : "RUN";

    /* measured quantities as deci-units (one decimal place, no float printf) */
    int idc = (int)(m->meas->idc_f  * 10.0f);
    int vdc = (int)(m->meas->vdc_f  * 10.0f);
    int tmp = (int)(m->meas->temp_c * 10.0f);
    int rpm = (int)(m->rpm + 0.5f);
    int trq = (int)(m->torque_pct + 0.5f);

    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"frq\":%d.%d,\"fcur\":%d.%d,\"dir\":%d,\"duty\":%u,"
        "\"pwr\":%d,\"flt\":%d,\"ramp\":%d.%d,\"sta\":\"%s\","
        "\"fmin\":%d,\"fmax\":%d,"
        "\"idc\":%d.%d,"
        "\"vdc\":%d.%d,\"temp\":%d.%d,\"rpm\":%d,\"trq\":%d,"
        "\"poles\":%d,\"mode\":\"%s\"}\r\n",
        whole10(cur_x10), frac10(cur_x10),
        whole10(tgt_x10), frac10(tgt_x10),
        dir, duty, pwr, flt,
        whole10(ramp_x10), frac10(ramp_x10),
        sta, bt->fmin_hz, bt->fmax_hz,
        whole10(idc), frac10(idc),
        whole10(vdc), frac10(vdc), whole10(tmp), frac10(tmp),
        rpm, trq, m->poles,
        (m->mode == MOTOR_MODE_FOC) ? "FOC" : "VF");

    bt_send(bt, buf);
}

/* ------------------------------------------------------------------ */
/*  NAMEPLATE=volt,curr,freq,rpm,acc,fmin[,poles]                     */
/* ------------------------------------------------------------------ */
static void apply_nameplate(BtMotor_t *bt, char *args)
{
    char *tok; int idx = 0;
    int volt = 0, freq = 0, rpm = 0, acc = 1, fmin = MOTOR_MIN_SPEED_HZ, poles = 0;
    int curr_x10 = 0;

    tok = strtok(args, ",");
    while (tok && idx < 7)
    {
        switch (idx)
        {
            case 0: volt     = parse_int(tok); break;
            case 1: curr_x10 = parse_x10(tok); break;
            case 2: freq     = parse_int(tok); break;
            case 3: rpm      = parse_int(tok); break;
            case 4: acc      = parse_int(tok); break;
            case 5: fmin     = parse_int(tok); break;
            case 6: poles    = parse_int(tok); break;
        }
        idx++;
        tok = strtok(NULL, ",");
    }
    if (acc < 1)  acc = 1;
    if (freq < 1) freq = 50;

    int fmax  = (int)(freq * 1.15f + 0.5f);          /* fmax ~ freq*1.15  */
    int ramp  = freq / acc;                          /* Hz/s = freq/acc   */
    int oc10  = (int)(curr_x10 * 1.05f + 0.5f);      /* overcurrent (x10) */

    bt->fmin_hz = clampi(fmin, MOTOR_MIN_SPEED_HZ, MOTOR_MAX_SPEED_HZ);
    bt->fmax_hz = clampi(fmax, bt->fmin_hz,        MOTOR_MAX_SPEED_HZ);
    bt->oc_x10  = oc10;
    if (ramp < 1) ramp = 1;

    Motor_SetRampRate(bt->motor, ramp);
    Motor_ConfigNameplate(bt->motor, (float)volt, (float)curr_x10 * 0.1f,
                          (float)freq, (float)rpm);
    if (poles >= 2) Motor_SetPoles(bt->motor, poles);

    bt_send_ok(bt, "NAMEPLATE");
    bt_send(bt, "LOG:NP_OK\r\n");
}

/* ------------------------------------------------------------------ */
/*  Command dispatch                                                  */
/* ------------------------------------------------------------------ */
static void process_line(BtMotor_t *bt, char *l)
{
    Motor_Handle_t *m = bt->motor;

    if (strcmp(l, "START") == 0)
    {
        int hz = clampi(bt->cmd_freq_hz_x10 / 10, bt->fmin_hz,
                        (bt->fmax_hz < MOTOR_MAX_SPEED_HZ ? bt->fmax_hz : MOTOR_MAX_SPEED_HZ));
        Motor_Start(m, hz, bt->reverse);
        bt_send_ok(bt, "START");
    }
    else if (strcmp(l, "STOP") == 0)
    {
        Motor_Stop(m, false);
        bt_send_ok(bt, "STOP");
    }
    else if (strcmp(l, "FWD") == 0)
    {
        bt->reverse = false;
        Motor_SetDirection(m, false);
        bt_send_ok(bt, "FWD");
    }
    else if (strcmp(l, "REV") == 0)
    {
        bt->reverse = true;
        Motor_SetDirection(m, true);
        bt_send_ok(bt, "REV");
    }
    else if (strcmp(l, "CLR_FLT") == 0)
    {
        Motor_ClearFault(m);
        bt_send_ok(bt, "CLR_FLT");
        bt_send(bt, "LOG:FAULT_CLEARED=ON\r\n");
    }
    else if (starts_with(l, "FREQ="))
    {
        int v = parse_x10(l + 5);
        v = clampi(v, bt->fmin_hz * 10,
                   (bt->fmax_hz < MOTOR_MAX_SPEED_HZ ? bt->fmax_hz : MOTOR_MAX_SPEED_HZ) * 10);
        bt->cmd_freq_hz_x10 = v;
        Motor_SetFrequency(m, v / 10);
        bt_send_ok(bt, l);
    }
    else if (starts_with(l, "RAMP="))
    {
        int n = parse_int(l + 5);
        Motor_SetRampRate(m, n);
        bt_send_ok(bt, l);
    }
    else if (starts_with(l, "TORQUE="))
    {
        int n = clampi(parse_int(l + 7), 0, 100);
        Motor_SetTorque(m, n);
        bt_send_ok(bt, l);
    }
    else if (starts_with(l, "POLES="))
    {
        int n = parse_int(l + 6);
        Motor_SetPoles(m, n);
        bt_send_ok(bt, l);
    }
    else if (starts_with(l, "HZSYS="))
    {
        int n = parse_int(l + 6);
        Motor_SetLineFreq(m, n);
        bt_send_ok(bt, l);
    }
    else if (starts_with(l, "MODE="))
    {
        Motor_SetMode(m, (strcmp(l + 5, "FOC") == 0) ? MOTOR_MODE_FOC : MOTOR_MODE_VF);
        bt_send_ok(bt, l);
    }
    else if (starts_with(l, "NAMEPLATE="))
    {
        apply_nameplate(bt, l + 10);
    }
    else
    {
        bt_send(bt, "ERROR:UNKNOWN CMD\r\n");
    }

    /* snappy UI: push a fresh telemetry frame right after each command */
    bt_send_telemetry(bt);
    bt->last_telem_ms = HAL_GetTick();
}

/* ================================================================== */
/*  Public API                                                        */
/* ================================================================== */
void BtMotor_Init(BtMotor_t *bt, Motor_Handle_t *motor, UART_HandleTypeDef *huart)
{
    memset(bt, 0, sizeof(*bt));
    bt->motor = motor;
    bt->huart = huart;

    bt->cmd_freq_hz_x10 = MOTOR_DEFAULT_RUN_HZ * 10;
    bt->fmin_hz = MOTOR_MIN_SPEED_HZ;
    bt->fmax_hz = MOTOR_MAX_SPEED_HZ;
    bt->oc_x10  = 200;
    bt->reverse = false;
    bt->telem_period_ms = BT_TELEM_PERIOD_MS;
    bt->last_telem_ms = HAL_GetTick();
    bt->line_len = 0;
    bt->line_ready = false;

    g_bt = bt;

    motor->on_start_callback            = cb_start;
    motor->on_stop_callback             = cb_stop;
    motor->on_direction_change_callback = cb_dir;
    motor->on_fault_callback            = cb_fault;

    HAL_UART_Receive_IT(bt->huart, &bt->rx_byte, 1);

    bt_send(bt, "READY\r\n");
    bt_send(bt, "LOG:SYS=ON\r\n");
}

void BtMotor_OnRxByte(BtMotor_t *bt)
{
    char c = (char)bt->rx_byte;
    
    if (c == '\n' || c == '\r') {
        if (bt->line_len > 0) {
            bt->line[bt->line_len] = '\0';
            /* نسخ آمن للسطر (هذا ضروري) */
            memcpy(bt->ready_line, bt->line, bt->line_len + 1);
            bt->line_ready = true;
            bt->line_len = 0;
        }
    } else if (bt->line_len < (BT_LINE_MAX - 1)) {
        bt->line[bt->line_len++] = c;
    } else {
        bt->line_len = 0;
    }
    
    HAL_UART_Receive_IT(bt->huart, &bt->rx_byte, 1);
}

void BtMotor_Task(BtMotor_t *bt)
{
    if (bt->line_ready)
    {
        char local[BT_LINE_MAX];
        __disable_irq();
        strcpy(local, bt->ready_line);
        bt->line_ready = false;
        __enable_irq();

        process_line(bt, local);
    }

    uint32_t now = HAL_GetTick();
    if ((now - bt->last_telem_ms) >= bt->telem_period_ms)
    {
        bt->last_telem_ms = now;
        bt_send_telemetry(bt);
    }
}
