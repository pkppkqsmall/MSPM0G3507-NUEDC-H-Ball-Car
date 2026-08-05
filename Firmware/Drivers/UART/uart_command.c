#include <stdlib.h>
#include "UART.h"
#include "Button.h"
#include "motor_pwm.h"

/* AI 自动调参输出模式开关。
 * 0：普通 VOFA+ FireWater 模式，发送 speed:left,right。
 * 1：llm-pid-tuner 模式，发送 timestamp,setpoint,input,pwm,error,p,i,d。 */
static volatile uint8_t uart_llm_tuner_mode_enabled = 0U;

/* 蓝牙解析成功后，PID 参数会落到这个全局变量里。 */
volatile PID_Params g_pid_params = {0.0f, 0.0f, 0.0f};

typedef void (*UART_WheelPiGetter)(float *kp, float *ki);
typedef void (*UART_WheelPiSetter)(float kp, float ki);

/* 判断字符是否可以作为命令名前缀后的分隔符。 */
static uint8_t UART_IsCommandDelimiter(char character)
{
    return (uint8_t) ((character == '\0') ||
                      (character == '?') ||
                      (character == ' ') ||
                      (character == ',') ||
                      (character == ':') ||
                      (character == '='));
}

/* 不区分大小写地匹配命令前缀，并要求前缀后面跟着合法分隔符。 */
static uint8_t UART_HasCommandPrefix(const char *command, const char *prefix)
{
    uint16_t index = 0U;
    char command_char;
    char prefix_char;

    if ((command == NULL) || (prefix == NULL)) {
        return 0U;
    }

    while (prefix[index] != '\0') {
        command_char = command[index];
        prefix_char = prefix[index];

        if ((command_char >= 'a') && (command_char <= 'z')) {
            command_char = (char) (command_char - ('a' - 'A'));
        }

        if ((prefix_char >= 'a') && (prefix_char <= 'z')) {
            prefix_char = (char) (prefix_char - ('a' - 'A'));
        }

        if ((command[index] == '\0') || (command_char != prefix_char)) {
            return 0U;
        }

        index++;
    }

    return UART_IsCommandDelimiter(command[index]);
}

static uint8_t UART_IsPidCommand(const char *command)
{
    return UART_HasCommandPrefix(command, "PID");
}

static uint8_t UART_IsTargetCommand(const char *command)
{
    return UART_HasCommandPrefix(command, "TARGET");
}

static uint8_t UART_IsLeftPidCommand(const char *command)
{
    return UART_HasCommandPrefix(command, "LPID");
}

static uint8_t UART_IsRightPidCommand(const char *command)
{
    return UART_HasCommandPrefix(command, "RPID");
}

static uint8_t UART_IsStopCommand(const char *command)
{
    return UART_HasCommandPrefix(command, "STOP");
}

static uint8_t UART_IsStartCommand(const char *command)
{
    return UART_HasCommandPrefix(command, "START");
}

static uint8_t UART_IsEncoderQueryCommand(const char *command)
{
    return UART_HasCommandPrefix(command, "ENC");
}

static uint8_t UART_IsCarStatusCommand(const char *command)
{
    return UART_HasCommandPrefix(command, "CAR");
}

static uint8_t UART_IsAiTuneCommand(const char *command)
{
    return (uint8_t) ((UART_HasCommandPrefix(command, "AITUNE") != 0U) ||
                      (UART_HasCommandPrefix(command, "TUNE") != 0U));
}

static uint8_t UART_IsSetCommand(const char *command)
{
    return UART_HasCommandPrefix(command, "SET");
}

static uint8_t UART_IsStatusCommand(const char *command)
{
    return UART_HasCommandPrefix(command, "STATUS");
}

static const char *UART_SkipSpaces(const char *text)
{
    while ((text != NULL) && ((*text == ' ') || (*text == '\t'))) {
        text++;
    }

    return text;
}

static const char *UART_SkipCommandSeparators(const char *text)
{
    while ((text != NULL) &&
           ((*text == ' ') || (*text == '\t') ||
            (*text == ',') || (*text == ':') || (*text == '='))) {
        text++;
    }

    return text;
}

static uint8_t UART_TokenEqualsIgnoreCase(const char *text, const char *expected)
{
    char text_char;
    char expected_char;

    if ((text == NULL) || (expected == NULL)) {
        return 0U;
    }

    text = UART_SkipSpaces(text);

    while (*expected != '\0') {
        text_char = *text;
        expected_char = *expected;

        if ((text_char >= 'a') && (text_char <= 'z')) {
            text_char = (char) (text_char - ('a' - 'A'));
        }

        if ((expected_char >= 'a') && (expected_char <= 'z')) {
            expected_char = (char) (expected_char - ('a' - 'A'));
        }

        if (text_char != expected_char) {
            return 0U;
        }

        text++;
        expected++;
    }

    return UART_IsCommandDelimiter(text[0]);
}

static uint8_t UART_ParseNextFloatField(const char **text, float *value)
{
    char *end_ptr;
    const char *current;

    if ((text == NULL) || (*text == NULL) || (value == NULL)) {
        return 0U;
    }

    current = UART_SkipSpaces(*text);
    if ((*current == '\0') || (*current == ',')) {
        return 0U;
    }

    *value = strtof(current, &end_ptr);
    if (end_ptr == current) {
        return 0U;
    }

    current = UART_SkipSpaces(end_ptr);
    *text = current;
    return 1U;
}

static uint8_t UART_ParsePidValues(const char *payload, float *kp, float *ki, float *kd)
{
    const char *current = payload;

    if ((payload == NULL) || (kp == NULL) || (ki == NULL) || (kd == NULL)) {
        return 0U;
    }

    if (UART_ParseNextFloatField(&current, kp) == 0U) {
        return 0U;
    }

    if (*current != ',') {
        return 0U;
    }
    current++;

    if (UART_ParseNextFloatField(&current, ki) == 0U) {
        return 0U;
    }

    if (*current != ',') {
        return 0U;
    }
    current++;

    if (UART_ParseNextFloatField(&current, kd) == 0U) {
        return 0U;
    }

    current = UART_SkipSpaces(current);
    return (uint8_t) ((*current == '\0') ? 1U : 0U);
}

static uint8_t UART_ParseTargetValues(const char *payload, float *left_target, float *right_target)
{
    const char *current = payload;

    if ((payload == NULL) || (left_target == NULL) || (right_target == NULL)) {
        return 0U;
    }

    if (UART_ParseNextFloatField(&current, left_target) == 0U) {
        return 0U;
    }

    current = UART_SkipSpaces(current);
    if (*current == '\0') {
        *right_target = *left_target;
        return 1U;
    }

    if (*current != ',') {
        return 0U;
    }
    current++;

    if (UART_ParseNextFloatField(&current, right_target) == 0U) {
        return 0U;
    }

    current = UART_SkipSpaces(current);
    return (uint8_t) ((*current == '\0') ? 1U : 0U);
}

static uint8_t UART_ParsePiValues(const char *payload, float *kp, float *ki)
{
    const char *current = payload;

    if ((payload == NULL) || (kp == NULL) || (ki == NULL)) {
        return 0U;
    }

    if (UART_ParseNextFloatField(&current, kp) == 0U) {
        return 0U;
    }

    if (*current != ',') {
        return 0U;
    }
    current++;

    if (UART_ParseNextFloatField(&current, ki) == 0U) {
        return 0U;
    }

    current = UART_SkipSpaces(current);
    return (uint8_t) ((*current == '\0') ? 1U : 0U);
}

void PID_SetParameters(float kp, float ki, float kd)
{
    g_pid_params.kp = kp;
    g_pid_params.ki = ki;
    g_pid_params.kd = kd;
}

void PID_GetParameters(PID_Params *params)
{
    if (params == NULL) {
        return;
    }

    params->kp = g_pid_params.kp;
    params->ki = g_pid_params.ki;
    params->kd = g_pid_params.kd;
}

uint8_t UART_BluetoothHandlePidCommand(const char *command)
{
    const char *payload;
    float kp;
    float ki;
    float kd;

    /* 支持的命令格式：
     * 1. PID,1.2,0.3,0.05
     * 2. PID:1.2,0.3,0.05
     * 3. PID=1.2,0.3,0.05
     * 4. PID?  查询当前 PID 参数 */
    if (UART_IsPidCommand(command) == 0U) {
        return 0U;
    }

    payload = command + 3;

    if ((*payload == '?') && (*(payload + 1) == '\0')) {
        UART_Printf("[PID NOW] Kp=%.3f, Ki=%.3f, Kd=%.3f\r\n",
                    g_pid_params.kp,
                    g_pid_params.ki,
                    g_pid_params.kd);
        return 1U;
    }

    payload = UART_SkipCommandSeparators(payload);

    if (UART_ParsePidValues(payload, &kp, &ki, &kd) != 0U) {
        PID_SetParameters(kp, ki, kd);
        UART_Printf("[PID OK] Kp=%.3f, Ki=%.3f, Kd=%.3f\r\n", kp, ki, kd);
    } else {
        UART_SendString("[PID ERR] Use: PID,1.0,0.2,0.05\r\n");
    }

    return 1U;
}

uint8_t UART_BluetoothHandleTargetCommand(const char *command)
{
    const char *payload;
    float left_target;
    float right_target;

    if (UART_IsTargetCommand(command) == 0U) {
        return 0U;
    }

    payload = command + 6;

    if ((*payload == '?') && (*(payload + 1) == '\0')) {
        Button_GetSpeedTargets(&left_target, &right_target);
        UART_Printf("[TARGET NOW] Left=%.1f, Right=%.1f\r\n", left_target, right_target);
        return 1U;
    }

    payload = UART_SkipCommandSeparators(payload);

    if (UART_ParseTargetValues(payload, &left_target, &right_target) != 0U) {
        Button_SetSpeedTargets(left_target, right_target);
        UART_Printf("[TARGET OK] Left=%.1f, Right=%.1f\r\n", left_target, right_target);
    } else {
        UART_SendString("[TARGET ERR] Use: TARGET,24 or TARGET,24,22\r\n");
    }

    return 1U;
}

uint8_t UART_BluetoothHandleStopCommand(const char *command)
{
    if (UART_IsStopCommand(command) == 0U) {
        return 0U;
    }

    Button_StopMotorControl();
    UART_SendString("[STOP OK] Motor output disabled\r\n");
    return 1U;
}

uint8_t UART_BluetoothHandleStartCommand(const char *command)
{
    if (UART_IsStartCommand(command) == 0U) {
        return 0U;
    }

    Button_StartMotorControl();
    UART_SendString("[START OK] Motor output enabled\r\n");
    return 1U;
}

uint8_t UART_BluetoothHandleEncoderQueryCommand(const char *command)
{
    const char *payload;

    if (UART_IsEncoderQueryCommand(command) == 0U) {
        return 0U;
    }

    payload = command + 3;
    payload = UART_SkipSpaces(payload);

    if ((*payload != '\0') && ((*payload != '?') || (*(payload + 1) != '\0'))) {
        UART_SendString("[ENC ERR] Use: ENC?\r\n");
        return 1U;
    }

    /*
     * Raw 是编码器原始累计值，Fwd 是按“小车前进方向为正”修正后的值。
     * 这个命令只做查询，不改变电机启停和速度环状态，适合现场快速排查方向问题。
     */
    UART_Printf("[ENC] RawL=%ld RawR=%ld FwdL=%ld FwdR=%ld\r\n",
                (long) MotorSpeed_GetLeftEncoderTotalCount(),
                (long) MotorSpeed_GetRightEncoderTotalCount(),
                (long) MotorSpeed_GetLeftEncoderForwardCount(),
                (long) MotorSpeed_GetRightEncoderForwardCount());
    return 1U;
}

uint8_t UART_BluetoothHandleCarStatusCommand(const char *command)
{
    const char *payload;
    float left_target;
    float right_target;
    float left_rpm;
    float right_rpm;
    float left_kp;
    float left_ki;
    float right_kp;
    float right_ki;

    if (UART_IsCarStatusCommand(command) == 0U) {
        return 0U;
    }

    payload = command + 3;
    payload = UART_SkipSpaces(payload);

    if ((*payload != '\0') && ((*payload != '?') || (*(payload + 1) != '\0'))) {
        UART_SendString("[CAR ERR] Use: CAR?\r\n");
        return 1U;
    }

    Button_GetSpeedTargets(&left_target, &right_target);
    MotorSpeed_GetWheelRPM(&left_rpm, &right_rpm);
    MotorSpeedLoop_GetLeftWheelPI(&left_kp, &left_ki);
    MotorSpeedLoop_GetRightWheelPI(&right_kp, &right_ki);

    UART_Printf("[CAR] Run=%u Tune=%u Target=%.1f,%.1f Speed=%.1f,%.1f\r\n",
                Button_IsMotorControlRunning(),
                UART_LlmTunerIsEnabled(),
                left_target,
                right_target,
                left_rpm,
                right_rpm);
    UART_Printf("[CAR] LPI=%.4f,%.4f RPI=%.4f,%.4f Duty=%.1f,%.1f\r\n",
                left_kp,
                left_ki,
                right_kp,
                right_ki,
                MotorSpeedLoop_GetLeftWheelDutyPercent(),
                MotorSpeedLoop_GetRightWheelDutyPercent());
    return 1U;
}

static uint8_t UART_HandleWheelPiCommand(const char *command,
                                         const char *label,
                                         UART_WheelPiGetter get_pi,
                                         UART_WheelPiSetter set_pi,
                                         const char *usage)
{
    const char *payload;
    float kp;
    float ki;

    if ((command == NULL) || (label == NULL) ||
        (get_pi == NULL) || (set_pi == NULL) || (usage == NULL)) {
        return 0U;
    }

    payload = command + 4;
    if ((*payload == '?') && (*(payload + 1) == '\0')) {
        get_pi(&kp, &ki);
        UART_Printf("[%s NOW] Kp=%.3f, Ki=%.3f\r\n", label, kp, ki);
        return 1U;
    }

    payload = UART_SkipCommandSeparators(payload);
    if (UART_ParsePiValues(payload, &kp, &ki) != 0U) {
        set_pi(kp, ki);
        UART_Printf("[%s OK] Kp=%.3f, Ki=%.3f\r\n", label, kp, ki);
    } else {
        UART_Printf("[%s ERR] Use: %s\r\n", label, usage);
    }

    return 1U;
}

uint8_t UART_BluetoothHandleWheelTuneCommand(const char *command)
{
    /*
     * LPID/RPID 的语法完全一致，只是读写的车轮不同。
     * 共用同一个内部函数，避免以后改提示或解析规则时左右轮不一致。
     */
    if (UART_IsLeftPidCommand(command) != 0U) {
        return UART_HandleWheelPiCommand(command,
                                         "LPID",
                                         MotorSpeedLoop_GetLeftWheelPI,
                                         MotorSpeedLoop_SetLeftWheelPI,
                                         "LPID,0.12,0.05");
    }

    if (UART_IsRightPidCommand(command) != 0U) {
        return UART_HandleWheelPiCommand(command,
                                         "RPID",
                                         MotorSpeedLoop_GetRightWheelPI,
                                         MotorSpeedLoop_SetRightWheelPI,
                                         "RPID,0.12,0.05");
    }

    return 0U;
}

uint8_t UART_LlmTunerIsEnabled(void)
{
    return uart_llm_tuner_mode_enabled;
}

void UART_LlmTunerSetEnabled(uint8_t enable)
{
    uart_llm_tuner_mode_enabled = (enable != 0U) ? 1U : 0U;
}

static void UART_LlmTunerPrintStatus(void)
{
    float left_target;
    float right_target;
    float kp;
    float ki;

    Button_GetSpeedTargets(&left_target, &right_target);
    MotorSpeedLoop_GetLeftWheelPI(&kp, &ki);

    /* 这些行以 # 开头，llm-pid-tuner 会把它们当作说明文本跳过。 */
    UART_Printf("# AITUNE=%s TargetL=%.2f TargetR=%.2f P=%.4f I=%.4f D=0.0000\r\n",
                (uart_llm_tuner_mode_enabled != 0U) ? "ON" : "OFF",
                left_target,
                right_target,
                kp,
                ki);
    UART_SendString("# Format: timestamp_ms,setpoint,input,pwm,error,p,i,d\r\n");
}

/*
 * 处理 AI 自动调参相关命令。
 * AITUNE,ON  ：切换到 llm-pid-tuner CSV 输出。
 * AITUNE,OFF ：切回 VOFA+ FireWater 的 speed:left,right 输出。
 * AITUNE?    ：查询当前输出模式。
 * STATUS     ：兼容 llm-pid-tuner 启动握手，并自动打开 CSV 输出模式。
 */
uint8_t UART_BluetoothHandleLlmTunerCommand(const char *command)
{
    const char *payload;

    if (UART_IsStatusCommand(command) != 0U) {
        UART_LlmTunerSetEnabled(1U);
        UART_LlmTunerPrintStatus();
        return 1U;
    }

    if (UART_IsAiTuneCommand(command) == 0U) {
        return 0U;
    }

    if (UART_HasCommandPrefix(command, "AITUNE") != 0U) {
        payload = command + 6;
    } else {
        payload = command + 4;
    }

    payload = UART_SkipCommandSeparators(payload);

    if ((*payload == '?') && (*(payload + 1) == '\0')) {
        UART_LlmTunerPrintStatus();
        return 1U;
    }

    if ((UART_TokenEqualsIgnoreCase(payload, "ON") != 0U) ||
        (UART_TokenEqualsIgnoreCase(payload, "1") != 0U)) {
        UART_LlmTunerSetEnabled(1U);
        UART_SendString("[AITUNE OK] CSV output enabled\r\n");
        UART_LlmTunerPrintStatus();
        return 1U;
    }

    if ((UART_TokenEqualsIgnoreCase(payload, "OFF") != 0U) ||
        (UART_TokenEqualsIgnoreCase(payload, "0") != 0U)) {
        UART_LlmTunerSetEnabled(0U);
        UART_SendString("[AITUNE OK] VOFA FireWater speed output enabled\r\n");
        UART_LlmTunerPrintStatus();
        return 1U;
    }

    UART_SendString("[AITUNE ERR] Use: AITUNE,ON / AITUNE,OFF / AITUNE?\r\n");
    return 1U;
}

/*
 * 处理 llm-pid-tuner 下发的 SET 命令。
 * 支持格式: SET P:kp I:ki D:kd
 * 收到后同步更新左右轮 PI 参数（Kd 忽略）。
 */
uint8_t UART_BluetoothHandleSetCommand(const char *command)
{
    const char *payload;
    const char *p_ptr;
    const char *i_ptr;
    const char *d_ptr;
    float kp;
    float ki;
    float kd;

    if (UART_IsSetCommand(command) == 0U) {
        return 0U;
    }

    payload = command + 3;

    while ((*payload == ' ') || (*payload == '\t')) {
        payload++;
    }

    p_ptr = payload;
    while ((*p_ptr != '\0') && (*p_ptr != 'P') && (*p_ptr != 'p')) {
        p_ptr++;
    }

    if (*p_ptr == '\0') {
        UART_SendString("[SET ERR] Use: SET P:kp I:ki D:kd\r\n");
        return 1U;
    }

    kp = g_pid_params.kp;
    ki = g_pid_params.ki;
    kd = 0.0f;

    /* 解析 P: */
    {
        const char *cursor = p_ptr + 1;
        if (*cursor == ':') {
            cursor++;
        }
        if (UART_ParseNextFloatField(&cursor, &kp) == 0U) {
            UART_SendString("[SET ERR] P value parse failed\r\n");
            return 1U;
        }
    }

    /* 解析 I: */
    i_ptr = p_ptr + 1;
    while ((*i_ptr != '\0') && (*i_ptr != 'I') && (*i_ptr != 'i')) {
        i_ptr++;
    }

    if (*i_ptr != '\0') {
        const char *cursor = i_ptr + 1;
        if (*cursor == ':') {
            cursor++;
        }
        if (UART_ParseNextFloatField(&cursor, &ki) == 0U) {
            UART_SendString("[SET ERR] I value parse failed\r\n");
            return 1U;
        }
    }

    /*
     * 解析 D: 从 i_ptr 当前位置开始找，不能直接 i_ptr + 1。
     * 如果上一段没有找到 I，i_ptr 已经指向字符串结尾，+1 会越过 '\0'，
     * 继续扫描就可能读到行缓冲里上一条命令的残留内容。
     */
    d_ptr = i_ptr;
    while ((*d_ptr != '\0') && (*d_ptr != 'D') && (*d_ptr != 'd')) {
        d_ptr++;
    }

    if (*d_ptr != '\0') {
        const char *cursor = d_ptr + 1;
        if (*cursor == ':') {
            cursor++;
        }
        UART_ParseNextFloatField(&cursor, &kd);
    }

    /* 同步更新左右轮 */
    PID_SetParameters(kp, ki, kd);
    MotorSpeedLoop_SetLeftWheelPI(kp, ki);
    MotorSpeedLoop_SetRightWheelPI(kp, ki);

    UART_Printf("# PID Updated: P=%.4f I=%.4f D=%.4f\r\n", kp, ki, kd);
    return 1U;
}
