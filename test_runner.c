/**********************************************************************************************************************
 * File:        test_runner.c
 * Component:   WiperControl_SWC host test bench
 * Description: Non-interactive regression harness for the automatic wiper control
 *              runnable. Provides the mock RTE API, drives the runnable through
 *              scripted scenarios and checks the resulting wiper speed command
 *              against an independent reference model.
 *
 * Usage:       ./wiper_sim            run all scenarios
 * Exit code:   0 = all scenarios passed, 1 = at least one failure
 *
 * Note:        The scenarios encode the behavioural contract documented in
 *              WiperControl.h. In particular the stalk encoding
 *              (0=OFF,1=AUTO,2=LOW,3=HIGH) is a project assumption, not a
 *              standard-mandated mapping - see WiperControl.h.
 **********************************************************************************************************************/

#include <stdio.h>
#include <string.h>

#include "WiperControl.h"

/**********************************************************************************************************************
 * Mock RTE - transport for the simulation signals
 **********************************************************************************************************************/

/* Values the mocked ports present to the SWC on the next runnable invocation. */
static uint8 g_StalkPosition;
static uint8 g_RainIntensity;

/* Value captured by the last Rte_Write call - i.e. the SWC's output. */
static uint8 g_WiperSpeedCmd;

/* Error injection: when non-zero, the corresponding read returns E_NOT_OK and
 * leaves the caller's variable at its initialised default. */
static Std_ReturnType g_StalkReadResult;
static Std_ReturnType g_RainReadResult;

/* Write capture instrumentation. */
static uint8       g_LastWriteValue;
static uint8       g_WriteCount;

FUNC(Std_ReturnType, RTE_CODE) Rte_Read_RPort_StalkPosition_DE_StalkPosition(uint8 *const data)
{
    if (g_StalkReadResult != E_OK)
    {
        return E_NOT_OK;
    }
    if (data != NULL_PTR_CHECK)
    {
        *data = g_StalkPosition;
    }
    return E_OK;
}

FUNC(Std_ReturnType, RTE_CODE) Rte_Read_RPort_RainIntensity_DE_RainIntensity(uint8 *const data)
{
    if (g_RainReadResult != E_OK)
    {
        return E_NOT_OK;
    }
    if (data != NULL_PTR_CHECK)
    {
        *data = g_RainIntensity;
    }
    return E_OK;
}

FUNC(Std_ReturnType, RTE_CODE) Rte_Write_PPort_WiperSpeedCmd_DE_WiperSpeedCmd(uint8 data)
{
    g_LastWriteValue = data;
    g_WriteCount++;
    g_WiperSpeedCmd  = data;
    return E_OK;
}

/**********************************************************************************************************************
 * Test infrastructure
 **********************************************************************************************************************/

static uint32 g_TestCount;
static uint32 g_PassCount;
static uint32 g_FailCount;

static void ResetMocks(void)
{
    g_StalkPosition    = STALK_POSITION_OFF;
    g_RainIntensity    = 0U;
    g_WiperSpeedCmd    = 0xFFU;   /* invalid - "no write happened yet" */
    g_StalkReadResult  = E_OK;
    g_RainReadResult   = E_OK;
    g_LastWriteValue   = 0xFFU;
    g_WriteCount       = 0U;
}

/* Runs one 10 ms cycle with the given inputs and returns the written speed. */
static uint8 RunCycle(uint8 stalk, uint8 rain)
{
    g_StalkPosition = stalk;
    g_RainIntensity = rain;

    g_LastWriteValue = 0xFFU;
    g_WriteCount     = 0U;

    Runnable_WiperControl_10ms();

    /* The runnable must write exactly one value per cycle. */
    if (g_WriteCount != 1U)
    {
        printf("      ERROR: runnable performed %u writes (expected exactly 1)\n",
               (unsigned)g_WriteCount);
        g_FailCount++;
    }

    return g_LastWriteValue;
}

static void Expect(const char *name, uint8 stalk, uint8 rain,
                   uint8 expectedSpeed, uint8 actualSpeed)
{
    g_TestCount++;
    if (actualSpeed == expectedSpeed)
    {
        g_PassCount++;
        printf("  [PASS] %-46s stalk=%u rain=%3u%% -> speed=%u\n",
               name, (unsigned)stalk, (unsigned)rain, (unsigned)actualSpeed);
    }
    else
    {
        g_FailCount++;
        printf("  [FAIL] %-46s stalk=%u rain=%3u%% -> speed=%u (expected %u)\n",
               name, (unsigned)stalk, (unsigned)rain,
               (unsigned)actualSpeed, (unsigned)expectedSpeed);
    }
}

/**********************************************************************************************************************
 * Reference model
 *
 * Deliberately written as a flat function that is independent of the SWC
 * implementation, so the test compares two separately written expressions of
 * the same requirement rather than comparing the code with itself.
 **********************************************************************************************************************/
static uint8 ReferenceSpeed(uint8 state, uint8 stalk, uint8 rain)
{
    /* Mirror of the documented transition rules, expressed directly. */
    switch (stalk)
    {
        case STALK_POSITION_OFF:  state = WC_STATE_OFF;  break;
        case STALK_POSITION_LOW:  state = WC_STATE_LOW;  break;
        case STALK_POSITION_HIGH: state = WC_STATE_HIGH; break;
        case STALK_POSITION_AUTO:
            if ((state == WC_STATE_OFF) || (state == WC_STATE_AUTO))
            {
                state = WC_STATE_AUTO;
            }
            break;
        default:                  state = WC_STATE_OFF;  break;
    }

    switch (state)
    {
        case WC_STATE_LOW:  return WIPER_SPEED_LOW;
        case WC_STATE_HIGH: return WIPER_SPEED_HIGH;
        case WC_STATE_AUTO:
            if (rain < WC_RAIN_AUTO_LOWER_THRESHOLD)      { return WIPER_SPEED_OFF;  }
            if (rain > WC_RAIN_AUTO_UPPER_THRESHOLD)      { return WIPER_SPEED_HIGH; }
            return WIPER_SPEED_LOW;
        case WC_STATE_OFF:
        default:            return WIPER_SPEED_OFF;
    }
}

/**********************************************************************************************************************
 * Scenarios
 **********************************************************************************************************************/

/* Scenario A: AUTO mode across the rain thresholds, including the exact edges. */
static void Scenario_AutoRainThresholds(void)
{
    static const uint8 rains[] = { 0U, 5U, 9U, 10U, 11U, 30U, 59U, 60U, 61U, 75U, 100U };
    uint8 i;
    uint8 actual;

    printf("\n--- Scenario A: AUTO mode rain-intensity mapping ---\n");

    for (i = 0U; i < (sizeof(rains) / sizeof(rains[0])); i++)
    {
        actual = RunCycle(STALK_POSITION_AUTO, rains[i]);
        Expect("AUTO rain mapping",
               STALK_POSITION_AUTO, rains[i],
               ReferenceSpeed(WC_STATE_OFF, STALK_POSITION_AUTO, rains[i]),
               actual);
    }
}

/* Scenario B: each stalk position forces its own speed, independent of rain. */
static void Scenario_StalkForcesState(void)
{
    static const uint8 stalks[]  = { STALK_POSITION_OFF, STALK_POSITION_LOW, STALK_POSITION_HIGH };
    static const uint8 rains[]   = { 0U, 40U, 100U };
    uint8 s, r, actual;

    printf("\n--- Scenario B: stalk-driven states override rain ---\n");

    for (s = 0U; s < (sizeof(stalks) / sizeof(stalks[0])); s++)
    {
        for (r = 0U; r < (sizeof(rains) / sizeof(rains[0])); r++)
        {
            actual = RunCycle(stalks[s], rains[r]);
            Expect("stalk overrides rain", stalks[s], rains[r],
                   ReferenceSpeed(WC_STATE_OFF, stalks[s], rains[r]), actual);
        }
    }
}

/* Scenario C: MANUAL LOW/HIGH latch and only release when the stalk moves. */
static void Scenario_ManualLatchBehaviour(void)
{
    uint8 actual;

    printf("\n--- Scenario C: manual selection latches over AUTO ---\n");

    /* Select AUTO, then force LOW, then return the stalk to AUTO. */
    (void)RunCycle(STALK_POSITION_AUTO, 80U);   /* AUTO, high rain  */
    (void)RunCycle(STALK_POSITION_HIGH, 0U);    /* driver: HIGH    */

    actual = RunCycle(STALK_POSITION_AUTO, 0U);  /* stalk back to AUTO, dry */
    Expect("HIGH latches until stalk changes", STALK_POSITION_AUTO, 0U,
           WIPER_SPEED_HIGH, actual);

    /* Driver selects OFF -> must release the latch immediately. */
    actual = RunCycle(STALK_POSITION_OFF, 100U);
    Expect("OFF releases the latch", STALK_POSITION_OFF, 100U,
           WIPER_SPEED_OFF, actual);

    /* AUTO is re-enterable from OFF. */
    actual = RunCycle(STALK_POSITION_AUTO, 80U);
    Expect("AUTO re-entered from OFF", STALK_POSITION_AUTO, 80U,
           WIPER_SPEED_HIGH, actual);
}

/* Scenario D: invalid stalk values fail safe to OFF. */
static void Scenario_InvalidStalk(void)
{
    static const uint8 bad[] = { 4U, 5U, 42U, 200U, 255U };
    uint8 i, actual;

    printf("\n--- Scenario D: invalid stalk values fail safe ---\n");

    for (i = 0U; i < (sizeof(bad) / sizeof(bad[0])); i++)
    {
        actual = RunCycle(bad[i], 100U);
        Expect("invalid stalk -> OFF", bad[i], 100U, WIPER_SPEED_OFF, actual);
    }
}

/* Scenario E: RTE read failures must not produce a spurious output. */
static void Scenario_ReadFailureFailSafe(void)
{
    uint8 actual;

    printf("\n--- Scenario E: RTE read failure handling ---\n");

    /* Stalk read fails: the SWC must not drive the wipers. */
    ResetMocks();
    g_StalkPosition  = STALK_POSITION_HIGH;
    g_RainIntensity  = 80U;
    g_StalkReadResult = E_NOT_OK;
    g_LastWriteValue = 0xFFU;
    g_WriteCount     = 0U;
    Runnable_WiperControl_10ms();
    actual = g_LastWriteValue;

    g_TestCount++;
    if (actual == WIPER_SPEED_OFF)
    {
        g_PassCount++;
        printf("  [PASS] %-46s -> speed=%u\n",
               "stalk read failure -> wipers off", (unsigned)actual);
    }
    else
    {
        g_FailCount++;
        printf("  [FAIL] %-46s -> speed=%u (expected 0)\n",
               "stalk read failure -> wipers off", (unsigned)actual);
    }

    /* Rain read fails while AUTO is selected: last-known/defaulted rain is 0,
     * so the safe outcome is no wipe rather than full speed. */
    ResetMocks();
    g_StalkPosition = STALK_POSITION_AUTO;
    g_RainIntensity = 100U;
    g_RainReadResult = E_NOT_OK;
    g_LastWriteValue = 0xFFU;
    g_WriteCount     = 0U;
    Runnable_WiperControl_10ms();
    actual = g_LastWriteValue;

    g_TestCount++;
    if (actual == WIPER_SPEED_OFF)
    {
        g_PassCount++;
        printf("  [PASS] %-46s -> speed=%u\n",
               "rain read failure -> no wipe", (unsigned)actual);
    }
    else
    {
        g_FailCount++;
        printf("  [FAIL] %-46s -> speed=%u (expected 0)\n",
               "rain read failure -> no wipe", (unsigned)actual);
    }

    ResetMocks();
}

/* Scenario F: repeated cycles must be stable (no drift or latching artefacts). */
static void Scenario_RepeatedCyclesStable(void)
{
    uint8 i, actual, reference;
    uint8 stable = 1U;

    printf("\n--- Scenario F: 100 identical cycles are stable ---\n");

    reference = ReferenceSpeed(WC_STATE_OFF, STALK_POSITION_AUTO, 35U);

    for (i = 0U; i < 100U; i++)
    {
        actual = RunCycle(STALK_POSITION_AUTO, 35U);
        if (actual != reference)
        {
            stable = 0U;
            printf("      drift detected at cycle %u: got %u, expected %u\n",
                   (unsigned)i, (unsigned)actual, (unsigned)reference);
            break;
        }
    }

    g_TestCount++;
    if (stable)
    {
        g_PassCount++;
        printf("  [PASS] %-46s speed=%u (100/100 cycles)\n",
               "AUTO steady state has no drift", (unsigned)reference);
    }
    else
    {
        g_FailCount++;
        printf("  [FAIL] %-46s\n", "AUTO steady state has no drift");
    }
}

/* Scenario G: full stalk x rain sweep against the reference model. */
static void Scenario_FullMatrix(void)
{
    uint8 stalk, rain, actual, expected;
    uint8 mismatches = 0U;
    uint8 checked    = 0U;

    printf("\n--- Scenario G: full stalk x rain matrix vs reference model ---\n");

    for (stalk = 0U; stalk <= 4U; stalk++)          /* +1 = one invalid value */
    {
        for (rain = 0U; rain <= 100U; rain++)
        {
            actual   = RunCycle(stalk, rain);
            expected = ReferenceSpeed(WC_STATE_OFF, stalk, rain);
            checked++;
            if (actual != expected)
            {
                mismatches++;
                printf("      mismatch stalk=%u rain=%u: got %u expected %u\n",
                       (unsigned)stalk, (unsigned)rain,
                       (unsigned)actual, (unsigned)expected);
            }
        }
    }

    g_TestCount++;
    if (mismatches == 0U)
    {
        g_PassCount++;
        printf("  [PASS] %-46s %u combinations\n",
               "exhaustive matrix matches reference", (unsigned)checked);
    }
    else
    {
        g_FailCount++;
        printf("  [FAIL] %-46s %u/%u mismatched\n",
               "exhaustive matrix matches reference",
               (unsigned)mismatches, (unsigned)checked);
    }
}

/**********************************************************************************************************************
 * main
 **********************************************************************************************************************/
int main(void)
{
    printf("======================================================================\n");
    printf(" WiperControl_SWC host test bench\n");
    printf("======================================================================\n");

    Scenario_AutoRainThresholds();
    Scenario_StalkForcesState();
    Scenario_ManualLatchBehaviour();
    Scenario_InvalidStalk();
    Scenario_ReadFailureFailSafe();
    Scenario_RepeatedCyclesStable();
    Scenario_FullMatrix();

    printf("\n======================================================================\n");
    printf(" Result: %u/%u passed, %u failed\n",
           (unsigned)g_PassCount, (unsigned)g_TestCount, (unsigned)g_FailCount);
    printf("======================================================================\n");

    return (g_FailCount == 0U) ? 0 : 1;
}
