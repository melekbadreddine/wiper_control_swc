/**********************************************************************************************************************
 * File:        cli_sim.c
 * Component:   WiperControl_SWC interactive simulator
 * Description: Terminal front-end for driving the wiper control runnable manually.
 *              Provides a mock RTE, lets the stalk position and rain intensity be
 *              changed live, executes one 10 ms cycle on demand and reports the
 *              resulting wiper speed command together with the active SWC state.
 *
 * Commands:    s <0-3>   set stalk position (0=OFF, 1=AUTO, 2=LOW, 3=HIGH)
 *              r <0-100> set rain intensity in percent
 *              t         trigger a single runnable cycle
 *              c <n>     run n consecutive cycles (<=10000)
 *              a         run one cycle for every stalk position (AUTO demo sweep)
 *              show      print the current status without running a cycle
 *              reset     return both inputs to their power-on defaults
 *              h / ?     help
 *              q         quit
 *
 * Note:        No ncurses dependency - plain stdio works over ssh, in CI logs and
 *              inside container exec, where a curses UI would need a real TTY.
 **********************************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "WiperControl.h"

/**********************************************************************************************************************
 * Mock RTE
 **********************************************************************************************************************/

static uint8 g_StalkPosition;
static uint8 g_RainIntensity;
static uint8 g_WiperSpeedCmd;

FUNC(Std_ReturnType, RTE_CODE) Rte_Read_RPort_StalkPosition_DE_StalkPosition(uint8 *const data)
{
    if (data != NULL_PTR_CHECK)
    {
        *data = g_StalkPosition;
    }
    return E_OK;
}

FUNC(Std_ReturnType, RTE_CODE) Rte_Read_RPort_RainIntensity_DE_RainIntensity(uint8 *const data)
{
    if (data != NULL_PTR_CHECK)
    {
        *data = g_RainIntensity;
    }
    return E_OK;
}

FUNC(Std_ReturnType, RTE_CODE) Rte_Write_PPort_WiperSpeedCmd_DE_WiperSpeedCmd(uint8 data)
{
    g_WiperSpeedCmd = data;
    return E_OK;
}

/**********************************************************************************************************************
 * Simulator state and helpers
 **********************************************************************************************************************/

static unsigned long g_CycleCount;

static const char *StalkName(uint8 stalk)
{
    switch (stalk)
    {
        case STALK_POSITION_OFF:  return "OFF";
        case STALK_POSITION_AUTO: return "AUTO";
        case STALK_POSITION_LOW:  return "LOW";
        case STALK_POSITION_HIGH: return "HIGH";
        default:                  return "INVALID";
    }
}

static const char *SpeedName(uint8 speed)
{
    switch (speed)
    {
        case WIPER_SPEED_OFF:  return "OFF ";
        case WIPER_SPEED_LOW:  return "LOW ";
        case WIPER_SPEED_HIGH: return "HIGH";
        default:               return "?????";
    }
}

static const char *StateName(uint8 state)
{
    switch (state)
    {
        case WC_STATE_OFF:  return "OFF ";
        case WC_STATE_AUTO: return "AUTO";
        case WC_STATE_LOW:  return "LOW ";
        case WC_STATE_HIGH: return "HIGH";
        default:            return "?????";
    }
}

/* Runs one 10 ms cycle and refreshes the captured output. */
static void RunOneCycle(void)
{
    Runnable_WiperControl_10ms();
    g_CycleCount++;
}

/* Prints the full status block: inputs, output and active state. */
static void ShowStatus(void)
{
    printf("  ------------------ status ------------------\n");
    printf("   Stalk Position : %u (%s)\n",
           (unsigned)g_StalkPosition, StalkName(g_StalkPosition));
    printf("   Rain Intensity : %u%%\n", (unsigned)g_RainIntensity);
    printf("   Wiper Speed Cmd: %u (%s)\n",
           (unsigned)g_WiperSpeedCmd, SpeedName(g_WiperSpeedCmd));
    printf("   Active State   : %u (%s)\n",
           (unsigned)Wc_GetState(), StateName(Wc_GetState()));
    printf("   Cycles Run     : %lu\n", g_CycleCount);
    printf("  -------------------------------------------\n");
}

/* Runs one cycle and shows the result. */
static void Step(void)
{
    RunOneCycle();
    ShowStatus();
}

static void PrintHelp(void)
{
    printf("\n  Commands:\n");
    printf("    s <0-3>    set stalk position   (0=OFF, 1=AUTO, 2=LOW, 3=HIGH)\n");
    printf("    r <0-100>  set rain intensity   in percent\n");
    printf("    t          trigger one 10 ms runnable cycle and show the result\n");
    printf("    c <n>      run n consecutive cycles (max 10000)\n");
    printf("    a          AUTO sweep: one cycle per stalk position at the current rain\n");
    printf("    show       print current status without running a cycle\n");
    printf("    reset      restore power-on defaults (stalk=OFF, rain=0%%)\n");
    printf("    h / ?      show this help\n");
    printf("    q          quit\n");
    printf("\n  The runnable also runs once automatically after every 's' or 'r'.\n\n");
}

/* AUTO demo sweep: shows how the output follows the stalk in one rain level. */
static void AutoSweep(void)
{
    static const uint8 stalks[] = { STALK_POSITION_OFF, STALK_POSITION_AUTO,
                                    STALK_POSITION_LOW, STALK_POSITION_HIGH };
    unsigned i;

    printf("\n  AUTO sweep at rain = %u%%\n", (unsigned)g_RainIntensity);
    printf("  ------------------ status ------------------\n");
    printf("   %-10s | %-12s | %-12s\n", "Stalk", "Speed Cmd", "State");
    printf("  -----------------------------------------------\n");

    for (i = 0U; i < (sizeof(stalks) / sizeof(stalks[0])); i++)
    {
        g_StalkPosition = stalks[i];
        RunOneCycle();
        printf("   %-10s | %-12s | %-12s\n",
               StalkName(stalks[i]),
               SpeedName(g_WiperSpeedCmd),
               StateName(Wc_GetState()));
    }
    printf("  -----------------------------------------------\n\n");
}

/* Parses an unsigned value from the tail of the input line. Returns 1 on success. */
static int ParseNumber(const char *text, unsigned long *out)
{
    char *end = NULL;
    unsigned long value;

    while ((*text != '\0') && isspace((unsigned char)*text))
    {
        text++;
    }
    if (*text == '\0')
    {
        return 0;
    }

    value = strtoul(text, &end, 10);
    if ((end == text) || (end == NULL_PTR_CHECK))
    {
        return 0;
    }
    /* Reject trailing garbage so "50abc" is not silently accepted as 50. */
    while ((*end != '\0') && isspace((unsigned char)*end))
    {
        end++;
    }
    if (*end != '\0')
    {
        return 0;
    }

    *out = value;
    return 1;
}

/**********************************************************************************************************************
 * main
 **********************************************************************************************************************/
int main(void)
{
    char line[128];
    int  running = 1;

    /* Power-on defaults. */
    g_StalkPosition = STALK_POSITION_OFF;
    g_RainIntensity = 0U;
    g_WiperSpeedCmd = WIPER_SPEED_OFF;

    printf("======================================================================\n");
    printf(" WiperControl_SWC interactive simulator\n");
    printf(" Runnable: Runnable_WiperControl_10ms (10 ms period)\n");
    printf("======================================================================\n");
    printf(" Stalk encoding: 0=OFF 1=AUTO 2=LOW 3=HIGH   (project assumption)\n");
    printf(" Rain thresholds: <%u%% off, %u..%u%% low, >%u%% high\n",
           (unsigned)WC_RAIN_AUTO_LOWER_THRESHOLD,
           (unsigned)WC_RAIN_AUTO_LOWER_THRESHOLD,
           (unsigned)WC_RAIN_AUTO_UPPER_THRESHOLD,
           (unsigned)WC_RAIN_AUTO_UPPER_THRESHOLD);
    printf(" Type 'h' for help, 'q' to quit.\n");

    /* Initial status only - no cycle executed yet. */
    ShowStatus();

    while (running)
    {
        printf("\n> ");
        fflush(stdout);

        if (fgets(line, (int)sizeof(line), stdin) == NULL_PTR_CHECK)
        {
            /* EOF (Ctrl-D) or closed pipe. */
            printf("\n");
            break;
        }

        /* Strip the newline. */
        {
            size_t len = strlen(line);
            while ((len > 0U) && ((line[len - 1U] == '\n') || (line[len - 1U] == '\r')))
            {
                line[len - 1U] = '\0';
                len--;
            }
        }

        /* Word-form commands first: several of them share a first letter with
         * the single-key commands ("show" vs "s", "reset" vs "r"). */
        if ((strcmp(line, "show") == 0) || (strcmp(line, "status") == 0))
        {
            ShowStatus();
            continue;
        }
        if (strcmp(line, "reset") == 0)
        {
            g_StalkPosition = STALK_POSITION_OFF;
            g_RainIntensity = 0U;
            printf("  Inputs reset to power-on defaults.\n");
            Step();
            continue;
        }
        if (strcmp(line, "quit") == 0)
        {
            line[0] = 'q';
        }
        else if (strcmp(line, "exit") == 0)
        {
            line[0] = 'q';
        }
        else if (strcmp(line, "help") == 0)
        {
            line[0] = 'h';
        }

        switch (line[0])
        {
            case 's':
            case 'S':
            {
                unsigned long value;
                if (!ParseNumber(&line[1], &value) || (value > 3UL))
                {
                    printf("  ERROR: stalk must be 0..3 (0=OFF, 1=AUTO, 2=LOW, 3=HIGH)\n");
                    break;
                }
                g_StalkPosition = (uint8)value;
                printf("  Stalk set to %s\n", StalkName(g_StalkPosition));
                Step();
                break;
            }

            case 'r':
            case 'R':
            {
                unsigned long value;
                if (!ParseNumber(&line[1], &value) || (value > 100UL))
                {
                    printf("  ERROR: rain intensity must be 0..100 percent\n");
                    break;
                }
                g_RainIntensity = (uint8)value;
                printf("  Rain set to %u%%\n", (unsigned)g_RainIntensity);
                Step();
                break;
            }

            case 't':
            case 'T':
                printf("  Running one cycle...\n");
                Step();
                break;

            case 'c':
            case 'C':
            {
                unsigned long value, i;
                if (!ParseNumber(&line[1], &value) || (value == 0UL) || (value > 10000UL))
                {
                    printf("  ERROR: cycle count must be 1..10000\n");
                    break;
                }
                for (i = 0UL; i < value; i++)
                {
                    RunOneCycle();
                }
                printf("  Ran %lu cycles.\n", value);
                ShowStatus();
                break;
            }

            case 'a':
            case 'A':
                AutoSweep();
                break;

            case 'd':
            case 'D':
                ShowStatus();
                break;

            case 'f':
            case 'F':
                g_StalkPosition = STALK_POSITION_OFF;
                g_RainIntensity = 0U;
                printf("  Inputs reset to power-on defaults.\n");
                Step();
                break;

            case 'h':
            case 'H':
            case '?':
                PrintHelp();
                break;

            case 'q':
            case 'Q':
                running = 0;
                break;

            case '\0':
                /* Blank line - ignore. */
                break;

            default:
                printf("  ERROR: unknown command '%c'. Type 'h' for help.\n", line[0]);
                break;
        }
    }

    printf("\n  Simulator stopped after %lu cycles. Bye.\n", g_CycleCount);
    return 0;
}
