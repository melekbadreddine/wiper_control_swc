/**********************************************************************************************************************
 * File:        WiperControl.c
 * Component:   WiperControl_SWC (AUTOSAR Classic 4.4 Atomic Application SWC)
 * Description: Implementation of the automatic wiper control logic as a plain-C
 *              state machine (Stateflow-equivalent model).
 * Model:       Stateflow chart "WiperControl" - states OFF / AUTO / LOW / HIGH
 * Language:    ANSI-C (ISO/IEC 9899:1999)
 **********************************************************************************************************************/

#include "WiperControl.h"

/**********************************************************************************************************************
 * Local variables
 **********************************************************************************************************************/

/*! \brief Current state of the wiper control state machine. */
static uint8 Wc_State = WC_STATE_OFF;

/**********************************************************************************************************************
 * Local function prototypes
 **********************************************************************************************************************/

static FUNC(void, WIPERCONTROL_CODE) Wc_SetState(uint8 NewState);
static FUNC(void, WIPERCONTROL_CODE) Wc_ProcessStalkPosition(uint8 StalkPosition);
static FUNC(uint8, WIPERCONTROL_CODE) Wc_GetSpeedFromAuto(uint8 RainIntensity);
static FUNC(uint8, WIPERCONTROL_CODE) Wc_GetSpeedCommand(uint8 State, uint8 RainIntensity);

/**********************************************************************************************************************
 * Global functions
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Function:      Runnable_WiperControl_10ms
 * Description:   Periodic runnable triggered every 10 ms. Executes one cycle of the
 *                wiper control state machine.
 **********************************************************************************************************************/
FUNC(void, WIPERCONTROL_CODE) Runnable_WiperControl_10ms(void)
{
    uint8 StalkPosition = STALK_POSITION_OFF;
    uint8 RainIntensity  = 0U;
    uint8 SpeedCommand   = WIPER_SPEED_OFF;

    /* Read the inputs from the receiver ports. Fail-safe defaults are used so that
     * an unconnected port never leads to an undefined behaviour. */
    (void)Rte_Read_RPort_StalkPosition_DE_StalkPosition(&StalkPosition);
    (void)Rte_Read_RPort_RainIntensity_DE_RainIntensity(&RainIntensity);

    /* --- Transition logic ------------------------------------------------ */
    Wc_ProcessStalkPosition(StalkPosition);

    /* --- Per-state output computation ------------------------------------ */
    SpeedCommand = Wc_GetSpeedCommand(Wc_State, RainIntensity);

    /* Write the wiper speed command to the sender port. */
    (void)Rte_Write_PPort_WiperSpeedCmd_DE_WiperSpeedCmd(SpeedCommand);
}

/**********************************************************************************************************************
 * Function:      Wc_GetState
 * Description:   Returns the currently active state of the state machine. Read-only;
 *                intended for diagnostics and testing.
 * Returns:       Current state (WC_STATE_*)
 **********************************************************************************************************************/
FUNC(uint8, WIPERCONTROL_CODE) Wc_GetState(void)
{
    return Wc_State;
}

/**********************************************************************************************************************
 * Local functions
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Function:      Wc_SetState
 * Description:   Sets the state machine to the requested state.
 * Parameters:    NewState - state to be entered
 **********************************************************************************************************************/
static FUNC(void, WIPERCONTROL_CODE) Wc_SetState(uint8 NewState)
{
    Wc_State = NewState;
}

/**********************************************************************************************************************
 * Function:      Wc_ProcessStalkPosition
 * Description:   Evaluates the stalk position and performs the state transitions that
 *                originate from it. Any invalid stalk value is treated as OFF.
 * Parameters:    StalkPosition - current stalk position (uint8)
 **********************************************************************************************************************/
static FUNC(void, WIPERCONTROL_CODE) Wc_ProcessStalkPosition(uint8 StalkPosition)
{
    switch (StalkPosition)
    {
        case STALK_POSITION_OFF:
            /* Driver selected OFF -> always take effect. */
            Wc_SetState(WC_STATE_OFF);
            break;

        case STALK_POSITION_AUTO:
            /* Driver selected AUTO -> only enter AUTO if not already forced by an
             * explicit LOW/HIGH command, so the driver can override the automation. */
            if ((Wc_State == WC_STATE_OFF) ||
                (Wc_State == WC_STATE_AUTO))
            {
                Wc_SetState(WC_STATE_AUTO);
            }
            else
            {
                /* WC_STATE_LOW / WC_STATE_HIGH are latched until the stalk changes. */
            }
            break;

        case STALK_POSITION_LOW:
            Wc_SetState(WC_STATE_LOW);
            break;

        case STALK_POSITION_HIGH:
            Wc_SetState(WC_STATE_HIGH);
            break;

        default:
            /* Undefined stalk value -> safe state. */
            Wc_SetState(WC_STATE_OFF);
            break;
    }
}

/**********************************************************************************************************************
 * Function:      Wc_GetSpeedFromAuto
 * Description:   Maps the rain intensity to the AUTO wipe speed.
 *                  < 10%          -> 0 (OFF)
 *                  10% .. 60%    -> 1 (LOW)
 *                  > 60%          -> 2 (HIGH)
 * Parameters:    RainIntensity - rain intensity in percent (0..100)
 * Returns:       Wiper speed command (0, 1 or 2)
 **********************************************************************************************************************/
static FUNC(uint8, WIPERCONTROL_CODE) Wc_GetSpeedFromAuto(uint8 RainIntensity)
{
    uint8 Speed = WIPER_SPEED_OFF;

    if (RainIntensity < WC_RAIN_AUTO_LOWER_THRESHOLD)
    {
        /* Dry or almost dry -> wipers off. */
        Speed = WIPER_SPEED_OFF;
    }
    else if (RainIntensity > WC_RAIN_AUTO_UPPER_THRESHOLD)
    {
        /* Heavy rain -> high wipe speed. */
        Speed = WIPER_SPEED_HIGH;
    }
    else
    {
        /* Light to moderate rain -> low wipe speed. */
        Speed = WIPER_SPEED_LOW;
    }

    return Speed;
}

/**********************************************************************************************************************
 * Function:      Wc_GetSpeedCommand
 * Description:   Returns the wipe speed command belonging to the current state.
 *                This is the per-state "do" action of the state machine.
 * Parameters:    State         - current state (uint8)
 *                RainIntensity - rain intensity in percent (0..100), only used in AUTO
 * Returns:       Wiper speed command
 **********************************************************************************************************************/
static FUNC(uint8, WIPERCONTROL_CODE) Wc_GetSpeedCommand(uint8 State, uint8 RainIntensity)
{
    uint8 Speed = WIPER_SPEED_OFF;

    switch (State)
    {
        case WC_STATE_OFF:
            Speed = WIPER_SPEED_OFF;
            break;

        case WC_STATE_AUTO:
            /* In AUTO the speed is a pure function of the current rain intensity. */
            Speed = Wc_GetSpeedFromAuto(RainIntensity);
            break;

        case WC_STATE_LOW:
            Speed = WIPER_SPEED_LOW;
            break;

        case WC_STATE_HIGH:
            Speed = WIPER_SPEED_HIGH;
            break;

        default:
            Speed = WIPER_SPEED_OFF;
            break;
    }

    return Speed;
}
