/**********************************************************************************************************************
 * File:        WiperControl.h
 * Component:   WiperControl_SWC (AUTOSAR Classic 4.4 Atomic Application SWC)
 * Description: Public interface of the Automatic Wiper Control runnables.
 * Language:    ANSI-C (ISO/IEC 9899:1999)
 **********************************************************************************************************************/

#ifndef WIPERCONTROL_H
#define WIPERCONTROL_H

/**********************************************************************************************************************
 * Standard AUTOSAR / RTE includes
 **********************************************************************************************************************/
#include "Std_Types.h"     /* AUTOSAR standard type definitions */
#include "Rte_Type.h"      /* RTE generated type definitions  */
#include "Rte_WiperControl_SWC.h" /* RTE generated prototypes for this SWC */

/**********************************************************************************************************************
 * Memory / code class macros
 **********************************************************************************************************************/
#define WIPERCONTROL_CODE
#define WIPERCONTROL_APPL_DATA
#define WIPERCONTROL_CONST

/**********************************************************************************************************************
 * Wiper speed command values (sent on PPort_WiperSpeedCmd)
 **********************************************************************************************************************/
#define WIPER_SPEED_OFF    ((uint8)0U)  /* Wipers parked / not moving */
#define WIPER_SPEED_LOW    ((uint8)1U)  /* Low  wipe speed             */
#define WIPER_SPEED_HIGH   ((uint8)2U)  /* High wipe speed             */

/**********************************************************************************************************************
 * Stalk position values (read from RPort_StalkPosition)
 *
 * NOTE: This mapping is a project-level assumption. It is the natural mapping of the
 *       controller model but it is NOT defined by the AUTOSAR standard. Align it with
 *       the actual signal definition of the receiving ECU / signal matrix.
 **********************************************************************************************************************/
#define STALK_POSITION_OFF  ((uint8)0U)  /* Driver selected OFF  */
#define STALK_POSITION_AUTO ((uint8)1U)  /* Driver selected AUTO */
#define STALK_POSITION_LOW  ((uint8)2U)  /* Driver selected LOW  */
#define STALK_POSITION_HIGH ((uint8)3U)  /* Driver selected HIGH */

/**********************************************************************************************************************
 * Internal states of the wiper control state machine
 **********************************************************************************************************************/
#define WC_STATE_OFF       ((uint8)0U)   /* Wipers off                        */
#define WC_STATE_AUTO      ((uint8)1U)   /* Automatic mode, rain-driven       */
#define WC_STATE_LOW       ((uint8)2U)   /* Forced low speed                  */
#define WC_STATE_HIGH      ((uint8)3U)   /* Forced high speed                 */

/**********************************************************************************************************************
 * Rain intensity thresholds used in AUTO (value in percent, 0..100)
 **********************************************************************************************************************/
#define WC_RAIN_AUTO_LOWER_THRESHOLD  ((uint8)10U) /* < 10%  -> wipers off    */
#define WC_RAIN_AUTO_UPPER_THRESHOLD  ((uint8)60U) /* > 60%  -> high speed     */

/**********************************************************************************************************************
 * Exported function prototypes of the Application SWC
 **********************************************************************************************************************/

/*! \brief  Periodic runnable, executed every 10 ms via TimingEvent.
 *         Reads stalk position and rain intensity, computes the wiper speed
 *         command and writes it to the sender port. */
FUNC(void, WIPERCONTROL_CODE) Runnable_WiperControl_10ms(void);

/*! \brief  Read-only access to the current state machine state, for diagnostics,
 *         HIL observation and unit testing.
 *         \return One of WC_STATE_OFF / WC_STATE_AUTO / WC_STATE_LOW / WC_STATE_HIGH */
FUNC(uint8, WIPERCONTROL_CODE) Wc_GetState(void);

#endif /* WIPERCONTROL_H */
