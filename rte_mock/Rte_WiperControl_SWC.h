/**********************************************************************************************************************
 * File:    Rte_WiperControl_SWC.h
 * Purpose: Host-side stand-in for the RTE API header generated from
 *          WiperControl_SWC.arxml. Signatures match the ARXML port/data-element
 *          definition so the SWC source compiles unmodified against the real RTE.
 *
 *          Data consistency is EXPLICIT on all port points, which is why the
 *          generated names carry the _DE_<DataElement> suffix.
 **********************************************************************************************************************/

#ifndef RTE_WIPERCONTROL_SWC_H
#define RTE_WIPERCONTROL_SWC_H

#include "Std_Types.h"
#include "Rte_Type.h"

#ifdef __cplusplus
extern "C" {
#endif

/* RPort_StalkPosition / StalkPosition */
FUNC(Std_ReturnType, RTE_CODE) Rte_Read_RPort_StalkPosition_DE_StalkPosition(uint8 *const data);

/* RPort_RainIntensity / RainIntensity */
FUNC(Std_ReturnType, RTE_CODE) Rte_Read_RPort_RainIntensity_DE_RainIntensity(uint8 *const data);

/* PPort_WiperSpeedCmd / WiperSpeedCmd */
FUNC(Std_ReturnType, RTE_CODE) Rte_Write_PPort_WiperSpeedCmd_DE_WiperSpeedCmd(uint8 data);

#ifdef __cplusplus
}
#endif

#endif /* RTE_WIPERCONTROL_SWC_H */
