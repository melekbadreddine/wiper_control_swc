/**********************************************************************************************************************
 * File:    Std_Types.h
 * Purpose: Host-side stand-in for the AUTOSAR standard types header.
 *          The real Std_Types.h is supplied by the MCAL/BSW on target; this mock only
 *          exists so the SWC can be compiled and simulated on a workstation.
 **********************************************************************************************************************/

#ifndef STD_TYPES_H
#define STD_TYPES_H

typedef unsigned char  uint8;
typedef signed char    sint8;
typedef unsigned short uint16;
typedef unsigned int   uint32;

typedef unsigned char  Std_ReturnType;
typedef uint32         Std_VersionInfoType;

#define E_OK      ((Std_ReturnType)0U)
#define E_NOT_OK  ((Std_ReturnType)1U)

#define RTE_CODE
#define RTE_APPL_DATA

/* AUTOSAR null pointer literal (normally from Compilers.h / Platform.h). */
#ifndef NULL_PTR_CHECK
#define NULL_PTR_CHECK ((void *)0)
#endif

/* AUTOSAR memory / class macros, normally provided by Compiler.h + MemMap.h */
#define FUNC(rettype, memclass) rettype
#define P2FUNC(rettype, ptrclass, memclass) rettype
#define FUNC_P2VAR(rettype, ptrclass, memclass, ptr) rettype ptr

#endif /* STD_TYPES_H */
