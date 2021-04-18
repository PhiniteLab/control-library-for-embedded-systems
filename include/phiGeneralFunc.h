#ifndef __PHI_GENERAL_FUNC_H__
#define __PHI_GENERAL_FUNC_H__

#ifdef __cplusplus
extern "C"
{
#endif

    ////////////////////////////////////////////////////////////
    // including some libraries for using input/output functions

// standard library usage
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "time.h"

// phinite Library usage
#include "phiSystemDynamic.h"
#include "phiMathLib.h"

    // including some libraries for using input/output functions
    ////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////
    // type decleration for usage of general functions

    void delay(int milliseconds); /*
    * standard delay function in milliseconds
    * output -> return nothing
    * input  -> milliseconds
    * */

    // type decleration for usage of general functions
    ////////////////////////////////////////////////////////////

#ifdef __cplusplus
}
#endif

#endif