#ifndef __PHI_CONTOL_LIBRARY_H__
#define __PHI_CONTOL_LIBRARY_H__

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
#include "math.h"

// phinite Library usage
#include "phiSystemDynamicSetting.h"

    // including some libraries for using input/output functions
    ////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////
    // type decleration for usage of contoller dynamic solver

    /*
    * This structure represents the basic variables in terms of
    * control dynamics parameters.
    * 
    * There are only pointer types of object, the rest of them is
    * created in the control/solver blocks!
    * 
    */

    typedef struct control_Dynamics_Parameter
    {
        ///////////////////////////////////////////////////
        ///////////////////////////////////////////////////
        float **errorMatrices; /*
                                * storing the error values in time 
                                * (this is generally assigned to 3x1 vector) 
                                * */

        ///////////////////////////////////////////////////
        ///////////////////////////////////////////////////

        ///////////////////////////////////////////////////
        ///////////////////////////////////////////////////

        float inputNow; /*
                        * storing input value at the present state
                        * */
        float inputPre; /*
                        * storing input value at the previous state
                        * */

        float errorNow;    /*
                        * storing error value at the present state
                        * */
        float errorPre;    /*
                        * storing error value at the previous state
                        * */
        float errorPrePre; /*
                        * storing error value at the previous previous state
                        * */

        ///////////////////////////////////////////////////
        ///////////////////////////////////////////////////

        ///////////////////////////////////////////////////
        ///////////////////////////////////////////////////

        float phiRoots_1; /*
                        * storing root value of controller
                        * this root should be given between 0 to 1 for discrete controller
                        * */
        float phiRoots_2; /*
                        * storing root value of controller
                        *this root should be given between 0 to 1 for discrete controller
                        * */
        float phiRoots_3; /*
                        * storing root value of controller
                        *this root should be given between 0 to 1 for discrete controller
                        * */

        float Kp; /*
                  * storing discrete proportional coefficient 
                  * */
        float Kd; /*
                  * storing discrete derivative coefficient 
                  * */
        float Ki; /*
                  * storing discrete integral coefficient 
                  * */
        float Ae; /*
                  * storing discrete second order coefficient 
                  * */

        ///////////////////////////////////////////////////
        ///////////////////////////////////////////////////

        ///////////////////////////////////////////////////
        ///////////////////////////////////////////////////

        float samplingPeriod; /*
                  * storing embeded system sampling period 
                  * */
        int stepNumber;       /*
                  * storing abstract discrete value for specific sampling rate
                  * */
        int stepNumberPre;    /*
                  * storing abstract discrete value for specific sampling rate
                  * */

        ///////////////////////////////////////////////////
        ///////////////////////////////////////////////////

    } controlDynamicsParameter;

    typedef controlDynamicsParameter *controlDynamicsParameterPtr;

    // type decleration for usage of contoller dynamic solver
    ////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////
    // function prototype decleration

    void setPIDRoots(controlDynamicsParameterPtr ptrCont, float root1, float root2,
                     float root3, float samplingPeriod); /*
    * creating proper roots assignments
    * output -> returns nothing
    * input -> address of control parameter
    *          value of root1
    *          value of root2
    *          value of root3
    *          value of sampling period
    */

    void setPIDCoefficients(controlDynamicsParameterPtr ptrCont, float Kp, float Kd,
                            float Ki, float samplingPeriod); /*
    * creating proper roots assignments
    * output -> returns nothing
    * input -> address of control parameter
    *          value of Kp
    *          value of Kd
    *          value of Ki
    *          value of sampling period
    */

    float generateOnlyPIDOutput(controlDynamicsParameterPtr ptrCont,
                                systemDynamicParameterPtr ptrSys,
                                float referenceSignal,
                                float forwardReferenceSignal,
                                int selectControlState,
                                float simulationTime,
                                float min, float max); /*
    * generating proper controller input via discrete PID design
    * output -> returns the value of control input
    * input -> address of control parameter
    *          address of system parameter
    *          value of reference signal
    *          value of next reference signal
    *          state of controlled state
    *          value of simulation time
    *          value of minimum PID controller output
    *          value of maximum PID controller output
    */

    void initializeControllerDesign(controlDynamicsParameterPtr ptrCont); /*
    * initialize controller design
    * output -> returns nothing
    * input -> address of control parameter
    */

    /////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////
    int findingInputState(systemDynamicParameterPtr ptrSys); /*
    * finding input state
    * output -> input state of controller
    * input -> address of system dynamic parameter
    */

    /////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////

    // function prototype decleration
    ////////////////////////////////////////////////////////////

#ifdef __cplusplus
}
#endif

#endif