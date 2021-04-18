/* Copyright (C) 2021 Phinite Lab. All rights reserved.

 Contact information
 ------------------------------------------------------
 Developers: 
 Mehmet İşcan    : mehmetmkt@gmail.com
 Ali Burak Özden : ozdenaliburak@gmail.com 

 Company: 
 Phinite Engineering, Istanbul
 Web      :  http://phinitelab.com
 e-mail   :  info@phinitelab.com
 
 */

#ifndef __PHI_SYSTEM_DYNAMIC__
#define __PHI_SYSTEM_DYNAMIC__

#ifdef __cplusplus
extern "C"
{
#endif

    ////////////////////////////////////////////////////////////
    // including some libraries for using input/output functions
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

#include "phiSystemDynamicSetting.h"
    // including some libraries for using input/output functions
    ////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////
    // define error types

#define ALLOCATION_ERROR 1
#define INCONSISTENT_ROW_COLUMN 2
#define FILE_OPEN_ERROR 3
#define SAMPLING_RATE_ERROR 4

    // define error types
    ////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////
    // type decleration for usage of system dynamic solver

    /*
    * This structure represents the basic variables in terms of
    * system dynamics parameters.
    * 
    * There are only pointer types of object, the rest of them is
    * created in the solver blocks!
    * 
    */
    typedef struct system_Dynamics_Parameter
    {
        float **stateMatrices; /*
                                * storing the state matrices of equation of motion
                                * */

        float **inputMatrices; /*
                                * storing the input matrices of equation of motion
                                * */

        float **stateNow; /*
                           * storing the state vectors in the next value
                           * */

        float **statePre; /*
                           * storing the state vectors in the now value
                           * */

        float **inputNow; /*
                           * storing the input vectors in the now value
                           * */

        ///////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////

        int rows; /*
                   * storing row value of state space
                   * */
        int cols; /*
                   * storing column value of state space
                   * */

        float inputValue; /*
                   * storing instanteous value of input
                   * */

        int inputNumber; /*
                          * storing the number of input value
                          * */

        float xRef; /*
                  * storing the reference value
                  * */
        ///////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////

        float dt; /*
                   * storing the sampling period of the state space equation
                   * */

        clock_t tStart; /*
                   * storing the dynamic sampling period of the state space equation
                   * */

        float simulationTime; /*
                   * storing the simulation time
                   * */

        ///////////////////////////////////////////////7
        // file characteristics

        FILE *fileProcess; /*
                   * file pointer to construct the txt file object
                   * */

        const char *fileName; /*
                   * filename to give a name to the text
                   * */

        // file characteristics
        ///////////////////////////////////////////////7

    } systemDynamicsParameter; /*
                                * typedef name -> masking general template name
                                */

    typedef systemDynamicsParameter *systemDynamicParameterPtr; /*
                                * typedef name -> masking general pointer template name
                                */

    // type decleration for usage of system dynamic solver
    ////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////
    // function prototype decleration

    float **creatingEmptyStateMatrices(systemDynamicParameterPtr ptrSys); /*
    * creating dynamically empty state matrices for state space equation 
    * output -> address of matrices (float **)
    * input  -> address of systemDynamicsParameter
    * */

    float **creatingEmptyInputMatrices(systemDynamicParameterPtr ptrSys); /*
    * creating dynamically empty input matrices for state space equation
    * output -> address of matrices (float **)
    * input  -> address of systemDynamicsParameter
    * */

    void writeTheMatrices(systemDynamicParameterPtr ptrSys); /*
    * printing state and input matrices
    * output -> return nothing
    * input  -> address of systemDynamicsParameter
    * */

    float **eyeMatricesCreation(systemDynamicParameterPtr ptrSys); /*
    * creating dynamically eye matrices to produces discrete to continuos from conversion
    * output -> address of eye matrices
    * input  -> address of systemDynamicsParameter
    * */

    float **creatingEmptyMatrices(int rows,
                                  int cols); /*
    * creating dynamically empty matrices for general usage
    * output -> address of empty matrices
    * input  -> rows and columns values of matrices
    * */

    void initializeTheModel(systemDynamicParameterPtr ptrSys); /*
   * initializing the model
   * output -> returns nothing
   * input -> address of systemDynamicsParameter
   */

    void niteStaticSolver(systemDynamicParameterPtr ptrSys, float finalTime, const char *fileName); /*
    * creating a solver for static patterns!
    * output -> return nothing
    * input  -> address of systemDynamicsParameter
    *           finalTime in seconds
    *           file name to be stored data
    * 
    * Important Note: This code creates a text file to keep the data in matrices form.
    *                 The format parameter of this file is
    *                 "%f %f %f"  -> x1 state, x2 state, elapsedTime 
    * */

    void niteDynamicSolver(systemDynamicParameterPtr ptrSys, float samplingPeriod); /*
    * creating a solver for static patterns!
    * output -> return nothing
    * input  -> address of systemDynamicsParameter
    *           sampling period in seconds
    *           
    * 
    * Important Note: This code creates a text file to keep the data in matrices form.
    *                 The format parameter of this file is
    *                 "%f %f %f"  -> x1 state, x2 state, elapsedTime 
    * */

    void phiFree(float **pd,
                 int row,
                 int col); /*
    * free the whole memory allocation
    * output -> return nothing
    * input  -> address of memory
    *           row value of matrices
    *           column value of matrices
    * */

    void phiExit(systemDynamicParameterPtr ptrSys); /*
    * free the systemDynamics parameter
    * output -> return nothing
    * input  -> address of systemDynamicParameter
    * */

    void phiErrorHandler(int errorType); /*
    * notifying the error result
    * output -> return nothing
    * input  -> type of error
    * */

    // function prototype decleration
    ////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////
    // demos

    void ex1Demo(); /*
                     * First order system example
                     * */

    void ex2Demo(); /*
                     * Second order system example
                     * */

    void ex3Demo(); /*
                     * nth order system example
                     * */

    // demos
    ////////////////////////////////////////////////////////////

#ifdef __cplusplus
}
#endif

#endif