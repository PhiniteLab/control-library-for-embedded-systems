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
#ifndef __PHI_MATH_LIB__
#define __PHI_MATH_LIB__

// in order to generalize our code, we need to use
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
    // define constants

#define phi_PI acos(-1.0)

    // define constants
    ////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////
    // function prototype decleration

    float **phiVectorMatrixMultiplication(float **firstTerm,
                                          float **SecondTerm,
                                          int row1,
                                          int col1,
                                          int row2,
                                          int col2); /*
    * creating dynamically matrices produced by multiplication
    * output -> address of multiplied matrices
    * input  -> address of first Matrices
    *           address of second Matrices
    *           row of first Matrices
    *           col of first Matrices
    *           row of second Matrices
    *           col of second Matrices
    * */

    float **phiSkalarMatrixMultiplication(float skalarTerm,
                                          float **SecondTerm,
                                          int row,
                                          int col); /*
    * creating dynamically matrices produced by skalar multiplication
    * output -> address of multiplied matrices
    * input  -> value of skalar term
    *           address of second Matrices
    *           row of second Matrices
    *           col of second Matrices
    * */

    float **phiMatrixSummation(float **firstTerm,
                               float **SecondTerm,
                               int row,
                               int col); /*
    * creating dynamically matrices produced by summation
    * output -> address of summed matrices
    * input  -> address of first Matrices
    *           address of second Matrices
    *           row of second Matrices
    *           col of second Matrices
    * */

    void phiMatrixAssignment(float **assignedTerm,
                             float **SecondTerm,
                             int row,
                             int col); /*
    * creating dynamically matrices produced by summation
    * output -> address of summed matrices
    * input  -> address of first Matrices
    *           address of second Matrices
    *           row of second Matrices
    *           col of second Matrices
    * */

    // function prototype decleration
    ////////////////////////////////////////////////////////////

#ifdef __cplusplus
}
#endif

#endif