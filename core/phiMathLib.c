#include "..\include\phiSystemDynamicSetting.h"

////////////////////////////////////////////////////////////////////
// function definition

float **phiVectorMatrixMultiplication(float **firstTerm, float **SecondTerm, int row1, int col1, int row2, int col2)
{
    float **pd = creatingEmptyMatrices(row1, col2);
    float sum = 0;

    if (pd == NULL)
    {
        phiErrorHandler(ALLOCATION_ERROR);
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < row1; i++)
    {
        for (int j = 0; j < col2; j++)
        {
            pd[i][j] = 0;
        }
    }

    for (int i = 0; i < row1; i++) //row of first matrix
    {
        for (int j = 0; j < col2; j++) //column of second matrix
        {
            sum = 0;
            for (int k = 0; k < col1; k++)
            {
                sum = sum + firstTerm[i][k] * SecondTerm[k][j];
            }
            pd[i][j] = sum;
        }
    }
    return pd;
}

float **phiSkalarMatrixMultiplication(float skalarTerm, float **SecondTerm, int row, int col)
{
    float **pd = creatingEmptyMatrices(row, col);

    if (pd == NULL)
    {
        phiErrorHandler(ALLOCATION_ERROR);
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            pd[i][j] = skalarTerm * SecondTerm[i][j];
        }
    }

    return pd;
}

float **phiMatrixSummation(float **firstTerm, float **SecondTerm, int row, int col)
{
    float **pd = creatingEmptyMatrices(row, col);

    if (pd == NULL)
    {
        phiErrorHandler(ALLOCATION_ERROR);
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            pd[i][j] = firstTerm[i][j] + SecondTerm[i][j];
        }
    }

    return pd;
}

void phiMatrixAssignment(float **assignedTerm, float **SecondTerm, int row, int col)
{
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            assignedTerm[i][j] = SecondTerm[i][j];
        }
    }
}

// function definition
////////////////////////////////////////////////////////////////////
