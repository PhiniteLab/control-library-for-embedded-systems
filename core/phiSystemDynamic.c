#include "..\include\phiSystemDynamicSetting.h"

void initializeTheModel(systemDynamicParameterPtr ptrSys)
{
    if (ptrSys->rows != ptrSys->cols)
    {
        phiErrorHandler(INCONSISTENT_ROW_COLUMN);
        exit(EXIT_FAILURE);
    }

    if (ptrSys->dt <= 0)
    {
        phiErrorHandler(SAMPLING_RATE_ERROR);
        exit(EXIT_FAILURE);
    }

    ptrSys->tStart = (float)clock();
    ptrSys->simulationTime = 0.0;
    ////////////////////////////////////////////////
    // creating dynamic matrices
    ptrSys->stateNow = creatingEmptyMatrices(ptrSys->rows, 1);
    ptrSys->statePre = creatingEmptyMatrices(ptrSys->rows, 1);
    ptrSys->inputNow = creatingEmptyMatrices(1, 1);

    for (int i = 0; i < ptrSys->rows; i++)
    {
        ptrSys->stateNow[i][0] = 0.0;
        ptrSys->statePre[i][0] = 0.0;
    }

    // creating dynamic matrices
    ////////////////////////////////////////////////

    FILE *fileProcess;

    ptrSys->fileProcess = fopen(ptrSys->fileName, "w");

    ////////////////////////////////////////////////
    // file creation to write txt file

    if (ptrSys->fileProcess == NULL)
    {
        phiErrorHandler(FILE_OPEN_ERROR);
        exit(EXIT_FAILURE);
    }

    // file creation to write txt file
    ////////////////////////////////////////////////
}

float **creatingEmptyStateMatrices(systemDynamicParameterPtr ptrSys)
{
    float **pd = (float **)malloc(ptrSys->rows * sizeof(float *));

    if (pd == NULL)
    {
        phiErrorHandler(ALLOCATION_ERROR);
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < ptrSys->rows; i++)
        pd[i] = (float *)malloc(ptrSys->cols * sizeof(float));

    return pd;
}

float **creatingEmptyInputMatrices(systemDynamicParameterPtr ptrSys)
{
    float **pd = (float **)malloc(ptrSys->rows * sizeof(float *));

    if (pd == NULL)
    {
        phiErrorHandler(ALLOCATION_ERROR);
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < ptrSys->rows; i++)
        pd[i] = (float *)malloc(ptrSys->cols * sizeof(float));

    return pd;
}

void writeTheMatrices(systemDynamicParameterPtr ptrSys)
{
    printf("Printing state matrices...\n");
    for (int i = 0; i < ptrSys->rows; i++)
    {
        for (int j = 0; j < ptrSys->cols; j++)
        {
            printf("%f ", ptrSys->stateMatrices[i][j]);
        }
        printf("\n");
    }
    printf("\n");

    printf("Printing input matrices...\n");
    for (int i = 0; i < ptrSys->rows; i++)
    {
        for (int j = 0; j < ptrSys->inputNumber; j++)
        {
            printf("%f ", ptrSys->inputMatrices[i][j]);
        }
        printf("\n");
    }
    printf("\n");
}

float **eyeMatricesCreation(systemDynamicParameterPtr ptrSys)
{
    float **pd = (float **)malloc(ptrSys->rows * sizeof(float *));

    if (pd == NULL)
    {
        phiErrorHandler(ALLOCATION_ERROR);
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < ptrSys->rows; i++)
        pd[i] = (float *)malloc(ptrSys->cols * sizeof(float));

    for (int i = 0; i < ptrSys->rows; i++)
    {
        for (int j = 0; j < ptrSys->cols; j++)
        {
            if (i == j)
            {
                pd[i][j] = 1;
            }
            else
            {
                pd[i][j] = 0;
            }
        }
    }

    return pd;
}

float **creatingEmptyMatrices(int rows, int cols)
{
    float **pd = (float **)malloc(rows * sizeof(float *));

    if (pd == NULL)
    {
        phiErrorHandler(ALLOCATION_ERROR);
        exit(EXIT_FAILURE);
    }
    for (int i = 0; i < rows; i++)
        pd[i] = (float *)malloc(cols * sizeof(float));

    return pd;
}

void niteStaticSolver(systemDynamicParameterPtr ptrSys, float finalTime, const char *fileName)
{
    ////////////////////////////////////////////////
    // internal terms
    float **DtA;
    float **eyeDtA;
    float **DtB;
    float **stateMultiplicationMatrices;
    float **inputMultiplicationMatrices;
    float **eye = eyeMatricesCreation(ptrSys);
    int numberOfLength = (int)(finalTime / ptrSys->dt);

    // internal terms
    ////////////////////////////////////////////////

    ////////////////////////////////////////////////
    // file creation to write txt file
    FILE *fp;

    fp = fopen(fileName, "w");

    if (fp == NULL)
    {
        phiErrorHandler(FILE_OPEN_ERROR);
        exit(EXIT_FAILURE);
    }

    // file creation to write txt file
    ////////////////////////////////////////////////

    ////////////////////////////////////////////////
    // creating dynamic matrices
    ptrSys->stateNow = creatingEmptyMatrices(ptrSys->rows, 1);
    ptrSys->statePre = creatingEmptyMatrices(ptrSys->rows, 1);
    ptrSys->inputNow = creatingEmptyMatrices(1, 1);

    // creating dynamic matrices
    ////////////////////////////////////////////////

    ////////////////////////////////////////////////
    // Solving discrete form of state space

    // the whole operations
    ptrSys->inputNow[0][0] = ptrSys->inputValue;

    DtA = phiSkalarMatrixMultiplication(ptrSys->dt, ptrSys->stateMatrices, ptrSys->rows, ptrSys->cols);
    DtB = phiSkalarMatrixMultiplication(ptrSys->dt, ptrSys->inputMatrices, ptrSys->rows, ptrSys->inputNumber);
    eyeDtA = phiMatrixSummation(eye, DtA, ptrSys->rows, ptrSys->cols);

    for (int iter = 0; iter < numberOfLength; iter++)
    {
        stateMultiplicationMatrices = phiVectorMatrixMultiplication(eyeDtA, ptrSys->statePre,
                                                                    ptrSys->rows, ptrSys->cols, ptrSys->rows, 1);
        inputMultiplicationMatrices = phiVectorMatrixMultiplication(DtB, ptrSys->inputNow,
                                                                    ptrSys->rows, 1, 1, 1);

        float currentTime = iter * ptrSys->dt;
        for (int i = 0; i < ptrSys->rows; i++)
        {
            ptrSys->stateNow[i][0] = stateMultiplicationMatrices[i][0] + inputMultiplicationMatrices[i][0];
        }

        ///////////////////////////////////////////////////////////
        // printing results

        printf("State values : ");

        for (int i = 0; i < ptrSys->rows; i++)
        {
            printf("x[%d] : %f ", i, ptrSys->statePre[i][0]);
            fprintf(fp, "%f ", ptrSys->statePre[i][0]);
        }
        printf("elapsed Time : %f seconds\n", currentTime);
        fprintf(fp, "%f \n", currentTime);

        // printing results
        ///////////////////////////////////////////////////////////

        phiMatrixAssignment(ptrSys->statePre, ptrSys->stateNow, ptrSys->rows, 1);
    }

    // Solving discrete form of state space
    ////////////////////////////////////////////////

    ///////////////////////////////////////////////////////
    // free the whole memory
    phiFree(DtA, ptrSys->rows, ptrSys->cols);
    phiFree(eyeDtA, ptrSys->rows, ptrSys->cols);
    phiFree(DtB, ptrSys->rows, ptrSys->inputNumber);
    phiFree(stateMultiplicationMatrices, ptrSys->rows, 1);
    phiFree(inputMultiplicationMatrices, ptrSys->rows, 1);

    phiFree(ptrSys->stateNow, ptrSys->rows, 1);
    phiFree(ptrSys->statePre, ptrSys->rows, 1);
    phiFree(ptrSys->inputNow, ptrSys->inputNumber, 1);

    fclose(fp);

    // free the whole memory
    ///////////////////////////////////////////////////////
}

void niteDynamicSolver(systemDynamicParameterPtr ptrSys, float samplingPeriod)
{
    ////////////////////////////////////////////////
    // sampling period calculation

    float fixedTimeMilFloat = 1000 * samplingPeriod;

    clock_t fixedTimeMilliseconds = 0;

    clock_t timeElapsed = clock() - ptrSys->tStart;

    if (samplingPeriod != -1)
    {
        fixedTimeMilliseconds = (clock_t)fixedTimeMilFloat;
        delay(fixedTimeMilliseconds - timeElapsed);
        ptrSys->dt = ((float)fixedTimeMilliseconds) / 1000;
        ptrSys->tStart = clock();
    }
    else
    {
        ptrSys->tStart = clock();
        ptrSys->dt = ((float)timeElapsed) / 1000;
    }

    ////////////////////////////////////////////////
    // internal terms
    float **DtA;
    float **eyeDtA;
    float **DtB;
    float **stateMultiplicationMatrices;
    float **inputMultiplicationMatrices;

    // internal terms
    ////////////////////////////////////////////////

    ////////////////////////////////////////////////
    // Solving discrete form of state space

    // the whole operations
    ptrSys->inputNow[0][0] = ptrSys->inputValue;

    float **eye = eyeMatricesCreation(ptrSys);
    DtA = phiSkalarMatrixMultiplication(ptrSys->dt, ptrSys->stateMatrices, ptrSys->rows, ptrSys->cols);
    DtB = phiSkalarMatrixMultiplication(ptrSys->dt, ptrSys->inputMatrices, ptrSys->rows, ptrSys->inputNumber);
    eyeDtA = phiMatrixSummation(eye, DtA, ptrSys->rows, ptrSys->cols);

    stateMultiplicationMatrices = phiVectorMatrixMultiplication(eyeDtA, ptrSys->statePre,
                                                                ptrSys->rows, ptrSys->cols, ptrSys->rows, 1);
    inputMultiplicationMatrices = phiVectorMatrixMultiplication(DtB, ptrSys->inputNow,
                                                                ptrSys->rows, 1, 1, 1);

    ptrSys->simulationTime = ptrSys->simulationTime + ptrSys->dt;

    for (int i = 0; i < ptrSys->rows; i++)
    {
        ptrSys->stateNow[i][0] = stateMultiplicationMatrices[i][0] + inputMultiplicationMatrices[i][0];
    }

    ///////////////////////////////////////////////////////////
    // printing results

    printf("State values : ");

    for (int i = 0; i < ptrSys->rows; i++)
    {
        printf("x[%d] : %f ", i, ptrSys->statePre[i][0]);
        fprintf(ptrSys->fileProcess, "%f ", ptrSys->statePre[i][0]);
    }
    printf("xRef : %f ", ptrSys->xRef);
    fprintf(ptrSys->fileProcess, "%f ", ptrSys->xRef);
    printf("input : %f ", ptrSys->inputValue);
    fprintf(ptrSys->fileProcess, "%f ", ptrSys->inputValue);
    printf("elapsed Time : %f seconds\n", ptrSys->simulationTime);
    fprintf(ptrSys->fileProcess, "%f \n", ptrSys->simulationTime);

    // printing results
    ///////////////////////////////////////////////////////////

    phiMatrixAssignment(ptrSys->statePre, ptrSys->stateNow, ptrSys->rows, 1);

    // Solving discrete form of state space
    ////////////////////////////////////////////////

    ///////////////////////////////////////////////////////
    // free the whole memory
    phiFree(DtA, ptrSys->rows, ptrSys->cols);
    phiFree(eyeDtA, ptrSys->rows, ptrSys->cols);
    phiFree(DtB, ptrSys->rows, ptrSys->inputNumber);
    phiFree(stateMultiplicationMatrices, ptrSys->rows, 1);
    phiFree(inputMultiplicationMatrices, ptrSys->rows, 1);

    // free the whole memory
    ///////////////////////////////////////////////////////
}

void phiFree(float **pd, int row, int col)
{
    for (int i = 0; i < row; i++)
        free(pd[i]);

    free(pd);
}

void phiExit(systemDynamicParameterPtr ptrSys)
{

    phiFree(ptrSys->stateNow, ptrSys->rows, 1);
    phiFree(ptrSys->statePre, ptrSys->rows, 1);
    phiFree(ptrSys->inputNow, ptrSys->inputNumber, 1);

    free(ptrSys->stateMatrices);
    free(ptrSys->inputMatrices);

    fclose(ptrSys->fileProcess);
}

///////////////////////////////////////////////////////////////////////
// demos

void ex1Demo()
{
    systemDynamicsParameter pSys;

    pSys.rows = 1;        // number of row in state space
    pSys.cols = 1;        // number of col in state space
    pSys.inputNumber = 1; // number of input in state space
    pSys.dt = 0.001;      // sampling period

    pSys.stateMatrices = creatingEmptyStateMatrices(&pSys);
    // state matrices creation
    pSys.inputMatrices = creatingEmptyInputMatrices(&pSys);
    // input matrices creation

    // matrices value assignment
    pSys.stateMatrices[0][0] = -1;
    pSys.inputMatrices[0][0] = 1;

    printf("Writing state and input matrices \n\n");

    // write the state space matrices
    writeTheMatrices(&pSys);

    printf("\n\n");

    /// solver solution
    // select the inputValue to be given to the system
    pSys.inputValue = 10.0;

    // static time parameter solver
    niteStaticSolver(&pSys, 10, "ver1MCK.txt");

    // should be called to free memory
    phiExit(&pSys);
}

void ex2Demo()
{
    systemDynamicsParameter pSys;

    pSys.rows = 2;        // number of row in state space
    pSys.cols = 2;        // number of col in state space
    pSys.inputNumber = 1; // number of input in state space
    pSys.dt = 0.001;      // sampling period

    pSys.stateMatrices = creatingEmptyStateMatrices(&pSys);
    // state matrices creation
    pSys.inputMatrices = creatingEmptyInputMatrices(&pSys);
    // input matrices creation

    // matrices value assignment
    pSys.stateMatrices[0][0] = 0;
    pSys.stateMatrices[0][1] = 1;
    pSys.stateMatrices[1][0] = -0.1;
    pSys.stateMatrices[1][1] = -1;

    pSys.inputMatrices[0][0] = 0;
    pSys.inputMatrices[1][0] = 1;

    printf("Writing state and input matrices \n\n");

    // write the state space matrices
    writeTheMatrices(&pSys);

    printf("\n\n");

    /// solver solution
    // select the inputValue to be given to the system
    pSys.inputValue = 10.0;

    // static time parameter solver
    niteStaticSolver(&pSys, 10, "ver1MCK.txt");

    // should be called to free memory
    phiExit(&pSys);
}

void ex3Demo()
{
    systemDynamicsParameter pSys;

    pSys.rows = 3;        // number of row in state space
    pSys.cols = 3;        // number of col in state space
    pSys.inputNumber = 1; // number of input in state space
    pSys.dt = 0.001;      // sampling period

    pSys.stateMatrices = creatingEmptyStateMatrices(&pSys);
    // state matrices creation
    pSys.inputMatrices = creatingEmptyInputMatrices(&pSys);
    // input matrices creation

    // matrices value assignment
    pSys.stateMatrices[0][0] = 0;
    pSys.stateMatrices[0][1] = 1;
    pSys.stateMatrices[0][2] = 0;

    pSys.stateMatrices[1][0] = 0;
    pSys.stateMatrices[1][1] = 0;
    pSys.stateMatrices[1][2] = 1;

    pSys.stateMatrices[2][0] = -2;
    pSys.stateMatrices[2][1] = -3;
    pSys.stateMatrices[2][2] = -4;

    pSys.inputMatrices[0][0] = 0;
    pSys.inputMatrices[1][0] = 0;
    pSys.inputMatrices[2][0] = 1;

    printf("Writing state and input matrices \n\n");

    // write the state space matrices
    writeTheMatrices(&pSys);

    printf("\n\n");

    /// solver solution
    // select the inputValue to be given to the system
    pSys.inputValue = 2.0;

    // static time parameter solver
    niteStaticSolver(&pSys, 10, "ver1MCK.txt");

    // should be called to free memory
    phiExit(&pSys);
}
// demos
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// error Handler

void phiErrorHandler(int errorType)
{
    switch (errorType)
    {
    case FILE_OPEN_ERROR:
        printf("System Dynamic Parameter files cannot be created!\n");
        break;
    case INCONSISTENT_ROW_COLUMN:
        printf("The rows and columns are not consistent!\n");
        break;
    case ALLOCATION_ERROR:
        printf("Memory allocation cannot be done!\n");
        break;
    case SAMPLING_RATE_ERROR:
        printf("Sampling period cannot be assigned to either negative or zero value!\n");
        break;

    default:
        break;
    }
}

// error Handler
//////////////////////////////////////////////////////////////////////
