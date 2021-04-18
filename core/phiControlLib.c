#include "..\include\phiSystemDynamicSetting.h"

void initializeControllerDesign(controlDynamicsParameterPtr ptrCont)
{
    // error update
    ptrCont->errorNow = 0.0;
    ptrCont->errorPre = 0.0;
    ptrCont->errorPrePre = 0.0;

    ptrCont->Kp = -1.0;
    ptrCont->Kd = -1.0;
    ptrCont->Ki = -1.0;

    ptrCont->stepNumber = 0;
    ptrCont->stepNumberPre = 0;
}

void setPIDRoots(controlDynamicsParameterPtr ptrCont,
                 float root1,
                 float root2,
                 float root3,
                 float samplingPeriod)
{
    ptrCont->phiRoots_1 = root1;
    ptrCont->phiRoots_2 = root2;
    ptrCont->phiRoots_3 = root3;

    // pid calculator

    ptrCont->Kd = -root1 * root2 * root3 / (samplingPeriod);

    ptrCont->Kp = -(root3 * (root2 + root1) + root1 * root2 +
                    2 * ptrCont->Kd * samplingPeriod) /
                  (samplingPeriod);

    ptrCont->Ki = -(root3 + root2 + root1 + ptrCont->Kd * samplingPeriod +
                    ptrCont->Kp * samplingPeriod) /
                  (samplingPeriod);

    ptrCont->Ae = (root1 + root2 + root3) / 3;

    ptrCont->samplingPeriod = samplingPeriod;
}

void setPIDCoefficients(controlDynamicsParameterPtr ptrCont, float Kp, float Kd,
                        float Ki, float samplingPeriod)
{
    ptrCont->Kd = Kd;

    ptrCont->Kp = Kp;

    ptrCont->Ki = Ki;

    ptrCont->Ae = (Kp + Kd + Ki) / 3;

    ptrCont->samplingPeriod = samplingPeriod;
}

int findingInputState(systemDynamicParameterPtr ptrSys)
{
    for (int i = 0; i < ptrSys->rows; i++)
    {
        if (ptrSys->inputMatrices[i][0] != 0)
        {
            return i;
        }
    }

    return -1;
}

float generateOnlyPIDOutput(controlDynamicsParameterPtr ptrCont,
                            systemDynamicParameterPtr ptrSys,
                            float referenceSignal,
                            float forwardReferenceSignal,
                            int selectControlState,
                            float simulationTime,
                            float min, float max)
{
    float returnInputValue = 0.0;
    float addControlPart = 0.0;

    int selectedInputState = 0;

    ptrCont->stepNumber = (int)(simulationTime / ptrCont->samplingPeriod);

    selectedInputState = findingInputState(ptrSys);

    if (selectedInputState == -1)
    {
        printf("No input is fed into the system");
        exit(EXIT_FAILURE);
    }

    if (ptrCont->stepNumber != ptrCont->stepNumberPre)
    {

        ////////////////////////////////////////////////////////
        // additional parts to reference tracking control

        float intAddControlPart = 0.0;

        if (selectedInputState == selectControlState)
        {
            for (int i = 0; i < ptrSys->cols; i++)
            {
                intAddControlPart = intAddControlPart +
                                    (ptrCont->samplingPeriod *
                                     ptrSys->stateMatrices[selectedInputState][i]) *
                                        ptrSys->statePre[i][0];

                if (i == selectedInputState)
                {
                    intAddControlPart = intAddControlPart +
                                        ptrSys->statePre[selectControlState][0];
                }
            }
            addControlPart = (forwardReferenceSignal - intAddControlPart);

            // error calculation
            ptrCont->errorNow = referenceSignal - ptrSys->statePre[selectControlState][0];

            // calculation of PID term
            float kiTerm = -ptrCont->Ki * ptrCont->errorNow;
            float kpTerm = -ptrCont->Kp * (ptrCont->errorNow - ptrCont->errorPre);
            float kdTerm = -ptrCont->Kd * (ptrCont->errorNow - 2 * ptrCont->errorPre + ptrCont->errorPrePre);

            // forward inputVariable is calculated!
            ptrCont->inputNow = ptrCont->inputPre +
                                (1 / (ptrCont->samplingPeriod * ptrSys->inputMatrices[selectedInputState][0])) *
                                    addControlPart +
                                (1 / ptrSys->inputMatrices[selectedInputState][0]) * (kiTerm + kpTerm + kdTerm);
        }
        else
        {
            // second order forward difference equation should be added here!
            float f1Constant = 2 + ptrSys->stateMatrices[1][1] * ptrCont->samplingPeriod;
            float f2Constant = -ptrSys->stateMatrices[1][1] * ptrCont->samplingPeriod + ptrSys->stateMatrices[1][0] * ptrCont->samplingPeriod * ptrCont->samplingPeriod - 1;

            float g1Constant = ptrCont->samplingPeriod * ptrCont->samplingPeriod * ptrSys->inputMatrices[1][0];

            float intaddControlPart = (forwardReferenceSignal - f1Constant * (ptrSys->statePre[0][0] + ptrCont->samplingPeriod * ptrSys->statePre[1][0]) - ptrSys->statePre[0][0] * f2Constant);

            // error calculation
            ptrCont->errorNow = referenceSignal - ptrSys->statePre[selectControlState][0];

            // forward inputVariable is calculated!
            ptrCont->inputNow = 1 / g1Constant * (intaddControlPart - ptrCont->Ae * ptrCont->errorNow);
        }

        // additional parts to reference tracking control
        ////////////////////////////////////////////////////////

        // check max and min in control input
        if (ptrCont->inputNow > max)
        {
            ptrCont->inputNow = max;
        }
        else
        {
            if (ptrCont->inputNow < min)
            {
                ptrCont->inputNow = min;
            }
        }

        // return value assignment
        returnInputValue = ptrCont->inputNow;

        // next iteration is updated!
        ptrCont->inputPre = ptrCont->inputNow;

        ////////////////////////////////////////////////
        // error update
        ptrCont->errorPre = ptrCont->errorNow;
        ptrCont->errorPrePre = ptrCont->errorPre;

        ptrCont->stepNumberPre = ptrCont->stepNumber;
        // error update
        ////////////////////////////////////////////////
    }
    else
    {
        // forward inputVariable is calculated!
        ptrCont->inputNow = ptrCont->inputPre;

        // check max and min in control input
        if (ptrCont->inputNow > max)
        {
            ptrCont->inputNow = max;
        }
        else
        {
            if (ptrCont->inputNow < min)
            {
                ptrCont->inputNow = min;
            }
        }

        // return value assignment
        returnInputValue = ptrCont->inputNow;

        // next iteration is updated!
        ptrCont->inputPre = ptrCont->inputNow;
    }

    return returnInputValue;
}
