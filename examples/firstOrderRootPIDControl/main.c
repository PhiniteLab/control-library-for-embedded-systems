#include "..\..\include\phiSystemDynamicSetting.h "

int main()
{

    systemDynamicsParameter pSys;
    controlDynamicsParameter pCont;

    pSys.rows = 1;        // number of row in state space
    pSys.cols = 1;        // number of col in state space
    pSys.inputNumber = 1; // number of input in state space
    pSys.dt = 0.01;       // sampling period
    pSys.fileName = "ver1MCK.txt";

    /////////////////////////////////////////////////////
    // initialize the System
    initializeTheModel(&pSys);
    initializeControllerDesign(&pCont);

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

    setPIDRoots(&pCont, 0.35, 0.4, 0.3, 0.01);

    while (pSys.simulationTime < 5)
    {
        // reference signal generation
        pSys.xRef = 10 + 10 * sin(2 * phi_PI * 1 * pSys.simulationTime);
        float xRefForward = 10 + 10 * sin(2 * phi_PI * 1 * (pSys.simulationTime + pSys.dt));

        pSys.inputValue = generateOnlyPIDOutput(&pCont,
                                                &pSys,
                                                pSys.xRef,
                                                xRefForward,
                                                0,
                                                pSys.simulationTime,
                                                -100,
                                                100);

        niteDynamicSolver(&pSys, 0.001);
    }

    // should be called to free memory
    phiExit(&pSys);

    return 0;
}
// SYSTEM DYNAMICS
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
