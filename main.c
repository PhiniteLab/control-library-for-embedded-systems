#include "include\phiSystemDynamicSetting.h "

int main()
{

    systemDynamicsParameter pSys;
    controlDynamicsParameter pCont;

    pSys.rows = 2;        // number of row in state space
    pSys.cols = 2;        // number of col in state space
    pSys.inputNumber = 1; // number of input in state space
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
    pSys.stateMatrices[0][0] = 0;
    pSys.stateMatrices[0][1] = 1;
    pSys.stateMatrices[1][0] = -20;
    pSys.stateMatrices[1][1] = -30;

    pSys.inputMatrices[0][0] = 0;
    pSys.inputMatrices[1][0] = 10;

    printf("Writing state and input matrices \n\n");

    // write the state space matrices
    writeTheMatrices(&pSys);

    printf("\n\n");

    setPIDRoots(&pCont, 0.2, 0.1, 0.1, 0.01);
    //setPIDCoefficients(&pCont, 1, -1, -1, 0.1);

    while (pSys.simulationTime < 10)
    {

        // reference signal generation
        pSys.xRef = 1 + 0.2 * sin(2 * phi_PI * 1 * pSys.simulationTime);
        float xRefForward = 1 + 0.2 * sin(2 * phi_PI * 1 * (pSys.simulationTime + pSys.dt));

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
