#include "control_loop.h"
#include <string.h>

using namespace std;

//the goal is to implement a simple PID controller, 
//running on input value to reach the desired one

// Sense: read a value from keyboard
// Plan:  generate the correct input
// Act:   set the input

int main(int argc, char** argv) {
    double initial_ref;

    if (argc > 1 ) {
        initial_ref = stod(argv[1]);
    }
    else initial_ref = 0.0;

    //gains obtained through simulink PID tuner
    double kp = 0.0;
    double ki = 2;
    double kd = 0.0;

    CONTROLLER contr(kp, ki, kd, initial_ref);

    contr.system_start();

    contr.run();

    return 0;
}

