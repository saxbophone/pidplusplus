#include <stdio.h>
#include "pid.h"


// a brief test of the PidController class
int main(int argc, char const *argv[])
{
    // create new instance, using default constructor
    PidController controller;
    // now, continually loop and ask user for input values of target and actual
    while (true) {
        double a, t; // variables to temporarily store the inputted values of actual and target
        // read in user input:
        printf("Actual:\t");
        scanf("%lf", &a);
        printf("Target:\t");
        scanf("%lf", &t);
        // run algorithm and store result
        double r = controller.update(a, t);
        // output result
        printf("Output:\t%lf\n", r);
    }
    return 0;
}
