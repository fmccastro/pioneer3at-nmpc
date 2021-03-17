#include "../scripts/codegen_demo.h"
#include "/home/fmccastro/Thesis_RoverNavigation/casADi/casadi-linux-matlabR2014b-v3.5.2/include/casadi/mem.h"
#include <iostream>
#include <cmath>
#include <cstdio>
#include <chrono>
#include <vector>

#define pi 3.1415926535897932384

int main(int argc, char** argv)
{ 
    printf("Number of inputs: %lli.\n", M_n_in() );
    printf("Number of outputs: %lli.\n", M_n_out() );
    
    return 0;
}