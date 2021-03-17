#include "../ACADO_CodeGeneration/export_NMPC/acado_common.h"
#include "../ACADO_CodeGeneration/export_NMPC/acado_auxiliary_functions.h"
#include <iostream>
#include <cmath>
#include <cstdio>
#include <chrono>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Vector3.h"

/* Some convenient definitions. */
#define NX          ACADO_NX    /* Number of differential state variables.  */
#define NXA         ACADO_NXA   /* Number of algebraic variables. */ ////Irrelevant to this one
#define NU          ACADO_NU    /* Number of control inputs. */
#define NOD         ACADO_NOD   /* Number of online data values. */

#define NY          ACADO_NY    /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN   /* Number of measurements/references on node N. */

#define N           ACADO_N     /* Number of intervals in the horizon. */

#define NUM_STEPS   10          /* Number of real-time iterations. */  //Doesnt matter when using a while(ros::ok())
#define VERBOSE     1           /* Show iterations: 1, silent: 0.  */

#define pi 3.1415926535897932384

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

using namespace std;

/* Global variable for desired position and orientation {x, y, theta} */
struct GOAL {
    double initialPoint[2];
    double goalPoint[2];            // In fact, theta can be neglected. The goal point is what matters.
    double targetState[3];
    float step;
    double direction[2];
};

GOAL robotGuidance;

void initializeGOALStruct()
{
    robotGuidance.initialPoint[0] = 0.0;
    robotGuidance.initialPoint[1] = 0.0;

    robotGuidance.goalPoint[0] = 10.0;
    robotGuidance.goalPoint[1] = 5.0;

    double directionX = robotGuidance.goalPoint[0] - robotGuidance.initialPoint[0];
    double directionY = robotGuidance.goalPoint[1] - robotGuidance.initialPoint[1];

    double mod = sqrt( pow( directionX , 2) + pow( directionY , 2) );

    robotGuidance.direction[0] = directionX / mod;
    robotGuidance.direction[1] = directionY / mod;
	
    robotGuidance.step = 2.0;

    /* First Checkpoint */
    robotGuidance.targetState[0] = robotGuidance.initialPoint[0] + 
                                        robotGuidance.step * robotGuidance.direction[0];
    
    robotGuidance.targetState[1] = robotGuidance.initialPoint[1] + 
                                        robotGuidance.step * robotGuidance.direction[1];         

    if( robotGuidance.goalPoint[1] >= robotGuidance.initialPoint[1] )
    {    
        robotGuidance.targetState[2] = acos( robotGuidance.direction[0] );
    }
    else
    {
        robotGuidance.targetState[2] = -acos( robotGuidance.direction[0] );         
    }

    for(int i = 0; i < N + 1; ++i )    
    {
        acadoVariables.od[i * NOD] = robotGuidance.targetState[0];                                
        acadoVariables.od[i * NOD + 1] = robotGuidance.targetState[1];
        acadoVariables.od[i * NOD + 2] = robotGuidance.targetState[2];
        acadoVariables.od[i * NOD + 3] = 0.7;
        acadoVariables.od[i * NOD + 4] = 0.0;
        acadoVariables.od[i * NOD + 5] = 0.0;
        acadoVariables.od[i * NOD + 6] = 0.0;
        acadoVariables.od[i * NOD + 7] = 0.0;
    }
}

void getCheckpoint()
{
    robotGuidance.targetState[0] = robotGuidance.targetState[0] + 
                                        robotGuidance.step * robotGuidance.direction[0];
    
    robotGuidance.targetState[1] = robotGuidance.targetState[1] + 
                                        robotGuidance.step * robotGuidance.direction[1];
}

void getTargetValues()
{
    getCheckpoint();

    for(int i = 0; i < N + 1; ++i )    
    {
        acadoVariables.od[i * NOD] = robotGuidance.targetState[0];                                
        acadoVariables.od[i * NOD + 1] = robotGuidance.targetState[1];
        acadoVariables.od[i * NOD + 2] = robotGuidance.targetState[2];
        acadoVariables.od[i * NOD + 3] = 0.7;
        acadoVariables.od[i * NOD + 4] = 0.0;
    }
}

double distBtwPoints(int i)
{ 
    if (i == 0)     /* Distance to goal */
    {
        return sqrt( pow( acadoVariables.x0[0] - robotGuidance.goalPoint[0], 2) 
                        + pow( acadoVariables.x0[1] - robotGuidance.goalPoint[1], 2) );
    }
    else            /* Distance to current checkpoint */
    {
        return sqrt( pow( acadoVariables.x0[0] - robotGuidance.targetState[0], 2) 
                        + pow( acadoVariables.x0[1] - robotGuidance.targetState[1], 2) );
    }
}                        

void get_cost_matrices()
{
    int i, j;

    Eigen::VectorXd W(NY);
    Eigen::VectorXd WN(NY);

    for (i = 0; i < NY; i++)
    {
        for (j = 0; j < NY; j++)
        {
            if(i == j)
                acadoVariables.W[ i * NY + j] = 0.8;
            else
                acadoVariables.W[ i * NY + j ] = 0;
        }
    }

    for (i = 0; i < NYN; i++)
    {
        for (j = 0; j < NYN; j++)
        {
            if(i == j)
                acadoVariables.WN[ i * NYN + j] = 0.1;
            else
                acadoVariables.WN[ i * NYN + j ] = 0;
        }
    }
}

void odometryCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    geometry_msgs::Vector3 orientation;
    
    acadoVariables.x0[0] = msg->linear.x;
    acadoVariables.x0[1] = msg->linear.y;
    acadoVariables.x0[2] = msg->angular.z;
}

void initializeNMPC()
{
    int i;
    
    /* Initialize the states and controls. */
	for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 0.0;
	for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;

	/* Initialize the measurements/reference. */
	for (i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0;
	for (i = 0; i < NYN; ++i)  acadoVariables.yN[ i ] = 0.0;

    /* Online Data should be included here */
    for(i = 0; i < N + 1; ++i )    
    {
        acadoVariables.od[i * NOD] = 0.0;
        acadoVariables.od[i * NOD + 1] = 0.0;
        acadoVariables.od[i * NOD + 2] = 0.0;
        acadoVariables.od[i * NOD + 3] = 0.0;
        acadoVariables.od[i * NOD + 4] = 0.0;
        acadoVariables.od[i * NOD + 5] = 0.0;
        acadoVariables.od[i * NOD + 6] = 0.0;
        acadoVariables.od[i * NOD + 7] = 0.0;
    }

	/* MPC: initialize the current state feedback. */
    #if ACADO_INITIAL_STATE_FIXED
        for (i = 0; i < NX; ++i) acadoVariables.x0[ i ] = 0.0;
    #endif
}

int main(int argc, char** argv)
{   
    getchar();
    cout << "Enter a character to start simulation.\n";
    getchar();

    ros::init(argc, argv, "nmpc");
    ros::NodeHandle nmpc;

    ros::Subscriber odometry_sub = nmpc.subscribe("/pioneer3at/odomTransformed", 1000, odometryCallback);

    ros::Publisher controls_pub = nmpc.advertise<geometry_msgs::Twist>("/pioneer3at/cmd_vel", 1000);
    ros::Publisher target_pub = nmpc.advertise<geometry_msgs::Twist>("/pioneer3at/targetValues", 1000);

    ros::Rate rate(10);

    printf("|***************************************************************|\n");
    printf("| Number of differential states = %d\n", NX                         );
    printf("| Number of algebraic variables = %d\n", NXA                        );
    printf("| Number of control inputs = %d\n", NU                              );
    printf("| Number of online data values = %d\n", NOD                         );
    printf("| Number of measurements/references on nodes 0...N-1 = %d\n", NY    );
    printf("| Number of measurements/references on node N = %d\n", NYN          );
    printf("| Number of intervals in the horizon = %d .\n", N                   );
    printf("|***************************************************************|\n");

    getchar();

    /* Actuation Twist */
    geometry_msgs::Twist actuation, target;

    /* Some temporary variables. */
	int    i, iter;
	acado_timer t;

	/* Initialize the solver. */
	acado_initializeSolver();

    initializeNMPC();

	if( VERBOSE ) acado_printHeader();
    
    /* Initialise weight matrices */
    get_cost_matrices();

    initializeGOALStruct();

    target.linear.x = robotGuidance.targetState[0];
    target.linear.y = robotGuidance.targetState[1];
    target.angular.z = robotGuidance.targetState[2];

    target_pub.publish(target);

    /* Prepare first step */
	acado_preparationStep();

	/* Get the time before start of the loop. */
	acado_tic( &t );

    int count = 0;
    while( ros::ok() && distBtwPoints(0) > 0.25 )
    {   
        if (distBtwPoints(1) <= 0.5)
        {
            getTargetValues();
        }

        target.linear.x = robotGuidance.targetState[0];
        target.linear.y = robotGuidance.targetState[1];
        target.angular.z = robotGuidance.targetState[2];

        target_pub.publish(target);

        /* Perform the feedback step. */
		acado_feedbackStep();

        /* Publish actuation */

        actuation.linear.x = acadoVariables.u[0];
        actuation.angular.z = acadoVariables.u[1];
        controls_pub.publish(actuation);

        /* Apply the new control immediately to the process, first NU components. */
        printf( "|*****************************************************|\n");
		if( VERBOSE ) printf("| Real-Time Iteration %d:  KKT Tolerance = %.3e\n", count, acado_getKKT() );
        if( VERBOSE ) printf("| Real-Time Iteration %d:  Objective = %.3e\n", count, acado_getObjective() );

        /* Optional: shift the initialization (look at acado_common.h). */
        acado_shiftStates(2, 0, 0);
		acado_shiftControls( 0 );
        
        ros::spinOnce();

        /* Prepare for the next step. */
		acado_preparationStep();

        printf( "| Position x : %f\n", acadoVariables.x0[0]                     );
        printf( "| Position y : %f\n", acadoVariables.x0[1]                     );
        printf( "| Yaw : %f\n", acadoVariables.x0[2]                            );
        printf( "| Control V_x : %f\n", acadoVariables.u[0]                     );
        printf( "| Control W_z : %f\n", acadoVariables.u[1]                     );
        printf( "| Distance to GOAL: %f\n", distBtwPoints(0)                    );
        printf( "| Distance to TARGET: %f\n", distBtwPoints(1)                  );
        printf( "|*****************************************************|\n\n\n" );

        rate.sleep();

        ++count;
    }

    actuation.linear.x = 0;
    actuation.angular.z = 0;
    controls_pub.publish(actuation);

	/* Read the elapsed time. */
	real_t te = acado_toc( &t );

	if( VERBOSE ) printf("\n\nEnd of the RTI loop. \n\n\n");

	/* Eye-candy. */

	if( !VERBOSE )
	printf("\n\n Average time of one real-time iteration:   %.3g microseconds\n\n", 1e6 * te / NUM_STEPS);

	acado_printDifferentialVariables();
	acado_printControlVariables();

    return 0;
}
