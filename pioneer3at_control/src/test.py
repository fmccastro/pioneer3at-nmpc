#!/usr/bin/env python
import math

import numpy as np

from scipy.spatial.transform import Rotation

def array_squared_norm(array):
    
    nb_elements = len(array)
    index = 0
    norm = 0

    while index < nb_elements:
        norm = norm + array[index]**2
        index = index + 1 
    
    return norm

def process_model(pose, time_step, rot_matrix, command):
    #diag_rot_matrix = np.vstack( [ np.hstack( [rot_matrix, np.zeros((3,3)) ] ), 
    #                                np.hstack( [np.zeros((3,3)), rot_matrix] ) ] )
    
    #print(diag_rot_matrix)
    #print(( np.array( [ pose ] ) ).T)
    #print(( np.array( [ command ] ) ).T)
    
    actuation_world_frame = np.array( [ rot_matrix[0][0] * command[0], 
                                            rot_matrix[1][0] * command[0], 
                                                rot_matrix[2][0] * command[0],
                                                    rot_matrix[0][2] * command[1],
                                                        rot_matrix[1][2] * command[1], 
                                                            rot_matrix[2][2] * command[1] ] )

    #print( np.array( [ actuation_world_frame ] ).T )
    #print( pose )
    #print( time_step )
    #print( rot_matrix )

    return ( np.array( [ pose ] ) ).T + time_step * ( np.array( [ actuation_world_frame ] ) ).T

"""
def cost_function(commands, *fixed_args): #= (pose, hor, ctrl_weight, rot_matrix, time_step)):
    
    args = []
    for element in fixed_args:
        args.append(element)
        #print(args)
    
    print( args[0] )
    #print(args[1])
    #print(args[2])
    #print(args[3])
    #print(args[4])

    pose = []

    pose.append( args[0] ) 
    index = 0
    final_result = 0

    #print(pose[0])
    #print(commands)

    print( "horizon" + str(args[1]) )

    while( index < args[1] ):
        if(index == 1):
            pose.append( process_model(pose[index - 1], args[4], args[3], commands[index - 1]) )
        elif(index > 1):
            # compute rotation matrix

            #print( pose[index - 1][3][0] )
            #print( pose[index - 1][4][0] )
            #print( pose[index - 1][5][0] )
            
            rotation_matrix_aux = Rotation.from_rotvec( [ pose[index - 1][3][0], 
                                                          pose[index - 1][4][0], 
                                                          pose[index - 1][5][0] ] ) 

            rotation_matrix = rotation_matrix_aux.as_dcm()

            #print( commands[index - 1] )
            
            pose[index] = process_model( pose[index - 1], args[4], 
                                rotation_matrix, commands[index - 1] )
        else:
            pass
        
        final_result = array_squared_norm(pose[index]) + \
                                    args[2] * array_squared_norm(commands[index])         
        index = index + 1

    return final_result

    return array_squared_norm( args[0] ) + args[2] * array_squared_norm( command[0] ) +
                sum( process_model(pose, time_step, rot_matrix, command) + 
                    args[2] * array_squared_norm( command[1:] ) )
"""
if __name__ == '__main__':
    pose_aux = np.array([1, 1, 1, 1, 1, 1])
    R_matrix = np.identity(3) 

    print(pose_aux)
    print('\n')
    print(R_matrix)
    print('\n')

    index = 1

    command = np.array( [1, 1] )

    """while ( index < 4 ):
        command = np.vstack( [ command, [1, 0, 0, 0, 0, 1] ] )
        index = index + 1"""

    print(command)
    print('\n')

    time_step = 5

    result = process_model(pose_aux, time_step, R_matrix, command)

    print( result )

    #def cost_function(commands, *args) #= (pose, hor, ctrl_weight, rot_matrix, time_step)):
    #result = cost_function(command, pose_aux, 4, 0.2, R_matrix, 5)