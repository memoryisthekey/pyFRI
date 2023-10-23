



"""
Joint Configuration safety check
===================
A simple function to check if the delta between the current configuration and the goal configuration 
is below a certain value (TBD) to avoid configuration jumps
"""

def checkQ(current_q, goal_q, max_step=0.0174533):

    for i in range(len(current_q)):
        if(abs(current_q[i] - goal_q[i]) > max_step):
            return False
    
    return True