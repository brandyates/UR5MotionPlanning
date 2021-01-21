#Created for CAP 4662. setup code by Tianze Chen, Number 1 and supporting functions by Brandon Yates
import sys
import numpy as np
sys.path.append('PythonAPI')
import math
import time

class Node:
    def __init__(self, data):
        self.left = None
        self.right = None
        self.data = data

    def PrintTree(self):
        print(self.data)

try:    
    import sim
except:    
    print ('--------------------------------------------------------------')    
    print ('"sim.py" could not be imported. This means very probably that')   
    print ('either "sim.py" or the remoteApi library could not be found.')   
    print ('Make sure both are in the same folder as this file,')    
    print ('or appropriately adjust the file "sim.py"')    
    print ('--------------------------------------------------------------')    
    print ('')
    
print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:    
    print ('Connected to remote API server')
else:    
    print ('Failed connecting to remote API server')    
    sys.exit('Could not connect to Vrep')

# get the handles of arm joints
err_code, armjoint1_handle = sim.simxGetObjectHandle(clientID,"UR5_joint1", sim.simx_opmode_blocking)
err_code, armjoint2_handle = sim.simxGetObjectHandle(clientID,"UR5_joint2", sim.simx_opmode_blocking)
err_code, armjoint3_handle = sim.simxGetObjectHandle(clientID,"UR5_joint3", sim.simx_opmode_blocking)
err_code, armjoint4_handle = sim.simxGetObjectHandle(clientID,"UR5_joint4", sim.simx_opmode_blocking)
err_code, armjoint5_handle = sim.simxGetObjectHandle(clientID,"UR5_joint5", sim.simx_opmode_blocking)
err_code, armjoint6_handle = sim.simxGetObjectHandle(clientID,"UR5_joint6", sim.simx_opmode_blocking)
# get the handles of hand joints
err_code, endeffector_handle = sim.simxGetObjectHandle(clientID,"suctionPad", sim.simx_opmode_blocking)

# set the arm to position control
sim.simxSetObjectIntParameter(clientID, armjoint1_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint1_handle, 2001, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint2_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint2_handle, 2001, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint3_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint3_handle, 2001, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint4_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint4_handle, 2001, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint5_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint5_handle, 2001, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint6_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint6_handle, 2001, 1, sim.simx_opmode_oneshot)

# get the collision handles
collision_handle_list = []
for i in range(40):    
    err_code, collision_handle = sim.simxGetCollisionHandle(clientID, "Collision" +str(i), sim.simx_opmode_blocking)
    sim.simxReadCollision(clientID, collision_handle, sim.simx_opmode_streaming)    
    collision_handle_list.append(collision_handle)
    
# You do not need to modify the code above


# function to control the movement of the arm, the input are the angles of joint1, joint2, joint3, joint4, joint5, joint6. The unit are in degrees
def move_arm(armpose):    
    armpose_convert = []    
    for i in range(6):        
        armpose_convert.append(round(armpose[i]/180 * math.pi,3))    
    sim.simxPauseCommunication(clientID,True)    
    sim.simxSetJointTargetPosition(clientID, armjoint1_handle, armpose_convert[0], sim.simx_opmode_oneshot)    
    sim.simxSetJointTargetPosition(clientID, armjoint2_handle, armpose_convert[1], sim.simx_opmode_oneshot)    
    sim.simxSetJointTargetPosition(clientID, armjoint3_handle, armpose_convert[2], sim.simx_opmode_oneshot)    
    sim.simxSetJointTargetPosition(clientID, armjoint4_handle, armpose_convert[3], sim.simx_opmode_oneshot)    
    sim.simxSetJointTargetPosition(clientID, armjoint5_handle, armpose_convert[4], sim.simx_opmode_oneshot)     
    sim.simxSetJointTargetPosition(clientID, armjoint6_handle, armpose_convert[5], sim.simx_opmode_oneshot)        
    sim.simxPauseCommunication(clientID,False)    
    time.sleep(0.1)
        
# function to check collision
def check_collision():    
    collision_reading = np.zeros(40)    
    is_collision = 0    
    for i in range(40):        
        collision_reading[i] = sim.simxReadCollision(clientID, collision_handle_list[i], sim.simx_opmode_buffer)[1]        
        if collision_reading[i] == 1:            
            is_collision = 1    
    if is_collision == 1:        
        print('Collision detected!')
        return 1    
    else:        
        return 0


#closest neighbor
def near(xrand, root):
    current = root
    stack = []
    closest = root
    dist = 1000
    while True:
        if current is not None:
            stack.append(current)
            if abs(xrand[1] - current.data[1]) < dist:
                dist = abs(xrand[1] - current.data[1])
                closest = current
            current = current.left
        elif(stack):
            current = stack.pop()
            current = current.right
        else:
            break
    
    return closest



def direction(xnear, xrand):
        u = (xrand[1] - xnear.data[1]) / (np.sqrt(xrand[1]**2 - xnear.data[1]**2))
        return u 
    
def no_collide(xnew, xnear):
    a = xnew[1]
    a = round(a)
    b = xnear.data
    if a < b[1]:
        while b[1] >= a:
            if check_collision == 1:
                return False
            move_arm(b)
            new_theta = b[1] - 1
            b = [new_theta, new_theta, new_theta, new_theta, new_theta, new_theta]
        return True
    else:
        while b[1] <= a:
            if check_collision == 1:
                return False
            move_arm(b)
            new_theta = b[1] + 1
            b = [new_theta, new_theta, new_theta, new_theta, new_theta, new_theta]
        return True
    return True



#number 1: RRT
def rrt(xinit, xend, root):
    t = 5
    for i in range(100):
        theta = np.random.randint(-70, 70)
        xrand = [theta, theta, theta, theta, theta, theta] #getting x rand
        move_arm(xrand)
        if check_collision() == 0: #check if xrand is in free space
            xnear = near(xrand, root) #get nearest neighbor in tree
            u = direction(xnear, xrand) #unit vector
            dx = u*t
            xnew = [xnear.data[1] + dx] * 6#t-step size
            clear  = no_collide(xnew, xnear)
            if clear == True: #check collision between x new and x near
                node = Node(xnew)
                if abs(node.data[1] - xend[1]) < t:
                    end = Node(xend)
                    xnear.right = node
                    node.right = end
                    return root
                elif node.data > xnear.data:
                    xnear.right = node
                else:
                    xnear.left = node

def printPath(path):
    if path is None:
        return
    queue = []
    queue.append(path)
    while(len(queue) > 0):
        print(queue[0].data)
        curr = queue.pop(0)
        if curr.left is not None:
            queue.append(curr.left)
        if curr.right is not None:
            queue.append(curr.right)
    
xinit = [0, 0, 0, 0, 0, 0]
move_arm(xinit)
xend = [-20, -20, -20, -20, -20, -20]
root = Node(xinit)
path = rrt(xinit, xend, root)   
print("Path is from " + str(xinit) + " to " + str(xend))
print("FULL TREE (LEVEL ORDER):")
printPath(path)
    

        
    

# no need to modify the code below
# # close the communication between collision handles
for i in range(40):    
    sim.simxReadCollision(clientID, collision_handle_list[i], sim.simx_opmode_discontinue)
    


print ('Program ended')