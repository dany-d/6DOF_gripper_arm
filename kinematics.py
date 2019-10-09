import numpy as np
#expm is a matrix exponential function
from scipy.linalg import expm

""" 
TODO: Here is where you will write all of your kinematics functions 
There are some functions to start with, you may need to implement a few more

"""

def FK_mat(a,d,alpha,theta_off,joint_angle):
    return np.array([[np.cos(theta_off+joint_angle),-np.sin(joint_angle+theta_off)*np.cos(alpha),np.sin(joint_angle+theta_off)*np.sin(alpha),a*np.cos(joint_angle+theta_off)],
    [np.sin(joint_angle+theta_off),np.cos(joint_angle+theta_off)*np.cos(alpha),-np.cos(joint_angle+theta_off)*np.sin(alpha),a*np.sin(joint_angle+theta_off)],
    [0,np.sin(alpha),np.cos(alpha),d],
    [0,0,0,1]])



def FK_dh(joint_angles, link):
    # link = np.array([0,117.5,100.5,113,94.5])
    a = [0,-link[1],0,0,0,0]
    d = [0,0,0,link[2],0,0]
    alpha = [-np.pi/2,0,np.pi/2,-np.pi/2,-np.pi/2,0]
    # theta_off = [0,-np.pi/2,-np.pi/2,0,np.pi/2]
    theta_off = [0,np.pi/2,-np.pi/2,0,0,0] # check for theta_offset on joint2  - need to correct it!!!!
    hom_mat= np.eye(4,4)
    # print(np.array(joint_angles) / np.pi * 180)
    nq = len(joint_angles)-2 
    for i in range(nq+1):
       
        hom_mat = np.dot(FK_mat(a[nq-i],d[nq-i],alpha[nq-i],theta_off[nq-i],joint_angles[nq-i]),hom_mat)
       

    return hom_mat
    """
    TODO: implement this function

    Calculate forward kinematics for rexarm unp.sing DH convention

    return a transformation matrix representing the pose of the 
    desired link

    note: pi is the euler angle about the y-axis in the base frame

    """
    pass

def FK_dh_new(joint_angles, link):
    # link = np.array([0,117.5,100.5,113,108])
    a = [0,link[1],0,0,0,0]
    d = [0,0,0,link[2],0,0]
    alpha = [np.pi/2,0,np.pi/2,-np.pi/2,np.pi/2,0]
    # theta_off = [0,-np.pi/2,-np.pi/2,0,np.pi/2]
    theta_off = [0,0,np.pi/2,0,0,0] # check for theta_offset on joint2  - need to correct it!!!!
    hom_mat= np.eye(4,4)
    # print(joint_angles)
    joint_angles[1] = joint_angles[1]+np.pi/2 # adjusting to the gui offset
    # print(np.array(joint_angles) / np.pi * 180)

    nq = len(joint_angles)-2 
    for i in range(3):
        
        hom_mat = hom_mat.dot(FK_mat(a[i],d[i],alpha[i],theta_off[i],joint_angles[i]))

    return hom_mat
    """
    TODO: implement this function

    Calculate forward kinematics for rexarm unp.sing DH convention

    return a transformation matrix representing the pose of the 
    desired link

    note: pi is the euler angle about the y-axis in the base frame

    """
    pass

def FK_pox(joint_angles):



    """
    TODO: implement this function

    Calculate forward kinematics for rexarm
    unp.sing product of exponential formulation

    return a 4-tuple (x, y, z, pi) representing the pose of the 
    desired link

    note: pi is the euler angle about y in the base frame

    """
    pass

def IK(rexarm,pose):
    l1 = 100.5
    l2=113
    
    [x,y,z]  = [pose[0],pose[1],pose[2]]

    theta0 = np.arctan2(y,x)
    # if theta0 < np.pi:
    #     theta0 = theta0 + np.pi
    # else:
    #     theta0 = theta0 - np.pi

    cos = (z**2+y**2+x**2- l2**2 - l1**2)/2/l1/l2
    if (abs(cos) > 1):
        print('cos>1 impossible')
        return None
    
    theta2 = -np.arccos((z**2+y**2+x**2- l2**2 - l1**2)/2/l1/l2)
    # theta1 = np.arctan2(z,r)- np.arctan2(l2*np.sin(theta2),l1+l2*np.cos(theta2)) - np.pi /2
    theta1 = np.arctan2(z,np.sqrt(x**2+y**2))- np.arctan2(l2*np.sin(theta2),l1+l2*np.cos(theta2)) - np.pi /2

    if theta1 > rexarm.angle_limits[1][1] or theta1 < rexarm.angle_limits[1][0]:
        print('reach shoulder joint limits')
        theta2 = -theta2
        theta1 = np.arctan2(z,np.sqrt(x**2+y**2))- np.arctan2(l2*np.sin(theta2),l1+l2*np.cos(theta2)) - np.pi /2

    if theta2 > rexarm.angle_limits[2][1] or theta2 < rexarm.angle_limits[2][0]:
        print('reach elbow joint limits')
        return None

    theta1=-theta1
    theta2=-theta2

    np.set_printoptions(suppress=True)

    print(theta0/3.14*180,theta1/3.14*180,theta2/3.14*180)
    print(FK_dh([theta0,theta1,theta2,0,0],np.array([0,117.5,100.5,113,108]))[:,-1])

    # rexarm.set_positions([theta0,theta1,theta2,0,0])

    return [theta0,theta1,theta2,0,0]
    
    """
    TODO: implement this function

    Calculate inverse kinematics for rexarm

    return the required joint angles

    """
    pass



def IK2(rexarm,pose):
    l2 = 100.5
    l3 = 113
    
    [x,y,z]  = [pose[0],pose[1],pose[2]]

    r = np.sqrt(x**2 + y**2)
    theta0 = np.arctan2(y,x)
    cos = ((r**2 + z**2) - l3**2 - l2**2)/(2*l2*l3)
    if (abs(cos) > 1):
        print('cos>1 impossible')
        return
    
    theta2 = - np.arccos(((r**2 + z**2) - l3**2 - l2**2)/(2*l2*l3)) #condition for elbow up or down
    # theta2_2 = -theta2
    
    theta1 = np.arctan2(z,r) - np.arctan2(l3*np.sin(theta2),l2+l3*np.cos(theta2))
    # theta1_2 = np.arctan2(z,r) - np.arctan2(l3*np.sin(theta2_2),l2+l3*np.cos(theta2_2))

    if theta1 > rexarm.angle_limits[1][1] or theta1 < rexarm.angle_limits[1][0]:
        print('reach shoulder joint limits')
        theta2 = -theta2
        theta1 = np.arctan2(z,np.sqrt(x**2+y**2))- np.arctan2(l2*np.sin(theta2),l2+l3*np.cos(theta2)) - np.pi /2

    if theta2 > rexarm.angle_limits[2][1] or theta2 < rexarm.angle_limits[2][0]:
        print('reach elbow joint limits')
        return

    np.set_printoptions(suppress=True)

    return([theta0,theta1-np.pi/2,theta2,0,0])



    """
    TODO: implement this function

    Calculate inverse kinematics for rexarm

    return the required joint angles

    """
    pass

# def wrist_angles(rexarm,end_pos,orientation,fromtop = True, current_angles = None, i =None):
#     # wrist_len = 94.5
#     # if current_angles == None:
#     #     current_angles = rexarm.get_positions()[:]
#     wrist_len = 141-19
#     link = np.array([117.5,100.5,113,108,0])
#     end_pos = np.array(end_pos)

#     phi =np.pi
#     psi = orientation





#     theta_mat = None

#     if i == None:
#         for i in range(4):
#             print(psi)
#             rot_y = np.array([[np.cos(phi),0,np.sin(phi)],[0,1,0],[-np.sin(phi),0,np.cos(phi)]])
#             rot_z = np.array([[np.cos(psi),-np.sin(psi),0],[np.sin(psi),np.cos(psi),0],[0,0,1]])
#             rot_x = np.array([[1,0,0],[0,np.cos(psi),-np.sin(psi)],[0,np.sin(psi),np.cos(psi)]])
#             # R_mat06 =np.array([[-1,0,0],[0,1,0],[0,0,-1]])
#             if fromtop == True:
#                 end_pos[0] += 20
#                 # end_pos[1] -=5

#                 R_mat06 = rot_y.dot(rot_z)
#                 wrist_pos = end_pos.transpose() - wrist_len*R_mat06[:,-1] + 10 * R_mat06[:,1]
#                 # print ("wrist_pos",wrist_pos)
#                 theta_mat = IK2(rexarm,wrist_pos)
#             elif fromtop == False:
#                 print('x')
#                 end_pos[0] -= 0
               
#                 phi = np.pi/2
#                 rot_y = np.array([[np.cos(phi),0,np.sin(phi)],[0,1,0],[-np.sin(phi),0,np.cos(phi)]])
#                 R_mat06 = rot_y.dot(rot_x)
#                 wrist_pos = end_pos.transpose() - (wrist_len+10)*R_mat06[:,-1] + 5 * R_mat06[:,1]
#                 # print ("wrist_pos",wrist_pos)
#                 theta_mat = IK2(rexarm,wrist_pos)
#             # if not theta_mat:
#             #     phi = -np.pi/2
#             #     rot_y = np.array([[np.cos(phi),0,np.sin(phi)],[0,1,0],[-np.sin(phi),0,np.cos(phi)]])
#             #     R_mat06 = rot_y.dot(rot_x)
#             #     wrist_pos = end_pos.transpose() - wrist_len*R_mat06[:,-1]
#             #     # print ("wrist_pos",wrist_pos)
#             #     theta_mat = IK2(rexarm,wrist_pos)
                
#             if theta_mat:
#                 break
#             psi += np.pi / 2
#     else: 
#         psi = np.pi * i + orientation
#         rot_y = np.array([[np.cos(phi),0,np.sin(phi)],[0,1,0],[-np.sin(phi),0,np.cos(phi)]])
#         rot_z = np.array([[np.cos(psi),-np.sin(psi),0],[np.sin(psi),np.cos(psi),0],[0,0,1]])
#         rot_x = np.array([[1,0,0],[0,np.cos(psi),-np.sin(psi)],[0,np.sin(psi),np.cos(psi)]])
#         # R_mat06 =np.array([[-1,0,0],[0,1,0],[0,0,-1]])
#         if fromtop == True:
#             end_pos[0] += 20
#                 # end_pos[1] -=5
#             # end_pos[0] += 10
#             # end_pos[1]
#             R_mat06 = rot_y.dot(rot_z)
#             wrist_pos = end_pos.transpose() - wrist_len*R_mat06[:,-1] + 10 * R_mat06[:,1]
#             # print ("wrist_pos",wrist_pos)
#             theta_mat = IK2(rexarm,wrist_pos)
#         elif fromtop == False:
#             # end_pos[0] += 30
#             # end_pos[1] -=5
#             # print('x')
#             # end_pos[0] += 10
#             # end_pos[1] -= 5
#             phi = np.pi/2
#             rot_y = np.array([[np.cos(phi),0,np.sin(phi)],[0,1,0],[-np.sin(phi),0,np.cos(phi)]])
#             R_mat06 = rot_y.dot(rot_x)
#             wrist_pos = end_pos.transpose() - (wrist_len+20)*R_mat06[:,-1] + 10 * R_mat06[:,1]
#             # print ("wrist_pos",wrist_pos)
#             theta_mat = IK2(rexarm,wrist_pos)
#         # if not theta_mat:
#         #     phi = -np.pi/2
#         #     rot_y = np.array([[np.cos(phi),0,np.sin(phi)],[0,1,0],[-np.sin(phi),0,np.cos(phi)]])
#         #     R_mat06 = rot_y.dot(rot_x)
#         #     wrist_pos = end_pos.transpose() - wrist_len*R_mat06[:,-1]
#         #     # print ("wrist_pos",wrist_pos)
#         #     theta_mat = IK2(rexarm,wrist_pos)
#     # print(theta_mat)
#     if theta_mat:
#         theta_mat = np.array(theta_mat)
#         R_mat03 = FK_dh_new(theta_mat.copy(),link)[:-1,:-1]
#         R_mat36 = np.dot(R_mat03.transpose(),R_mat06)
#         # print ("theta_mat",theta_mat)
#         # print(R_mat03)
#         # print(R_mat36)


#         theta4 = np.arctan2(R_mat36[1][2],R_mat36[0][2])
#         theta5 = np.arctan2(np.sqrt(1-(R_mat36[2][2])**2),R_mat36[2][2]) #TODO add + - for atan2
#         if (theta4 < rexarm.angle_limits[4][0]):
#             theta4 = np.pi + theta4
#             theta5 = - theta5

#         if (theta4 > rexarm.angle_limits[4][1]):
#             theta4 = -np.pi + theta4
#             theta5 = - theta5
        

#         theta1 = theta_mat[0]
#         theta2 = theta_mat[1]
#         theta3 = theta_mat[2]
#         # print(theta2)

#         theta2 = theta2 - 5./180*np.pi * np.sign(theta2)

#         # print(theta2)
#         theta6 = np.arctan2(R_mat36[2][1],-R_mat36[2][0])
#         print(theta6)
#         # print(np.array([current_angles[5],theta6])*180/np.pi)
#         if theta6 > 2 * np.pi / 3 and (not current_angles or np.abs(current_angles[5] - theta6) > np.pi/2):
#         # if theta6 > 2 * np.pi / 3 and np.abs(current_angles[5] - theta6) > np.abs(current_angles[5] - theta6 + np.pi) :
#             theta6 = theta6 - np.pi
#         elif theta6 < -2 * np.pi / 3 and (not current_angles or np.abs(current_angles[5] - theta6) > np.pi/2):
#         # elif theta6 < -2 * np.pi / 3 and np.abs(current_angles[5] - theta6) > np.abs(current_angles[5] - theta6 - np.pi) :
#             theta6 = theta6 + np.pi
#         # print(np.array([current_angles[5],theta6])*180/np.pi)
#         # print(theta_mat)
#         joint_angles = [theta1,theta2,theta3,theta4,theta5,theta6]
#         # return [theta4,theta5,theta6]
#         return joint_angles
#     else:
#         return


def wrist_angles(rexarm,end_pos,orientation,fromtop = True, current_angles = None, i =None):
    # wrist_len = 94.5
    # if current_angles == None:
    #     current_angles = rexarm.get_positions()[:]
    wrist_len = 141-19
    link = np.array([117.5,100.5,113,108,0])
    end_pos = np.array(end_pos)

    phi =np.pi
    psi = orientation

    theta_mat = None

    if i == None:
        for i in range(4):
            print(psi)
            rot_y = np.array([[np.cos(phi),0,np.sin(phi)],[0,1,0],[-np.sin(phi),0,np.cos(phi)]])
            rot_z = np.array([[np.cos(psi),-np.sin(psi),0],[np.sin(psi),np.cos(psi),0],[0,0,1]])
            rot_x = np.array([[1,0,0],[0,np.cos(psi),-np.sin(psi)],[0,np.sin(psi),np.cos(psi)]])
            # R_mat06 =np.array([[-1,0,0],[0,1,0],[0,0,-1]])
            if fromtop == True:
                end_pos[0] += 10
                end_pos[1] -= 5

                R_mat06 = rot_y.dot(rot_z)
                wrist_pos = end_pos.transpose() - wrist_len*R_mat06[:,-1] + 10 * R_mat06[:,1]
                # print ("wrist_pos",wrist_pos)
                theta_mat = IK2(rexarm,wrist_pos)
            elif fromtop == False:
                print('x')
                end_pos[0] += 10
                end_pos[1] -= 5
               
                phi = np.pi/2
                rot_y = np.array([[np.cos(phi),0,np.sin(phi)],[0,1,0],[-np.sin(phi),0,np.cos(phi)]])
                R_mat06 = rot_y.dot(rot_x)
                wrist_pos = end_pos.transpose() - (wrist_len+10)*R_mat06[:,-1] + 5 * R_mat06[:,1]
                # print ("wrist_pos",wrist_pos)
                theta_mat = IK2(rexarm,wrist_pos)
            # if not theta_mat:
            #     phi = -np.pi/2
            #     rot_y = np.array([[np.cos(phi),0,np.sin(phi)],[0,1,0],[-np.sin(phi),0,np.cos(phi)]])
            #     R_mat06 = rot_y.dot(rot_x)
            #     wrist_pos = end_pos.transpose() - wrist_len*R_mat06[:,-1]
            #     # print ("wrist_pos",wrist_pos)
            #     theta_mat = IK2(rexarm,wrist_pos)
                
            if theta_mat:
                break
            psi += np.pi / 2
    else: 
        psi = np.pi * i + orientation
        rot_y = np.array([[np.cos(phi),0,np.sin(phi)],[0,1,0],[-np.sin(phi),0,np.cos(phi)]])
        rot_z = np.array([[np.cos(psi),-np.sin(psi),0],[np.sin(psi),np.cos(psi),0],[0,0,1]])
        rot_x = np.array([[1,0,0],[0,np.cos(psi),-np.sin(psi)],[0,np.sin(psi),np.cos(psi)]])
        # R_mat06 =np.array([[-1,0,0],[0,1,0],[0,0,-1]])
        if fromtop == True:
            # end_pos[0] += 10
            # end_pos[1]
            R_mat06 = rot_y.dot(rot_z)
            wrist_pos = end_pos.transpose() - wrist_len*R_mat06[:,-1] + 10 * R_mat06[:,1]
            # print ("wrist_pos",wrist_pos)
            theta_mat = IK2(rexarm,wrist_pos)
        elif fromtop == False:
            # print('x')
            # end_pos[0] += 10
            # end_pos[1] -= 5
            phi = np.pi/2
            rot_y = np.array([[np.cos(phi),0,np.sin(phi)],[0,1,0],[-np.sin(phi),0,np.cos(phi)]])
            R_mat06 = rot_y.dot(rot_x)
            wrist_pos = end_pos.transpose() - (wrist_len+20)*R_mat06[:,-1] + 10 * R_mat06[:,1]
            # print ("wrist_pos",wrist_pos)
            theta_mat = IK2(rexarm,wrist_pos)

        theta_mat = np.array(theta_mat)
        R_mat03 = FK_dh_new(theta_mat.copy(),link)[:-1,:-1]
        R_mat36 = np.dot(R_mat03.transpose(),R_mat06)

        theta4 = np.arctan2(R_mat36[1][2],R_mat36[0][2])
        theta5 = np.arctan2(np.sqrt(1-(R_mat36[2][2])**2),R_mat36[2][2]) #TODO add + - for atan2
        if (theta4 < rexarm.angle_limits[4][0]):
            theta4 = np.pi + theta4
            theta5 = - theta5

        if (theta4 > rexarm.angle_limits[4][1]):
            theta4 = -np.pi + theta4
            theta5 = - theta5
        

        theta1 = theta_mat[0]
        theta2 = theta_mat[1]
        theta3 = theta_mat[2]
        # print(theta2)

        theta2 = theta2 - 5./180*np.pi * np.sign(theta2)

        # print(theta2)
        theta6 = np.arctan2(R_mat36[2][1],-R_mat36[2][0])
        print(theta6)
        # print(np.array([current_angles[5],theta6])*180/np.pi)
        if theta6 > 2 * np.pi / 3 and (not current_angles or np.abs(current_angles[5] - theta6) > np.pi/2):
        # if theta6 > 2 * np.pi / 3 and np.abs(current_angles[5] - theta6) > np.abs(current_angles[5] - theta6 + np.pi) :
            theta6 = theta6 - np.pi
        elif theta6 < -2 * np.pi / 3 and (not current_angles or np.abs(current_angles[5] - theta6) > np.pi/2):
        # elif theta6 < -2 * np.pi / 3 and np.abs(current_angles[5] - theta6) > np.abs(current_angles[5] - theta6 - np.pi) :
            theta6 = theta6 + np.pi
        # print(np.array([current_angles[5],theta6])*180/np.pi)
        # print(theta_mat)
        joint_angles = [theta1,theta2,theta3,theta4,theta5,theta6]
        # return [theta4,theta5,theta6]
        return joint_angles
    else:
        return


def wrist_angles_task5(rexarm,end_pos,orientation, theta, current_angles = None, i =None):

    wrist_len = 141-19
    link = np.array([117.5,100.5,113,108,0])
    end_pos = np.array(end_pos)

    phi =np.pi
    psi = orientation

    angles = [0.007677401401759543, 0.22776290825219947, 0.06397834501466271]

    theta_mat = None

    if i == None:
        for i in range(4):
            print(psi)
            phi = 3*np.pi/4
            rot_x = np.array([[1,0,0],[0,np.cos(psi),-np.sin(psi)],[0,np.sin(psi),np.cos(psi)]])
            rot_y = np.array([[np.cos(phi),0,np.sin(phi)],[0,1,0],[-np.sin(phi),0,np.cos(phi)]])
            rot_z1 = np.array([[np.cos(psi),-np.sin(psi),0],[np.sin(psi),np.cos(psi),0],[0,0,1]])
            rot_z2 = np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
            R_mat06 = rot_z2.dot(rot_y.dot(rot_z1))
            wrist_pos = end_pos.transpose() - wrist_len*R_mat06[:,-1]
            # print ("wrist_pos",wrist_pos)
            theta_mat = IK2(rexarm,wrist_pos)
        # if not theta_mat:
        #     phi = -np.pi/2
        #     rot_y = np.array([[np.cos(phi),0,np.sin(phi)],[0,1,0],[-np.sin(phi),0,np.cos(phi)]])
        #     R_mat06 = rot_y.dot(rot_x)
        #     wrist_pos = end_pos.transpose() - wrist_len*R_mat06[:,-1]
        #     # print ("wrist_pos",wrist_pos)
        #     theta_mat = IK2(rexarm,wrist_pos)
                
            if theta_mat:
                break
            psi += np.pi / 2
    else: 
        psi = np.pi * i + orientation
        phi = 3*np.pi/4
        rot_y = np.array([[np.cos(phi),0,np.sin(phi)],[0,1,0],[-np.sin(phi),0,np.cos(phi)]])
        rot_z1 = np.array([[np.cos(psi),-np.sin(psi),0],[np.sin(psi),np.cos(psi),0],[0,0,1]])
        rot_z2 = np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
        R_mat06 = rot_z2.dot(rot_y.dot(rot_z1))
        wrist_pos = end_pos.transpose() - wrist_len*R_mat06[:,-1]
        # print ("wrist_pos",wrist_pos)
        theta_mat = IK2(rexarm,wrist_pos)
        # if not theta_mat:
        #     phi = -np.pi/2
        #     rot_y = np.array([[np.cos(phi),0,np.sin(phi)],[0,1,0],[-np.sin(phi),0,np.cos(phi)]])
        #     R_mat06 = rot_y.dot(rot_x)
        #     wrist_pos = end_pos.transpose() - wrist_len*R_mat06[:,-1]
        #     # print ("wrist_pos",wrist_pos)
        #     theta_mat = IK2(rexarm,wrist_pos)
    # print(theta_mat)
    if theta_mat:
        theta_mat = np.array(theta_mat)
        R_mat03 = FK_dh_new(theta_mat.copy(),link)[:-1,:-1]
        R_mat36 = np.dot(R_mat03.transpose(),R_mat06)
        # print ("theta_mat",theta_mat)
        # print(R_mat03)
        # print(R_mat36)


        theta4 = np.arctan2(R_mat36[1][2],R_mat36[0][2])
        theta5 = np.arctan2(np.sqrt(1-(R_mat36[2][2])**2),R_mat36[2][2]) #TODO add + - for atan2
        if (theta4 < rexarm.angle_limits[4][0]):
            theta4 = np.pi + theta4
            theta5 = - theta5

        if (theta4 > rexarm.angle_limits[4][1]):
            theta4 = -np.pi + theta4
            theta5 = - theta5
        

        theta1 = theta_mat[0]
        theta2 = theta_mat[1]
        theta3 = theta_mat[2]
        # print(theta2)

        theta2 = theta2 - 5./180*np.pi * np.sign(theta2)

        # print(theta2)
        theta6 = np.arctan2(R_mat36[2][1],-R_mat36[2][0])
        print(theta6)
        # print(np.array([current_angles[5],theta6])*180/np.pi)
        if theta6 > 2 * np.pi / 3 and (not current_angles or np.abs(current_angles[5] - theta6) > np.pi/2):
        # if theta6 > 2 * np.pi / 3 and np.abs(current_angles[5] - theta6) > np.abs(current_angles[5] - theta6 + np.pi) :
            theta6 = theta6 - np.pi
        elif theta6 < -2 * np.pi / 3 and (not current_angles or np.abs(current_angles[5] - theta6) > np.pi/2):
        # elif theta6 < -2 * np.pi / 3 and np.abs(current_angles[5] - theta6) > np.abs(current_angles[5] - theta6 - np.pi) :
            theta6 = theta6 + np.pi
        # print(np.array([current_angles[5],theta6])*180/np.pi)
        # print(theta_mat)
        joint_angles = [theta1,theta2,theta3,theta4,theta5,theta6]
        # return [theta4,theta5,theta6]
        return joint_angles
    else:
        return

def wrist_angles_task5_wp(rexarm,end_pos,orientation, theta, d, current_angles = None, i =None):
    # wrist_len = 94.5
    # if current_angles == None:
    #     current_angles = rexarm.get_positions()[:]
    wrist_len = 141-19
    link = np.array([117.5,100.5,113,108,0])
    end_pos = np.array(end_pos)

    phi =np.pi
    psi = orientation

    theta_mat = None

    if i == None:
        for i in range(4):
            print(psi)
            phi = 3*np.pi/4
            rot_x = np.array([[1,0,0],[0,np.cos(psi),-np.sin(psi)],[0,np.sin(psi),np.cos(psi)]])
            rot_y = np.array([[np.cos(phi),0,np.sin(phi)],[0,1,0],[-np.sin(phi),0,np.cos(phi)]])
            rot_z1 = np.array([[np.cos(psi),-np.sin(psi),0],[np.sin(psi),np.cos(psi),0],[0,0,1]])
            rot_z2 = np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
            R_mat06 = rot_z2.dot(rot_y.dot(rot_z1)) 
            wrist_pos = end_pos.transpose() - (wrist_len+d)*R_mat06[:,-1]
            # wrist_pos = wp1.transpose() - wrist_len*R_mat06[:,-1]
            # print ("wrist_pos",wrist_pos)
            
            theta_mat = IK2(rexarm,wrist_pos)

            if theta_mat:
                break
            psi += np.pi / 2
    else: 
        psi = np.pi * i + orientation
        phi = 3*np.pi/4
        rot_y = np.array([[np.cos(phi),0,np.sin(phi)],[0,1,0],[-np.sin(phi),0,np.cos(phi)]])
        rot_z1 = np.array([[np.cos(psi),-np.sin(psi),0],[np.sin(psi),np.cos(psi),0],[0,0,1]])
        rot_z2 = np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
        R_mat06 = rot_z2.dot(rot_y.dot(rot_z1))
        wrist_pos = end_pos.transpose() - wrist_len*R_mat06[:,-1]
        # print ("wrist_pos",wrist_pos)
        theta_mat = IK2(rexarm,wrist_pos)

    # print(theta_mat)
    if theta_mat:
        theta_mat = np.array(theta_mat)
        R_mat03 = FK_dh_new(theta_mat.copy(),link)[:-1,:-1]
        R_mat36 = np.dot(R_mat03.transpose(),R_mat06)
        # print ("theta_mat",theta_mat)
        # print(R_mat03)
        # print(R_mat36)


        theta4 = np.arctan2(R_mat36[1][2],R_mat36[0][2])
        theta5 = np.arctan2(np.sqrt(1-(R_mat36[2][2])**2),R_mat36[2][2]) #TODO add + - for atan2
        if (theta4 < rexarm.angle_limits[4][0]):
            theta4 = np.pi + theta4
            theta5 = - theta5

        if (theta4 > rexarm.angle_limits[4][1]):
            theta4 = -np.pi + theta4
            theta5 = - theta5
        

        theta1 = theta_mat[0]
        theta2 = theta_mat[1]
        theta3 = theta_mat[2]
        # print(theta2)

        theta2 = theta2 - 5./180*np.pi * np.sign(theta2)

        # print(theta2)
        theta6 = np.arctan2(R_mat36[2][1],-R_mat36[2][0])
        print(theta6)
        # print(np.array([current_angles[5],theta6])*180/np.pi)
        if theta6 > 2 * np.pi / 3 and (not current_angles or np.abs(current_angles[5] - theta6) > np.pi/2):
        # if theta6 > 2 * np.pi / 3 and np.abs(current_angles[5] - theta6) > np.abs(current_angles[5] - theta6 + np.pi) :
            theta6 = theta6 - np.pi
        elif theta6 < -2 * np.pi / 3 and (not current_angles or np.abs(current_angles[5] - theta6) > np.pi/2):
        # elif theta6 < -2 * np.pi / 3 and np.abs(current_angles[5] - theta6) > np.abs(current_angles[5] - theta6 - np.pi) :
            theta6 = theta6 + np.pi

        if theta4 > 2 * np.pi / 3 and (not current_angles or np.abs(current_angles[3] - theta4) > np.pi/2):
        # if theta6 > 2 * np.pi / 3 and np.abs(current_angles[5] - theta6) > np.abs(current_angles[5] - theta6 + np.pi) :
            theta4 = theta4 - np.pi
        elif theta4 < -2 * np.pi / 3 and (not current_angles or np.abs(current_angles[3] - theta4) > np.pi/2):
        # elif theta6 < -2 * np.pi / 3 and np.abs(current_angles[5] - theta6) > np.abs(current_angles[5] - theta6 - np.pi) :
            theta4 = theta4 + np.pi
        # print(np.array([current_angles[5],theta6])*180/np.pi)
        # print(theta_mat)
        joint_angles = [theta1,theta2,theta3,theta4,theta5,theta6]
        # return [theta4,theta5,theta6]
        return joint_angles
    else:
        return

def get_euler_angles_from_T(T):
    """
    TODO: implement this function
    return the Euler angles from a T matrix
    
    """
    pass

def get_pose_from_T(T):
    """
    TODO: implement this function
    return the joint pose from a T matrix
    of the form (x,y,z,pi) where pi is rotation about base frame y-axis
    
    """
    pass




def to_s_matrix(w,v):
    """
    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)
    """
    pass