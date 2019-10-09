import numpy as np
import time
from numpy.linalg import inv

"""
TODO: build a trajectory generator and waypoint planner
        so it allows your state machine to iterate through
        the plan at the desired command update rate
"""

class TrajectoryPlanner():
    def __init__(self, rexarm):
        self.idle = True
        self.rexarm = rexarm
        self.num_joints = rexarm.num_joints
        self.initial_wp = [0.0]*self.num_joints
        self.final_wp = [0.0]*self.num_joints
        self.dt = 0.05 # command rate

    def set_initial_wp(self):
        return self.rexarm.get_positions()[:]


    def set_final_wp(self, waypoint):
        return waypoint

    def go(self, final_wp, max_speed = 2.5):
        
        initial_wp = self.set_initial_wp()
        # final_wp = self.set_final_wp([-1.0,-0.8,-1.0,-0.5, -1.0,0])
        T = self.calc_time_from_waypoints(initial_wp,final_wp,max_speed)
        # self.rexarm.pause(2)
        # n = int(T*200)
        n = 2
        # print(T)
        q,q_dot = self.generate_quintic_spline(initial_wp,final_wp,T,n)
        # print(q,q_dot)
        t =time.time()
        for i in range(len(q)):
            # print(i)
            # t1=time.time()
            if i==0:
                continue
            self.rexarm.set_speeds((q_dot[i]+q_dot[i-1])/2)
            self.rexarm.set_positions(q[i])
            # dt = time.time() - t1
            if (i>0 and q_dot[i,1] != 0):
                # print((q[i]-q[i-1])/q_dot[i])
                tx=min((q[i]-q[i-1])/((q_dot[i]+q_dot[i-1])/2))
                # print(tx,q[i])
                self.rexarm.pause(tx)
                
            # angles = [ 0.0, 0.0, 0.0, 0.0, q[i]]
            # while True:
            #     current_angles = self.rexarm.get_positions()[:]
            #     diff = sum([(current_angles[i] - angles[i])**2 for i in range(5)])
            #     # print(diff)
            #     # print(current_angles, angles)
            #     if diff < 0.01:
            #         break
            #     time.sleep(T/n/2)
            #     self.rexarm.set_positions(angles)
            # self.rexarm.set_positions()
            # print(diff)
            # print(current_angles, angles)
        # print(time.time()-t)
        self.rexarm.set_speeds([2.5 for i in range(6)])

    def stop(self):
        pass

    def calc_time_from_waypoints(self, initial_wp, final_wp, max_speed):
        return max([abs(final_wp[i]-initial_wp[i]) for i in range(len(final_wp))]) / max_speed


    def generate_quintic_spline(self, initial_wp, final_wp, T,n):
        # waypoints = [0 70 25 90 35 80 0]
        q0 = initial_wp
        qf = final_wp
        t0 = 0
        tf = T
        t_vector = np.array(range(n+1))*tf/n
        # print(t_vector)
        q=np.zeros((len(t_vector),6))
        q_dot=np.zeros((len(t_vector),6))
        for j in range(6):
            M = np.array([[1,t0,t0**2,t0**3,t0**4,t0**5],
            [0,1,2*t0,3*t0**2,4*t0**3,5*t0**4],
            [0,0,2,6*t0,12*t0**2,20*t0**3],
            [1,tf,tf**2,tf**3,tf**4,tf**5],
            [0,1,2*tf,3*tf**2,4*tf**3,5*tf**4],
            [0,0,2,6*tf,12*tf**2,20*tf**3]])
            b = np.array([q0[j],0,0,qf[j],0,0]).transpose()
            a = np.dot(inv(M),b)
            # print("b_matrix"),b
            for i in range(len(t_vector)):
                q[i,j] = a[0]+ a[1]*t_vector[i] + a[2]*t_vector[i]**2 + a[3]*t_vector[i]**3 + a[4]*t_vector[i]**4 +a[5]*t_vector[i]**5
                q_dot[i,j] = a[1] + 2*a[2]*t_vector[i] + 3*a[3]*t_vector[i]**2 + 4*a[4]*t_vector[i]**3 + 5*a[5]*t_vector[i]**4
        # print(q)
        # print(q_dot)
        # q_dot[0]=q_dot[1]
        return q, q_dot
    
    
    def generate_cubic_spline(self, initial_wp, final_wp, T,n):
        # waypoints = [0 70 25 90 35 80 0]

        q0 = initial_wp
        qf = final_wp
        t0 = 0
        tf = T
        t_vector = np.array(range(n))*tf/n
        q=np.zeros((len(t_vector),6))
        q_dot=np.zeros((len(t_vector),6))
        for j in range(6):
            M = np.array([[1,t0,t0**2,t0**3],[0,1,2*t0,3*t0**2],[1,tf,tf**2,tf**3],[0,1,2*tf,3*tf**2]])
            b = np.array([q0[j],0,qf[j],0]).transpose()
            a = np.dot(inv(M),b)
            for i in range(len(t_vector)):
                q[i,j] = a[0]+ a[1]*t_vector[i] + a[2]*t_vector[i]**2 + a[3]*t_vector[i]**3
                q_dot[i,j] = a[1] + 2*a[2]*t_vector[i] + 3*a[3]*t_vector[i]**2
        # print(q)
        # print(q_dot)
        # q_dot[0]=q_dot[1]
        return q, q_dot

    def execute_plan(self, plan, look_ahead=8):
        pass