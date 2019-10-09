import time
import numpy as np
from kinematics import IK, FK_dh, IK2,FK_dh_new, wrist_angles, wrist_angles_task5,wrist_angles_task5_wp
from trajectory_planner import TrajectoryPlanner
import cv2
from random import shuffle


"""
TODO: Add states and state functions to this class
        to implement all of the required logic for the armlab
"""
class StateMachine():
    def __init__(self, rexarm, planner, kinect):
        self.rexarm = rexarm
        self.tp = planner
        self.kinect = kinect
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"
        self.waypoints = []
        self.wp1 = []
        self.wp2 = []


    def set_next_state(self, state):
        self.next_state = state

    """ This function is run continuously in a thread"""

    def run(self):
        
        # #print("end eff. pos")
        link = np.array([117.5,100.5,113,108,0])
        # link = np.array([0,100.5,113])
        last_link = np.array([0])
        a = FK_dh_new(np.append(self.rexarm.joint_angles_fb,last_link), link)[:,-1][0:3]
        # #print(IK2(self.rexarm,a))
        

        
        # #print(self.current_state, self.next_state)
        if(self.current_state == "manual"):
            if (self.next_state):
    # with gripper link = np.array([0,117.5_state == "manual"):
                self.manual()
            if(self.next_state == "idle"):
                self.idle()                
            if(self.next_state == "estop"):
                self.estop()

        if(self.current_state == "idle"):
            if(self.next_state == "manual"):
                self.manual()
            if(self.next_state == "execute"):
                self.execute()
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()
            if(self.next_state == "teach"):
                self.teach()
            if(self.next_state == "repeat"):
                self.repeat()
            if(self.next_state == "traj"):
                self.traj()
            if(self.next_state == "calibrate"):
                self.calibrate()
            if(self.next_state == "pickplace"):
                self.pickplace()
            if(self.next_state == "task1"):
                self.task1()
            if(self.next_state == "task2"):
                self.task2()
            if(self.next_state == "task3"):
                self.task3()
            if(self.next_state == "task4"):
                self.task4()
            if(self.next_state == "task5"):
                self.task5()
                
        if(self.current_state == "estop"):
            self.next_state = "estop"
            self.estop()  

        if(self.current_state == "teach"):
             if(self.next_state == "repeat"):
                self.repeat()

        if(self.current_state == "repeat"):
             if(self.next_state == "teach"):
                self.teach()


        if(self.current_state == "execute"):
            self.next_state = "idle"
            self.execute() 
            # self.estop() 

        if(self.current_state == "calibrate"):
            if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "traj"):
            if(self.next_state == "idle"):
                self.idle()
        
        if(self.current_state == "pickplace"):
            if(self.next_state == "idle"):
                self.idle()
        
        if(self.current_state == "task1"):
            if(self.next_state == "idle"):
                self.idle()
        
        if(self.current_state == "task2"):
            if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "task3"):
            if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "task4"):
            if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "task5"):
            if(self.next_state == "idle"):
                self.idle()
               

    """Functions run for each state"""


    def manual(self):
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"
        self.rexarm.send_commands()
        self.rexarm.get_feedback()

    def idle(self):
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"
        self.rexarm.get_feedback()

    def estop(self):
        self.status_message = "EMERGENCY STOP - Check Rexarm and restart program"
        self.current_state = "estop"
        self.rexarm.disable_torque()
        self.rexarm.get_feedback()

    def execute(self):
        # #print('exec')
        self.status_message = "State: Execute"
        self.current_state = "execute"
        self.next_state = "idle"
        self.rexarm.set_positions([ 0.0, 0.0, 0.0, 0.0, 0.0])
        self.rexarm.pause(2)
        self.rexarm.set_positions([ 1.0, 0.8, 1.0, 0.5, 1.0])
        self.rexarm.pause(2)
        self.rexarm.set_positions([-1.0,-0.8,-1.0,-0.5, -1.0])
        self.rexarm.pause(2)
        self.rexarm.set_positions([-1.0, 0.8, 1.0, 0.5, 1.0])
        self.rexarm.pause(2)
        self.rexarm.set_positions([1.0, -0.8,-1.0,-0.5, -1.0])
        self.rexarm.pause(2)
        self.rexarm.set_positions([ 0.0, 0.0, 0.0, 0.0, 0.0])
        self.rexarm.pause(2)
        self.rexarm.get_feedback()
    
    def teach(self):
        self.status_message = "State: Teach"
        self.waypoints = []
        self.rexarm.disable_torque()
        self.current_state='teach'

    def repeat(self):
        # #print(self.waypoints)
        self.status_message = "State: Repeat"
        self.current_state="repeat"
        self.rexarm.enable_torque()
        # #print(self.wp1)
        # self.rexarm.set_positiprintns(self.wp1)
        # self.rexarm.pause(2)
        # #print(self.wp2)
        # self.rexarm.set_positions(self.wp2)
        # self.rexarm.pause(2)
        self.waypoints = [
            [-1.7453292524444446, -1.2420606857299146, -1.6455961523333333, -1.0569222596422287, 1.4152009917243396, 0.2891821194662758],
            [-1.625649532311111, -1.238991974957265, -1.668611483128205, -0.9187290344105572, 1.404964456521994, 0.2738273166627567],
            [-1.512107233723077, -1.2359232641846154, -1.6839550369914529, -0.7907723443812318, 1.404964456521994, 0.25335424625806446],
            [-1.3832213812717948, -1.223648421094017, -1.6946955246957265, -0.6423425839472141, 1.3589000481114373, 0.22776290825219947],
            [-1.2650760165247865, -1.2267171318666668, -1.6640084169692309, -0.5246224291202348, 1.3179539073020532, 0.21240810544868038],
            [-1.1438619410051283, -1.2405263303435896, -1.5918937138119658, -0.42225707709677396, 1.2462981608856305, 0.20217157024633448],
            [-1.0379914193487179, -1.2696790826837607, -1.504435456791453, -0.3147734574721408, 1.179760682070381, 0.19193503504398857],
            [-0.9244491207606842, -1.3049692565692308, -1.3939618689760684, -0.19193503504398812, 1.0978684004516133, 0.16122542943694995],
            [-0.8538687729897436, -1.3602060504769231, -1.2574042395931624, -0.11004275342521996, 1.0159761188328447, 0.13563409143108496],
            [-0.7878914913777777, -1.4461299521111113, -1.0610067501435898, -0.06909661261583544, 0.9084924992082111, 0.11516102102639314],
            [-0.7802197144461536, -1.5213133660410256, -0.9428613853965815, -0.06909661261583544, 0.8829011612023461, 0.11004275342521996],
            [-0.726517275924786, -1.4660765721333333, -1.0993656348017096, -0.03326873940762498, 0.9852665132258065, 0.09980621822287405],
            [-0.6881583912666667, -1.447664307497436, -1.1791521148905983, 0.01279566900293272, 1.0671587948445747, 0.07933314781818179],
            [-0.6544025727675216, -1.4231146213162396, -1.2834882811606838, 0.04862354221114362, 1.1592876116656892, 0.03326873940762454],
            [-0.5960970680871793, -1.418511555157265, -1.3356563642957267, 0.0895696830205277, 1.2309433580821119, 0.023032204205278628],
            [-0.4917609018170941, -1.3985649351350429, -1.424648976702564, 0.2226446406510263, 1.328190442504399, -0.01791393660410545],
            [-0.39969957863760674, -1.3847557366581196, -1.4860231921555556, 0.30965518987096763, 1.374254850914956, -0.023032204205278628],
            [-0.2953634123675215, -1.347931207386325, -1.556603539926496, 0.41202054189442805, 1.3947279213196477, -0.03838700700879771],
            [-0.22017999843760716, -1.346396852, -1.5780845153350427, 0.4887945559120235, 1.4356740621290323, -0.05886007741348953],
            [-0.10050027830427366, -1.340259430454701, -1.604168556902564, 0.6372243163460412, 1.4612654001348968, -0.07421488021700862],
            [0.0007671771931625493, -1.340259430454701, -1.6103059784478633, 0.7293531331671552, 1.4868567381407622, -0.08956968302052815],
            [0.11737818655384613, -1.340259430454701, -1.5934280691982907, 0.8470732879941352, 1.4919750057419359, -0.1151610210263927],
            [0.23092048514188024, -1.340259430454701, -1.5704127384034188, 0.9801482456246333, 1.4970932733431086, -0.15098889423460404],
            [0.33065358525299127, -1.3433281412273506, -1.5520004737675215, 1.0364491892375365, 1.4970932733431086, -0.16634369703812357],
            [0.4165774868871792, -1.354068628931624, -1.5059698121777778, 1.1081049356539592, 1.5022115409442813, -0.21752637304985356],
            [0.49022654543076927, -1.3663434720222223, -1.447664307497436, 1.169524146868035, 1.5022115409442813, -0.2635907814604108]]
        for angle in (self.waypoints):
            #print("going to ")
            #print(np.array(angle)*180/np.pi)
            angle[5] = angle[5]
            self.tp.go(angle,0.5)
            # self.rexarm.pause(1)
        self.rexarm.set_positions([ 0.0, 0.0, 0.0, 0.0, 0.0,0.0])
        self.rexarm.pause(2)
        self.rexarm.get_feedback()

        
    def calibrate(self):
        self.current_state = "calibrate"
        self.next_state = "idle"
        self.kinect.mask = np.ones((480,640))
        self.kinect.depth2rgb_affine = np.float32([[1,0,0],[0,1,0]])
        self.kinect.rgb2world_affine = np.float32([[1,0,0],[0,1,0]])
        self.kinect.kinectCalibrated = False
        location_strings = ["lower left corner of board",
                            "upper left corner of board",
                            "upper right corner of board",
                            "lower right corner of board",
                            "center of shoulder motor"]
        i = 0
        for j in range(5):
            self.status_message = "Calibration - Click %s in RGB image" % location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    self.kinect.rgb_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False        
        
        i = 0
        for j in range(5):
            self.status_message = "Calibration - Click %s in depth image" % location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    self.kinect.depth_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False
   
        # #print(self.kinect.rgb_click_points)
        # #print(self.kinect.depth_click_points)

        """TODO Perform camera calibration here"""
        self.kinect.depth2rgb_affine = self.kinect.getAffineTransform(self.kinect.depth_click_points,self.kinect.rgb_click_points)
        self.kinect.rgb2world_affine = self.kinect.getAffineTransform(self.kinect.rgb_click_points,self.kinect.world_points)
        np.savez('affine.npz',depth2rgb=self.kinect.depth2rgb_affine,rgb2world=self.kinect.rgb2world_affine)
        self.kinect.getMask()
        self.kinect.kinectCalibrated = True

        self.status_message = "Calibration - Completed Calibration"

        time.sleep(1)

    def task1(self):
        self.rexarm.set_torque_limits([1,1,1,1,1,1])
        self.current_state = "task1"
        self.next_state = "idle"
        self.status_message = "Press ESC to stop the task"
        place = (-140,-0,907-19)
        # n = int((950 - z) / 35)
        i = 0
        place_coord_arm = (self.kinect.world2arm(place))

        while True:
            #print('x')
            self.tp.go([0,0,0,0,0,0])
            # ch = 0xFF & cv2.waitKey(10)
            # if ch == 27:
            #     break
            blocks = self.kinect.blocks
            next = None
            shuffle(blocks)
            # #print(blocks)
            for block in blocks:
                if block[0] > 0:
                    next = block
                    break
            if next == None:
                continue
            #print(next)
            block = next
            #print('pick ' + block[4])
            block_coord_arm = self.kinect.world2arm((block[0],block[1],block[2]))
            angle = block[3]
            # #print(block_coord_arm)
            if self.pickBlock1(block_coord_arm, angle) == None:
                #print('unreacheable')
                continue
            angle = 0.
            for block in blocks:
                if block[0] < 0:
                    blockp = block
                    break
            # blockp = self.kinect.locateNearestBlock((-120,0),1000)
            # #print(blockp)
            # if not blockp:
            #     n = 0
            # else:
            #     n = blockp[5]
            # #print('place ' , n)
            # #print((place_coord_arm[0], place_coord_arm[1], place_coord_arm[2] + i * 38))
            self.placeBlock1((place_coord_arm[0], place_coord_arm[1], place_coord_arm[2] + i * 38), angle)
            i = i+1

    def task2(self):
        self.current_state = "task2"
        self.status_message = "Press ESC to stop the task"
        self.next_state = "idle"
        # blocks = self.kinect.blocks
        # place_coord = (,-150,908)
        # n = int((950 - z) / 35)
        i = 0
        colors = ['black','red','orange','yellow','green','blue','violet','pink']
        # colors = ['black','yellow','green','blue','violet']

        while True:
            if i == len(colors): 
                break
            self.tp.go([0,0,0,0,0,0])
            ch = 0xFF & cv2.waitKey(10)
            if ch == 27:
                break
            blocks = self.kinect.blocks
            shuffle(blocks)
            next = None
            place = False
            blockp=None
            retry = False
            next1 = None 
            next2=None 
            next3=None
            for j in range(len(blocks)):
                block = blocks[j][:]
                if block[1] < -30:
                    continue
                # #print(block)
                if block[5] >= 1:
                    next1 = block[:]
                    blockp = blocks[(j+1)%len(blocks)][:]
                if i > 0 and block[4] == colors[i-1]:
                    #print(block[4], colors[i])
                    next2 = block[:]
                    place = True
                    retry = True
                if block[4] == colors[i]:
                    #print(block[4], colors[i])
                    next3 = block[:]
                    place = True
            
            if place == False:
                next = next1
            elif place == True and retry == True:
                next = next2
                i = i - 1
            else:
                next = next3
            
            if not next:
                continue
            # blocks = self.kinect.blocks
            # next = None
            # shuffle(blocks)
            # for block in blocks:
            #     if block[1] > 0:
            #         next = block
            #         break
            # if next == None:
            #     continue
            block = next[:]
            #print(block)
            #print(place,blockp)
            if (block[4] == 'orange'):
                block[0] = -185
                block[1] = 0
                block[2] = 907
                block[3] = 0
            self.rexarm.pause(1)
            block_coord_arm = self.kinect.world2arm((block[0],block[1],block[2]))
            angle = block[3]
            #print('pick ' + colors[i])

            
            if self.pickBlock2(block_coord_arm, angle) == None:
                #print('unreacheable')
                continue

            if place == True:
                place_coord = (-155+28*(i+1),-150,908+19)
                i = i + 1
                place_coord = (place_coord[0]+32,place_coord[1],place_coord[2])
                place_coord_arm = (self.kinect.world2arm(place_coord))
                angle = 90
                self.placeBlock2((place_coord_arm[0], place_coord_arm[1], place_coord_arm[2]),i, angle)
            else:
                place_coord_arm = self.kinect.world2arm((blockp[0],blockp[1],blockp[2]))
                place_coord_arm = (place_coord_arm[0], place_coord_arm[1], place_coord_arm[2] + 38)
                angle = blockp[3]
                #print("place on the block at (%.0f,%.0f,%.0f)" % place_coord_arm)
                self.placeBlock((place_coord_arm[0], place_coord_arm[1], place_coord_arm[2]),angle)
            
            
            
            

    def task3(self):
        self.current_state = "task3"
        self.status_message = "Press ESC to stop the task"
        self.next_state = "idle"
        i = 0
        colors = ['black','red','orange','orange','green','blue','violet','pink']

        while True:
            if i == len(colors): 
                break
            self.tp.go([0,0,0,0,0,0])
            blocks = self.kinect.blocks
            shuffle(blocks)
            next = None
            place = False
            blockp=None
            retry = False
            next1 = None 
            next2=None 
            next3=None
            for j in range(len(blocks)):
                block = blocks[j][:]
                if block[1] < 0:
                    continue
                # #print(block)
                if block[5] >= 1:
                    next1 = block[:]
                    blockp = blocks[(j+1)%len(blocks)][:]
                # if i > 0 and block[4] == colors[i-1]:
                    # #print(block[4], colors[i])
                    # next2 = block[:]
                    # place = True
                    # retry = True
                if block[4] == colors[i]:
                    #print(block[4], colors[i])
                    next3 = block[:]
                    place = True
            
            if place == False:
                next = next1
            elif place == True and retry == True:
                next = next2
                i = i - 1
            else:
                next = next3
            
            if not next:
                continue
            # blocks = self.kinect.blocks
            # next = None
            # shuffle(blocks)
            # for block in blocks:
            #     if block[1] > 0:
            #         next = block
            #         break
            # if next == None:
            #     continue
            block = next[:]


            if (i == 2):
                block[0] = -205
                block[1] = 0
                block[2] = 907
                block[3] = 0
                place = True

            # if (i = 3)

            self.rexarm.pause(1)
            block_coord_arm = self.kinect.world2arm((block[0],block[1],block[2]))
            angle = block[3]
            #print('pick ' + colors[i])
            if self.pickBlock2(block_coord_arm, angle) == None:
                #print('unreacheable')
                continue

            # place=True

            if place == True:
                # place_coord = (-174.0106602214206, -210.2833167615397, 880)
                angle = -50.38013458251953
                #print(i)
                place_coord = (-175.0106602214206, -175.2833167615397,907+5-i*38)
                i = i + 1
                place_coord = (place_coord[0],place_coord[1],place_coord[2])
                place_coord_arm = (self.kinect.world2arm(place_coord))
                # angle = 45.
                self.placeBlock3((place_coord_arm[0], place_coord_arm[1], place_coord_arm[2]), angle, i)
            else:
                # place_coord_arm = self.kinect.world2arm((blockp[0],blockp[1],blockp[2]+15))
                place_coord_arm = self.kinect.world2arm((-100,10,blockp[2]+15))
                place_coord_arm = (place_coord_arm[0], place_coord_arm[1], place_coord_arm[2] + 38)
                angle = blockp[3]
                #print("place on the block at (%.0f,%.0f,%.0f)" % place_coord_arm)
                self.placeBlock1((place_coord_arm[0], place_coord_arm[1], place_coord_arm[2]),angle)

            # #print('pick ' + block[4])
            # block_coord_arm = self.kinect.world2arm((block[0],block[1],block[2]))
            # angle = block[3]
            # # #print(block_coord_arm)
            # if self.pickBlock1(block_coord_arm, angle) == None:
            #     #print('unreacheable')
            #     continue
            # angle = 0.
            # for block in blocks:
            #     if block[0] < 0:
            #         blockp = block
            #         break
            # # blockp = self.kinect.locateNearestBlock((-120,0),1000)
            # # #print(blockp)
            # # if not blockp:
            # #     n = 0
            # # else:
            # #     n = blockp[5]
            # # #print('place ' , n)
            # # #print((place_coord_arm[0], place_coord_arm[1], place_coord_arm[2] + i * 38))
            # self.placeBlock1((place_coord_arm[0], place_coord_arm[1], place_coord_arm[2] + i * 38), angle)
            # i = i+1
    
    def task4wp(self,wp,reverse = False):
        if reverse:
            for i in range(len(wp)):
                #print("going to ")
                angle = wp[len(wp) - 1 - i]
                #print(np.array(angle)*180/np.pi)
                # angle[1] = angle[1] * 0.97
                # angle[0] = angle[0] - np.pi / 2
                # if angle[0] > np.pi:
                #     angle[0] -= np.pi * 2
                # elif angle[0] < -np.pi:
                #     angle[0] += np.pi * 2
                angle[4] = angle[4] * 1.05
                self.tp.go(angle,0.7)
        else:
            for angle in (wp):
                #print("going to ")
                #print(np.array(angle)*180/np.pi)
                # angle[1] = angle[1] * 0.97
                # angle[0] = angle[0] - np.pi / 2
                # if angle[0] > np.pi:
                #     angle[0] -= np.pi * 2
                # elif angle[0] < -np.pi:
                #     angle[0] += np.pi * 2
                angle[4] = angle[4] * 1.05
                self.tp.go(angle,0.99)
        self.rexarm.pause(1)

    def task4(self):
        self.current_state = "task4"
        self.next_state = "idle"
        self.rexarm.open_gripper()
        #print('section00')
        section00 = [[-0.7326546974700854, 0.0038358879658120237, -0.20483644457435934, 0.06909661261583588, 1.0210943864340178, -0.03326873940762498],
            [-0.7387921190153848, 0.08362236805470058, -1.547397407608547, 0.06909661261583588, 1.0057395836304983, -0.03326873940762498],
            [-0.7081050112888891, -1.238991974957265, -1.4415268859521366, 0.14075235903225813, 0.9801482456246333, -0.0025591338005868103],
            [-0.7065706559025644, -1.5228477214273506, -1.0364570639623931, 0.13563409143108496, 0.9699117104222874, -0.15098889423460404]]
        # section00 = [[-0.7096393666752134, -1.2435950411162393, -2.0046353127333334, 0.043505274609970446, 1.691587442187684, -0.007677401401759543],
        #     [-0.7387921190153848, -1.2374576195709404, -1.699298590854701, 0.03838700700879771, 1.4305557945278595, -0.007677401401759543],
        #     [-0.7587387390376072, -1.3356563642957267, -1.3203128104324786, 0.043505274609970446, 1.1490510764633433, -0.007677401401759543],
        #     [-0.7740822929008551, -1.3801526704991454, -1.1990987349128204, 0.03326873940762454, 1.0569222596422287, -0.01791393660410545]]
        self.task4wp(section00)
        self.rexarm.close_gripper()
        self.rexarm.pause(1)

        #print('section01')
        # section01 = [[-0.7648761605829058, -1.4584047952017094, -1.0932282132564106, 0.01279566900293272, 1.0364491892375365, -0.0025591338005868103],
        #     [-0.8017006898547008, -1.4077710674529915, -1.2251827764803418, 0.043505274609970446, 1.087631865249267, 0.002559133800586366],
        #     [-0.8369908637401711, -1.3786183151128206, -1.3233815212051283, 0.043505274609970446, 1.1490510764633433, 0.002559133800586366],
        #     [-0.9045025007384617, -1.340259430454701, -1.4461299521111113, -0.0025591338005868103, 1.2309433580821119, 0.002559133800586366],
        #     [-0.9612736500324783, -1.3233815212051283, -1.5412599860632479, -0.03326873940762498, 1.323072174903226, 0.002559133800586366],
        #     [-1.076350304006838, -1.3095723227282052, -1.6271838876974358, -0.14587062663343087, 1.404964456521994, 0.01279566900293272],
        #     [-1.1883582472085472, -1.2926944134786325, -1.7008329462410257, -0.294300387067449, 1.456147132533724, 0.017913936604105896],
        #     [-1.3433281412273506, -1.2850226365470085, -1.757604095535043, -0.4580849503049853, 1.5175663437478013, 0.023032204205278628],
        #     [-1.5029011014051283, -1.289625702705983, -1.7606728063076924, -0.6423425839472141, 1.5380394141524931, 0.023032204205278628],
        #     [-1.676283260059829, -1.2911600580923077, -1.7591384509213674, -0.8163636823870968, 1.5533942169560122, 0.023032204205278628],
        #     [-1.8680776833504273, -1.2988318350239316, -1.713107789331624, -1.0159761188328444, 1.5533942169560122, 0.058860077413489975],
        #     [-1.9938948250290598, -1.31110667811453, -1.6440617969470086, -1.1592876116656892, 1.5533942169560122, 0.09980621822287405],
        #     [-2.1074371236170943, -1.3279845873641025, -1.5504661183811965, -1.2872443016950148, 1.5533942169560122, 0.19193503504398857],
        #     [-2.20870457911453, -1.3509999181589745, -1.4461299521111113, -1.3947279213196482, 1.5533942169560122, 0.26870904906158355],
        #     [-2.35293398542906, -1.340259430454701, -1.5029011014051283, -1.5124480761466275, 1.5687490197595304, 0.26870904906158355]]
        section01 = [[-0.7050363005162397, -1.519779010654701, -1.039525774735043, 0.14075235903225813, 0.9750299780234606, -0.11004275342521996],
            [-0.8063037560136754, -1.3801526704991454, -1.3709465381811967, 0.13051582382991178, 1.113223203255132, -0.11004275342521996],
            [-0.8937620130341877, -1.355602984317949, -1.4783514152239317, 0.04862354221114362, 1.2155885552785923, -0.07421488021700862],
            [-1.0333883531897436, -1.3417937858410256, -1.5888250030393163, -0.09980621822287405, 1.328190442504399, -0.03838700700879771],
            [-1.1960300241401711, -1.3218471658188036, -1.6900924585367523, -0.2635907814604108, 1.4152009917243396, -0.01791393660410545],
            [-1.3141753888871797, -1.3141753888871797, -1.7376574755128207, -0.39666573909090896, 1.4663836677360704, -0.01791393660410545],
            [-1.4507330182700855, -1.266610371911111, -1.8005660463521367, -0.5143858939178889, 1.4510288649325513, -0.0025591338005868103],
            [-1.576550159948718, -1.2343889087982907, -1.8281844433059828, -0.6525791191495602, 1.4510288649325513, 0.023032204205278628],
            [-1.8174439556017095, -1.2006330902991453, -1.8189783109880342, -0.9187290344105572, 1.4203192593255132, 0.07933314781818179],
            [-2.0015666019606835, -1.2129079333897437, -1.7514666739897435, -1.0927501328504399, 1.4203192593255132, 0.12027928862756587],
            [-2.1611395621384615, -1.2481981072752137, -1.6164433999931624, -1.2872443016950148, 1.4254375269266868, 0.23799944345454538],
            [-2.265475728408547, -1.2926944134786325, -1.547397407608547, -1.3742548509149561, 1.4459105973313786, 0.3301282602756599]]

        # s
        self.task4wp(section01)
        self.rexarm.open_gripper()
        self.rexarm.pause(1)
        
        # section02 = [[-2.9068362798923078, -1.0441288408940168, -1.8419936417829061, -1.9065546814369503, 1.5533942169560122, 0.2635907814604104],
        #     [-2.9083706352786325, -1.0349227085760684, -1.8435279971692309, -1.9065546814369503, 1.5533942169560122, 0.2635907814604104],
        #     [-2.9099049906649572, -0.9996325346905981, -1.8435279971692309, -1.9219094842404694, 1.5533942169560122, 0.2635907814604104]]

        #print('section02')

        section02 = [[-2.303834613066667, -1.238991974957265, -1.5796188707213676, -1.3947279213196482, 1.4612654001348968, 0.3557195982815249],
            [-2.4480640193811967, -1.0487319070529915, -1.8987647910769232, -1.3998461889208211, 1.5124480761466277, 0.3506013306803517],
            [-2.4925603255846154, -0.7572043836512821, -2.073681305117949, -1.389609653718475, 1.5789855549618768, 0.3557195982815249],
            [-2.748797675100855, 0.0038358879658120237, -1.7100390785589743, -1.410082724123167, 1.5841038225630504, 0.3403647954780058],
            [-2.9835540492085473, -0.09129414598632479, 1.215976643162393, -1.379373118516129, 1.6455230337771267, 0.33524652787683307]]
        self.task4wp(section02)
        # self.rexarm.pause(1)

        # self.tp.go([0,0,0,0,0,0])

        # section10 = [[2.7012326571247858, 1.4231146203162393, 1.3187784540461536, -1.3435452453079177, 1.660877836580645, 2.617993878],
            # [2.305368967452991, 1.4507330172700854, 1.2758165032290592, -1.5789855549618768, 1.6045768929677422, 2.617993878],
            # [2.2501321735452997, 1.4584047942017087, 1.2712134370700854, -1.650641301378299, 1.5789855549618768, 2.617993878]]
        #print('section10')
        section10 = [[3.008103734389744, 0.508638810066667, 1.77294764839829, -1.3998461889208211, 1.609695160568915, 0.33524652787683307],
            [2.6613394170803417, 0.9121742766700853, 1.7652758714666668, -1.3998461889208211, 1.5533942169560122, -0.3813109362873903],
            [2.4204456214273504, 1.1515337169367523, 1.4906262573145304, -1.4715019353372434, 1.5943403577653958, -0.4734397531085044],
            [2.2025671565692315, 1.2543355278205128, 1.2267171308666667 * 1.05, -1.5175663437478006, 1.5943403577653958, -0.5246224291202348]]
        self.task4wp(section10)
        self.rexarm.close_gripper()
        # self.rexarm.pause(100)

        # section11 = [[2.145796007275214, 1.4430612403384613, 1.355602983317949, -1.7530066534017594, 1.5841038225630504, 2.617993878],
        #     [2.066009527186325, 1.3893588018170941, 1.4706796372923074, -1.8041893294134899, 1.5278028789501468, 2.617993878],
        #     [1.986223047097436, 1.3924275125897436, 1.5151759434957262, -1.8911998786334312, 1.5380394141524931, 2.617993878],
        #     [1.8818868808273503, 1.401633644907693, 1.5182446542683756, -1.9577373574486803, 1.543157681753665, 2.617993878],
        #     [1.77294764839829, 1.401633644907693, 1.5289851419726492, -2.0447479066686216, 1.5329211465513195, 2.617993878],
        #     [1.6594053498102568, 1.4031680002940172, 1.5397256296769228, -2.147113258692082, 1.5278028789501468, 2.617993878],
        #     [1.555069183540171, 1.401633644907693, 1.541259985063248, -2.2392420755131965, 1.5124480761466277, 2.617993878],
        #     [1.467610926519658, 1.3970305787487174, 1.5397256296769228, -2.300661286727273, 1.4817384705395895, 2.617993878],
        #     [1.3847557356581204, 1.395496223362393, 1.5305194973589744, -2.377435300744868, 1.4663836677360704, 2.617993878],
        #     [1.2942287678649578, 1.3970305787487174, 1.5167102988820513, -2.4798006527683283, 1.440792329730205, 2.617993878],
        #     [1.189892601594872, 1.4093054218393162, 1.4660765711333337, -2.597520807595308, 1.3998461889208214, 2.617993878],
        #     [1.1055030553470084, 1.4185115541572655, 1.4123741326119656, -2.617993878, 1.338426977706745, 2.617993878],
        #     [1.0456631952803415, 1.4307863972478634, 1.361740404863248, -2.617993878, 1.2974808368973605, 2.617993878],
        #     [0.9827546244410259, 1.4599391495880347, 1.2742821478427349, -2.617993878, 1.2462981608856305, 2.617993878],
        #     [0.9213804089880338, 1.4722139926786317, 1.196030023140171, -2.617993878, 1.1541693440645164, 2.617993878],
        #     [0.8492657058307689, 1.4998323896324788, 1.1008999891880347, -2.617993878, 1.0569222596422287, 2.617993878],
        #     [0.8032350442410254, 1.5443286958358975, 0.988892045986324, -2.617993878, 1.0057395836304983, 2.617993878]]
        #print('section11')
        section11 = [[2.1810861811606834, 1.2435950401162392, 1.2466637508888887, -1.5482759493548388, 1.6199316957712613, -0.5195041615190616],
            [1.9954291794153844, 1.154602427709401, 1.4798857696102568, -1.7018239773900294, 1.543157681753665, -0.4171388094956012],
            [1.8297187976923075, 1.1392588738461535, 1.5688783820170942, -1.870726808228739, 1.5124480761466277, -0.345483063079179],
            [1.6041685559025645, 1.1131748322786326, 1.7330544083538468, -2.1931776671026393, 1.5124480761466277, -0.1868167674428154],
            [1.4369238187931623, 1.1147091876649577, 1.7039016560136755, -2.351843962739003, 1.471501935337244, -0.1868167674428154],
            [1.2267171308666667, 1.1300527415282051, 1.6256495313111108, -2.5309833287800587, 1.389609653718475, -0.1868167674428154],
            [1.0610067491435897, 1.1791521138905985, 1.473748348064957, -2.617993878, 1.2872443016950146, -0.1816984998416422],
            [0.8952963674205128, 1.2405263293435898, 1.2850226355470085, -2.617993878, 1.138814541260997, -0.16122542943694995],
            [0.8860902351025643, 1.2405263293435898, 1.2926944124786317, -2.617993878, 1.14393280886217, -0.14587062663343087],
        ]
        self.task4wp(section11)
        
        #print('section11')
        self.task4wp(section11,True)
        self.rexarm.open_gripper()
        self.rexarm.pause(1)
        self.task4wp(section10,True)
        self.tp.go([0,0,0,0,0,0])
        self.task4wp(section02,True)
        self.rexarm.close_gripper()
        self.task4wp(section01,True)

        # return

        section21 = [[-0.7341890528564101, -1.4200459105435899, -1.1960300241401711, 0.11516102102639314, 0.9955030484281524, -0.01279566900293272],
            [-0.657471283540171, -1.4139084889982907, -1.268144727297436, 0.13563409143108496, 1.0722770624457478, -0.0537418098123168],
            [-0.5654099603606837, -1.3571373397042736, -1.4384581751794874, 0.19193503504398857, 1.1899972172727273, -0.0537418098123168],
            [-0.4702799264085473, -1.3525342735452992, -1.5213133660410256, 0.25335424625806446, 1.3025991044985337, -0.05886007741348953],
            [-0.3567376278205132, -1.324915876591453, -1.656336640037607, 0.30965518987096763, 1.4203192593255132, -0.0537418098123168],
            [-0.12965303064444456, -1.3233815212051283, -1.676283260059829, 0.5962781755366571, 1.4663836677360704, -0.08956968302052815],
            [0.06674445880512803, -1.3233815212051283, -1.676283260059829, 0.7907723443812316, 1.4919750057419359, -0.11004275342521996],
            [0.2631419482547006, -1.324915876591453, -1.6317869538564105, 1.0364491892375365, 1.507329808545454, -0.16634369703812357],
            [0.4227149084324786, -1.332587653523077, -1.5612066060854701, 1.1490510764633433, 1.507329808545454, -0.23799944345454538],
            [0.49022654543076927, -1.3341220089094017, -1.5427943414495726, 1.1951154848739005, 1.507329808545454, -0.25847251385923764]
            ]
        
        section22 = [[0.5868909347692308, -1.2297858426393165, -1.7806194263299147, 1.2053520200762464, 1.543157681753665, -0.2533542462580649],
            [0.6850896794940171, -0.9137086330564101, -2.012307089664957, 1.2821260340938418, 1.5380394141524931, -0.2482359786568913],
            [1.048731906052991, -0.46567686024957267, -1.9463298080529916, 1.2872443016950146, 1.543157681753665, -0.2482359786568913],
            [1.082487724552137, -0.33372229702564127, -0.5516007618837606, 1.3025991044985337, 1.5380394141524931, -0.2482359786568913]]
        

        section30 = [[-1.548931762994872, -0.5930283573145299, -2.0429941973914527, 1.7478883858005867, -1.6608778365806454, 0.3557195982815249],
            [-1.0671441716888888, -1.2190453549350428, -1.3801526704991454, 1.4356740621290323, -1.5226846113489738, 0.4222570770967744],
            [-0.9244491207606842, -1.27581650422906, -1.3448624966136753, 1.507329808545454, -1.4510288649325513, 0.4222570770967744],
            [-0.7449295405606837, -1.27581650422906, -1.3371907196820514, 1.660877836580645, -1.4254375269266861, 0.4222570770967744]]
            # [-1.0579380393709403, -1.9923604696427353, 0.1710806250752137, 1.558512484557185, -1.7018239773900294, -1.7478883858005867]]


        section31 = [[-0.7556700282649573, -1.2773508596153846, -1.3448624966136753, 1.6557595689794722, -1.4254375269266861, 0.6679339219530793],
            [-0.6666774158581195, -1.3172440996598291, -1.3310532981367522, 1.6711143717829913, -1.4305557945278593, 0.6474608515483871],
            [-0.520913654157265, -1.3141753888871797, -1.3663434720222223, 1.7376518505982412, -1.4305557945278593, 0.6372243163460412],
            [-0.335256652411966, -1.3095723227282052, -1.4445955967247863, 1.8911998786334312, -1.4152009917243402, 0.5297406967214076],
            [-0.19869902302905995, -1.289625702705983, -1.5397256306769231, 2.044747906668621, -1.379373118516129, 0.41202054189442805],
            [0.025316863374359233, -1.3095723227282052, -1.555069184540171, 2.3057795543284465, -1.36401831571261, 0.3250099926744867],
            [0.27081372518632474, -1.3371907196820514, -1.5228477214273506, 2.582166004791789, -1.3128356397008798, 0.26870904906158355]]
            # [0.48255476849914514, -1.3172440996598291, -1.4461299521111113, 2.617993878, -1.128578006058651, 0.23288117585337265],
            # [0.7188454979931622, -1.5642753168581198, -1.002701245463248, 2.617993878, -1.0159761188328444, 0.1816984998416422]]

        

        self.task4wp(section21)
        self.rexarm.open_gripper()
        self.rexarm.pause(1)
        self.task4wp(section22)
        # return
        self.tp.go([0,0,0,0,0,0])
        self.task4wp(section30)
        self.rexarm.close_gripper()
        self.rexarm.pause(1)
        self.task4wp(section31)
        # section21 = [[0.7894258457641028, 1.3463968509999997, 1.1576711384820513, 0.058860077413489975, -0.9033742316070381, -0.03326873940762498],
        #     [0.8477313504444441, 1.2528011724341885, 1.375549603340171, 0.06397834501466271, -0.9750299780234604, -0.03838700700879771],
        #     [0.9152429874427357, 1.2021674446854709, 1.4952293234735041, 0.08445141541935453, -1.0313309216363638, -0.03838700700879771],
        #     [1.0119073767811964, 1.1638085600273502, 1.6287182420837611, 0.13051582382991178, -1.14393280886217, -0.03838700700879771],
        #     [1.1377245184598284, 1.1530680723230766, 1.6962298790820514, 0.23288117585337265, -1.2155885552785923, -0.03838700700879771],
        #     [1.2405263293435898, 1.1453962953914534, 1.7453292514444438, 0.30453692226979445, -1.2667712312903228, -0.04862354221114362],
        #     [1.3540686279316247, 1.1469306507777777, 1.7790850699435898, 0.3915474714897358, -1.3281904425043989, -0.04862354221114362],
        #     [1.4952293234735041, 1.1499993615504271, 1.7913599130341877, 0.5143858939178885, -1.3742548509149561, -0.04862354221114362],
        #     [1.633321308242735, 1.1515337169367523, 1.792894268420512, 0.6525791191495598, -1.384491386117302, -0.04862354221114362],
        #     [1.7821537807162393, 1.1638085600273502, 1.7821537807162393, 0.7702992739765397, -1.404964456521994, -0.0537418098123168],
        #     [1.9141083439401703, 1.1668772707999997, 1.7453292514444438, 0.9340838372140765, -1.389609653718475, -0.07933314781818179],
        #     [2.038391130232479, 1.1776177585042733, 1.6701458375145295, 1.062040527243402, -1.384491386117302, -0.13563409143108496],
        #     [2.1212463210940173, 1.1944956677538459, 1.5980311343572655, 1.138814541260997, -1.384491386117302, -0.1970533026451613],
        #     [2.185689247319658, 1.2129079323897436, 1.5182446542683756, 1.2104702876774192, -1.384491386117302, -0.25847251385923764],
        #     [2.239391685841026, 1.2328545524119665, 1.4538017280427349, 1.2667712312903223, -1.384491386117302, -0.3045369222697949],
        #     [2.303834612066667, 1.2650760155247864, 1.3648091156358975, 1.338426977706745, -1.389609653718475, -0.3557195982815249],
        #     [0.8906933012615381, 1.281953924774359, 1.3356563632957261, 0.15098889423460404, -1.0006213160293256, -0.10492448582404679],
        #     [0.9290521859196588, 1.2788852140017095, 1.4062367110666667, 0.15610716183577722, -1.108104935653959, -0.09468795062170088],
        #     [1.0073043106222226, 1.2512668170478634, 1.4952293234735041, 0.21240810544868038, -1.169524146868035, -0.09468795062170088],
        #     [1.1377245184598284, 1.2328545524119665, 1.5750158035623931, 0.36595613348387124, -1.2462981608856305, -0.09468795062170088],
        #     [1.269679081683761, 1.2313201970256404, 1.6455961513333337, 0.4887945559120235, -1.348663512909091, -0.10492448582404679],
        #     [1.4323207526341886, 1.222114064707692, 1.6916268129230767, 0.6372243163460412, -1.3998461889208211, -0.09468795062170088],
        #     [1.6302525974700854, 1.2267171308666667, 1.6916268129230767, 0.8010088795835779, -1.4305557945278593, -0.09980621822287405],
        #     [1.7836881361025645, 1.222114064707692, 1.6578709944239316, 1.010857851231672, -1.410082724123167, -0.12027928862756632],
        #     [1.980085625552137, 1.2558698832068371, 1.5458630512222218, 1.179760682070381, -1.4305557945278593, -0.1970533026451613],
        #     [2.0506659733230768, 1.268144726297436, 1.481420124996581, 1.2411798932844578, -1.4305557945278593, -0.2533542462580649],
        #     [2.1488647180478635, 1.2911600570923074, 1.395496223362393, 1.36401831571261, -1.4407923297302054, -0.30965518987096763],
        #     [2.245529107386324, 1.321847164818803, 1.2589385939794866, 1.471501935337244, -1.4459105973313784, -0.40178400669208214],
        #     [2.3145750997709404, 1.361740404863248, 1.1147091876649577, 1.5636307521583586, -1.4510288649325513, -0.5092676263167153],
        #     [2.377483670610256, 1.4123741326119656, 0.9551362274871789, 1.625049963372434, -1.4766202029384163, -0.6116329783401762],
        #     [2.432720464517949, 1.48755754654188, 0.7664105149692308, 1.660877836580645, -1.4970932733431084, -0.7242348655659825],
        #     [2.4618732168581197, 1.5397256296769228, 0.6436620840632479, 1.6864691745865104, -1.5124480761466275, -0.7856540767800588],
        #     [2.49716339074359, 1.647130506719658, 0.4150431315008545, 1.7069422449912022, -1.5431576817536659, -0.9033742316070381]]
        # self.task4wp(section21)
        # self.rexarm.open_gripper()
        
        # section12 = [[2.510972589220513, 1.59342806819829, 0.5270510747025643, 1.758124921002933, -1.5329211465513197, -0.9033742316070381],
        #     [2.649064573989744, 1.4906262573145304, 0.73725776262906, 1.7478883858005867, -1.5380394141524927, -0.8829011612023461],
        #     [2.848530774211966, 1.2205797093213677, 1.1944956677538459, 1.7478883858005867, -1.4868567381407625, -0.6116329783401762],
        #     [3.1385239422273505, 1.010373021394872, 1.5520004727675216, 1.793952794211144, -1.5022115409442816, -0.4478484151026394]]
        # section13 = []
        # section2 = [[-1.6394587307880344, -1.2512668180478632*0.95, -1.6931611693094017, -0.8214819499882697, 1.4459105973313786, 0.13051582382991178],
        #     [-1.5612066060854701, -1.274282148842735, -1.67014583851453, -0.7958906119824047, 1.456147132533724, 0.12539755622873905],
        #     [-1.461473505974359, -1.2834882811606838, -1.6425274415606836, -0.6986435275601175, 1.4356740621290323, 0.12539755622873905],
        #     [-1.3648091166358975, -1.3003661904102564, -1.605702912288889, -0.6321060487448682, 1.4356740621290323, 0.12027928862756587],
        #     [-1.2712134380700857, -1.3203128104324786, -1.555069184540171, -0.5399772319237535, 1.4100827241231677, 0.11004275342521996],
        #     [-1.1714803379589744, -1.347931207386325, -1.4875575475418805, -0.45296668270381213, 1.36401831571261, 0.11004275342521996],
        #     [-1.065609816302564, -1.3586716950905984, -1.4491986628837608, -0.3250099926744867, 1.328190442504399, 0.11004275342521996],
        #     [-0.9704797823504276, -1.3924275135897435, -1.3448624966136753, -0.22776290825219947, 1.2616529636891496, 0.10492448582404723],
        #     [-0.9060368561247865, -1.4231146213162396, -1.254335528820513, -0.16122542943694995, 1.1951154848739005, 0.10492448582404723],
        #     [-0.8446626406717948, -1.4323207536341882, -1.20216744568547, -0.07933314781818179, 1.128578006058651, 0.09468795062170088],
        #     [-0.8047694006273507, -1.4829544813829059, -1.0870907917111112, -0.04350527460997089, 1.0671587948445747, 0.08445141541935453],
        #     [-0.7771510036735041, -1.5427943414495726, -0.9566705838735046, -0.04350527460997089, 1.0057395836304983, 0.07933314781818179],
        #     [-0.763341805196581, -1.5980311353572652, -0.866143616080342, -0.007677401401759543, 0.9801482456246333, 0.07421488021700862]]
        # section3=[[-0.763341805196581, -1.5980311353572652, -0.866143616080342, -0.007677401401759543, 0.9801482456246333, 0.07421488021700862],
        #     [-0.7341890528564101, -1.5612066060854701, -0.9612736500324783, -0.007677401401759543, 1.0159761188328447, 0.07421488021700862],
        #     [-0.7065706559025644, -1.4936949690871795, -1.1361901640735041, 0.002559133800586366, 1.113223203255132, 0.07421488021700862],
        #     [-0.6559369281538463, -1.476817059837607, -1.209839222617094, 0.0537418098123168, 1.1951154848739005, 0.058860077413489975],
        #     [-0.6145093327230771, -1.4399925305658121, -1.2957631242512822, 0.06397834501466271, 1.2309433580821119, 0.043505274609970446],
        #     [-0.5516007618837606, -1.4000992905213676, -1.3924275135897435, 0.14075235903225813, 1.2821260340938418, 0.028150471806451804],
        #     [-0.4886921910444446, -1.4000992905213676, -1.4261833320888888, 0.20217157024633448, 1.3333087101055718, 0.017913936604105896],
        #     [-0.42118055404615395, -1.3862900920444445, -1.4844888367692308, 0.25847251385923764, 1.389609653718475, 0.002559133800586366],
        #     [-0.3475314955025639, -1.3786183151128206, -1.519779010654701, 0.319891725073314, 1.4203192593255132, -0.007677401401759543],
        #     [-0.2508671061641028, -1.3709465381811967, -1.5627409614717949, 0.4171388094956012, 1.476620202938416, -0.02815047180645136],
        #     [-0.15420271682564124, -1.3586716950905984, -1.605702912288889, 0.4887945559120235, 1.5226846113489731, -0.03326873940762498],
        #     [-0.039126062851282395, -1.3602060504769231, -1.625649532311111, 0.6116329783401757, 1.5380394141524931, -0.03838700700879771],
        #     [0.09743156653162366, -1.3586716950905984, -1.6317869538564105, 0.7293531331671552, 1.558512484557185, -0.0537418098123168],
        #     [0.20790515434700874, -1.3602060504769231, -1.625649532311111, 0.8163636823870966, 1.573867287360704, -0.07421488021700862],
        #     [0.3352566514119659, -1.3663434720222223, -1.5796188707213676, 0.9699117104222874, 1.573867287360704, -0.11004275342521996],
        #     [0.4380584622957264, -1.3694121827948718, -1.533588209131624, 1.0466857244398824, 1.5687490197595304, -0.16122542943694995],
        #     [0.5761504470649572, -1.4000992905213676, -1.4200459105435899, 1.1490510764633433, 1.5687490197595304, -0.25847251385923764],
        #     [0.6298528855863248, -1.418511555157265, -1.3694121827948718, 1.2053520200762464, 1.5687490197595304, -0.3045369222697949]]
        # section31 = [[2.510972589220513, 1.59342806819829, 0.5270510747025643, 1.758124921002933, -1.5329211465513197, -0.9033742316070381],
        #     [2.649064573989744, 1.4906262573145304, 0.73725776262906, 1.7478883858005867, -1.5380394141524927, -0.8829011612023461],
        #     [2.848530774211966, 1.2205797093213677, 1.1944956677538459, 1.7478883858005867, -1.4868567381407625, -0.6116329783401762],
        #     [3.1385239422273505, 1.010373021394872, 1.5520004727675216, 1.793952794211144, -1.5022115409442816, -0.4478484151026394]]
        # section32 = []
        # section4 = [[-0.5454633403384617, -1.5136415891094017, -1.0548693285982909, 1.7325335829970676, -1.4715019353372434, 0.6116329783401757],
        #     [-0.4871578356581199, -1.5029011014051283, -1.1714803379589744, 1.8246623998181821, -1.4663836677360704, 0.580923372733138],
        #     [-0.4457302402273502, -1.49062625831453, -1.2083048672307692, 1.8400172026217012, -1.4561471325337245, 0.5297406967214076],
        #     [-0.38128731400170945, -1.4630078613606838, -1.2942287688649574, 1.9065546814369503, -1.4510288649325513, 0.4734397531085044],
        #     [-0.2876916354358974, -1.4292520428615385, -1.3970305797487181, 2.0396296390674493, -1.4459105973313784, 0.38642920388856306],
        #     [-0.18488982455213687, -1.4200459105435899, -1.4461299521111113, 2.1727045966979475, -1.4305557945278593, 0.3250099926744867],
        #     [-0.057538327487179686, -1.404702356680342, -1.4967636798598292, 2.3108978219296183, -1.4254375269266861, 0.28406385186510263],
        #     [0.0759505911230769, -1.3954962243623932, -1.5228477214273506, 2.4746823851671556, -1.410082724123167, 0.25335424625806446],
        #     [0.19563031125640995, -1.3985649351350429, -1.5136415891094017, 2.5924025399941346, -1.3742548509149561, 0.2226446406510263],
        #     [0.3091726098444445, -1.4169771997709402, -1.4630078613606838, 2.617993878, -1.3333087101055718, 0.1970533026451613],
        #     [0.4365241069094017, -1.4261833320888888, -1.4200459105435899, 2.617993878, -1.2718894988914957, 0.1714619646392963],
        #     [0.5669443147470083, -1.447664307497436, -1.3448624966136753, 2.617993878, -1.2053520200762464, 0.12539755622873905],
        #     [0.6359903071316237, -1.476817059837607, -1.2558698842068376, 0.6013964431378298, -1.1336962736598242, 0.09468795062170088],
        #     [0.7019675887435897, -1.5075041675641028, -1.1822208256632478, 0.6116329783401757, -1.0978684004516128, 0.07421488021700862],
        #     [0.7219142087658121, -1.5520004737675215, -1.0579380393709403, 0.6013964431378298, -1.0108578512316715, 0.08445141541935453],
        #     [0.7495326057196579, -1.5903593584256412, -0.9704797823504276, 0.5553320347272725, -0.9647934428211145, 0.06909661261583588],
        #     [0.7633418041965809, -1.619512110765812, -0.902968145352137, 0.5860416403343112, -0.9187290344105572, 0.06909661261583588]]
        # # #print(np.array(section2[0])/np.pi*180)
        # self.rexarm.open_gripper()
        
        # # self.tp.go(section2[-1],0.5)
        # # self.tp.go(section2[0],0.5)
        # # self.tp.go([section1[0][0],0,0,0,0,0])
        # # self.tp.go([section1[0][0],0,0,section1[0][3],0,0])
        # self.tp.go(section1[0],0.5)
        # self.rexarm.close_gripper()
        # #print('section1')
        # for angle in (section1):
        #     # #print("going to ")
        #     #print(np.array(angle)*180/np.pi)
        #     angle[0] = angle[0]*0.9
        #     self.tp.go(angle,0.5)
        # self.rexarm.open_gripper()
        # #print('section12')
        # for angle in (section12):
        #     # #print("going to ")
        #     #print(np.array(angle)*180/np.pi)
        #     angle[0] = angle[0]*0.9
        #     self.tp.go(angle,0.5)
        # # for angle in (section2):
        # #     #print("going to ")
        # #     #print(np.array(angle)*180/np.pi)
        # #     angle[5] = angle[5]
        # #     self.tp.go(angle,0.5)
        # #     self.rexarm.open_gripper()
        # #     self.rexarm.pause(1)
        # # for angle in (section3):
        # #     #print("going to ")
        # #     #print(np.array(angle)*180/np.pi)
        # #     angle[5] = angle[5]
        # #     self.tp.go(angle,0.5)
        # #     self.rexarm.open_gripper()
        # #     self.rexarm.pause(1)
        # # self.rexarm.set_positions([ 0.0, 0.0, 0.0, 0.0, 0.0,0.0])
        # self.rexarm.pause(2)

        # # blocks = self.kinect.blocks
        # pts = [(-200,-200, 907+19),
        #     #    (-200,   0, 907),
        #        (-200, 200, 907+19),
        #     #    (   0, 200, 907),
        #        ( 200, 200, 907+19),
        #     #    ( 200,   0, 907),
        #        ( 200,-200, 907+19),
        #     #    (   0,-200, 907),
        #        (-200,-200, 907+19),]
        # # n = int((950 - z) / 35)
        # step = 10
        # i = 0
        # while True:
        #     self.tp.go([0,0,0,0,0,0])
        #     # blocks = self.kinect.blocks
        #     # for block in blocks:
        #     #     next = block
        #     #     break
        #     # if next == None:
        #     #     continue
        #     # block = next
        #     # block_coord_arm = self.kinect.world2arm((block[0],block[1],block[2]))
        #     # angle = block[3]
        #     # if self.pickBlock(block_coord_arm, angle) == None:
        #     #     #print('unreacheable')
        #     #     continue
        #     angle = 135
        #     place_coord_arm = self.kinect.world2arm(pts[0])
        #     # self.placeBlock1(place_coord_arm, angle)
        #     place_coord = np.array(pts[0])
        #     for pt in pts:
        #         if place_coord[0] > pt[0]:
        #             place_coord[0] -= step
        #         elif place_coord[0] < pt[0]:
        #             place_coord[0] += step
        #         if place_coord[1] > pt[1]:
        #             place_coord[1] -= step
        #         elif place_coord[1] < pt[1]:
        #             place_coord[1] += step
        #         place_coord_arm = self.kinect.world2arm(place_coord)
        #         for x in range(4):
        #             if (self.placeBlock1(place_coord_arm, angle + x * 90)):
        #                 break

    def task5(self):
        self.current_state = "task5"
        self.next_state = "idle"
        self.status_message = "task5"

        blockb = self.kinect.blocks[:]

        # while True:
        #     #print(len(blockb))
        #     self.rexarm.pause(0.3)
        #     if len(blockb)== 2: 
        #         #print('found')
        #         break



        self.kinect.task5=True
        self.kinect.blockDetectorTask5()

        self.rexarm.pause(2)

        

        i = 0

        while True:

            # #print('x')
            blocks = self.kinect.blocks[:]

            if len(blocks) != 2:
                continue
            block0 = blocks[0][:]
            block45 = blocks[1][:]
            
            # #print(blocks[0][0]-blocks[1][0],blocks[0][1]-blocks[1][1])
            

            theta = - np.arctan2(block0[0]-block45[0],block0[1]-block45[1]) 


            if int(block0[3] - theta / np.pi * 180) % 45 > 22.5:
                block0 = blocks[1]
                block45 = blocks[0]
                theta = - np.arctan2(block0[0]-block45[0],block0[1]-block45[1])
            self.rexarm.open_gripper()
            for i in range(2):
                
                if i == 0:
                    block = block0[:]
                    orientation = np.pi /4
                else:
                    block = block45[:]
                    orientation = 0
                    block = block0[:]
                    orientation = np.pi /4

                print (i)


                wps = [[],[]]

                block_coord_arm = self.kinect.world2arm((block[0],block[1],block[2]))
                
                angle = None
                possible = True
                angles1 = None
                for k in range(3):
                    # #print(k)

                    angles1 = wrist_angles_task5_wp(self.rexarm, block_coord_arm, orientation , theta, 40 - k * 20, angles1 )
                    if not angles1:
                        possible = False
                        break
                        
                    if angle == None:
                        angle = angles1[5]
                    angles1[5] = angle
                    wps[i].append(angles1[:])
                    # self.tp.go(angles1,1.5)
                    # self.rexarm.pause(2)
                if possible == False:
                    print('impossible')
                    return
                

                # i += 1

                # self.pickBlock1(self.kinect.world2arm((blockb[i][0],blockb[i][1],blockb[i][2] - 20)), blockb[i][3])

                # for j in range(len(wps)):
                #     self.tp.go(wps[j],1.5)
                # self.rexarm.open_gripper()
                # for j in range(len(wps)):
                #     self.tp.go(wps[len(wps)-1-j],1.5)
                
                    



                # # block_coord_arm = self.kinect.world2arm((block45[0],block45[1],block45[2]))

                

                # # angles2 = wrist_angles_task5(self.rexarm, block_coord_arm, np.pi / 4 , theta )

                # # # #print(block[3]-theta)

                # # if not angles1 or not angles2:
                # #     continue

                # # # angles = np.array(angles)
                # # # angles2 = angles1.copy()
                # # # angles2[1:3] = np.array(0,0)


                # # self.tp.go(angles1,0.5)
                # # self.tp.go(angles2,0.5)
                
                # # self.tp.go(angles1,0.5)

            if possible == False:
                break

            for i in range(2):
                

                self.rexarm.open_gripper()
                print('pick',i)
                for j in range(len(wps[i])):
                    self.tp.go(wps[i][j],1.5)

                self.rexarm.close_gripper()
                self.rexarm.pause(1)

                for j in range(len(wps[i])):
                    self.tp.go(wps[i][len(wps[i])-1-j],1.5)
                print('place',i)

                self.rexarm.pause(1)
                
                self.placeBlock1(self.kinect.world2arm((-100 + i * 50,100 + i * 50,907)), 0)

                self.tp.go([0,0,0,0,0,0],1)
                self.rexarm.pause(1)

            for i in range(2):
                
                if i == 0:
                    block = block45[:]
                    orientation = 0
                else:
                    block = block45[:]
                    orientation = 0

                print (i)


                wps = [[],[]]

                block_coord_arm = self.kinect.world2arm((block[0],block[1],block[2]))
                
                angle = None
                possible = True
                angles1 = None
                for k in range(3):
                    # #print(k)

                    angles1 = wrist_angles_task5_wp(self.rexarm, block_coord_arm, orientation , theta, 40 - k * 20, angles1 )
                    if not angles1:
                        possible = False
                        break
                        
                    if angle == None:
                        angle = angles1[5]
                    angles1[5] = angle
                    wps[i].append(angles1[:])
                    # self.tp.go(angles1,1.5)
                    # self.rexarm.pause(2)
                if possible == False:
                    print('impossible')
                    return
                

                # i += 1

                # self.pickBlock1(self.kinect.world2arm((blockb[i][0],blockb[i][1],blockb[i][2] - 20)), blockb[i][3])

                # for j in range(len(wps)):
                #     self.tp.go(wps[j],1.5)
                # self.rexarm.open_gripper()
                # for j in range(len(wps)):
                #     self.tp.go(wps[len(wps)-1-j],1.5)
                
                    



                # # block_coord_arm = self.kinect.world2arm((block45[0],block45[1],block45[2]))

                

                # # angles2 = wrist_angles_task5(self.rexarm, block_coord_arm, np.pi / 4 , theta )

                # # # #print(block[3]-theta)

                # # if not angles1 or not angles2:
                # #     continue

                # # # angles = np.array(angles)
                # # # angles2 = angles1.copy()
                # # # angles2[1:3] = np.array(0,0)


                # # self.tp.go(angles1,0.5)
                # # self.tp.go(angles2,0.5)
                
                # # self.tp.go(angles1,0.5)

            if possible == False:
                break

            for i in range(2):
                

                self.rexarm.open_gripper()
                print('pick',i)
                for j in range(len(wps[i])):
                    self.tp.go(wps[i][j],1.5)

                self.rexarm.close_gripper()
                self.rexarm.pause(1)

                for j in range(len(wps[i])):
                    self.tp.go(wps[i][len(wps[i])-1-j],1.5)
                print('place',i)

                self.rexarm.pause(1)
                
                self.placeBlock1(self.kinect.world2arm((-100 ,100 ,907)), 0)

                self.tp.go([0,0,0,0,0,0],1)
                self.rexarm.pause(1)
        
            
            # self.rexarm.pause(30)


            # break

            # # 

            # break

    def pickplace(self):
        self.current_state = "pickplace"
        self.next_state = "idle"
        self.status_message = "Click a block to grab"
        while True:
            if(self.kinect.new_click == True):
                grab_coord = self.kinect.last_click.copy()
                self.kinect.new_click = False
                block = self.kinect.locateNearestBlock(grab_coord)
                if block:
                    # #print(block)
                    break
                    block_coord_arm = self.kinect.world2arm((block[0],block[1],block[2]))
                    angle = block[3]
                    # #print((block_coord_arm, angle))
                    if self.pickBlock(block_coord_arm, angle) == None:
                        self.status_message = "(%.0f,%.0f,%.0f) unreachable, please replace the block or click another one" % block_coord_arm
                    else: 
                        break
                else: 
                    self.status_message = "No blocks found, please click on a block"
        
        

        # self.status_message = "Click a location to place the block"
        # while True:
        #     if(self.kinect.new_click == True):
        #         place_coord = self.kinect.last_click.copy()
        #         self.kinect.new_click = False
        #         block = self.kinect.locateNearestBlock(place_coord)
        #         if block:
        #             place_coord_arm = self.kinect.world2arm((block[0],block[1],block[2]))
        #             place_coord_arm = (place_coord_arm[0], place_coord_arm[1], place_coord_arm[2] + 38)
        #             angle = block[3]
        #             self.status_message = "place on the block at (%.0f,%.0f,%.0f)" % place_coord_arm
        #         else: 
        #             place_coord = self.kinect.getWorldCoord(place_coord)

        #             place_coord_arm = (self.kinect.world2arm(place_coord))
        #             #print(place_coord_arm)
        #             place_coord_arm = (place_coord_arm[0], place_coord_arm[1], place_coord_arm[2] + 38)
        #             angle = 0
        #             self.status_message = "place on the board at (%.0f,%.0f,%.0f)" % place_coord_arm
                    
        #         if self.placeBlock(place_coord_arm, angle) == None:
        #             self.status_message = "(%.0f,%.0f,%.0f) unreachable, please choose another one" % place_coord_arm
        #         else:
        #             break

        # self.rexarm.set_positions([0,0,0,0,0,0])
  
    def pickBlock(self,block_coord,angle):
        # return None if unreachable
        # #print(block_coord)
        # self.rexarm.set_speeds(1)
        # self.rexarm.set_speeds([1,1,1,1,1,1])
        block_coord=np.array(block_coord)
        pt1 = block_coord.copy()
        pt2 = block_coord.copy()
        pt3 = block_coord.copy()
        pt1[2] = pt1[2] + 40
        pt2[2] = pt2[2] - 20
        # pt3[2] = pt3[2] + 50
        fromtop = True
        angles1 = wrist_angles(self.rexarm,pt1,angle/180*np.pi,fromtop)
        angles2 = wrist_angles(self.rexarm,pt2,angle/180*np.pi,fromtop)
        if not angles1 or not angles2:
            #print('from top impossble')
            fromtop = False
            angles1 = wrist_angles(self.rexarm,pt1,angle/180*np.pi,fromtop)
            angles2 = wrist_angles(self.rexarm,pt2,angle/180*np.pi,fromtop)

        if not angles1 or not angles2:
            #print('from side impossble')
            return
        # #print(np.array(angles1)/np.pi*180)
        
        # angles3 = wrist_angles(self.rexarm,pt3,angle/180*np.pi)
        # if not angles3:
        #     return
        # self.rexarm.set_positions(np.array(angles3))
        # self.rexarm.pause(3)
        angles2 = np.array(angles2)
        # angles1[5] = angles2[5]
        self.tp.go(np.array(angles1))
        #print('pt1')
        self.rexarm.open_gripper()
        self.rexarm.pause(1)
        self.tp.go(np.array(angles2))
        #print('pt2')
        self.rexarm.pause(1)
        self.rexarm.close_gripper()
        self.rexarm.pause(1)
        self.tp.go(angles1)
        #print('pt3')
        self.rexarm.pause(1)

        return 1

    def pickBlock1(self,block_coord,angle):
        block_coord=np.array(block_coord)
        pt1 = block_coord.copy()
        pt2 = block_coord.copy()
        pt3 = block_coord.copy()
        pt1[2] = pt1[2] + 0
        pt2[2] = pt2[2] - 50
        # pt3[2] = pt3[2] + 50
        fromtop = True
        for i in range(4):
            angles1 = wrist_angles(self.rexarm,pt1,angle/180*np.pi,fromtop,i=i)
            angles2 = wrist_angles(self.rexarm,pt2,angle/180*np.pi,fromtop,angles1,i=i)
        if not angles1 or not angles2:
            #print('from top impossble')
            fromtop = False
            for i in range(4):
                angles1 = wrist_angles(self.rexarm,pt1,angle/180*np.pi,fromtop,i=i)
                angles2 = wrist_angles(self.rexarm,pt2,angle/180*np.pi,fromtop,angles1,i=i)

        if not angles1 or not angles2:
            #print('from side impossble')
            return
        # #print(np.array(angles1)/np.pi*180)
        
        # angles3 = wrist_angles(self.rexarm,pt3,angle/180*np.pi)
        # if not angles3:
        #     return
        # self.rexarm.set_positions(np.array(angles3))
        # self.rexarm.pause(3)
        angles2 = np.array(angles2)
        # angles1[5] = angles2[5]
        self.tp.go(np.array(angles1))
        #print('pt1')
        self.rexarm.open_gripper()
        self.rexarm.pause(1)
        self.tp.go(np.array(angles2))
        #print('pt2')
        self.rexarm.pause(1)
        self.rexarm.close_gripper()
        self.rexarm.pause(1)
        self.tp.go(angles1)
        #print('pt3')
        self.rexarm.pause(1)
        # return None if unreachable
        # #print(block_coord)
        # self.rexarm.set_speeds(1)
        # self.rexarm.set_speeds([1,1,1,1,1,1])
        
        # block_coord=np.array(block_coord)
        # pt1 = block_coord.copy()
        # pt2 = block_coord.copy()
        # pt3 = block_coord.copy()
        # pt1[2] = pt1[2] + 40
        # pt2[2] = pt2[2] - 20
        # # pt3[2] = pt3[2] + 50
        # angles1 = wrist_angles(self.rexarm,pt1,angle/180*np.pi)
        # if not angles1:
        #     return
        # #print(np.array(angles1)/np.pi*180)
        # self.rexarm.open_gripper()
        # self.tp.go(np.array(angles1))
        # #print('pt1')
        # # angles3 = wrist_angles(self.rexarm,pt3,angle/180*np.pi)
        # # if not angles3:
        # #     return
        # # self.rexarm.set_positions(np.array(angles3))
        # angles2 = wrist_angles(self.rexarm,pt2,angle/180*np.pi)
        # if not angles2:
        #     return
        # self.tp.go(np.array(angles2),1)
        # #print('pt2')
        # self.rexarm.pause(1)
        # self.rexarm.close_gripper()
        # self.rexarm.pause(1)
        # self.tp.go(np.array(angles1),1)
        # #print('pt3')
        # # self.rexarm.pause(1)
        return 1

    def pickBlock2(self,block_coord,angle):
        block_coord=np.array(block_coord)
        pt1 = block_coord.copy()
        pt2 = block_coord.copy()
        pt3 = block_coord.copy()
        pt1[2] = pt1[2] + 0
        pt2[2] = pt2[2] - 50
        # pt3[2] = pt3[2] + 50
        fromtop = True
        for i in range(4):
            angles1 = wrist_angles(self.rexarm,pt1,angle/180*np.pi,fromtop)
            angles2 = wrist_angles(self.rexarm,pt2,angle/180*np.pi,fromtop,angles1)
        if not angles1 or not angles2:
            #print('from top impossble')
            fromtop = False
            for i in range(4):
                angles1 = wrist_angles(self.rexarm,pt1,angle/180*np.pi,fromtop)
                angles2 = wrist_angles(self.rexarm,pt2,angle/180*np.pi,fromtop,angles1)
            if not angles1 or not angles2:
                #print('from side impossble')
                return
        # #print(np.array(angles1)/np.pi*180)
        
        # angles3 = wrist_angles(self.rexarm,pt3,angle/180*np.pi)
        # if not angles3:
        #     return
        # self.rexarm.set_positions(np.array(angles3))
        # self.rexarm.pause(3)
        angles2 = np.array(angles2)
        # angles1[5] = angles2[5]
        self.tp.go(np.array(angles1))
        #print('pt1')
        self.rexarm.open_gripper()
        self.rexarm.pause(1)
        self.tp.go(np.array(angles2))
        #print('pt2')
        self.rexarm.pause(1)
        self.rexarm.close_gripper()
        self.rexarm.pause(1)
        self.tp.go(angles1)
        #print('pt3')
        self.rexarm.pause(1)
        # return None if unreachable
        # #print(block_coord)
        # self.rexarm.set_speeds(1)
        # self.rexarm.set_speeds([1,1,1,1,1,1])
        
        # block_coord=np.array(block_coord)
        # pt1 = block_coord.copy()
        # pt2 = block_coord.copy()
        # pt3 = block_coord.copy()
        # pt1[2] = pt1[2] + 40
        # pt2[2] = pt2[2] - 20
        # # pt3[2] = pt3[2] + 50
        # angles1 = wrist_angles(self.rexarm,pt1,angle/180*np.pi)
        # if not angles1:
        #     return
        # #print(np.array(angles1)/np.pi*180)
        # self.rexarm.open_gripper()
        # self.tp.go(np.array(angles1))
        # #print('pt1')
        # # angles3 = wrist_angles(self.rexarm,pt3,angle/180*np.pi)
        # # if not angles3:
        # #     return
        # # self.rexarm.set_positions(np.array(angles3))
        # angles2 = wrist_angles(self.rexarm,pt2,angle/180*np.pi)
        # if not angles2:
        #     return
        # self.tp.go(np.array(angles2),1)
        # #print('pt2')
        # self.rexarm.pause(1)
        # self.rexarm.close_gripper()
        # self.rexarm.pause(1)
        # self.tp.go(np.array(angles1),1)
        # #print('pt3')
        # # self.rexarm.pause(1)
        return 1

    def placeBlock1(self,place_coord_arm,angle):
        # return None if unreachable
        # self.rexarm.set_positions([0,0,0,0,0,0])
        # self.rexarm.pause(1)
        # self.tp.go([0,0,0,0,0,0])
        place_coord_arm=np.array(place_coord_arm)
        pt1 = place_coord_arm.copy()
        pt2 = place_coord_arm.copy()
        pt1[2] = pt1[2]  + 0
        pt2[2] = pt2[2] - 45
        # angles1 = wrist_angles(self.rexarm,pt1,angle/180*np.pi)
        fromtop = True
        for i in range(4):
            angles1 = wrist_angles(self.rexarm,pt1,angle/180*np.pi,fromtop,i=i)
            angles2 = wrist_angles(self.rexarm,pt2,angle/180*np.pi,fromtop,angles1,i=i)
        if not angles1 or not angles2:
            #print('from top impossble')
            fromtop = False
            for i in range(4):
                angles1 = wrist_angles(self.rexarm,pt1,angle/180*np.pi,fromtop,i=i)
                angles2 = wrist_angles(self.rexarm,pt2,angle/180*np.pi,fromtop,angles1,i=i)

        if not angles1 or not angles2:
            #print('from side impossble')
            return
        
        # #print(angles1)
        # self.tp.go(np.array(angles1)/2)
        angles3 = np.array(angles1).copy()
        angles3[2:4] = np.array(angles1)[2:4]/2
        self.tp.go(np.array(angles3))
        self.tp.go(np.array(angles1))
        # self.rexarm.pause(1)
        # angles2 = wrist_angles(self.rexarm,pt2,np.pi/2)
        # # angles2 = wrist_angles(self.rexarm,pt2,angle/180*np.pi)
        # if not angles2:
        #     return
        self.tp.go(np.array(angles2),1)
        self.rexarm.pause(0.5)
        self.rexarm.open_gripper()
        self.rexarm.pause(0.5)
        self.tp.go(angles1)
        # self.rexarm.pause(1)
        # wrist_angles(self.rexarm,place_coord_arm, angle) 
        # self.rexarm.set_positions(coord)
        self.tp.go([0,0,0,0,0,0])
        return 1

    def placeBlock3(self,place_coord_arm,angle, i):
        # return None if unreachable
        # self.rexarm.set_positions([0,0,0,0,0,0])
        # self.rexarm.pause(1)
        # self.tp.go([0,0,0,0,0,0])
        place_coord_arm=np.array(place_coord_arm)
        pt1 = place_coord_arm.copy()
        pt2 = place_coord_arm.copy()
        pt1[2] = pt1[2]  + 40
        pt2[2] = pt2[2] - 7
        # if (i > 5):
        #     pt1 = pt2.copy()
        #     pt1[0] = pt2[0] + 20
        #     pt1[1] = pt2[1] + 20
        # angles1 = wrist_angles(self.rexarm,pt1,angle/180*np.pi)
        fromtop = True
        if (i < 4):
            for i in range(4):
                angles1 = wrist_angles(self.rexarm,pt1,angle/180*np.pi,fromtop)
                angles2 = wrist_angles(self.rexarm,pt2,angle/180*np.pi,fromtop,angles1)
            if not angles1 or not angles2:
                #print('from top impossble')
                fromtop = False
                for i in range(4):
                    angles1 = wrist_angles(self.rexarm,pt1,angle/180*np.pi,fromtop)
                    angles2 = wrist_angles(self.rexarm,pt2,angle/180*np.pi,fromtop,angles1)
            #print(fromtop)
            if not angles1 or not angles2:
                #print('from side impossble')
                return
            
            # #print(angles1)
            # self.tp.go(np.array(angles1)/2)
            angles3 = np.array(angles1).copy()
            angles3[1:4] = np.array(angles1)[1:4]/2
            self.tp.go(np.array(angles3))


            self.tp.go(np.array(angles1))
            # if (i>3):
            #     angles12 = np.array(angles2)
            #     angles12[2] = angles12[2] + np.pi/3
            #     self.tp.go(np.array(angles12),1)
            # angles12 = np.array(angles2)
            # angles12[0:4] = angles1[0:4]
            # self.tp.go(np.array(angles12),1)
            # self.rexarm.pause(1)
            # angles2 = wrist_angles(self.rexarm,pt2,np.pi/2)
            # # angles2 = wrist_angles(self.rexarm,pt2,angle/180*np.pi)
            # if not angles2:
            #     return
            
            self.tp.go(np.array(angles2),1)
            self.rexarm.pause(0.5)
            self.rexarm.open_gripper()
            self.rexarm.pause(0.5)
            self.tp.go(angles1,1)

        else:
            
            fromtop = False
            for i in range(4):
                angles2 = wrist_angles(self.rexarm,pt2,angle/180*np.pi,fromtop)
            # #print(fromtop)
            if not angles2:
                #print('from side impossble')
                return
            
            # #print(angles1)
            # self.tp.go(np.array(angles1)/2)
            # angles3 = np.array(angles1).copy()
            # angles3[1:4] = np.array(angles1)[1:4]/2
            # self.tp.go(np.array(angles3))
            # if (i > 3):
            #     angles1[2] = angles1[2]+np.pi/3

            # self.tp.go(np.array(angles1))
            # if (i>3):
            angles12 = np.array(angles2)
            angles12[2] = angles12[2] + np.pi/3
            self.tp.go(np.array(angles12),1)
            # angles12 = np.array(angles2)
            # angles12[0:4] = angles1[0:4]
            # self.tp.go(np.array(angles12),1)
            # self.rexarm.pause(1)
            # angles2 = wrist_angles(self.rexarm,pt2,np.pi/2)
            # # angles2 = wrist_angles(self.rexarm,pt2,angle/180*np.pi)
            # if not angles2:
            #     return
            
            self.tp.go(np.array(angles2),1)
            self.rexarm.pause(0.5)
            self.rexarm.open_gripper()
            self.rexarm.pause(0.5)
            self.tp.go(np.array(angles12),1)
            # self.tp.go(angles1,1)

        # self.rexarm.pause(1)
        # wrist_angles(self.rexarm,place_coord_arm, angle) 
        # self.rexarm.set_positions(coord)
        self.tp.go([0,0,0,0,0,0])
        return 1
    
    def placeBlock2(self,place_coord_arm,i,angle=90):
        # return None if unreachable
        # self.rexarm.set_positions([0,0,0,0,0,0])
        # self.rexarm.pause(1)
        fromtop = True
        place_coord_arm=np.array(place_coord_arm)
        offset = 0
        if i > 6:
            offset = (i-6) * 20
        pt1 = place_coord_arm.copy()
        pt2 = place_coord_arm.copy()
        pt1[0] = pt1[0] - 40 + offset
        pt1[2] = pt1[2] + 80
        pt2[2] = pt2[2] - 15
        pt2[0] = pt2[0] - 40 + offset
        pt3=pt2.copy()
        pt3[0]=pt3[0]+50 + offset
        angles1 = wrist_angles(self.rexarm,pt1,angle/180*np.pi,fromtop)
        if not angles1:
            return
        # #print(angles1)
        # self.tp.go(np.array(angles1)/2)
        angles2 = np.array(angles1).copy()
        angles2[1:5] = np.array(angles1)[1:5]/2
        self.tp.go(np.array(angles2))
        self.tp.go(np.array(angles1))
        # self.rexarm.pause(1)
        angles2 = wrist_angles(self.rexarm,pt2,angle/180*np.pi,fromtop,angles1)
        # #print(np.array(angles2)/np.pi*180)
        if not angles2:
            return
        self.tp.go(np.array(angles2))
        # self.rexarm.pause(1)
        # while pt3[0] > pt2[0]:
        #     #print(pt3[0])
        #     pt3[0] -= 5
        #     angles3 = wrist_angles(self.rexarm,pt3,angle/180*np.pi)
        #     if not angles3:
        #         return
        #     self.tp.go(np.array(angles3),0.2)
        angles3 = wrist_angles(self.rexarm,pt3,angle/180*np.pi,fromtop,angles2)
        # #print(np.array(angles3)/np.pi*180)
        # angles3[5] = angles2[5]
        self.tp.go(np.array(angles3),0.5)
        self.rexarm.pause(1)
        self.rexarm.open_gripper()
        self.rexarm.pause(1)
        self.tp.go(angles1,0.5)
        self.rexarm.pause(1)
        # wrist_angles(self.rexarm,place_coord_arm, angle) 
        # self.rexarm.set_positions(coord)
        self.tp.go([0,0,0,0,0,0])
        return 1

    

    def placeBlock(self,place_coord_arm,angle):
        # return None if unreachable
        # self.rexarm.set_positions([0,0,0,0,0,0])
        # self.rexarm.pause(1)
        
        place_coord_arm=np.array(place_coord_arm)
        pt1 = place_coord_arm.copy()
        pt2 = place_coord_arm.copy()
        pt1[2] = pt1[2] + 20
        pt2[2] = pt2[2] - 25
        # angles1 = wrist_angles(self.rexarm,pt1,angle/180*np.pi)
        angles1 = wrist_angles(self.rexarm,pt1,np.pi/2)

        if not angles1:
            return
        # #print(angles1)
        self.tp.go(np.array(angles1)/2)
        angles2 = np.array(angles1).copy()
        angles2[0:3] = np.array(angles1)[0:3]/2
        self.tp.go(np.array(angles2))
        self.tp.go(np.array(angles1))
        self.rexarm.pause(1)
        angles2 = wrist_angles(self.rexarm,pt2,np.pi/2)
        # angles2 = wrist_angles(self.rexarm,pt2,angle/180*np.pi)
        if not angles2:
            return
        self.tp.go(np.array(angles2))
        self.rexarm.pause(1)
        self.rexarm.open_gripper()
        self.rexarm.pause(1)
        self.tp.go(angles1)
        self.rexarm.pause(1)
        wrist_angles(self.rexarm,place_coord_arm, angle) 
        # self.rexarm.set_positions(coord)
        self.tp.go([0,0,0,0,0,0])
        return 1

    
    def traj(self):
        # pass
        # IK2(self.rexarm,[-50.,-60.,150.])
        self.rexarm.pause(1)
        link = np.array([117.5,100.5,113,108,0])
        # #print(FK_dh_new([0,0,0,0,-np.pi/2,0],link))[0:-1,0:-1]
               # #print(FK_dh_new([0,0,np.pi/2,0,np.pi/2,0],link))
        end_pos = np.array([200,200,0])
        angle = wrist_angles(self.rexarm,end_pos,np.pi/6)
        # #print(np. array(angle))

        # #print(np. array(angle) / 3.14 * 180)
        # self.rexarm.set_positions(angle)
        self.tp.go([0.1,0.2,np.pi/2,0.3,np.pi/2,0.4])

        # IK(self.rexarm,[300,0,0])
        # self.rexarm.pause(1)
        # IK(self.rexarm,[0,0,300])
        # self.rexarm.pause(1)
        # IK(self.rexarm,[247.,169.,-9.66])
        self.rexarm.pause(1)
        self.status_message = "State: Traj"
        # self.current_state="traj"
        self.next_state = "idle"
        # # self.rexarm.set_positions_wait([ 0.1, 0.1, 0.1, 0.1, 0.1])
        # # #print('done')
        # self.rexarm.joints[4].set_speed(2)
        # self.rexarm.set_positions([ 0.0, 0.0, 0.0, 0.0, 0.0])
        # self.rexarm.pause(2)
        # # link = np.array([0,117.5,100.5,113,108])
        # # np.set_printoptions(suppress=True)
        # planner = TrajectoryPlanner(self.rexarm)
        # planner.go([ 0.0, 0.0, 0.0, 0.0, 0.0])
        # planner.go([ 0.2, 0.2, 0, 0.5, 0.8])
        # planner.go([-1.0,-0.8,-1.0,-0.5, -1.0])angles = wrist_angles_task5(self.rexarm, block_coord_arm, np.pi / 4 , theta )
        # planner.go([-1.0, 0.8, 1.0, 0.5, 1.0])angles = wrist_angles_task5(self.rexarm, block_coord_arm, np.pi / 4 , theta )
        # planner.go([1.0, -0.8,-1.0,-0.5, -1.0])angles = wrist_angles_task5(self.rexarm, block_coord_arm, np.pi / 4 , theta )
        # planner.go([ 0.0, 0.0, 0.0, 0.0, 0.0])angles = wrist_angles_task5(self.rexarm, block_coord_arm, np.pi / 4 , theta )
        # # self.rexarm.pause(2)
        # # self.rexarm.joints[4].set_speed(2)
        # # self.rexarm.set_positions([ 0.0, 0.0, 0.0, 0.0, 0.0])