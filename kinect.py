import cv2
import numpy as np
from PyQt4.QtGui import QImage
import freenect
import time


class Kinect():
    def __init__(self):
        self.currentVideoFrame = np.array([])
        self.currentDepthFrame = np.array([])
        self.currentBlockFrame = np.array([])
        self.mask = np.ones((480,640))
        self.block_mask = np.zeros((480,640))
        if(freenect.sync_get_depth() == None):
            self.kinectConnected = False
        else:
            self.kinectConnected = True
        self.camX = 320
        self.camY = 240
        self.boardZ = 950
        self.size = {'XMIN': -304.8, 'XMAX': 304.8, 'YMIN': -304.8,'YMAX': 304.8}
        self.blocks = []
        self.task5 = False

        

        # mouse clicks & calibration variables
        try:
            affine = np.load('affine.npz')
            self.depth2rgb_affine = affine['depth2rgb']
            self.rgb2world_affine = affine['rgb2world']
            self.kinectCalibrated = True
            self.getMask()
        except:
            self.depth2rgb_affine = np.float32([[1,0,0],[0,1,0]])
            self.rgb2world_affine = np.float32([[1,0,0],[0,1,0]])
            self.kinectCalibrated = False
        self.last_click = np.array([0,0])
        self.new_click = False
        self.rgb_click_points = np.zeros((5,2),int)
        self.depth_click_points = np.zeros((5,2),int)
        self.world_points = np.array([
            [self.size['XMIN'],self.size['YMIN']],
            [self.size['XMIN'],self.size['YMAX']],
            [self.size['XMAX'],self.size['YMAX']],
            [self.size['XMAX'],self.size['YMIN']],
            [0,0]])

        """ Extra arrays for colormaping the depth image"""
        self.DepthHSV = np.zeros((480,640,3)).astype(np.uint8)
        self.DepthCM=np.array([])

        """ block info """
        self.hue_range=np.array([[0,5],[25,35],[50,70],[110,120],[160,180]])
        self.block_contours = np.array([])

    def captureVideoFrame(self):
        """                      
        Capture frame from Kinect, format is 24bit RGB    
        """
        if(self.kinectConnected):
            frame = freenect.sync_get_video()[0]
            # print(self.mask)
            frame = cv2.bitwise_and(frame,frame,mask = self.mask.astype('uint8'))
            self.currentVideoFrame = frame
        else:
            self.loadVideoFrame()
        

    def processVideoFrame(self):
        # cv2.drawContours(self.currentVideoFrame,self.block_contours,-1,(255,0,255),3)
        if self.kinectCalibrated:
            if self.task5:
                self.blockDetectorTask5()
            else:
                self.blockDetector()

    def captureDepthFrame(self):
        """                      
        Capture depth frame from Kinect, format is 16bit Grey, 10bit resolution.
        """
        if(self.kinectConnected):
            if(self.kinectCalibrated):
                self.currentDepthFrame = self.registerDepthFrame(freenect.sync_get_depth()[0])
            else:
                self.currentDepthFrame = freenect.sync_get_depth()[0]

        else:
            self.loadDepthFrame()

    
    def loadVideoFrame(self):
        self.currentVideoFrame = cv2.cvtColor(
            cv2.imread("data/ex0_bgr.png",cv2.IMREAD_UNCHANGED),
            cv2.COLOR_BGR2RGB
            )

    def loadDepthFrame(self):
        self.currentDepthFrame = cv2.imread("data/ex0_depth16.png",0)

    def convertFrame(self):
        """ Converts frame to format suitable for Qt  """
        try:
            img = QImage(self.currentVideoFrame,
                             self.currentVideoFrame.shape[1],
                             self.currentVideoFrame.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def convertBlockFrame(self):
        """ Converts frame to format suitable for Qt  """
        try:
            img = QImage(self.currentBlockFrame,
                             self.currentBlockFrame.shape[1],
                             self.currentBlockFrame.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def getMask(self):
        (m,n) = (480,640)
        mask = np.ones((m,n))
        for y in range(m):
            for x in range(n):
                world = self.rgb2world_affine.dot(np.array([x,y,1]))
                # print(world)
                if world[0] < self.size['YMIN'] or world[0] > self.size['YMAX'] or world[1] < self.size['XMIN'] or world[1] > self.size['XMAX']:
                    mask[y,x] = 0
        self.mask = (mask * 255).astype(np.uint8)
        # print(mask)

    def convertDepthFrame(self):
        """ Converts frame to a colormaped format suitable for Qt  
            Note: this cycles the spectrum over the lowest 8 bits
        """
        try:

            """ 
            Convert Depth frame to rudimentary colormap
            """
            self.DepthHSV[...,0] = self.currentDepthFrame
            self.DepthHSV[...,1] = 0x9F
            self.DepthHSV[...,2] = 0xFF
            self.DepthCM = cv2.cvtColor(self.DepthHSV,cv2.COLOR_HSV2RGB)
            cv2.drawContours(self.DepthCM,self.block_contours,-1,(0,0,0),3)

            img = QImage(self.DepthCM,
                             self.DepthCM.shape[1],
                             self.DepthCM.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def getWorldCoord(self, pt):
        if len(pt) == 3:
            (x,y,z) = pt
            if (x >= 640 or y >= 480):
                # print((x,y))
                return None,None,None
        elif len(pt) == 2:
            (x,y) = pt
            # print(pt)
            if (x >= 640 or y >= 480):
                # print((x,y))
                return None,None,None
            z = self.currentDepthFrame[y,x]
        else:
            return None,None,None
        
        dist = 0.1236 * np.tan(z/2842.5 + 1.1863) * 1000
        x = (x-self.camX) * dist / self.boardZ + self.camX
        y = (y-self.camY) * dist / self.boardZ + self.camY
        img_pt = np.array([x,y,1])
        world_pt = (self.rgb2world_affine).dot(img_pt.T) 
        # print(world_pt)
        return (world_pt[0],world_pt[1], dist)

    def world2arm(self,world_pt):
        return (-world_pt[0], -world_pt[1], - world_pt[2] + self.boardZ - 117)


    def getAffineTransform(self, coord1, coord2):
        """
        self.depth2rgb_affine = affine
        self.kinectCalibrated = True
        Given 2 sets of corresponding coordinates, 
        find the affine matrix transform between them.

        TODO: Rewrite this function to take in an arbitrary number of coordinates and 
        find the transform without using cv2 functions
        """
        # pts1 = coord1[0:3].astype(np.float32)
        # pts2 = coord2[0:3].astype(np.float32)
        # print(cv2.getAffineTransform(pts1,pts2))
        # return cv2.getAffineTransform(pts1,pts2)

        x = coord1
        y = coord2

        n,_ = coord1.shape

        # n -= 1

        A = np.array([ \
            [[x[i,0], x[i,1], 1, 0, 0, 0], \
            [0, 0, 0, x[i,0], x[i,1], 1]]  \
            for i in range(n)])

        B = np.array(y[:,0:2]).flatten()

        A = A.reshape((2*n, 6))
        B = B.reshape((2*n, 1))

        affine = np.array(np.linalg.lstsq(A, B))[0].reshape((2,3))

        return affine

    def registerDepthFrame(self, frame):
        """
        Using an Affine transformation, transform the depth frame to match the RGB frame
        """
        # print(self.depth2rgb_affine.shape)
        # print(frame)
        (m,n) = frame.shape
        return cv2.warpAffine(frame,self.depth2rgb_affine,(n,m))
        # print(affined)

    def loadCameraCalibration(self):
        """
        Load camera intrinsic matrix from file.
        """
        calib = np.load('calib_map.npz')
        mtx=calib['mtx']
        self.camX = mtx[1,2]
        self.camY = mtx[0,2]
    
    def blockDetector(self):
        bframe = self.currentVideoFrame[:]
        blocks = []
        
        hsvFrame = cv2.cvtColor(cv2.cvtColor(self.currentVideoFrame, cv2.COLOR_RGB2BGR), cv2.COLOR_BGR2HSV)

        hsvRanges = [
            # ([0, 0, 0], [255, 255, 255]),
            ([0, 0, 0], [255, 255, 125],'black'), # black
            ([140, 115, 140], [180, 240, 220],'red'), # red1
            ([0, 80, 150], [40, 240, 255],'orange'), # orange
            ([0, 0, 150], [45, 120, 255],'yellow'), # yellow
            ([40, 0, 125],[80, 150, 255],'green'), # green
            ([85, 50, 120], [130, 200, 255],'blue'), # blue
            ([120, 50, 185], [180, 150, 255],'pink'), # pink
            ([130, 20, 100], [160, 200, 180],'violet'), # purple
        ]
        # if self.task5:
        #     contours = self.detectBlocksInColorImage()
        # else:
        contours = self.detectBlocksInDepthImage()
        # cv2.drawContours(self.currentVideoFrame,contours,0,(0,0,255),2)
        # print(len(contours))
        level = 0
        for contour in contours:
            # cv2.drawContours(self.currentVideoFrame,contour,0,(0,0,255),2)
            for c in contour:
                area = cv2.contourArea(c)
                if (area < 100):
                    continue
                moments = cv2.moments(c)
                centerX, centerY = int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])
                centerXw, centerYw, centerZw = self.getWorldCoord((centerX,centerY))
                if centerYw < self.size['YMIN'] or centerYw > self.size['YMAX'] or centerXw < self.size['XMIN'] or centerXw > self.size['XMAX']:
                    continue
                # cv2.drawContours(self.currentVideoFrame,[c],-1,(0,0,255),2)

                # print(area,(centerX,centerY),(centerXw,centerYw))
                mask = np.zeros((480,640), np.uint8)
                cv2.drawContours(mask, [c], -1, 255, -1)
                # self.currentDepthFrame = mask
                mean,_ = cv2.meanStdDev(hsvFrame, mask=mask.astype(np.uint8))
                # print('mean'),mean
                # mean = cv2.mean(hsvFrame)
                # print('mean'),mean
                # mean = hsvFrame[centerY,centerX]
                # print('mean'),mean
                # # print(mean[1])
                for hsv in hsvRanges:
                    inrange = True
                    for i in range(3):
                        inrange = inrange and mean[i,0] >= hsv[0][i] and mean[i,0] <= hsv[1][i]
                    if inrange:
                        # print(mean[0],mean[1],mean[2])
                        color = hsv[2]
                        break
                if not inrange:
                    continue
                # print(inrange)
                rect = cv2.minAreaRect(c)
                area = rect[1][0] * rect[1][1]
                k = 905 / centerZw
                # print(area)
                if (area > 27 * 27 * k * k or area < 21 * 21 * k * k or rect[1][0] < 20 * k  or rect[1][0] > 28 * k  or rect[0][0] < 20 * k  or rect[0][0] < 28 * k ):
                    continue
                # print(rect[1])
                orientation = rect[2]
                # print(rect)
                verteces = cv2.boxPoints(rect).astype('int')
                centerZw = centerZw - 19
                blocks.append([centerXw, centerYw, centerZw, orientation, color, level])
                cv2.drawContours(bframe,[verteces],0,(0,255,255),2)
                # if (area > 27 * 27 * k * k or area < 21 * 21 * k * k or rect[1][0] < 20 * k  or rect[1][0] > 28 * k  or rect[0][0] < 20 * k  or rect[0][0] < 28 * k ):
                #     cv2.drawContours(bframe,[verteces],0,(255,255,255),2)
            level += 1
        self.blocks = blocks
        # print(blocks)
        self.currentBlockFrame = bframe
            

        # print(contours)

    def locateNearestBlock(self, coord, min = 500):
        blockn = None
        coord = self.getWorldCoord(coord)
        self.blockDetector()
        for block in self.blocks:
            dist = (coord[0] - block[0]) ** 2 + (coord[1] - block[1]) ** 2
            if dist < min:
                min = dist
                blockn = block
        return blockn

    def detectBlocksInDepthImage(self):
        depths = []
        # dist = 0.1236 * np.tan(z/2842.5 + 1.1863) * 1000
        for i in range(1,10):
            blockZ = self.boardZ - i*38.3
            blockDepth = (np.arctan(blockZ / 1000 / 0.1246) - 1.1863)*2842.5
            depths.append(blockDepth.astype(np.int))
        # depths.append()
        # print(depths)
        # ranges = [[705,715],[685,695],[670,680],[645,655]]
        contours = []
        depthImage = self.currentDepthFrame[:]
        # print(depthImage)
        # print(np.max(depthImage))
        # print(depthImage.shape)
        binaryBlock = np.zeros(depthImage.shape)
        for d in depths:
            # binaryBlock = np.logical_or(binaryBlock, cv2.inRange(depthImage,d-5,d+5))
            binaryBlock = cv2.inRange(depthImage,d-5,d+5)
            contour, _ = cv2.findContours(binaryBlock.astype(np.uint8),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            contours.append(contour)
            # kernel = np.ones((5,5),np.uint8)
            # depthImage = cv2.dilate(depthImage,kernel,iterations = 1)
            # contour, _ = cv2.findContours(depthImage,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            # print(len(contour))
            # contours = contours + contour
            # print(len(contours))
        # cv2.drawContours(self.currentVideoFrame,contours,-1,(255,0,255),2)
        # self.currentD
        # # epthFrame = depthImage
        # binaryBlock = binaryBlock * 255
        # contour, _ = cv2.findContours(binaryBlock.astype(np.uint8),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def detectBlocksInColorImage(self):
        hsvFrame = cv2.cvtColor(cv2.cvtColor(self.currentVideoFrame, cv2.COLOR_RGB2BGR), cv2.COLOR_BGR2HSV)

        hsvRanges = [
            # ([0, 0, 0], [255, 255, 255]),
            ([0, 0, 0], [255, 255, 110],'black'), # black
            ([140, 115, 140], [180, 240, 220],'red'), # red1
            ([0, 100, 150], [30, 240, 255],'orange'), # orange
            # ([0, 20, 150], [45, 150, 255],'yellow'), # yellow
            ([40, 20, 125],[80, 150, 255],'green'), # green
            ([85, 50, 120], [130, 200, 255],'blue'), # blue
            ([120, 50, 185], [180, 150, 255],'pink'), # pink
            ([130, 20, 100], [160, 200, 180],'violet'), # purple
        ]

        contours = []

        for hsv in hsvRanges:
            # binaryBlock = np.logical_or(binaryBlock, cv2.inRange(depthImage,d-5,d+5))
            hsvBlock = cv2.inRange(hsvFrame,np.array(hsv[0],np.uint8),np.array(hsv[1],np.uint8))
            contour, _ = cv2.findContours(hsvBlock.astype(np.uint8),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            contours.append(contour)

        return contours

    def blockDetectorTask5(self):
        bframe = self.currentVideoFrame[:]
        blocks = []
        
        hsvFrame = cv2.cvtColor(cv2.cvtColor(self.currentVideoFrame, cv2.COLOR_RGB2BGR), cv2.COLOR_BGR2HSV)
        contours = self.detectBlocksInColorImage()

        # cv2.drawContours(self.currentVideoFrame,contours,0,(0,0,255),2)
        # print(len(contours))
        for contour in contours:
            # cv2.drawContours(self.currentVideoFrame,contour,0,(0,0,255),2)
            for c in contour:
                area = cv2.contourArea(c)
                if (area < 100):
                    continue
                moments = cv2.moments(c)
                centerX, centerY = int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])
                centerXw, centerYw, centerZw = self.getWorldCoord((centerX,centerY))
                if centerYw < self.size['YMIN'] or centerYw > self.size['YMAX'] or centerXw < self.size['XMIN'] or centerXw > self.size['XMAX'] or centerZw < 700 or centerZw > 950:
                    continue
                # cv2.drawContours(self.currentVideoFrame,[c],-1,(0,0,255),2)

                # print(area,(centerX,centerY),(centerXw,centerYw))
                mask = np.zeros((480,640), np.uint8)
                cv2.drawContours(mask, [c], -1, 255, -1)
                rect = cv2.minAreaRect(c)
                area = rect[1][0] * rect[1][1]
                # k = 905 / centerZw
                # print(area)
                
                if (area > 1250 or area < 500 or rect[1][0]<19 or rect[1][1]<19 or centerZw > 890 or centerZw < 850):
                    continue
                    # pass
                # print(area,rect[1])
                # print(rect[1])
                orientation = rect[2]
                # print(rect)
                verteces = cv2.boxPoints(rect).astype('int')
                # print(centerZw)
                centerZw = centerZw - 19
                blocks.append([centerXw, centerYw, self.boardZ - 44, orientation, None, None])
                cv2.drawContours(bframe,[verteces],0,(0,255,255),2)
                # if (area > 1250 or area < 500 or rect[1][0]<19 or rect[1][1]<19):
                #     cv2.drawContours(bframe,[verteces],0,(255,255,255),2)
                blocks
            # level += 1
        self.blocks = blocks
        # print(blocks)
        self.currentBlockFrame = bframe