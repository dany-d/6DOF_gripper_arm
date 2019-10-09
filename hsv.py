import sys
import cv2

font = cv2.FONT_HERSHEY_SIMPLEX

def mouse_callback(event,x,y,flags,param):
    r = img[y][x][2]
    g = img[y][x][1]
    b = img[y][x][0]
    h = hsv[y][x][0]
    s = hsv[y][x][1]
    v = hsv[y][x][2]
    output_rgb = "R:%d, G:%d, B:%d " % (r, g, b)
    output_hsv = "H:%d, S:%d, V:%d" % (h, s, v)
    tmp = img.copy()
    cv2.putText(tmp,output_rgb, (10,20), font, 0.5, (0,0,0))
    cv2.putText(tmp,output_hsv, (10,40), font, 0.5, (0,0,0))
    cv2.imshow('window', tmp)
    if event == cv2.EVENT_LBUTTONDOWN:
        print "bgr: (%d, %d, %d) \nhsv: (%d, %d, %d)" % (b,g,r,h,s,v)

if len(sys.argv) == 2:
    print "Opening " + str(sys.argv[1])
    img = cv2.imread(sys.argv[1])
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    cv2.namedWindow("window",1)
    cv2.imshow('window', img)
    cv2.setMouseCallback("window",mouse_callback)

    while True:
        ch = 0xFF & cv2.waitKey(10)
        if ch == 27:
            break
    cv2.destroyAllWindows()

else:
    print "Expected filename as argument"


def blockDetector(self, boundMin, boundMax):
        """
        TODO:
        Implement your block detector here.  
        You will need to locate
        blocks in 3D space
        """
        self.contoursByDepth = self.detectBlocksInDepthImage(boundMin, boundMax)[:]
        cubeColor = ['black','red','orange','yellow','green','blue','purple','pink']
        # rgbBoundaries = [ # b,g,r
        #     ([4, 160, 240], [50, 210, 253]), # yellow
        #     ([10, 85, 195], [30, 130, 205]), # orange
        #     ([85, 48, 189], [110, 60, 210]), # pink
        #     ([10, 21, 21], [50, 30, 38]), # black
        #     ([26, 22, 140], [60, 38, 160]), # red
        #     ([110, 65, 120], [127, 86, 140]), # purple
        #     ([94, 120, 87],[108, 130, 105]), # greens
        #     ([130, 95, 80], [158, 99, 85]) # blue
        #     ]
        hsvBoundaries = [ # h,s,v
            ([0, 0, 30], [255, 255, 120]), # black
            ([161, 100, 135], [200, 240, 200]), # red
            ([0, 140, 200], [15, 240, 255]), # orange
            ([0, 0, 200], [45, 150, 255]), # yellow
            ([40, 80, 135],[80, 150, 170]), # green
            ([85, 100, 135], [130, 200, 230]), # blue
            ([131, 60, 122], [160, 140, 180]), # purple
            ([150, 130, 210], [180, 190, 255]), # pink
            ]

        ### color detection in rgb image
        # r = self.rgbImage[centerY][centerX][2]
        # g = self.rgbImage[centerY][centerX][1]
        # b = self.rgbImage[centerY][centerX][0]

        self.cubeCenter = []
        del self.detectedCubeColor[:]
        self.cubeContours = []
        self.rectVertex = []
        del self.cubeOrient[:]
        # vertexCoordInWorld = []
        
        camera_coord = []
        colorDetectionPoints = []

        print "debug: hsv"
        print len(hsvBoundaries)
        for j in range(len(hsvBoundaries)):
            (lower,upper) = hsvBoundaries[j]
            lower = np.array(lower, dtype="uint8")
            upper = np.array(upper, dtype="uint8")        
            
            print "debug: contourDepth"
            print len(self.contoursByDepth)  
            for i in range(len(self.contoursByDepth)):      
                # find center of mass
                cubeMoment = cv2.moments(self.contoursByDepth[i])
                centerX = int(cubeMoment["m10"] / cubeMoment["m00"])
                centerY = int(cubeMoment["m01"] / cubeMoment["m00"])

                # # find if center is in world frame
                centerCoordInWorld = np.matmul(self.convert_to_world, [centerX,centerY,1])[:]
            
                if centerCoordInWorld[0] < 0 or centerCoordInWorld[0] > 608 or centerCoordInWorld[1] < 0 or centerCoordInWorld[1] > 603.25 or (220 < centerCoordInWorld[0] and centerCoordInWorld[0] < 390 and 220 < centerCoordInWorld[1] and centerCoordInWorld[1] < 375):
                    continue
            
                # color detection points array
                colorDetectionPoints = [(centerX-3,centerY-3), (centerX-3,centerY-2), (centerX-3,centerY-1), (centerX-3,centerY), (centerX-3,centerY+1), (centerX-3,centerY+2), (centerX-3,centerY+3), 
                    (centerX-2,centerY-3), (centerX-2,centerY-2), (centerX-2,centerY-1), (centerX-2,centerY), (centerX-2,centerY+1), (centerX-2,centerY+2), (centerX-2,centerY+3), 
                    (centerX-1,centerY-3), (centerX-1,centerY-2), (centerX-1,centerY-1), (centerX-1,centerY), (centerX-1,centerY+1), (centerX-1,centerY+2), (centerX-1,centerY+3), 
                    (centerX,centerY-3), (centerX,centerY-2), (centerX,centerY-1), (centerX,centerY), (centerX,centerY+1), (centerX,centerY+2), (centerX,centerY+3), 
                    (centerX+1,centerY-3), (centerX+1,centerY-2), (centerX+1,centerY-1), (centerX+1,centerY), (centerX+1,centerY+1), (centerX+1,centerY+2), (centerX+1,centerY+3),
                    (centerX+2,centerY-3), (centerX+2,centerY-2), (centerX+2,centerY-1), (centerX+2,centerY), (centerX+2,centerY+1), (centerX+2,centerY+2), (centerX+2,centerY+3),
                    (centerX+3,centerY-3), (centerX+3,centerY-2), (centerX+3,centerY-1), (centerX+3,centerY), (centerX+3,centerY+1), (centerX+3,centerY+2), (centerX+3,centerY+3),
                    ]
                hSum = 0
                sSum = 0
                vSum = 0
                hAve = 0
                sAve = 0
                vAve = 0
                len1 = 0

                for k in range(len(colorDetectionPoints)):
                    # find hsv
                    h = self.hsvImage[colorDetectionPoints[k][1]][colorDetectionPoints[k][0]][0]
                    s = self.hsvImage[colorDetectionPoints[k][1]][colorDetectionPoints[k][0]][1]
                    v = self.hsvImage[colorDetectionPoints[k][1]][colorDetectionPoints[k][0]][2]
                    hSum = hSum + h
                    sSum = sSum + s
                    vSum = vSum + v
                hAve = hSum/len(colorDetectionPoints)
                sAve = sSum/len(colorDetectionPoints)
                vAve = vSum/len(colorDetectionPoints)

                if hAve >= lower[0] and hAve <= upper[0] and sAve >= lower[1] and sAve <= upper[1] and vAve >= lower[2] and vAve <= upper[2]:
                    
                    # approximate bounding rectangle
                    rect = cv2.minAreaRect(self.contoursByDepth[i])
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)

                    # judge if contour is cube # cube length in mouse coord: at least 19
                    len1 = ((box[0][0]-box[1][0])**2+(box[0][1]-box[1][1])**2)**0.5
                    len2 = ((box[0][0]-box[3][0])**2+(box[0][1]-box[3][1])**2)**0.5
                    if (int(len1) < 21) or (int(len2) < 21) or (int(len1) > 40) or (int(len2) > 40):
                        continue

                    # record vertexs
                    self.rectVertex.append(box)
                    # vertex coords in world frame
                    vertexCoord1 = np.matmul(self.convert_to_world, [box[0][0],box[0][1],1])
                    vertexCoord2 = np.matmul(self.convert_to_world, [box[1][0],box[1][1],1])
                    # self.vertexCoordInWorld.append([vertexCoord1,vertexCoord2])
                    # record orientation in world frame
                    angle = np.arctan((vertexCoord2[1]-vertexCoord1[1])/(vertexCoord2[0]-vertexCoord1[0]))
                    self.cubeOrient.append(angle)
                    # define colors
                    self.detectedCubeColor.append(cubeColor[j])
                    # record contours
                    self.cubeContours.append(self.contoursByDepth[i])
                    # record center coords
                    self.cubeCenter.append([int(centerCoordInWorld[0]),int(centerCoordInWorld[1])])
                    camera_coord.append([centerX,centerY])
                else:
                    continue

        if(self.blockMessage or True):
            print "kinect msg"
            print self.cubeCenter
            print self.cubeOrient
            print self.detectedCubeColor
            print camera_coord

        return camera_coord


def detectBlocksInDepthImage(self,boundMin, boundMax):
        """
        Implement a blob detector to find blocks
        in the depth image
        """
        self.captureDepthFrame()
        print "bound: ", boundMin, boundMax
        contoursDepth = np.array([])
        # convert depthImage into 8 bits
        depthImage = self.currentDepthFrame[:]
        np.clip(depthImage,0,2**10 - 1,depthImage)
        depthImage >>= 2
        depthImage = depthImage.astype(np.uint8)

        # load rgb image produce hsv image 
        self.rgbImage = self.currentVideoFrame
        self.rgbImage = cv2.cvtColor(self.rgbImage, cv2.COLOR_BGR2RGB)
        self.hsvImage = cv2.cvtColor(self.rgbImage, cv2.COLOR_BGR2HSV)
        # use grayscale in depth to measure the depth of object

        # detect object
        grayBoundaries = [(boundMin, boundMax)]
            # (160,169)] # 3st layer
            # (170,173), # 2st layer
            # (174,177)] # 1st layer

        for i in range(len(grayBoundaries)):
            (grayLower,grayUpper) = grayBoundaries[i]
            grayLower = np.array(grayLower,dtype = "uint8")
            grayUpper = np.array(grayUpper,dtype = "uint8")
            grayThreshold = cv2.inRange(depthImage,grayLower,grayUpper)
            # dilation
            kernel = np.ones((5,5),np.uint8)
            grayDilation = cv2.dilate(grayThreshold,kernel,iterations = 1)
            # find countors
            _, contours, _ = cv2.findContours(grayDilation,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            contoursDepth = np.append(contoursDepth, contours)

        print "in detectBlocksInDepthImage, contour depth has length", len(contoursDepth)
        return contoursDepth