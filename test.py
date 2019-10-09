import numpy as np

def getAffineTransform(coord1, coord2):
    """
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

    A = np.array([ \
        [[x[i,0], x[i,1], 1, 0, 0, 0], \
        [0, 0, 0, x[i,0], x[i,1], 1]]  \
        for i in range(n)])

    B = np.array(y[:,0:2]).flatten()

    A = A.reshape((2*n, 6))
    B = B.reshape((2*n, 1))

    affine = np.append(np.array(np.linalg.lstsq(A, B))[0],[0,0,1]).reshape((3,3))

    return affine


range = [700,715]
depthImage = self.currentDepthFrame[:]
print(np.max(depthImage))
print(depthImage.shape)
depthImage = cv2.inRange(depthImage,range[0],range[1])
kernel = np.ones((5,5),np.uint8)
dilation = cv2.dilate(depthImage,kernel,iterations = 1)
contours, _ = cv2.findContours(dilation,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
