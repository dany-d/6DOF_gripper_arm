import cv2
import numpy as np
import freenect

calib = np.load('calib_map.npz')
ret=calib['ret']
mtx=calib['mtx']
dist=calib['dist']
rvecs=calib['rvecs']
tvecs=calib['tvecs']

while True:
    depth_frame = freenect.sync_get_depth()[0]

    rgb_frame = freenect.sync_get_video()[0]
    rgb_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)

    np.clip(depth_frame,0,2**10 - 1,depth_frame)
    depth_frame >>= 2
    depth_frame = depth_frame.astype(np.uint8)
    print(depth_frame.shape)
    print(rgb_frame.shape)

    h,  w = rgb_frame.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
    dst = cv2.undistort(rgb_frame, mtx, dist, None, newcameramtx)

    # crop the image
    # x,y,w,h = roi
    # dst = dst[y:y+h, x:x+w]

    frame = np.zeros(rgb_frame.shape)
    for i in range(3):
        frame[:,:,i] = rgb_frame[:,:,i] / 2 + depth_frame / 2

    cv2.namedWindow("window",cv2.WINDOW_AUTOSIZE)
    cv2.imshow('window', depth_frame.astype('uint8'))
    ch = 0xFF & cv2.waitKey(10)
    if ch == 0x1B:
            break
cv2.destroyAllWindows()