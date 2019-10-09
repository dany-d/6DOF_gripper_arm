import cv2
import freenect

img_counter=0

while True:
    rgb_frame = freenect.sync_get_video()[0]
    rgb_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)

    cv2.namedWindow("window",cv2.WINDOW_AUTOSIZE)

    k = cv2.waitKey(1)

    gray = cv2.cvtColor(rgb_frame,cv2.COLOR_BGR2GRAY)
    # cv2.imshow('winname', gray)
    # cv2.waitKey(0)  

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (8,6),None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        chess = cv2.drawChessboardCorners(rgb_frame, (8,6), corners,ret)
        cv2.imshow('winname', chess)
        # cv2.imwrite('chess3/chess'+str(counter)+side+fname.split('/')[1],chess)
        # cv2.waitKey(0)
    else:
        cv2.imshow('window', rgb_frame)

    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        break
    elif k%256 == 32:
        # SPACE pressed
        img_name = "blocks/img_{}.jpg".format(img_counter)
        cv2.imwrite(img_name, rgb_frame)
        print("{} written!".format(img_name))
        img_counter += 1
cv2.destroyAllWindows()
