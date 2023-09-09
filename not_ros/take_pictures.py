#take pictures from camera when space is rpessed and save into folder
import cv2

cap = cv2.VideoCapture(2)
count = 0
while True:
    ret, frame = cap.read()
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord(' '):
        print('Click')
        cv2.imwrite('not_ros/images/pic'+str(count)+'.jpg',frame)
        print("path", 'not_ros/images/pic'+str(count)+'.jpg')                                                                                                                                                                               
        count += 1
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print('Quit')
        cap.release()
        break