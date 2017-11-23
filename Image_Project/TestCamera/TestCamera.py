import cv2

cam = cv2.VideoCapture(0)
while True:
    ret, frame = cam.read()
    cv2.imshow("Camera", frame)
    k = cv2.waitKey(1)
    if k == ord("c"):
        cv2.imwrite("Capture.png", frame)
    elif k == 27:
        cv2.destroyAllWindows()
        cam.release()
        break
