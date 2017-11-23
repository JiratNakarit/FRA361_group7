import cv2

cam = cv2.VideoCapture(0)
counter = 0
while True:
    ret, frame = cam.read()
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    img = cv2.Canny(img, 50, 255)

    cv2.imshow("Camera", frame)
    k = cv2.waitKey(1)
    if k == ord("c"):
        cv2.imwrite("cari" + str(counter) + ".jpg", frame)
        counter += 1
    elif k == 27:
        cv2.destroyAllWindows()
        cam.release()
        break
