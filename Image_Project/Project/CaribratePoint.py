import numpy as np
import cv2

cam = cv2.VideoCapture(0)
counter = 0
co_or = []


# frame = cv2.imread("Capture5.png") # TODO Using image


def get_co_or(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        co_or.append((x, y))
    if len(co_or) == 2:
        cv2.destroyWindow("Camera")


cv2.namedWindow("Camera")
cv2.setMouseCallback('Camera', get_co_or)
while True:
    ret, frame = cam.read()
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    cv2.imshow("Camera", frame)
    k = cv2.waitKey(1)
    if k == 27:
        cv2.destroyAllWindows()
        cam.release()
        break
    if len(co_or) == 2:
        cv2.destroyWindow("Camera")
        cam.release()
        break
if len(co_or) != 2:
    print("Not enough Co-ordinate (Need 2)")
else:
    print(co_or)
    np.savez("Co_oreiei", co_or=co_or)
