import numpy as np
import cv2


def get_coor(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        coor.append((x, y))
    if len(coor) == 2:
        cv2.destroyWindow("image")


img = cv2.imread("Capture7.png")
img = cv2.GaussianBlur(img, (3, 3), 3)
coor = []
valid_contour = []

cv2.namedWindow("image")
cv2.setMouseCallback('image', get_coor)
cv2.imshow("image", img)
cv2.waitKey(0)
print("Co-ordinate", coor)
if len(coor) != 2:
    print("Not enough Co-ordinate (Need 2)")
    pass
else:
    crop_img = img[coor[0][1]:coor[1][1], coor[0][0]:coor[1][0]]
    edges = cv2.Canny(crop_img, 125, 255)
    # cv2.imshow("Crop_Img", np.hstack([crop_img, edges]))

    _, contours, hierarchy = cv2.findContours(edges, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    print("All contour: ", len(contours))
    for cnt in contours:
        M = cv2.moments(cnt)
        if int(M['m00']) <= 50:
            continue
        elif int(M['m00']) >= 80000:
            continue
        else:
            valid_contour.append(cnt)

    print("Valid contour: ", float(len(valid_contour)) / 2)
    for valid in valid_contour[::2]:
        M = cv2.moments(valid)
        print(int(M['m00']))
        cv2.drawContours(crop_img, [valid], 0, (0, 255, 0), -1)
        cv2.imshow("Crop_img", crop_img)
        cv2.waitKey(0)

    # cv2.imshow("Crop_img", crop_img)
    cv2.imshow("Edge", edges)

    cv2.waitKey(0)
    cv2.destroyWindow("Crop_Img")
    cv2.destroyWindow("Edge")
