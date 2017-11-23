import numpy as np
import cv2
import sys


def get_coor(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        coor.append((x, y))
    if len(coor) == 4:
        cv2.destroyWindow("image")


img = cv2.imread("newcap1.png")
# img = cv2.GaussianBlur(img, (3, 3), 1)

kernel = np.zeros((3, 3), np.uint8)

dat = np.load("Variable.npz")

h, w = img.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(dat['mtx'], dat['dist'], (w, h), 1, (w, h))
mapx, mapy = cv2.initUndistortRectifyMap(dat['mtx'], dat['dist'], None, newcameramtx, (w, h), 5)
dst = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

pts = []
re_arrange = []
package = []

'''
# crop the image
x, y, w, h = roi
# dst = dst[y:y + h, x:x + w]
cv2.imwrite('calibresult2.png', dst)
'''

coor = []
valid_contour = []

cv2.namedWindow("image")
cv2.setMouseCallback('image', get_coor)
cv2.imshow("image", img)
cv2.waitKey(0)
print("Co-ordinate", coor)
if len(coor) != 4:
    print("Not enough Co-ordinate (Need 2)")
    pass
else:

    cv2.norm((coor[0][0], coor[0][1]), (coor[1][0], coor[1][1]))
    crop_img = img[coor[2][1]:coor[3][1], coor[2][0]:coor[3][0]]
    gap_y = abs(coor[0][1] - coor[2][1])
    gap_x = abs(coor[0][0] - coor[2][0])
    edges = cv2.Canny(crop_img, 0, 125)
    # for i in range(2):
    cv2.bitwise_not(edges, edges)
    edges = cv2.morphologyEx(edges, cv2.MORPH_DILATE, kernel)
    cv2.bitwise_not(edges, edges)

    # cv2.imshow("Crop_Img", np.hstack([crop_img, edges]))

    _, contours, hierarchy = cv2.findContours(edges, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    print("All contour: ", len(contours))
    for cnt in contours:
        M = cv2.moments(cnt)
        if int(M['m00']) <= 0:
            continue
        elif int(M['m00']) >= 15:
            continue
        else:
            valid_contour.append(cnt)

    print("Valid contour: ", float(len(valid_contour)) / 2)
    for valid in valid_contour[::2]:
        M = cv2.moments(valid)
        print(int(M['m00']))
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        midpoint = [cy, cx]
        cv2.drawContours(crop_img, [valid], -1, (0, 255, 0), -1)
        # cv2.imshow("Crop_img", crop_img)
        # cv2.waitKey(0)

        approx = cv2.approxPolyDP(valid, 0.01 * cv2.arcLength(valid, True), True)
        # cv2.drawContours(crop_img, [approx], -1, (0, 0, 255), 3)

        pts.append([[midpoint]])
    counter = 0
    # print(pts)
    for i in pts:
        # re_arrange.append([])
        for j in i:
            re_arrange.append((int((j[0][0] + gap_x) * 0.077668 * 100), int((j[0][1] + gap_y) * 0.077668 * 100)))
        counter += 1
        # TODO If error about truth value appear again use counter instead (same indent level as this comment)
    # print(re_arrange)
    package.append([])
    for i in [re_arrange]:

        for c in range(len(i)):
            package[0].append([])
            package[0][c].append(i[c])
            if c == len(i) - 1:
                package[0][c].append(i[0])
            else:
                package[0][c].append(i[c + 1])
    print(package)
    for i in package:
        for j in i:
            print(j)  # TODO Change print to send function
        print("Next")

    cv2.imshow("Edge and Crop_img", crop_img)
    cv2.imshow("Edge", edges)

    cv2.waitKey(0)
    cv2.destroyWindow("Crop_Img")
    cv2.destroyWindow("Edge")
    exit()
