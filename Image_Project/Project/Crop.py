import numpy as np
import cv2
import sys


def get_coor(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        coor.append((x, y))
    if len(coor) == 2:
        cv2.destroyWindow("image")


def color_detection(color, poly):
    example = np.uint8([[color]])
    hsv_ = cv2.cvtColor(example, cv2.COLOR_BGR2HSV_FULL)
    # print example, hsv_
    if cv2.inRange(hsv_, np.uint8([[[0*2, 80, 40]]]), np.uint8([[[8*2, 255, 255]]])) or cv2.inRange(hsv_, np.uint8(
            [[[172*2, 80, 40]]]), np.uint8([[[180*2, 255, 255]]])):
        return 'Red' + ' ' + poly
    elif cv2.inRange(hsv_, np.uint8([[[18, 80, 40]]]), np.uint8([[[19*2, 255, 255]]])):
        return 'Orange' + ' ' + poly
    elif cv2.inRange(hsv_, np.uint8([[[47*2, 80, 40]]]), np.uint8([[[74*2, 255, 255]]])):
        return 'Green' + ' ' + poly
    elif cv2.inRange(hsv_, np.uint8([[[100*2, 80, 40]]]), np.uint8([[[130*2, 255, 255]]])):
        return 'Blue' + ' ' + poly
    elif cv2.inRange(hsv_, np.uint8([[[0*2, 0, 0]]]), np.uint8([[[180*2, 255, 80]]])):
        return 'Black' + ' ' + poly
    elif cv2.inRange(hsv_, np.uint8([[[137*2, 80, 40]]]), np.uint8([[[150*2, 255, 255]]])):
        return 'Violet' + ' ' + poly
    elif cv2.inRange(hsv_, np.uint8([[[20*2, 80, 40]]]), np.uint8([[[46*2, 255, 255]]])):
        return 'Brown' + ' ' + poly
    else:
        return 'Any color' + ' ' + poly


img = cv2.imread("Capture4_red.png")
img2 = img
img = cv2.GaussianBlur(img, (3, 3), 2)

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
if len(coor) != 2:
    print("Not enough Co-ordinate (Need 2)")
    pass
else:
    cv2.norm((coor[0][0], coor[0][1]), (coor[1][0], coor[1][1]))
    crop_img = dst[coor[0][1]:coor[1][1], coor[0][0]:coor[1][0]]
    edges = cv2.Canny(crop_img, 125, 255)
    # cv2.imshow("Crop_Img", np.hstack([crop_img, edges]))

    _, contours, hierarchy = cv2.findContours(edges, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    print("All contour: ", len(contours))
    for cnt in contours:
        M = cv2.moments(cnt)
        if int(M['m00']) <= 250:
            continue
        elif int(M['m00']) >= 80000:
            continue
        else:
            valid_contour.append(cnt)

    print("Valid contour: ", float(len(valid_contour)) / 2)
    for valid in valid_contour[::2]:
        M = cv2.moments(valid)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        colour = img2[cy, cx]
        # print(int(M['m00']))
        cv2.drawContours(crop_img, [valid], -1, (0, 255, 0), -1)
        # cv2.imshow("Crop_img", crop_img)
        # cv2.waitKey(0)

        approx = cv2.approxPolyDP(valid, 0.01 * cv2.arcLength(valid, True), True)
        # cv2.drawContours(crop_img, [approx], -1, (0, 0, 255), 3)

        if len(approx) == 5:
            polygon = "Pentagon"
            print(color_detection(colour, polygon))
            # cv2.drawContours(img, [valid], 0, (255, 255, 0), -1)
            cv2.putText(img2, color_detection(colour, polygon), (cx, cy), 1, 1.5, (
                np.int(cv2.bitwise_not(colour)[0][0]), np.int(cv2.bitwise_not(colour)[1][0]),
                np.int(cv2.bitwise_not(colour)[2][0])), 1, cv2.LINE_AA)
        if len(approx) == 6:
            polygon = "Hexagon"
            print(color_detection(colour, polygon))
            # cv2.drawContours(img, [cnt], 0, (255, 255, 0), -1)
            cv2.putText(img2, color_detection(colour, polygon), (cx, cy), 1, 1.5, (
                np.int(cv2.bitwise_not(colour)[0][0]), np.int(cv2.bitwise_not(colour)[1][0]),
                np.int(cv2.bitwise_not(colour)[2][0])), 1, cv2.LINE_AA)
        elif len(approx) == 3:
            polygon = "Triangle"
            print(color_detection(colour, polygon))
            # cv2.drawContours(img, [cnt], 0, (0, 255, 0), -1)
            cv2.putText(img2, color_detection(colour, polygon), (cx, cy), 1, 1.5, (
                np.int(cv2.bitwise_not(colour)[0][0]), np.int(cv2.bitwise_not(colour)[1][0]),
                np.int(cv2.bitwise_not(colour)[2][0])), 1, cv2.LINE_AA)
            # print approx
            for i in range(0, len(approx)):
                cv2.circle(img, tuple(approx[i][0]), 0, (0, 0, 255), -1)
        elif len(approx) == 4:
            polygon = "Square"
            print(color_detection(colour, polygon))
            # cv2.drawContours(img, [cnt], 0, (0, 0, 255), -1)
            cv2.putText(img2, color_detection(colour, polygon), (cx, cy), 1, 1.5, (
                np.int(cv2.bitwise_not(colour)[0][0]), np.int(cv2.bitwise_not(colour)[1][0]),
                np.int(cv2.bitwise_not(colour)[2][0])), 1, cv2.LINE_AA)
        elif len(approx) == 9:
            polygon = "Half-circle"
            print(color_detection(colour, polygon))
            # cv2.drawContours(img, [cnt], 0, (255, 255, 0), -1)
            cv2.putText(img2, color_detection(colour, polygon), (cx, cy), 1, 1.5, (
                np.int(cv2.bitwise_not(colour)[0][0]), np.int(cv2.bitwise_not(colour)[1][0]),
                np.int(cv2.bitwise_not(colour)[2][0])), 1, cv2.LINE_AA)
        elif len(approx) > 15:
            polygon = "Circle"
            print(color_detection(colour, polygon))
            # cv2.drawContours(img, [cnt], 0, (0, 255, 255), -1)
            cv2.putText(img2, color_detection(colour, polygon), (cx, cy), 1, 1.5, (
                np.int(cv2.bitwise_not(colour)[0][0]), np.int(cv2.bitwise_not(colour)[1][0]),
                np.int(cv2.bitwise_not(colour)[2][0])), 1, cv2.LINE_AA)
        else:
            print(None)

        pts.append(valid)

    for i in pts:
        re_arrange.append([])
        for j in i:
            re_arrange[pts.index(i)].append((int(j[0][0] * 0.077668 * 100), int(j[0][1] * 0.077668 * 100)))
            # TODO If error about truth value appear again use counter instead (same indent level as this comment)
    print(re_arrange)
    for i in re_arrange:
        package.append([])
        for c in range(len(i)):
            package[re_arrange.index(i)].append([])
            package[re_arrange.index(i)][c].append(i[c])
            if c == len(i) - 1:
                package[re_arrange.index(i)][c].append(i[0])
            else:
                package[re_arrange.index(i)][c].append(i[c + 1])
    # print(package)
    for i in package:
        for j in i:
            print(j)  # TODO Change print to send function
        print("Next")

    cv2.imshow("Crop_img", crop_img)
    cv2.imshow("Edge", edges)
    cv2.imshow("position and colour", img2)

    cv2.waitKey(0)
    cv2.destroyWindow("Crop_Img")
    cv2.destroyWindow("Edge")
    exit()
