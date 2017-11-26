import numpy as np
import cv2
import math
import ReadSerial
import time

# str_to_send = []
ser1 = ReadSerial.SerialPIC(port="COM19", brate=115200)


ser2 = ReadSerial.SerialPIC(port="COM18", brate=115200)
# rcv = []
# state = 0


def get_coor(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        coor.append((x, y))
    if len(coor) == 4:
        cv2.destroyWindow("image")


def find_angle_distance(point):
    """Return the angle between B-A and the positive x-axis.
    Values go from 0 to pi in the upper half-plane, and from
    0 to -pi in the lower half-plane.
    """
    ax, ay = point[0]
    bx, by = point[1]
    dist = int((math.hypot(bx - ax, by - ay)) * 10)
    ang = math.degrees(math.atan2(by - ay, bx - ax))
    if ang < 0:
        ang += 360
    ang = int(ang * 10)
    return dist, ang
    # return math.degrees(math.atan2(by - ay, bx - ax)), cv2.norm(tuple(point[0]),tuple(point[1]),cv2.NORM_L2)


def send_serial(pack):
    rcv = []
    global ser1
    global ser2

    def receive(alphabet, ser):
        state = 0
        while True:
            if ser == 1:
                rcv_data = ser1.recieveRawPackage()
            else:
                pass
                # rcv_data = ser2.recieveRawPackage()
            for uni in rcv_data:
                if chr(uni) == "[":
                    state = 1
                    continue
                elif chr(uni) == "]" and state == 1:
                    state = 0
                    break
                if state == 1:
                    # print(chr(o), end='')
                    rcv.append(chr(uni))
            if len(rcv) == alphabet:
                break
        print(rcv)
        ser1.serial.flush()
        if 'h' in rcv:
            print('Home')
        elif 'e' in rcv:
            print("End")

    for i in '[h]':
        ser1.Package.append(ord(i))
    ser1.SEND(ser1.Package)
    ser1.Package = []
    receive(1, 1)
    rcv = []
    ser1.serial.flush()

    for ct, shape in enumerate(pack, 0):
        for ct_, point in enumerate(shape, 0):
            for chr_ in str(point):
                if chr_ != ' ':
                    ser1.Package.append(ord(chr_))
                else:
                    continue
            ser1.SEND(ser1.Package)

            print("SEND!", ser1.Package)
            ser1.Package = []
            rcv = []
            receive(1, 1)
            if ct_ == 0:
                for p in '[d]':
                    ser1.Package.append(ord(p))
                ser2.SEND(ser1.Package)
                ser2.Package = []
                time.sleep(6.5)
                rcv = []
                ser1.serial.flush()
            time.sleep(1)
            print(ct_)
        rcv = []


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
        cv2.drawContours(crop_img, [valid], -1, (0, 0, 255), -1)
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
            re_arrange.append(
                (100 + int((j[0][0] + gap_x) * 0.077668 * 100), abs(3000 - int((j[0][1] + gap_y) * 0.077668 * 100))))
        counter += 1
        # TODO If error about truth value appear again use counter instead (same indent level as this comment)
    # print(re_arrange)
    package.append([])
    for i in [re_arrange]:

        for c in range(len(i)):
            package[0].append([])
            package[0][c].append(i[c])
            if c == len(i) - 1:
                package[0].append([i[0]])
    print(package)
    send_serial(package)
    # for out, i in enumerate(package, 0):
    #     # str_to_send.append([])
    #     for inner, j in enumerate(i, 0):
    #         time.sleep(0.5)
    #         print(j)  # TODO Change print to send function
    #         for ord_ in str(j):  # TODO Pack ord package
    #             if ord_ == ' ':
    #                 pass
    #             else:
    #                 pass
    #                 ser1.Package.append(ord(ord_))
    #         ser1.SEND(ser.Package)
    #         ser1.Package = []
    #         j = ser1.recieveRawPackage()
    #         for o in j:
    #             if chr(o) == "[":
    #                 state = 1
    #                 continue
    #             elif chr(o) == "]":
    #                 state = 0
    #             if state == 1:
    #                 # print(chr(o), end='')
    #                 rcv.append(chr(o))
    #         # print(int(math.degrees(math.atan2(j[0][1] - j[1][1], j[0][0] - j[1][0])) * 10))
    #         if inner == 0:
    #             pass
    #         # str_to_send[out].append(str(find_angle_distance(((0, 0), j[0]))))
    #         # str_to_send[out].append(str(find_angle_distance(j)))
    #         if inner == len(i) - 1 and out != len(package) - 1:
    #             print('Next')
    #     if out == len(package) - 1:
    #         print("End")
    # print(str_to_send)
    # print(rcv)
    cv2.imshow("Edge and Crop_img", crop_img)
    cv2.imshow("Edge", edges)

    cv2.waitKey(0)
    cv2.destroyWindow("Crop_Img")
    cv2.destroyWindow("Edge")
    exit()
