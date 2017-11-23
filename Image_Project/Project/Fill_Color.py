import numpy as np
import cv2
import copy

valid_contour = []
pts = []
re_arrange = []
package = []

img = cv2.imread("NewCap2.png", 0)

diagmask = cv2.imread("Diagonal.png", 0)

crop_point = np.load("Co_or.npz")

cropped_image = copy.copy(
    img[crop_point['co_or'][2][1]:crop_point['co_or'][3][1], crop_point['co_or'][2][0]:crop_point['co_or'][3][0]])

diagmask = diagmask[crop_point['co_or'][2][1]:crop_point['co_or'][3][1], crop_point['co_or'][2][0]:crop_point['co_or'][3][0]]

gap_y = abs(crop_point['co_or'][0][1] - crop_point['co_or'][2][1])
gap_x = abs(crop_point['co_or'][0][0] - crop_point['co_or'][2][0])

h, w = cropped_image.shape[:2]

black_img = np.zeros((h, w), np.uint8)

black_img2 = copy.copy(black_img)

edge_img = copy.copy(cv2.Canny(cropped_image, 100, 255))

_, contours, hierarchy = cv2.findContours(edge_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

for cnt in contours:
    moment = cv2.moments(cnt)
    if 100 < moment['m00'] < 80000:
        valid_contour.append(cnt)

for valid in valid_contour:
    moment = cv2.moments(cnt)
    cv2.drawContours(edge_img, [cnt], -1, 255, -1)
    approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
    cv2.drawContours(black_img, [valid], 0, 255, -1)

    if 1:
        pts.append(valid)
    else:
        pts.append(approx)

filled = cv2.bitwise_and(diagmask, black_img)

_fill, contoursfill, hierarchyfill = cv2.findContours(filled, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

for cntfill in contoursfill:
    pts.append(cntfill)

for i in pts:
    cv2.drawContours(black_img2, [i], 0, 255, 1)

# print(len(contours))
# print(len(valid_contour))
# print(len(contoursfill))
# print(len(pts))

counter = 0

for i in pts:
    re_arrange.append([])
    for j in i:
        re_arrange[counter].append((int((j[0][0] + gap_x) * 0.077668 * 100),
                                    int((j[0][1] + gap_y) * 0.077668 * 100)))
    counter += 1

for i in re_arrange:
    package.append([])
    for c in range(len(i)):
        package[re_arrange.index(i)].append([])
        package[re_arrange.index(i)][c].append(i[c])
        if c == len(i) - 1:
            package[re_arrange.index(i)][c].append(i[0])
        else:
            package[re_arrange.index(i)][c].append(i[c + 1])
            
for i in package:
    for j in i:
        print(j)  # TODO Change print to send function
    if package.index(i) == len(package) - 1:
        print("End")
    else:
        print("Next")

cv2.imshow("Edge", edge_img)
cv2.imshow("Diag", diagmask)
cv2.imshow("Mask", black_img)
cv2.imshow("Filled", filled)
cv2.imshow("Finished", black_img2)
cv2.waitKey(0)
