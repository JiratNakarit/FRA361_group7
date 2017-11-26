import numpy as np
import cv2
import time
import copy
import ReadSerial


class XYPlotter:
    def __init__(self):
        self.ser1 = ReadSerial.SerialPIC('COM19', 115200)
        self.ser2 = ReadSerial.SerialPIC('COM18', 115200)
        self.TRACKING_CON = 0
        self.TRACKING_APP = 1
        self.TRACKING_SHOW = 1
        self.TRACKING_HIDE = 2
        self.TRACKING_POINT = 2
        self.CROP_NO_BLUR = 0
        self.FILL_NO = 0
        self.FILL_COLOR = 1
        self.CROP_BLUR = 1
        self.dat = np.load("Variable.npz")
        self.crop_point = np.load("Co_or.npz")
        self.img = np.ndarray([])
        self.copy_img = np.ndarray([])
        self.valid_contour = []
        self.cropped_image = np.ndarray([])
        self.pts = []
        self.co_or_for_cari = []
        self.re_arrange = []
        self.package = []
        self.gap_y = abs(self.crop_point['co_or'][0][1] - self.crop_point['co_or'][2][1])
        self.gap_x = abs(self.crop_point['co_or'][0][0] - self.crop_point['co_or'][2][0])
        self.counter = 0
        self.midpoint = []
        self.str_to_send = []

    def color_detection(self, color, poly):
        example = np.uint8([[color]])
        hsv_ = cv2.cvtColor(example, cv2.COLOR_BGR2HSV)
        # print example, hsv_ # TODO Show color value in BGR and HSV
        if cv2.inRange(hsv_, np.uint8([[[0, 80, 40]]]), np.uint8([[[8, 255, 255]]])) or cv2.inRange(hsv_, np.uint8(
                [[[172, 80, 40]]]), np.uint8([[[180, 255, 255]]])):
            return 'Red' + ' ' + poly
        elif cv2.inRange(hsv_, np.uint8([[[9, 80, 40]]]), np.uint8([[[19, 255, 255]]])):
            return 'Orange' + ' ' + poly
        elif cv2.inRange(hsv_, np.uint8([[[47, 80, 40]]]), np.uint8([[[74, 255, 255]]])):
            return 'Green' + ' ' + poly
        elif cv2.inRange(hsv_, np.uint8([[[100, 80, 40]]]), np.uint8([[[130, 255, 255]]])):
            return 'Blue' + ' ' + poly
        elif cv2.inRange(hsv_, np.uint8([[[0, 0, 0]]]), np.uint8([[[180, 255, 80]]])):
            return 'Black' + ' ' + poly
        elif cv2.inRange(hsv_, np.uint8([[[137, 80, 40]]]), np.uint8([[[150, 255, 255]]])):
            return 'Violet' + ' ' + poly
        elif cv2.inRange(hsv_, np.uint8([[[20, 80, 40]]]), np.uint8([[[46, 255, 255]]])):
            return 'Brown' + ' ' + poly
        else:
            return 'Any color' + ' ' + poly

    def get_co_or(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.co_or_for_cari.append((x, y))
        if len(self.co_or_for_cari) == 4:
            cv2.destroyWindow("Camera")

    def caribrate_cam(self):
        cam = cv2.VideoCapture(0)

        cv2.namedWindow("Camera")
        cv2.setMouseCallback('Camera', self.get_co_or)

        while True:
            ret, frame = cam.read()
            frame = cv2.imread("TESTCap0.png")
            cv2.imshow("Camera", frame)
            k = cv2.waitKey(1)
            if k == 27:
                cv2.destroyAllWindows()
                cam.release()
                break
            if len(self.co_or_for_cari) == 4:
                cv2.destroyWindow("Camera")
                cam.release()
                break
        if len(self.co_or_for_cari) != 4:
            print("Not enough Co-ordinate (Need 4)")
        else:
            # print(self.co_or_for_cari)  # TODO For debug
            np.savez("Co_or", co_or=self.co_or_for_cari)
            exit(0)

    def capture(self):
        cam = cv2.VideoCapture(0)
        while True:
            self.img = cv2.imread("TESTCap0.png")
            ret, frame = cam.read()
            # cv2.imshow("Camera", frame)
            cv2.imshow("Camera", self.img)
            k = cv2.waitKey(1)
            if k == ord("c"):
                self.img = frame
                self.img = cv2.imread("TESTCap0.png")
                # self.copy_img = copy.copy(frame)
                self.copy_img = copy.copy(self.img)
                # cv2.imwrite("Capture.jpg", frame)  # TODO In case we need to save this image
                cv2.destroyAllWindows()
                cam.release()
                break
            elif k == 27:
                cv2.destroyAllWindows()
                cam.release()
                exit()

    def crop_cari_image(self, blur=0):
        h, w = self.img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.dat['mtx'], self.dat['dist'], (w, h), 1, (w, h))
        mapx, mapy = cv2.initUndistortRectifyMap(self.dat['mtx'], self.dat['dist'], None, newcameramtx, (w, h), 5)
        # dst = cv2.remap(self.img, mapx, mapy, cv2.INTER_LINEAR) # TODO For camera calibration

        # print("Co-ordinate", self.crop_point['co_or'])  # TODO For Debug

        if len(self.crop_point['co_or']) != 4:
            print("Not enough Co-ordinate (Need 2)", len(self.crop_point['co_or']))
        else:
            self.cropped_image = copy.copy(self.img[self.crop_point['co_or'][2][1]:self.crop_point['co_or'][3][1],
                                           self.crop_point['co_or'][2][0]:self.crop_point['co_or'][3][0]])

            if blur == 0:
                pass
            elif blur == 1:
                self.cropped_image = cv2.GaussianBlur(self.cropped_image, (9, 9), 1)  # TODO use 2 for moon shape

            # self.unsharpmask = cv2.subtract(self.cropped_image,cv2.GaussianBlur(self.cropped_image, (3, 3), 3))
            # self.cropped_image = cv2.add(self.cropped_image, self.unsharpmask*1)

            cv2.imshow("image", self.cropped_image)
            cv2.waitKey(0)
            cv2.destroyWindow("image")

    def detect_basic_shape(self, track_method=0, show_colorpos=0):
        # print("here!") # TODO For Debug
        edges = cv2.Canny(self.cropped_image, 125, 255)

        _, contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print("All contour: ", len(contours))
        for cnt in contours:
            moment = cv2.moments(cnt)
            if track_method == 0 or track_method == 1:
                if int(moment['m00']) <= 50:
                    continue
                elif int(moment['m00']) >= 80000:
                    continue
                else:
                    self.valid_contour.append(cnt)
            elif track_method == 2:
                if int(moment['m00']) <= 0:
                    continue
                elif int(moment['m00']) >= 15:
                    continue
                else:
                    self.valid_contour.append(cnt)

        print("Valid contour: ", float(len(self.valid_contour)))

        for valid in self.valid_contour[::1]:
            moment = cv2.moments(valid)
            cx = int(moment['m10'] / moment['m00'])
            cy = int(moment['m01'] / moment['m00'])
            self.midpoint = [cx + self.gap_x + self.crop_point['co_or'][0][0],
                             cy + self.gap_y + self.crop_point['co_or'][0][1]]
            colour = self.copy_img[
                cy + self.gap_y + self.crop_point['co_or'][0][1], cx + self.gap_x + self.crop_point['co_or'][0][0]]
            # print('Color:', colour, cv2.cvtColor(np.uint8([[colour]]), cv2.COLOR_BGR2HSV), "At:" ,cx * 0.077668 ,', ',cy * 0.077668 )
            # print(int(moment['m00']))
            cv2.drawContours(self.cropped_image, [valid], 0, (0, 255, 0), -1)
            # cv2.imshow("Crop_img", self.cropped_image)
            # cv2.waitKey(0)
            approx = cv2.approxPolyDP(valid, 0.01 * cv2.arcLength(valid, True), True)
            '''*********************************************************************************'''
            if show_colorpos == 0:
                pass
            elif show_colorpos == 1:
                if len(approx) == 5:
                    polygon = "Pentagon"
                    # print(self.color_detection(colour, polygon))
                    cv2.putText(self.copy_img, self.color_detection(colour, polygon), (
                        cx + self.gap_x + self.crop_point['co_or'][0][0],
                        cy + self.gap_y + self.crop_point['co_or'][0][1]),
                                1, 1.5, (
                                    np.int(cv2.bitwise_not(colour)[0][0]), np.int(cv2.bitwise_not(colour)[1][0]),
                                    np.int(cv2.bitwise_not(colour)[2][0])), 1, cv2.LINE_AA)
                elif len(approx) == 6:
                    polygon = "Hexagon"
                    # print(self.color_detection(colour, polygon))
                    cv2.putText(self.copy_img, self.color_detection(colour, polygon), (
                        cx + self.gap_x + self.crop_point['co_or'][0][0],
                        cy + self.gap_y + self.crop_point['co_or'][0][1]),
                                1, 1.5, (
                                    np.int(cv2.bitwise_not(colour)[0][0]), np.int(cv2.bitwise_not(colour)[1][0]),
                                    np.int(cv2.bitwise_not(colour)[2][0])), 1, cv2.LINE_AA)
                elif len(approx) == 3:
                    polygon = "Triangle"
                    print(self.color_detection(colour, polygon))
                    cv2.putText(self.copy_img, self.color_detection(colour, polygon), (
                        cx + self.gap_x + self.crop_point['co_or'][0][0],
                        cy + self.gap_y + self.crop_point['co_or'][0][1]),
                                1, 1.5, (
                                    np.int(cv2.bitwise_not(colour)[0][0]), np.int(cv2.bitwise_not(colour)[1][0]),
                                    np.int(cv2.bitwise_not(colour)[2][0])), 1, cv2.LINE_AA)
                elif len(approx) == 4:
                    polygon = "Square"
                    # print(self.color_detection(colour, polygon))
                    cv2.putText(self.copy_img, self.color_detection(colour, polygon), (
                        cx + self.gap_x + self.crop_point['co_or'][0][0],
                        cy + self.gap_y + self.crop_point['co_or'][0][1]),
                                1, 1.5, (
                                    np.int(cv2.bitwise_not(colour)[0][0]), np.int(cv2.bitwise_not(colour)[1][0]),
                                    np.int(cv2.bitwise_not(colour)[2][0])), 1, cv2.LINE_AA)
                elif len(approx) == 9:
                    polygon = "Half-circle"
                    # print(self.color_detection(colour, polygon))
                    cv2.putText(self.copy_img, self.color_detection(colour, polygon), (
                        cx + self.gap_x + self.crop_point['co_or'][0][0],
                        cy + self.gap_y + self.crop_point['co_or'][0][1]),
                                1, 1.5, (
                                    np.int(cv2.bitwise_not(colour)[0][0]), np.int(cv2.bitwise_not(colour)[1][0]),
                                    np.int(cv2.bitwise_not(colour)[2][0])), 1, cv2.LINE_AA)
                elif len(approx) > 15:
                    polygon = "Circle"
                    # print(self.color_detection(colour, polygon))
                    cv2.putText(self.copy_img, self.color_detection(colour, polygon), (
                        cx + self.gap_x + self.crop_point['co_or'][0][0],
                        cy + self.gap_y + self.crop_point['co_or'][0][1]),
                                1, 1.5, (
                                    np.int(cv2.bitwise_not(colour)[0][0]), np.int(cv2.bitwise_not(colour)[1][0]),
                                    np.int(cv2.bitwise_not(colour)[2][0])), 1, cv2.LINE_AA)
                else:
                    # print(colour)
                    polygon = "Any Polygon"
                    cv2.putText(self.copy_img, "Any Polygon HERE!", (
                        cx + self.gap_x + self.crop_point['co_or'][0][0],
                        cy + self.gap_y + self.crop_point['co_or'][0][1]),
                                1, 1.5, (
                                    np.int(cv2.bitwise_not(colour)[0][0]), np.int(cv2.bitwise_not(colour)[1][0]),
                                    np.int(cv2.bitwise_not(colour)[2][0])), 1, cv2.LINE_AA)
                print(self.color_detection(colour, polygon), "At:", (cx + self.gap_x) * 0.077668, ', ',
                      (cy + self.gap_y) * 0.077668)

            '''*********************************************************************************'''

            cv2.drawContours(self.cropped_image, [approx], -1, (0, 0, 255), 3)
            if track_method == 0:
                print("Use Continue Tracking")
                self.pts.append(valid)
            elif track_method == 1:
                print("Use Approx Tracking")
                self.pts.append(approx)
            elif track_method == 2:
                print("Use Point Tracking")
                self.pts.append([[self.midpoint]])
        cv2.imshow("Crop_img", self.cropped_image)

        cv2.imshow("Edge", edges)

        cv2.imshow("Main PIC", self.copy_img)

        cv2.waitKey(0)
        # cv2.destroyAllWindows()
        cv2.destroyWindow("Crop_Img")
        cv2.destroyWindow("Edge")

    def prepare_data(self, prepare_method=0):
        self.counter = 0
        if prepare_method == 0 or prepare_method == 1:
            for i in self.pts:
                self.re_arrange.append([])
                for j in i:
                    self.re_arrange[self.counter].append((int((j[0][0]) * 0.077668 * 100),
                                                          int((j[0][1]) * 0.077668 * 100)))
                self.counter += 1
            # TODO If error about truth value appear again use counter instead (same indent level as this comment)
            for i in self.re_arrange:
                self.package.append([])
                for c in range(len(i)):
                    self.package[self.re_arrange.index(i)].append([])
                    self.package[self.re_arrange.index(i)][c].append(i[c])
                    if c == len(i) - 1:
                        self.package[self.re_arrange.index(i)][c].append(i[0])
        elif prepare_method == 2:
            for i in self.pts:
                for j in i:
                    self.re_arrange.append(
                        (int((j[0][0] ) * 0.077668 * 100), int((j[0][1] ) * 0.077668 * 100)))
                self.counter += 1
                # TODO If error about truth value appear again use counter instead (same indent level as this comment)
            self.package.append([])
            for i in [self.re_arrange]:

                for c in range(len(i)):
                    self.package[0].append([])
                    self.package[0][c].append(i[c])
                    if c == len(i) - 1:
                        self.package[0].append([i[0]])
        # print(self.package)

    def send_serial(self, pack):
        rcv = []

        def receive(alphabet, ser):
            state = 0
            while True:
                if ser == 1:
                    rcv_data = self.ser1.recieveRawPackage()
                else:
                    rcv_data = self.ser2.recieveRawPackage()
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
            self.ser1.serial.flush()
            self.ser2.serial.flush()
            if 'h' in rcv:
                print('Home')
            elif 'e' in rcv:
                print("End")

        def pack_n_send(stri, ser, re):
            global rcv
            for i in stri:
                if ser == 1:
                    self.ser1.Package.append(ord(i))
                elif ser == 2:
                    self.ser2.Package.append(ord(i))
            if ser == 1:
                self.ser1.SEND(self.ser1.Package)
                self.ser1.Package = []
                if re:
                    receive(1, 1)
                rcv = []
                self.ser1.serial.flush()
            elif ser == 2:
                self.ser2.SEND(self.ser2.Package)
                self.ser2.Package = []
                if re:
                    receive(1, 1)
                rcv = []
                self.ser2.serial.flush()

        pack_n_send('[h]', 1, True)

        for ct, shape in enumerate(pack, 0):
            for ct_, point in enumerate(shape, 0):
                for chr_ in str(point):
                    if chr_ != ' ':
                        self.ser1.Package.append(ord(chr_))
                    else:
                        continue
                self.ser1.SEND(self.ser1.Package)

                print("SEND!", self.ser1.Package)
                self.ser1.Package = []
                rcv = []
                receive(1, 1)
                if ct_ == 0:
                    pack_n_send('[d]', 2, False)
                    time.sleep(7)
                time.sleep(1)
                print(ct_)
            rcv = []


if __name__ == '__main__':
    test = XYPlotter()
    # test.caribrate_cam()
    test.capture()
    test.crop_cari_image(test.CROP_NO_BLUR)
    test.detect_basic_shape(test.TRACKING_POINT, test.TRACKING_HIDE)
    test.prepare_data(test.TRACKING_POINT)
    print(test.package)
    test.send_serial(test.package)
else:
    print("Imported!")
