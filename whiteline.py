#-*- coding: utf-8 -*-

import cv2
import numpy as np
from pylsd.lsd import lsd
import time
import rospy
from std_msgs.msg import Bool

IMAGE_PATH = "tsukuba_20_test.mp4"
CAMERA_PATH = 0

# トリミングrate
trim_rate = 0
# リサイズrate
resize_rate = 0.5

# 二値化の閾値
thresh_min = 230
thresh_max = 255
# 適応値の計算時に使う領域のサイズ（奇数）
adaptive_block_size = 61
# 適応型二値化での閾値からの加減値
distribution = -30

# 絞り込みをするオブジェクトサイズ
lower_size = 3000
upper_size= 5000

# モルフォロジー変換でのブロックサイズ
morphology_size = (30,30)

# 検出した直線を長さで絞り込む閾値
line_size = 15000

cnt = 0

cap = cv2.VideoCapture(CAMERA_PATH)
# cap = cv2.VideoCapture(IMAGE_PATH)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

# 画像の読み込み
img = ret, img = cap.read()

# 画像のピクセル数
height, width = img.shape[:2]
print("---Inpit size---")
print("width: " + str(width))
print("height: " + str(height))

# トリミング時のサイズ計算
trim_height = int(height * trim_rate)
print("---after size---")
print("width: "+str(width))
print("height: "+str(height-trim_height))

start = time.time()
stop = time.time()

pub = rospy.Publisher('detect_whiteline', Bool, queue_size=10)
rospy.init_node('whiteline')


def get_gray(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return gray

def remove_objects(img, lower_size=None, upper_size=None):
    # find all objects
    nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(img)

    sizes = stats[1:, -1]
    _img = np.zeros((labels.shape))

    # process all objects, label=0 is background, objects are started from 1
    for i in range(1, nlabels):

        # remove small objects
        if (lower_size is not None) and (upper_size is not None):
            if lower_size < sizes[i - 1] and sizes[i - 1] < upper_size:
                _img[labels == i] = 255

        elif (lower_size is not None) and (upper_size is None):
            if lower_size < sizes[i - 1]:
                _img[labels == i] = 255

        elif (lower_size is None) and (upper_size is not None):
            if sizes[i - 1] < upper_size:
                _img[labels == i] = 255

    return _img

def detect():
    global start, stop,cnt
    while not rospy.is_shutdown():
        ret, src = cap.read()
        # cv2.imshow("src", img)

        # 画像のトリミング
        trim_img = src[trim_height:height, 0:width]
        # cv2.imshow("trim", trim_img)

        resize_img = cv2.resize(trim_img, (0, 0), fx=resize_rate, fy=resize_rate)
        # cv2.imshow("resize", resize_img)

        # グレースケール変換
        gray = get_gray(resize_img)
        # cv2.imshow("gray", gray)

        # ノイズ除去
        gauss = cv2.GaussianBlur(gray,(3,3),3)
        # cv2.imshow("gauss", gauss)

        # 二値化
        # ret, thresh = cv2.threshold(gauss, thresh_min, thresh_max, cv2.THRESH_BINARY)
        thresh = cv2.adaptiveThreshold(gauss, thresh_max, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,adaptive_block_size , distribution)
        # cv2.imshow("thresh", thresh)

        pixel_sum = np.sum(thresh) #全ピクセルの輝度の合計をpixel_sumに代入
        if pixel_sum > 0:
            binary_clean = remove_objects(thresh, lower_size, upper_size)
            # cv2.imshow("img_greenarea_clean", binary_clean)

            kernel = np.ones((morphology_size),np.uint8)
            connect = cv2.morphologyEx(binary_clean,cv2.MORPH_CLOSE, kernel)
            cv2.imshow("connect", connect)

            lines = lsd(connect)
            # print(lines)
            # line1 = resize_img.copy()
            line2 = resize_img.copy()
            count = 0
            x_ave = 0
            y_ave = 0
            for line in lines:
                x1, y1, x2, y2 = map(int,line[:4])
                # line1 = cv2.line(line1, (x1,y1), (x2,y2), (0,0,255), 3)
                if (x2-x1)**2 + (y2-y1)**2 > line_size:
                    if count > 0:
                        x_ave += (x2-x1)**2 - buf_x
                        y_ave += (y2-y1)**2 - buf_y
                    else:
                        x_ave = 0
                        y_ave = 0
                    buf_x = (x2-x1)**2
                    buf_y = (y2-y1)**2
                    count += 1
                    # 赤線を引く
                    line2 = cv2.line(line2, (x1,y1), (x2,y2), (0,0,255), 3)
            if x_ave > 0 and y_ave > 0:
                x_ave = np.sqrt(x_ave/count)
                y_ave = np.sqrt(y_ave/count)
                if x_ave * y_ave < 1000 and y_ave < 10:
                    if cnt == 0:
                        start = time.time()
                        # print(start)
                        cnt = 1
            if (time.time() - start) > 1 and cnt == 1:
                pub.publish(True)
                print("This is white line")
                cnt = 0
            else:
                pub.publish(False)
            cv2.imshow("line", line2)
            
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    try:
        detect()
    except rospy.ROSInterruptException:
        pass