import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp


class HandDetector:
    """
    使用mediapipe库查找手。导出地标像素格式。添加了额外的功能。
    如查找方式，许多手指向上或两个手指之间的距离。而且提供找到的手的边界框信息。
    """
    def __init__(self, mode=False, maxHands=2, detectionCon=0.5, minTrackCon = 0.5):
        """
        :param mode: 在静态模式下，对每个图像进行检测
        :param maxHands: 要检测的最大手数
        :param detectionCon: 最小检测置信度
        :param minTrackCon: 最小跟踪置信度
        """
        self.mode = mode
        self.maxHands = maxHands
        self.modelComplex = False
        self.detectionCon = detectionCon
        self.minTrackCon = minTrackCon

		# 初始化手部识别模型
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, self.modelComplex,
                                        self.detectionCon, self.minTrackCon)
        self.mpDraw = mp.solutions.drawing_utils	# 初始化绘图器
        self.tipIds = [4, 8, 12, 16, 20]			# 指尖列表
        self.fingers = []
        self.lmList = []


    def findHands(self, img, draw=False):
        """
        从图像(BRG)中找到手部。
        :param img: 用于查找手的图像。
        :param draw: 在图像上绘制输出的标志。
        :return: 带或不带图形的图像
        """
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # 将传入的图像由BGR模式转标准的Opencv模式——RGB模式，
        self.results = self.hands.process(imgRGB)
        # print("results：--------------------")
        # print(self.results)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms,
                                               self.mpHands.HAND_CONNECTIONS)
        return img

    def findPosition(self, img, handNo=0, draw=False):
        """
        查找单手的地标并将其放入列表中像素格式。还可以返回手部周围的边界框。
        :param img: 要查找的主图像
        :param handNo: 如果检测到多只手，则为手部id
        :param draw: 在图像上绘制输出的标志。(默认绘制矩形框)
        :return: 像素格式的手部关节位置列表；手部边界框
        """

        xList = []
        yList = []
        bbox = []
        bboxInfo =[]
        self.lmList = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                h, w, c = img.shape
                px, py = int(lm.x * w), int(lm.y * h)
                xList.append(px)
                yList.append(py)
                self.lmList.append([px, py])
                if draw:
                    cv2.circle(img, (px, py), 5, (255, 0, 255), cv2.FILLED)
            xmin, xmax = min(xList), max(xList)
            ymin, ymax = min(yList), max(yList)
            boxW, boxH = xmax - xmin, ymax - ymin
            bbox = xmin, ymin, boxW, boxH
            cx, cy = bbox[0] + (bbox[2] // 2), \
                     bbox[1] + (bbox[3] // 2)
            bboxInfo = {"id": id, "bbox": bbox,"center": (cx, cy)}

            if draw:
                cv2.rectangle(img, (bbox[0] - 20, bbox[1] - 20),
                              (bbox[0] + bbox[2] + 20, bbox[1] + bbox[3] + 20),
                              (0, 255, 0), 2)

        return self.lmList, bboxInfo

    def fingersUp(self):
        """
        查找列表中打开并返回的手指数。会分别考虑左手和右手
        ：return：竖起手指的列表
        """
        if self.results.multi_hand_landmarks:
            myHandType = self.handType()
            fingers = []
            # Thumb
            if myHandType == "Right":
                if self.lmList[self.tipIds[0]][0] > self.lmList[self.tipIds[0] - 1][0]:
                    fingers.append(1)
                else:
                    fingers.append(0)
            else:
                if self.lmList[self.tipIds[0]][0] < self.lmList[self.tipIds[0] - 1][0]:
                    fingers.append(1)
                else:
                    fingers.append(0)
            # 4 Fingers
            for id in range(1, 5):
                if self.lmList[self.tipIds[id]][1] < self.lmList[self.tipIds[id] - 2][1]:
                    fingers.append(1)
                else:
                    fingers.append(0)
        return fingers

    def handType(self):
        """
        通过食指(5)和小指(17)指根的位置,检查传入的手部是左还是右
        小拇指在左边，是右手
        食指在左边，是左手
        ：return: "Right" 或 "Left"
        """
        if self.results.multi_hand_landmarks:
            if self.lmList[17][0] < self.lmList[5][0]:
                # print('right')
                return "Right"
            else:
                # print('left')
                return "Left"

    def Gesture_recognition_img(self, img, draw=True):
        img = self.findHands(img, draw=draw)
        lmList, bbox = self.findPosition(img, draw=draw)

        if lmList:
            x_1, y_1 = bbox["bbox"][0], bbox["bbox"][1]
            x1, x2, x3, x4, x5 = self.fingersUp()
            
            print(f'{x1} {x2} {x3} {x4} {x5}')

            # 检测大拇指是否竖起有点问题
            if (x2 == 1 and x3 == 1) and (x4 == 0 and x1 == 0):
                cv2.putText(img, "2_TWO", (x_1, y_1), cv2.FONT_HERSHEY_PLAIN, 3,
                            (0, 0, 255), 3)
            elif (x3 == 1 and x4 == 1 and x5 == 1) and (x2 == 0):  # 比3的手势的时候，大拇指是否竖起的逻辑有点问题，因此没有判断x1
                cv2.putText(img, "3_THREE", (x_1, y_1), cv2.FONT_HERSHEY_PLAIN, 3,
                            (0, 0, 255), 3)
            elif (x2 == 1 and x3 == 1 and x4 == 1 and x5 == 1) and (x1 == 0):
                cv2.putText(img, "4_FOUR", (x_1, y_1), cv2.FONT_HERSHEY_PLAIN, 3,
                            (0, 0, 255), 3)
            elif x1 == 1 and x2 == 1 and x3 == 1 and x4 == 1 and x5 == 1:
                cv2.putText(img, "5_FIVE", (x_1, y_1), cv2.FONT_HERSHEY_PLAIN, 3,
                            (0, 0, 255), 3)
            elif x2 == 1 and (x1 == 0, x3 == 0, x4 == 0, x5 == 0):
                cv2.putText(img, "1_ONE", (x_1, y_1), cv2.FONT_HERSHEY_PLAIN, 3,
                            (0, 0, 255), 3)
            elif (x1 and x5) and (x2 == 0 and x3 == 0 and x4 == 0):
                cv2.putText(img, "SIX!", (x_1, y_1), cv2.FONT_HERSHEY_PLAIN, 3,
                            (0, 0, 255), 3)
            # else:
            #     return None
        return img



class GestureRecognitionNode(Node):

    def __init__(self):
        super().__init__('gesture_recognition_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Image, '/gesture_image', 10)
        self.bridge = CvBridge()
        self.detector = HandDetector()


        self.get_logger().info('Gesture recognition node has been started.')

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        recognized_image = self.detector.Gesture_recognition_img(cv_image)


        gesture_image_msg = self.bridge.cv2_to_imgmsg(recognized_image, encoding='bgr8')
        self.publisher.publish(gesture_image_msg)
            



def main(args=None):
    rclpy.init(args=args)
    node = GestureRecognitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
