
# encoding=utf-8

import sys
import almath  # almath.TO_RAD, 角度转弧度
import argparse  # 参数解析
import threading  # 多线程类
from naoqi import ALProxy
from naoqi import ALModule
from naoqi import ALBroker
import vision_definitions
import time

isBegin = 0
landmark_num = 0
hasFallen = 0

class avoidance(threading.Thread):

    def change_post(self):		#在行动之前改变姿态提高稳定性
        names = list()
        times = list()
        keys = list()

        names.append("HeadPitch")
        times.append([1.52])
        keys.append([[0.322098, [3, -0.506667, 0], [3, 0, 0]]])

        names.append("HeadYaw")
        times.append([1.52])
        keys.append([[0.00916195, [3, -0.506667, 0], [3, 0, 0]]])

        names.append("LAnklePitch")
        times.append([1.52])
        keys.append([[-0.50311, [3, -0.506667, 0], [3, 0, 0]]])

        names.append("LAnkleRoll")
        times.append([1.52])
        keys.append([[0.00157595, [3, -0.506667, 0], [3, 0, 0]]])

        names.append("LElbowRoll")
        times.append([1.52])
        keys.append([[-1.49723, [3, -0.506667, 0], [3, 0, 0]]])

        names.append("LElbowYaw")
        times.append([1.52])
        keys.append([[-1.54776, [3, -0.506667, 0], [3, 0, 0]]])

        names.append("LHand")
        times.append([1.52])
        keys.append([[0.0108, [3, -0.506667, 0], [3, 0, 0]]])

        names.append("LHipPitch")
        times.append([1.52])
        keys.append([[-0.337522, [3, -0.506667, 0], [3, 0, 0]]])

        names.append("LHipRoll")
        times.append([1.52])
        keys.append([[-0.00924587, [3, -0.506667, 0], [3, 0, 0]]])

        names.append("LHipYawPitch")
        times.append([1.52])
        keys.append([[0.00464392, [3, -0.506667, 0], [3, 0, 0]]])

        names.append("LKneePitch")
        times.append([1.52])
        keys.append([[0.851412, [3, -0.506667, 0], [3, 0, 0]]])

        names.append("LShoulderPitch")
        times.append([1.52])
        keys.append([[1.94822, [3, -0.506667, 0], [3, 0, 0]]])

        names.append("LShoulderRoll")
        times.append([1.52])
        keys.append([[-0.162562, [3, -0.506667, 0], [3, 0, 0]]])

        names.append("LWristYaw")
        times.append([1.52])
        keys.append([[0.131966, [3, -0.506667, 0], [3, 0, 0]]])

        names.append("RAnklePitch")
        times.append([1.52])
        keys.append([[-0.50311, [3, -0.506667, 0], [3, 0, 0]]])

        names.append("RAnkleRoll")
        times.append([1.52])
        keys.append([[-0.00157595, [3, -0.506667, 0], [3, 0, 0]]])

        names.append("RElbowRoll")
        times.append([1.52])
        keys.append([[1.49723, [3, -0.506667, 0], [3, 0, 0]]])

        names.append("RElbowYaw")
        times.append([1.52])
        keys.append([[1.54776, [3, -0.506667, 0], [3, 0, 0]]])

        names.append("RHand")
        times.append([1.52])
        keys.append([[0.0108, [3, -0.506667, 0], [3, 0, 0]]])

        names.append("RHipPitch")
        times.append([1.52])
        keys.append([[-0.337522, [3, -0.506667, 0], [3, 0, 0]]])

        names.append("RHipRoll")
        times.append([1.52])
        keys.append([[0.00924587, [3, -0.506667, 0], [3, 0, 0]]])

        names.append("RHipYawPitch")
        times.append([1.52])
        keys.append([[0.00464392, [3, -0.506667, 0], [3, 0, 0]]])

        names.append("RKneePitch")
        times.append([1.52])
        keys.append([[0.851412, [3, -0.506667, 0], [3, 0, 0]]])

        names.append("RShoulderPitch")
        times.append([1.52])
        keys.append([[1.94822, [3, -0.506667, 0], [3, 0, 0]]])

        names.append("RShoulderRoll")
        times.append([1.52])
        keys.append([[0.162562, [3, -0.506667, 0], [3, 0, 0]]])

        names.append("RWristYaw")
        times.append([1.52])
        keys.append([[-0.131966, [3, -0.506667, 0], [3, 0, 0]]])

        try:
            # uncomment the following line and modify the IP if you use this script outside Choregraphe.
            motion = ALProxy("ALMotion", "192.168.2.103", 9559)
            # motion = ALProxy("ALMotion")
            motion.angleInterpolationBezier(names, times, keys)
        except BaseException, err:
            print err

    def __init__(self, robot_ip, robot_port=9559):
        # 线程类初始化
        threading.Thread.__init__(self)
        # 障碍物标志
        self.obstacle_left = False  # True则左侧有障碍
        self.obstacle_right = False  # True则右侧有障碍
        self.go_back = False
        self.run_flag = False  # 避障运行标志位，为False时表示退出避障循环
        # 障碍物全局变量
        self.check_distance = 0.3  # 设置检测的安全距离
        self.too_close_distance = 0.15
        self.delay_seconds = 0.1  # 设置延时事件, 单位：秒
        self.move_speed = 0.08  # 移动速度, 单位: m/s
        self.turn_angle = 10  # 旋转角度，单位: 角度
        self.wall_angle = 45  # 转身是否后退的阈值
        self.test_angle = 50
        self.easy_angle_right = 20
        self.walk_delay = 0.2
        self.back_delay = 0.6
        self.turn_delay = 1.0
        self.test_delay = 2
        self.state = 0  # 状态机的变量
        self.num = 0
        self.test_num = 0
        self.test_turn_right = 20  # 控制右转的阈值
        self.avoid_hand = 0
        self.image = None
        self.last_left = 100
        self.last_right = 100
        self.delte_distance = 0  # 距离差的阈值
        self.kCameraSelectID = 18
        self.memValue = "LandmarkDetected"
        self.val = 0  # 初始化

        # naoqi.ALProxy
        try:
            self.motion = ALProxy("ALMotion", robot_ip, robot_port)
            self.memory = ALProxy("ALMemory", robot_ip, robot_port)
            self.sonar = ALProxy("ALSonar", robot_ip, robot_port)
            self.camera = ALProxy('ALVideoDevice', robot_ip, robot_port)
            self.tts = ALProxy("ALTextToSpeech", robot_ip, robot_port)
            self.posture = ALProxy("ALRobotPosture", robot_ip, robot_port)
            self.tracker = ALProxy("ALTracker", robot_ip, robot_port)
            self.cameraModule = ALProxy("ALVideoDevice", robot_ip, robot_port)
            self.landMarkProxy = ALProxy("ALLandMarkDetection", robot_ip, robot_port)
            self.memoryProxy = ALProxy("ALMemory", robot_ip, robot_port)
        except Exception, e:
            print "Could not create proxy by ALProxy in Class avoidance"
            print "Error was: ", e

        resolution = vision_definitions.kVGA
        colorSpace = vision_definitions.kRGBColorSpace
        fps = 15
        try:
            self.cameraModule.setParam(self.kCameraSelectID, 1)
            self.video_client = self.camera.subscribeCamera('python_client', 1, resolution, colorSpace, fps)
        except Exception, e:
            print 'Subscribe error'
            print 'Error was: ', e

        period = 500
        try:
            self.landMarkProxy.subscribe("Test_LandMark", period, 0.0)
        except Exception, e:
            print 'Subscribe error'
            print 'Error was: ', e

    def getflag(self):
        '''
            返回运行FLAG，为True表示正在运行, 为False表示停止工作;
        '''
        return self.run_flag

    def setflag(self, bools):
        '''
            设置运行FLAG, 从而控制避障功能的on/off;
        '''
        self.run_flag = bools
        return self.run_flag

    def run(self):
        '''
            固定间隔循环检测是否存在障碍，根据障碍物标志决定机器人的行走方向
            通过设置run_flag标志位为False来停止。
        '''
        # 初始时设置运行标志位为True
        self.setflag(True)
        # 机器人行走初始化
        self.motion.wakeUp()
        self.motion.moveInit()
        # 订阅超声波
        self.sonar.subscribe("Class_avoidance")
        self.change_post()
        time.sleep(2)
        while self.run_flag :  # 避障标识为True，则持续循环检测
            print "now state is %d, delay: %f" % (self.state, self.delay_seconds)
            # 1. 检测障碍物
            self.avoid_check()
            # 2. 根据障碍物标志决定行走方向
            self.avoid_operation()
            self.fall_check()
            # 3. 延时

            time.sleep(self.delay_seconds)
            if landmark_num < 4:
                self.avoid_land_check()

        # 直到run_flag为False才会跳出while循环;
        # 取消订阅超声波
        print 'unsubscribing...'
        self.sonar.unsubscribe("Class_avoidance")
        self.camera.unsubscribe(self.video_client)
        # 机器人复位
        self.motion.stopMove()
        self.motion.rest()

    def stop(self):
        self.setflag(False)

    def pause(self):
        self.__flag.clear()  # 设置为False, 让线程阻塞

    def resume(self):
        self.__flag.set()  # 设置为True, 让线程停止阻塞

    def avoid_check(self):
        '''
            检测超声波数值，设置标志位
        '''
        left_value = self.memory.getData("Device/SubDeviceList/US/Left/Sensor/Value")
        right_value = self.memory.getData("Device/SubDeviceList/US/Right/Sensor/Value")
        if left_value > self.check_distance:  # 超过安全距离，无障碍
            self.obstacle_left = False
        else:
            self.obstacle_left = True  # 小于安全距离，有障碍

        if right_value > self.check_distance:  # 超过安全距离，无障碍
            self.obstacle_right = False
        else:  # 小于安全距离，有障碍
            self.obstacle_right = True
        if self.obstacle_right or self.obstacle_left:  # 感觉可以注释掉 BY CP
            if abs(right_value - left_value) < self.delte_distance:
                self.obstacle_left = True
                self.obstacle_right = True

        if left_value < self.too_close_distance or right_value < self.too_close_distance:  # and 改成 or BY CP
            self.go_back = True
        else:
            self.go_back = False
        print "now state is %d, left: %f, right: %f" % (self.state, left_value, right_value)

    def avoid_land_check(self):
        self.val = self.memoryProxy.getData(self.memValue, 0)
        print ""
        print "\*****"
        print ""  # Check whether we got a valid output: a list with two fields.
        if (self.val and isinstance(self.val, list) and len(self.val) >= 2):
            timeStamp = self.val[0]
            markInfoArray = self.val[1]
            try:
                data = self.memory.getData("LandmarkDetected", 0)
                # if there is position
                if (data):
                    # get information about the size
                    sizeX = data[1][0][0][3]
                    sizeY = data[1][0][0][4]
                    size = 0
                    if (sizeX >= sizeY):
                        size = sizeX
                    else:
                        size = sizeY
                    # check the distance from Nao till the landmark

                    global distance
                    if (size <= 0.242):
                        distance = 0.575
                    elif (size <= 0.254):
                        distance = 0.55
                    elif (size <= 0.271):
                        distance = 0.50
                    elif (size <= 0.291):
                        distance = 0.45
                    elif (size <= 0.309):
                        distance = 0.4
                    elif (size <= 0.329):
                        distance = 0.35
                    elif (size <= 0.355):
                        distance = 0.3
                    elif (size <= 0.385):
                        distance = 0.25
                    elif(size<= 0.392):
                        distance = 0.23
                    else:
                        distance = 0.2
                else:
                    distance = 0
                print distance
                print "x"
                print  sizeX
                print "y"
                print sizeY

                distance -= distance*0.1
                if distance > 0:
                    while distance > 0.1:
                        self.avoid_check()
                        self.avoid_easy()
                        self.motion.moveTo(0.1, 0.0, 0, [["MaxStepFrequency", 1.0], ["StepHeight", 0.04] ])
                        distance -= 0.1

                    self.avoid_check()
                    self.avoid_easy()
                    self.motion.moveTo(distance, 0.0, 0, [["MaxStepFrequency", 1.0], ["StepHeight", 0.04] ])

                    print "reach target"
                    self.landmark_decision()
            except KeyboardInterrupt:
                print
                print "Interrupted by user"
                print "Stopping..."

        return

    def landmark_decision(self):
        global landmark_num
        global isBegin
        if landmark_num == 0 :
            print "turn right 90 landmark"
            self.motion_turn_right(self.test_angle)
            time.sleep(0.5)
            self.motion_turn_right(self.test_angle)
            self.state = 1
            landmark_num += 1
        elif landmark_num == 1:
            print "turn right 90 landmark"
            self.motion_turn_right(self.test_angle)
            time.sleep(0.5)
            self.motion_turn_right(self.test_angle)
            self.state = 1
            landmark_num += 1
        elif landmark_num == 2:
            print "turn left 90 landmark"
            self.motion_turn_left(self.test_angle)
            time.sleep(0.5)
            self.motion_turn_left(self.test_angle)
            self.state = 1
            landmark_num += 1
        elif landmark_num == 3:
                self.motion_turn_left(self.test_angle)
                time.sleep(0.5)
                self.motion_turn_left(self.test_angle)
                print "turn left 90 landmark"
            self.state = 1
            landmark_num += 1
        else:
            print "nothing to do"
            return
        return

    def fall_check(self):
        global hasFallen
        if hasFallen == 1:
            hasFallen = 0
            time.sleep(20)
            self.change_post()
            return

    def avoid_easy(self):

        if self.obstacle_left == False:
            if self.obstacle_right == False:
                print "normal"
            else:
                self.motion_turn_left(self.easy_angle_right)
                print "lllll"
                time.sleep(0.5)
        else:
            if self.obstacle_right == False:
                self.motion_turn_right(self.easy_angle_right / 2)
                print "rrrr"
                time.sleep(0.3)
        return

    def avoid_operation(self):

        self.delay_seconds = self.walk_delay

        if self.go_back == True:
            self.motion_back()
            return

        if self.state == 0:
            if self.obstacle_left == True and self.obstacle_right == True:
                self.delay_seconds = self.test_delay
                self.motion.moveTo(0.05, 0, 0, [["MaxStepFrequency", 1.0], ["StepHeight", 0.04]])
                self.motion_turn_left(self.test_angle)
                time.sleep(0.5)
                self.motion_turn_left(self.test_angle)
                print "Turn left 90"
                self.state = 1
                self.num = 0
                return
            else:
                self.state = 0
        elif self.state == 1:
            self.num += 1
            if self.obstacle_left == True and self.obstacle_right == True:
                self.motion_turn_right(self.test_angle)
                time.sleep(0.5)
                self.motion_turn_right(self.test_angle)
                time.sleep(0.5)
                self.motion_turn_right(self.test_angle)
                time.sleep(0.5)
                self.motion_turn_right(20)
                time.sleep(0.5)
                self.motion.moveTo(-0.05, 0, 0, [["MaxStepFrequency", 1.0], ["StepHeight", 0.04]])
                print "Turn Right 180 !"
                self.state = 2
                self.delay_seconds = self.test_delay
                self.num = 0
                return
            else:
                self.state = 1
        elif self.state == 2:
            self.num += 1
            if self.obstacle_left == True and self.obstacle_right == True:
                self.motion_turn_right(self.test_angle)
                time.sleep(0.5)
                self.motion_turn_right(self.test_angle)
                time.sleep(0.5)
                self.delay_seconds = self.test_delay
                "Turn Right 90 !"
                self.state = 0
                self.num = 0
                return
            else:
                self.state = 2

        if self.num >= 8  :
            self.num = 0
            self.state = 0

        if self.obstacle_left == False:
            if self.obstacle_right == False:
                self.motion_go()
            else:
                self.motion_turn_left(self.turn_angle)
        else:
            if self.obstacle_right == False:
                self.motion_turn_right(self.turn_angle)

    def motion_stand(self):
        self.motion.stopMove()

    def motion_go(self):
        # Example showing how to disable left arm motions during a move
        leftArmEnable = False
        rightArmEnable = False
        self.motion.setMoveArmsEnabled(leftArmEnable, rightArmEnable)
        self.motion.moveTo(self.move_speed, 0, 0, [["MaxStepFrequency", 1.0], ["StepHeight", 0.04] ])

    def motion_back(self):
        self.motion.move(-0.2 * self.move_speed, 0, 0)

    def motion_turn_left(self, turn_angle):
        if not turn_angle < self.wall_angle:
            # step back a bit
            self.motion_back()
            time.sleep(self.walk_delay)
        self.motion.moveTo(0, 0, turn_angle * almath.TO_RAD)

    def motion_turn_right(self, turn_angle):
        if not turn_angle < self.wall_angle:
            # step back a bit
            self.motion_back()
            time.sleep(self.walk_delay)
        self.motion.moveTo(0, 0, -1 * turn_angle * almath.TO_RAD)


class NowStandupModule(ALModule):
    """ A simple module able to react
    to fell down events
    """

    def __init__(self, name):
        ALModule.__init__(self, name)
        # No need for IP and port here because
        # we have our Python broker connected to NAOqi broker

        # Create a proxy to ALTextToSpeech for later use
        self.tts = ALProxy("ALTextToSpeech")
        self.motionProxy = ALProxy("ALMotion")

        self.postureProxy = ALProxy("ALRobotPosture")

        # Subscribe to the FaceDetected event:
        global memory
        memory = ALProxy("ALMemory")
        memory.subscribeToEvent("robotHasFallen",
                                "NowStandup",
                                "onFallenDetected")

    def onFallenDetected(self, *_args):
        """ This will be called each time Nao fell down

        """
        # Unsubscribe to the event when talking,
        # to avoid repetitions
        memory.unsubscribeToEvent("robotHasFallen",
                                  "NowStandup")
        global hasFallen
        hasFallen = 1

        self.tts.say("naive")

        self.postureProxy.goToPosture("StandInit", 1.0)
        # Subscribe again to the event
        memory.subscribeToEvent("robotHasFallen",
                                "NowStandup",
                                "onFallenDetected")
        print "begin"


class ReactToTouch(ALModule):
    """ A simple module able to react
        to touch events.
    """

    def __init__(self, name):
        ALModule.__init__(self, name)
        self.tts = ALProxy("ALTextToSpeech")
        self.motionProxy = ALProxy("ALMotion")

        # Subscribe to TouchChanged event:
        global memory
        memory = ALProxy("ALMemory")
        memory.subscribeToEvent("TouchChanged",
                                "ReactToTouch",
                                "onTouched")

    def onTouched(self, strVarName, value):

        memory.unsubscribeToEvent("TouchChanged",
                                  "ReactToTouch")

        touched_bodies = []
        for p in value:
            if p[1]:
                touched_bodies.append(p[0])

        self.say(touched_bodies)

        # Subscribe again to the event
        memory.subscribeToEvent("TouchChanged",
                                "ReactToTouch",
                                "onTouched")

    def say(self, bodies):
        if (bodies == []):
            return

        sentence = "My " + bodies[0]

        for b in bodies[1:]:
            sentence = sentence + " and my " + b
            print b
            if (b.find('Head') == 0):
                if b.find('Front') != -1:
                    print "1"
                    self.motionProxy.moveInit()
                    global isBegin
                    isBegin = 1    # 与C无关
                    self.tts.say("on my way")
                elif b.find('RHand') != -1:
                    print "right obstacle"
                    # self.motionProxy.moveInit()
                    self.motionProxy.moveTo(0, 0.05, 0, [["MaxStepFrequency", 1.0], ["StepHeight", 0.04]])
                elif b.find('LHand') != -1:
                    print "left obstacle"
                    # self.motionProxy.moveInit()
                    self.motionProxy.moveTo(0, -0.05, 0, [["MaxStepFrequency", 1.0], ["StepHeight", 0.04]])



def main(robot_IP, robot_PORT=9559):
    # ----------> avoidance <----------
    avoid = avoidance(robot_IP, robot_PORT)
    global avoid

    myBroker = ALBroker("myBroker",
                        "0.0.0.0",  # listen to anyone
                        0,  # find a free port and use it
                        robot_IP,  # parent broker IP
                        robot_PORT)  # parent broker port

    NowStandup = NowStandupModule("NowStandup")
    global NowStandup

    ReactToTouch = ReactToTouch("ReactToTouch")
    global ReactToTouch

    global isBegin
    while isBegin == 0:
        time.sleep(0.5)

    try:
        avoid.run()

    except KeyboardInterrupt:
        # 中断程序
        avoid.stop()
        print "Interrupted by user, shutting down"
        avoid.sonar.unsubscribe("Class_avoidance")
        avoid.camera.unsubscribe(avoid.video_client)
        # 机器人复位
        avoid.motion.stopMove()
        avoid.motion.wakeUp()
        avoid.motion.moveInit()
        sys.exit(0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="nao.local", help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559, help="Robot port number")
    args = parser.parse_args()
    # ----------> 执行main函数 <----------
    main(args.ip, args.port)
