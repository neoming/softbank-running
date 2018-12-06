import sys
sys.path.append('/home/liyiming/naoqi/lib/python2.7/site-packages')

from naoqi import ALProxy
from naoqi import ALModule
from naoqi import ALBroker
import threading
import socket
import time
import math
import numpy
import cv2
import vision_definitions
import time

FLAG = 0 #FLAG TO RUNNING
HOST = '192.168.1.109'    # partner ip
PORT = 50007              # The same port as used by the server

robot_IP = "192.168.1.101"
robot_PORT = 9559
memory = None
connect = None
sk = None
ra = None

class race(threading.Thread):
    def __init__(self, robot_ip, robot_port=9559):
        threading.Thread.__init__(self)
        # some parameter
        self.bMustStop = False
        self.time = 0.7
        self.headPitch = 24.0
        self.headYaw = 0.0
        self.turnFlag = 0
        self.count = 0
        # naoqi.ALProxy
        try:
            self.motion = ALProxy("ALMotion", robot_ip, robot_port)
            self.memory = ALProxy("ALMemory", robot_ip, robot_port)
            self.cam = ALProxy('ALVideoDevice', robot_ip, robot_port)
            self.tts = ALProxy("ALTextToSpeech", robot_ip, robot_port)
            self.posture = ALProxy("ALRobotPosture", robot_ip, robot_port)

        except Exception, e:
            print "Could not create proxy by ALProxy in Class avoidance"
            print "Error was: ", e

    def postReady(self):
        # stand up
        self.posture.goToPosture("StandInit", 1.0)
        # set head state
        HeadPitch = self.headPitch
        HeadYaw = self.headYaw
        rPitch = HeadPitch * math.pi / 180
        rYaw = HeadYaw * math.pi / 180
        time = self.time
        names = ["HeadYaw", "HeadPitch"]
        angleList = [rYaw, rPitch]
        timeList = [time, time]
        self.motion.angleInterpolation(names, angleList, timeList, True)

    def processing(self, img):
        array = bytearray(img[6])
        img = numpy.array(array)
        img = img.reshape([480, 640, 3])

        bm = numpy.array([0, 0, 128])
        bx = numpy.array([180, 50, 255])

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, bm, bx)
        bitwiseGreen = cv2.bitwise_and(img, img, mask=mask1)
        gray = cv2.cvtColor(bitwiseGreen, cv2.COLOR_BGR2GRAY)
        equalizehist = cv2.equalizeHist(gray)
        blur = cv2.bilateralFilter(equalizehist, 9, 75, 75)
        retval, binary = cv2.threshold(blur, 30, 255, cv2.THRESH_BINARY)
        kernel = numpy.ones((5, 5), numpy.uint8)
        final = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        return final

    def detectWhite(self, img, p1, p2, p3, p4):
        aMin = 6000
        aSquare = img[p2:p4, p1:p3]
        count = numpy.count_nonzero(aSquare)
        print count
        # cv2.imshow("debug", aSquare)
        # cv2.waitKey()
        if count > aMin:
            # self.log("detected White Pixel:%d" %count)
            return True
        else:
            return False

    def getCamID(self, str):
        resolution = vision_definitions.kVGA
        colorSpace = vision_definitions.kBGRColorSpace
        fps = 30
        nameId = self.cam.subscribeCamera(str, 1, resolution, colorSpace, fps)
        result = self.cam.setParameter(1, 22, 2)
        return nameId

    def saveImg(self,img,name,p1,p2,p3,p4):
        name = "./imgs/"+name+".jpg"
        cv2.imwrite(name, img[p2:p4, p1:p3])

    def runtest(self):
        self.cam.setActiveCamera(1)
        timestr = time.strftime('%x')+str(time.time())
        print timestr
        nameId = self.getCamID(timestr)
        self.bMustStop = False
        left = (0, 0, 320, 240)
        right = (320, 0, 640, 240)
        dleft = (20, 240, 320, 350)
        dright = (320, 220, 640, 350)
        self.motion.angleInterpolation(["HeadYaw", "HeadPitch"], [0., 0.418], [0.5, 0.5], True)
        while (not self.bMustStop):
            img = self.cam.getImageRemote(nameId)
            final = self.processing(img)
            if (self.detectWhite(final, *left) and self.detectWhite(final, *right)):
                self.tts.say("reach")
                self.saveImg(final, nameId + "right_reach", *right)
                self.saveImg(final, nameId + "left_reach", *left)
                print ("reach")
                # self.dash(self.turnFlag)
                self.bMustStop = True
            else:
                if (self.detectWhite(final, *left) or self.detectWhite(final, *dleft)):
                    self.tts.say("left")
                    self.saveImg(final,nameId+"dleft_left",*dleft)
                    self.saveImg(final,nameId+"left_left",*left)
                    print ("left")
                    self.turnFlag = -1
                    # self.go(self.turnFlag)
                if (self.detectWhite(final, *right) or self.detectWhite(final, *dright)):
                    self.tts.say("right")
                    self.saveImg(final, nameId + "dright_right", *dright)
                    self.saveImg(final, nameId + "right_right", *right)
                    print ("right")
                    self.turnFlag = 1
                    # self.go(self.turnFlag)
                if ((not self.detectWhite(final, *left)) and (not self.detectWhite(final, *right)) and
                        (not self.detectWhite(final, *dleft)) and (
                                not self.detectWhite(final, *dright))):
                    self.tts.say("mid")
                    self.saveImg(final, nameId + "dright_mid", *dright)
                    self.saveImg(final, nameId + "right_mid", *right)
                    self.saveImg(final, nameId + "dleft_mid", *dleft)
                    self.saveImg(final, nameId + "left_mid", *left)
                    print ("mid")
                    self.turnFlag = 0
                    # self.go(self.turnFlag)
            time.sleep(0.5)
        self.cam.unsubscribe(nameId)
        self.motion.angleInterpolation(["HeadYaw", "HeadPitch"], [0., 0.418], [0.5, 0.5], True)

    def run(self):
        self.cam.setActiveCamera(1)
        nameId = self.getCamID(time.strftime('%x'))
        self.bMustStop = False
        left = (0, 0, 320, 240)
        right = (320, 0, 640, 240)
        dleft = (20, 240, 320, 350)
        dright = (320, 220, 640, 350)
        self.turnFlag = 0

        self.motion.angleInterpolation(["HeadYaw", "HeadPitch"], [0., 0.418], [0.5, 0.5], True)
        while (not self.bMustStop):
            img = self.cam.getImageRemote(nameId)
            final = self.processing(img)
            self.dash(0)
            if (self.detectWhite(final, *left) and self.detectWhite(final, *right)):
                print("reach")
                self.dash(self.turnFlag)
                self.bMustStop = True
            else:
                if (self.detectWhite(final, *left) or self.detectWhite(final, *dleft)):
                    print("left")
                    self.turnFlag = -1
                    self.go(self.turnFlag)
                if (self.detectWhite(final, *right) or self.detectWhite(final, *dright)):
                    print ("right")
                    self.turnFlag = 1
                    self.go(self.turnFlag)
                if ((not self.detectWhite(final, *left)) and (not self.detectWhite(final, *right)) and
                        (not self.detectWhite(final, *dleft)) and (
                        not self.detectWhite(final, *dright))):
                    print("mid")
                    self.turnFlag = 0
                    self.go(self.turnFlag)
            time.sleep(0.5)
        self.cam.unsubscribe(nameId)

    def go(self,p):
        self.count = 0
        while self.count <= 2:
            self.motion.moveTo(0.5, 0.2 * p, 0.4 * p,  # 0.2p,0.4p
                               [["MaxStepX", 0.042],  # 0.08,0.18,0.1,0.6,0.002
                                ["MaxStepY", 0.18],
                                ["MaxStepTheta", 0.1],
                                ["MaxStepFrequency", 0.65],
                                ["StepHeight", 0.015],
                                ["TorsoWx", 0.0],
                                ["TorsoWy", 0.0]])
            self.count = self.count + 1
        self.motion.move(0, 0, 0)
        self.motion.waitUntilMoveIsFinished()
        self.count = 0

    def dash(self,p):
        self.count = 0
        while self.count <= 2:
            self.motion.moveTo(0.5, 0.2 * p, 0.4 * p,  # 0.2p,0.4p
                               [["MaxStepX", 0.042],   # 0.08,0.18,0.1,0.6,0.002
                                ["MaxStepY", 0.18],
                                ["MaxStepTheta", 0.1],
                                ["MaxStepFrequency", 0.65],
                                ["StepHeight", 0.015],
                                ["TorsoWx", 0.0],
                                ["TorsoWy", 0.0]])
            self.count = self.count + 1
        self.motion.move(0, 0, 0)
        self.motion.waitUntilMoveIsFinished()
        self.count = 0

    def sendReach(self):
        global sk
        sk = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sk.connect((HOST, PORT))
        sk.send("ecnu first runner")
        sk.close()
        self.tts.say("go go go !")

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
        self.postureProxy.goToPosture("StandInit", 1.0)
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
        global ra
        ra.run()
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
        global  FLAG
        if FLAG==0:
            FLAG =1
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
                    isBegin = 1
                    self.tts.say("on my way")
                elif b.find('RHand') != -1:
                    print "right obstacle"
                    # self.motionProxy.moveInit()
                    self.motionProxy.moveTo(0, 0.05, 0, [["MaxStepFrequency", 1.0], ["StepHeight", 0.04]])
                elif b.find('LHand') != -1:
                    print "left obstacle"
                    # self.motionProxy.moveInit()
                    self.motionProxy.moveTo(0, -0.05, 0, [["MaxStepFrequency", 1.0], ["StepHeight", 0.04]])
class ReactToWhistleModule(ALModule):
    """ A simple module able to react
        to touch events.
    """

    def __init__(self, name):
        ALModule.__init__(self, name)
        self.sd = ALProxy("ALSoundDetection")
        self.tts = ALProxy("ALTextToSpeech")
        self.sd.setParameter("Sensitivity", 0.004)
        #self.sd.subscribe("Whistle")
        # Subscribe to TouchChanged event:
        global memory
        memory = ALProxy("ALMemory")
        memory.subscribeToEvent("SoundDetected",
                                "ReactToWhistle",
                                "onHeard")

    def onHeard(self,value):

        memory.unsubscribeToEvent("SoundDetected",
                                  "ReactToWhistle")
        sound = memory.getData("SoundDetected")
        global FLAG
        if FLAG == 0:
            FLAG = 1
        self.tts.say("yes")
        # Subscribe again to the event
        memory.subscribeToEvent("SoundDetected",
                                "ReactToWhistle",
                                "onHeard")



def main():
    # ----------> avoidance <----------

    myBroker = ALBroker("myBroker",
                        "0.0.0.0",  # listen to anyone
                        0,  # find a free port and use it
                        robot_IP,  # parent broker IP
                        robot_PORT)  # parent broker port
    global NowStandup
    NowStandup = NowStandupModule("NowStandup")

    global ReactToWhistle
    ReactToWhistle = ReactToWhistleModule("ReactToWhistle")

    global ra
    ra = race(robot_IP,robot_PORT)
    ra.postReady()

    tts = ALProxy("ALTextToSpeech")
    tts.say("I am Waiting for whistle!")
    global FLAG
    while FLAG == 0:
        time.sleep(0.2)

    #ra.run()
    ra.sendReach()
    # global sk
    # sk = socket.socket()
    # sk.bind((robot_IP, LISTEN_PORT))
    # sk.listen(10)
    # tts.say("connect")
    # try:
    #     while True:
    #         global connection
    #         connection,address = sk.accept()
    #         CONNECT = True
    #         while CONNECT == True:
    #             try:
    #                 buf = connection.recv(1024)
    #                 tts.say("I get"+buf)
    #                 connection.send("I get"+buf)
    #             except socket.timeout:
    #                 CONNECT = False
    #                 print 'time out'
    #         connection.close()
    #         tts.say("byby")
    #         motion = ALProxy("ALMotion");
    #         motion.rest()
    # except KeyboardInterrupt:
    #     print ""
    #     print "Interrupted by user, shutting down"
    #     motion = ALProxy("ALMotion");
    #     motion.rest()
    #     sys.exit(0)
    # try:
    #     avoid.run()
    #
    # except KeyboardInterrupt:
    #     avoid.stop()
    #     print "Interrupted by user, shutting down"
    #     avoid.sonar.unsubscribe("Class_avoidance")
    #     avoid.camera.unsubscribe(avoid.video_client)
    #
    #     avoid.motion.stopMove()
    #     avoid.motion.wakeUp()
    #     avoid.motion.moveInit()
    #     sys.exit(0)


if __name__ == "__main__":
    main()
