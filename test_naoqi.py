import sys

from naoqi import ALProxy


# print sys.path

tts = ALProxy("ALTextToSpeech","192.168.1.100",9559)
tts.say("hello")



