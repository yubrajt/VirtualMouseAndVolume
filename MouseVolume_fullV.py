import cv2
import time
import autopy
import numpy
import math
import pyautogui
import mediapipe

from ctypes import cast, POINTER #allows use of c compatible data data files and dlls and typecasting
from comtypes import CLSCTX_ALL #comtypes allows use of COM interfaces
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume #importing volumecontrol functions

widthOfCamera, heightOfCamera = 640, 480 #heignt and width of camera
frameSkip = 10 #for lesser cpu usage
smoothening = 3 #smoothening factor
previousTime = 0
vol = 0
VolumeBar = 0
volumePercentage = 0
previousXordinate, previousYordinate = 0,0 #initializing ordinates of mouse cursor in previous frame
currentXordinate, currentYordinate = 0,0 #initializing ordinates of mouse cursor in current frame
VolumebarColor = (255, 0, 0) #filled rectangle representing volume

cap = cv2.VideoCapture(0) #0 for internal webcam, 1 for external

widthOfScreen, heightOfScreen = autopy.screen.size() #get screensize using autopy
devices = AudioUtilities.GetSpeakers() #get speaker usage
interface = devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
volume = cast(interface, POINTER(IAudioEndpointVolume))
volRange = volume.GetVolumeRange()
minimumVolume = volRange[0] #min decreasable volume
maximumVolume = volRange[1] #max increseable volume

class HandClass():
    def __init__(self, AlwaysTrack=False, NoOfHands=1, modelC=1, detectConfidence=0.7, trackConfidence=0.7): #constructor call for initializing variables
        self.AlwaysTrack = AlwaysTrack
        self.NoOfHands = NoOfHands
        self.modelC = modelC
        self.detectConfidence = detectConfidence
        self.trackConfidence = trackConfidence
        self.mediapipeHands = mediapipe.solutions.hands
        self.hands = self.mediapipeHands.Hands(self.AlwaysTrack, self.NoOfHands, self.modelC, self.detectConfidence, self.trackConfidence)
        self.mediapipeDraw = mediapipe.solutions.drawing_utils
        self.tipIds = [4, 8, 12, 16, 20]

    def detectHands(self, Video, draw=True):
        VideoRGB = cv2.cvtColor(Video, cv2.COLOR_BGR2RGB) #convert BGR to RGB
        self.results = self.hands.process(VideoRGB) #store processed video in result
        if self.results.multi_hand_landmarks:
            for handLandmarks in self.results.multi_hand_landmarks:
                if draw:
                    self.mediapipeDraw.draw_landmarks(Video, handLandmarks, self.mediapipeHands.HAND_CONNECTIONS)
        return Video

    def findPosition(self, Video, handNo=0, draw=True):
        xList = []
        yList = []
        boundingBox = []
        self.LandmarkList = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                # print(id, lm)
                h, w, c = Video.shape
                cx, cy = int(lm.x * w), int(lm.y * h) #changing to pixels
                xList.append(cx)
                yList.append(cy)
                # print(id, cx, cy)
                self.LandmarkList.append([id, cx, cy])
                if draw:
                    cv2.circle(Video, (cx, cy), 5, (255, 255, 255), cv2.FILLED)

            xmin, xmax = min(xList), max(xList)
            ymin, ymax = min(yList), max(yList)
            boundingBox = xmin, ymin, xmax, ymax

            if draw:
                cv2.rectangle(Video, (xmin - 20, ymin - 20), (xmax + 20, ymax + 20), (0, 255, 0), 2)
        return self.LandmarkList, boundingBox

    def fingersUp(self):
        fingers = []
        # For thumb
        if self.LandmarkList[self.tipIds[0]][1] > self.LandmarkList[self.tipIds[0] - 1][1]:
            fingers.append(1)
        else:
            fingers.append(0)

        # For all fingers
        for id in range(1, 5):
            if self.LandmarkList[self.tipIds[id]][2] < self.LandmarkList[self.tipIds[id] - 2][2]:
                fingers.append(1)
            else:
                fingers.append(0)
        return fingers

    def findDistance(self, p1, p2, Video, draw=True):
        x1, y1 = self.LandmarkList[p1][1], self.LandmarkList[p1][2]
        x2, y2 = self.LandmarkList[p2][1], self.LandmarkList[p2][2]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

        if draw:
            cv2.line(Video, (x1, y1), (x2, y2), (255, 0, 255), 3)
            cv2.circle(Video, (x1, y1), 10, (0, 0, 255), cv2.FILLED)
            cv2.circle(Video, (x2, y2), 10, (0, 0, 255), cv2.FILLED)
            cv2.circle(Video, (cx, cy), 10, (255, 0, 255), cv2.FILLED)
        length = math.hypot(x2 - x1, y2 - y1)

        return length, Video, [x1, y1, x2, y2, cx, cy]

detector = HandClass(NoOfHands=1)
'''
fps issue if detector object is placed inside main
'''
while True:
    #1. Find landmarks{
    success, Video = cap.read() #first variable returns if video is readable or not i.e. bool value
    Video = detector.detectHands(Video)
    LandmarksArray, boundingBox = detector.findPosition(Video)

    if len(LandmarksArray) != 0:
        x1,y1 = LandmarksArray[8][1:]
        x2,y2 = LandmarksArray[12][1:]
        #print(x1, y1, x2, y2) #Print the coordinates of fingertips
        fingers = detector.fingersUp()
        cv2.rectangle(Video, (frameSkip, frameSkip), (widthOfCamera - frameSkip, heightOfCamera - 10*frameSkip), (255, 255, 255), 2)
        if fingers[1] == 1 and fingers[2] == 0 and fingers[0] == 0:
            x3 = numpy.interp(x1*1.3, (frameSkip,widthOfCamera-frameSkip),(0,widthOfScreen))
            y3 = numpy.interp(y1*1.1, (frameSkip,heightOfCamera-frameSkip), (0,heightOfScreen))
            currentXordinate = previousXordinate + (x3 - previousXordinate) /smoothening
            currentYordinate = previousYordinate + (y3 - previousYordinate) /smoothening
            area = (boundingBox[2] - boundingBox[0]) * (boundingBox[3] - boundingBox[1]) // 100
            autopy.mouse.move(widthOfScreen- currentXordinate, currentYordinate)
            cv2.circle(Video, (x1,y1), 15, (0,0,255), cv2.FILLED)
            previousXordinate, previousYordinate = currentXordinate, currentYordinate

        if fingers[1] == 1 and fingers[2] == 1 and fingers[0] == 0:
            length, Video , lineInfo = detector.findDistance(8,12,Video)
            #print(length)

            if length<30:
                cv2.circle(Video, (lineInfo[4], lineInfo[5]), 15, (0, 255, 0), cv2.FILLED)
                autopy.mouse.click()



        if fingers[0] ==1 and fingers[1] == 1 and fingers[2] == 1 and fingers[3] == 0 and fingers[4] == 0:
            clength, Video , lineInfo = detector.findDistance(8, 12, Video)
            if clength < 30:
                cv2.circle(Video, (lineInfo[4], lineInfo[5]), 15, (0, 255, 0), cv2.FILLED)
                pyautogui.click(button='right')

        if fingers[0] ==1 and fingers[1] ==1 and fingers[2]==1 and fingers[3]==1 and fingers[4]==1 :
            x1, y1 = LandmarksArray[4][1], LandmarksArray[4][2]
            x2, y2 = LandmarksArray[8][1], LandmarksArray[8][2]
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

            cv2.circle(Video, (x1, y1), 15, (0, 0, 255), cv2.FILLED)
            cv2.circle(Video, (x2, y2), 15, (0, 0, 255), cv2.FILLED)
            cv2.line(Video, (x1, y1), (x2, y2), (25, 25, 200), 3)

            length = math.hypot(x2 - x1, y2 - y1)

            vol = numpy.interp(length, [20, 300], [minimumVolume, maximumVolume])
            VolumeBar = numpy.interp(length, [50, 300], [400, 150])
            volumePercentage = numpy.interp(length, [50, 300], [0, 100])
            volume.SetMasterVolumeLevel(vol, None)

            #volume, volumebar, volume% display
            cv2.rectangle(Video, (50, 150), (85, 400), (100, 0, 255), 3)
            cv2.rectangle(Video, (50, int(VolumeBar)), (85, 400), (0, 0, 255), cv2.FILLED)
            cv2.putText(Video, f':{int(volumePercentage)} %', (40, 450), cv2.FONT_HERSHEY_COMPLEX_SMALL, 2, (255, 0, 100), 3)
            cVol = int(volume.GetMasterVolumeLevelScalar() * 100)
            cv2.putText(Video, f'VOL SET: {int(cVol)}', (400, 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 2, VolumebarColor, 3)

    currentTime = time.time()
    fps = 1 / (currentTime - previousTime)
    previousTime = currentTime
    cv2.putText(Video, str(int(fps)), (20, 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 3, (100,255,0), 3)
    cv2.putText(Video, "fps" , (100, 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 3, (100,250,0), 3)
    cv2.imshow("MouseTracker", Video)
    cv2.waitKey(1)