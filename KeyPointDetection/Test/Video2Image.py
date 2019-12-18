#CV2 module
#to get image from video file


import cv2
import os

def extractFrames(videoPath,imgsavePath):
    cap = cv2.VideoCapture(videoPath)
    count = 0
    while (cap.isOpened()):
        ret, frame = cap.read()
        if ret:
            print('Read %d frame: ' % count, ret)
            #save file as JPEG file
            cv2.imwrite(os.path.join(imgsavePath, "frame{:d}.jpg".format(count)),frame)
            count +=1
        else:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    videoPath = "../../data/20191218_100925.mp4"
    imgsavePath = "../../data/KeyFrames"
    extractFrames(videoPath,imgsavePath)
