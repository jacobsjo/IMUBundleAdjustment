//
// Created by Oushesh on 18/12/2019.
//
//https://docs.opencv.org/3.4/d8/dfe/classcv_1_1VideoCapture.html
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2(ighgui.hpp>
#include <stdio.h>
#include <string>

using namespace cv;
using namespace std;

//https://www.learnopencv.com/read-write-and-display-a-video-using-opencv-cpp-python/
void extractFrames(string & videoPath, string & imgsavePath)
{
  cap = cv.VideoCapture(videoPath)
  int count = 0;
  Mat frmae;
  while (cap.isOpened())
  {
    cap >> frame; //Capture Frame by Frame
    if (image.empty())
    {
      cout << "Can't read frames from camera";
      break;
    }
    imgsavePath = imgsavePath + "/frame" + count + ".jpg";
    imwrite(imgsavePath,frame);
    count +=1;
  }
}

int main(int, char**)
{
    string videoPath = "../../data/20191218_100925.mp4";
    string imgsavePath = "../../data/KeyFrames";
    extractFrames(videoPath,imgsavePath);
    return 0;
}
