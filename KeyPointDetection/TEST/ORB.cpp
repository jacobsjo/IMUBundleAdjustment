//How to build? g++ ORB.cpp -o ORB `pkg-config --cflags --libs opencv`
//How to call the Function: ./ORB ../../data/1.png ../../data/2.png
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>

//#include <ceres/ceres.h>
//#include <ceres/rotation.h>

using namespace std;
using namespace cv;

//File Format we need to follow is:

void write2File(string output,vector <int> Frames,vector<KeyPoint>keypoints_1,vector<KeyPoint>keypoints_2,vector<DMatch> good_matches,int num_cameras)
{
  ofstream outfile(output);
  //Add file headers
  //<num_cameras> <num_keypoints> <num_total keypoints_index> <num_observations= num_frames*num_keypoints>

  //outfile << "num_cameras\t" << "num_keypoints\t" << "num_observations" << endl;

  //TODO: add the values of the header

  outfile<<num_cameras<<" "<<keypoints_1.size()<<" "<< num_cameras*keypoints_1.size()<<" "<<endl;
  //outfile <<"Frame Index\t" << "Keypoint Index\t" << "x-coordinate of Keypoint\t" << "y-coordinate of Keypoint\t" << endl;

  for (vector<DMatch>::size_type i=0;i<good_matches.size();i++)
  {
    cout << good_matches[i].queryIdx << "QueryIndex" << endl;
    cout << good_matches[i].trainIdx << "trainIndex" << endl;
    outfile << Frames[0]<<" "<<good_matches[i].queryIdx<<"\t"<<keypoints_1[good_matches[i].queryIdx].pt.x<<" "<<keypoints_1[good_matches[i].queryIdx].pt.y<<endl;
    outfile << Frames[1]<<" "<<good_matches[i].trainIdx<<"\t"<<keypoints_2[good_matches[i].trainIdx].pt.x<<" "<<keypoints_2[good_matches[i].trainIdx].pt.y<<endl;
  }
  outfile.close();
}

void BundleAdjustment()
{
   //ceres::Solver::Summary summary;
}
int main (int argc, char** argv)
{

    if ( argc != 3 )
    {
        cout<<"usage: feature_extraction img1 img2"<<endl;
        return 1;
    }

    //TODO: One more for Loop to get pair wise images
    //perform keypoint extraction and then bundle adjustment
    //Loop over the data folder build the image file inde
    //Build a keyFrame name. Each image is a KeyFrame Name
    vector<cv::String> fn;
    glob("../../data/*.png", fn, false);
    vector<Mat> images;
    vector <int> Frames;
    size_t num_cameras = fn.size(); //number of png files in images folder
    for (size_t i=0;i<num_cameras;i++)
    {
      Frames.push_back(i);
      cout << "image names:" << fn[i] << endl;
      cout << "Frame names:" << Frames[i] << endl;
    }
    Mat img_1 = imread(argv[1],CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2],CV_LOAD_IMAGE_COLOR);

    //Vector Keypoints, keypoints
    vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create("BruteForce-Hamming");

    detector->detect(img_1,keypoints_1);
    cout << "Size of keypoints1:" << keypoints_1.size() << endl;
    detector->detect(img_2,keypoints_2);
    cout << "Size of Keypoints2:" << keypoints_2.size() << endl;

    descriptor->compute(img_1,keypoints_1,descriptors_1);
    descriptor->compute(img_2,keypoints_2,descriptors_2);

    Mat outimg1;
    drawKeypoints(img_1,keypoints_1,outimg1,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
    imshow("original img",outimg1);

    vector<DMatch> matches; //stores the matches descriptors
    matcher->match(descriptors_1, descriptors_2, matches);
    double min_dist=10000, max_dist=0;

    for (int i=0;i<descriptors_1.rows;i++)
    {
        double dist = matches[i].distance;
        if (dist<min_dist) min_dist=dist;
        if (dist>max_dist) max_dist=dist;
    }
    cout << "-- Max dist : %f \n" << max_dist << endl;
    cout << "-- Min dist : %f \n"  << min_dist << endl;

    vector<DMatch> good_matches;
    for (int i=0;i<descriptors_1.rows;i++)
    {
        if (matches[i].distance<=max(2*min_dist, 30.0))
        {
            good_matches.push_back(matches[i]);
        }
    }

    //We read each pair of images from the data directory, perfom keypoint,
    //detection, matching, and write it on the text file
    string output = "../../output/bundle_data.txt";
    write2File(output,Frames,keypoints_1,keypoints_2,good_matches,num_cameras);

    Mat img_match;
    Mat img_goodmatch;
    drawMatches (img_1,keypoints_1,img_2,keypoints_2,matches,img_match);
    drawMatches (img_1,keypoints_1,img_2,keypoints_2,good_matches,img_goodmatch);
    //drawMatches
    drawMatches (img_1,keypoints_1,img_2,keypoints_2,matches,img_match);

    imshow ("result low",img_match);
    imshow ("result high",img_goodmatch);
    waitKey(0);
    return 0;
}
