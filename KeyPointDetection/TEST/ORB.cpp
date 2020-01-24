//How to build? g++ ORB.cpp -o ORB `pkg-config --cflags --libs opencv`
//How to call the Function: ./ORB ../../data/1.png ../../data/2.png
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;
int main (int argc, char** argv)
{

    if ( argc != 3 )
    {
        cout<<"usage: feature_extraction img1 img2"<<endl;
        return 1;
    }


    //Loop over the data folder build the image file inde
    //Build a keyFrame name. Each image is a KeyFrame Name
    vector<cv::String> fn;

    glob("../../data/*.png", fn, false);
    vector<Mat> images;
    vector <int> Frames;
    size_t count = fn.size(); //number of png files in images folder
    for (size_t i=0;i<count;i++)
    {
      Frames.push_back(i);

      cout << "image names:" << fn[i] << endl;
      cout << "Frame names:" << Frames[i] << endl;
    }

    /*
    for (size_t i=0; i<count; i++)
      images.push_back(imread(fn[i]));
      cout << "images" << images[i] << endl;
    */
    Mat img_1 = imread ( argv[1], CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread ( argv[2], CV_LOAD_IMAGE_COLOR );


    //Vector Keypoints, keypoints
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();

    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );


    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );


    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    Mat outimg1;
    drawKeypoints( img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    imshow("original img",outimg1);

    vector<DMatch> matches; //stores the matches descriptors
    matcher->match (descriptors_1, descriptors_2, matches);
    double min_dist=10000, max_dist=0;

    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = matches[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }
    cout << "-- Max dist : %f \n" << max_dist << endl;
    cout << "-- Min dist : %f \n"  << min_dist << endl;

    vector< DMatch > good_matches;
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( matches[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            good_matches.push_back (matches[i]);
        }
    }
    //Get the index of the Keypoints
    for (vector<DMatch>::size_type i=0;i<good_matches.size();i++)
    {
      cout << keypoints_1[good_matches[i].queryIdx].pt.x << "Query points_x" << "\t" << keypoints_1[good_matches[i].queryIdx].pt.y << "Query POints_Y" << endl;
      cout << keypoints_2[good_matches[i].trainIdx].pt.x << "Train points_x" << "\t" << keypoints_2[good_matches[i].trainIdx].pt.y << "Train POints_Y" << endl;
    }
    //TODO: write the keypoints in the format <x>,<y>
    //Datatype x and y as float
    //TODO index of images

    Mat img_match;
    Mat img_goodmatch;
    drawMatches (img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
    drawMatches (img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);
    //drawMatches

    drawMatches (img_1, keypoints_1, img_2, keypoints_2, matches, img_match);

    imshow ( "result low", img_match );
    imshow ( "result high", img_goodmatch );
    waitKey(0);
    return 0;
}
