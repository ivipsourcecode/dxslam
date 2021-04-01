


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <unistd.h>
#include<opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "cnpy.h"
#include<System.h>

#ifndef CV_LOAD_IMAGE_UNCHANGED
#define CV_LOAD_IMAGE_UNCHANGED -1
#endif

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

void getdescriptor(string filename,cv::Mat & descriptor,int nkeypoints);
void getGlobaldescriptor(string filename,cv::Mat & descriptor);
void getKeyPoint(string filename , vector<cv::KeyPoint> & keyPoints);

int main(int argc, char **argv)
{
    if(argc != 6)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association path_to_feature" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strSequenceFolder = string(argv[3]);
    string strAssociationFilename = string(argv[4]);
    string featureFolder = string(argv[5]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    DXSLAM::System SLAM(argv[1],argv[2],DXSLAM::System::RGBD, true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    //cat the image path
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    //main loop
    cv::Mat imRGB, imD;
    for (int ni = 0; ni < vstrImageFilenamesRGB.size(); ni++) {
        // Read image and depthmap from file
        imRGB = cv::imread(strSequenceFolder + "/" + vstrImageFilenamesRGB[ni], CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(strSequenceFolder+ "/" + vstrImageFilenamesD[ni], CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if (imRGB.empty()) {
            cerr << endl << "Failed to load image at: "
                 << strSequenceFolder << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image and hf-net output to the SLAM system
        cv::Mat local_desc;
        cv::Mat global_desc;
        vector<cv::KeyPoint> keypoints;

        // Get keyPoint,local descriptor and global descriptor
        getKeyPoint(featureFolder + "/point-txt/" + to_string(vTimestamps[ni]) + ".txt" , keypoints);
        local_desc.create(keypoints.size(), 256, CV_32F);
        getdescriptor(featureFolder + "/des/" + to_string(vTimestamps[ni]) + ".npy" , local_desc , keypoints.size());
        global_desc.create(4096, 1, CV_32F);
        getGlobaldescriptor(featureFolder + "/glb/" + to_string(vTimestamps[ni]) + ".npy" , global_desc);

        SLAM.TrackRGBD(imRGB, imD, tframe, keypoints , local_desc , global_desc);
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame
        usleep(1000 * 20);

    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    vstrImageFilenamesD.clear();
    vstrImageFilenamesRGB.clear();
    vTimestamps.clear();
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}

void getdescriptor(string filename,cv::Mat & descriptor,int nkeypoints){
    cnpy::NpyArray arr = cnpy::npy_load(filename);
    for(int i = 0 ; i < nkeypoints ; i ++){
        float* pdata= descriptor.ptr<float>(i);
        for(int j = 0 ; j < 256 ; j ++ ){
            float temp = arr.data<float>()[i *256 + j];
            pdata[j]= temp;
        }
    }
}

void getGlobaldescriptor(string filename,cv::Mat & descriptor){
    cnpy::NpyArray arr = cnpy::npy_load(filename);
    float* pdata= descriptor.ptr<float>(0);
    for(int j = 0 ; j < 4096 ; j ++ ){
        pdata[j]= arr.data<float>()[j];
    }
}

void getKeyPoint(string filename , vector<cv::KeyPoint> & keyPoints){
    ifstream getfile(filename);

    for(int i = 0 ; i < 550 && !getfile.eof()  ; i++)
    {
        string s;
        getline(getfile,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t_x;
            double t_y;
            ss >> t_x;
            ss >> t_y;
            cv::KeyPoint keyPoint (t_x,t_y,1);
            keyPoint.octave = 0;
            keyPoints.push_back(keyPoint);
        }
    }
}
