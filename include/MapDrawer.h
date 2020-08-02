#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include"Map.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include<pangolin/pangolin.h>

#include<mutex>

namespace DXSLAM
{

class MapDrawer
{
public:
    MapDrawer(Map* pMap, const std::string &strSettingPath);

    Map* mpMap;

    void DrawMapPoints();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentCameraPose(const cv::Mat &Tcw);
    void SetReferenceKeyFrame(KeyFrame *pKF);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

private:

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    cv::Mat mCameraPose;

    std::mutex mMutexCamera;
};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
