//

#include "CameraApi.h"
#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;

class CMVCamera
{
public:
    unsigned char           * g_pRgbBuffer;     //处理后数据缓存区
    int                     hCamera;
    tSdkFrameHead           sFrameInfo;
    BYTE*			        pbyBuffer;
    IplImage *iplImage;
    int                     channel;
    tSdkCameraCapbility     tCapability;      //设备描述信息

    CMVCamera();
    ~CMVCamera();
    int InitMindVision();
    bool GetImage(cv::Mat& curImage);
    void ReleseCamera();
    void SetParameters();

private:

};



