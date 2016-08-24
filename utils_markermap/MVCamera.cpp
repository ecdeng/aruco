#include "MVCamera.hpp"


using namespace std;

CMVCamera::CMVCamera()
{
    iplImage = NULL;
    channel=1;
}


int CMVCamera::InitMindVision()
{

        int                     iCameraCounts = 1;
        int                     iStatus=-1;
        tSdkCameraDevInfo       tCameraEnumList;



        CameraSdkInit(1);

        //枚举设备，并建立设备列表
        CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);
        cout << "get Camera: " << tCameraEnumList.acProductName << endl;
        //没有连接设备
        if(iCameraCounts==0){
            return -1;
        }


        //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
        iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);

        //初始化失败
        if(iStatus!=CAMERA_STATUS_SUCCESS){
            return -1;
        }

        //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
        CameraGetCapability(hCamera,&tCapability);

        //
        g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
        //g_readBuf = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);


        /*让SDK进入工作模式，开始接收来自相机发送的图像
        数据。如果当前相机是触发模式，则需要接收到
        触发帧以后才会更新图像。    */
        CameraPlay(hCamera);

        SetParameters();

        /*其他的相机参数设置
        例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
             CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
             CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
             更多的参数的设置方法，，清参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
        */

        if(tCapability.sIspCapacity.bMonoSensor){
            channel=1;
            CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
        }else{
            channel=3;
            CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
        }

        return 0;
}

void CMVCamera::SetParameters()
{
    BOOL            AEstate=FALSE;
    int             pbyAeTarget;
    double          pfExposureTime;
    int             pusAnalogGain;
    BOOL            FlickEnable=FALSE;
    int             piFrequencySel;
    double	        m_fExpLineTime=0;   //当前的行曝光时间，单位为us
    tSdkExpose      *  SdkExpose =   &tCapability.sExposeDesc;


    int piModeSel;
    CameraGetTriggerMode(hCamera,&piModeSel);


    int iFrameSpeed_get = -1;
    CameraSdkStatus status2 = CameraGetFrameSpeed(hCamera,&iFrameSpeed_get);
    if(status2!=0)
        cout << "CameraSetFrameSpeed Failed" << endl;

    int  iFrameSpeed = tCapability.iFrameSpeedDesc - 1;
       CameraSdkStatus  status = CameraSetFrameSpeed(hCamera, iFrameSpeed );

    // set to manual exposure
    CameraSetAeState(hCamera,false);

    double fExposureTime = 2500;
    CameraSetExposureTime(hCamera,fExposureTime);


    int iAnalogGain = 100;
    CameraSetAnalogGain(hCamera,iAnalogGain);


    int iGamma = 50;
    CameraSetGamma(hCamera,iGamma);

    int iContrast = 200;
    CameraSetContrast(hCamera,iContrast);



    //获得相机当前的曝光模式。
    CameraGetAeState(hCamera,&AEstate);

    //获得自动曝光的亮度目标值。
    CameraGetAeTarget(hCamera,&pbyAeTarget);

    //获得自动曝光时抗频闪功能的使能状态。
    CameraGetAntiFlick(hCamera,&FlickEnable);

    //获得相机的曝光时间。
    CameraGetExposureTime(hCamera,&pfExposureTime);

    //获得图像信号的模拟增益值。
    CameraGetAnalogGain(hCamera,&pusAnalogGain);

    //获得自动曝光时，消频闪的频率选择。
    CameraGetLightFrequency(hCamera,&piFrequencySel);

    int piGamma;
    CameraGetGamma(hCamera,&piGamma);

    int pemLutMode;
    CameraGetLutMode(hCamera,&pemLutMode);


    int piContrast;
    CameraGetContrast(hCamera,&piContrast);

/*
    获得一行的曝光时间。对于CMOS传感器，其曝光
    的单位是按照行来计算的，因此，曝光时间并不能在微秒
    级别连续可调。而是会按照整行来取舍。这个函数的
    作用就是返回CMOS相机曝光一行对应的时间。
*/
    CameraGetExposureLineTime(hCamera, &m_fExpLineTime);


    cout << "AEstate = " << AEstate << endl;
    cout << "pbyAeTarget = " << pbyAeTarget << endl;
    cout << "FlickEnable = " << FlickEnable << endl;
    cout << "pfExposureTime = " << pfExposureTime << endl;
    cout << "pusAnalogGain = " << pusAnalogGain << endl;
    cout << "piFrequencySel = " << piFrequencySel << endl;
    cout << "m_fExpLineTime = " << m_fExpLineTime << endl;
    cout << "piGamma = " << piGamma << endl;
    cout << "piContrast = " << piContrast << endl;
    cout << "\n";
}

bool CMVCamera::GetImage(cv::Mat& Iimag)
{

        if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)
        {
            CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);
            if (iplImage)
            {
                cvReleaseImageHeader(&iplImage);
            }
            iplImage = cvCreateImageHeader(cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight),IPL_DEPTH_8U,channel);
            cvSetData(iplImage,g_pRgbBuffer,sFrameInfo.iWidth*channel);//此处只是设置指针，无图像块数据拷贝，不需担心转换效率
//            //以下两种方式都可以显示图像或者处理图像
//            #if 0
//            cvShowImage("OpenCV Demo",iplImage);
//            #else
            Iimag = Mat(iplImage);//这里只是进行指针转换，将IplImage转换成Mat类型
//            imshow("OpenCV Demo",Iimag);
//            cout << "out image" << endl;
//            #endif

//             waitKey(5);

            //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
            //否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
            CameraReleaseImageBuffer(hCamera,pbyBuffer);

            return true;
        }

        else
            return false;
}

void CMVCamera::ReleseCamera()
{
    CameraUnInit(hCamera);
    //注意，现反初始化后再free
    free(g_pRgbBuffer);
}

CMVCamera::~CMVCamera(){
    ReleseCamera();
}

