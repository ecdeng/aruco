/*****************************************************************************************
Copyright 2016 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************************************************************************/

/**Shows how to use the tracker to track independent markers. This example assumes that marker can move wrt to each other
 * The tracker should sove the ambiguity problem that arises in some configurations
 */

#include <string>

#include <iostream>
#include "aruco.h"
#include "cvdrawingutils.h"
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace aruco;
//class for parsing command line
class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};


int main(int argc, char **argv) {
    try {
        CmdLineParser cml(argc,argv);
        if (argc <4 || cml["-h"] ){
            cerr << "Usage: video.avi  cameraParams.yml  markerSize [-d <dicionary>:ARUCO default] " << endl;
            cerr<<"\tDictionaries: "; for(auto dict:aruco::Dictionary::getDicTypes())    cerr<<dict<<" ";cerr<<endl;
            cout<<"Example to work with apriltags dictionary : video.avi -d TAG36h11"<<endl<<endl;
            return 0;
        }


        aruco::CameraParameters CamParam;

        // read the input image
        cv::Mat InImage;
        // Open input and read image
        VideoCapture vreader(argv[1]);
        if (vreader.isOpened()) vreader>>InImage;
        else{cerr<<"Could not open input"<<endl;return -1;}
        // read camera parameters if specifed
        CamParam.readFromXMLFile(argv[2]);
        CamParam.resize(InImage.size());
        // read marker size if specified (default value -1)
        float MarkerSize = std::stof(argv[3]);
        //Create the detector
        MarkerDetector MDetector;
        MDetector.setThresholdParams(7, 7);
        MDetector.setThresholdParamRange(2, 0);
        std::map<uint32_t,MarkerPoseTracker> MTracker;//use a map so that for each id, we use a different pose tracker
        //Set the dictionary you want to work with, if you included option -d in command line
        //see dictionary.h for all types
        if (cml["-d"])  MDetector.setDictionary( cml("-d"),0.f);
    
        int delay = 0;
        char key = 0;
        do{
            vreader.retrieve(InImage);
            // Ok, let's detect
            vector< Marker >  Markers=MDetector.detect(InImage);
            for(auto & marker:Markers)//for each marker
                MTracker[marker.id].estimatePose(marker,CamParam,MarkerSize);//call its tracker and estimate the pose

            // for each marker, draw info and its boundaries in the image
            for (unsigned int i = 0; i < Markers.size(); i++) {
                cout << Markers[i] << endl;
                Markers[i].draw(InImage, Scalar(0, 0, 255), 2);
            }
            // draw a 3d cube in each marker if there is 3d info
            if (CamParam.isValid() && MarkerSize != -1){
                for (unsigned int i = 0; i < Markers.size(); i++) {
                    CvDrawingUtils::draw3dCube(InImage, Markers[i], CamParam);
                    CvDrawingUtils::draw3dAxis(InImage, Markers[i], CamParam);
                }
            }
            // show input with augmented information
            cv::namedWindow("in", 1);
            cv::imshow("in", InImage);

            // play continuously if we press 's', abort on esc
            key = char(cv::waitKey(delay));
            if (key == 115) // key is 's'
                delay = (delay == 0) ? 15 : 0;

        } while( key != 27 && vreader.grab()); // wait for esc to be pressed


        if (cml["-o"]) cv::imwrite(cml("-o"), InImage);
    } catch (std::exception &ex)

    {
        cout << "Exception :" << ex.what() << endl;
    }
}

