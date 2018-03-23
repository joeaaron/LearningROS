#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#define CAMERAMAT "CameraMat"
#define DISTCOEFF "DistCoeff"

using namespace cv;
using namespace std;

Point3f cloudPoint;
Point2f imgPoint;
vector<Point3f> cloudPoints;
vector<Point2f> imgPoints;

Mat R;
Mat T;
Mat camMat;
Mat distCoeff;

void estimatePose()
{
    Mat rvec, tvec,inliers;
    cv::solvePnPRansac( cloudPoints, imgPoints, camMat, cv::Mat(), rvec, tvec, false, 100, 1.0, 0.99, inliers );
    Rodrigues(rvec, R);
    cv::Mat Extrinc(3, 4, R.type()); // Extrinc is 4x4
    Extrinc( cv::Range(0,3), cv::Range(0,3) ) = R * 1; // copies R into Extrinc
    Extrinc( cv::Range(0,3), cv::Range(3,4) ) = tvec * 1; // copies tvec into Extrinc
    
    cout<<"R:"<<R<<endl;
    cout<<"t:"<<tvec <<endl;
    cout<<"Extrinc: "<<Extrinc << endl;

    string resultName = "../param/result.yml";
    FileStorage calibrate(resultName, FileStorage::WRITE);
    calibrate<< "CameraExtrinsicMat" << Extrinc;
    calibrate.release(); 
    
   // cout<<"inliers: "<<inliers<<endl;
}

int readPara(string filename)
{
    cv::FileStorage fs(filename,cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        std::cout<<"Invalid calibration filename."<<std::endl;
        return 0;
    }
    fs[CAMERAMAT]>>camMat;
    fs[DISTCOEFF]>>distCoeff;
}

int readData(string filename)
{
    ifstream imageCloudPoints;
    imageCloudPoints.open(filename);
    if(!imageCloudPoints)
    {
        std::cerr<<"can't open imageCloudPoints file!"<<std::endl;
        return 0;
    }
    char c;
    int i =0,j=0;
    char buf[50];
    float temp;

    while(imageCloudPoints.getline(buf,sizeof(buf)))
    {
        int i=0;
        char *subarr = strtok(buf," ");  //get string from arr[i] and use blank space as separator
        /*!
         * laset coordinate:x~left,y~front,z~up
         * camera coordinate:x~left,y~down,z~front
         * so we shoud align the two coordinate to before we get the calibtation matrix
         */
        while(subarr!=NULL){
            temp = atof(subarr);
            switch (i%5) {
            case 0:
                cloudPoint.x = temp;
                break;
            case 1:
                cloudPoint.z = temp;
                break;
            case 2:
                cloudPoint.y = -temp;
                break;
            case 3:
                imgPoint.x = temp;
                break;
            case 4:
                imgPoint.y = temp;
                break;
            default:
                break;
            }
            subarr = strtok(NULL," ");  //go on to get string from arr[i]
            i++;
        }
        cloudPoints.push_back(cloudPoint);
        imgPoints.push_back(imgPoint);
    }
}

int main(void)
{
    readPara("../param/calib.yml");
    readData("../imageCloudPoints.txt");
    estimatePose();
    return 0;
}
