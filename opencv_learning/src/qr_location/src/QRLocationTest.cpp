#include "QRLocation.h"
#include <stdio.h>

int main()
{
    QRLocation qrLoc;
    if(!qrLoc.init(1,0.60,true))
        return 1;
    QRPose_t pose;
    while(true)
    {
        if(qrLoc.getQRPose(&pose))
        {
            double aInDegree=pose.a*180/3.1415;
            double bInDegree=pose.b*180/3.1415;
            printf("a=%.2lf,b=%.2lf,z=%.2lf\n",aInDegree,bInDegree,pose.z);
        }
    }
}
