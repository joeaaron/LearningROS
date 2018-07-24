#include "findMarker.h"

using namespace cv;

int main()
{
	VideoCapture cap(1);

	while(1)
	{
		Mat frame;
		cap >> frame;

		if (!frame.empty())
			imshow("VideoRead", frame);
		else
			std::cout<<"error!";
		
		FindMarker *mark;
    	mark->ProcessFrame(frame);

		waitKey(1);
	}
 
  
    
    return 0;
}