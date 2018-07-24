#include "zbarScanner.h"


ImageConverter::ImageConverter(ros::NodeHandle nh)
	: it(nh)
{
	
    //使用image_transport订阅图像话题“in” 和 发布图像话题“out” /camera/rgb/image_raw
    image_sub=it.subscribe("/usb_cam/image_raw",1,&ImageConverter::imageCb,this);
    image_pub=it.advertise("zbar_opencv",1);

}

void ImageConverter::zbarscanner(cv_bridge::CvImagePtr cv_ptr)
{
	//Create a zbar reader
	ImageScanner scanner;
	//Config the reader
	scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

	Mat frame, frame_grayscale;
	frame = cv_ptr->image;
	cvtColor(frame, frame_grayscale, CV_BGR2GRAY);

	int width = frame_grayscale.cols;
	int height = frame_grayscale.rows;
	uchar *raw = (uchar*)(frame_grayscale.data);

	//Wrap image data
	Image image(width, height, "Y800", raw, width*height);

	//scan the image for barcodes
	scanner.scan(image);
	//Extract results
	int  counter = 0;
	for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++ symbol)
	{
		cout << symbol->get_data() << endl;
		//Draw location of the symbols found 
		if (symbol->get_location_size() == 4)
		{
			
			line(frame, Point(symbol->get_location_x(0), symbol->get_location_y(0)), Point(symbol->get_location_x(1), symbol->get_location_y(1)), Scalar(0, 255, 0), 2, 8, 0);
			line(frame, Point(symbol->get_location_x(1), symbol->get_location_y(1)), Point(symbol->get_location_x(2), symbol->get_location_y(2)), Scalar(0, 255, 0), 2, 8, 0);
			line(frame, Point(symbol->get_location_x(2), symbol->get_location_y(2)), Point(symbol->get_location_x(3), symbol->get_location_y(3)), Scalar(0, 255, 0), 2, 8, 0);
			line(frame, Point(symbol->get_location_x(3), symbol->get_location_y(3)), Point(symbol->get_location_x(0), symbol->get_location_y(0)), Scalar(0, 255, 0), 2, 8, 0);
		}
		counter++;
	}
}

//订阅回调函数
void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //将ROS图像消息转化为适合Opencv的CvImage
        cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);

    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return;
    }
    //梯度运算
    //cv_ptr=gradient(cv_ptr);
    //水平投影法
    // projection(cv_ptr);
    zbarscanner(cv_ptr);
    // printf("OK1\n");
    image_pub.publish(cv_ptr->toImageMsg());
}
