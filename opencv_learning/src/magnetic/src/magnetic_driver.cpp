#include <magnetic_driver.h>
#include <std_msgs/Int32MultiArray.h>
#include <bitset>

MagneticDriver::MagneticDriver()
    :nh_()
    ,magnetic_pub_()
    ,serial_dev_()
    ,serial_fd_()
    ,magnetic_type_()
    ,magnetic_mode_()
    ,length_read_magnetic_(7)
    ,length_read_magnetic8_io_reply_(8)          //接收数据：标识符（5Byte）+ 数据（1Byte） + 校验（2Byte） = 8Byte  /52 4D 67 73 77 + 数据（1Byte）  + 校验（2Byte）
    ,length_read_magnetic16_io_reply_(10)        //接收数据：标识符（5Byte）+ 数据（3Byte） + 校验（2Byte） = 10Byte /52 4D 67 73 77 + 数据（3Byte）  + 校验（2Byte）
    ,length_read_magnetic8_intensity_reply_(23)  //接收数据：标识符（5Byte）+ 数据（16Byte）+ 校验（2Byte） = 23Byte /52 4D 67 64 61 + 数据（16Byte） + 校验（2Byte）
    ,length_read_magnetic16_intensity_reply_(39) //接收数据：标识符（5Byte）+ 数据（32Byte）+ 校验（2Byte） = 39Byte /52 4D 67 64 61 + 数据（32Byte） + 校验（2Byte）
    ,buf_send_point_io_{0x52,0x4D,0x67,0x73,0x77,0x00,0x00} //8位和16位相同，发送数据：标识符（5Byte）+ 校验（2Byte） = 7Byte  /52 4D 67 73 77 /5E(低位) 69(高位)
    ,buf_send_point_intensity_{0x52,0x4D,0x67,0x64,0x61,0x00,0x00} //8位和16位相同，发送数据：标识符（5Byte）+ 校验（2Byte） = 7Byte  /52 4D 67 64 61 /0xA001
{
    magnetic_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/magnetic_head_data",1);

    string magnetic_type;
    string magnetic_mode;
    nh_.param( "serial_dev", serial_dev_, (string)"/dev/ttyUSB0" );
    nh_.param( "magnetic_type", magnetic_type, (string)"POINT_8" );
    nh_.param( "magnetic_mode", magnetic_mode, (string)"POINT_IO" );

    if ( magnetic_type == "POINT_8" )
    {
        magnetic_type_ = POINT_8;
    }
    else if ( magnetic_type == "POINT_16" )
    {
        magnetic_type_ = POINT_16;
    }
    else
    {
        ROS_ERROR("undefied magnetic type :%s", magnetic_type.c_str());
    }

    if ( magnetic_mode == "POINT_IO" )
    {
        magnetic_mode_ = POINT_IO;
    }
    else if ( magnetic_mode == "POINT_INTENSITY" )
    {
        magnetic_mode_ = POINT_INTENSITY;
    }
    else
    {
        ROS_ERROR("undefied magnetic mode :%s", magnetic_mode.c_str());
    }
}

MagneticDriver::~MagneticDriver()
{
}

void MagneticDriver::InitBufSend()
{
    unsigned int check_result = GetCrcCheck(buf_send_point_io_, length_read_magnetic_-2);

    buf_send_point_io_[length_read_magnetic_-2] = check_result & 0xFF;
    buf_send_point_io_[length_read_magnetic_-1] = (check_result>>8) & 0xFF;

    check_result = GetCrcCheck(buf_send_point_intensity_, length_read_magnetic_-2);

    buf_send_point_intensity_[length_read_magnetic_-2] = check_result & 0xFF;
    buf_send_point_intensity_[length_read_magnetic_-1] = (check_result>>8) & 0xFF;
}
   
void MagneticDriver::SerialClose()
{
    close(serial_fd_);
}

int MagneticDriver::SerialInit(string serial_dev, int baud_rate, int data_bits, char parity, int stop_bits)
{
    int fd = open(serial_dev.c_str(), O_RDWR|O_NOCTTY|O_NDELAY);
    if (fd < 0)
    {
        printf("failure: open magnetic serial port %s, error message: %s\n", serial_dev.c_str(), strerror(errno));
        return fd;
    }
    else if (fcntl(fd, F_SETFL, 0) < 0)
    {
        printf("failure: set fcntl file %s\n", serial_dev.c_str());
        close(fd);
        return -1;
    }

    struct termios newtio, oldtio;
    if (0 != tcgetattr(fd, &oldtio))
    {
        printf("failure: get attr %s\n", serial_dev.c_str());
        close(fd);
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch (baud_rate)
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 19200:
        cfsetispeed(&newtio, B19200);
        cfsetospeed(&newtio, B19200);
        break;
    case 38400:
        cfsetispeed(&newtio, B38400);
        cfsetospeed(&newtio, B38400);
        break;
    case 57600:
        cfsetispeed(&newtio, B57600);
        cfsetospeed(&newtio, B57600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    default:
        printf("failure: set %s baud rate %d\n", serial_dev.c_str(), baud_rate);
        close(fd);
        return -1;
    }

    switch (data_bits)
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    default:
        printf("failure: set %s data bits %d\n", serial_dev.c_str(), data_bits);
        close(fd);
        return -1;
    }

    switch (parity)
    {
    case 'O':                     //奇校验
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':                     //偶校验
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':                     //无校验
        newtio.c_cflag &= ~PARENB;
        break;
    default:
        printf("failure: set %s parity %c\n", serial_dev.c_str(), parity);
        close(fd);
        return -1;
    }

    switch (stop_bits)
    {
    case 1:
        newtio.c_cflag &=  ~CSTOPB;
        break;
    case 2:
        newtio.c_cflag |=  CSTOPB;
        break;
    default:
        printf("failure: set %s stop bits %d\n", serial_dev.c_str(), stop_bits);
        close(fd);
        return -1;
    }

    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    if (0 != (tcsetattr(fd, TCSANOW, &newtio)))
    {
        printf("failure: set attr %s\n", serial_dev.c_str());
        close(fd);
        return -1;
    }

    tcflush(fd, TCIFLUSH);
//    printf("success: setup serial port %s", tty_name.c_str());

    return fd;
}

unsigned int MagneticDriver::GetCrcCheck(unsigned char *buffer, unsigned int buffer_length)
{
    unsigned int i, j;
    unsigned int wCrc = 0xffff;
    unsigned int wPolynom = 0xA001;
    for (i = 0; i < buffer_length; i++)
    {
        wCrc ^= buffer[i];
        for (j = 0; j < 8; j++)
        {
            if (wCrc &0x0001)
            {
                wCrc = (wCrc >> 1) ^ wPolynom;
            }
            else
            {
                wCrc = wCrc >> 1;
            }
        }
    }
    return wCrc;
}

//用强度数据用来确定磁条位置不可行
void MagneticDriver::GetMagneticIntensity(int length_read)
{
    char buf_recv[length_read];

    ros::Rate r(1);

    while (ros::ok())
    {
        int length_real_send = write(serial_fd_, buf_send_point_intensity_, length_read_magnetic_);
        if ( length_real_send != length_read_magnetic_ )
        {
            ROS_ERROR("write data failed");
            tcflush(serial_fd_, TCOFLUSH);
            r.sleep();
            continue;
        }

        ros::Duration(0.02).sleep();
        std_msgs::Int32MultiArray magnetic_data_msg;//存放16个检测点的检测结果

        memset(buf_recv, 0, sizeof(buf_recv));

        int length_real_read = read(serial_fd_, buf_recv, length_read);
        cout << "-----------------------------length_real_read: " << length_real_read << endl;
        if(length_real_read == length_read)
        {
            for ( int i = 5; i < length_read-3; i = i+2 )
            {
                magnetic_data_msg.data.push_back( (buf_recv[i]<<8)|(buf_recv[i+1]) );
            }
            for ( int i = 0; i < 8; ++i )
            {
                printf("intensity result: %d is %d\n", i, magnetic_data_msg.data[i]);
            }
            magnetic_pub_.publish(magnetic_data_msg);
        }
        else
        {
            ROS_ERROR("read data failed");
            tcflush(serial_fd_, TCOFLUSH);
        }
        r.sleep();
    }
}

void MagneticDriver::GetMagneticPointIo(int length_read)
{
    unsigned char buf_recv[length_read];

    ros::Rate r(20);

    while (ros::ok())
    {
//        for (int i=0; i<length_read_magnetic_; ++i)
//        {
//            printf("send %d, %x\n", i, buf_send_point_io_[i]);
//        }
        int length_real_send = write(serial_fd_, buf_send_point_io_, length_read_magnetic_);
        if ( length_real_send != length_read_magnetic_ )
        {
            ROS_ERROR("write data failed");
            tcflush(serial_fd_, TCOFLUSH);
            r.sleep();
            continue;
        }

        std_msgs::Int32MultiArray magnetic_data_msg;//存放16个检测点的检测结果

        memset(buf_recv, 0, sizeof(buf_recv));

        ros::Duration(0.02).sleep();
        int length_real_read = read(serial_fd_, buf_recv, 2*length_read);
//        for (int i=0; i<length_read_magnetic8_io_reply_; ++i)
//        {
//            printf("recv %d, %x\n", i, buf_recv[i]);
//        }
        if( length_real_read == length_read )
        {
            int result_byte = length_read - 5 - 2; //5符号，2校验

            if ( result_byte == 1 )
            {
                std::bitset<8> bit_0_7 = buf_recv[5];
                for ( int i = 0; i < 8; ++i )
                {
                    magnetic_data_msg.data.push_back(bit_0_7[i]);
                }
            }
            else
            {
                std::bitset<8> bit_0_7 = buf_recv[5];//buf_recv[5]第1~7位有效,第0位无效
                std::bitset<8> bit_8_15 = buf_recv[6];//buf_recv[6]第0~7位有效
                std::bitset<8> bit_16 = buf_recv[7];//buf_recv[7]第0位有效

                for ( int i = 0; i < 17; ++i )
                {
                    if ( (i > 0) && (i < 8) ) //i>0,buf_recv[5]第0位无效
                    {
                        magnetic_data_msg.data.push_back(bit_0_7[i]);
                    }
                    else if ( i < 16 )
                    {
                        magnetic_data_msg.data.push_back(bit_8_15[i-8]);
                    }
                    else
                    {
                        magnetic_data_msg.data.push_back(bit_16[0]);
                    }
                }
            }

            magnetic_pub_.publish(magnetic_data_msg);
        }
        else
        {
            ROS_ERROR("read data failed, %d", length_real_read);
            tcflush(serial_fd_, TCOFLUSH);
        }
        r.sleep();
    }
    SerialClose();
}

void MagneticDriver::Run()
{
    serial_fd_ = SerialInit(serial_dev_, 115200, 8, 'N', 1);
    while ( serial_fd_ <= 0 && ros::ok() )
    {
        SerialClose();
        ros::Duration(1.0).sleep();
        serial_fd_ = SerialInit(serial_dev_, 115200, 8, 'N', 1);
    }

    InitBufSend();

    int length_read = 0;

    if ( magnetic_mode_ == POINT_IO )
    {
        if ( magnetic_type_ == POINT_8 )
        {
            length_read = length_read_magnetic8_io_reply_;
        }
        else if ( magnetic_type_ == POINT_16 )
        {
            length_read = length_read_magnetic16_io_reply_;
        }
        else
        {
            ROS_ERROR("undefied magnetic type :%d", magnetic_type_);
            SerialClose();
            return;
        }
        GetMagneticPointIo(length_read);
    }
    else if ( magnetic_mode_ == POINT_INTENSITY )
    {
        if ( magnetic_type_ == POINT_8 )
        {
            length_read = length_read_magnetic8_intensity_reply_;
        }
        else if ( magnetic_type_ == POINT_16 )
        {
            length_read = length_read_magnetic16_intensity_reply_;
        }
        else
        {
            ROS_ERROR("undefied magnetic type :%d", magnetic_type_);
            SerialClose();
            return;
        }
        GetMagneticIntensity(length_read);
    }
    else
    {
        ROS_ERROR("undefied magnetic mode :%d", magnetic_mode_);
        SerialClose();
        return;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "magnetic_driver");
    MagneticDriver magnetic_driver;
    magnetic_driver.Run();
    return EXIT_SUCCESS;
}
