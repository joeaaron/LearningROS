#ifndef MAGNETIC_DRIVER
#define MAGNETIC_DRIVER

#include <ros/ros.h>
#include <fcntl.h>
#include <termios.h>

using namespace std;
class MagneticDriver
{
public:
    MagneticDriver();
    ~MagneticDriver();

    void Run();

private:
    ros::NodeHandle nh_;
    ros::Publisher magnetic_pub_;

    string serial_dev_;
    int serial_fd_;

    enum MagneticType
    {
        POINT_8,
        POINT_16
    };

    enum MagneticMode
    {
        POINT_IO,
        POINT_INTENSITY
    };

    MagneticType magnetic_type_;
    MagneticMode magnetic_mode_;

    int length_read_magnetic_;

    int length_read_magnetic8_io_reply_;
    int length_read_magnetic8_intensity_reply_;
    int length_read_magnetic16_io_reply_;
    int length_read_magnetic16_intensity_reply_;

    unsigned char buf_send_point_io_[7];
    unsigned char buf_send_point_intensity_[7];

    void InitBufSend();
    void SerialClose();
    int SerialInit(string serial_dev, int baud_rate, int data_bits, char parity, int stop_bits);
    unsigned int GetCrcCheck(unsigned char *cBuffer, unsigned int iBufLen);
    void GetMagneticIntensity(int length_read);
    void GetMagneticPointIo(int length_read);
};

#endif
