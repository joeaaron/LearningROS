#ifndef TEST_DISPATCH_ROBOT_COMM
#define TEST_DISPATCH_ROBOT_COMM


#include "dispatch_robot_comm.h"

class TestDispatch:public Dispatch
{
public:
    void RunTest();
    void DeserializedJsonTest();
};

#endif
