#include <dispatch_robot_comm/test_dispatch_robot_comm.h>

void TestDispatch::RunTest()
{
    DeserializedJsonTest();

    ros::Rate r(1);
    while (ros::ok())
    {
        ros::spinOnce();

        DoDispatchTask();

        r.sleep();
    }
}

void TestDispatch::DeserializedJsonTest()
{
    // std::string strValue = \
    // "{\"Dev_Type\":\"Core\",\
    //   \"Dev_ID\":\"1\",\
    //   \"Comm_ID\":101,\
    //   \"Process\":\"PT_CAMERAGRAIN\",\
    //   \"Move\":[\
    //             {\"Points_Num\":10},\
    //             {\"Points_Array\":[\
    //                                {\"X\":1},\
    //                                {\"Y\":1.56},\
    //                                {\"a\":90.0}\
    //                               ]\
    //             }\
    //            ]\
    //  }";

    // std::string strValue = \
    // "{\"Dev_Type\":\"Core\",\
    //   \"Dev_ID\":\"1\",\
    //   \"Comm_ID\":101,\
    //   \"Process\":\"PT_CAMERAGRAIN\",\
    //   \"Move\":[\
    //             {\"Points_Num\":10},\
    //             {\"Points_Array\":[\
    //                                {\"X\":1.0,\"Y\":1.5,\"a\":0.0},\
    //                                {\"X\":4.0,\"Y\":-1.5,\"a\":0.0}\
    //                               ]\
    //             }\
    //            ]\
    //  }";
 std::string strValue;// = \
// {
//    "Dev_Data" :
//    {
//       "Comm_ID" : 1,
//       "Move" :
//       {
//          "Points_Array" :
//          [
//             {
//                "ID" : 1,
//                "X" : 1.450000762939453,
//                "Y" : 0.250,
//                "a" : 0.0
//             },
//             {
//                "ID" : 2,
//                "X" : 0.50,
//                "Y" : 0.250,
//                "a" : 206.5650482177734
//             }
//          ],
//          "Points_Num" : 2
//       },
//       "Process" :
//       {
//          "Proc_Name" : ""
//       },
//       "Tar_ID" : 1,
//       "Time_stamp" : ""
//    },
//    "Dev_ID" : 1,
//    "Dev_Type" : "Core"
// }

    Json::Reader reader;
    Json::Value json_object;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dispatch_client_node");
    TestDispatch test_dispatch;

    test_dispatch.RunTest();
    return EXIT_SUCCESS;
}
