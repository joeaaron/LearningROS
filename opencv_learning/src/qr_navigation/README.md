话题名称（暂定）: "/task_switch"
消息类型：std_msgs/Header
        uint32 seq
        time stamp
        string frame_id

seq:填写开关信息，0值表示停止，大于等于1的值表示开始
stamp：暂时不使用，有需要的时候再用
frame_id：填写node名称，用于指定哪个node来响应该topic；如果为空，所有订阅该topic的node都要响应

举例1：开始三角识别
seq = 1;
frame_id = "lighthouse_recognize";
三角识别节点lighthouse_recognize订阅到该topic后开始三角识别，其它节点不响应

举例2：停止磁条检测
seq = 0;
frame_id = "hinson_magnetic_driver";
磁条检测节点hinson_magnetic_driver订阅到该topic后停止磁条检测，其它节点不响应

举例3：
seq = 0;
frame_id = "";
所有订阅该topic的节点停止其任务的执行
