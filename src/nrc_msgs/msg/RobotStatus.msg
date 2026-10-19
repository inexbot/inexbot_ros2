std_msgs/Header header

# 伺服状态: 0-停止 1-就绪 2-报警 3-运行
int32 servo_state

# 操作模式: 0-示教 1-远程 2-运行
int32 operation_mode

# 程序运行状态: 0-停止 1-暂停 2-运行
int32 program_state

# 控制器连接状态
bool connected
