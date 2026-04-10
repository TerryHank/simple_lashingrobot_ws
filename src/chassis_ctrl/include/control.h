#define LIGHT 5084
bool light_state =0;
ros::Publisher pub;
ros::Publisher pub_linear_module_data_upload;
chassis_ctrl::motion transform_msg;
chassis_ctrl::linear_module_upload linear_module_data_upload;
new_camera::ProcessImage srv;
ros::ServiceClient client;
// 以下为向PLC发送指令时写入的地址偏移量
#define WX_SPEED 5050 // 写入三轴速度和坐标
#define WY_SPEED 5054
#define WZ_SPEED 5058
#define WX_COORDINATE 5062
#define WY_COORDINATE 5066
#define WZ_COORDINATE 5070
#define MODULE_STOP 5075 // 置0x0001时停止x轴运动，置0x0002时停止y轴运动，置0x0004时停止z轴运动，置0x0008时停止三轴运动
#define EN_DISABLE 5076 // 置1时使能模组运动，绑扎枪打开；置0时模组失能，绑扎枪关闭
#define WARNING_RESET 5077 // 故障复位
#define LASHING 5078 // 执行绑扎动作
#define WRITING_ANGLE 5080 // 写入旋转电机目标角度（°）
#define WRITING_RMOTOR_SPEED 5082// 写入电机旋转速度（°/s）
#define EN_DISABLE_RMOTOR 5087 // 置1时失能旋转电机，置0时使能旋转电机

#define RESET_RMOTOR 5088 // 置1时旋转电机报警时复位，需要手动清零


// 以下为读取PLC状态的地址偏移量
#define RX_SPEED 5150 // 读取三轴速度和坐标
#define RY_SPEED 5154
#define RZ_SPEED 5158
#define RX_COORDINATE 5162  
#define RY_COORDINATE 5166
#define RZ_COORDINATE 5170
#define IS_ERROR 5174 // 读取到1时代表设备正常
#define ERROR_INQUIRE 5175 // bit0=1代表x轴出现异常，bit1=1代表y轴出现异常，bit2=1代表z轴出现异常
#define MODULE_STATUS 5176 // 读取到1时代表设备处于远程控制状态，读取到0代表设备正处于显示屏操作状态
#define EMERGENCY_STOP 5177 // 读取到1时代表模组处于紧急停止状态
#define READING_ANGLE 5178 // 读取旋转电机当前角度（°）
#define READING_RMOTOR_SPEED 5182// 读取电机旋转速度（°/s）

#define BATTERY_VOLTAGE 5187 // 电池电压
#define INNER_TEM 5191 // 机器人内腔温度
#define X_GESTURE 5198
#define Y_GESTURE 5196

#define AXIS_X 0
#define AXIS_Y 3
#define AXIS_Z 4
#define AXIS_MOTOR 5

// 全局变量，modbust_t 套接字
modbus_t* plc;
double left_motor_speed = 0;
double right_motor_speed = 0;
double gain_speed = 0;
int channel1_percent=0;
int channel2_percent=0;
int channel3_percent=0;
int state = 0;
int fd = -1;
uint8_t data;
ros::Publisher pub_ctl_vel;
ros::Publisher pub_wheel_direction;
std_msgs::Int32 msg_state;
std_msgs::Float32 msg_vel;
// 用于存储浮点数转化成的2个uint16_t数据
uint16_t UtoF_Register[2];
uint16_t DtoU_Register[2];
// 数组中存储2个uint16_t数据转化为浮点数
uint16_t UtoD_Register[2];

// 线性模组的运行速度
double module_speed= 250;
// 旋转电机运行速度
double motor_speed= 50;
std_msgs::Int32 msg;
// 旋转电机默认角度
double reset_angle =0;
static double last_left_motor_speed = 0;
static double last_right_motor_speed = 0;
// 子区域绑扎结束的标志位
ros::Publisher pub_lashing_finish;
// 电机角度发布者
ros::Publisher pub_motor_theata;
// 线性模组返回原点的标志位
ros::Publisher pub_linear_module_gb_origin;
// 线性模组节点急停信号发布者
ros::Publisher pub_forced_stop;

ros::Publisher pub_pause;
// 暂停中断标志位 0为未启用暂停中断或恢复，1为启用暂停中断
int pause_interrupt = 0;

// 定义互斥锁来保护 module_state 的读写操作
std::mutex module_state_mutex;
std::mutex plc_mutex;

// 线性模组状态的结构体
typedef struct {
    double X;
    double Y;
    double Z;
    double X_SPEED;
    double Y_SPEED;
    double Z_SPEED;
    uint16_t ERROR_FLAG_X;
    uint16_t ERROR_FLAG_Y;
    uint16_t ERROR_FLAG_Z;
    uint16_t ERROR_FLAG_LASHING;
    float x_gesture;
    float y_gesture;
} Module_State;
Module_State module_state;

// 旋转电机状态的结构体
typedef struct {
    double MOTOR_ANGLE;
    double MOTOR_SPEED;
    uint16_t ERROR_FLAG_MOTOR;
} Motor_State;
Motor_State motor_state;

bool enable_lashing=false;
std::mutex lashing_mutex;  // 新增互斥锁
ros::Subscriber enb_las_sub;  // 新增订阅者

// 在全局变量部分添加计数器和发布者
int pause_interrupt_count = 0; // 记录暂停信号的次数
ros::Publisher pub_pause_interrupt_count; // 发布者，用于发送故障次数