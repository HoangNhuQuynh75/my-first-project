

#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define MIN_ABS_SPEED 20

MPU6050 mpu;

// MPU control/status vars
// Kiểm soát MPU / vars trạng thái
bool dmpReady = false; // set true if DMP init was successful đặt đúng nếu DMP init thành công
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU giữ byte trạng thái ngắt thực tế từ MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error) trạng thái trở lại sau mỗi hoạt động của thiết bị
uint16_t packetSize; // expected DMP packet size (default is 42 bytes) kích thước gói DMP dự kiến ​​(mặc định là 42 byte)
uint16_t fifoCount; // count of all bytes currently in FIFO số lượng tất cả các byte hiện tại trong FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer Bộ đệm lưu trữ FIFO

// orientation/motion vars
// vars định hướng / chuyển động
Quaternion q; // [w, x, y, z] quaternion container container tứ phương
VectorFloat gravity; // [x, y, z] gravity vector vector trọng lực
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector container và vector trọng lực

//PID
double originalSetpoint = 173;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;

//adjust these values to fit your own design
// điều chỉnh các giá trị này để phù hợp với thiết kế của riêng bạn
double Kp = 60;   
double Kd = 1.4 ;
double Ki = 70;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.6;
double motorSpeedFactorRight = 0.6;
//MOTOR CONTROLLER
int ENA = 5;
int IN1 = 6;
int IN2 = 7;
int IN3 = 8;
int IN4 = 9;
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
mpuInterrupt = true;
}


void setup()
{
// join I2C bus (I2Cdev library doesn't do this automatically)
// tham gia bus I2C (thư viện I2Cdev không tự động làm điều này)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
Wire.begin();
TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
Fastwire::setup(400, true);
#endif

mpu.initialize();

devStatus = mpu.dmpInitialize();

// supply your own gyro offsets here, scaled for min sensitivity
// cung cấp bù đắp con quay hồi chuyển của riêng bạn ở đây, được chia tỷ lệ cho độ nhạy tối thiểu
mpu.setXGyroOffset(220);
mpu.setYGyroOffset(76);
mpu.setZGyroOffset(-85);
mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

// make sure it worked (returns 0 if so)
if (devStatus == 0)
{
// turn on the DMP, now that it's ready
mpu.setDMPEnabled(true);

// enable Arduino interrupt detection
attachInterrupt(0, dmpDataReady, RISING);
mpuIntStatus = mpu.getIntStatus();

// set our DMP Ready flag so the main loop() function knows it's okay to use it
// đặt cờ DMP Ready của chúng tôi để hàm loop () chính biết sử dụng nó là ổn
dmpReady = true;

// get expected DMP packet size for later comparison
// lấy kích thước gói DMP dự kiến ​​để so sánh sau
packetSize = mpu.dmpGetFIFOPacketSize();

//setup PID
pid.SetMode(AUTOMATIC);
pid.SetSampleTime(10);
pid.SetOutputLimits(-255, 255); 
}
else
{
// ERROR!
// 1 = initial memory load failed
// 2 = DMP configuration updates failed
// (if it's going to break, usually the code will be 1)
Serial.print(F("DMP Initialization failed (code "));
Serial.print(devStatus);
Serial.println(F(")"));
}
}


void loop()
{
// if programming failed, don't try to do anything
// nếu lập trình thất bại, đừng cố làm gì cả
if (!dmpReady) return;

// wait for MPU interrupt or extra packet(s) available
// chờ MPU ngắt hoặc gói bổ sung có sẵn
while (!mpuInterrupt && fifoCount < packetSize)
{
//no mpu data - performing PID calculations and output to motors 
// không có dữ liệu mpu - thực hiện tính toán PID và xuất ra động cơ
pid.Compute();
motorController.move(output, MIN_ABS_SPEED);

}

// reset interrupt flag and get INT_STATUS byte
// đặt lại cờ ngắt và nhận byte INT_STATUS
mpuInterrupt = false;
mpuIntStatus = mpu.getIntStatus();

// get current FIFO count
// lấy số lượng FIFO hiện tại
fifoCount = mpu.getFIFOCount();

// check for overflow (this should never happen unless our code is too inefficient)
// kiểm tra lỗi tràn (điều này sẽ không bao giờ xảy ra trừ khi mã của chúng tôi quá kém hiệu quả)
if ((mpuIntStatus & 0x10) || fifoCount == 1024)
{
// reset so we can continue cleanly
// thiết lập lại để chúng ta có thể tiếp tục dọn dẹp
mpu.resetFIFO();
Serial.println(F("FIFO overflow!"));

// otherwise, check for DMP data ready interrupt (this should happen frequently)
// nếu không, hãy kiểm tra ngắt dữ liệu DMP (điều này sẽ xảy ra thường xuyên)
}
else if (mpuIntStatus & 0x02)
{
// wait for correct available data length, should be a VERY short wait
// chờ độ dài dữ liệu có sẵn chính xác, nên chờ RẤT ngắn
while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

// read a packet from FIFO
// đọc một gói từ FIFO
mpu.getFIFOBytes(fifoBuffer, packetSize);

// track FIFO count here in case there is > 1 packet available
// theo dõi số lượng FIFO ở đây trong trường hợp có> 1 gói có sẵn
// (this lets us immediately read more without waiting for an interrupt)
// (điều này cho phép chúng tôi ngay lập tức đọc thêm mà không cần chờ ngắt)
fifoCount -= packetSize;

mpu.dmpGetQuaternion(&q, fifoBuffer);
mpu.dmpGetGravity(&gravity, &q);
mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
input = ypr[1] * 180/M_PI + 180;
}
}

//PID Proportional Intergral Derivatice Là cơ chế điều khiển ( tỉ lệ, tích phân, đạo hàm ) 
