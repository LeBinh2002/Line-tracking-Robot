#include <Arduino.h>
#include <PID_v1.h>

// Định nghĩa các chân kết nối cảm biến
const int sensorLeft = A0;
const int sensorCenter = A1;
const int sensorRight = A2;

// Định nghĩa các chân điều khiển động cơ
const int motorLeftForward = 3;
const int motorLeftBackward = 5;
const int motorRightForward = 6;
const int motorRightBackward = 9;

// Biến PID
double setPoint, input, output;
double Kp = 1.0, Ki = 0.0, Kd = 0.0;

// Khởi tạo PID
PID myPID(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);

// Khai báo hàm motorControl trước
void motorControl(double pidOutput);

void setup() {
  // Khởi tạo các chân cảm biến
  pinMode(sensorLeft, INPUT);
  pinMode(sensorCenter, INPUT);
  pinMode(sensorRight, INPUT);

  // Khởi tạo các chân động cơ
  pinMode(motorLeftForward, OUTPUT);
  pinMode(motorLeftBackward, OUTPUT);
  pinMode(motorRightForward, OUTPUT);
  pinMode(motorRightBackward, OUTPUT);

  // Thiết lập setPoint cho PID
  setPoint = 0;

  // Kích hoạt PID
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  // Đọc giá trị cảm biến
  int sensorLeftValue = analogRead(sensorLeft);
  int sensorCenterValue = analogRead(sensorCenter); // Giá trị này hiện tại không được sử dụng, có thể bỏ qua hoặc sử dụng sau
  int sensorRightValue = analogRead(sensorRight);

  // Tính toán lỗi (error)
  input = (sensorLeftValue - sensorRightValue);

  // Tính toán đầu ra PID
  myPID.Compute();

  // Điều khiển động cơ dựa trên giá trị output của PID
  motorControl(output);
}

void motorControl(double pidOutput) {
  int baseSpeed = 150; // Tốc độ cơ bản của động cơ
  int leftSpeed = baseSpeed + pidOutput;
  int rightSpeed = baseSpeed - pidOutput;

  // Giới hạn giá trị tốc độ từ 0 đến 255
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Điều khiển động cơ trái
  if (leftSpeed >= 0) {
    analogWrite(motorLeftForward, leftSpeed);
    analogWrite(motorLeftBackward, 0);
  } else {
    analogWrite(motorLeftForward, 0);
    analogWrite(motorLeftBackward, -leftSpeed);
  }

  // Điều khiển động cơ phải
  if (rightSpeed >= 0) {
    analogWrite(motorRightForward, rightSpeed);
    analogWrite(motorRightBackward, 0);
  } else {
    analogWrite(motorRightForward, 0);
    analogWrite(motorRightBackward, -rightSpeed);
  }
}
