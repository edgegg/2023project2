#include <TinyGPS.h>  // GPS 라이브러리
#include <L298NX2.h>  // 모터드라이버 라이브러리
#include <SoftwareSerial.h> // 소프트웨어 시리얼 통신 라이브러리
#include <TimerOne.h> // TimerOne 인터럽트 라이브러리
#include <Wire.h> // I2C 통신 라이브러리
#include <QMC5883LCompass.h>  // 지자기 센서 라이브러리

//---------지자기 센서 설정---------------------
QMC5883LCompass compass;

float declinationAngle = (-8.0 + (50.0 / 60.0)) / (180 / PI); // 자편각 보정

int xOffset = -1500;  // x축 방향 오차
int yOffset = 250;  // y축 방향 오차

//---------GPS 설정---------------------
TinyGPS gps;
SoftwareSerial gpsSerial(10, 11);  // RX, TX 핀 설정

//-----------모터 드라이버 핀 설정----------------

const unsigned int EN_A = 4;  //왼쪽 바퀴
const unsigned int IN1_A = 22;
const unsigned int IN2_A = 23;

const unsigned int IN1_B = 24; // 오른쪽 바퀴
const unsigned int IN2_B = 25;
const unsigned int EN_B = 5;

L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

//---------블루투스 설정---------------------

#define RXD_PIN 12 //아두이노 수신 핀
#define TXD_PIN 13 //아두이노 발신 핀

SoftwareSerial bt(RXD_PIN, TXD_PIN);  // Rx,Tx 설정 / 시리얼 통신 객체 생성

//---------전역 변수 설정---------------------
String data =""; // 문자열 받기
String data1="";
String data2="";
int spd = 50; // 속력
volatile bool GPS_flag = false;  // 10초마다 GPS값 읽었을 때 true
float heading; // 현재 방향
float target_angle; // 목표 방향
bool goal_flag = false; // 도착 플레그

struct Location{
  volatile float latitude; // 위도
  volatile float longitude;  // 경도
};

struct Location goal; // 목표지점 구조체

struct Location CurrentLocation; // 현재지점 구조체

struct Location FirstLocation;  // 처음 지점 구조체

//---------함수 설정---------------------
void MoveCar(int n) // 모터 동작 함수
{
  switch(n)
  {
    case 1: // 전진
      motors.setSpeed(spd);
      motors.forwardA();
      motors.forwardB();
      break;
    case 2: // 후진
      motors.setSpeed(spd);
      motors.backwardA();
      motors.backwardB();
      break;
    case 3: // 우회전(CW)
      motors.setSpeed(spd);
      motors.forwardA();
      motors.backwardB();
      break;
    case 4: // 좌회전(CCW)
      motors.setSpeed(spd);
      motors.backwardA();
      motors.forwardB();
      break;
    case 5: // 정지
      motors.stop();
      break;
    default:
      motors.stop();
      break;
  }
}

void splitString(String input, String &part1, String &part2)  // ','를 기준으로 문자열을 나누어주는 함수
{
  int delimiterIndex = input.indexOf(',');
  part1 = input.substring(0, delimiterIndex);
  part2 = input.substring(delimiterIndex + 1);
}

void readGPSData(float* latitude, float* longitude)  // 현재 위치정보를 읽어오는 함수
{
  bool newData = false;
  
  while(!newData)
  {
    gpsSerial.listen();
    while(gpsSerial.available())
    {
      char c = gpsSerial.read();
      if (gps.encode(c))
      {
        gps.f_get_position(latitude, longitude);
        newData = true;
        //break;
      }
    }
  }
}

void Compass()  // 현재 로봇의 방향 북쪽이 0도
{
  int x, y;
  
  compass.read();

  x = compass.getX() - xOffset;
  y = compass.getY() - yOffset;
  
  heading = atan2(y, x);
  
  heading += declinationAngle;

  if(heading < 0)
  {
    heading += 2 * PI;
  }
  if(heading > 2 * PI)
  {
    heading -= 2 * PI;
  }
  
  heading = heading * 180 / PI;
}

void gpsVector()  // 목표 위치 방향 계산 함수
{
  float x = goal.longitude - CurrentLocation.longitude; // 경도
  float y = goal.latitude - CurrentLocation.latitude; // 위도
  float c = sqrt((x*x)+(y*y));
  float angle = acos(x/c) * 180/PI;
  
  if((x>=0) && (y>=0))
  {
    target_angle = 90 - angle;
  }
  else if((x<=0) && (y>=0))
  {
    angle = 180 - angle;
    target_angle = 270 + angle;
  }
  else if((x<=0) && (y<=0))
  {
    angle = 180 - angle;
    target_angle = 270 - angle;
  }
  else if((x>=0) && (y<=0))
  {
    target_angle = 90 + angle;
  }
}

void Micro_Turn() // 미세 회전 함수
{
  float angle = target_angle - heading; // 각도의 차이
  if(angle > (float)0.0)
  {
    if(abs(angle) >= (float)180.0)
    {
      // CW
      MoveCar(3); // 우회전
    }
    else
    {
      // CCW
      MoveCar(4); // 좌회전
    }
  }
  else
  {
    if(abs(angle) >= (float)180.0)
    {
      // CCW
      MoveCar(4); // 좌회전
    }
    else
    {
      // CW
      MoveCar(3); // 우회전
    }
  }
  delay(200); // 0.2초 회전
  MoveCar(5); // 정지
  delay(300); // 0.3초 정지 딜레이
}

bool angle_Check(float Offset)
{
  return (abs(target_angle - heading) < Offset);
}

void Auto_Turn()  // 목표방향 향하게 회전해주는 함수
{
  bool complete_flag = false; // 방향조정 완료 플레그
  MoveCar(5); // 정지
  delay(500); // 정지 딜레이
  readGPSData(&CurrentLocation.latitude, &CurrentLocation.longitude); // 현재 위치 확인하는 함수
  gpsVector();  // 목표 위치 방향 계산 함수
  
  while(!complete_flag) // 방향 조정완료까지 무한 루프
  {
    Compass();  // 현재 로봇의 방향 확인하는 함수
    if(angle_Check(2.5))
    {
      complete_flag = true;  // 회전 완료
    }
    else Micro_Turn(); // 회전하는 함수
  }
  
  initializeTimer();  // 타이머 초기화
}

bool isGoal(float* g_lat, float* g_lon, float* c_lat, float* c_lon, float offset) // 도착했는지 확인하는 함수
{
  if(*g_lat > *c_lat - offset && *g_lat < *c_lat + offset && *g_lon > *c_lon - offset && *g_lon < *c_lon + offset) goal_flag = true;
  else goal_flag = false;
  
  return goal_flag;
}

void Adj_vector() // 타이머 인터럽트 함수
{
  GPS_flag = true;
}

void initializeTimer()  // 타이머 최기화
{
  Timer1.detachInterrupt(); // 이전에 연결된 인터럽트 제거
  Timer1.initialize(10000000); // 타이머 초기화 (10초마다)
  Timer1.attachInterrupt(Adj_vector); // 함수를 타이머 인터럽트에 연결
}

void button() // 버튼을 눌렀다 때면 목표 위치가 처음위치로 바뀌는 ISR
{
  goal.latitude = FirstLocation.latitude;
  goal.longitude = FirstLocation.longitude;
}

void setup() {
  pinMode(2, INPUT_PULLUP); // 버튼 핀설정
  attachInterrupt(0,button,RISING);  // 외부 인터럽트
  Serial.begin(9600); // 시리얼 통신 시작(실제 동작시 사용 안함)
  gpsSerial.begin(9600); // gps 시리얼 통신 시작
  bt.begin(9600); // bt 시리얼 통신 시작
  motors.setSpeed(spd); // 초기 속도 설정
  compass.init(); // 지자기 센서 시작
  MoveCar(5); // 정지

  // 블루투스로 도착지점 위도,경도 받기
  while(1)
  {
    bt.listen();
    if(bt.available())
    {
      data = bt.readStringUntil('e');   // e직전까지 읽기 data = '위도값,경도값'
      splitString(data, data1, data2);  // ',' 기준으로 나눈다.
      goal.latitude = data1.toFloat();  // 위도를 실수형으로
      goal.longitude = data2.toFloat();  // 경도를 실수형으로
      break;
    }
  }
  // 초기 위치 저장
  readGPSData(&FirstLocation.latitude, &FirstLocation.longitude);

  // 방향 조정
  Auto_Turn();

  // 타이머 초기화
  initializeTimer();
}

void loop()
{
  // 현재 위치를 읽어서 도착했는지 확인
  readGPSData(&CurrentLocation.latitude, &CurrentLocation.longitude);
  while(isGoal(&goal.latitude, &goal.longitude, &CurrentLocation.latitude, &CurrentLocation.longitude, 0.0002))
  {
    MoveCar(5); // 정지
  }
  
  if(GPS_flag)
  {
    Auto_Turn();  // 회전해서 방향 조정
    GPS_flag = false;
    delay(1000);  // 회전할 시간을 주기위해
  }
  if(!GPS_flag)
  {
    MoveCar(1); // 전진
    delay(1000);
  }
}