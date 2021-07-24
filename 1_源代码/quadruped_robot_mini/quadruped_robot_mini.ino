//Wi-Fi AP模式，手机链接"dachou_robot"热点（密码：55588888）后使用UDP遥控器APP遥控机器人。
//感谢jasonleung8866的3D打印机身图纸 下载地址 https://www.thingiverse.com/thing:4149143
//代码修改为esp8266 Wi-Fi模块直接控制无需其他外围设备
//共享代码切勿商用，转载请保留以上文字！

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Servo.h>
#include <Ticker.h>  //定时器中断库
//----网络部分---
unsigned int localPort = 6666;      // 可以自定义端口号
char packetBuffer[UDP_TX_PACKET_MAX_SIZE + 1]; 
char  ReplyBuffer[] = "acknowledged\r\n";       
WiFiUDP Udp;
uint16_t lx,ly,rx,ry;
uint8_t sit_mode;
uint8_t cmd = 16;

const int numberOfServos = 8; // Number of servos
const int numberOfACE = 9; // Number of action code elements
int servoCal[] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // Servo calibration data
int servoPos[] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // Servo current position
int servoPrevPrg[] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // Servo previous prg
int servoPrgPeriod = 20; // 50 ms
Servo servo[numberOfServos]; // Servo object

// Servo zero position
int servoAct00 [] PROGMEM =
  // P10, P11, P12, P16, P02, P04, P07, P15
{  135,  45, 135,  45,  45, 135,  45, 135 };

// Zero
int servoPrg00step = 1;
int servoPrg00 [][numberOfACE] PROGMEM = {
  // P10, P11, P12, P16, P02, P04, P07, P15,  ms
  {  135,  45, 135,  45,  45, 135,  45, 135, 1000  }, // zero position
};

// Standby
int servoPrg01step = 2;
int servoPrg01 [][numberOfACE] PROGMEM = {
  // P10, P11, P12, P16, P02, P04, P07, P15,  ms
  {   90,  90,  90,  90,  90,  90,  90,  90,  200  }, // prep standby
  {  -20,   0,   0,  20,  20,   0,   0, -20,  200  }, // standby
};

// Forward
int servoPrg02step = 11;
int servoPrg02 [][numberOfACE] PROGMEM = {
  // P10, P11, P12, P16, P02, P04, P07, P15,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {   20,   0,   0,   0,   0,   0, -45,  20,  100  }, // leg1,4 up; leg4 fw
  {  -20,   0,   0,   0,   0,   0,   0, -20,  100  }, // leg1,4 dn
  {    0,   0,   0, -20, -20,   0,   0,   0,  100  }, // leg2,3 up
  {    0, -45,  45,   0,   0,   0,  45,   0,  100  }, // leg1,4 bk; leg2 fw
  {    0,   0,   0,  20,  20,   0,   0,   0,  100  }, // leg2,3 dn
  {   20,  45,   0,   0,   0,   0,   0,  20,  100  }, // leg1,4 up; leg1 fw
  {    0,   0, -45,   0,   0,  45,   0,   0,  100  }, // leg2,3 bk
  {  -20,   0,   0,   0,   0,   0,   0, -20,  100  }, // leg1,4 dn
  {    0,   0,   0,   0, -20,   0,   0,   0,  100  }, // leg3 up
  {    0,   0,   0,   0,  20, -45,   0,   0,  100  }, // leg3 fw dn
};

// Backward
int servoPrg03step = 11;
int servoPrg03 [][numberOfACE] PROGMEM = {
  // P10, P11, P12, P16, P02, P04, P07, P15,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {   20, -45,   0,   0,   0,   0,   0,  20,  100  }, // leg4,1 up; leg1 fw
  {  -20,   0,   0,   0,   0,   0,   0, -20,  100  }, // leg4,1 dn
  {    0,   0,   0, -20, -20,   0,   0,   0,  100  }, // leg3,2 up
  {    0,  45,   0,   0,   0,  45, -45,   0,  100  }, // leg4,1 bk; leg3 fw
  {    0,   0,   0,  20,  20,   0,   0,   0,  100  }, // leg3,2 dn
  {   20,   0,   0,   0,   0,   0,  45,  20,  100  }, // leg4,1 up; leg4 fw
  {    0,   0,  45,   0,   0, -45,   0,   0,  100  }, // leg3,1 bk
  {  -20,   0,   0,   0,   0,   0,   0, -20,  100  }, // leg4,1 dn
  {    0,   0,   0, -20,   0,   0,   0,   0,  100  }, // leg2 up
  {    0,   0, -45,  20,   0,   0,   0,   0,  100  }, // leg2 fw dn
};

// Move Left
int servoPrg04step = 11;
int servoPrg04 [][numberOfACE] PROGMEM = {
  // P10, P11, P12, P16, P02, P04, P07, P15,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {    0,   0, -45, -20, -20,   0,   0,   0,  100  }, // leg3,2 up; leg2 fw
  {    0,   0,   0,  20,  20,   0,   0,   0,  100  }, // leg3,2 dn
  {   20,   0,   0,   0,   0,   0,   0,  20,  100  }, // leg1,4 up
  {    0,  45,  45,   0,   0, -45,   0,   0,  100  }, // leg3,2 bk; leg1 fw
  {  -20,   0,   0,   0,   0,   0,   0, -20,  100  }, // leg1,4 dn
  {    0,   0,   0, -20, -20,  45,   0,   0,  100  }, // leg3,2 up; leg3 fw
  {    0, -45,   0,   0,   0,   0,  45,   0,  100  }, // leg1,4 bk
  {    0,   0,   0,  20,  20,   0,   0,   0,  100  }, // leg3,2 dn
  {    0,   0,   0,   0,   0,   0,   0,  20,  100  }, // leg4 up
  {    0,   0,   0,   0,   0,   0, -45, -20,  100  }, // leg4 fw dn
};

// Move Right
int servoPrg05step = 11;
int servoPrg05 [][numberOfACE] PROGMEM = {
  // P10, P11, P12, P16, P02, P04, P07, P15,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {    0,   0,   0, -20, -20, -45,   0,   0,  100  }, // leg2,3 up; leg3 fw
  {    0,   0,   0,  20,  20,   0,   0,   0,  100  }, // leg2,3 dn
  {   20,   0,   0,   0,   0,   0,   0,  20,  100  }, // leg4,1 up
  {    0,   0, -45,   0,   0,  45,  45,   0,  100  }, // leg2,3 bk; leg4 fw
  {  -20,   0,   0,   0,   0,   0,   0, -20,  100  }, // leg4,1 dn
  {    0,   0,  45, -20, -20,   0,   0,   0,  100  }, // leg2,3 up; leg2 fw
  {    0,  45,   0,   0,   0,   0, -45,   0,  100  }, // leg4,1 bk
  {    0,   0,   0,  20,  20,   0,   0,   0,  100  }, // leg2,3 dn
  {   20,   0,   0,   0,   0,   0,   0,   0,  100  }, // leg1 up
  {  -20, -45,   0,   0,   0,   0,   0,   0,  100  }, // leg1 fw dn
};

// Turn left
int servoPrg06step = 8;
int servoPrg06 [][numberOfACE] PROGMEM = {
  // P10, P11, P12, P16, P02, P04, P07, P15,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {   20,   0,   0,   0,   0,   0,   0,  20,  100  }, // leg1,4 up
  {    0,  45,   0,   0,   0,   0,  45,   0,  100  }, // leg1,4 turn
  {  -20,   0,   0,   0,   0,   0,   0, -20,  100  }, // leg1,4 dn
  {    0,   0,   0, -20, -20,   0,   0,   0,  100  }, // leg2,3 up
  {    0,   0,  45,   0,   0,  45,   0,   0,  100  }, // leg2,3 turn
  {    0,   0,   0,  20,  20,   0,   0,   0,  100  }, // leg2,3 dn
  {    0, -45, -45,   0,   0, -45, -45,   0,  100  }, // leg1,2,3,4 turn
};

// Turn right
int servoPrg07step = 8;
int servoPrg07 [][numberOfACE] PROGMEM = {
  // P10, P11, P12, P16, P02, P04, P07, P15,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {    0,   0,   0, -20, -20,   0,   0,   0,  100  }, // leg2,3 up
  {    0,   0, -45,   0,   0, -45,   0,   0,  100  }, // leg2,3 turn
  {    0,   0,   0,  20,  20,   0,   0,   0,  100  }, // leg2,3 dn
  {   20,   0,   0,   0,   0,   0,   0,  20,  100  }, // leg1,4 up
  {    0, -45,   0,   0,   0,   0, -45,   0,  100  }, // leg1,4 turn
  {  -20,   0,   0,   0,   0,   0,   0, -20,  100  }, // leg1,4 dn
  {    0,  45,  45,   0,   0,  45,  45,   0,  100  }, // leg1,2,3,4 turn
};

// Lie
int servoPrg08step = 1;
int servoPrg08 [][numberOfACE] PROGMEM = {
  // P10, P11, P12, P16, P02, P04, P07, P15,  ms
  {  110,  90,  90,  70,  70,  90,  90, 110,  500  }, // leg1,4 up
};

// Say Hi
int servoPrg09step = 4;
int servoPrg09 [][numberOfACE] PROGMEM = {
  // P10, P11, P12, P16, P02, P04, P07, P15,  ms
  {  120,  90,  90, 110,  60,  90,  90,  70,  200  }, // leg1, 3 down
  {  -50,   0,   0,   0,  50,   0,   0,   0,  200  }, // standby
  {   50,   0,   0,   0, -50,   0,   0,   0,  200  }, // leg1, 3 down
  {  -50,   0,   0,   0,  50,   0,   0,   0,  200  }, // standby
};

// Fighting
int servoPrg10step = 11;
int servoPrg10 [][numberOfACE] PROGMEM = {
  // P10, P11, P12, P16, P02, P04, P07, P15,  ms
  {  120,  90,  90, 110,  60,  90,  90,  70,  200  }, // leg1, 2 down
  {    0, -20, -20,   0,   0, -20, -20,   0,  200  }, // body turn left
  {    0,  40,  40,   0,   0,  40,  40,   0,  200  }, // body turn right
  {    0, -40, -40,   0,   0, -40, -40,   0,  200  }, // body turn left
  {    0,  40,  40,   0,   0,  40,  40,   0,  200  }, // body turn right
  {  -50, -20, -20, -40,  50, -20, -20,  40,  200  }, // leg1, 2 up ; leg3, 4 down
  {    0, -20, -20,   0,   0, -20, -20,   0,  200  }, // body turn left
  {    0,  40,  40,   0,   0,  40,  40,   0,  200  }, // body turn right
  {    0, -40, -40,   0,   0, -40, -40,   0,  200  }, // body turn left
  {    0,  40,  40,   0,   0,  40,  40,   0,  200  }, // body turn right
  {    0, -20, -20,   0,   0, -20, -20,   0,  200  }, // leg1, 2 up ; leg3, 4 down
};

// Push up
int servoPrg11step = 11;
int servoPrg11 [][numberOfACE] PROGMEM = {
  // P10, P11, P12, P16, P02, P04, P07, P15,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  300  }, // start
  {   30,   0,   0, -30, -30,   0,   0,  30,  400  }, // down
  {  -30,   0,   0,  30,  30,   0,   0, -30,  500  }, // up
  {   30,   0,   0, -30, -30,   0,   0,  30,  600  }, // down
  {  -30,   0,   0,  30,  30,   0,   0, -30,  700  }, // up
  {   30,   0,   0, -30, -30,   0,   0,  30,  1300 }, // down
  {  -30,   0,   0,  30,  30,   0,   0, -30,  1800 }, // up
  {   65,   0,   0, -65, -65,   0,   0,  65,  200  }, // fast down
  {  -65,   0,   0,   0,  15,   0,   0,   0,  500  }, // leg1 up
  {    0,   0,   0,   0,  50,   0,   0,   0,  500  }, // leg2 up
  {    0,   0,   0,  65,   0,   0,   0, -65,  500  }, // leg3, leg4 up
};

// Sleep
int servoPrg12step = 2;
int servoPrg12 [][numberOfACE] PROGMEM = {
  // P10, P11, P12, P16, P02, P04, P07, P15,  ms
  {   35,  90,  90, 145, 145,  90,  90,  35,  400  }, // leg1,4 dn
  {    0, -45,  45,   0,   0,  45, -45,   0,  400  }, // protect myself
};

// Dancing 1
int servoPrg13step = 10;
int servoPrg13 [][numberOfACE] PROGMEM = {
  // P10, P11, P12, P16, P02, P04, P07, P15,  ms
  {   90,  90,  90,  90,  90,  90,  90,  90,  300  }, // leg1,2,3,4 up
  {  -40,   0,   0,   0,   0,   0,   0,   0,  300  }, // leg1 dn
  {   40,   0,   0,  40,   0,   0,   0,   0,  300  }, // leg1 up; leg2 dn
  {    0,   0,   0, -40,   0,   0,   0, -40,  300  }, // leg2 up; leg4 dn
  {    0,   0,   0,   0,  40,   0,   0,  40,  300  }, // leg4 up; leg3 dn
  {  -40,   0,   0,   0, -40,   0,   0,   0,  300  }, // leg3 up; leg1 dn
  {   40,   0,   0,  40,   0,   0,   0,   0,  300  }, // leg1 up; leg2 dn
  {    0,   0,   0, -40,   0,   0,   0, -40,  300  }, // leg2 up; leg4 dn
  {    0,   0,   0,   0,  40,   0,   0,  40,  300  }, // leg4 up; leg3 dn
  {    0,   0,   0,   0, -40,   0,   0,   0,  300  }, // leg3 up
};

// Dancing 2
int servoPrg14step = 9;
int servoPrg14 [][numberOfACE] PROGMEM = {
  // P10, P11, P12, P16, P02, P04, P07, P15,  ms
  {   70,  45, 135, 110, 110, 135,  45,  70,  300  }, // leg1,2,3,4 two sides
  {   45,   0,   0, -45,   0,   0,   0,   0,  300  }, // leg1,2 up
  {  -45,   0,   0,  45, -45,   0,   0,  45,  300  }, // leg1,2 dn; leg3,4 up
  {   45,   0,   0, -45,  45,   0,   0, -45,  300  }, // leg3,4 dn; leg1,2 up
  {  -45,   0,   0,  45, -45,   0,   0,  45,  300  }, // leg1,2 dn; leg3,4 up
  {   45,   0,   0, -45,  45,   0,   0, -45,  300  }, // leg3,4 dn; leg1,2 up
  {  -45,   0,   0,  45, -45,   0,   0,  45,  300  }, // leg1,2 dn; leg3,4 up
  {   45,   0,   0, -45,  45,   0,   0, -45,  300  }, // leg3,4 dn; leg1,2 up
  {  -40,   0,   0,  40,   0,   0,   0,   0,  300  }, // leg1,2 dn
};

// Dancing 3
int servoPrg15step = 10;
int servoPrg15 [][numberOfACE] PROGMEM = {
  // P10, P11, P12, P16, P02, P04, P07, P15,  ms
  {   70,  45,  45, 110, 110, 135, 135,  70,  300  }, // leg1,2,3,4 bk
  {   40,   0,   0, -50, -40,   0,   0,   0,  300  }, // leg1,2,3 up
  {  -40,   0,   0,  50,  40,   0,   0,   0,  300  }, // leg1,2,3 dn
  {   40,   0,   0,   0, -40,   0,   0,  50,  300  }, // leg1,3,4 up
  {  -40,   0,   0,   0,  40,   0,   0, -50,  300  }, // leg1,3,4 dn
  {   40,   0,   0, -50, -40,   0,   0,   0,  300  }, // leg1,2,3 up
  {  -40,   0,   0,  50,  40,   0,   0,   0,  300  }, // leg1,2,3 dn
  {   40,   0,   0,   0, -40,   0,   0,  50,  300  }, // leg1,3,4 up
  {  -40,   0,   0,   0,  40,   0,   0, -50,  300  }, // leg1,3,4 dn
  {    0,  45,  45,   0,   0, -45, -45,   0,  300  }, // standby
};

// up
int servoPrg16step = 1;
int servoPrg16 [][numberOfACE] PROGMEM = {
  // P10, P11, P12, P16, P02, P04, P07, P15,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // stop
};

int servoPrg17step = 1;
int servoPrg17 [][numberOfACE] PROGMEM = {
  // P10, P11, P12, P16, P02, P04, P07, P15,  ms
  {   180,  90,  90,   0,   0,  90,  90,  180,  300 },
//  {   +20,   0,   0,  -20,  -20,   0,   0,  +20,  300 },
//  {     200,   0,   0,    0,    0,   0,   0,    0,  1000 },
};

int servoPrg18step = 9;
int servoPrg18 [][numberOfACE] PROGMEM = {
  // P10, P11, P12, P16, P02, P04, P07, P15,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  200  }, // standby
  {   0,  0,  0, 0, 0,  -60,  0,  0,  200  },
  {   +150,  +50,  0, 0, 0,  0,  0,  0,  300  },
  {   -70,  0,  0, 0, 0,  0,  0,  0,  300  },
  {   +70,  0,  0, 0, 0,  0,  0,  0,  300  },
  {   -70,  0,  0, 0, 0,  0,  0,  0,  300  },
  {   +70,  0,  0, 0, 0,  0,  0,  0,  300  },
  {   -70,  0,  0, 0, 0,  0,  0,  0,  300  },
  {   +70,  0,  0, 0, 0,  0,  0,  0,  300  },
//  {   +20,   0,   0,  -20,  -20,   0,   0,  +20,  300 },
//  {     0,   0,   0,    0,    0,   0,   0,    0,  1000 },
};

void setup() {
  Serial.begin(115200);
  Serial.println("Robot Starts Initialization");
//---------------
  WiFi.mode(WIFI_AP);
  IPAddress softLocal(192,168,1,1);                      // 设置内网WIFI IP地址
  IPAddress softGateway(192,168,1,1);
  IPAddress softSubnet(255,255,255,0);
  WiFi.softAPConfig(softLocal, softGateway, softSubnet);
   
  String apName = ("dachou_robot");  // 设置WIFI名称 //+(String)ESP.getChipId()
  const char *softAPName = apName.c_str();
   
  WiFi.softAP(softAPName, "55588888");      //  创建wifi  名称 + 密码 
   
  IPAddress myIP = WiFi.softAPIP(); 
  Serial.println("");
  Serial.print("AP直链模式 网关IP: ");     
  Serial.println(myIP);
   
  Serial.print("AP SSID: ");   //串口打印WIFI 名称
  Serial.println(apName);
  Serial.print("AP 密码: ");   
  Serial.println("55588888");  //串口打印WIFI 密码
//---------
    Udp.begin(localPort);   //UDP服务初始化
    //------------
  servo[0].attach(D1);
  servo[0].write(90 + servoCal[0]);
  servo[1].attach(D2);
  servo[1].write(90 + servoCal[1]);
  servo[2].attach(D3);
  servo[2].write(90 + servoCal[2]);
  servo[3].attach(D4);
  servo[3].write(90 + servoCal[3]);
  servo[4].attach(D5);
  servo[4].write(90 + servoCal[4]);
  servo[5].attach(D6);
  servo[5].write(90 + servoCal[5]);
  servo[6].attach(D7);
  servo[6].write(90 + servoCal[6]);
  servo[7].attach(D8);
  servo[7].write(90 + servoCal[7]);
}

void loop() {

    s_udp();
   switch (cmd) {
    case 0:
      runServoPrgV(servoPrg06, servoPrg06step); // turnLeft左转
      break;
    case 1:
      runServoPrgV(servoPrg02, servoPrg02step); // forward前进
      break;
    case 2:
      runServoPrgV(servoPrg07, servoPrg07step); // turnRight右转
       break;
    case 3:
     runServoPrgV(servoPrg04, servoPrg04step); // moveLeft左移动
      break;
    case 4:
      runServoPrgV(servoPrg03, servoPrg03step); // backward后退
       break;
    case 5:
     runServoPrgV(servoPrg05, servoPrg05step); // moveRight右移动
       break;
    case 6:
     runServoPrgV(servoPrg01, servoPrg01step); // standby(待机)
       break;
    case 7:
     runServoPrgV(servoPrg09, servoPrg09step); // sayHi打招呼
      break;
    case 8:
      runServoPrgV(servoPrg11, servoPrg11step); // pushUp俯卧撑
      break;
    case 9:
      runServoPrgV(servoPrg08, servoPrg08step); //sleep睡觉
      break;
    case 10:
      runServoPrgV(servoPrg10, servoPrg10step); // 恰恰舞
      break;
    case 11:
      runServoPrgV(servoPrg12, servoPrg12step); 
      break;
    case 12:
      runServoPrgV(servoPrg13, servoPrg13step); // dancing1太空步
      break;
    case 13:
      runServoPrgV(servoPrg14, servoPrg14step); // dancing2左右摇摆
      break;
    case 14:
      runServoPrgV(servoPrg15, servoPrg15step); // dancing3抬腿舞
      break;
    case 15:
      runServoPrgV(servoPrg00, servoPrg00step); // zero不好用
    case 16:
      runServoPrgV(servoPrg16, servoPrg16step); // stop
      break;
    case 17:
      runServoPrgV(servoPrg17, servoPrg17step); // lie躺
      break;
    case 18:
      runServoPrgV(servoPrg18, servoPrg18step); // provocation 挑衅
      break;
}
}

void s_udp(){
  int packetSize = Udp.parsePacket();
  if (packetSize) {           //等待接受UDP数据，一旦收到就进入处理

    int n = Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);  
    packetBuffer[n] = 0;
    Serial.print("收到数据:");
    Serial.println(packetBuffer);
    if(!(strcmp(packetBuffer,"go")))
    {
      cmd=1;
    }
    else if(!(strcmp(packetBuffer,"right")))
    {
      cmd=5;
    }
    else if(!(strcmp(packetBuffer,"retreat")))
    {
      cmd=4;
    }
    else if(!(strcmp(packetBuffer,"left")))
    {
      cmd=3;
    }
    else if(!(strcmp(packetBuffer,"sayHi")))
    {
      cmd=7;
    }
    else if(!(strcmp(packetBuffer,"turnLeft")))
    {
      cmd=0;
    }
    else if(!(strcmp(packetBuffer,"turnRight")))
    {
      cmd=2;
    }
    else if(!(strcmp(packetBuffer,"pushUp")))
    {
      cmd=8;
    } 
    else if(!(strcmp(packetBuffer,"sleep")))
    {
      cmd=9;
    }
    else if(!(strcmp(packetBuffer,"fighting")))
    {
      cmd=10;
    }
    else if(!(strcmp(packetBuffer,"lie")))
    {
      cmd=17;
    }
    else if(!(strcmp(packetBuffer,"dancing1")))
    {
      cmd=12;
    }
    else if(!(strcmp(packetBuffer,"dancing2")))
    {
      cmd=13;
    }
    else if(!(strcmp(packetBuffer,"dancing3")))
    {
      cmd=14;
    }
    else if(!(strcmp(packetBuffer,"provocation")))
    {
      cmd=18;
    }
    else
    {
      cmd=16;
    }
    memset(packetBuffer,'\0',sizeof(packetBuffer));
  }
}

void runServoPrg(int servoPrg[][numberOfACE], int step)
{
  for (int i = 0; i < step; i++) { // Loop for step

    int totalTime = servoPrg[i][numberOfACE - 1]; // Total time of this step

    // Get servo start position
    for (int s = 0; s < numberOfServos; s++) {
      servoPos[s] = servo[s].read() - servoCal[s];
    }

    for (int j = 0; j < totalTime / servoPrgPeriod; j++) { // Loop for time section
      for (int k = 0; k < numberOfServos; k++) { // Loop for servo
        servo[k].write((map(j, 0, totalTime / servoPrgPeriod, servoPos[k], servoPrg[i][k])) + servoCal[k]);
      }
      delay(servoPrgPeriod);
    }
  }
}

void runServoPrgV(int servoPrg[][numberOfACE], int step) {
  for (int i = 0; i < step; i++) { // Loop for step

    int totalTime = servoPrg[i][numberOfACE - 1]; // Total time of this step

    // Get servo start position
    for (int s = 0; s < numberOfServos; s++) {
      servoPos[s] = servo[s].read() - servoCal[s];
    }

    for (int p = 0; p < numberOfServos; p++) { // Loop for servo
      if (i == 0) {
        servoPrevPrg[p] = servoPrg[i][p];
      } else {
        servoPrevPrg[p] = servoPrevPrg[p] + servoPrg[i][p];
      }
    }

    for (int j = 0; j < totalTime / servoPrgPeriod; j++) { // Loop for time section
      for (int k = 0; k < numberOfServos; k++) { // Loop for servo
        servo[k].write((map(j, 0, totalTime / servoPrgPeriod, servoPos[k], servoPrevPrg[k]) + servoCal[k]));
        s_udp();
      }
      delay(servoPrgPeriod);
      s_udp();
      
    }
    s_udp();
  }
}
