#define IN1  7   //K1、K2 motor direction
#define IN2  8     //K1、K2 motor direction
#define IN3  9    //K3、K4 motor direction
#define IN4  10   //K3、K4 motor direction
#define ENA  5    // Needs to be a PWM pin to be able to control motor speed ENA
#define ENB  6    // Needs to be a PWM pin to be able to control motor speed ENA
#define LFSensor_1 2 //line follow sensor1
#define LFSensor_2 3 //line follow sensor2
char sensor[2]={0,0};
#define M_SPEED1   150  //motor speed
#define M_SPEED2   170  //motor speed
#define MAX_PACKETSIZE 32    //Serial receive buffer
char buffUART[MAX_PACKETSIZE];
unsigned int buffUARTIndex = 0;
unsigned long preUARTTick = 0;
bool stopFlag = true;
bool JogFlag = false;
uint16_t JogTimeCnt = 0;
uint32_t JogTime=0;
unsigned int barIter = 0;
enum DS
{
  MANUAL_DRIVE,
  AUTO_DRIVE_LF, //line follow
}Drive_Status=AUTO_DRIVE_LF;

enum DN
{ 
  GO_ADVANCE, 
  GO_LEFT, 
  GO_RIGHT,
  GO_BACK,
  STOP_STOP,
  DEF
}Drive_Num=DEF;
/*motor control*/
void go_back(int t)  //motor rotate clockwise -->robot go ahead
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4,HIGH);
  delay(t);
}
void go_ahead(int t) //motor rotate counterclockwise -->robot go back
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4,LOW); 
  delay(t);
}
void go_stop() //motor brake -->robot stop
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4,LOW); 
}
void turn_left(int t)  //left motor rotate clockwise and right motor rotate counterclockwise -->robot turn right
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(t);
}
void turn_right(int t) //left motor rotate counterclockwise and right motor rotate clockwise -->robot turn left
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(t);
}
/*set motor speed */
void set_motorspeed(int lspeed,int rspeed) //change motor speed
{
  analogWrite(ENA,lspeed);//lspeed:0-255
  analogWrite(ENB,rspeed);//rspeed:0-255   
}
/*read line folloe sensors*/
void read_sensor_values()
{
  sensor[0]=digitalRead(LFSensor_1);
  sensor[1]=digitalRead(LFSensor_2);
}
void auto_tarcking(){
  read_sensor_values();
  if((sensor[0]==LOW)&&(sensor[1]==HIGH)){ //The right sensor is on the black line.The left sensor is on the white line
    set_motorspeed(M_SPEED1,M_SPEED1);
    turn_right(20);
  }
else if((sensor[0]==HIGH)&&(sensor[1]==LOW)){//The right sensor is on the white line.The left sensor is on the black line
    set_motorspeed(M_SPEED1,M_SPEED1);
    turn_left(20);
  }
  else if((sensor[0]==LOW)&&(sensor[1]==LOW)){//The left an right sensor are on the white line.
    set_motorspeed(M_SPEED2,M_SPEED2);
    go_ahead(0);
  }
  else if((sensor[0]==HIGH)&&(sensor[1]==HIGH)){//The left an right sensor are on the black line.
    set_motorspeed(M_SPEED2,M_SPEED2);
    barIter++;
    Serial.println(barIter);
    switch (barIter) {
      case 1:
        go_ahead(300);
        break;
      case 2:
        go_back(300);
        break;
      case 3:
        go_ahead(300);
      default:
        break;
    }
    go_stop();
    delay(500);
  }
 }
 //WiFi / Bluetooth through the serial control
void do_Uart_Tick() {
  if(Serial.available()) {
  }
}
//robot motor control
void do_Drive_Tick()
{
  if(Drive_Status == MANUAL_DRIVE)
  {
    switch (Drive_Num) 
    {
      case GO_ADVANCE:
          set_motorspeed(255,255);
          go_ahead(15);
          JogFlag = true;
          JogTimeCnt = 3;
          JogTime=millis();
          break;
      case GO_LEFT: 
          set_motorspeed(100,100);
          turn_left(2);
          JogFlag = true;
          JogTimeCnt = 1;
          JogTime=millis();
          break;
      case GO_RIGHT:  
           set_motorspeed(100,100);
           turn_right(2);
          JogFlag = true;
          JogTimeCnt = 1;
          JogTime=millis();
          break;
      case GO_BACK: 
          set_motorspeed(150,150);
          go_back(10);
          JogFlag = true;
          JogTimeCnt = 1;
          JogTime=millis();
          break;
      case STOP_STOP: 
          go_stop();
          JogTime = 0;
          break;
      default:
          break;
    }
    Drive_Num=DEF;
    //keep the car running for 100ms
    if(millis()-JogTime>=100)
    {
      JogTime=millis();
      if(JogFlag == true) 
      {
        stopFlag = false;
        if(JogTimeCnt <= 0) 
        {
          JogFlag = false; stopFlag = true;
        }
        JogTimeCnt--;
      }
      if(stopFlag == true) 
      {
        JogTimeCnt=0;
        go_stop();
      }
    }
  }
  else if(Drive_Status==AUTO_DRIVE_LF)
  {
    auto_tarcking();
  }
}
void setup() {
  /*line follow sensors */
  pinMode(LFSensor_1,INPUT);
  pinMode(LFSensor_2,INPUT); 
  /******L298N******/
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT); 
  pinMode(ENA, OUTPUT);  
  pinMode(ENB, OUTPUT);
  go_stop();
  Serial.begin(9600);
}

void loop() {
  do_Drive_Tick();
  do_Uart_Tick();
}
