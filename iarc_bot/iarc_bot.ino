/*
The programme is written by Mrinmoy sarkar.
 It is a project for iARC Robotic Competetion TECHKRITI 2014
 */
//all header file
//#include<EEPROM.h>
#include<SD.h>
//#include<LiquidCrystal.h>

int sensor_calibration_time = 1500;
int node_time = 350;
int turn_delay = 270;
int turn_speed = 117;
// all variable
int stop_flag = 1;
String path = "";
String short_path = "";
//int speed_m = 100;
int dlay = 50;
int forward_speed = 90;
int forward_speed_for_straight = 75;
int left_speed = 75;
int right_speed = 75;
int u_speed = 60;
int backward_speed = 80;
float speed_ratio_n = .3;
float speed_ratio_l = .2;
int Light_sensor_data_forward = 0;//to store the sensor value
int Light_sensor_data_backward = 0;
// to indicate left turn
int thres_val_mid = 110;
int mid_pin = 10;

double distance;//to store the distance of any object
double threshold_distance = 5;//distance from the destination box
//buzzer pin
int buzzer_pin = 21;
//all sensor input pin
int precision = 70;
int total_sensor = 10;//no of sensors
int sensorMin[] = {
  1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023      };
int sensorMax[] = {
  0,0,0,0,0,0,0,0,0,0,0      };
int white[11] = {
  128,128,128,128,128,128,128,128,128,128,128};//referance for white or black

//motor control pin
int right_motor_forward = 4;
int right_motor_backward = 5;
int left_motor_forward = 6;
int left_motor_backward = 7;
//sonor pin
int sonor_input_pin = 15;
int sonor_trigger_pin = 16;

int loop_switch = 20;

int u_flag = 1;
int temp_val=0;
int flag = 0;
int fl = 0;
int sdflag = 0;
boolean left_flag = false;
boolean right_flag = false;
boolean straight_flag = true;
void setup()
{
  pinMode(buzzer_pin,OUTPUT);

  pinMode(left_motor_forward,OUTPUT);
  pinMode(left_motor_backward,OUTPUT);
  pinMode(right_motor_forward,OUTPUT);
  pinMode(right_motor_backward,OUTPUT);

  pinMode(sonor_input_pin,INPUT);
  pinMode(sonor_trigger_pin,OUTPUT);

  pinMode(loop_switch ,INPUT); 
  //initialize sd   card
  Serial.begin(9600);

  calibrate_sensor();

  delay(5000);
  //check_sensor_data();

}

void loop()
{
  stop_flag = 1;
  //check_sensor_data();
  //left_turn();
  //while(1);
  if(true)//!(digitalRead(loop_switch) == 1))
  {
    blow_buzzer(200);
    delay(3000);
    //straight();
    //analog_go_forward(70);
    navigate_arena();
    //do_the_job();
    //navigate_arena();
    //Serial.println(path);
    short_path = solve_maze(path);
    Serial.println(short_path);
    //short_path = "RRLSL";
    //go_through_shortest_path();
    //flow_line();
    //right_turn();
    //short_path = "RLSLLRSL";
    go_through_shortest_path();
    while(1)
      check_sensor_data();
  }
}

void go_through_shortest_path()
{
  int indx = 0;
  int len = short_path.length();
  u_turn();
  delay(100);
  while(1)
  {
    flow_line();  
    get_sensor_data();
    if((Light_sensor_data_forward & 1) == 1)
    {
      left_flag = true;
    }
    else if(((Light_sensor_data_forward & 0b10000) == 0b10000) && !left_flag)
    {
      right_flag = true;
    }
    if(left_flag || right_flag)
    {
      left_flag = false;
      right_flag = false;
      if(short_path[indx] == 'R')
      {
        straight_flag = false;
        right_turn();
        indx++;
      }
      else if(short_path[indx] == 'L')
      {
        left_turn();
        indx++;
      }
      else if(short_path[indx] == 'S')
      {
        straight();
        indx++;
      }
    }
    if(indx == len)
    {
      stop_motor();
      break;
    }
    stop_flag = 1;
  }
  while(1)
  {
    flow_line();
    get_sensor_data();
    if((Light_sensor_data_forward == 0) || (Light_sensor_data_forward == 0b11111))
    {
      stop_motor();
      break;
    }
  }
}

void flow_line()
{
  get_sensor_data();
  if((Light_sensor_data_forward == 0) && (Light_sensor_data_backward == 0) && (check_mid() == 0))
  {
    stop_motor();
  }
  else if(!((Light_sensor_data_forward & 1 ) > 0 || (Light_sensor_data_forward & 16)>0))
  {
    get_sensor_data();
    if(((Light_sensor_data_forward & 0b1110) == 0b100) || ((Light_sensor_data_forward & 0b1110) == 0b1110))
    {
      analog_go_forward(forward_speed);
    }
    else if(((Light_sensor_data_forward & 0b1110) == 0b110) || ((Light_sensor_data_forward & 0b1110) == 0b010))
    {
      analog_go_left_n(left_speed,speed_ratio_l);
    }
    else if(((Light_sensor_data_forward & 0b1110) == 0b1100) || ((Light_sensor_data_forward & 0b1110) == 0b1000))
    {
      analog_go_right_n(right_speed,speed_ratio_l);
    } 
  }
  else
  {
    u_flag = 1;
    get_sensor_data();
    if(((Light_sensor_data_backward & 0b1110) == 0b100) || ((Light_sensor_data_forward & 0b1110) == 0b1110))
    {
      analog_go_forward(forward_speed);
    }
    else if(((Light_sensor_data_backward & 0b1110) == 0b110) || ((Light_sensor_data_backward & 0b1110) == 0b010))
    {
      analog_go_left_n(left_speed,speed_ratio_n);
    }
    else if(((Light_sensor_data_backward & 0b1110) == 0b1100) || ((Light_sensor_data_backward & 0b1110) == 0b1000))
    {
      analog_go_right_n(right_speed,speed_ratio_n);
    }
  }
}

void navigate_arena()
{
  while(stop_flag)
  {
    flow_line();
    get_sensor_data();
    if((Light_sensor_data_forward & 1) == 1)
    {
      left_flag = true;
    }
    else if(((Light_sensor_data_forward & 0b10000) == 0b10000) && !left_flag)
    {
      right_flag = true;
    }
    if(left_flag)
    {
      left_flag = false;
      stop_motor();
      left_turn();
      //analog_go_forward(forward_speed);
      delay(100);
    }
    else if((!left_flag) && right_flag)
    {
      if(!left_check())
      {  
        straight_flag = true;
        stop_motor();
        straight();
        delay(70);
      }  
    }
    if((!left_flag) && right_flag)
    {
      //if(!left_check())
      {
        right_flag = false;
        stop_motor();
        right_turn();
        delay(70);
      }
    }
    else if(!left_flag && !right_flag && (Light_sensor_data_forward == 0) && (Light_sensor_data_backward == 0) && (check_mid() == 0))
    {
      stop_motor();
      u_turn();
      delay(70);
    }
  }
}
boolean left_check()
{
  long start = millis();
  long stop_m = 0;
  boolean rtn = false;
  analog_go_right_n(60,0);
  while((stop_m-start) < 300)
  {
    get_sensor_data();
    if((Light_sensor_data_forward & 1) == 1)
    {
      left_flag = true;
      right_flag = false;
      rtn = true;
      break;
    }
    stop_m = millis();
  }
  stop_motor();
  analog_go_backright(60);
  delay((stop_m-start));
  stop_motor();
  delay(20);
  return rtn;
}
void left_turn()
{
  analog_go_forward(turn_speed);
  delay(turn_delay);
  stop_motor();
  left_flag = false;
  int stp = 0;
  while(stop_flag)
  {
    get_sensor_data();
    if((Light_sensor_data_forward & 1) == 1)
    {
      stop_motor();
      break;
    }
    else
    {
      analog_go_u_left(u_speed);
    }
    end_point_check();
  }
  while(stop_flag)
  {
    get_sensor_data();
    int sensor_data = (Light_sensor_data_forward & 0b11111);// & (check_mid()>>2) & (Light_sensor_data_backward & 0b100);
    if((sensor_data == 0b00100)|| (sensor_data == 0b01100) || (sensor_data == 0b00110))
    {
      stop_motor();
      break;
    }
    else
    {
      analog_go_u_left(u_speed);
    }
    end_point_check();
  }
  if(stop_flag == 1)
  {
    path += "L";
  }
}


void straight()
{
  analog_go_forward(turn_speed);
  delay(turn_delay);
  stop_motor();
  get_sensor_data();
  if(((Light_sensor_data_forward & 0b10101) == 0b00100) || ((Light_sensor_data_forward & 0b10011) == 0b00010) || ((Light_sensor_data_forward & 0b11001) == 0b01000))
  {
    stop_motor();
    path+="S";
    right_flag = false;
  }
}

void u_turn()
{
  //  analog_go_forward(turn_speed);
  //  delay(turn_delay);
  //  stop_motor(); 
  path += "U";
  while(1)
  {
    get_sensor_data();
    if((Light_sensor_data_forward & 0b1110) == 0)
    {
      stop_motor();
      break;
    }
    else
    {
      analog_go_u_left(u_speed);
    }
  }
  while(1)
  {
    get_sensor_data();
    int sensor_data = Light_sensor_data_forward & 0b11111;
    if((sensor_data == 0b00100) || (sensor_data == 0b01100) || (sensor_data == 0b00110))
    {
      stop_motor();
      break;
    }
    else
    {
      analog_go_u_left(u_speed);
    }
  } 
}

void right_turn()
{  
  if(straight_flag != true)
  {
    analog_go_forward(turn_speed);
    delay(turn_delay);
    stop_motor();
  }
  straight_flag = false;
  right_flag = false;
  path += "R";
  while(1)
  {
    get_sensor_data();
    if((Light_sensor_data_forward & 0b10000) == 0b10000)
    {
      stop_motor();
      break;
    }
    else
    {
      analog_go_u_right(u_speed);
    }
  }
  while(1)
  {
    get_sensor_data();
    int sensor_data = Light_sensor_data_forward & 0b11111;
    if((sensor_data == 0b00100) || (sensor_data == 0b01100) || (sensor_data == 0b00110))
    {
      stop_motor();
      break;
    }
    else
    {
      analog_go_u_right(u_speed);
    }
  }
}





void end_point_check()
{
  get_sensor_data();
  boolean t = (Light_sensor_data_forward == 0b11111);
  if(t)
  {
    stop_flag = 0;
    stop_motor();
  }
  if(digitalRead(loop_switch) == 0)
  {
    stop_flag = 0;
    blow_buzzer(200);
    stop_motor();
  }
}

void do_the_job()
{
  navigate_arena();
  Serial.println(path);
  short_path = solve_maze(path);
  Serial.println(short_path);
  delay(500);
  go_through_shortest_path();
}

void calibrate_sensor()
{
  blow_buzzer(250);
  while(millis() < sensor_calibration_time)
  {
    analog_go_forward(forward_speed);
    int analog_data = 0;
    for(int i = 0; i <= total_sensor; i++)
    {
      analog_data = analogRead(i);
      if(analog_data > sensorMax[i])
      {
        sensorMax[i] = analog_data;
      }
      if(analog_data < sensorMax[i])
      {
        sensorMin[i] = analog_data;
      }
    }
  }
  stop_motor();
  while(millis() < (sensor_calibration_time * 2))
  {
    analog_go_backward(forward_speed);
    int analog_data = 0;
    for(int i = 0; i <= total_sensor; i++)
    {
      analog_data = analogRead(i);
      if(analog_data > sensorMax[i])
      {
        sensorMax[i] = analog_data;
      }
      if(analog_data < sensorMax[i])
      {
        sensorMin[i] = analog_data;
      }
    }
  }
  for(int i = 0; i <= total_sensor; i++)
  {
    int analog_data = int((sensorMax[i] + sensorMin[i]) / 2);
    white[i] = map(analog_data, sensorMin[i], sensorMax[i], 0, 255);
    Serial.println(white[i]);
  }
  Serial.println("%%%%%%%%%%%%%%%%%%%%%%%%%%");
  stop_motor();
  blow_buzzer(250);
}

void get_sensor_data()
{
  unsigned int Light_sensor_data = 0;
  int analog_data = 0;
  for(int i = 0; i < total_sensor; i++)
  {
    analog_data = analogRead(i);
    analog_data = map(analog_data, sensorMin[i], sensorMax[i], 0, 255);
    analog_data = constrain(analog_data, 0, 255);
    if(analog_data > white[i])
    {
      Light_sensor_data |= 1<<(i);
    }
  }
  Light_sensor_data_forward = Light_sensor_data & 31;
  Light_sensor_data_backward = (Light_sensor_data & 512)>>9 | (Light_sensor_data & 224)>>4 | (Light_sensor_data & 256)>>4;
}


void get_sensor_data_ck()
{
  unsigned int Light_sensor_data = 0;
  int analog_data = 0;
  for(int i = 0; i < total_sensor; i++)
  {
    analog_data = analogRead(i);
    analog_data = map(analog_data, sensorMin[i], sensorMax[i], 0, 255);
    analog_data = constrain(analog_data, 0, 255);
    Serial.println(analog_data);
    if(analog_data > white[i])
    {
      Light_sensor_data |= 1<<(i);
    }
  }
  Light_sensor_data_forward = Light_sensor_data & 31;
  Light_sensor_data_backward = (Light_sensor_data & 512)>>9 | (Light_sensor_data & 224)>>4 | (Light_sensor_data & 256)>>4;
}


int check_mid()
{
  int analog_data = analogRead(mid_pin);
  analog_data = map(analog_data, sensorMin[mid_pin], sensorMax[mid_pin], 0, 255);
  analog_data = constrain(analog_data, 0, 255);
  if(analog_data < white[mid_pin])
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

int check_mid_ck()
{
  int analog_data = analogRead(mid_pin);
  analog_data = map(analog_data, sensorMin[mid_pin], sensorMax[mid_pin], 0, 255);
  analog_data = constrain(analog_data, 0, 255);
  Serial.println(analog_data);
  if(analog_data < white[mid_pin])
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

double get_distance()
{
  digitalWrite(sonor_trigger_pin,HIGH);
  delayMicroseconds(10);
  digitalWrite(sonor_trigger_pin,LOW);
  int i = pulseIn(sonor_input_pin,HIGH);
  double distance = 0.017 * i;
  delay(3);
  return distance;
}

void check_sensor_data()
{
  get_sensor_data_ck();
  Serial.println("****************************");
  Serial.println("forward sensor");
  Serial.println(Light_sensor_data_forward,BIN);
  Serial.println("backward sensor");
  Serial.println(Light_sensor_data_backward,BIN);
  Serial.println("mid:");
  Serial.println(check_mid_ck());
  Serial.println("****************************");
  delay(3000);
}





void blow_buzzer(int delay_time)
{
  digitalWrite(buzzer_pin,HIGH);
  delay(delay_time);
  digitalWrite(buzzer_pin,LOW);
}

void stop_motor()
{
  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_backward,LOW);
  digitalWrite(left_motor_forward,LOW);
  digitalWrite(right_motor_forward,LOW);
}

void analog_go_right_n(int speed_m,float speed_ratio)
{

  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_backward,LOW);
  analogWrite(left_motor_forward,speed_m);
  analogWrite(right_motor_forward,int(speed_m * speed_ratio));
}

void analog_go_backright(int speed_m)
{
  analogWrite(left_motor_backward,speed_m);
  digitalWrite(right_motor_backward,LOW);
  digitalWrite(left_motor_forward,LOW);
  digitalWrite(right_motor_forward,LOW);
}

void analog_go_u_right(int speed_m)
{
  digitalWrite(left_motor_backward,LOW);
  analogWrite(right_motor_backward,speed_m);
  analogWrite(left_motor_forward,speed_m);
  digitalWrite(right_motor_forward,LOW); 
}

void analog_go_left_n(int speed_m,float speed_ratio)
{
  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_backward,LOW);
  analogWrite(left_motor_forward,int(speed_ratio*speed_m));
  analogWrite(right_motor_forward,speed_m);
}

void analog_go_backleft(int speed_m)
{
  digitalWrite(left_motor_backward,LOW);
  analogWrite(right_motor_backward,speed_m);
  digitalWrite(left_motor_forward,LOW);
  digitalWrite(right_motor_forward,LOW);
}
void analog_go_u_left(int speed_m)
{
  analogWrite(left_motor_backward,speed_m);
  digitalWrite(right_motor_backward,LOW);
  digitalWrite(left_motor_forward,LOW);
  analogWrite(right_motor_forward,(speed_m));
}

void analog_go_forward(int speed_m)
{
  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_backward,LOW);
  analogWrite(left_motor_forward,speed_m);
  analogWrite(right_motor_forward,speed_m);
}


void analog_go_backward(int speed_m)
{
  analogWrite(left_motor_backward,speed_m);
  analogWrite(right_motor_backward,speed_m);
  digitalWrite(left_motor_forward,LOW);
  digitalWrite(right_motor_forward,LOW);
}





String solve_maze(String total_path)
{
  int len = total_path.length();
  //println(len);
  for(int i=0;i<len-2;i++)
  {
    String sub = total_path.substring(i,i+3);
    if(sub.equals("LUL"))
    {
      total_path = total_path.substring(0,i)+"S"+total_path.substring(i+3,len);
      len = total_path.length();
      i=-1;
    }
    else if(sub.equals("SUL") || sub.equals("LUS"))
    { 
      total_path = total_path.substring(0,i)+"R"+total_path.substring(i+3,len);
      len = total_path.length();
      i=-1;
    }
    else if(sub.equals("RUL"))
    { 
      total_path = total_path.substring(0,i)+"U"+total_path.substring(i+3,len);
      len = total_path.length();
      i=-1;
    }
  }
  // println(total_path);
  len = total_path.length();
  for(int i = 0;i < len;i++)
  {
    if(total_path[i] == 'R')
    {
      total_path[i] = 'L';
    }
    else if(total_path[i] == 'L')
    {
      total_path[i] = 'R';
    }
  }
  String tem = total_path;
  for(int i = 0;i < len;i++)
  {
    total_path[i] = tem[len-i-1];
  }
  return total_path;
}




































