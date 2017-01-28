/*
The programme is written by Mrinmoy sarkar.
 It is a project for iARC Robotic Competetion TECHKRITI 2014
 */
//all header file

int sensor_calibration_time = 1500;
int node_time = 350;
// all variable
int stop_flag = 1;
String path = "";
String short_path = "";
//int speed_m = 100;
int dlay = 50;
int forward_speed = 100;
int forward_speed_for_straight = 70;
int left_speed = 100;
int right_speed = 100;
int u_speed = 60;
int backward_speed = 40;
float speed_ratio_n = .2;
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
  //init_sd();
  calibrate_sensor();

  //delay(5000);
  //check_sensor_data();

}

void loop()
{
  stop_flag = 1;
  check_sensor_data();
  delay(3000);
  //left_turn();
  if(true)//&&!(digitalRead(loop_switch)==0))
  {
    blow_buzzer(500);
    while(1)
    {
      flow_line();
      get_sensor_data();
      if(((Light_sensor_data_backward & 1) == 1) && (Light_sensor_data_forward == 0) && (Light_sensor_data_backward != 0b11111))
      {
        left_turn();
      }
      else if(((Light_sensor_data_forward & 0b10000) == 0b10000) && (Light_sensor_data_forward == 0) && (Light_sensor_data_backward != 0b11111))
      {
        right_turn();
      }
    }
  }
}


void flow_line()
{
  get_sensor_data();
  if( (Light_sensor_data_forward == 0)&&(Light_sensor_data_backward == 0) && (check_mid() == 0))
  {
    analog_go_backward(backward_speed);
  }
  else if(!((Light_sensor_data_forward & 1 ) > 0 || (Light_sensor_data_forward & 16)>0))
  {
    u_flag = 1;
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
    get_sensor_data();
    if(((Light_sensor_data_backward & 0b1110) == 0b100) || ((Light_sensor_data_forward & 0b1110) == 0b1110))
    {
      analog_go_forward(forward_speed_for_straight);
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






void left_turn()
{
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
    int sensor_data = (Light_sensor_data_forward & 0b11111);// & (check_mid()>>2) & (Light_sensor_data_backward & 0b100);
    if((sensor_data == 0b00100)) //|| (sensor_data == 0b01100) || (sensor_data == 0b00110))
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


void straight()
{
  right_flag = false;
  path += "S";
  while(1)
  {
    flow_line();
    get_sensor_data();
    if(((Light_sensor_data_backward & 0b1) == 0) && ((Light_sensor_data_backward & 0b10000) == 0))
    {
      stop_motor();
      break;
    }
  }
}

void u_turn()
{
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
}

void right_turn()
{
  while(1)
  {
    get_sensor_data();
    if(((Light_sensor_data_forward & 0b1110) == 0) && (check_mid() == 0))
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
  analogWrite(right_motor_forward,speed_m);
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



































