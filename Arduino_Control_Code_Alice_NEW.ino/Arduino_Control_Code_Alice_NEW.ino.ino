// Designed and Written by Fernando Martínez-García and Jesús Tamez-Duque. //
//        Copyright INDI Ingénierie et Design SAS France, 2019-2021.       //
/////////////////////////////////////////////////////////////////////////////


//////////////////////////////Calling libraries//////////////////////////////
#include <PID_L1_V4.h>
#include <PID_L2_V4.h>
#include <PID_L3_V4.h>
#include <PID_L4_V4.h>
////////////////////////////////////////////////////////////////////////////

///////////////////////////////Calling rosserial library///////////////////////////
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

#include <rosserial_arduino/Test.h>

//ROS inizialization
ros::NodeHandle  nh;
using rosserial_arduino::Test;

void callback (const Test::Request &req,Test::Response &res)
{

  // alcuni usano req.input e res.output
  if (req.input == 'n'){
  res.output=Storing();
  }
  else if (req.input == 's'){
  res.output=Stand_Straight();
  
  }
  else if (req.input == '1'){
  res.output=Step_One();
  
  }
  else if (req.input == '2'){
  res.output=Step_Two();
  
  }
  else if (req.input == '3'){
  res.output=Step_Three();
  
  }
  else if (req.input == '4'){
  res.output=Step_Four();
  
  }
  else if (req.input == '5'){
  res.output=Step_Five();
  
  }


}

ros::ServiceServer<Test::Request, Test::Response> server("/Movement_srv",&callback);
Test::Request test_req; //capire req da chi viene definita
Test::Response test_res;
std_msgs::Float32MultiArray potentiometer_msg;  //messaggio da pubblicare
ros::Publisher pub_potentiometer("/angle", &potentiometer_msg); //go to line 295

int Flag_Movement;//we must define it as global variable and it is callback from the function connected to the movement
float Maximum_time_Step = 5.0; //add this new global viariable that we use during the step function

//////////////////////////////Deacceleration values for the PID/////////////////////////////////
float General_deacceleration = 0;//If this value is less than 1, no deacceleration is implemented.
float Stopping_speed = 20.0;//This is the Stopping speed at which the motors are assumed to enter static friction regime
////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////Keys that are considered valid commands for SOME menus/////////////////////////////
char commands_for_menus[16] = {'1', '2', '3', '4', '0', 'y', 'u', 'h',
                               '6', 'i', 'o', 'k', 'l', 'n', 'M', '5'
                              }; // This defines the type of commands that are accepted to activate a set of instructions.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////Variables defined by user/////////////////////////////////////////////////////

///////////////////////-----------------------Motor Information-----------------------////////////////////////
//----------------------------- 1 Right Hip is 1R -------------------------------//
const int potPinRH = A2; // Potentiometer Pin INPUT
const int pwmPinRH = 2; // Motor PWM Pin OUTPUT
const int d1PinRH = 22; // Motor Direction - Backwards OUTPUT
const float directionflagRH = 1.0; // +1.0 for RIGHT MOTORS & -1.0 for LEFT MOTORS

int minangleRH = 0; // (Degrees)
int minpotvalueRH = 471; // 10-bit Resolution (0-1023)
int maxangleRH = 90; // (Degrees)
int maxpotvalueRH = 802; // 10-bit Resolution (0-1023)
int midpotvalueRH = (minpotvalueRH + maxpotvalueRH) / 2; // 10-bit Resolution (0-1023)
int midangleRH = (minangleRH + maxangleRH) / 2; // (Degrees)

float maxPWMRH = 90.0; // Max PWM in Percentage (0 to 100)
const float pwm_minimum_movementRH[2] = { 0.0, 0.0 };//These are values for overcoming the static friction (THE FIRST VALUE MUST BE THE LOWEST, AND IT IS RELATED TO FLEXON)

// ONOFF Mode
int PWMforwardsRH = 60; // (Command '1' = Direction A - Forwards) (0 to 120)
int PWMbackwardsRH = 60; // (Command '2' = Direction B - Backwards) (0 to 120)

// PID-Regulated Angluar Steps
float anglestepRH = 30.0; // level values for increment(Degrees)
float stepspeed_default_RH = 30.0; // (Degrees per second)

float minlimitstep_default_RH =  -10; // Min Angular Position Allowed by Code (Degrees)
float maxlimitstep_default_RH =  90; // Max Angular Position Allowed by Code (Degrees)
//--------------------------------------------------------------------------------------------//

//------------------------- 2 Right Knee is 2R ------------------------------//
const int potPinRK = A4; // Potentiometer Pin INPUT
const int pwmPinRK = 4; // Motor PWM Pin OUTPUT
const int d1PinRK = 24; // B Motor Direction - Backwards OUTPUT
const float directionflagRK = 1.0; // +1.0 for RIGHT MOTORS & -1.0 for LEFT MOTORS

int minangleRK = -90; // (Degrees)
int minpotvalueRK = 65; // 10-bit Resolution (0-1023)
int maxangleRK = 0; // (Degrees)
int maxpotvalueRK = 405; // 10-bit Resolution (0-1023)
int midpotvalueRK = (minpotvalueRK + maxpotvalueRK) / 2; // 10-bit Resolution (0-1023)
int midangleRK = (minangleRK + maxangleRK) / 2; // (Degrees)

float maxPWMRK = 90.0; // Max PWM in Percentage (0 to 100)
const float pwm_minimum_movementRK[2] = { 0.0, 0.0 };//These are values for overcoming the static friction (THE FIRST VALUE MUST BE THE LOWEST, AND IT IS RELATED TO FLEXON)

// ONOFF Mode
int PWMforwardsRK = 60; // (Command '1' = Direction A - Forwards) (0 to 120)
int PWMbackwardsRK = 60; // (Command '2' = Direction B - Backwards) (0 to 120)

// PID-Regulated Angluar Steps
float anglestepRK = 30.0; // level values for increment(Degrees)
float stepspeed_default_RK = 30.0; // (Degrees per second)

float minlimitstep_default_RK =  -90; // Min Angular Position Allowed by Code (Degrees)
float maxlimitstep_default_RK =  -5; // Max Angular Position Allowed by Code (Degrees)
//--------------------------------------------------------------------------------------------//

//--------------------------- 3 Left Knee is 2L --------------------------------//
const int potPinLK = A6; // Potentiometer Pin INPUT
const int pwmPinLK = 6; // Motor PWM Pin OUTPUT
const int d1PinLK = 26; // B Motor Direction - Backwards OUTPUT
const float directionflagLK = -1.0; // +1.0 for RIGHT MOTORS & -1.0 for LEFT MOTORS

int minangleLK = -90; // (Degrees)
int minpotvalueLK = 1005; // 10-bit Resolution (0-1023)
int maxangleLK = 0; // (Degrees)
int maxpotvalueLK = 700; // 10-bit Resolution (0-1023)
int midpotvalueLK = (minpotvalueLK + maxpotvalueLK) / 2; // 10-bit Resolution (0-1023)
int midangleLK = (minangleLK + maxangleLK) / 2; // (Degrees)

float maxPWMLK = 90.0; // Max PWM in Percentage (0 to 100)
const float pwm_minimum_movementLK[2] = { 0.0, 0.0 };//These are values for overcoming the static friction (THE FIRST VALUE MUST BE THE LOWEST, AND IT IS RELATED TO FLEXON)

// ONOFF Mode
int PWMforwardsLK = 60; // (Command '1' = Direction A - Forwards) (0 to 120)
int PWMbackwardsLK = 60; // (Command '2' = Direction B - Backwards) (0 to 120)

// PID-Regulated Angluar Steps
float anglestepLK = 30.0; // level values for increment(Degrees)
float stepspeed_default_LK = 30.0; // (Degrees per second)

float minlimitstep_default_LK =  -90; // Min Angular Position Allowed by Code (Degrees)
float maxlimitstep_default_LK =  -5; // Max Angular Position Allowed by Code (Degrees)
//--------------------------------------------------------------------------------------------//

//------------------------------ 4 Left Hip is 1L ------------------------------//
const int potPinLH = A8; // Potentiometer Pin INPUT
const int pwmPinLH = 8; // Motor PWM Pin OUTPUT
const int d1PinLH = 28; // B Motor Direction - Backwards OUTPUT
const float directionflagLH = -1.0; // +1.0 for RIGHT MOTORS & -1.0 for LEFT MOTORS

int minangleLH = 0; // (Degrees)
int minpotvalueLH = 677; // 10-bit Resolution (0-1023)
int maxangleLH = 90; // (Degrees)
int maxpotvalueLH = 364; // 10-bit Resolution (0-1023)
int midpotvalueLH = (minpotvalueLH + maxpotvalueLH) / 2; // 10-bit Resolution (0-1023)
int midangleLH = (minangleLH + maxangleLH) / 2; // (Degrees)


float maxPWMLH = 90.0; // Max PWM in Percentage (0 to 100)
const float pwm_minimum_movementLH[2] = { 0.0, 0.0 };//These are values for overcoming the static friction (THE FIRST VALUE MUST BE THE LOWEST, AND IT IS RELATED TO FLEXON)

// ONOFF Mode
int PWMforwardsLH = 60; // (Command '1' = Direction A - Forwards) (0 to 120)
int PWMbackwardsLH = 60; // (Command '2' = Direction B - Backwards) (0 to 120)

// PID-Regulated Angluar Steps
float anglestepLH = 30; // level value for increment(Degrees)
float stepspeed_default_LH = 30.0; // (Degrees per second)

float minlimitstep_default_LH =  -10; // Min Angular Position Allowed by Code (Degrees)
float maxlimitstep_default_LH =  90; // Max Angular Position Allowed by Code (Degrees)

const float Time_Dynamic_integral_component = 1.0; //time before achieving maximum user-designed power (seconds)

float Speed_array_default[4] = {stepspeed_default_LH, stepspeed_default_LK, stepspeed_default_RH, stepspeed_default_RK};
float Limit_array_default[8] = {minlimitstep_default_LH, maxlimitstep_default_LH, minlimitstep_default_LK, maxlimitstep_default_LK, minlimitstep_default_RH, maxlimitstep_default_RH, minlimitstep_default_RK, maxlimitstep_default_RK};
//--------------------------------------------------------------------------------------------//
///////////////////////---------------------------------------------------------------///////////////////////

//--------------------------------- PID Variables --------------------------------------------------------------------------------------//
float pwm_valueLH; // variable in the range [-255,255];
int manualPWMALH = PWMforwardsLH * directionflagLH * (+1); // RIGHT LEG = POSITIVE A and NEGATIVE B & LEFT LEG = NEGATIVE A and POSITIVE B
int manualPWMBLH = PWMbackwardsLH * directionflagLH * (-1);

float pwm_valueLK; // variable in the range [-255,255];
int manualPWMALK = PWMforwardsLK * directionflagLK * (+1); // RIGHT LEG = POSITIVE A and NEGATIVE B & LEFT LEG = NEGATIVE A and POSITIVE B
int manualPWMBLK = PWMbackwardsLK * directionflagLK * (-1);

float pwm_valueRH; // variable in the range [-255,255];
int manualPWMARH = PWMforwardsRH * directionflagRH * (+1); // RIGHT LEG = POSITIVE A and NEGATIVE B & LEFT LEG = NEGATIVE A and POSITIVE B
int manualPWMBRH = PWMbackwardsRH * directionflagRH * (-1);

float pwm_valueRK; // variable in the range [-255,255];
int manualPWMARK = PWMforwardsRK * directionflagRK * (+1); // RIGHT LEG = POSITIVE A and NEGATIVE B & LEFT LEG = NEGATIVE A and POSITIVE B
int manualPWMBRK = PWMbackwardsRK * directionflagRK * (-1);
//--------------------------------------------------------------------------------------------------------------------------------------//

//---------------------Sensor variables--------------------//
int potentiometer_valueLH;
float angleLH;
float angular_velocityLH;

int potentiometer_valueLK;
float angleLK;
float angular_velocityLK;

int potentiometer_valueRH;
float angleRH;
float angular_velocityRH;

int potentiometer_valueRK;
float angleRK;
float angular_velocityRK;
//---------------------------------------------------------
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////Controller System variables/////////////////////////////////////////
const float Power_levelLH = maxPWMLH; //Maximum power (percentage) allowed by the algorithm (this should be a number between 0 and 100, 80 is recomendable)
const float Power_levelLK = maxPWMLK; //Maximum power (percentage) allowed by the algorithm (this should be a number between 0 and 100, 80 is recomendable)
const float Power_levelRH = maxPWMRH; //Maximum power (percentage) allowed by the algorithm (this should be a number between 0 and 100, 80 is recomendable)
const float Power_levelRK = maxPWMRK; //Maximum power (percentage) allowed by the algorithm (this should be a number between 0 and 100, 80 is recomendable)

float Cal_valuesLH[6] = {minpotvalueLH, midpotvalueLH , maxpotvalueLH, minangleLH, midangleLH, maxangleLH}; // The first two values are the potentiometer values, the third is the minimum angle and the forth the maximum angle (potentiometer values should correspond with the angle values)
float Cal_valuesLK[6] = {minpotvalueLK, midpotvalueLK , maxpotvalueLK, minangleLK, midangleLK , maxangleLK}; // The first two values are the potentiometer values, the third is the minimum angle and the forth the maximum angle (potentiometer values should correspond with the angle values)
float Cal_valuesRH[6] = {minpotvalueRH, midpotvalueRH , maxpotvalueRH, minangleRH, midangleRH , maxangleRH}; // The first two values are the potentiometer values, the third is the minimum angle and the forth the maximum angle (potentiometer values should correspond with the angle values)
float Cal_valuesRK[6] = {minpotvalueRK, midpotvalueRK , maxpotvalueRK, minangleRK, midangleRK , maxangleRK}; // The first two values are the potentiometer values, the third is the minimum angle and the forth the maximum angle (potentiometer values should correspond with the angle values)

const float Sampling_time = 0.0064; // 0.0016 sampling time (s) for the controller (THIS VALUE MUST BE GREATHER THAN 1.6 MILISECONDS, "0.0016" for each motor used in parallel)

const float MultiplierLH = 1.5; //this value increases the overal angular accelaration used to achieve the desired speed. THE VALUE MUST BE IN THE RANGE [0.5,2]
const float MultiplierLK = 1.2; //this value increases the overal angular accelaration used to achieve the desired speed. THE VALUE MUST BE IN THE RANGE [0.5,2]
const float MultiplierRH = 1.0; //this value increases the overal angular accelaration used to achieve the desired speed. THE VALUE MUST BE IN THE RANGE [0.5,2]
const float MultiplierRK = 1.2; //this value increases the overal angular accelaration used to achieve the desired speed. THE VALUE MUST BE IN THE RANGE [0.5,2]

const float Power_level_pwmLH = floor(255 * min(max(Power_levelLH, 0), 100) / 100) * 1.0; //floor round the number to the nearest integer
const float Power_level_pwmLK = floor(255 * min(max(Power_levelLK, 0), 100) / 100) * 1.0;
const float Power_level_pwmRH = floor(255 * min(max(Power_levelRH, 0), 100) / 100) * 1.0;
const float Power_level_pwmRK = floor(255 * min(max(Power_levelRK, 0), 100) / 100) * 1.0;

const float pwm_constrains_mMLH[2] = { -Power_level_pwmLH, Power_level_pwmLH}; //These are additional constrains for the maximum and minimum pwm values (THE FIRST VALUE MUST BE THE LOWEST)
const float pwm_constrains_mMLK[2] = { -Power_level_pwmLK, Power_level_pwmLK}; //These are additional constrains for the maximum and minimum pwm values (THE FIRST VALUE MUST BE THE LOWEST)
const float pwm_constrains_mMRH[2] = { -Power_level_pwmRH, Power_level_pwmRH}; //These are additional constrains for the maximum and minimum pwm values (THE FIRST VALUE MUST BE THE LOWEST)
const float pwm_constrains_mMRK[2] = { -Power_level_pwmRK, Power_level_pwmRK}; //These are additional constrains for the maximum and minimum pwm values (THE FIRST VALUE MUST BE THE LOWEST)

const float Kp_pidLH = MultiplierLH * 2.0 ; //PROPORTIONAL Gain
const float Ki_pidLH = MultiplierLH * 3.0 * min(1.0, 1.0 / (100 * Sampling_time)); //INTEGRAL Gain
const float Kd_pidLH = MultiplierLH * 0 * Sampling_time; //DERIVATIVE Gain

const float Kp_pidLK = MultiplierLK * 3.0 ; //PROPORTIONAL Gain
const float Ki_pidLK = MultiplierLK * 5.0 * min(1.0, 1.0 / (100 * Sampling_time)); //INTEGRAL Gain
const float Kd_pidLK = MultiplierLK * 0 * Sampling_time; //DERIVATIVE Gain

const float Kp_pidRH = MultiplierRH * 2.0 ; //PROPORTIONAL Gain
const float Ki_pidRH = MultiplierRH * 3.0 * min(1.0, 1.0 / (100 * Sampling_time)); //INTEGRAL Gain
const float Kd_pidRH = MultiplierRH * 0 * Sampling_time; //DERIVATIVE Gain

const float Kp_pidRK = MultiplierRK * 3.0 ; //PROPORTIONAL Gain
const float Ki_pidRK = MultiplierRK * 5.0 * min(1.0, 1.0 / (100 * Sampling_time)); //INTEGRAL Gain
const float Kd_pidRK = MultiplierRK * 0.0 * Sampling_time; //DERIVATIVE Gain

const float Tc_Theta = max(0.02, 2 * Sampling_time); //Cutoff time (s) for angle's filter
const float Tc_Theta_p = max(0.05, 4 * Sampling_time); //Cutoff time(s) for velocity's filter
const float Tc_e_p = max(0.1, 2 * Sampling_time); //Cutoff time(s) for e_p's filter
const float Window_time_u = 0.1; //Window time (s) considered by the filter of the manipulation
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////PID CLASSES INSTANTIATION/////////////////////////////////////////////////
Time_L1 Time_regulation_Controller = Time_L1(Sampling_time);//Only one of these is needed, despite having 4 available. This is used for step movements
Time_L2 Time_regulation_Controller_R1 = Time_L2(1 * Sampling_time); // 4 Only one of these is needed, despite having 4 available. This is used for routines 1A and 1B
Time_L3 Time_regulation_Controller_R2 = Time_L3(1 * Sampling_time); // 2 Only one of these is needed, despite having 4 available. This is used for routines 1A and 1B


Preprocessor_L1 Preprocessing_LH = Preprocessor_L1(Sampling_time, Tc_Theta, Tc_Theta_p, 0, 0, stepspeed_default_LH);
Control_L1 PID_LH = Control_L1(Sampling_time, Kp_pidLH, Ki_pidLH, Kd_pidLH, pwm_constrains_mMLH[0], pwm_constrains_mMLH[1], Tc_e_p, pwm_minimum_movementLH[0],  pwm_minimum_movementLH[1]);
Power_L1 Execute_LH = Power_L1(d1PinLH, pwmPinLH);

Preprocessor_L2 Preprocessing_LK = Preprocessor_L2(Sampling_time, Tc_Theta, Tc_Theta_p, 0, 0, stepspeed_default_LK);
Control_L2 PID_LK = Control_L2(Sampling_time, Kp_pidLK, Ki_pidLK, Kd_pidLK, pwm_constrains_mMLK[0], pwm_constrains_mMLK[1], Tc_e_p, pwm_minimum_movementLK[0],  pwm_minimum_movementLK[1]);
Power_L2 Execute_LK = Power_L2(d1PinLK, pwmPinLK);

Preprocessor_L3 Preprocessing_RH = Preprocessor_L3(Sampling_time, Tc_Theta, Tc_Theta_p, 0, 0, stepspeed_default_RH);
Control_L3 PID_RH = Control_L3(Sampling_time, Kp_pidRH, Ki_pidRH, Kd_pidRH, pwm_constrains_mMRH[0], pwm_constrains_mMRH[1], Tc_e_p, pwm_minimum_movementRH[0],  pwm_minimum_movementRH[1]);
Power_L3 Execute_RH = Power_L3(d1PinRH, pwmPinRH);

Preprocessor_L4 Preprocessing_RK = Preprocessor_L4(Sampling_time, Tc_Theta, Tc_Theta_p, 0, 0, stepspeed_default_RK);
Control_L4 PID_RK = Control_L4(Sampling_time, Kp_pidRK, Ki_pidRK, Kd_pidRK, pwm_constrains_mMRK[0], pwm_constrains_mMRK[1], Tc_e_p, pwm_minimum_movementRK[0],  pwm_minimum_movementRK[1]);
Power_L4 Execute_RK = Power_L4(d1PinRK, pwmPinRK);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////


float time_out_handshake = 0.1;
int Flag_python = 1; // 0 = MANUAL,  1 = GUI


void setup() {
  // put your setup code here, to run once:
  pinMode(potPinLH, INPUT);
  pinMode(d1PinLH, OUTPUT);
  pinMode(pwmPinLH, OUTPUT);
  digitalWrite(d1PinLH, LOW);
  analogWrite(pwmPinLH, 0);

  pinMode(potPinLK, INPUT);
  pinMode(d1PinLK, OUTPUT);
  pinMode(pwmPinLK, OUTPUT);
  digitalWrite(d1PinLK, LOW);
  analogWrite(pwmPinLK, 0);

  pinMode(potPinRH, INPUT);
  pinMode(d1PinRH, OUTPUT);
  pinMode(pwmPinRH, OUTPUT);
  digitalWrite(d1PinRH, LOW);
  analogWrite(pwmPinRH, 0);

  pinMode(potPinRK, INPUT);
  pinMode(d1PinRK, OUTPUT);
  pinMode(pwmPinRK, OUTPUT);
  digitalWrite(d1PinRK, LOW);
  analogWrite(pwmPinRK, 0);
  pinMode(13, OUTPUT);    // sets the digital pin 13 as output

  //for rosserial
  //nh.getHardware()->setBaud(57600)
  nh.initNode();
  nh.advertiseService(server);
  nh.advertise(pub_potentiometer); 
  while(!nh.connected()) nh.spinOnce();
  
}

void loop() {
  // put your main code here, to run repeatedly:ù
  float Maximum_time_Step = 5.0; //this variable is defined also in the original code
  float Pub_angleRH;
  float Pub_angleLH;
  int potentiometer_pubLH = 1.0 * analogRead(potPinLH); //voltage meassurenment (10bits resolution);
  Pub_angleLH = Preprocessing_LH.PID_map_float(potentiometer_pubLH, Cal_valuesLH[0], Cal_valuesLH[1], Cal_valuesLH[2], Cal_valuesLH[3], Cal_valuesLH[5], Cal_valuesLH[5]); //estimate initial angle used to meassure the step increment
  int potentiometer_pubRH = 1.0 * analogRead(potPinRH); //voltage meassurenment (10bits resolution);
  Pub_angleRH = Preprocessing_RH.PID_map_float(potentiometer_pubRH, Cal_valuesRH[0], Cal_valuesRH[1], Cal_valuesRH[2], Cal_valuesRH[3], Cal_valuesRH[4], Cal_valuesRH[5]); //estimate initial angle used to meassure the step increment
  Pub_angleRH = Preprocessing_RH.PID_butter_filter_angle(Pub_angleRH, 1);//estimate initial is filtered 
  Pub_angleLH = Preprocessing_LH.PID_butter_filter_angle(Pub_angleLH, 1);//estimate initial is filtered
  float angle[2]={Pub_angleRH,Pub_angleLH};
  potentiometer_msg.data=angle;
  potentiometer_msg.data_length=2;
  pub_potentiometer.publish(&potentiometer_msg);

nh.spinOnce();
  delay(500);
}





/////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////
// Position_FM is an array containing the targets for final positions or rotations.
// Flag_forward_FM is an array containing the flags to identify the motors activation, 1 represents that motor will move.
// Max_time_FM is the maximum time for the movement, when this time is reached the whole movement is stoped.
// Flag_movement_type is the flag difinig if the motion is absolute (0) or relative(1)

/////////////////////////////////////////////////////////////////////////////////////

int Flexible_Movement(float * Position_FM, int* Flag_forward_FM, float Max_time_FM, int Flag_movement_type, float * Speed_array_FM, float * Limitstep_array_FM, float Deaccelaration_FM) {
  float Threshold_position[4] = {0.0, 0.0, 0.0, 0.0};
  if (Deaccelaration_FM >= 1) {
    Threshold_position[0] = ((Speed_array_FM[0] + Stopping_speed) / Deaccelaration_FM) * ((Speed_array_FM[0] - Stopping_speed) / 2);
    Threshold_position[1] = ((Speed_array_FM[1] + Stopping_speed) / Deaccelaration_FM) * ((Speed_array_FM[1] - Stopping_speed) / 2);
    Threshold_position[2] = ((Speed_array_FM[2] + Stopping_speed) / Deaccelaration_FM) * ((Speed_array_FM[2] - Stopping_speed) / 2);
    Threshold_position[3] = ((Speed_array_FM[3] + Stopping_speed) / Deaccelaration_FM) * ((Speed_array_FM[3] - Stopping_speed) / 2);
  }


  //-----------------------Initialization of variables for joints---------------------------//
  bool stop_condition;
  //---------------------------Limits of movements assignation------------------------------//
  float minlimitstepLH = Limitstep_array_FM[0];
  float maxlimitstepLH = Limitstep_array_FM[1];
  float minlimitstepLK = Limitstep_array_FM[2];
  float maxlimitstepLK = Limitstep_array_FM[3];
  float minlimitstepRH = Limitstep_array_FM[4];
  float maxlimitstepRH = Limitstep_array_FM[5];
  float minlimitstepRK = Limitstep_array_FM[6];
  float maxlimitstepRK = Limitstep_array_FM[7];

  float velocity_incLH;
  int reset_flagLH = 1; //reset flag used to erased previous values in the PID controller
  float sign_angleLH; //auxiliary variable used for stopping conditions in the movement

  int Flag_forwardLH = Flag_forward_FM[0];//Flag used to start and stop the movement of the motor regulated by completion of movement and limit positions
  float Initial_angleLH;
  potentiometer_valueLH = 1.0 * analogRead(potPinLH); //voltage meassurenment (10bits resolution);
  Initial_angleLH = Preprocessing_LH.PID_map_float(potentiometer_valueLH, Cal_valuesLH[0], Cal_valuesLH[1], Cal_valuesLH[2], Cal_valuesLH[3], Cal_valuesLH[5], Cal_valuesLH[5]); //estimate initial angle used to meassure the step increment
  Initial_angleLH = Preprocessing_LH.PID_butter_filter_angle(Initial_angleLH, 1);//estimate initial is filtered

  for (int i_ia = 0; i_ia < 3; i_ia++) {
    potentiometer_valueLH = 1.0 * analogRead(potPinLH); //voltage meassurenment (10bits resolution);
    Initial_angleLH = Preprocessing_LH.PID_map_float(potentiometer_valueLH, Cal_valuesLH[0], Cal_valuesLH[1], Cal_valuesLH[2], Cal_valuesLH[3], Cal_valuesLH[4], Cal_valuesLH[5]); //estimate initial angle used to meassure the step increment
    Initial_angleLH = Preprocessing_LH.PID_butter_filter_angle(Initial_angleLH, 0);//estimate initial is filtered
  }

  if (Flag_movement_type == 0) {
    if (Initial_angleLH <= Position_FM[0]) {
      velocity_incLH = Speed_array_FM[0];//angular velocity implemented in the steps
      sign_angleLH = 1.0;
    }
    else {
      velocity_incLH = -Speed_array_FM[0];//angular velocity implemented in the steps
      sign_angleLH = -1.0;
    }
  }
  else {
    if (Position_FM[0] > 0) {
      velocity_incLH = Speed_array_FM[0];//angular velocity implemented in the steps
      sign_angleLH = 1.0;
    }
    else {
      velocity_incLH = -Speed_array_FM[0];//angular velocity implemented in the steps
      sign_angleLH = -1.0;
    }
  }

  float velocity_incLK;
  int reset_flagLK = 1; //reset flag used to erased previous values in the PID controller
  float sign_angleLK; //auxiliary variable used for stopping conditions in the movement

  int Flag_forwardLK = Flag_forward_FM[1];//Flag used to start and stop the movement of the motor regulated by completion of movement and limit positions
  float Initial_angleLK;
  potentiometer_valueLK = 1.0 * analogRead(potPinLK); //voltage meassurenment (10bits resolution);
  Initial_angleLK = Preprocessing_LK.PID_map_float(potentiometer_valueLK, Cal_valuesLK[0], Cal_valuesLK[1], Cal_valuesLK[2], Cal_valuesLK[3], Cal_valuesLK[4], Cal_valuesLK[5]); //estimate initial angle used to meassure the step increment
  Initial_angleLK = Preprocessing_LK.PID_butter_filter_angle(Initial_angleLK, 1);//estimate initial is filtered
  for (int i_ia = 0; i_ia < 3; i_ia++) {
    potentiometer_valueLK = 1.0 * analogRead(potPinLK); //voltage meassurenment (10bits resolution);
    Initial_angleLK = Preprocessing_LK.PID_map_float(potentiometer_valueLK, Cal_valuesLK[0], Cal_valuesLK[1], Cal_valuesLK[2], Cal_valuesLK[3], Cal_valuesLK[4], Cal_valuesLK[5]);//estimate initial angle used to meassure the step increment
    Initial_angleLK = Preprocessing_LK.PID_butter_filter_angle(Initial_angleLK, 0);//estimate initial is filtered
  }


  if (Flag_movement_type == 0) {
    if (Initial_angleLK <= Position_FM[1]) {
      velocity_incLK = Speed_array_FM[1];//angular velocity implemented in the steps
      sign_angleLK = 1.0;
    }
    else {
      velocity_incLK = -Speed_array_FM[1];//angular velocity implemented in the steps
      sign_angleLK = -1.0;
    }
  }
  else {
    if (Position_FM[1] > 0) {
      velocity_incLK = Speed_array_FM[1];//angular velocity implemented in the steps
      sign_angleLK = 1.0;
    }
    else {
      velocity_incLK = -Speed_array_FM[1];//angular velocity implemented in the steps
      sign_angleLK = -1.0;
    }
  }

  float velocity_incRH;
  int reset_flagRH = 1; //reset flag used to erased previous values in the PID controller
  float sign_angleRH; //auxiliary variable used for stopping conditions in the movement

  int Flag_forwardRH = Flag_forward_FM[2];//Flag used to start and stop the movement of the motor regulated by completion of movement and limit positions
  float Initial_angleRH;
  potentiometer_valueRH = 1.0 * analogRead(potPinRH); //voltage meassurenment (10bits resolution);
  Initial_angleRH = Preprocessing_RH.PID_map_float(potentiometer_valueRH, Cal_valuesRH[0], Cal_valuesRH[1], Cal_valuesRH[2], Cal_valuesRH[3], Cal_valuesRH[4], Cal_valuesRH[5]); //estimate initial angle used to meassure the step increment
  Initial_angleRH = Preprocessing_RH.PID_butter_filter_angle(Initial_angleRH, 1);//estimate initial is filtered
  for (int i_ia = 0; i_ia < 3; i_ia++) {
    potentiometer_valueRH = 1.0 * analogRead(potPinRH); //voltage meassurenment (10bits resolution);
    Initial_angleRH = Preprocessing_RH.PID_map_float(potentiometer_valueRH, Cal_valuesRH[0], Cal_valuesRH[1], Cal_valuesRH[2], Cal_valuesRH[3], Cal_valuesRH[4], Cal_valuesRH[5]);//estimate initial angle used to meassure the step increment
    Initial_angleRH = Preprocessing_RH.PID_butter_filter_angle(Initial_angleRH, 0);//estimate initial is filtered
  }


  if (Flag_movement_type == 0) {
    if (Initial_angleRH <= Position_FM[2]) {
      velocity_incRH = Speed_array_FM[2];//angular velocity implemented in the steps
      sign_angleRH = 1.0;
    }
    else {
      velocity_incRH = -Speed_array_FM[2];//angular velocity implemented in the steps
      sign_angleRH = -1.0;
    }
  }
  else {
    if (Position_FM[2] > 0) {
      velocity_incRH = Speed_array_FM[2];//angular velocity implemented in the steps
      sign_angleRH = 1.0;
    }
    else {
      velocity_incRH = -Speed_array_FM[2];//angular velocity implemented in the steps
      sign_angleRH = -1.0;
    }
  }

  float velocity_incRK;
  int reset_flagRK = 1; //reset flag used to erased previous values in the PID controller
  float sign_angleRK; //auxiliary variable used for stopping conditions in the movement

  int Flag_forwardRK = Flag_forward_FM[3];//Flag used to start and stop the movement of the motor regulated by completion of movement and limit positions
  float Initial_angleRK;
  potentiometer_valueRK = 1.0 * analogRead(potPinRK); //voltage meassurenment (10bits resolution);
  Initial_angleRK = Preprocessing_RK.PID_map_float(potentiometer_valueRK, Cal_valuesRK[0], Cal_valuesRK[1], Cal_valuesRK[2], Cal_valuesRK[3], Cal_valuesRK[4], Cal_valuesRK[5]);//estimate initial angle used to meassure the step increment
  Initial_angleRK = Preprocessing_RK.PID_butter_filter_angle(Initial_angleRK, 1);//estimate initial is filtered
  for (int i_ia = 0; i_ia < 3; i_ia++) {
    potentiometer_valueRK = 1.0 * analogRead(potPinRK); //voltage meassurenment (10bits resolution);
    Initial_angleRK = Preprocessing_RK.PID_map_float(potentiometer_valueRK, Cal_valuesRK[0], Cal_valuesRK[1], Cal_valuesRK[2], Cal_valuesRK[3], Cal_valuesRK[4], Cal_valuesRK[5]);//estimate initial angle used to meassure the step increment
    Initial_angleRK = Preprocessing_RK.PID_butter_filter_angle(Initial_angleRK, 0);//estimate initial is filtered
  }

  if (Flag_movement_type == 0) {
    if (Initial_angleRK <= Position_FM[3]) {
      velocity_incRK = Speed_array_FM[3];//angular velocity implemented in the steps
      sign_angleRK = 1.0;
    }
    else {
      velocity_incRK = -Speed_array_FM[3];//angular velocity implemented in the steps
      sign_angleRK = -1.0;
    }
  }
  else {
    if (Position_FM[3] > 0) {
      velocity_incRK = Speed_array_FM[3];//angular velocity implemented in the steps
      sign_angleRK = 1.0;
    }
    else {
      velocity_incRK = -Speed_array_FM[3];//angular velocity implemented in the steps
      sign_angleRK = -1.0;
    }
  }

  Timeout_step(Max_time_FM, 1); //A timer is set to establish a stop in the movement (first value is the maximum time in SECONDS)
  //---------------------------------------------------------------------------------------------------//

  //-------------------------------Step movement while-loop starts---------------------------------//
  while (Flag_forwardLH + Flag_forwardLK + Flag_forwardRH + Flag_forwardRK > 0) {
    //-----The timer for termination is incremented and its value is checked-----//
    if (Timeout_step(Max_time_FM, 0)) {
      Execute_LH.PID_Power(0); //Implement pwm signal of 0 into the driver to stop movement
      Execute_LK.PID_Power(0); //Implement pwm signal of 0 into the driver to stop movement
      Execute_RH.PID_Power(0); //Implement pwm signal of 0 into the driver to stop movement
      Execute_RK.PID_Power(0); //Implement pwm signal of 0 into the driver to stop movement
      if (Flag_python == 0) {
        Serial.println("Timeout for Routine 1");
      }
      return 1;
      break;//The while-loop is terminated
    }
    //---------------------------------------------------------------------------//

    Time_regulation_Controller_R1.PID_Time(1);//Clock to regulate sampling time finishes

    //------------------------Control Algorithm execution for Left Hip------------------------//
    if (Flag_forwardLH == 1) {
      potentiometer_valueLH = 1.0 * analogRead(potPinLH); //voltage meassurenment with 10bits resolution
      angleLH = Preprocessing_LH.PID_map_float(potentiometer_valueLH, Cal_valuesLH[0], Cal_valuesLH[1], Cal_valuesLH[2], Cal_valuesLH[3], Cal_valuesLH[4], Cal_valuesLH[5]);//linear mapping is implemented to estimate angle
      angleLH = Preprocessing_LH.PID_butter_filter_angle(angleLH, 0);//angle value is filtered
      angular_velocityLH = Preprocessing_LH.PID_angle_derivative(angleLH, reset_flagLH); //angular velocity is estimated
      angular_velocityLH = Preprocessing_LH.PID_butter_filter_velocity(angular_velocityLH, reset_flagLH); //angular velocity is filtered
      pwm_valueLH = PID_LH.PID_Control(velocity_incLH, angular_velocityLH, Window_time_u, reset_flagLH);//PWM is computed by the control algorithm
      pwm_valueLH = directionflagLH * pwm_valueLH;//direction of the PWM is changed according to user definitions
    }
    else {
      pwm_valueLH = 0;
    }
    //---------------------------------------------------------------------------//

    //------------------------Terminating conditions for the movement are checked for Left Hip------------------------//
    if (Flag_movement_type == 0) {
      stop_condition = abs(angleLH - Position_FM[0]) <= 2;
    }
    else {
      stop_condition = (sign_angleLH * (angleLH - Initial_angleLH) >= abs(Position_FM[0]) - 2);
    }
    if (((stop_condition) || ((angleLH <= minlimitstepLH) && (velocity_incLH < 0)) || ((angleLH >= maxlimitstepLH) && (velocity_incLH > 0))) && (Flag_forwardLH == 1)) {
      Flag_forwardLH = 0;//The movement of the motor is stoped and the while loop will be terminated
      Execute_LH.PID_Power(0); //Implement pwm signal of 0 into the driver to stop movement
      if (Flag_python == 0) {
        Serial.print("Movement limit reached in Left Hip:");
        Serial.print(" ");
        Serial.print(angleLH);
        Serial.println(" degrees");
      }
    }
    else {
      Execute_LH.PID_Power(pwm_valueLH); //Implement pwm signal into the driver
      reset_flagLH = 0;//The PID will continue normal operation (previous values are preserved)
    }

    //------------------------Deaccelaration through speed manipulation----------------------------//
    if (Deaccelaration_FM >= 1) {
      if (abs(angular_velocityLH) >= Speed_array_FM[0]) {
        Threshold_position[0] = ((abs(angular_velocityLH)  + Stopping_speed) / Deaccelaration_FM) * ((abs(angular_velocityLH)  - Stopping_speed) / 2);
      }
      if (abs(angleLH - Position_FM[0]) <= Threshold_position[0]) {
        if (velocity_incLH > 0) {
          velocity_incLH = min(Speed_array_FM[0], sqrt(2 * abs(angleLH - Position_FM[0]) * Deaccelaration_FM + Stopping_speed * Stopping_speed));
        }
        else {
          velocity_incLH = -1.0 * min(Speed_array_FM[0], sqrt(2 * abs(angleLH - Position_FM[0]) * Deaccelaration_FM + Stopping_speed * Stopping_speed));
        }
      }
    }
    //--------------------------------------------------------------------------------------------//

    //------------------------Control Algorithm execution for Left Knee------------------------//
    if (Flag_forwardLK == 1) {
      potentiometer_valueLK = 1.0 * analogRead(potPinLK); //voltage meassurenment with 10bits resolution
      angleLK = Preprocessing_LK.PID_map_float(potentiometer_valueLK, Cal_valuesLK[0], Cal_valuesLK[1], Cal_valuesLK[2], Cal_valuesLK[3], Cal_valuesLK[4], Cal_valuesLK[5]); //linear mapping is implemented to estimate angle
      angleLK = Preprocessing_LK.PID_butter_filter_angle(angleLK, 0);//angle value is filtered
      angular_velocityLK = Preprocessing_LK.PID_angle_derivative(angleLK, reset_flagLK); //angular velocity is estimated
      angular_velocityLK = Preprocessing_LK.PID_butter_filter_velocity(angular_velocityLK, reset_flagLK); //angular velocity is filtered
      pwm_valueLK = PID_LK.PID_Control(velocity_incLK, angular_velocityLK, Window_time_u, reset_flagLK);//PWM is computed by the control algorithm
      pwm_valueLK = directionflagLK * pwm_valueLK;//direction of the PWM is changed according to user definitions
    }
    else {
      pwm_valueLK = 0;
    }
    //---------------------------------------------------------------------------//

    //------------------------Terminating conditions for the movement are checked for Left Knee------------------------//
    if (Flag_movement_type == 0) {
      stop_condition = abs(angleLK - Position_FM[1]) <= 2;
    }
    else {
      stop_condition = (sign_angleLK * (angleLK - Initial_angleLK) >= abs(Position_FM[1]) - 2);
    }
    if (((stop_condition) || ((angleLK <= minlimitstepLK) && (velocity_incLK < 0)) || ((angleLK >= maxlimitstepLK) && (velocity_incLK > 0))) && (Flag_forwardLK == 1)) {
      Flag_forwardLK = 0;//The movement of the motor is stoped and the while loop will be terminated
      Execute_LK.PID_Power(0); //Implement pwm signal of 0 into the driver to stop movement
      if (Flag_python == 0) {
        Serial.print("Movement limit reached in Left Knee:");
        Serial.print(" ");
        Serial.print(angleLK);
        Serial.println(" degrees");
      }
    }
    else {
      Execute_LK.PID_Power(pwm_valueLK); //Implement pwm signal into the driver
      reset_flagLK = 0;//The PID will continue normal operation (previous values are preserved)
    }

    //------------------------Deaccelaration through speed manipulation----------------------------//
    if (Deaccelaration_FM >= 1) {
      if (abs(angular_velocityLK) >= Speed_array_FM[1]) {
        Threshold_position[1] = ((abs(angular_velocityLK)  + Stopping_speed) / Deaccelaration_FM) * ((abs(angular_velocityLK)  - Stopping_speed) / 2);
      }
      if (abs(angleLK - Position_FM[1]) <= Threshold_position[1]) {
        if (velocity_incLK > 0) {
          velocity_incLK = min(Speed_array_FM[1], sqrt(2 * abs(angleLK - Position_FM[1]) * Deaccelaration_FM + Stopping_speed * Stopping_speed));
        }
        else {
          velocity_incLK = -1.0 * min(Speed_array_FM[1], sqrt(2 * abs(angleLK - Position_FM[1]) * Deaccelaration_FM + Stopping_speed * Stopping_speed));
        }
      }
    }
    //---------------------------------------------------------------------------------------------//

    //------------------------Control Algorithm execution for Right Hip------------------------//
    if (Flag_forwardRH == 1) {
      potentiometer_valueRH = 1.0 * analogRead(potPinRH); //voltage meassurenment with 10bits resolution
      angleRH = Preprocessing_RH.PID_map_float(potentiometer_valueRH, Cal_valuesRH[0], Cal_valuesRH[1], Cal_valuesRH[2], Cal_valuesRH[3], Cal_valuesRH[4], Cal_valuesRH[5]); //linear mapping is implemented to estimate angle
      angleRH = Preprocessing_RH.PID_butter_filter_angle(angleRH, 0);//angle value is filtered
      angular_velocityRH = Preprocessing_RH.PID_angle_derivative(angleRH, reset_flagRH); //angular velocity is estimated
      angular_velocityRH = Preprocessing_RH.PID_butter_filter_velocity(angular_velocityRH, reset_flagRH); //angular velocity is filtered
      pwm_valueRH = PID_RH.PID_Control(velocity_incRH, angular_velocityRH, Window_time_u, reset_flagRH);//PWM is computed by the control algorithm
      pwm_valueRH = directionflagRH * pwm_valueRH;//direction of the PWM is changed according to user definitions
    }
    else {
      pwm_valueRH = 0;
    }
    //---------------------------------------------------------------------------//

    //------------------------Terminating conditions for the movement are checked for Right Hip------------------------//
    if (Flag_movement_type == 0) {
      stop_condition = abs(angleRH - Position_FM[2]) <= 2;
    }
    else {
      stop_condition = (sign_angleRH * (angleRH - Initial_angleRH) >= abs(Position_FM[2]) - 2);
    }
    if (((stop_condition) || ((angleRH <= minlimitstepRH) && (velocity_incRH < 0)) || ((angleRH >= maxlimitstepRH) && (velocity_incRH > 0))) && (Flag_forwardRH == 1)) {
      Flag_forwardRH = 0;//The movement of the motor is stoped and the while loop will be terminated
      Execute_RH.PID_Power(0); //Implement pwm signal of 0 into the driver to stop movement
      if (Flag_python == 0) {
        Serial.print("Movement limit reached in Right Hip:");
        Serial.print(" ");
        Serial.print(angleRH);
        Serial.println(" degrees");
      }
    }
    else {
      Execute_RH.PID_Power(pwm_valueRH); //Implement pwm signal into the driver
      reset_flagRH = 0;//The PID will continue normal operation (previous values are preserved)
    }


    //------------------------Deaccelaration through speed manipulation----------------------------//
    if (Deaccelaration_FM >= 1) {
      if (abs(angular_velocityRH) >= Speed_array_FM[2]) {
        Threshold_position[2] = ((abs(angular_velocityRH)  + Stopping_speed) / Deaccelaration_FM) * ((abs(angular_velocityRH)  - Stopping_speed) / 2);
      }
      if (abs(angleRH - Position_FM[2]) <= Threshold_position[2]) {
        if (velocity_incRH > 0) {
          velocity_incRH = min(Speed_array_FM[2], sqrt(2 * abs(angleRH - Position_FM[2]) * Deaccelaration_FM + Stopping_speed * Stopping_speed));
        }
        else {
          velocity_incRH = -1.0 * min(Speed_array_FM[2], sqrt(2 * abs(angleRH - Position_FM[2]) * Deaccelaration_FM + Stopping_speed * Stopping_speed));
        }
      }
    }
    //--------------------------------------------------------------------------------------------//

    //------------------------Control Algorithm execution for Right Hip------------------------//
    if (Flag_forwardRK == 1) {
      potentiometer_valueRK = 1.0 * analogRead(potPinRK); //voltage meassurenment with 10bits resolution
      angleRK = Preprocessing_RK.PID_map_float(potentiometer_valueRK, Cal_valuesRK[0], Cal_valuesRK[1], Cal_valuesRK[2], Cal_valuesRK[3], Cal_valuesRK[4], Cal_valuesRK[5]); //linear mapping is implemented to estimate angle
      angleRK = Preprocessing_RK.PID_butter_filter_angle(angleRK, 0);//angle value is filtered
      angular_velocityRK = Preprocessing_RK.PID_angle_derivative(angleRK, reset_flagRK); //angular velocity is estimated
      angular_velocityRK = Preprocessing_RK.PID_butter_filter_velocity(angular_velocityRK, reset_flagRK); //angular velocity is filtered
      pwm_valueRK = PID_RK.PID_Control(velocity_incRK, angular_velocityRK, Window_time_u, reset_flagRK);//PWM is computed by the control algorithm
      pwm_valueRK = directionflagRK * pwm_valueRK;//direction of the PWM is changed according to user definitions
    }
    else {
      pwm_valueRK = 0;
    }
    //---------------------------------------------------------------------------//

    //------------------------Terminating conditions for the movement are checked for Left Knee------------------------//
    if (Flag_movement_type == 0) {
      stop_condition = abs(angleRK - Position_FM[3]) <= 2;
    }
    else {
      stop_condition = (sign_angleRK * (angleRK - Initial_angleRK) >= abs(Position_FM[3]) - 2);
    }
    if (((stop_condition) || ((angleRK <= minlimitstepRK) && (velocity_incRK < 0)) || ((angleRK >= maxlimitstepRK) && (velocity_incRK > 0))) && (Flag_forwardRK == 1)) {
      Flag_forwardRK = 0;//The movement of the motor is stoped and the while loop will be terminated
      Execute_RK.PID_Power(0); //Implement pwm signal of 0 into the driver to stop movement
      if (Flag_python == 0) {
        Serial.print("Movement limit reached in Right Knee:");
        Serial.print(" ");
        Serial.print(angleRK);
        Serial.println(" degrees");
      }
    }
    else {
      Execute_RK.PID_Power(pwm_valueRK); //Implement pwm signal into the driver
      reset_flagRK = 0;//The PID will continue normal operation (previous values are preserved)
    }
    //--------------------------------------------------------------------------------------------------//

    //------------------------Deaccelaration through speed manipulation----------------------------//
    if (Deaccelaration_FM >= 1) {
      if (abs(angular_velocityRK) >= Speed_array_FM[3]) {
        Threshold_position[3] = ((abs(angular_velocityRK) + Stopping_speed) / Deaccelaration_FM) * ((abs(angular_velocityRK)  - Stopping_speed) / 2);
      }
      if (abs(angleRK - Position_FM[3]) <= Threshold_position[3]) {
        if (velocity_incRK > 0) {
          velocity_incRK = min(Speed_array_FM[3], sqrt(2 * abs(angleRK - Position_FM[3]) * Deaccelaration_FM + Stopping_speed * Stopping_speed));
        }
        else {
          velocity_incRK = -1.0 * min(Speed_array_FM[3], sqrt(2 * abs(angleRK - Position_FM[3]) * Deaccelaration_FM + Stopping_speed * Stopping_speed));
        }
      }
    }
    //---------------------------------------------------------------------------------------------//

    Time_regulation_Controller_R1.PID_Time(2);//Clock to regulate sampling time finishes

  }
  return 0;
  //-------------------------------Step movement while loop ends---------------------------------//
}




//fuction to put the exo in the storage position
char Storing(){
// else if (Straight_key == 'n')
  float Final_position_1[4] = {80.0, -45.0, 80.0, -45.0};//(LH, LK, RH, RK)
  int Flag_forward_1[4] = {1, 1, 1, 1};//0 represents that motor will not be used (LH, LK, RH, RK)
  Flag_Movement=Flexible_Movement(Final_position_1, Flag_forward_1, 5.0, 0, Speed_array_default, Limit_array_default, General_deacceleration); //(position array, flag activation array, maximum time for movement)
   if (Flag_Movement == 1) { return '1';}
  else { return '0' ;}
}

//function to put the exo in the Stand position
char Stand_Straight()
{
   //if (Stand_exer_key == 'S') {
  float Final_position_1[4] = {0.0, 0.0, 0.0, 0.0};//(position array, flag activation array, maximum time for movement)
  int Flag_forward_1[4] = {1, 1, 1, 1};//(LH, LK, RH, RK)//0 represents that motor will not be used (LH, LK, RH, RK)
  float Speed_array_customized[4] = {40.0, 40.0, 40.0, 40.0};
  Flag_Movement=Flexible_Movement(Final_position_1, Flag_forward_1, 8.0, 0, Speed_array_customized, Limit_array_default, 0); //(position array, flag activation array, maximum time for movement)
   if (Flag_Movement == 1) { return '1';}
  else { return '0'; }
}

//function that is claimed in the different phase of the movement

//function for the first step of the walking

//ho dovuto definire anche flag_movement come variabile globale

//if (Gait_key == '1') {
char Step_One(){
float Final_position_1[4] = { -1.0, -1.0, 55.0, -45.0};//LH,LK,RH,RK. -1.0 represents that motor will not be used (this is just redundancy for the operator).
int Flag_forward_1[4] = {0, 0, 1, 1};//0 represents that motor will not be used (LH, LK, RH, RK)
Flag_Movement = Flexible_Movement(Final_position_1, Flag_forward_1, Maximum_time_Step, 0, Speed_array_default, Limit_array_default, General_deacceleration); //(position array, flag activation array, maximum time for movement)
 if (Flag_Movement == 1) { return '1';}
  else { return '0' ;}
}

char Step_Two(){
//if (Gait_key == '2') {
  float Final_position_1[4] = { -10.0, -1.0, 25.0, 0.0};//LH,LK,RH,RK. -1.0 represents that motor will not be used (this is just redundancy for the operator).
  int Flag_forward_1[4] = {1, 0, 1, 1};//0 represents that motor will not be used (LH, LK, RH, RK)
  Flag_Movement =  Flexible_Movement(Final_position_1, Flag_forward_1, Maximum_time_Step, 0, Speed_array_default, Limit_array_default, General_deacceleration); //(position array, flag activation array, maximum time for movement)
   if (Flag_Movement == 1) { return '1';}
  else { return '0' ;}
}

char Step_Three(){
//if (Gait_key == '3') {
  float Final_position_1[4] = { 30.0, -45.0, 0.0, -1.0};//LH,LK,RH,RK. -1.0 represents that motor will not be used (this is just redundancy for the operator).
  int Flag_forward_1[4] = {1, 1, 1, 0};//0 represents that motor will not be used (LH, LK, RH, RK)
  float Speed_array_customized[4] = {Speed_array_default[0] * 1.2, Speed_array_default[1], Speed_array_default[2], Speed_array_default[3]};
  Flag_Movement =  Flexible_Movement(Final_position_1, Flag_forward_1, Maximum_time_Step, 0, Speed_array_customized, Limit_array_default, General_deacceleration); //(position array, flag activation array, maximum time for movement)
   if (Flag_Movement == 1) { return '1';}
  else { return '0' ;}
}

char Step_Four()
{
  //if (Gait_key == '4') {
  float Final_position_1[4] = { 20.0, 0.0, -10.0, -1.0};//LH,LK,RH,RK. -1.0 represents that motor will not be used (this is just redundancy for the operator).
  int Flag_forward_1[4] = {1, 1, 1, 0};//0 represents that motor will not be used (LH, LK, RH, RK)
  Flag_Movement =  Flexible_Movement(Final_position_1, Flag_forward_1, Maximum_time_Step, 0, Speed_array_default, Limit_array_default, General_deacceleration); //(position array, flag activation array, maximum time for movement)
   if (Flag_Movement == 1) { return '1';}
  else { return '0' ;}

}

char Step_Five(){
//if (Gait_key == '5') {
  float Final_position_1[4] = { 0.0, 0.0, -1.0, -1.0};//LH,LK,RH,RK. -1.0 represents that motor will not be used (this is just redundancy for the operator).
  int Flag_forward_1[4] = {1, 1, 0, 0};//0 represents that motor will not be used (LH, LK, RH, RK)
  Flag_Movement =  Flexible_Movement(Final_position_1, Flag_forward_1, Maximum_time_Step, 0, Speed_array_default, Limit_array_default, General_deacceleration); //(position array, flag activation array, maximum time for movement)
  if (Flag_Movement == 1) { return '1';}
  else { return '0' ;}
}
/*
void callback (const Test::Request & req,Test::Response & res)
{

  // alcuni usano req.input e res.output
  if (req.input == 'n'){
  res.output=Storing();
  }
  else if (req.input == 's'){
  res.output=Stand_Straight();
  
  }
  else if (req.input == '1'){
  res.output=Step_One();
  
  }
  else if (req.input == '2'){
  res.output=Step_Two();
  
  }
  else if (req.input == '3'){
  res.output=Step_Three();
  
  }
  else if (req.input == '4'){
  res.output=Step_Four();
  
  }
  else if (req.input == '5'){
  res.output=Step_Five();
  
  }


}
*/


///////////This function detects if valid values have been introduced for the movement of a motor//////////////////
int correct_commands_step_validation(int direction_BF_ccsv) {
  int Flag_1 = 0;
  int number_valid_commands = sizeof(commands_for_menus) / sizeof(commands_for_menus[0]);//commands_for_menus should not be empty
  for (int i_ccsv = 0; i_ccsv < number_valid_commands; i_ccsv++) {
    if (direction_BF_ccsv == commands_for_menus[i_ccsv]) {
      Flag_1 = 1;
      break;
    }
  }
  return Flag_1;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////This function returns 1 when "Timer_ts surpassed" the threshold "threshold_ts" /////////
int Timeout_step(float threshold_ts, int reset_ts) {
  int Flag_timerout_stLK = 0;
  static float Timer_ts = 0;
  static float Time_stamp_ts = 0;
  if (reset_ts == 1) {
    Time_stamp_ts = micros();
    Timer_ts = 0;
  }
  else {
    Timer_ts = max(micros() - Time_stamp_ts, 0) / 1000000;//This part relies on the correct implementation of the  sampling time "Sampling_time"
  }

  if (Timer_ts >= threshold_ts) {
    Flag_timerout_stLK = 1;
  }
  else {
    Flag_timerout_stLK = 0;
  }
  return Flag_timerout_stLK;
}



int Manual_movement(int joint_calibration, bool info_MM) {
  //////////////////////////////////Left Hip Motor//////////////////////////////
  if ((joint_calibration == 'u') || (joint_calibration == 'y')) {

    int manualPWMLH;
    if (joint_calibration == 'u') {
      manualPWMLH = manualPWMALH;
    }
    else {
      manualPWMLH = manualPWMBLH;
    }
    Execute_LH.PID_Power(manualPWMLH);
    delay(300);
    Execute_LH.PID_Power(0);
    delay(500);
    potentiometer_valueLH = 1.0 * analogRead(potPinLH); //voltage meassurenment with 10bits resolution
    if (info_MM) {
      if (Flag_python == 0) {
        Serial.println("Pot Value LH: ");
        Serial.println(potentiometer_valueLH);
        Serial.println("Angle: ");
        Serial.println(Preprocessing_LH.PID_map_float(potentiometer_valueLH, minpotvalueLH, midpotvalueLH, maxpotvalueLH, minangleLH, midangleLH , maxangleLH));
        Serial.println(" ");
      }
    }
    return 1;
  }
  ///////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////Left Knee Motor/////////////////////////////
  else if ((joint_calibration == 'j') || (joint_calibration == 'h')) {

    int manualPWMLK;
    if (joint_calibration == 'j') {
      manualPWMLK = manualPWMALK;
    }
    else {
      manualPWMLK = manualPWMBLK;
    }
    Execute_LK.PID_Power(manualPWMLK);
    delay(300);
    Execute_LK.PID_Power(0);
    delay(500);
    potentiometer_valueLK = 1.0 * analogRead(potPinLK); //voltage meassurenment with 10bits resolution
    if (info_MM) {
      if (Flag_python == 0) {
        Serial.println("Pot Value LK: ");
        Serial.println(potentiometer_valueLK);
        Serial.println("Angle: ");
        Serial.println(Preprocessing_LK.PID_map_float(potentiometer_valueLK, minpotvalueLK, midpotvalueLK, maxpotvalueLK, minangleLK, midangleLK , maxangleLK));
        Serial.println(" ");
      }
    }
    return 1;
  }
  ///////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////Right Hip Motor//////////////////////////////
  else if ((joint_calibration == 'o') || (joint_calibration == 'i')) {
    int manualPWMRH;
    if (joint_calibration == 'o') {
      manualPWMRH = manualPWMARH;
    }
    else {
      manualPWMRH = manualPWMBRH;
    }
    Execute_RH.PID_Power(manualPWMRH);
    delay(300);
    Execute_RH.PID_Power(0);
    delay(500);
    potentiometer_valueRH = 1.0 * analogRead(potPinRH); //voltage meassurenment with 10bits resolution
    if (info_MM) {
      if (Flag_python == 0) {
        Serial.println("Pot Value RH: ");
        Serial.println(potentiometer_valueRH);
        Serial.println("Angle: ");
        Serial.println(Preprocessing_RH.PID_map_float(potentiometer_valueRH, minpotvalueRH, midpotvalueRH , maxpotvalueRH, minangleRH, midangleRH , maxangleRH));
        Serial.println(" ");
      }
    }
    return 1;
  }
  ///////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////Right Knee Motor//////////////////////////////
  else if ((joint_calibration == 'l') || (joint_calibration == 'k')) {
    int manualPWMRK;
    if (joint_calibration == 'l') {
      manualPWMRK = manualPWMARK;
    }
    else {
      manualPWMRK = manualPWMBRK;
    }
    Execute_RK.PID_Power(manualPWMRK);
    delay(300);
    Execute_RK.PID_Power(0);
    delay(500);
    potentiometer_valueRK = 1.0 * analogRead(potPinRK); //voltage meassurenment with 10bits resolution
    if (info_MM) {
      if (Flag_python == 0) {
        Serial.println("Pot Value RK: ");
        Serial.println(potentiometer_valueRK);
        Serial.println("Angle: ");
        Serial.println(Preprocessing_RK.PID_map_float(potentiometer_valueRK, minpotvalueRK, midpotvalueRK , maxpotvalueRK, minangleRK, midangleRK, maxangleRK));
        Serial.println(" ");
      }
    }
    return 1;
  }

  else {
    return 0;
  }
}


void Movement_with_check_points(float * Final_position_mwcp, int max_Cont_movement_mwcp, int *Flag_forward_mwcp, float Maximum_time_mwcp, int Flag_relative, float * Speed_array_mwcp, float * Limit_array_mwcp, float Deacceleration_mwcp, int extra_time_mwcp) {
  int Flag_Continue_movement_mwcp = 0;
  int Flag_Continue_movement_mwcp_aux = 0;
  int Cont_movement_mwcp = 0;
  float Final_position_movement_mwcp[4];//LH, LK, RH, RK
  int Flag_forward_step_mwcp[4];//LH, LK, RH, RK
  while ((Cont_movement_mwcp < max_Cont_movement_mwcp) && (Flag_Continue_movement_mwcp < 2)) {
    //Positions for each joint are assigned
    Final_position_movement_mwcp[0] = Final_position_mwcp[Cont_movement_mwcp * 4 + 0];//LH
    Final_position_movement_mwcp[1] = Final_position_mwcp[Cont_movement_mwcp * 4 + 1]; //LK
    Final_position_movement_mwcp[2] = Final_position_mwcp[Cont_movement_mwcp * 4 + 2]; //RH
    Final_position_movement_mwcp[3] = Final_position_mwcp[Cont_movement_mwcp * 4 + 3]; //RH

    Flag_forward_step_mwcp[0] = Flag_forward_mwcp[Cont_movement_mwcp * 4 + 0]; //LH
    Flag_forward_step_mwcp[1] = Flag_forward_mwcp[Cont_movement_mwcp * 4 + 1]; //LK
    Flag_forward_step_mwcp[2] = Flag_forward_mwcp[Cont_movement_mwcp * 4 + 2]; //RH
    Flag_forward_step_mwcp[3] = Flag_forward_mwcp[Cont_movement_mwcp * 4 + 3]; //RH

    Flag_Continue_movement_mwcp = Flag_Continue_movement_mwcp +
                                  Flexible_Movement(Final_position_movement_mwcp, Flag_forward_step_mwcp, Maximum_time_mwcp, Flag_relative, Speed_array_mwcp, Limit_array_mwcp, Deacceleration_mwcp); //(position array, flag activation array, maximum time for movement)

    if (Flag_Continue_movement_mwcp == 1) {
      if (Flag_Continue_movement_mwcp_aux == 1) {
        Flag_Continue_movement_mwcp = 0;
        Flag_Continue_movement_mwcp_aux = 0;
        Maximum_time_mwcp = 5.0;
      }
      else {
        if (extra_time_mwcp == 1) {
          Flag_Continue_movement_mwcp_aux = 1;
          Maximum_time_mwcp = 3.0;
        }
        else {
          Flag_Continue_movement_mwcp = 2;
        }
      }
    }
    if (Flag_Continue_movement_mwcp == 0) {
      if (Flag_python == 0) {
        Serial.print("Finished case: ");
        Serial.println(Cont_movement_mwcp);
      }
      Cont_movement_mwcp = Cont_movement_mwcp + 1;
      Maximum_time_mwcp = 5.0;
    }
  }
  if (Flag_Continue_movement_mwcp == 2) {
    for (int i_buffer = 0; i_buffer < 20; i_buffer++ ) {
      Serial.read();
    }
    if (Flag_python == 0) {
      Serial.println("An Error has occured");
    }
  }
}

void Recalibration(int Joint_R) {
  int iterations_R = 100;
  if (Joint_R == 'y') {
    minpotvalueLH = Average_meassurement_calibration(potPinLH, iterations_R); //voltage meassurenment (10bits resolution);
    minangleLH = -30; // (Degrees)
    midpotvalueLH = (minpotvalueLH + maxpotvalueLH) / 2; // 10-bit Resolution (0-1023)
    midangleLH = (minangleLH + maxangleLH) / 2; // (Degrees)
    if (Flag_python == 0) {
      Serial.println("To make change permanent");
      Serial.print("Modify the value of minpotvalueLH to:");
      Serial.println(minpotvalueLH);
      Serial.print("Modify the value of minangleLH to:");
      Serial.println(minangleLH);
    }
  }
  else if (Joint_R == 'u') {
    maxpotvalueLH = Average_meassurement_calibration(potPinLH, iterations_R); //voltage meassurenment (10bits resolution);
    maxangleLH = 110; // (Degrees)
    midpotvalueLH = (minpotvalueLH + maxpotvalueLH) / 2; // 10-bit Resolution (0-1023)
    midangleLH = (minangleLH + maxangleLH) / 2; // (Degrees)
    if (Flag_python == 0) {
      Serial.println("To make change permanent");
      Serial.print("Modify the value of maxpotvalueLH to:");
      Serial.println(maxpotvalueLH);
      Serial.print("Modify the value of maxangleLH to:");
      Serial.println(maxangleLH);
    }
  }
  else if (Joint_R == 'i') {
    minpotvalueRH = Average_meassurement_calibration(potPinRH, iterations_R); //voltage meassurenment (10bits resolution);
    minangleRH = -30; // (Degrees)
    midpotvalueRH = (minpotvalueRH + maxpotvalueRH) / 2; // 10-bit Resolution (0-1023)
    midangleRH = (minangleRH + maxangleRH) / 2; // (Degrees)
    if (Flag_python == 0) {
      Serial.println("To make change permanent");
      Serial.print("Modify the value of minpotvalueRH to:");
      Serial.println(minpotvalueRH);
      Serial.print("Modify the value of minangleRH to:");
      Serial.println(minangleRH);
    }
  }
  else if (Joint_R == 'o') {
    maxpotvalueRH  = Average_meassurement_calibration(potPinRH, iterations_R); //voltage meassurenment (10bits resolution);
    maxangleRH = 110; // (Degrees)
    midpotvalueRH = (minpotvalueRH + maxpotvalueRH) / 2; // 10-bit Resolution (0-1023)
    midangleRH = (minangleRH + maxangleRH) / 2; // (Degrees)
    if (Flag_python == 0) {
      Serial.println("To make change permanent");
      Serial.print("Modify the value of maxpotvalueLH to:");
      Serial.println(maxpotvalueRH);
      Serial.print("Modify the value of maxangleLH to:");
      Serial.println(maxangleRH);
    }
  }
  else if (Joint_R == 'h') {
    minpotvalueLK = Average_meassurement_calibration(potPinLK, iterations_R); //voltage meassurenment (10bits resolution);
    minangleLK = -30; // (Degrees)
    midpotvalueLK = (minpotvalueLK + maxpotvalueLK) / 2; // 10-bit Resolution (0-1023)
    midangleLK = (minangleLK + maxangleLK) / 2; // (Degrees)
    if (Flag_python == 0) {
      Serial.println("To make change permanent");
      Serial.print("Modify the value of minpotvalueLK to:");
      Serial.println(minpotvalueLK);
      Serial.print("Modify the value of minangleLK to:");
      Serial.println(minangleLK);
    }
  }
  else if (Joint_R == 'j') {
    maxpotvalueLK = Average_meassurement_calibration(potPinLK, iterations_R); //voltage meassurenment (10bits resolution);
    maxangleLK = 110; // (Degrees)
    midpotvalueLK = (minpotvalueLK + maxpotvalueLK) / 2; // 10-bit Resolution (0-1023)
    midangleLK = (minangleLK + maxangleLK) / 2; // (Degrees)
    if (Flag_python == 0) {
      Serial.println("To make change permanent");
      Serial.print("Modify the value of maxpotvalueLK to:");
      Serial.println(maxpotvalueLK);
      Serial.print("Modify the value of maxangleLK to:");
      Serial.println(maxangleLK);
    }
  }
  else if (Joint_R == 'k') {
    minpotvalueRK = Average_meassurement_calibration(potPinRK, iterations_R);  //voltage meassurenment (10bits resolution);
    minangleRK = -30; // (Degrees)
    midpotvalueRK = (minpotvalueRK + maxpotvalueRK) / 2; // 10-bit Resolution (0-1023)
    midangleRK = (minangleRK + maxangleRK) / 2; // (Degrees)
    if (Flag_python == 0) {
      Serial.println("To make change permanent");
      Serial.print("Modify the value of minpotvalueRK to:");
      Serial.println(minpotvalueRK);
      Serial.print("Modify the value of minangleRK to:");
      Serial.println(minangleRK);
    }
  }
  else if (Joint_R == 'l') {
    maxpotvalueRK = Average_meassurement_calibration(potPinRK, iterations_R); //voltage meassurenment (10bits resolution);
    maxangleRK = 110; // (Degrees)
    midpotvalueRK = (minpotvalueRK + maxpotvalueRK) / 2; // 10-bit Resolution (0-1023)
    midangleRK = (minangleRK + maxangleRK) / 2; // (Degrees)
    if (Flag_python == 0) {
      Serial.println("To make change permanent");
      Serial.print("Modify the value of maxpotvalueRK to:");
      Serial.println(maxpotvalueRK);
      Serial.print("Modify the value of maxangleRK to:");
      Serial.println(maxangleRK);
    }
  }
}

float Average_meassurement_calibration(int potpin_joint_am, int iterations_am) {
  float average = 0;
  for (int i_am = 0; i_am < iterations_am; i_am++) {
    average = average + 1.0 * analogRead(potpin_joint_am);
    delayMicroseconds(1000);
  }
  average = 1.0 * average / iterations_am;
  if (Flag_python == 0) {
    Serial.println("Done");
  }
  return average;
}




////////This function returns 1 when "Timer_ts surpassed" the threshold "threshold_ts" /////////
int Timeout_step2(float threshold_ts2, int reset_ts2) {
  int Flag_timerout_stLK2 = 0;
  static float Timer_ts2 = 0;
  static float Time_stamp_ts2 = 0;
  if (reset_ts2 == 1) {
    Time_stamp_ts2 = micros();
    Timer_ts2 = 0;
  }
  else {
    Timer_ts2 = max(micros() - Time_stamp_ts2, 0) / 1000000;//This part relies on the correct implementation of the  sampling time "Sampling_time"
  }

  if (Timer_ts2 >= threshold_ts2) {
    Flag_timerout_stLK2 = 1;
  }
  else {
    Flag_timerout_stLK2 = 0;
  }
  return Flag_timerout_stLK2;
}
/////////////////////////////////////////////////////////////////////////////////////

void delay_safe(int milliseconds_delay) {
  long iterations_ds = milliseconds_delay / 10;
  int remainder_ds = milliseconds_delay - iterations_ds * 10;
  if (iterations_ds > 0) {
    for (int i_sf = 0; i_sf < iterations_ds; i_sf++) {
      delayMicroseconds(10000);
    }
  }
  if (remainder_ds > 0) {
    delayMicroseconds(remainder_ds);
  }
}
