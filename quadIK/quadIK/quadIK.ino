//#include <MeetAndroid.h>
#include "Math.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN 180
#define SERVOMAX 550

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

float coxa = 37, fermur= 70, tibia = 100;

float leg_initial [3][4] = {{ 75.66,  75.66, -75.66, -75.66},  // [axis] [leg no] 0=x, 1=y 2=z; 0 to 3
                            { 75.66, -75.66,  75.66, -75.66},
                            { -60  ,  -60, -60, -60}};
                           
float leg_pos [3][4] = {0};
float servo_pos [3][4] = {0}; //servo position after ik calculations
float servo_pos_i [3][4] = {0};  // initial servo position

float body_dim [2][4] = {{ 40, -40, 40, -40},
                         { 40,  40,-40, -40}};
                         
float inPut;                                                  
                           
void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(60);
  int i,j,k;
  
  for (k = 2; k < 14; k++){
        pwm.setPWM(k, 0,  365);  //all motors at 180
    }
  
  for (i = 0; i < 3; i++) {
    for ( j = 0; j < 4; j++){
       servo_pos_i [i][j] = PI/2 ;
       
    }
   }

}

void loop (){
  

  if(Serial.available()){
    inPut = Serial.parseFloat();
    Serial.print( "the input is: ");
    Serial.println( inPut);
  

  }
  /*if (inPut != 0){
    creepwalk ( inPut );
    //movebody (inPut, 0);
    //
    inPut =0;
  }*/
  Serial.println("loop");
  delay(1000);
 
  int state = 2;
  if (state == 1 ){
    creepwalk ( 80 );
  }
  if (state == 2 ){  
    single_step(0, -40, 2);
    single_step(0, 40, 0);
  }
  if (state == 3 ){  
    home_pos();
  }
  if (state == 4 ){  
    movebody(15,0);
  }
}

void single_step (float d_x, float d_y, int leg_no){
  int i;
  leg_pos[0][leg_no] += d_x/2;
  leg_pos[1][leg_no] += d_y/2;
  leg_pos[2][leg_no] += 50;

  transformation (leg_no);
  for (i = 0; i < 4; i++){
    if(i != leg_no){
      transformation (i);
    }
  } 
  motor_actuation();
  
  leg_pos[0][leg_no] += d_x/2;
  leg_pos[1][leg_no] += d_y/2;
  leg_pos[2][leg_no] -= 50;

  transformation (leg_no);
   for (i = 0; i < 4; i++){
    if(i != leg_no){
      transformation (i);
    }
  } 
  motor_actuation();
  
  leg_pos[0][leg_no] -= d_x;
  leg_pos[1][leg_no] -= d_y;
  transformation (leg_no);
  motor_actuation();
}
  
void creepwalk (float stride){
  
  Serial.println( "Creepwalk");
  //initalization steps
  float side_step = 30;
  
  //if(Serial.available()){
    
  //movebody( -side_step, 0 );
  movestep(-side_step, stride/2, 4-1 );  //leg =3
  movestep(0, stride/2, 2-1 );
  //movebody( side_step , 0);
   //}
  // walking
  //if(Serial.available()){
    //int i;
  //for ( i = 0; i <5; i ++){} //takes 5 cycles 
    
    while ( inPut != 3 ){
      //movebody ( side_step , 0);
      movestep (2.0*side_step,  stride, 3-1 );
      movestep (0,  stride, 1-1 );
    
      //movebody (-side_step*2 , 0);
      movestep (-2.0*side_step,  stride, 4-1 );
      movestep (0,  stride, 2-1 );
      //movebody ( side_step , 0 );
      if(Serial.available()){
        inPut = Serial.parseFloat();
        Serial.print( "the input is: ");
        Serial.println( inPut);
     }
    }

}
    
void movestep (float d_x, float d_y, int leg_no){  // takes step and moves the body in the direction of the step
  Serial.println( "movestep");
  float dx_body = 0;
  float dy_body = d_y/3;
  int i;
  //d_x = d_x + dx_body;
  //d_y = d_y + dy_body;
  leg_pos[0][leg_no] += 2*d_x/3; // step moves forward 2/3 and up
  leg_pos[1][leg_no] += 2*d_y/3;
  leg_pos[2][leg_no] += 65;
  transformation (leg_no);
  
  for (i = 0; i < 4; i++){
    if(i != leg_no){
      
      leg_pos[0][i] -= dx_body/2; // body moves forward half 1/3 of the step (refer to excel sheet
      leg_pos[1][i] -= dy_body/2;
      transformation (i);
    }
  } 
         
  smooth_actuation();
  
  leg_pos[0][leg_no] += d_x/3; // step moves forward 1/3 and down
  leg_pos[1][leg_no] += d_y/3;
  leg_pos[2][leg_no] -= 65;
  transformation (leg_no);
  
  for (i = 0; i < 4; i++){
    if(i != leg_no){
      
      leg_pos[0][i] -= dx_body/2;  // body moves forward next half 1/3 of the step (refer to excel sheet
      leg_pos[1][i] -= dy_body/2;
      transformation (i);
    }
  }
 smooth_actuation();
  
}

void movebody (float d_x, float d_y){
  int i;
    for (i = 0; i < 4; i++){
        
     leg_pos[0][i] += d_x; 
     leg_pos[1][i] += d_y;
     transformation (i);  
  }
  smooth_actuation ();
}

void home_pos (){
    
  int i, j;
  for (i = 0; i < 3; i++) {
    for ( j = 0; j < 4; j++){
       leg_pos [i][j] = 0;
       servo_pos [i][j] = 0; 
    }
  } 
}

void body_rotation (float alpha, float beta, float gamma){
  
  float i, x, y, z, x_r, y_r, z_r;
  int leg_no;
  
  for ( i = 0; i<4; i++){
    x = leg_initial [0][leg_no] +body_dim [0][leg_no];
    y = leg_initial [1][leg_no] +body_dim [1][leg_no];
    z = leg_initial [2][leg_no]; 
    
    x_r = x*cos(beta)*cos(gamma)- y*cos(beta)*sin(gamma) + z*sin(beta) -x;
    y_r = x*(sin(alpha)*sin(beta)*cos(gamma)+cos(alpha)*sin(gamma)) + y * (-sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma)) - z * (sin(alpha)*cos(beta)) - y;
    z_r = x*(-cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*cos(gamma)) + y * (cos(alpha)*sin(beta)*sin(gamma)+sin(alpha)*cos(gamma)) + z * (cos(alpha)*cos(beta)) - z;
    
    leg_pos[0][leg_no] += x_r;
    leg_pos[1][leg_no] += y_r;
    leg_pos[2][leg_no] -= z_r; //because z is negative 
    
    transformation (i);
  }
  
  motor_actuation ();
}
  
void transformation (int leg_no){ //leg no  0,1,2,3 tranforms body coord to leg coord
  //Serial.println( "trans");
  float x, y, z, t_angle, t_x, t_y, t_z;
  x = leg_pos[0][leg_no]+leg_initial [0][leg_no];
  y = leg_pos[1][leg_no]+leg_initial [1][leg_no];
  z = leg_pos[2][leg_no]+leg_initial [2][leg_no];
  
  /*int i, j;
  for (i = 0; i < 3; i++) {
    for ( j = 0; j < 4; j++){
      Serial.print(leg_pos[i][j]);
      Serial.print("\t");
    }
    Serial.print("\n");
   }*/
  /*t_angle = leg_no*PI/2 +PI/4; */ // this is smart fix it later 

  switch (leg_no){
    case 0:
      t_angle = PI/4;
      
      break;
    case 1:
      t_angle = PI*7/4;
      x = -1.0*leg_pos[0][leg_no]+leg_initial [0][leg_no];
      y = -1.0*leg_pos[1][leg_no]+leg_initial [1][leg_no];
      break;
    case 2:
      t_angle = PI*3/4;
      x = -1.0*leg_pos[0][leg_no]+leg_initial [0][leg_no];
      y = -1.0*leg_pos[1][leg_no]+leg_initial [1][leg_no];
      break;
    case 3:
      t_angle = PI*5/4;
      break;
    default:
      Serial.print("something is wrong");
  }
  t_x = x * cos( t_angle ) + y * sin (t_angle); 
  t_y = x * cos( t_angle ) - y * sin (t_angle);
  t_z = z;
  Serial.println("Post transformation");
  Serial.print( leg_no); Serial.print("\t"); //test printing
  Serial.print( t_angle); Serial.print("\t");
  Serial.print(  t_x ); Serial.print("\t");
  Serial.print(  t_y ); Serial.print("\t");
  Serial.print(  t_z ); Serial.print("\t");
  Serial.print("\n"); 
  inverse_kinametic ( t_x, t_y, t_z, leg_no);

}
  
  

void inverse_kinametic ( float x, float y, float z, int leg_no){ //inputs transformed cordinates to convert them to servo angles
  //Serial.println( "IK");
  float z_offset = abs(z);
  float gamma = atan ( y / x );
  float x_proj = x / cos(gamma);
  float coxatofoot = sqrt( pow(z_offset, 2) + pow((x_proj- coxa), 2));
  float ika_1 = acos( z_offset / coxatofoot);
  float ika_2 = acos((pow(tibia,2 ) - pow(fermur, 2)- pow(coxatofoot, 2))/(-2*fermur*coxatofoot));
  float alpha = ika_1 + ika_2;
  float beta = acos((pow(coxatofoot,2 ) - pow(tibia, 2)- pow(fermur, 2))/(-2*tibia*fermur));
  
  /*Serial.print( leg_no); Serial.print("\t"); //test printing
  Serial.print( alpha ); Serial.print("\t");
  Serial.print( beta ); Serial.print("\t");
  Serial.print( gamma ); Serial.print("\t");
  Serial.print("\n");*/
  
  // all servo position are in radians and need to be convirted to PWM signals
  if ( leg_no == 1 || leg_no == 2){
    servo_pos [0][leg_no] = -gamma + PI/2; // compensates for opposite motor positions-NO IDEA HOW THIS WORKS
  }
  else{
    servo_pos [0][leg_no] = gamma + PI/2;
  }
  servo_pos [1][leg_no] = alpha ;  
  servo_pos [2][leg_no] = beta ;
 
  /*Serial.print(  servo_pos [0][leg_no] ); Serial.print("\t");
  Serial.print(  servo_pos [1][leg_no] ); Serial.print("\t");
  Serial.print(  servo_pos [2][leg_no] ); Serial.print("\t");
  Serial.print("\n");*/
  
}
void smooth_actuation(){
  float servo_increment [3][4] = {0};
  float delay_step = 5;
  int i, k, l;
  for (k = 0; k < 3; k++) {
    for ( l = 0; l < 4; l++){
       servo_increment [k][l] = (servo_pos [k][l] - servo_pos_i [k][l]) / delay_step ;
       
    }
   }
   
  int m;
  for (m = 1 ; m < delay_step; m++){
    for (k = 0; k < 3; k++) {
      for ( l = 0; l < 4; l++){
        servo_pos_i [k][l] = servo_pos_i [k][l] + servo_increment [k][l] ;
      }
    }
    int i,j;
    Serial.print(m);
    Serial.println( "actuation");
    j = 0; 
    for( i = 2; i < 5; i++){
    
      pwm.setPWM(i, 0, mapfloat(servo_pos_i[j][0], 0, PI, 180, 550)); //leg 1  
      j++;
    }
    j = 0;
    for( i = 5; i < 8; i++){
    
      pwm.setPWM(i, 0, mapfloat(servo_pos_i[j][1], 0, PI, 180, 550)); //leg 2
      j++;
    }
    j = 0;
    for( i = 8; i < 11; i++){
    
      pwm.setPWM(i, 0, mapfloat(servo_pos_i[j][2], 0, PI, 180, 550));  // leg 3
      j++;
    }
 
    j = 0;
    for( i = 11; i < 14; i++){
   
      pwm.setPWM(i, 0, mapfloat(servo_pos_i[j][3], 0, PI, 180, 550));  // leg 4
      j++;
    }
    delay(50);
  } 
  motor_actuation();
  for (k = 0; k < 3; k++) {  //calibration of servo_pos_i with servo_pos
      for ( l = 0; l < 4; l++){
        servo_pos_i [k][l] = servo_pos [k][l];
      }
    }
   
}
  
void motor_actuation(){
  
  int i,j;
  Serial.println( "actuation");
  j = 0; 
  for( i = 2; i < 5; i++){
    
    pwm.setPWM(i, 0, mapfloat(servo_pos[j][0], 0, PI, 180, 550)); //leg 1  
    j++;
  }
  j = 0;
  for( i = 5; i < 8; i++){
    
    pwm.setPWM(i, 0, mapfloat(servo_pos[j][1], 0, PI, 180, 550)); //leg 2
    j++;
  }
  j = 0;
  for( i = 8; i < 11; i++){
    
    pwm.setPWM(i, 0, mapfloat(servo_pos[j][2], 0, PI, 180, 550));  // leg 3
    j++;
  }
 
  j = 0;
  for( i = 11; i < 14; i++){
   
    pwm.setPWM(i, 0, mapfloat(servo_pos[j][3], 0, PI, 180, 550));  // leg 4
    j++;
  }
     
    delay(1000);
    
    /*int k, l;
    for (k = 0; k < 3; k++) {
    for ( l = 0; l < 4; l++){
      Serial.print(mapfloat( servo_pos[k][l], 0, PI, 0, 180));
      Serial.print("\t");
    }
    Serial.print("\n");
   }*/
  int k, l;
  for(k = 0; k < 3; k++) {
    for ( l = 0; l < 4; l++){
      Serial.print(leg_pos[k][l]);
      Serial.print("\t");
    }
    Serial.print("\n");
   }
    
} 


  
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
