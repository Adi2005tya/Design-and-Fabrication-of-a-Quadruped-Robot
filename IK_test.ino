#include <math.h>
#include <SCServo.h>
//Link Constant&s
const float length_AB = 3.0;   //AB
const float length_AE = 2.5;   //AE
const float length_ED = 10.0;  //ED
const float length_DC = 3.3;   //DC
const float length_CB = 10;   //CB
const float length_CF = 12.5;  //CF
const float one_degree = 11.37777777777778;

const float offset1 = -100;
const float offset2 = -40;

//Motor Constraints
int speed = 700;
int accel = 70;
SMS_STS st;
#define S_RXD 18
#define S_TXD 19

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  delay(1000);
  int pos1, pos2;
  pos1 = st.ReadPos(1);
  if (pos1 != -1) {
    Serial.println(pos1);
  } else {
    Serial.println("Error");
  }
  pos2 = st.ReadPos(2);
  if (pos2 != -1) {
    Serial.println(pos2);
  } else {
    Serial.println("Error");
  }

  float angles_rads[2];
  IK_solver(18, 2, angles_rads);
  float theta1_deg = angles_rads[0] * (180/PI);
  float theta5_deg = angles_rads[1] * (180/PI);
  //go_to_zero();

  if(0 < pos1 < 2000){
    if((theta5_deg*one_degree) > pos2)
    {
      st.WritePosEx(2, (theta5_deg*one_degree) - pos2 - offset2, speed, accel);
      Serial.println((theta5_deg*one_degree) - pos2);
    }
    else
    {
      st.WritePosEx(2, -(theta5_deg*one_degree) + pos2 - offset2, speed, accel);
      Serial.println(-(theta5_deg*one_degree) + pos2);
    }
    if((theta1_deg*one_degree) > pos1)
    {
      st.WritePosEx(1, (theta1_deg*one_degree) - pos1 - offset1, speed, accel);
      Serial.println((theta1_deg*one_degree) - pos1);
    }
    else
    {
      st.WritePosEx(1,-(theta1_deg*one_degree) + pos1 - offset1 , speed, accel);
      Serial.println(-(theta1_deg*one_degree) + pos1);
    }
 
  }
  else{
    Serial.println("Joint Violation!!!");
  }
  Serial.println(theta1_deg);
  Serial.println(theta5_deg);

}
void loop() {

}
void go_to_zero() {
  st.WritePosEx(1, 0, speed, accel);
  st.WritePosEx(2, 0, speed, accel);
}
void IK_solver(float coord_X, float coord_Y, float* angles_rads) {

  float constant_K1 = (pow(coord_X, 2) + pow(length_CB, 2) + pow(length_AB, 2) + pow(coord_Y, 2) - pow(length_CF, 2) - 2 * coord_X * length_AB) / (2 * length_CB);

  //Solving for Theta5 and Theta6
  float theta5 = 2 * atan2(coord_Y + sqrt(pow((coord_X - length_AB), 2) + pow(coord_Y, 2) - pow(constant_K1, 2)), (constant_K1 + (coord_X - length_AB)));
  float theta6 = atan2((coord_X - length_CB * cos(theta5) - length_AB), (length_CB * sin(theta5) - coord_Y));

  //Finding Coordinates of D
  float x2 = coord_X - (length_DC + length_CF) * sin(theta6);
  float y2 = coord_Y + (length_DC + length_CF) * cos(theta6);

  float k2 = (pow(x2, 2) + pow(y2, 2) + pow(length_AE, 2) - pow(length_ED, 2)) / (2 * length_AE);

  //Solving for Theta1 and Theta2
  float theta1 = 2 * atan2((y2 - sqrt(pow(x2, 2) + pow(y2, 2) - pow(k2, 2))), (k2 - x2));
  float theta2 = atan2((y2 + length_AE * sin(theta1)), (x2 + length_AE * cos(theta1)));

  float x1 = -length_AE * cos(theta1);
  float y_1 = length_AE * sin(theta1);

  float x3 = length_CB * cos(theta5) + length_AB;
  float y3 = length_CB * sin(theta5);

  float x4 = length_AB;
  float y4 = 0;

  float x5 = 0;
  float y5 = 0;

  //Coordinates of all points
  // float _A[2] = { x5, y5 };
  // float B_B[2] = { x4, y4 };
  // float C_C[2] = { x1, y_1 };
  // float _D[2] = { x2, y2 };
  // float _E[2] = { x3, y3 };
  angles_rads[0] = theta1;
  angles_rads[1] = theta5;
  // Serial.print(theta1);
  // Serial.print(" : ");
  // Serial.println(theta5);
}

void FK_solver(float theta1, float theta5) {
  // %Section formula Ratios
  float section_m = length_DC;
  float section_n = length_CF;

  // %Coordinates of Point C
  float point_cx = length_CB * cos(theta5) + length_AB;
  float point_cy = length_CB * sin(theta5);

  // %Coordinates of Point E
  float point_ex = -length_AE * cos(theta1);
  float point_ey = length_AE * sin(theta1);

  float constant_A = 2 * (point_cx - point_ex);
  float constant_B = 2 * (point_cy - point_ey);
  float constant_K = (length_ED * length_ED) - (length_DC * length_DC) + (point_cx * point_cx) - (point_ex * point_ex) + (point_cy * point_cy) - (point_ey * point_ey);
  float constant_D = point_cx + point_ex;
  float constant_E = point_ey + point_cy;
  float constant_Z = (length_ED * length_ED) + (length_DC * length_DC) - (point_cx * point_cx) - (point_ex * point_ex) - (point_cy * point_cy) - (point_ey * point_ey);
  float constant_A_tilda = (2 * (constant_B * constant_B) / (constant_A * constant_A)) + 2;
  float constant_B_tilda = (2 * constant_D * constant_B / constant_A) - (4 * constant_K * constant_B / (constant_A * constant_A)) - (2 * constant_E);
  float constant_C_tilda = -(constant_Z + (2 * constant_D * constant_K / constant_A) - (2 * (constant_K * constant_K) / (constant_A * constant_A)));
  float charct_eqn = (constant_B_tilda * constant_B_tilda) - 4 * constant_A_tilda * constant_C_tilda;
  float solution_y1 = (-constant_B_tilda + sqrt(charct_eqn)) / (2 * constant_A_tilda);
  float solution_y2 = (-constant_B_tilda - sqrt(charct_eqn)) / (2 * constant_A_tilda);
  float solution_x1 = (constant_K - constant_B * solution_y1) / constant_A;
  float solution_x2 = (constant_K - constant_B * solution_y2) / constant_A;

  float X1 = ((point_cx * (section_m + section_n) - section_n * solution_x1) / section_m);
  float Y1 = ((point_cy * (section_m + section_n) - section_n * solution_y1) / section_m);
  float X2 = ((point_cx * (section_m + section_n) - section_n * solution_x2) / section_m);
  float Y2 = ((point_cy * (section_m + section_n) - section_n * solution_y2) / section_m);


  Serial.print(X1);
  Serial.print(",");
  Serial.print(Y1);
  Serial.print(":");

  Serial.print(X2);
  Serial.print(",");
  Serial.println(Y2);
}