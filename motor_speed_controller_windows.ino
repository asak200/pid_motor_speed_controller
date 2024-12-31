#define maxSpeed 1.4
#define minSpeed 0.2
#define maxPWM 255
#define minPWM 0

#define WHEEL_RAD 0.035
#define WHEEL_SAP 0.2
#define ENC_COUNT_PER_REV 946

// motor pins
int right_PWM_pin = 10;
int right_in3_pin = 8;
int right_in4_pin = 9;
int left_PWM_pin = 11;
int left_in1_pin = 12;
int left_in2_pin = 13;

float distance_per_count = 2*3.14159 * WHEEL_RAD / ENC_COUNT_PER_REV;

int encoder_Pin_1 = 2;
int encoder_Pin_2 = 3;
int encoder_Pin_3 = 4;
int encoder_Pin_4 = 5;
long prv_el, prv_er;
volatile long er, el;
volatile long lastEncoded, lastEncoded1;

double pt, t, dt;
float xtl, xtr, rev, vl, vr, svl, svr, xr, xl, dxl, dxr, ang, st_ang;
String str_sr = "0.00", str_sl = "0.00", psl, psr;
float sr, sl, read_sl, read_sr, prsl, prsr;
short v_c, sl_0, sr_0;

float leftPIDOutput, rightPIDOutput;
int leftPWM, rightPWM;

// PID constants
float kp = .8;
float ki = .8;
float kd = 0.001;

// PID structure
struct PID {
  float kp, ki, kd;
  float error, integral, derivative;
  float previousError;
};
PID leftPID, rightPID;

float calculatePID(PID &pid, float setpoint, float currentRead, float dtt) {
  pid.error = setpoint - currentRead;
  pid.integral += pid.error * dtt;
  pid.derivative = (pid.error - pid.previousError) / dtt;
  pid.previousError = pid.error;

  float output = pid.kp * pid.error + pid.ki * pid.integral + pid.kd * pid.derivative;
  if (setpoint == 0.0) output = 0;
  return constrain(output, -maxSpeed, maxSpeed);
}

// Function to calculate PWM from PID output
int calculatePWM(float pidOutput) {
  int pwm = 0;
  if (pidOutput > 0.00) {
    pwm = (pidOutput) * (maxPWM - minPWM) / (maxSpeed) + minPWM;
    pwm = constrain(pwm, minPWM, maxPWM);
  }
  else if (pidOutput < -0.00){
    pwm = (-pidOutput) * (maxPWM - minPWM) / (maxSpeed) + minPWM;
    pwm = constrain(pwm, minPWM, maxPWM);
  }
  return pwm;
}

void setup() {
  Serial.begin(9600);

  pinMode(encoder_Pin_1, INPUT);
  pinMode(encoder_Pin_2, INPUT);

  digitalWrite(encoder_Pin_1, HIGH); //turn pullup resistor on
  digitalWrite(encoder_Pin_2, HIGH); //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3)
  attachInterrupt(digitalPinToInterrupt(2), updateEncoder, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(3), updateEncoder, CHANGE);

  PCICR |= B00000100;			//Bit2 = 1 -> "PCIE2" enabeled (PCINT16 to PCINT23)
  PCMSK2 |= B00110000;	  // Enable D4 and D5

  for (int i=8; i<14; i++){
    pinMode(i, OUTPUT);
  }

  leftPID.kp = kp;
  leftPID.ki = ki;
  leftPID.kd = kd;
  rightPID.kp = kp;
  rightPID.ki = ki;
  rightPID.kd = kd;
  
  // analogWrite(left_PWM_pin, 100);
  // digitalWrite(left_in1_pin, 1);
  // digitalWrite(left_in2_pin, 0);
  // analogWrite(right_PWM_pin, 255);
  // digitalWrite(right_in3_pin, 0);
  // digitalWrite(right_in4_pin, 1);

}

void loop() {
  if (Serial.available()){
    String msg = Serial.readStringUntil('\n'); // to recieve
    if (msg[0] == 'v' && msg[1] == 's') get_vel(msg);
  }
  get_real_vel_pose();

  leftPIDOutput = calculatePID(leftPID, sl, vl, dt);
  rightPIDOutput = calculatePID(rightPID, sr, vr, dt);
  
  leftPWM = calculatePWM(leftPIDOutput);
  rightPWM = calculatePWM(rightPIDOutput);
  
  move_motors();
  
  
  Serial.print(vl);
  Serial.print("  ");
  Serial.print(vr);
  Serial.print("  ");
  Serial.print(sl);
  Serial.print("  ");
  Serial.print(sr);
  Serial.print("  ");
  Serial.print(leftPWM);
  Serial.print("  ");
  Serial.print(rightPWM);
  Serial.print("  ");
  Serial.print(xl);
  Serial.print("  ");
  Serial.print(xr);
  Serial.print("  ");
  Serial.println("");

  delay(5);

}
// vs: 1.00 0.50
void get_vel(String a){
  Serial.println(a);

  str_sl[0] = a[4]; str_sl[2] = a[6]; str_sl[3] = a[7];
  read_sl = str_sl.toFloat();
  if (read_sl > 1.4) read_sl = 1.4;
  if (a[3] == '-') read_sl *= -1;
  
  str_sr[0] = a[9]; str_sr[2] = a[11]; str_sr[3] = a[12];
  read_sr = str_sr.toFloat();
  if (read_sr > 1.4) read_sr = 1.4;
  if (a[8] == '-') read_sr *= -1;
  
  if (read_sl == 0) sl_0++;
  else {
    sl = read_sl;
    sl_0 = 0;  
  }
  if (sl_0 > 0){
    sl = read_sl;
    sl_0 = 0;
  }

  if (read_sr == 0) sr_0++;
  else {
    sr = read_sr;
    sr_0 = 0;  
  }
  if (sr_0 > 0){
    sr = read_sr;
    sr_0 = 0;
  }
  prsl = sl;
  prsr = sr;
}
void get_real_vel_pose(){
  // x = 2t (m)
  // rev = x /(2*pi*r) (rad)
  // enc = 2500 * rev
  // xtl = 0.6*t;
  // xtr = 0.3*t;
  
  // el = xtl /distance_per_count;
  // er = xtr /distance_per_count;

  // calculate dx and measure the displacement
  int del = el - prv_el;
  dxl = del * distance_per_count;
  prv_el = el;
  xl += dxl;

  int der = er - prv_er;
  dxr = der * distance_per_count;
  prv_er = er;
  xr += dxr;

  // calculate angle
  float angl = (dxr-dxl)/WHEEL_SAP * 180 / 3.14159;
  ang += angl;
  
  // get dt and calculate the velocity
  t = millis() / 1000.;
  dt = t - pt;
  pt = t;

  if (dt>0) {
    svl += dxl/dt;
    svr += dxr/dt;
    v_c++;
  }
  // get the mean every 5 velocity calculations
  if (v_c >= 5) {
    vl = svl/5; vr = svr/5; 
    svl=0; svr=0; 
    v_c=0;
  }
}
void move_motors(){
  analogWrite(left_PWM_pin, leftPWM);
  analogWrite(right_PWM_pin, rightPWM);

  if(sl == 0. && sr == 0.) stop();
  else if(leftPIDOutput > 0. && rightPIDOutput > 0.) forward();
  else if(leftPIDOutput < 0. && rightPIDOutput < 0.) backward();
  else if(leftPIDOutput < 0. && rightPIDOutput > 0.) left();
  else if(leftPIDOutput > 0. && rightPIDOutput < 0.) right();

}

// left wheel encoder
void updateEncoder(){
  int MSB = digitalRead(encoder_Pin_1); //MSB = most significant bit
  int LSB = digitalRead(encoder_Pin_2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) el --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) el ++;

  lastEncoded = encoded; //store this value for next time
}
// right wheel encoder
ISR (PCINT2_vect){
  int MSB = digitalRead(encoder_Pin_3); //MSB = most significant bit
  int LSB = digitalRead(encoder_Pin_4); //LSB = least significant bit

  int encoded1 = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded1 << 2) | encoded1; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) er --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) er ++;

  lastEncoded1 = encoded1; //store this value for next time
}
void stop(){
  digitalWrite(left_in1_pin, 0);
  digitalWrite(left_in2_pin, 0);
  digitalWrite(right_in3_pin, 0);
  digitalWrite(right_in4_pin, 0);
}
void forward(){
  digitalWrite(left_in1_pin, 1);
  digitalWrite(left_in2_pin, 0);
  digitalWrite(right_in3_pin, 1);
  digitalWrite(right_in4_pin, 0);
}
void backward(){
  digitalWrite(left_in1_pin, 0);
  digitalWrite(left_in2_pin, 1);
  digitalWrite(right_in3_pin, 0);
  digitalWrite(right_in4_pin, 1);
}
void left(){
  digitalWrite(left_in1_pin, 0);
  digitalWrite(left_in2_pin, 1);
  digitalWrite(right_in3_pin, 1);
  digitalWrite(right_in4_pin, 0);
}
void right(){
  digitalWrite(left_in1_pin, 1);
  digitalWrite(left_in2_pin, 0);
  digitalWrite(right_in3_pin, 0);
  digitalWrite(right_in4_pin, 1);
}