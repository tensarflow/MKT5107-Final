#include "main.hpp"

bool DEBUG = 1;

// Create pins
PwmOut PWMr(PA_9);
DigitalOut RIN2(PA_11);
DigitalOut RIN1(PA_12);
DigitalOut STBY(PA_15);
DigitalOut LIN1(PB_3);
DigitalOut LIN2(PB_4);
PwmOut PWMl(PA_10);
Counter counter_left(PB_5);
Counter counter_right(PB_6);

// Vehicle-specific variables and constants
double DistancePerCount = (TWO_PI * 0.0315) / 20; // 2*PI*R/CPR  (WHEEL ENCODER 20 CPR)
double L = 0.135;
double x = 0.0;
double y = 0.0;
double th = 0.0;

// Initialize timer
volatile uint32_t us = 0;
Ticker us_tick;
void onMicrosecondTicker(void) { us = us + 100; }

void initPins() {
  PWMr.period(0.002f);
  PWMl.period(0.002f);
  RIN2 = RIN1 = LIN1 = LIN2 = 0;
  STBY = 1;
}

void turnForward() {
  RIN1 = 1;
  RIN2 = 0;
  LIN1 = 1;
  LIN2 = 0;
  PWMr.write(0.0f); // 0 = 0%, 1 = 100%
  PWMl.write(0.0f);
}

void setSpeedForward(float speed) {
	PWMr.write(speed); // percentage
  PWMl.write(speed);
}

void stopVehicle() {
  RIN1 = 0;
  RIN2 = 0;
  LIN1 = 0;
  LIN2 = 0;
  PWMr.write(0.0f); // percentage
  PWMl.write(0.0f);
}

int main() {
	printf("Starting program...\n");
	thread_sleep_for(3000);
  us_tick.attach_us(onMicrosecondTicker, 100);
  initPins();
  turnForward();

  // Initialize iteration variables
  int prevCounter_right = counter_right.read();
  int prevCounter_left = counter_left.read();
  int currCounter_right = counter_right.read();
  int currCounter_left = counter_left.read();
  int Dtick_right = 0;
  int Dtick_left = 0;
  double Ddistance_right = 0;
  double Ddistance_left = 0;
  double Ddistance_center = 0;
  double Dtheta = 0;
  double Dx = 0;
  double Dy = 0;
  double r_c;
  double icc_x;
  double icc_y;
  double v_x;
  double v_y = 0; // Differential drive kinematics has no y-velocity. Think about it :)
  double v_th;
  int currTime = us;
  int prevTime = us;
  int Dt = 0;
	bool isAcceleration = true;
	double setPosition_x = 2.0;
	double error = 0;
	double cumError = 0;
	double rateError = 0;
	double lastError = 0;
	double speed;
	double K_p = 1.5;
	double K_i = 0.0000000001;
	double K_d = 10;

  while (1) {

    currCounter_right = counter_right.read();
    currCounter_left = counter_left.read();
    currTime = us;

    Dtick_right = currCounter_right - prevCounter_right;
    Dtick_left = currCounter_left - prevCounter_left;
    Dt = currTime - prevTime;

		Ddistance_right = double(Dtick_right) * DistancePerCount; // linear distance right wheel
		Ddistance_left = double(Dtick_left) * DistancePerCount; // linear distance left wheel
		Ddistance_center = (Ddistance_left + Ddistance_right) / 2.0f;   // linear distance center
		// Dtheta = (Ddistance_right - Ddistance_left) / L; // delta angle commented out because only x-control 
		Dtheta = 0.0;

		Dx = Ddistance_left * cos(th);
		Dy = Ddistance_left * sin(th);
		
		// Update position and velocity
		x += Dx;
		y += Dy;
		th = fmod((th + Dtheta), TWO_PI);
		v_x = (Ddistance_center / Dt) * 100000000.0f; // Factor for conversion from m/µs to cm/s
		v_th = Dtheta / Dt;


		if(isAcceleration){
			speed = speed + 0.01f;
			setSpeedForward(speed);
			wait_us(2000);

			if(speed >= 0.4) isAcceleration = false; 
		}
		else{
			error = setPosition_x - x;
      cumError += error * Dt;
      rateError = (error - lastError) / Dt;
      
      //if (cumError >4000) cumError = 4000;
      //if (cumError <-4000) cumError = -4000;
      
			speed = K_p * error + K_i * cumError + K_d * rateError;
			if( speed > 1.0f ) { speed = 1.0f; }
			else if ( speed < 0.2f ) { speed = 0.2f; }

			setSpeedForward(speed);
			wait_us(20000);
		}

		prevCounter_right = currCounter_right;
		prevCounter_left = currCounter_left;
		prevTime = currTime;

		printf("%f\t %f\n", x, speed);

		if ( x >= 2.0 ){
			printf("Reached goal! \n");
			stopVehicle();
			break; 
		}
  }
}