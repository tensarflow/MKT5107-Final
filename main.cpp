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
  PWMr.write(0.4f); // percentage
  PWMl.write(0.4f);
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

  while (1) {

    currCounter_right = counter_right.read();
    currCounter_left = counter_left.read();
    currTime = us;

    Dtick_right = currCounter_right - prevCounter_right;
    Dtick_left = currCounter_left - prevCounter_left;
    Dt = currTime - prevTime;

    if (Dtick_right > 0 || Dtick_left > 0) {

      Ddistance_right = double(Dtick_right) * DistancePerCount; // linear distance right wheel
      Ddistance_left = double(Dtick_left) * DistancePerCount; // linear distance left wheel
      Ddistance_center = (Ddistance_left + Ddistance_right) / 2.0f;   // linear distance center
      Dtheta = (Ddistance_right - Ddistance_left) / L; // delta angle

			Dx = Ddistance_left * cos(th);
			Dy = Ddistance_left * sin(th);
			printf("straight\n");
      

      // Update position and velocity
      x += Dx;
      y += Dy;
      th = fmod((th + Dtheta), TWO_PI);
      v_x = (Ddistance_center / Dt) * 100000000.0f; // Factor for conversion from m/µs to cm/s
      v_th = Dtheta / Dt;
    }

    prevCounter_right = currCounter_right;
    prevCounter_left = currCounter_left;
    prevTime = currTime;

    wait_us(20000);
		printf("%f \t %d \t %d \t", x, Dtick_right, Dtick_left);
		if ( x >= 2.0 ){
			printf("Reached goal! \n");
			stopVehicle();
			break; 
			}

  }
}