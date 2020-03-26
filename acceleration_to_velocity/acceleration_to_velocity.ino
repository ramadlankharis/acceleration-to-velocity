#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <SimpleKalmanFilter.h>

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
volatile float offset_x = 0, offset_y = 0;
float vx = 0.0;
int i = 0;
bool done_sampling = false;

//deklarasi variabel a - v
float ax_prev = 0.0, ax_cur = 0.0;
float kax_prev = 0.0, kax_cur = 0.0;
float ay_prev = 0.0, ay_cur = 0.0;
float vx_prev = 0.0, vx_cur = 0.0;
float vy_prev = 0.0, vy_cur = 0.0;
////deklarasi variabel a - v

SimpleKalmanFilter simpleKalmanFilter(0.01, 0.061, 0.01);
SimpleKalmanFilter simpleKalmanFiltery(0.01, 0.06l, 0.01);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  if (!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1);
  }
  sensors_event_t event;
  for (i = 0; i < 100; i++) {
    accel.getEvent(&event);
    offset_x += event.acceleration.x;
    offset_y += event.acceleration.y;
    delay(10);
  }
  i = 0;
  offset_x = offset_x / 100.0;
  offset_y = offset_y / 100.0;

  // Timer/Counter 1 initialization
  // Clock source: System Clock
  // Clock value: 2000.000 kHz
  // Mode: Normal top=0xFFFF
  // OC1A output: Disconnected
  // OC1B output: Disconnected
  // Noise Canceler: Off
  // Input Capture on Falling Edge
  // Timer Period: 10 ms
  // Timer1 Overflow Interrupt: On
  // Input Capture Interrupt: Off
  // Compare A Match Interrupt: Off
  // Compare B Match Interrupt: Off
  TCCR1A = (0 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0) | (0 << WGM11) | (0 << WGM10);
  TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << WGM13) | (0 << WGM12) | (0 << CS12) | (1 << CS11) | (0 << CS10);
  TCNT1H = 0xB1;
  TCNT1L = 0xE0;
  ICR1H = 0x00;
  ICR1L = 0x00;
  OCR1AH = 0x00;
  OCR1AL = 0x00;
  OCR1BH = 0x00;
  OCR1BL = 0x00;
  // Timer/Counter 1 Interrupt(s) initialization
  TIMSK1 = (0 << ICIE1) | (0 << OCIE1B) | (0 << OCIE1A) | (1 << TOIE1);
  // Timer/Counter 2 initialization
  // Clock source: System Clock
  // Clock value: 46.875 kHz
  // Mode: Ph. correct PWM top=OCR2A
  // OC2A output: Non-Inverted PWM
  // OC2B output: Non-Inverted PWM
  // Timer Period: 0 us
  // Output Pulse(s):
  // OC2A Period: 0 us
  // OC2B Period: 0 us
  ASSR = (0 << EXCLK) | (0 << AS2);
  TCCR2A = (1 << COM2A1) | (0 << COM2A0) | (1 << COM2B1) | (0 << COM2B0) | (0 << WGM21) | (1 << WGM20);
  TCCR2B = (1 << WGM22) | (1 << CS22) | (1 << CS21) | (0 << CS20);
  TCNT2 = 0x00;
  OCR2A = 0x00;
  OCR2B = 0x00;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (done_sampling) {
    sensors_event_t event;
    accel.getEvent(&event);
    //Kalman Proses
    float ax = event.acceleration.x - offset_x;
    float ay = event.acceleration.y - offset_y;
    //calculate the estimated value with Kalman Filter

    float kax = simpleKalmanFilter.updateEstimate(ax);
    float kay = simpleKalmanFiltery.updateEstimate(ay);

    float kax_ron = roundf(kax * 10.0) / 10.0; // round float to 1 decimal places mengurangi responsibilty accelero
    float kay_ron = roundf(kay * 10.0) / 10.0;
    ax_cur = kax_ron;
    ay_cur = kay_ron;

    vx_prev = vx_cur;
    vy_prev = vy_cur;
    vx_cur = (vx_prev + ((ax_prev + ax_cur) / 2) * 0.11);
    vy_cur = (vy_prev + ((ay_prev + ay_cur) / 2) * 0.11); //0.11 delta t
    ax_prev = ax_cur;
    ay_prev = ay_cur;
    done_sampling = false;
  }
}
ISR(TIMER1_OVF_vect) {

  // Reinitialize Timer1 value
  TCNT1H = 0xB1E0 >> 8;
  TCNT1L = 0xB1E0 & 0xff;
  // Place your code here
  done_sampling = true;
}
