#include "RTClib.h"
#include <DeepSleepScheduler.h>
#include <Servo.h>
#include <TimeLib.h>
#include <Time.h>
#include <DS1307RTC.h> // Real Time Clock Library

#define INTERRUPT_PIN 2
#define LED 13
#define SERVO 7

#define SERVO_OPENED_POS 110    // degree
#define SERVO_CLOSED_POS 0      // degree
#define SERVO_OPEN_TIME  5000   // millisec

int32_t WAKEUP_TIMES[] = { 3*60, 8*60, 15*60 }; // Minute in the day, ascending

Servo stirServo;
int isr_counts = 0;
int servo_pos = 0;
bool door_is_opened = false;

void schedule_next();
void isr();
void scheduled();
void open();
void close();
void button();
class DateTime rtc_read();
TimeSpan get_next_alarm_offset(DateTime now);

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(SERVO, OUTPUT); //test d'activation de pin pour le moteur
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  while (!Serial) ; // wait for serial
  delay(200);
  Serial.println("Arduino Starting");
  Serial.flush();
  servo_move_to(SERVO_CLOSED_POS); // reset servo to position 0
  schedule_next();
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), isr, FALLING);
}

void isr()  {
  isr_counts++;
  if(isr_counts == 1) { // debouncing
    scheduler.schedule(button);
  }
}

void button() {
  open();
  isr_counts=0;
}

void scheduled() {
  open();
  schedule_next();
}

void schedule_next() {
  DateTime now;
  TimeSpan next_alarm;

  //now = DateTime(2020, 12, 19, 7, 00, 00); // for testing
  now = rtc_read();
  next_alarm = get_next_alarm_offset(now);

  scheduler.scheduleDelayed(scheduled, next_alarm.totalseconds()*1000);
}

void loop() {
  scheduler.execute();
}

TimeSpan get_next_alarm_offset(DateTime now)
{
  int32_t next_wakeup_minute = WAKEUP_TIMES[0];
  int32_t current_minute;
  int32_t next_alarm_minute;
  int32_t offset_sec;
  char buffer[] = "DDD, DD MMM YYYY hh:mm:ss";

  now = now - TimeSpan(0, 0, 0, now.second());

  // Get current time and set alarm to a time to wake
  current_minute = now.minute() + now.hour() * 60;

  for(int idx = 0;idx < sizeof(WAKEUP_TIMES)/sizeof(*WAKEUP_TIMES); idx++) {
    if(current_minute < WAKEUP_TIMES[idx]) {
      next_wakeup_minute = WAKEUP_TIMES[idx];
      break;
    }
  }

  next_alarm_minute = next_wakeup_minute-current_minute;
  if(next_alarm_minute >= 0) {
    // Later today: now + delta
    offset_sec = next_alarm_minute*60;
  } else {
    // Tomorrow: now + 24h + (negative delta)
    offset_sec = (24L*60L+next_alarm_minute)*60;
  }

  Serial.print("Next alarm at: ");
  Serial.println((now + TimeSpan(offset_sec)).toString(buffer));
  delay(1000);
  return offset_sec;
}

void servo_move_to(int pos) {
  stirServo.attach(SERVO);
  stirServo.write(pos);
  delay(1000); // give time for the servo to reach position
  stirServo.detach();
}

void close() {
  if(!door_is_opened) return;
  Serial.println("Closing door...");
  servo_move_to(SERVO_CLOSED_POS);
  digitalWrite(LED, LOW);
  door_is_opened = false;
}

void open() {
  if(door_is_opened) return;
  digitalWrite(LED, HIGH);
  Serial.println("Opening door...");
  servo_move_to(SERVO_OPENED_POS);
  door_is_opened = true;
  scheduler.scheduleDelayed(close, SERVO_OPEN_TIME);
}

class DateTime rtc_read() {
  tmElements_t tm;
  DateTime now;
  char buffer[] = "DDD, DD MMM YYYY hh:mm:ss";
  if (RTC.read(tm)) {
  } else {
    if (RTC.chipPresent()) {
      Serial.println("The DS1307 is stopped.  Please run the SetTime");
      Serial.println("example to initialize the time and begin running.\n");
    } else {
      Serial.println("DS1307 read error!  Please check the circuitry.\n");
    }
  }
  now = DateTime(tm.Year-30 /* 1970vs2000 */, tm.Month, tm.Day, tm.Hour, tm.Minute, tm.Second);
  Serial.println(now.toString(buffer));
  return now;
}