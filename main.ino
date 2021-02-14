#include <RTClib.h>
#include <TimeLib.h>
#include <Time.h>
#include <DS1307RTC.h> // Required even if not used
#include <Servo.h>
#include <LowPower.h>

#define BUTTON_INTERRUPT_PIN 2
#define RTC_INTERRUPT_PIN 3
#define LED 13
#define SERVO 7

#define SERVO_OPENED_POS 110    // degree
#define SERVO_CLOSED_POS 0      // degree
#define SERVO_OPEN_TIME  5000   // millisec

int32_t WAKEUP_TIMES[] = { 3*60, 8*60, 15*60 }; // Minute in the day, ascending

Servo stirServo;
RTC_DS3231 rtc;

bool door_is_opened = false;

void isr();
void open();
void close();
DateTime get_next_alarm_time(DateTime now);

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(SERVO, OUTPUT);
  pinMode(BUTTON_INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(RTC_INTERRUPT_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  while (!Serial) ; // wait for serial
  delay(200);
  Serial.println("Arduino Starting");
  rtc = RTC_DS3231();
  servo_move_to(SERVO_CLOSED_POS); // reset servo to position 0
  set_next_alarm();
}

void isr()  {
  // doing nothing
}

void set_next_alarm() {
  char now_buf[] = "DDD, DD MMM YYYY hh:mm:ss";
  DateTime now = rtc.now();

  Serial.print("Current time: ");
  Serial.println(now.toString(now_buf));

  DateTime next_alarm = get_next_alarm_time(now);
  rtc.clearAlarm(1);
  rtc.setAlarm1(next_alarm, DS3231_A1_Date);
}

void loop() {
    char buf[] = "DDD, DD MMM YYYY hh:mm:ss";
    attachInterrupt(digitalPinToInterrupt(RTC_INTERRUPT_PIN), isr, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_INTERRUPT_PIN), isr, FALLING);

    Serial.println("Going to sleep");
    Serial.flush();
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

    // woke up
    detachInterrupt(digitalPinToInterrupt(RTC_INTERRUPT_PIN));
    detachInterrupt(digitalPinToInterrupt(BUTTON_INTERRUPT_PIN));
    DateTime now = rtc.now();
    Serial.print("Woke up at: ");
    Serial.println(now.toString(buf));

    open();
    delay(SERVO_OPEN_TIME);
    close();
    set_next_alarm();
}

DateTime get_next_alarm_time(DateTime now)
{
  int32_t next_wakeup_minute = WAKEUP_TIMES[0];
  int32_t current_minute;
  int32_t next_alarm_minute;
  int32_t offset_sec;
  char buffer[] = "DDD, DD MMM YYYY hh:mm:ss";
  DateTime next_alarm;

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
  next_alarm = now + TimeSpan(offset_sec);
  Serial.println(next_alarm.toString(buffer));
  return next_alarm;
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
}