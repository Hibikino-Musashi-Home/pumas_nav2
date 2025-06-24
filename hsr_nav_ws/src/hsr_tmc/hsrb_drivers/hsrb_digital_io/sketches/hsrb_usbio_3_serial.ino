const int kStatusLedRed = 9;
const int kStatusLedGreen = 10;
const int kStatusLedBlue = 11;

const int kHeadLed = 5;
const int kHandLed = 3;
const int kDrivePower = 4;
const int kSuction = 2;

bool prev_head_led = false;
bool prev_hand_led = false;
bool prev_drive_power = true;
bool prev_suction = false;

const int kMagneticSensor1 = 8;
const int kMagneticSensor2 = 7;
const int kEmergencySwitch = 6;
const int kPressureSensor = 14;
const int kFrontBumperSensor = 18;
const int kRearBumperSensor = 19;

bool front_bumper = false;
bool rear_bumper = false;

unsigned int front_bumper_count = 0;
unsigned int rear_bumper_count = 0;

// 上位から信号が途絶えたと判断する時間[ms]
const int kCeaseComunicate = 4000;
unsigned int count_timeout = 0;
// 点滅の待ち時間[ms]
const int kBlinkWait = 1000;
unsigned int count_blinking = 0;
bool is_led_on = true;

void setup() {
  pinMode(kStatusLedRed, OUTPUT);
  pinMode(kStatusLedGreen, OUTPUT);
  pinMode(kStatusLedBlue, OUTPUT);

  pinMode(kHeadLed, OUTPUT);
  pinMode(kHandLed, OUTPUT);
  pinMode(kDrivePower, OUTPUT);
  pinMode(kSuction, OUTPUT);

  pinMode(kMagneticSensor1, INPUT);
  pinMode(kMagneticSensor2, INPUT);
  pinMode(kEmergencySwitch, INPUT);
  pinMode(kFrontBumperSensor, INPUT);
  pinMode(kRearBumperSensor, INPUT);

  digitalWrite(kMagneticSensor1, HIGH);
  digitalWrite(kMagneticSensor2, HIGH);
  digitalWrite(kEmergencySwitch, HIGH);
  digitalWrite(kFrontBumperSensor, HIGH);
  digitalWrite(kRearBumperSensor, HIGH);

  digitalWrite(kDrivePower, HIGH);

  Serial.begin(57600);
}

void WriteCommand(int pin_id, bool& state) {
  byte command = (byte)Serial.read();
  if (command == 0 && state) {
    state = false;
    digitalWrite(pin_id, LOW);
  } else if (command == 1 && !state) {
    state = true;
    digitalWrite(pin_id, HIGH);
  }
}

void loop() {
  ++count_blinking;
  ++count_timeout;

  // バンパセンサは1[ms]周期で10回連続反応した場合のみpublishする
  if (digitalRead(kFrontBumperSensor)) {
    ++front_bumper_count;
    if (front_bumper_count >= 10) {
      front_bumper = true;
    }
  } else {
    front_bumper_count = 0;
  }
  if (digitalRead(kRearBumperSensor)) {
    ++rear_bumper_count;
    if (rear_bumper_count >= 10) {
      rear_bumper = true;
    }
  } else {
    rear_bumper_count = 0;
  }

  // 駆動系が出力されていない
  if (digitalRead(kEmergencySwitch)) {
    if (count_blinking % kBlinkWait == 0) {
      count_blinking = 0;
      is_led_on = !is_led_on;
    }
  } else {
    is_led_on = true;
  }

  if (Serial.available() >= 7) {
    count_timeout = 0;

    if (is_led_on) {
      analogWrite(kStatusLedRed, (byte)Serial.read());
      analogWrite(kStatusLedGreen, (byte)Serial.read());
      analogWrite(kStatusLedBlue, (byte)Serial.read());
    } else {
      analogWrite(kStatusLedRed, 0);
      analogWrite(kStatusLedGreen, 0);
      analogWrite(kStatusLedBlue, 0);
      (byte)Serial.read();
      (byte)Serial.read();
      (byte)Serial.read();
    }
    WriteCommand(kHeadLed, prev_head_led);
    WriteCommand(kHandLed, prev_hand_led);
    WriteCommand(kDrivePower, prev_drive_power);
    WriteCommand(kSuction, prev_suction);

    Serial.print(!digitalRead(kMagneticSensor1));
    Serial.print(!digitalRead(kMagneticSensor1));
    Serial.print(digitalRead(kEmergencySwitch));

    int pressure_analog = analogRead(kPressureSensor);
    // ポンプの吸引センサの閾値512digit(AD変換で2.5[V])
    bool pressure_digital = pressure_analog <= 512;

    Serial.print(pressure_digital);
    Serial.print(front_bumper);
    Serial.print(rear_bumper);

    front_bumper = false;
    rear_bumper = false;
  }

  // 上位からの信号が、一定時間来ていない(4000ms)
  if (count_timeout > kCeaseComunicate) {
    count_timeout = kCeaseComunicate;
    if (is_led_on) {
      analogWrite(kStatusLedRed, 255);
      analogWrite(kStatusLedGreen, 255);
      analogWrite(kStatusLedBlue, 255);
    } else {
      analogWrite(kStatusLedRed, 0);
      analogWrite(kStatusLedGreen, 0);
      analogWrite(kStatusLedBlue, 0);
    }
  }

  delay(1);
}
