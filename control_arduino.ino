#include <Servo.h>
#include <Pixy2.h>

#define TRIG 5 //TRIG 핀 설정 (초음파 보내는 핀)
#define ECHO 4 //ECHO 핀 설정 (초음파 받는 핀)
#define FB_CONT 140
#define FB_EXT 10
#define SFB_CONT 120
#define SFB_EXT 10
#define FOOT_CONT 0
#define FOOT_EXT 170
#define ROT_LEFT 180
#define ROT_RIGHT 0
#define ROT_ALIGNED 90

#define TARGET_SIGNATURE 1

Servo fb; // front core
Servo sfb; // back core
Servo f_foot; // 앞발바닥모터
Servo b_foot; // 뒷발바닥모터
Servo rot; // 방향전환 모터 

int led = 3;

int sfb_motor = 6;
int fb_motor = 7;
int f_foot_motor = 8;
int b_foot_motor = 9;
int rot_motor = 10;

int fb_angle = FB_CONT; // 긴허리모터 정지 각도:140도 -> 가장 shrink
int sfb_angle = SFB_CONT; // 짧은 허리모터 정지 각도: 120도 -> 가장 shrink
int f_foot_angle = FOOT_CONT; // 앞발모터 초기 각도: 0도 -> 가장 shrink
int b_foot_angle = FOOT_CONT; // 뒷발모터 초기 각도: 0도 -> 가장 shrink
int rot_angle = 90; // 방향전환모터 초기 각도: 90도

float distance = 0;

Pixy2 pixy;

void setup() {
  fb.attach(fb_motor);
  sfb.attach(sfb_motor);
  f_foot.attach(f_foot_motor);
  b_foot.attach(b_foot_motor);
  rot.attach(rot_motor);

  fb.write(fb_angle);
  sfb.write(sfb_angle);
  f_foot.write(f_foot_angle);
  b_foot.write(b_foot_angle);
  rot.write(rot_angle);

  Serial.begin(9600);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(led, OUTPUT);

  pixy.init();
}

bool init_ = true;

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    executeCommand(command);
    sendData(); // 파이썬으로 초음파, x 데이터 전송
    if (init_) {
      delay(300);
      // 앞발 down
      f_foot_angle = FOOT_EXT;
      f_foot.write(f_foot_angle);
      delay(300);
      // 뒷발 down
      b_foot_angle = FOOT_EXT;
      b_foot.write(b_foot_angle);
      delay(300);
      init_ = false;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////

void executeCommand(String command) {
  if (command.startsWith("turn_left")) {
    // Find the positions of spaces in the command
    int firstSpace = command.indexOf(' ');                 // Between distance and left depth
    int secondSpace = command.indexOf(' ', firstSpace + 1); // Between left and right depth

    if (firstSpace != -1 && secondSpace != -1) {
      // Extract the angle, forward
      int angle = command.substring(firstSpace + 1, secondSpace).toInt();
      int forward = command.substring(secondSpace + 1).toInt();

      turn_left(angle, forward);
    }
  } else if (command.startsWith("turn_right")) {
    // Find the positions of spaces in the command
    int firstSpace = command.indexOf(' ');                 // Between distance and left depth
    int secondSpace = command.indexOf(' ', firstSpace + 1); // Between left and right depth

    if (firstSpace != -1 && secondSpace != -1) {
      // Extract the angle, forward
      int angle = command.substring(firstSpace + 1, secondSpace).toInt();
      int forward = command.substring(secondSpace + 1).toInt();

      turn_right(angle, forward);
    }
  } else if (command.startsWith("move_FRONT")) {
    move_FRONT();
  } else if (command.startsWith("move_front")) {
    // Find the positions of spaces in the command
    int firstSpace = command.indexOf(' ');                 // Between distance and left depth
    int secondSpace = command.indexOf(' ', firstSpace + 1); // Between left and right depth

    if (firstSpace != -1 && secondSpace != -1) {
      // Extract the distance, left depth, and right depth values
      float left_depth = command.substring(firstSpace + 1, secondSpace).toFloat();
      float right_depth = command.substring(secondSpace + 1).toFloat();

      move_front(left_depth, right_depth);
    }
  }
}


void align_rotation() {
  // Align rotation to the default position
  if (rot_angle > ROT_ALIGNED) {
    while (rot_angle > ROT_ALIGNED) {
      rot_angle -= 1; // Gradually reduce angle
      rot.write(rot_angle);
      delay(10); // Adjust delay for smoother movement
    }
  } else if (rot_angle < ROT_ALIGNED) {
    while (rot_angle < ROT_ALIGNED) {
      rot_angle += 1; // Gradually increase angle
      rot.write(rot_angle);
      delay(10); // Adjust delay for smoother movement
    }
  }
}

void turn_left(int angle, int forward) {
  // 앞발 up
  f_foot.attach(f_foot_motor);
  f_foot_angle = FOOT_CONT;
  f_foot.write(f_foot_angle);
  delay(300);

  // 방향 회전
  for (int i = 0; i < angle; i++) {
    if (rot_angle >= ROT_LEFT) {
      rot_angle = ROT_LEFT;
      break;
    }
    rot_angle += 1; // Increase angle
    rot.write(rot_angle);
    delay(10);
  }

  if (forward) {
    fb_angle = FB_EXT;
    fb.write(fb_angle);
    delay(300);
  }

  // 앞발 down
  f_foot_angle = FOOT_EXT;
  f_foot.write(f_foot_angle);
  delay(300);

  // 뒷발 up
  b_foot.attach(b_foot_motor);
  b_foot_angle = FOOT_CONT;
  b_foot.write(b_foot_angle);
  delay(300);

  if (forward) {
    fb_angle = FB_CONT;
    fb.write(fb_angle);
    delay(300);
  }
  // Align rotation 
  align_rotation();

  // 뒷발 down
  b_foot_angle = FOOT_EXT;
  b_foot.write(b_foot_angle);
  delay(300);
}

void turn_right(int angle, int forward) {
  // 앞발 up
  f_foot.attach(f_foot_motor);
  f_foot_angle = FOOT_CONT;
  f_foot.write(f_foot_angle);
  delay(300);

  // 방향 회전
  for (int i = 0; i < angle; i++) {
    if (rot_angle <= ROT_RIGHT) {
      rot_angle = ROT_RIGHT;
      break;
    }
    rot_angle -= 1; // Decrease angle
    rot.write(rot_angle);
    delay(10);
  }

  // front center
  if (forward) {
    fb_angle = FB_EXT;
    fb.write(fb_angle);
    delay(300);
  }

  // 앞발 down
  f_foot_angle = FOOT_EXT;
  f_foot.write(f_foot_angle);
  delay(300);

  // 뒷발 up
  b_foot_angle = FOOT_CONT;
  b_foot.write(b_foot_angle);
  delay(300);

  if (forward) {
    fb_angle = FB_CONT;
    fb.write(fb_angle);
    delay(300);
  }

  // Align rotation
  align_rotation();

  // 뒷발 down
  b_foot_angle = FOOT_EXT;
  b_foot.write(b_foot_angle);
  delay(300);
}

void move_FRONT(){
  // 앞발 up
  f_foot_angle = FOOT_CONT;
  f_foot.write(f_foot_angle);
  delay(300);

  // 가운데 늘리기
  fb_angle = FB_EXT;
  fb.write(fb_angle);
  delay(300);
  sfb_angle = SFB_EXT;
  sfb.write(sfb_angle);
  delay(300);

  // 앞발 down
  f_foot_angle = FOOT_EXT;
  f_foot.write(f_foot_angle);
  delay(300);

  // 뒷발 up
  b_foot_angle = FOOT_CONT;
  b_foot.write(b_foot_angle);
  delay(300);

  // 가운데 줄이기
  fb_angle = FB_CONT;
  fb.write(fb_angle);
  delay(300);
  sfb_angle = SFB_CONT;
  sfb.write(sfb_angle);
  delay(300);

  // 뒷발 down
  b_foot_angle = FOOT_EXT;
  b_foot.write(b_foot_angle);
  delay(300);
}

void move_front(float left_depth, float right_depth) {
  if (distance < 4) {
    if (left_depth - right_depth > 0) turn_left(40, 1);
    else turn_right(40, 1);
    return;
  }

  move_FRONT();
}

float readUltrasonicDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  float duration = pulseIn(ECHO, HIGH);
  float distance = duration * 0.034 / 2; // cm 단위 거리 계산
  return distance;
}

int x_of_block() {
  int x = -1;
  int detected = pixy.ccc.getBlocks();
  if (detected) {
    for (int i = 0; i < detected; i++) {
      // write when detect any object
      if (pixy.ccc.blocks[i].m_signature == TARGET_SIGNATURE) {
        x = pixy.ccc.blocks[i].m_x;
        analogWrite(led, 255);
      }
    }
  }
  else {
    analogWrite(led, 0);
  }
  return x;
}

void sendData() {
  distance = readUltrasonicDistance();
  int x = x_of_block();
  Serial.print(distance);
  Serial.print(" ");
  Serial.println(x);
}
