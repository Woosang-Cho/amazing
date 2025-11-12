// Tremaux-based Maze Explorer with 2 status LEDs
// UNO + 2 DC motors (L298N/L293D) + 3x HC-SR04 (Left/Front/Right) + 2 LEDs
// LED policy: sumFlags=0 -> both OFF, sumFlags=1 -> LED1 ON, sumFlags>=2 -> LED2 ON
// Exit detected -> both ON

// ---- Motor pins (edit if needed)
#define IN1 5   // Motor A PWM
#define IN2 4   // Motor A DIR
#define IN3 6   // Motor B PWM
#define IN4 7   // Motor B DIR

// ---- Ultrasonic pins: trig/echo
#define TRIG_L 8
#define ECHO_L 9
#define TRIG_F 10
#define ECHO_F 11
#define TRIG_R 12
#define ECHO_R 13

// ---- LED pins
#define LED1 2  // LSB indicator
#define LED2 3  // MSB indicator

// ---- Motion parameters (tune once for your chassis)
int PWM_FWD_L = 150;
int PWM_FWD_R = 160;
int PWM_TURN   = 160;
unsigned long TURN_90_MS   = 340;
unsigned long STEP_FWD_MS  = 120;
unsigned long COAST_MS     = 20;

// ---- Sensing thresholds (cm)
int front_th      = 18;
int side_th       = 18;
int deadend_clear = 12;

// ---- Tremaux edge flags: 2-bit per direction (L,F,R) -> 00=unseen, 01=once, 10=twice
struct EdgeMark {
  uint8_t key;    // local node key
  uint8_t flags;  // uses 6 bits: [LL|FF|RR]
};

#define TABLE_SIZE 128
EdgeMark marks[TABLE_SIZE];

uint8_t heading = 0;        // 0=N,1=E,2=S,3=W
uint32_t path_len_ticks = 0;

// ---- Ultrasonic ping (cm)
unsigned long pingCM(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long dur = pulseIn(echo, HIGH, 25000UL);
  if (dur == 0) return 400;
  return dur / 58UL;
}

// ---- Edge flag helpers
uint8_t getFlag(uint8_t flags, uint8_t dir) {
  return (flags >> (dir*2)) & 0x3;
}
uint8_t setFlag(uint8_t flags, uint8_t dir, uint8_t val) {
  uint8_t mask = ~(0x3 << (dir*2));
  flags = (flags & mask) | ((val & 0x3) << (dir*2));
  return flags;
}

// ---- Local node key (coarse)
uint8_t makeLocalKey(uint8_t heading_bucket, uint16_t tick_mod) {
  return (uint8_t)((heading_bucket & 0x3) | ((tick_mod & 0x3F) << 2));
}

// ---- Open addressing
int getOrCreate(uint8_t key) {
  int idx = key & (TABLE_SIZE-1);
  for (int i=0;i<TABLE_SIZE;i++) {
    int j = (idx + i) & (TABLE_SIZE-1);
    if (marks[j].flags == 0xFF) { marks[j].key = key; marks[j].flags = 0; return j; }
    if (marks[j].key == key) return j;
  }
  return -1;
}

// ---- Motors
void motorsStop() {
  analogWrite(IN1, 0); analogWrite(IN3, 0);
}
void motorsForward() {
  digitalWrite(IN2, HIGH);
  digitalWrite(IN4, HIGH);
  analogWrite(IN1, PWM_FWD_L);
  analogWrite(IN3, PWM_FWD_R);
}
void turnLeft90() {
  digitalWrite(IN2, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(IN1, PWM_TURN);
  analogWrite(IN3, PWM_TURN);
  delay(TURN_90_MS);
  motorsStop(); delay(COAST_MS);
  heading = (heading + 3) & 3;
}
void turnRight90() {
  digitalWrite(IN2, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(IN1, PWM_TURN);
  analogWrite(IN3, PWM_TURN);
  delay(TURN_90_MS);
  motorsStop(); delay(COAST_MS);
  heading = (heading + 1) & 3;
}
void uTurn() { turnLeft90(); turnLeft90(); }

void stepForward() {
  motorsForward();
  delay(STEP_FWD_MS);
  motorsStop();
  delay(COAST_MS);
  path_len_ticks++;
}

// ---- LED status update
void setLEDs(uint8_t sumFlags, bool exitMode=false) {
  if (exitMode) {
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    return;
  }
  // sumFlags policy: 0 -> 00, 1 -> 01, >=2 -> 10
  if (sumFlags == 0) {
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
  } else if (sumFlags == 1) {
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, LOW);
  } else {
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, HIGH);
  }
}

// ---- Summarize current node flags for LED policy
uint8_t currentNodeFlagSum() {
  uint8_t key = makeLocalKey(heading, (uint16_t)(path_len_ticks & 0x3F));
  int slot = getOrCreate(key);
  if (slot < 0) return 0;
  uint8_t f = marks[slot].flags;
  // clamp flags to {0,1,2}, sum across L,F,R
  uint8_t l = getFlag(f,0); if (l>2) l=2;
  uint8_t m = getFlag(f,1); if (m>2) m=2;
  uint8_t r = getFlag(f,2); if (r>2) r=2;
  uint8_t s = (l>0?1:0) + (m>0?1:0) + (r>0?1:0);
  // Interpretation choice: sum as count of explored directions (0/1/2/3)
  // For requested mapping we only discriminate 0,1,>=2
  if (s==0) return 0;
  if (s==1) return 1;
  return 2;
}

// ---- Tremaux marking
void markPassage(uint8_t dir, bool returning) {
  uint8_t key = makeLocalKey(heading, (uint16_t)(path_len_ticks & 0x3F));
  int slot = getOrCreate(key);
  if (slot < 0) return;
  uint8_t f = marks[slot].flags;
  uint8_t cur = getFlag(f, dir);
  if (!returning) {
    if (cur == 0) f = setFlag(f, dir, 1);
  } else {
    if (cur == 1) f = setFlag(f, dir, 2);
    else if (cur == 0) f = setFlag(f, dir, 2);
  }
  marks[slot].flags = f;
}

// ---- Choose direction
int chooseDirection(bool hasL, bool hasF, bool hasR) {
  uint8_t key = makeLocalKey(heading, (uint16_t)(path_len_ticks & 0x3F));
  int slot = getOrCreate(key);
  if (slot < 0) {
    if (hasL) return 0;
    if (hasF) return 1;
    if (hasR) return 2;
    return -1;
  }
  uint8_t f = marks[slot].flags;
  if (hasL && getFlag(f,0)==0) return 0;
  if (hasF && getFlag(f,1)==0) return 1;
  if (hasR && getFlag(f,2)==0) return 2;
  if (hasL && getFlag(f,0)==1) return 0;
  if (hasF && getFlag(f,1)==1) return 1;
  if (hasR && getFlag(f,2)==1) return 2;
  return -1;
}

void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  pinMode(LED1, OUTPUT); pinMode(LED2, OUTPUT);

  motorsStop();
  digitalWrite(LED1, LOW); digitalWrite(LED2, LOW);

  for (int i=0;i<TABLE_SIZE;i++){ marks[i].flags = 0xFF; marks[i].key=0; }
  delay(500);
}

void loop() {
  // Sense
  unsigned long dl = pingCM(TRIG_L, ECHO_L);
  unsigned long df = pingCM(TRIG_F, ECHO_F);
  unsigned long dr = pingCM(TRIG_R, ECHO_R);

  bool openL = (dl > side_th);
  bool openF = (df > front_th);
  bool openR = (dr > side_th);

  // Update LEDs for current node state
  setLEDs(currentNodeFlagSum(), false);

  // Dead-end -> backtrack
  if (!openL && !openF && !openR) {
    markPassage(1, true);
    uTurn();
    return;
  }

  // Choose per Tremaux
  int dir = chooseDirection(openL, openF, openR);
  if (dir < 0) {
    markPassage(1, true);
    uTurn();
    return;
  }

  // Execute turn and mark forward edge
  if (dir == 0) {
    markPassage(0, false);
    turnLeft90();
  } else if (dir == 2) {
    markPassage(2, false);
    turnRight90();
  } else {
    markPassage(1, false);
  }

  // Advance segment
  unsigned long seg_ms = 0;
  while (true) {
    stepForward();
    seg_ms += STEP_FWD_MS + COAST_MS;

    unsigned long dfl = pingCM(TRIG_F, ECHO_F);
    unsigned long dll = pingCM(TRIG_L, ECHO_L);
    unsigned long drr = pingCM(TRIG_R, ECHO_R);

    bool nearWall = (dfl < deadend_clear);
    bool junction = (dll > side_th+3) || (drr > side_th+3);
    if (nearWall || junction) break;
    if (seg_ms > 3000) break;
  }

  // Exit condition: large open space (example heuristic)
  if (df > 200 && dl > 200 && dr > 200) {
    motorsStop();
    setLEDs(0, true); // both ON for exit
    while(1);
  }
}
