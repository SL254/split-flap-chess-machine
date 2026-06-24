// ESP32 chess engine 1.0
// Sergey Urusov, ususovsv@gmail.com

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <LittleFS.h>
#include <TM1637Display.h>
#include <Adafruit_NeoPixel.h>

/// =============================================================
// [설정] 슬레이브 관리 (일반모드 + 테스트모드 통합)
// =============================================================
const int TOTAL_SLAVES = 8;

// [일반모드용] 부팅 시 AWAKE 체크
bool slaveAwake[TOTAL_SLAVES + 1] = { false };

// [테스트 제어 변수] — r_test는 TOKHOME 폴링 방식으로 동작
bool isLoopTestMode = false;               // r_test 활성화 여부
const unsigned long REBOOT_INTERVAL = 50;  // executeInitialSequence()가 사용
unsigned long g_homingTimeoutCount = 0;    // 호밍 타임아웃 누적 횟수

// =============================================================
// [1] 사용자 설정 및 프로토콜 정의
// =============================================================
const char *WIFI_FILE = "/wifi_config.txt";

const char *BOOK_FILE = "/opening_book.txt";

const char *LICHESS_TOKEN = "";

// 프로토콜 ID (1~27)
const int ID_EMPTY = 1;   // 빈칸
const int ID_UNUSED = 2;  // 안 쓰는 칸
const int ID_W_PAWN = 3;
const int ID_B_PAWN = 4;
const int ID_W_KNIGHT = 5;
const int ID_B_KNIGHT = 6;
const int ID_W_BISHOP = 7;
const int ID_B_BISHOP = 8;
const int ID_W_ROOK = 9;
const int ID_B_ROOK = 10;
const int ID_W_QUEEN = 11;
const int ID_B_QUEEN = 12;
const int ID_W_KING = 13;
const int ID_B_KING = 14;
// 상태 ID
const int ID_W_CHECK = 15;      // 백킹 체크
const int ID_B_CHECK = 16;      // 흑킹 체크
const int ID_W_DRAW = 17;       // 백킹 무승부
const int ID_B_DRAW = 18;       // 흑킹 무승부
const int ID_W_TIME_OVER = 19;  // 백킹 시간초과
const int ID_B_TIME_OVER = 20;  // 흑킹 시간초과
const int ID_W_MATE = 21;       // 백킹 메이트
const int ID_B_MATE = 22;       // 흑킹 메이트
const int ID_W_WIN = 23;        // 백 승리
const int ID_B_WIN = 24;        // 흑 승리
const int ID_W_RESIGN = 25;     // 백 기권
const int ID_B_RESIGN = 26;     // 흑 기권
const int ID_PADDING = 27;      // 안 쓰는 칸

// [추가] 흑백 시점 전환을 위한 전역 변수
bool g_playerIsWhite = true;

boolean halt = 0;
volatile bool g_abortThink = false;  // AI 생각 중 중단 요청 플래그

// =============================================================
// [NeoPixel] 체스판 LED
// =============================================================
#define PIN_NEOPIXEL 13
#define NUMPIXELS    80
Adafruit_NeoPixel strip(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// =============================================================
// [매트릭스 스캔] 8x10 키보드
// =============================================================
#define MATRIX_ROWS 8
#define MATRIX_COLS 10

const int rowPins[MATRIX_ROWS] = {35, 36, 37, 38, 39, 40, 41, 42};
const int colPins[MATRIX_COLS] = {4, 5, 6, 7, 15, 16, 17, 18, 8, 3};

bool rawState[MATRIX_ROWS][MATRIX_COLS]      = {false};
bool lastRawState[MATRIX_ROWS][MATRIX_COLS]  = {false};
bool debouncedState[MATRIX_ROWS][MATRIX_COLS]= {false};

unsigned long lastDebounceTime[MATRIX_ROWS][MATRIX_COLS] = {0};
const unsigned long debounceDelay = 5;

// i4=백선택, j4=흑선택, J8=게임시작 (keyboard row/col 인덱스)
#define BTN_WHITE_R 3   // rank4 → sw row = rank-1 = 3, 180° 보정
#define BTN_WHITE_C 1   // col i(8) → sw col = 9-8 = 1
#define BTN_BLACK_R 3
#define BTN_BLACK_C 0   // col j(9) → sw col = 9-9 = 0
#define BTN_START_R 7   // rank8 → sw row = 8-1 = 7
#define BTN_START_C 0

#define LED_BTN_WHITE 48  // (7-3)*10+(9-1) = 48
#define LED_BTN_BLACK 49  // (7-3)*10+(9-0) = 49
#define LED_BTN_START  9  // (7-7)*10+(9-0) = 9

#define BTN_RESIGN_R 6   // i7: rank7 → sw row = 6
#define BTN_RESIGN_C 1   // col I → sw col = 1
#define LED_RESIGN   18  // (7-6)*10+(9-1) = 18

// 타임컨트롤 버튼: I3 J3 I2 J2 I1 J1
#define BTN_TC5_R      2
#define BTN_TC5_C      1  // I3: rank3,colI → sw(2,1)
#define BTN_TC10_R     2
#define BTN_TC10_C     0  // J3: sw(2,0)
#define BTN_TC10_5_R   1
#define BTN_TC10_5_C   1  // I2: sw(1,1)
#define BTN_TC15_10_R  1
#define BTN_TC15_10_C  0  // J2: sw(1,0)
#define BTN_TC30_R     0
#define BTN_TC30_C     1  // I1: sw(0,1)
#define BTN_TCINF_R    0
#define BTN_TCINF_C    0  // J1: sw(0,0)

#define LED_TC_5       58  // (7-2)*10+(9-1)
#define LED_TC_10      59  // (7-2)*10+(9-0)
#define LED_TC_10_5    68  // (7-1)*10+(9-1)
#define LED_TC_15_10   69  // (7-1)*10+(9-0)
#define LED_TC_30      78  // (7-0)*10+(9-1)
#define LED_TC_INF     79  // (7-0)*10+(9-0)

// 프로모션 버튼: I6=퀸 J6=룩 I5=비숍 J5=나이트
#define BTN_PROMO_Q_R  5
#define BTN_PROMO_Q_C  1  // I6: sw(5,1)
#define BTN_PROMO_R_R  5
#define BTN_PROMO_R_C  0  // J6: sw(5,0)
#define BTN_PROMO_B_R  4
#define BTN_PROMO_B_C  1  // I5: sw(4,1)
#define BTN_PROMO_N_R  4
#define BTN_PROMO_N_C  0  // J5: sw(4,0)

#define LED_PROMO_Q   28  // (7-5)*10+(9-1)
#define LED_PROMO_R   29  // (7-5)*10+(9-0)
#define LED_PROMO_B   38  // (7-4)*10+(9-1)
#define LED_PROMO_N   39  // (7-4)*10+(9-0)

int  g_colorSelect = 0;   // 0=백, 1=흑, 2=랜덤
bool g_selectMode  = false;
int  g_selSq = -1;        // 선택된 기물 칸 (-1=없음)
bool g_legalTo[64]   = {false};
bool g_captureTo[64] = {false};

String g_abortCmd = "";              // 어떤 명령이었는지 (resign/exit/quit/stop
bool g_resignPending = false;        // 기권 버튼 1회 누름 → 확인 대기 상태
unsigned long g_resignBlinkLast = 0;
bool g_resignBlinkOn = false;

// =============================================================
// [2] 링 통신 설정
// =============================================================
#define RING_RX 48
#define RING_TX 47
#define RING_SERIAL Serial2

const int TOTAL_NODES = 8;
const unsigned long HB_INTERVAL = 5000;
const int HB_TIMEOUT = 500;
const int HB_MAX_RETRIES = 3;

unsigned long lastHbTime = 0;
unsigned long hbSendTime = 0;
int hbRetryCount = 0;
bool isHbWaiting = false;
bool isHbError = false;
String ringBuffer = "";

bool waitingForMoveComplete = false;
int pendingCompleteCount = 0;

bool g_bootInitDone = false;  // 부팅 초기화 1회 완료 플래그

// === 토큰 프로토콜 ===
const unsigned long TOKEN_REISSUE_INTERVAL = 300;   // ⚠ 1바퀴 RTT보다 충분히 크게(실측 후 조정)
const int TOKEN_MAX_RETRY = 20;                     // 약 6초
unsigned long g_moveGen[TOTAL_SLAVES + 1] = { 0 };  // 노드별 누적 이동 세대(마스터 기준)
uint8_t g_expectedMask = 0;                         // 직전 dispatch에서 명령 보낸 노드 비트
int g_tokenSeq = 0;                                 // 토큰 발행 일련번호
uint8_t g_gotMask = 0;                              // 되돌아온 mask

// =============================================================
// [3] 체스 엔진 변수 및 상수
// =============================================================
const signed char fp = 1;
const signed char fn = 2;
const signed char fb = 3;
const signed char fr = 4;
const signed char fq = 5;
const signed char fk = 6;
const int fig_weight[] = { 0, 100, 320, 330, 500, 900, 0 };
const char fig_symb[] = "  NBRQK";
const char fig_symb1[] = " pNBRQK";

int countin = 0, countall = 0;
const int MAXSTEPS = 150;
const int MAXDEPTH = 30;

typedef struct {
  signed char f1, f2;
  signed char c1, c2;
  signed char check;
  signed char type;
  short weight;
} step_t;

struct position_t {
  uint8_t w;
  uint8_t wrk, wrq, brk, brq;
  uint8_t pp;
  step_t steps[MAXSTEPS + 1];
  int n_steps;
  int cur_step;
  step_t best;
  int check_on_table;
  short weight_w;
  short weight_b;
  short weight_s;
  uint64_t hash;
  uint8_t halfmove_clock;
};

short pole[64];       // 현재 보드
short prev_pole[64];  // [변경감지용] 이전 보드

position_t pos[MAXDEPTH];
step_t steps2[MAXSTEPS];
int n_steps2;
struct packed_t {
  int q[8];
};
packed_t polep;

String fenstr;
const int MAXEPD = 5;
int bestcount = 0;
step_t bestmove[MAXEPD];
boolean bestsolved = 0;
boolean zero = 0;

uint64_t zobrist_table[13][64];
uint64_t zobrist_side;
uint64_t zobrist_castling[16];
uint64_t zobrist_ep[8];
uint64_t game_history_hash[500];
int game_history_count = 0;

int TRACE = 0;
unsigned long timelimith = 3 * 1000;
unsigned long starttime;
int nullmove = 1;
int multipov = 0;
int futility = 1;
int lazyeval = 1;
int depth = 0;
int nulldepth;
int lazy;
boolean endspiel = 0;
boolean stats = 1;
int lastbestdepth = 0;
step_t lastbeststep;

step_t bufsteps[MAXSTEPS + 1];
step_t game_steps[1000];
position_t game_pos;
int game_ply;
boolean game_w;
short game_pole[64];
int poswk = 0;
int posbk = 0;
unsigned long count = 0;
int task_start = 0;
int task_check = 0;
int task_l;
short task_s;
unsigned long task_time, task_execute;
int level;
int fdepth;

// =============================================================
// [VFD] PT6302 16자리 VFD 표시
// =============================================================
uint8_t vfd_din = 45;   // DA (SDI)
uint8_t vfd_clk = 21;   // CK
uint8_t vfd_cs  = 2;    // CS

#define VFD_DIGITS 16

void write_6302(unsigned char w_data) {
  for (unsigned char i = 0; i < 8; i++) {
    digitalWrite(vfd_clk, LOW);
    digitalWrite(vfd_din, (w_data & 0x01) ? HIGH : LOW);
    w_data >>= 1;
    digitalWrite(vfd_clk, HIGH);
  }
}

void VFD_init() {
  digitalWrite(vfd_cs, LOW);
  write_6302(0xe0);
  write_6302(0x0f);      // 16자리
  digitalWrite(vfd_cs, HIGH);
  delayMicroseconds(5);

  digitalWrite(vfd_cs, LOW);
  write_6302(0xe4);
  write_6302(255);       // 밝기 최대
  digitalWrite(vfd_cs, HIGH);
  delayMicroseconds(5);
}

void S1201_show(void) {
  digitalWrite(vfd_cs, LOW);
  write_6302(0xe8);
  digitalWrite(vfd_cs, HIGH);
}

void VFD_off(void) {
  digitalWrite(vfd_cs, LOW);
  write_6302(0xe4);
  write_6302(0);         // 밝기 0 = 화면 끔
  digitalWrite(vfd_cs, HIGH);
  delayMicroseconds(5);
}

// 16자리 화면 중앙 정렬 표시
void VFD_print(const char *str) {
  // 꺼져 있을 수 있으니 밝기 복구
  digitalWrite(vfd_cs, LOW);
  write_6302(0xe4);
  write_6302(255);
  digitalWrite(vfd_cs, HIGH);
  delayMicroseconds(5);

  uint8_t len = 0;
  while (str[len] && len < VFD_DIGITS) len++;

  uint8_t pad = VFD_DIGITS - len;
  uint8_t left = (len == 15) ? (pad + 1) / 2 : pad / 2;
  uint8_t right = pad - left;

  digitalWrite(vfd_cs, LOW);
  write_6302(0x20 + 0);                                   // 0번 위치부터
  for (uint8_t i = 0; i < left; i++)  write_6302(' ');
  for (uint8_t i = 0; i < len; i++)   write_6302(str[i]);
  for (uint8_t i = 0; i < right; i++) write_6302(' ');
  digitalWrite(vfd_cs, HIGH);
  S1201_show();
}

// =============================================================
// [TIME CONTROL] TM1637 7세그먼트 체스 시계 (AI전 전용)
// =============================================================
#define CLK1 9   // 백(White) 시계 CLK
#define DIO1 10  // 백(White) 시계 DIO
#define CLK2 11  // 흑(Black) 시계 CLK
#define DIO2 12  // 흑(Black) 시계 DIO

TM1637Display whiteClockDisp(CLK1, DIO1);
TM1637Display blackClockDisp(CLK2, DIO2);

// 콜론(:) 표시 비트 — 모듈/라이브러리 버전에 따라 다를 수 있으니 실제 디스플레이 보고 조정
#define CLOCK_COLON_BIT 0b01000000

// 밝기: TM1637 밝기 범위 0~7 중 약 1/4
#define CLOCK_BRIGHTNESS 2

// 시계 디스플레이 전원 on/off (부팅 시 꺼두고, 게임 시작(기물 세팅) 시 켬)
// setBrightness()만으로는 칩에 아무 것도 전송되지 않으므로,
// clocksDisplay()로 즉시 다시 써서 on/off가 그 자리에서 바로 반영되게 한다.
void clocksPower(bool on) {
  whiteClockDisp.setBrightness(CLOCK_BRIGHTNESS, on);
  blackClockDisp.setBrightness(CLOCK_BRIGHTNESS, on);
  clocksDisplay();
}

// ---- 타임컨트롤 종류 (추후 추가 가능한 구조) ----
struct TimeControlSetting {
  unsigned long baseMs;       // 기본 시간
  unsigned long incrementMs;  // 1수당 증가 시간
};

enum TimeControlType {
  TC_10_5 = 0,   // 10분 + 5초 (기본값)
  TC_5_0,        // 5분
  TC_10_0,       // 10분
  TC_15_10,      // 15분 + 10초
  TC_30_0,       // 30분
  TC_UNLIMITED,  // 무제한 (시계 끄기)
  // TODO: 다른 타임컨트롤은 여기와 TIME_CONTROLS[]에 항목 추가
  TC_COUNT
};

const TimeControlSetting TIME_CONTROLS[TC_COUNT] = {
  { 10UL * 60 * 1000UL, 5UL * 1000UL },   // TC_10_5: 10분 + 5초
  { 5UL * 60 * 1000UL, 0 },               // TC_5_0: 5분
  { 10UL * 60 * 1000UL, 0 },              // TC_10_0: 10분
  { 15UL * 60 * 1000UL, 10UL * 1000UL },  // TC_15_10: 15분 + 10초
  { 30UL * 60 * 1000UL, 0 },              // TC_30_0: 30분
  { 0, 0 },                                // TC_UNLIMITED: 사용 안 함
};

TimeControlType g_timeControl = TC_10_5;  // 현재 적용 중인 타임컨트롤

unsigned long g_whiteTimeMs = 0;
unsigned long g_blackTimeMs = 0;
unsigned long g_clockLastTick = 0;
bool g_clockRunning = false;   // false면 양쪽 시계 모두 정지(예: MOVING... 대기 중)
bool g_clockWhiteTurn = true;  // 현재 시간이 줄어드는 쪽
bool g_timeOver = false;
bool g_timeOverIsWhite = false;

void clockShow(TM1637Display &disp, unsigned long ms) {
  unsigned long totalSec = ms / 1000;
  int mm = totalSec / 60;
  int ss = totalSec % 60;
  if (mm > 99) mm = 99;
  disp.showNumberDecEx(mm * 100 + ss, CLOCK_COLON_BIT, true);
}

void clocksDisplay() {
  clockShow(whiteClockDisp, g_whiteTimeMs);
  clockShow(blackClockDisp, g_blackTimeMs);
}

// 게임 시작 시 시간 초기화 (현재 g_timeControl 기준)
void clocksInit() {
  const TimeControlSetting &tc = TIME_CONTROLS[g_timeControl];
  g_whiteTimeMs = tc.baseMs;
  g_blackTimeMs = tc.baseMs;
  g_clockRunning = false;
  g_timeOver = false;
  g_timeOverIsWhite = false;
  clocksDisplay();
}

// 시간 소모 + 타임아웃 감지. g_clockRunning일 때만 동작하며 자주 호출되어야 함.
void clockUpdate() {
  if (g_timeControl == TC_UNLIMITED) return;  // 무제한: 시계 동작 없음
  if (!g_clockRunning) return;
  unsigned long now = millis();
  unsigned long elapsed = now - g_clockLastTick;
  if (elapsed == 0) return;
  g_clockLastTick = now;

  unsigned long &t = g_clockWhiteTurn ? g_whiteTimeMs : g_blackTimeMs;
  if (t > elapsed) {
    t -= elapsed;
  } else {
    t = 0;
    if (!g_timeOver) {
      g_timeOver = true;
      g_timeOverIsWhite = g_clockWhiteTurn;
      halt = 1;  // AI 탐색 중이면 즉시 중단
    }
  }

  static unsigned long lastDisp = 0;
  if (now - lastDisp >= 200) {
    lastDisp = now;
    clocksDisplay();
  }
}

// 턴 시작: isWhiteTurn 쪽 시계 재개 (생각 시간 포함)
void clockResume(bool isWhiteTurn) {
  if (g_timeControl == TC_UNLIMITED) return;  // 무제한: 시계 동작 없음
  g_clockWhiteTurn = isWhiteTurn;
  g_clockLastTick = millis();
  g_clockRunning = true;
}

// 모터 이동(MOVING...) 대기 등: 양쪽 시계 일시정지
void clockPause() {
  clockUpdate();
  g_clockRunning = false;
  clocksDisplay();
}

// 수를 둔 쪽에 증가시간(increment) 적용
void clockAddIncrement(bool wasWhite) {
  unsigned long inc = TIME_CONTROLS[g_timeControl].incrementMs;
  if (wasWhite) g_whiteTimeMs += inc;
  else g_blackTimeMs += inc;
  clocksDisplay();
}

// =============================================================
// [4] 데이터 테이블 (가중치 및 이동 규칙)
// =============================================================
const short column[64] = {
  1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8,
  1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8,
  1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8,
  1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8
};
const short row[64] = {
  8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 7, 7,
  6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5,
  4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3,
  2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1
};
const short diag1[64] = {
  1, 2, 3, 4, 5, 6, 7, 8, 2, 3, 4, 5, 6, 7, 8, 9,
  3, 4, 5, 6, 7, 8, 9, 10, 4, 5, 6, 7, 8, 9, 10, 11,
  5, 6, 7, 8, 9, 10, 11, 12, 6, 7, 8, 9, 10, 11, 12, 13,
  7, 8, 9, 10, 11, 12, 13, 14, 8, 9, 10, 11, 12, 13, 14, 15
};
const short diag2[64] = {
  8, 7, 6, 5, 4, 3, 2, 1, 9, 8, 7, 6, 5, 4, 3, 2,
  10, 9, 8, 7, 6, 5, 4, 3, 11, 10, 9, 8, 7, 6, 5, 4,
  12, 11, 10, 9, 8, 7, 6, 5, 13, 12, 11, 10, 9, 8, 7, 6,
  14, 13, 12, 11, 10, 9, 8, 7, 15, 14, 13, 12, 11, 10, 9, 8
};

const int stat_weightw[7][64] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 100, 100, 100, 100, 100, 100, 100, 100,
    20, 30, 40, 50, 50, 40, 30, 20, 5, 5, 10, 25, 25, 10, 5, 5,
    0, 0, 0, 20, 20, 0, 0, 0, 5, -5, -10, 0, 0, -10, -5, 5,
    5, 10, 10, -20, -20, 10, 10, 5, 0, 0, 0, 0, 0, 0, 0, 0 },
  { -50, -40, -30, -30, -30, -30, -40, -50, -40, -20, 0, 0, 0, 0, -20, -40,
    -30, 0, 10, 15, 15, 10, 0, -30, -30, 5, 15, 20, 20, 15, 5, -30,
    -30, 0, 15, 20, 20, 15, 0, -30, -30, 5, 10, 15, 15, 10, 5, -30,
    -40, -20, 0, 5, 5, 0, -20, -40, -50, -40, -30, -30, -30, -30, -40, -50 },
  { -20, -10, -10, -10, -10, -10, -10, -20, -10, 0, 0, 0, 0, 0, 0, -10,
    -10, 0, 5, 10, 10, 5, 0, -10, -10, 5, 5, 10, 10, 5, 5, -10,
    -10, 0, 10, 10, 10, 10, 0, -10, -10, 10, 10, 10, 10, 10, 10, -10,
    -10, 5, 0, 0, 0, 0, 5, -10, -20, -10, -10, -10, -10, -10, -10, -20 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 5, 10, 10, 10, 10, 10, 10, 5,
    -5, 0, 0, 0, 0, 0, 0, -5, -5, 0, 0, 0, 0, 0, 0, -5,
    -5, 0, 0, 0, 0, 0, 0, -5, -5, 0, 0, 0, 0, 0, 0, -5,
    -5, 0, 0, 0, 0, 0, 0, -5, 0, 0, 0, 5, 5, 0, 0, 0 },
  { -20, -10, -10, -5, -5, -10, -10, -20, -10, 0, 0, 0, 0, 0, 0, -10,
    -10, 0, 5, 5, 5, 5, 0, -10, -5, 0, 5, 5, 5, 5, 0, -5,
    0, 0, 5, 5, 5, 5, 0, -5, -10, 5, 5, 5, 5, 5, 0, -10,
    -10, 0, 5, 0, 0, 0, 0, -10, -20, -10, -10, -5, -5, -10, -10, -20 },
  { -30, -40, -40, -50, -50, -40, -40, -30, -30, -40, -40, -50, -50, -40, -40, -30,
    -30, -40, -40, -50, -50, -40, -40, -30, -30, -40, -40, -50, -50, -40, -40, -30,
    -20, -30, -30, -40, -40, -30, -30, -20, -10, -20, -20, -20, -20, -20, -20, -10,
    10, 10, -10, -10, -10, -10, 10, 10, 10, 40, 30, 0, 0, 0, 50, 10 },
  { -50, -40, -30, -20, -20, -30, -40, -50, -30, -20, -10, 0, 0, -10, -20, -30,
    -30, -10, 20, 30, 30, 20, -10, -30, -30, -10, 30, 40, 40, 30, -10, -30,
    -30, -10, 30, 40, 40, 30, -10, -30, -30, -10, 20, 30, 30, 20, -10, -30,
    -30, -30, 0, 0, 0, 0, -30, -30, -50, -30, -30, -30, -30, -30, -30, -50 }
};
const int stat_weightb[7][64] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 5, 10, 10, -20, -20, 10, 10, 5,
    5, -5, -10, 0, 0, -10, -5, 5, 0, 0, 0, 20, 20, 0, 0, 0,
    5, 5, 10, 25, 25, 10, 5, 5, 20, 30, 40, 50, 50, 40, 30, 20,
    100, 100, 100, 100, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0 },
  { -50, -40, -30, -30, -30, -30, -40, -50, -40, -20, 0, 0, 0, 0, -20, -40,
    -30, 5, 10, 15, 15, 10, 5, -30, -30, 0, 15, 20, 20, 15, 0, -30,
    -30, 5, 15, 20, 20, 15, 5, -30, -30, 0, 10, 15, 15, 10, 0, -30,
    -40, -20, 0, 5, 5, 0, -20, -40, -50, -40, -30, -30, -30, -30, -40, -50 },
  { -20, -10, -10, -10, -10, -10, -10, -20, -10, 5, 0, 0, 0, 0, 5, -10,
    -10, 10, 10, 10, 10, 10, 10, -10, -10, 0, 10, 10, 10, 10, 0, -10,
    -10, 5, 5, 10, 10, 5, 5, -10, -10, 0, 5, 10, 10, 5, 0, -10,
    -10, 0, 0, 0, 0, 0, 0, -10, -20, -10, -10, -10, -10, -10, -10, -20 },
  { 0, 0, 0, 5, 5, 0, 0, 0, -5, 0, 0, 0, 0, 0, 0, -5,
    -5, 0, 0, 0, 0, 0, 0, -5, -5, 0, 0, 0, 0, 0, 0, -5,
    -5, 0, 0, 0, 0, 0, 0, -5, -5, 0, 0, 0, 0, 0, 0, -5,
    5, 10, 10, 10, 10, 10, 10, 5, 0, 0, 0, 0, 0, 0, 0, 0 },
  { -20, -10, -10, -5, -5, -10, -10, -20, -10, 0, 5, 0, 0, 0, 0, -10,
    -10, 5, 5, 5, 5, 5, 0, -10, 0, 0, 5, 5, 5, 5, 0, -5,
    -5, 0, 5, 5, 5, 5, 0, -5, -10, 0, 5, 5, 5, 5, 0, -10,
    -10, 0, 0, 0, 0, 0, 0, -10, -20, -10, -10, -5, -5, -10, -10, -20 },
  { 10, 40, 30, 0, 0, 0, 50, 10, 10, 10, -10, -10, -10, -10, 10, 10,
    -10, -20, -20, -20, -20, -20, -20, -10, -20, -30, -30, -40, -40, -30, -30, -20,
    -30, -40, -40, -50, -50, -40, -40, -30, -30, -40, -40, -50, -50, -40, -40, -30,
    -30, -40, -40, -50, -50, -40, -40, -30, -30, -40, -40, -50, -50, -40, -40, -30 },
  { -50, -30, -30, -30, -30, -30, -30, -50, -30, -30, 0, 0, 0, 0, -30, -30,
    -30, -10, 20, 30, 30, 20, -10, -30, -30, -10, 30, 40, 40, 30, -10, -30,
    -30, -10, 30, 40, 40, 30, -10, -30, -30, -10, 20, 30, 30, 20, -10, -30,
    -30, -20, -10, 0, 0, -10, -20, -30, -50, -40, -30, -20, -20, -30, -40, -50 }
};

const uint8_t diag_step[64][17] = {
  { 9, 18, 27, 36, 45, 54, 63, 99 }, { 10, 19, 28, 37, 46, 55, 88, 8, 99 }, { 11, 20, 29, 38, 47, 88, 9, 16, 99 }, { 12, 21, 30, 39, 88, 10, 17, 24, 99 }, { 13, 22, 31, 88, 11, 18, 25, 32, 99 }, { 14, 23, 88, 12, 19, 26, 33, 40, 99 }, { 15, 88, 13, 20, 27, 34, 41, 48, 99 }, { 14, 21, 28, 35, 42, 49, 56, 99 }, { 17, 26, 35, 44, 53, 62, 88, 1, 99 }, { 18, 27, 36, 45, 54, 63, 88, 16, 88, 0, 88, 2, 99 }, { 19, 28, 37, 46, 55, 88, 17, 24, 88, 1, 88, 3, 99 }, { 20, 29, 38, 47, 88, 18, 25, 32, 88, 2, 88, 4, 99 }, { 21, 30, 39, 88, 19, 26, 33, 40, 88, 3, 88, 5, 99 }, { 22, 31, 88, 20, 27, 34, 41, 48, 88, 4, 88, 6, 99 }, { 23, 88, 21, 28, 35, 42, 49, 56, 88, 5, 88, 7, 99 }, { 22, 29, 36, 43, 50, 57, 88, 6, 99 }, { 25, 34, 43, 52, 61, 88, 9, 2, 99 }, { 26, 35, 44, 53, 62, 88, 24, 88, 8, 88, 10, 3, 99 }, { 27, 36, 45, 54, 63, 88, 25, 32, 88, 9, 0, 88, 11, 4, 99 }, { 28, 37, 46, 55, 88, 26, 33, 40, 88, 10, 1, 88, 12, 5, 99 }, { 29, 38, 47, 88, 27, 34, 41, 48, 88, 11, 2, 88, 13, 6, 99 }, { 30, 39, 88, 28, 35, 42, 49, 56, 88, 12, 3, 88, 14, 7, 99 }, { 31, 88, 29, 36, 43, 50, 57, 88, 13, 4, 88, 15, 99 }, { 30, 37, 44, 51, 58, 88, 14, 5, 99 }, { 33, 42, 51, 60, 88, 17, 10, 3, 99 }, { 34, 43, 52, 61, 88, 32, 88, 16, 88, 18, 11, 4, 99 }, { 35, 44, 53, 62, 88, 33, 40, 88, 17, 8, 88, 19, 12, 5, 99 }, { 36, 45, 54, 63, 88, 34, 41, 48, 88, 18, 9, 0, 88, 20, 13, 6, 99 }, { 37, 46, 55, 88, 35, 42, 49, 56, 88, 19, 10, 1, 88, 21, 14, 7, 99 }, { 38, 47, 88, 36, 43, 50, 57, 88, 20, 11, 2, 88, 22, 15, 99 }, { 39, 88, 37, 44, 51, 58, 88, 21, 12, 3, 88, 23, 99 }, { 38, 45, 52, 59, 88, 22, 13, 4, 99 }, { 41, 50, 59, 88, 25, 18, 11, 4, 99 }, { 42, 51, 60, 88, 40, 88, 24, 88, 26, 19, 12, 5, 99 }, { 43, 52, 61, 88, 41, 48, 88, 25, 16, 88, 27, 20, 13, 6, 99 }, { 44, 53, 62, 88, 42, 49, 56, 88, 26, 17, 8, 88, 28, 21, 14, 7, 99 }, { 45, 54, 63, 88, 43, 50, 57, 88, 27, 18, 9, 0, 88, 29, 22, 15, 99 }, { 46, 55, 88, 44, 51, 58, 88, 28, 19, 10, 1, 88, 30, 23, 99 }, { 47, 88, 45, 52, 59, 88, 29, 20, 11, 2, 88, 31, 99 }, { 46, 53, 60, 88, 30, 21, 12, 3, 99 }, { 49, 58, 88, 33, 26, 19, 12, 5, 99 }, { 50, 59, 88, 48, 88, 32, 88, 34, 27, 20, 13, 6, 99 }, { 51, 60, 88, 49, 56, 88, 33, 24, 88, 35, 28, 21, 14, 7, 99 }, { 52, 61, 88, 50, 57, 88, 34, 25, 16, 88, 36, 29, 22, 15, 99 }, { 53, 62, 88, 51, 58, 88, 35, 26, 17, 8, 88, 37, 30, 23, 99 }, { 54, 63, 88, 52, 59, 88, 36, 27, 18, 9, 0, 88, 38, 31, 99 }, { 55, 88, 53, 60, 88, 37, 28, 19, 10, 1, 88, 39, 99 }, { 54, 61, 88, 38, 29, 20, 11, 2, 99 }, { 57, 88, 41, 34, 27, 20, 13, 6, 99 }, { 58, 88, 56, 88, 40, 88, 42, 35, 28, 21, 14, 7, 99 }, { 59, 88, 57, 88, 41, 32, 88, 43, 36, 29, 22, 15, 99 }, { 60, 88, 58, 88, 42, 33, 24, 88, 44, 37, 30, 23, 99 }, { 61, 88, 59, 88, 43, 34, 25, 16, 88, 45, 38, 31, 99 }, { 62, 88, 60, 88, 44, 35, 26, 17, 8, 88, 46, 39, 99 }, { 63, 88, 61, 88, 45, 36, 27, 18, 9, 0, 88, 47, 99 }, { 62, 88, 46, 37, 28, 19, 10, 1, 99 }, { 49, 42, 35, 28, 21, 14, 7, 99 }, { 48, 88, 50, 43, 36, 29, 22, 15, 99 }, { 49, 40, 88, 51, 44, 37, 30, 23, 99 }, { 50, 41, 32, 88, 52, 45, 38, 31, 99 }, { 51, 42, 33, 24, 88, 53, 46, 39, 99 }, { 52, 43, 34, 25, 16, 88, 54, 47, 99 }, { 53, 44, 35, 26, 17, 8, 88, 55, 99 }, { 54, 45, 36, 27, 18, 9, 0, 99 }
};

const uint8_t stra_step[64][18] = {
  { 1, 2, 3, 4, 5, 6, 7, 88, 8, 16, 24, 32, 40, 48, 56, 99 }, { 2, 3, 4, 5, 6, 7, 88, 9, 17, 25, 33, 41, 49, 57, 88, 0, 99 }, { 3, 4, 5, 6, 7, 88, 10, 18, 26, 34, 42, 50, 58, 88, 1, 0, 99 }, { 4, 5, 6, 7, 88, 11, 19, 27, 35, 43, 51, 59, 88, 2, 1, 0, 99 }, { 5, 6, 7, 88, 12, 20, 28, 36, 44, 52, 60, 88, 3, 2, 1, 0, 99 }, { 6, 7, 88, 13, 21, 29, 37, 45, 53, 61, 88, 4, 3, 2, 1, 0, 99 }, { 7, 88, 14, 22, 30, 38, 46, 54, 62, 88, 5, 4, 3, 2, 1, 0, 99 }, { 15, 23, 31, 39, 47, 55, 63, 88, 6, 5, 4, 3, 2, 1, 0, 99 }, { 9, 10, 11, 12, 13, 14, 15, 88, 16, 24, 32, 40, 48, 56, 88, 0, 99 }, { 10, 11, 12, 13, 14, 15, 88, 17, 25, 33, 41, 49, 57, 88, 8, 88, 1, 99 }, { 11, 12, 13, 14, 15, 88, 18, 26, 34, 42, 50, 58, 88, 9, 8, 88, 2, 99 }, { 12, 13, 14, 15, 88, 19, 27, 35, 43, 51, 59, 88, 10, 9, 8, 88, 3, 99 }, { 13, 14, 15, 88, 20, 28, 36, 44, 52, 60, 88, 11, 10, 9, 8, 88, 4, 99 }, { 14, 15, 88, 21, 29, 37, 45, 53, 61, 88, 12, 11, 10, 9, 8, 88, 5, 99 }, { 15, 88, 22, 30, 38, 46, 54, 62, 88, 13, 12, 11, 10, 9, 8, 88, 6, 99 }, { 23, 31, 39, 47, 55, 63, 88, 14, 13, 12, 11, 10, 9, 8, 88, 7, 99 }, { 17, 18, 19, 20, 21, 22, 23, 88, 24, 32, 40, 48, 56, 88, 8, 0, 99 }, { 18, 19, 20, 21, 22, 23, 88, 25, 33, 41, 49, 57, 88, 16, 88, 9, 1, 99 }, { 19, 20, 21, 22, 23, 88, 26, 34, 42, 50, 58, 88, 17, 16, 88, 10, 2, 99 }, { 20, 21, 22, 23, 88, 27, 35, 43, 51, 59, 88, 18, 17, 16, 88, 11, 3, 99 }, { 21, 22, 23, 88, 28, 36, 44, 52, 60, 88, 19, 18, 17, 16, 88, 12, 4, 99 }, { 22, 23, 88, 29, 37, 45, 53, 61, 88, 20, 19, 18, 17, 16, 88, 13, 5, 99 }, { 23, 88, 30, 38, 46, 54, 62, 88, 21, 20, 19, 18, 17, 16, 88, 14, 6, 99 }, { 31, 39, 47, 55, 63, 88, 22, 21, 20, 19, 18, 17, 16, 88, 15, 7, 99 }, { 25, 26, 27, 28, 29, 30, 31, 88, 32, 40, 48, 56, 88, 16, 8, 0, 99 }, { 26, 27, 28, 29, 30, 31, 88, 33, 41, 49, 57, 88, 24, 88, 17, 9, 1, 99 }, { 27, 28, 29, 30, 31, 88, 34, 42, 50, 58, 88, 25, 24, 88, 18, 10, 2, 99 }, { 28, 29, 30, 31, 88, 35, 43, 51, 59, 88, 26, 25, 24, 88, 19, 11, 3, 99 }, { 29, 30, 31, 88, 36, 44, 52, 60, 88, 27, 26, 25, 24, 88, 20, 12, 4, 99 }, { 30, 31, 88, 37, 45, 53, 61, 88, 28, 27, 26, 25, 24, 88, 21, 13, 5, 99 }, { 31, 88, 38, 46, 54, 62, 88, 29, 28, 27, 26, 25, 24, 88, 22, 14, 6, 99 }, { 39, 47, 55, 63, 88, 30, 29, 28, 27, 26, 25, 24, 88, 23, 15, 7, 99 }, { 33, 34, 35, 36, 37, 38, 39, 88, 40, 48, 56, 88, 24, 16, 8, 0, 99 }, { 34, 35, 36, 37, 38, 39, 88, 41, 49, 57, 88, 32, 88, 25, 17, 9, 1, 99 }, { 35, 36, 37, 38, 39, 88, 42, 50, 58, 88, 33, 32, 88, 26, 18, 10, 2, 99 }, { 36, 37, 38, 39, 88, 43, 51, 59, 88, 34, 33, 32, 88, 27, 19, 11, 3, 99 }, { 37, 38, 39, 88, 44, 52, 60, 88, 35, 34, 33, 32, 88, 28, 20, 12, 4, 99 }, { 38, 39, 88, 45, 53, 61, 88, 36, 35, 34, 33, 32, 88, 29, 21, 13, 5, 99 }, { 39, 88, 46, 54, 62, 88, 37, 36, 35, 34, 33, 32, 88, 30, 22, 14, 6, 99 }, { 47, 55, 63, 88, 38, 37, 36, 35, 34, 33, 32, 88, 31, 23, 15, 7, 99 }, { 41, 42, 43, 44, 45, 46, 47, 88, 48, 56, 88, 32, 24, 16, 8, 0, 99 }, { 42, 43, 44, 45, 46, 47, 88, 49, 57, 88, 40, 88, 33, 25, 17, 9, 1, 99 }, { 43, 44, 45, 46, 47, 88, 50, 58, 88, 41, 40, 88, 34, 26, 18, 10, 2, 99 }, { 44, 45, 46, 47, 88, 51, 59, 88, 42, 41, 40, 88, 35, 27, 19, 11, 3, 99 }, { 45, 46, 47, 88, 52, 60, 88, 43, 42, 41, 40, 88, 36, 28, 20, 12, 4, 99 }, { 46, 47, 88, 53, 61, 88, 44, 43, 42, 41, 40, 88, 37, 29, 21, 13, 5, 99 }, { 47, 88, 54, 62, 88, 45, 44, 43, 42, 41, 40, 88, 38, 30, 22, 14, 6, 99 }, { 55, 63, 88, 46, 45, 44, 43, 42, 41, 40, 88, 39, 31, 23, 15, 7, 99 }, { 49, 50, 51, 52, 53, 54, 55, 88, 56, 88, 40, 32, 24, 16, 8, 0, 99 }, { 50, 51, 52, 53, 54, 55, 88, 57, 88, 48, 88, 41, 33, 25, 17, 9, 1, 99 }, { 51, 52, 53, 54, 55, 88, 58, 88, 49, 48, 88, 42, 34, 26, 18, 10, 2, 99 }, { 52, 53, 54, 55, 88, 59, 88, 50, 49, 48, 88, 43, 35, 27, 19, 11, 3, 99 }, { 53, 54, 55, 88, 60, 88, 51, 50, 49, 48, 88, 44, 36, 28, 20, 12, 4, 99 }, { 54, 55, 88, 61, 88, 52, 51, 50, 49, 48, 88, 45, 37, 29, 21, 13, 5, 99 }, { 55, 88, 62, 88, 53, 52, 51, 50, 49, 48, 88, 46, 38, 30, 22, 14, 6, 99 }, { 63, 88, 54, 53, 52, 51, 50, 49, 48, 88, 47, 39, 31, 23, 15, 7, 99 }, { 57, 58, 59, 60, 61, 62, 63, 88, 48, 40, 32, 24, 16, 8, 0, 99 }, { 58, 59, 60, 61, 62, 63, 88, 56, 88, 49, 41, 33, 25, 17, 9, 1, 99 }, { 59, 60, 61, 62, 63, 88, 57, 56, 88, 50, 42, 34, 26, 18, 10, 2, 99 }, { 60, 61, 62, 63, 88, 58, 57, 56, 88, 51, 43, 35, 27, 19, 11, 3, 99 }, { 61, 62, 63, 88, 59, 58, 57, 56, 88, 52, 44, 36, 28, 20, 12, 4, 99 }, { 62, 63, 88, 60, 59, 58, 57, 56, 88, 53, 45, 37, 29, 21, 13, 5, 99 }, { 63, 88, 61, 60, 59, 58, 57, 56, 88, 54, 46, 38, 30, 22, 14, 6, 99 }, { 62, 61, 60, 59, 58, 57, 56, 88, 55, 47, 39, 31, 23, 15, 7, 99 }
};
const uint8_t knight_step[64][9] = {
  { 10, 17, 99 }, { 11, 18, 16, 99 }, { 12, 19, 17, 8, 99 }, { 13, 20, 18, 9, 99 }, { 14, 21, 19, 10, 99 }, { 15, 22, 20, 11, 99 }, { 23, 21, 12, 99 }, { 22, 13, 99 }, { 2, 18, 25, 99 }, { 3, 19, 26, 24, 99 }, { 4, 20, 27, 25, 16, 0, 99 }, { 5, 21, 28, 26, 17, 1, 99 }, { 6, 22, 29, 27, 18, 2, 99 }, { 7, 23, 30, 28, 19, 3, 99 }, { 31, 29, 20, 4, 99 }, { 30, 21, 5, 99 }, { 1, 10, 26, 33, 99 }, { 2, 11, 27, 34, 32, 0, 99 }, { 3, 12, 28, 35, 33, 24, 8, 1, 99 }, { 4, 13, 29, 36, 34, 25, 9, 2, 99 }, { 5, 14, 30, 37, 35, 26, 10, 3, 99 }, { 6, 15, 31, 38, 36, 27, 11, 4, 99 }, { 7, 39, 37, 28, 12, 5, 99 }, { 38, 29, 13, 6, 99 }, { 9, 18, 34, 41, 99 }, { 10, 19, 35, 42, 40, 8, 99 }, { 11, 20, 36, 43, 41, 32, 16, 9, 99 }, { 12, 21, 37, 44, 42, 33, 17, 10, 99 }, { 13, 22, 38, 45, 43, 34, 18, 11, 99 }, { 14, 23, 39, 46, 44, 35, 19, 12, 99 }, { 15, 47, 45, 36, 20, 13, 99 }, { 46, 37, 21, 14, 99 }, { 17, 26, 42, 49, 99 }, { 18, 27, 43, 50, 48, 16, 99 }, { 19, 28, 44, 51, 49, 40, 24, 17, 99 }, { 20, 29, 45, 52, 50, 41, 25, 18, 99 }, { 21, 30, 46, 53, 51, 42, 26, 19, 99 }, { 22, 31, 47, 54, 52, 43, 27, 20, 99 }, { 23, 55, 53, 44, 28, 21, 99 }, { 54, 45, 29, 22, 99 }, { 25, 34, 50, 57, 99 }, { 26, 35, 51, 58, 56, 24, 99 }, { 27, 36, 52, 59, 57, 48, 32, 25, 99 }, { 28, 37, 53, 60, 58, 49, 33, 26, 99 }, { 29, 38, 54, 61, 59, 50, 34, 27, 99 }, { 30, 39, 55, 62, 60, 51, 35, 28, 99 }, { 31, 63, 61, 52, 36, 29, 99 }, { 62, 53, 37, 30, 99 }, { 33, 42, 58, 99 }, { 34, 43, 59, 32, 99 }, { 35, 44, 60, 56, 40, 33, 99 }, { 36, 45, 61, 57, 41, 34, 99 }, { 37, 46, 62, 58, 42, 35, 99 }, { 38, 47, 63, 59, 43, 36, 99 }, { 39, 60, 44, 37, 99 }, { 61, 45, 38, 99 }, { 41, 50, 99 }, { 40, 42, 51, 99 }, { 43, 52, 48, 41, 99 }, { 44, 53, 49, 42, 99 }, { 45, 54, 50, 43, 99 }, { 46, 55, 51, 44, 99 }, { 47, 52, 45, 99 }, { 53, 46, 99 }
};
const uint8_t king_step[64][9] = {
  { 1, 9, 8, 99 }, { 2, 10, 9, 8, 0, 99 }, { 3, 11, 10, 9, 1, 99 }, { 4, 12, 11, 10, 2, 99 }, { 5, 13, 12, 11, 3, 99 }, { 6, 14, 13, 12, 4, 99 }, { 7, 15, 14, 13, 5, 99 }, { 15, 14, 6, 99 }, { 1, 9, 17, 16, 0, 99 }, { 2, 10, 18, 17, 16, 8, 0, 1, 99 }, { 3, 11, 19, 18, 17, 9, 1, 2, 99 }, { 4, 12, 20, 19, 18, 10, 2, 3, 99 }, { 5, 13, 21, 20, 19, 11, 3, 4, 99 }, { 6, 14, 22, 21, 20, 12, 4, 5, 99 }, { 7, 15, 23, 22, 21, 13, 5, 6, 99 }, { 23, 22, 14, 6, 7, 99 }, { 9, 17, 25, 24, 8, 99 }, { 10, 18, 26, 25, 24, 16, 8, 9, 99 }, { 11, 19, 27, 26, 25, 17, 9, 10, 99 }, { 12, 20, 28, 27, 26, 18, 10, 11, 99 }, { 13, 21, 29, 28, 27, 19, 11, 12, 99 }, { 14, 22, 30, 29, 28, 20, 12, 13, 99 }, { 15, 23, 31, 30, 29, 21, 13, 14, 99 }, { 31, 30, 22, 14, 15, 99 }, { 17, 25, 33, 32, 16, 99 }, { 18, 26, 34, 33, 32, 24, 16, 17, 99 }, { 19, 27, 35, 34, 33, 25, 17, 18, 99 }, { 20, 28, 36, 35, 34, 26, 18, 19, 99 }, { 21, 29, 37, 36, 35, 27, 19, 20, 99 }, { 22, 30, 38, 37, 36, 28, 20, 21, 99 }, { 23, 31, 39, 38, 37, 29, 21, 22, 99 }, { 39, 38, 30, 22, 23, 99 }, { 25, 33, 41, 40, 24, 99 }, { 26, 34, 42, 41, 40, 32, 24, 25, 99 }, { 27, 35, 43, 42, 41, 33, 25, 26, 99 }, { 28, 36, 44, 43, 42, 34, 26, 27, 99 }, { 29, 37, 45, 44, 43, 35, 27, 28, 99 }, { 30, 38, 46, 45, 44, 36, 28, 29, 99 }, { 31, 39, 47, 46, 45, 37, 29, 30, 99 }, { 47, 46, 38, 30, 31, 99 }, { 33, 41, 49, 48, 32, 99 }, { 34, 42, 50, 49, 48, 40, 32, 33, 99 }, { 35, 43, 51, 50, 49, 41, 33, 34, 99 }, { 36, 44, 52, 51, 50, 42, 34, 35, 99 }, { 37, 45, 53, 52, 51, 43, 35, 36, 99 }, { 38, 46, 54, 53, 52, 44, 36, 37, 99 }, { 39, 47, 55, 54, 53, 45, 37, 38, 99 }, { 55, 54, 46, 38, 39, 99 }, { 41, 49, 57, 56, 40, 99 }, { 42, 50, 58, 57, 56, 48, 40, 41, 99 }, { 43, 51, 59, 58, 57, 49, 41, 42, 99 }, { 44, 52, 60, 59, 58, 50, 42, 43, 99 }, { 45, 53, 61, 60, 59, 51, 43, 44, 99 }, { 46, 54, 62, 61, 60, 52, 44, 45, 99 }, { 47, 55, 63, 62, 61, 53, 45, 46, 99 }, { 63, 62, 54, 46, 47, 99 }, { 49, 57, 48, 99 }, { 50, 58, 56, 48, 49, 99 }, { 51, 59, 57, 49, 50, 99 }, { 52, 60, 58, 50, 51, 99 }, { 53, 61, 59, 51, 52, 99 }, { 54, 62, 60, 52, 53, 99 }, { 55, 63, 61, 53, 54, 99 }, { 62, 54, 55, 99 }
};

// =============================================================
// [5] 함수 정의
// =============================================================

//wifi 함수

bool loadWifiConfig(String &ssid, String &password) {
  File file = LittleFS.open(WIFI_FILE, "r");
  if (!file) return false;
  ssid = file.readStringUntil('\n');
  password = file.readStringUntil('\n');
  file.close();
  ssid.trim();
  password.trim();
  return (ssid.length() > 0);
}

void saveWifiConfig(String ssid, String password) {
  File file = LittleFS.open(WIFI_FILE, "w");
  if (!file) {
    Serial.println("[WiFi] Failed to open config file for writing.");
    return;
  }
  file.println(ssid);
  file.println(password);
  file.close();
  Serial.println("[WiFi] Config saved.");
}

void scanAndConnectWifi() {
  Serial.println("[WiFi] Scanning networks...");

  WiFi.disconnect(true);  // 기존 연결 완전 초기화
  delay(100);
  WiFi.mode(WIFI_STA);  // STA 모드 명시적 설정
  delay(100);

  int n = WiFi.scanNetworks();
  if (n == 0) {
    Serial.println("[WiFi] No networks found.");
  } else {
    Serial.println("[WiFi] Available networks:");
    for (int i = 0; i < n; i++) {
      Serial.printf("  [%d] %s (%d dBm)\n", i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i));
    }
  }

  Serial.println("[WiFi] Enter SSID:");
  String newSSID = "";
  while (newSSID == "") {
    if (Serial.available()) {
      newSSID = Serial.readStringUntil('\n');
      newSSID.trim();
    }
    delay(10);
  }

  Serial.println("[WiFi] Enter Password (leave blank if open):");
  String newPW = "";
  unsigned long pwStart = millis();
  while (millis() - pwStart < 30000) {
    if (Serial.available()) {
      newPW = Serial.readStringUntil('\n');
      newPW.trim();
      break;
    }
    delay(10);
  }

  WiFi.disconnect(true);  // 연결 시도 전 한번 더 초기화
  delay(100);

  Serial.printf("[WiFi] Connecting to %s ...\n", newSSID.c_str());
  WiFi.begin(newSSID.c_str(), newPW.c_str());

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
    delay(500);
    Serial.print(".");
    retry++;
    handleRingNetwork();
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    saveWifiConfig(newSSID, newPW);
  } else {
    Serial.println("\n[WiFi] Failed. Retrying...");
    scanAndConnectWifi();
  }
}

void initWifi() {
  String savedSSID = "", savedPW = "";

  WiFi.disconnect(true);  // 추가
  delay(100);
  WiFi.mode(WIFI_STA);  // 추가
  delay(100);

  if (loadWifiConfig(savedSSID, savedPW)) {
    Serial.printf("[WiFi] Saved config found: %s\n", savedSSID.c_str());
    WiFi.begin(savedSSID.c_str(), savedPW.c_str());

    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry < 20) {
      delay(500);
      Serial.print(".");
      retry++;
      handleRingNetwork();
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\n[WiFi] Connected!");
      Serial.print("IP: ");
      Serial.println(WiFi.localIP());
      return;
    }

    Serial.println("\n[WiFi] Saved config failed. Rescanning...");
  } else {
    Serial.println("[WiFi] No saved config found.");
  }

  scanAndConnectWifi();
}



// ---- (1) 링 통신 & 스마트 파서 ----

int processSmartCommand(String raw) {
  raw.trim();
  String finalProtocolMsg = "";
  int startIndex = 0;
  int semiIndex = raw.indexOf(';', startIndex);

  bool sentNodes[TOTAL_SLAVES + 1] = { false };
  uint8_t commandedMask = 0;

  while (startIndex < raw.length()) {
    int endIndex = (semiIndex == -1) ? raw.length() : semiIndex;
    String token = raw.substring(startIndex, endIndex);
    token.trim();

    int eqIndex = token.indexOf('=');
    if (eqIndex != -1 && token.length() >= 3) {
      String key = token.substring(0, eqIndex);
      String val = token.substring(eqIndex + 1);
      key.trim();
      val.trim();

      char motorChar = key.charAt(0);
      String nodeStr = key.substring(1);

      int motorIdx = -1;
      if (motorChar >= 'a' && motorChar <= 'h') motorIdx = motorChar - 'a';
      else if (motorChar >= 'A' && motorChar <= 'H') motorIdx = motorChar - 'A';

      int nodeId = nodeStr.toInt();
      int targetPos = val.toInt();

      if (motorIdx != -1 && nodeId >= 1) {
  int mask = (1 << motorIdx);
  if (finalProtocolMsg.length() > 0) finalProtocolMsg += ";";
  finalProtocolMsg += String(nodeId) + ":move(" + String(mask) + ", " + String(targetPos) + ")";

  if (nodeId <= TOTAL_SLAVES) {
    commandedMask |= (1 << (nodeId - 1));   // 비트는 중복 OR이라 무방
    g_moveGen[nodeId]++;                     // ★ 세그먼트마다 +1 (슬레이브 myMoveGen과 동일 폭)
  }
}
    }

    if (semiIndex == -1) break;
    startIndex = semiIndex + 1;
    semiIndex = raw.indexOf(';', startIndex);
  }

  if (finalProtocolMsg.length() > 0) {
    Serial.print("[TX] ");
    Serial.println(raw);
    RING_SERIAL.println(finalProtocolMsg);
  }

  return commandedMask;  // 노드 수 → 노드 비트마스크
}

void sendMoveToken(int seq) {
  String gens = "";
  for (int i = 1; i <= TOTAL_SLAVES; i++) {
    if (i > 1) gens += ",";
    gens += String(g_moveGen[i]);
  }
  RING_SERIAL.println("TOKMOVE:" + String(seq) + ":" + gens + ":0");
}

// 이동 완료 대기: g_expectedMask 노드들이 자기 세대를 다 채워 돌아올 때까지
bool waitMoveSettle(unsigned long timeoutMs) {
  Serial.printf("[DIAG] settle ENTER expected=0x%02X\n", g_expectedMask);
  if (g_expectedMask == 0) { Serial.println("[DIAG] settle SKIP mask=0"); return true; }
  g_tokenSeq++;
  g_gotMask = 0;
  sendMoveToken(g_tokenSeq);
  unsigned long tokenSent = millis(), startAll = millis();
  int retry = 0;

  while (true) {
    handleRingNetwork();
    pollThinkAbort();
    if (g_abortThink) return false;

    if ((g_gotMask & g_expectedMask) == g_expectedMask) {
      Serial.printf("[DIAG] settle DONE in %lums got=0x%02X\n", millis() - startAll, g_gotMask);
      return true;
    }

    if (millis() - tokenSent > TOKEN_REISSUE_INTERVAL) {
      if (++retry > TOKEN_MAX_RETRY || millis() - startAll > timeoutMs) {
        Serial.printf("[WARN] TOKMOVE timeout. missing mask=0x%02X\n",
                      (uint8_t)(g_expectedMask & ~g_gotMask));
        return false;
      }
      g_tokenSeq++;
      g_gotMask = 0;
      sendMoveToken(g_tokenSeq);
      tokenSent = millis();
    }
    delay(5);
  }
}

// TOKHOME / TOKAWAKE 공용 (세대 없음)
bool waitSimpleToken(const char *kind, uint8_t expectedMask, unsigned long timeoutMs) {
  g_tokenSeq++;
  g_gotMask = 0;
  RING_SERIAL.println(String(kind) + ":" + String(g_tokenSeq) + ":0");
  unsigned long tokenSent = millis(), startAll = millis();
  int retry = 0;

  while (true) {
    handleRingNetwork();
    if ((g_gotMask & expectedMask) == expectedMask) return true;

    if (millis() - tokenSent > TOKEN_REISSUE_INTERVAL) {
      if (++retry > TOKEN_MAX_RETRY || millis() - startAll > timeoutMs) {
        Serial.printf("[WARN] %s timeout. missing=0x%02X\n",
                      kind, (uint8_t)(expectedMask & ~g_gotMask));
        return false;
      }
      g_tokenSeq++;
      g_gotMask = 0;
      RING_SERIAL.println(String(kind) + ":" + String(g_tokenSeq) + ":0");
      tokenSent = millis();
    }
    delay(5);
  }
}

void handleRingNetwork() {
  while (RING_SERIAL.available()) {
    char c = RING_SERIAL.read();
    if (c == '\n') {
      ringBuffer.trim();
      if (ringBuffer.length() > 0) {

        // ============================================================
        // [토큰] TOKMOVE / TOKHOME / TOKAWAKE 회수
        //   공통 규칙: 첫 콜론 뒤 = seq, 마지막 콜론 뒤 = mask
        // ============================================================
        if (ringBuffer.startsWith("TOK")) {
          int firstColon = ringBuffer.indexOf(':');
          int secondColon = ringBuffer.indexOf(':', firstColon + 1);
          int lastColon = ringBuffer.lastIndexOf(':');
          if (firstColon != -1 && secondColon != -1 && lastColon >= secondColon) {
            int seq = ringBuffer.substring(firstColon + 1, secondColon).toInt();
            uint8_t mask = (uint8_t)ringBuffer.substring(lastColon + 1).toInt();
            if (seq == g_tokenSeq) g_gotMask = mask;  // 현재 발행 토큰만 인정(옛 토큰 무시)
          }
        }

        // [D] 하트비트 처리
        if (ringBuffer.equals("hb:ping")) {
          if (isHbWaiting) {
            isHbWaiting = false;
            lastHbTime = millis();
            hbRetryCount = 0;
          }
        }
      }
      ringBuffer = "";
    } else {
      ringBuffer += c;
    }
  }

  // 하트비트 타임아웃 처리
  if (!isHbWaiting && (millis() - lastHbTime >= HB_INTERVAL)) {
    RING_SERIAL.println("hb:ping");
    hbSendTime = millis();
    isHbWaiting = true;
    hbRetryCount = 0;
  }
}

// ---- (2) 통합 연동 함수 ----

int convertPieceToID(int pieceValue) {
  if (pieceValue == 0) return ID_EMPTY;
  int absVal = abs(pieceValue);
  bool isWhite = (pieceValue > 0);
  switch (absVal) {
    case fp: return isWhite ? ID_W_PAWN : ID_B_PAWN;
    case fn: return isWhite ? ID_W_KNIGHT : ID_B_KNIGHT;
    case fb: return isWhite ? ID_W_BISHOP : ID_B_BISHOP;
    case fr: return isWhite ? ID_W_ROOK : ID_B_ROOK;
    case fq: return isWhite ? ID_W_QUEEN : ID_B_QUEEN;
    case fk: return isWhite ? ID_W_KING : ID_B_KING;
    default: return ID_EMPTY;
  }
}

String getCoordString(int index) {
  int engineCol = column[index];
  int engineRow = row[index];

  if (g_playerIsWhite) {
    // 백 시점 (기존과 동일)
    char colChar = 'a' + (engineCol - 1);
    return String(colChar) + String(engineRow);
  } else {
    // 흑 시점: 물리적 보드 180도 회전 (원점 대칭)
    // 'a'(1) -> 'h'(8), 1행 -> 8행
    char colChar = 'a' + (8 - engineCol);
    int invertedRow = 9 - engineRow;
    return String(colChar) + String(invertedRow);
  }
}

// pole 인덱스(0-63) → NeoPixel LED 인덱스(0-79)
// 체스판은 키보드 col 0-7(a-h)에 매핑, col 8-9는 비체스 영역
// White 시점: rank1=키보드 row7(하단), rank8=row0(상단)
int chessToLedIndex(int poleIdx) {
  int file = column[poleIdx] - 1;  // 0(a) ~ 7(h)
  int rank = row[poleIdx];          // 1 ~ 8
  if (g_playerIsWhite) {
    return (8 - rank) * 10 + file;
  } else {
    return (rank - 1) * 10 + (7 - file);
  }
}

void updateChessLEDs() {
  for (int i = 0; i < NUMPIXELS; i++) strip.setPixelColor(i, 0);  // 전체 소등
  for (int i = 0; i < 64; i++) {
    if (pole[i] == 0) continue;
    int ledIdx = chessToLedIndex(i);
    if (ledIdx < 0 || ledIdx >= NUMPIXELS) continue;
    if (pole[i] > 0)
      strip.setPixelColor(ledIdx, strip.Color(255, 255, 255));  // 백 기물 = 흰색
    else
      strip.setPixelColor(ledIdx, strip.Color(255, 80, 0));     // 흑 기물 = 오렌지
  }
  strip.show();
}

// 매트릭스 스캔: 행을 순서대로 LOW로 구동해 열 상태를 읽음
void scanMatrix() {
  for (int r = 0; r < MATRIX_ROWS; r++) {
    pinMode(rowPins[r], OUTPUT);
    digitalWrite(rowPins[r], LOW);
    delayMicroseconds(10);
    for (int c = 0; c < MATRIX_COLS; c++) {
      rawState[r][c] = !digitalRead(colPins[c]);
    }
    pinMode(rowPins[r], INPUT);
  }
}

void debounceMatrix() {
  for (int r = 0; r < MATRIX_ROWS; r++)
    for (int c = 0; c < MATRIX_COLS; c++) {
      if (rawState[r][c] != lastRawState[r][c])
        lastDebounceTime[r][c] = millis();
      if ((millis() - lastDebounceTime[r][c]) > debounceDelay)
        debouncedState[r][c] = rawState[r][c];
      lastRawState[r][c] = rawState[r][c];
    }
}

void game(boolean playerIsWhite, bool doInit = true);  // forward declaration

void updateSelectLEDs() {
  // 색상 선택 버튼 (하늘색)
  strip.setPixelColor(LED_BTN_WHITE,
    (g_colorSelect == 0 || g_colorSelect == 2) ? strip.Color(0, 200, 255) : 0);
  strip.setPixelColor(LED_BTN_BLACK,
    (g_colorSelect == 1 || g_colorSelect == 2) ? strip.Color(0, 200, 255) : 0);
  strip.setPixelColor(LED_BTN_START, g_selectMode ? strip.Color(0, 200, 255) : 0);
  // 타임컨트롤 버튼 (노란색)
  uint32_t tcy = strip.Color(255, 180, 0);
  strip.setPixelColor(LED_TC_5,      g_timeControl == TC_5_0       ? tcy : 0);
  strip.setPixelColor(LED_TC_10,     g_timeControl == TC_10_0      ? tcy : 0);
  strip.setPixelColor(LED_TC_10_5,   g_timeControl == TC_10_5      ? tcy : 0);
  strip.setPixelColor(LED_TC_15_10,  g_timeControl == TC_15_10     ? tcy : 0);
  strip.setPixelColor(LED_TC_30,     g_timeControl == TC_30_0      ? tcy : 0);
  strip.setPixelColor(LED_TC_INF,    g_timeControl == TC_UNLIMITED  ? tcy : 0);
  strip.show();
}

void enterSelectMode() {
  g_selectMode  = true;
  g_colorSelect = 0;  // 기본값: 백
  updateSelectLEDs();
}

void processSelectButtons() {
  if (!g_selectMode) return;
  static bool prevWi = false, prevBl = false, prevSt = false;
  static bool bothLock = false;
  static unsigned long lastLedRefresh = 0;

  bool wi = debouncedState[BTN_WHITE_R][BTN_WHITE_C];
  bool bl = debouncedState[BTN_BLACK_R][BTN_BLACK_C];
  bool st = debouncedState[BTN_START_R][BTN_START_C];

  bool changed = false;

  if (wi && bl) {
    // 두 버튼 동시 누름: 처음 감지 시에만 랜덤 설정
    if (!bothLock) {
      g_colorSelect = 2;
      bothLock = true;
      changed = true;
    }
  } else {
    if (bothLock) {
      // 동시 누름 해제 중: 둘 다 떼야 잠금 해제
      if (!wi && !bl) bothLock = false;
    } else {
      // 단일 버튼은 누르는 순간(rising edge)에만 처리
      if (wi && !prevWi) { g_colorSelect = 0; changed = true; }
      if (bl && !prevBl) { g_colorSelect = 1; changed = true; }
    }
  }

  // 타임컨트롤 버튼 rising edge 감지
  static bool pTC0=false, pTC1=false, pTC2=false, pTC3=false, pTC4=false, pTC5=false;
  bool tc0 = debouncedState[BTN_TC5_R][BTN_TC5_C];
  bool tc1 = debouncedState[BTN_TC10_R][BTN_TC10_C];
  bool tc2 = debouncedState[BTN_TC10_5_R][BTN_TC10_5_C];
  bool tc3 = debouncedState[BTN_TC15_10_R][BTN_TC15_10_C];
  bool tc4 = debouncedState[BTN_TC30_R][BTN_TC30_C];
  bool tc5 = debouncedState[BTN_TCINF_R][BTN_TCINF_C];
  if (tc0 && !pTC0) { g_timeControl = TC_5_0;       changed = true; }
  if (tc1 && !pTC1) { g_timeControl = TC_10_0;      changed = true; }
  if (tc2 && !pTC2) { g_timeControl = TC_10_5;      changed = true; }
  if (tc3 && !pTC3) { g_timeControl = TC_15_10;     changed = true; }
  if (tc4 && !pTC4) { g_timeControl = TC_30_0;      changed = true; }
  if (tc5 && !pTC5) { g_timeControl = TC_UNLIMITED; changed = true; }
  pTC0=tc0; pTC1=tc1; pTC2=tc2; pTC3=tc3; pTC4=tc4; pTC5=tc5;

  if (changed || millis() - lastLedRefresh > 200) {
    updateSelectLEDs();
    lastLedRefresh = millis();
  }

  // J8 에지 감지 → 게임 시작
  if (st && !prevSt) {
    g_selectMode = false;
    updateSelectLEDs();
    bool playWhite = (g_colorSelect == 0) ? true
                   : (g_colorSelect == 1) ? false
                   : (random(2) == 0);
    game(playWhite);
    enterSelectMode();
  }

  prevWi = wi;
  prevBl = bl;
  prevSt = st;
}

// ---- 게임 키보드 입력 & 기물 선택 UI ----

// 프로모션 버튼 점등 후 키 선택 대기. 반환값: 4=나이트 5=비숍 6=룩 7=퀸
int waitPromoChoice() {
  VFD_print("PROMOTION!");
  strip.setPixelColor(LED_PROMO_Q, strip.Color(0, 200, 255));
  strip.setPixelColor(LED_PROMO_R, strip.Color(0, 200, 255));
  strip.setPixelColor(LED_PROMO_B, strip.Color(0, 200, 255));
  strip.setPixelColor(LED_PROMO_N, strip.Color(0, 200, 255));
  strip.show();

  bool prevQ=false, prevR=false, prevB=false, prevN=false;
  int selected = 7;  // 기본 퀸
  bool done = false;
  while (!done) {
    handleRingNetwork();
    clockUpdate();
    scanMatrix();
    debounceMatrix();
    bool pq = debouncedState[BTN_PROMO_Q_R][BTN_PROMO_Q_C];
    bool pr = debouncedState[BTN_PROMO_R_R][BTN_PROMO_R_C];
    bool pb = debouncedState[BTN_PROMO_B_R][BTN_PROMO_B_C];
    bool pn = debouncedState[BTN_PROMO_N_R][BTN_PROMO_N_C];
    if (pq && !prevQ) { selected = 7; done = true; }
    if (pr && !prevR) { selected = 6; done = true; }
    if (pb && !prevB) { selected = 5; done = true; }
    if (pn && !prevN) { selected = 4; done = true; }
    prevQ=pq; prevR=pr; prevB=pb; prevN=pn;
    if (!done) delay(10);
  }

  strip.setPixelColor(LED_PROMO_Q, 0);
  strip.setPixelColor(LED_PROMO_R, 0);
  strip.setPixelColor(LED_PROMO_B, 0);
  strip.setPixelColor(LED_PROMO_N, 0);
  strip.show();
  return selected;
}

int keyToSquare(int sw_r, int sw_c) {
  if (sw_c < 2) return -1;
  if (g_playerIsWhite) return (7 - sw_r) * 8 + (9 - sw_c);
  else                 return sw_r * 8 + (sw_c - 2);
}

String squareToAlg(int sq) {
  return String((char)('a' + sq % 8)) + String(8 - sq / 8);
}

void computeLegalMovesFrom(int sq) {
  memset(g_legalTo,   false, sizeof(g_legalTo));
  memset(g_captureTo, false, sizeof(g_captureTo));
  generate_steps(0);
  int n = pos[0].n_steps;
  step_t buf[MAXSTEPS];
  for (int i = 0; i < n; i++) buf[i] = pos[0].steps[i];
  for (int i = 0; i < n; i++) {
    if (buf[i].c1 != sq) continue;
    movestep(0, buf[i]);
    bool safe = (pos[0].w == 1) ? !check_w() : !check_b();
    backstep(0, buf[i]);
    if (!safe) continue;
    int dst = buf[i].c2;
    if (pole[dst] != 0 || buf[i].type == 3)
      g_captureTo[dst] = true;
    else
      g_legalTo[dst] = true;
  }
}

void showGameLEDs(bool blinkOn) {
  for (int i = 0; i < NUMPIXELS; i++) strip.setPixelColor(i, 0);
  for (int i = 0; i < 64; i++) {
    if (pole[i] == 0 || i == g_selSq) continue;
    int led = chessToLedIndex(i);
    if (led < 0 || led >= NUMPIXELS) continue;
    strip.setPixelColor(led, pole[i] > 0 ? strip.Color(255,255,255) : strip.Color(255,80,0));
  }
  for (int i = 0; i < 64; i++) {
    int led = chessToLedIndex(i);
    if (led < 0 || led >= NUMPIXELS) continue;
    if (g_legalTo[i])   strip.setPixelColor(led, strip.Color(0, 200, 0));
    if (g_captureTo[i]) strip.setPixelColor(led, strip.Color(200, 0, 0));
  }
  if (g_selSq != -1 && blinkOn) {
    int led = chessToLedIndex(g_selSq);
    if (led >= 0 && led < NUMPIXELS)
      strip.setPixelColor(led, pole[g_selSq] > 0 ? strip.Color(255,255,255) : strip.Color(255,80,0));
  }
  strip.show();
}

void clearGameSelection() {
  g_selSq = -1;
  memset(g_legalTo,   false, sizeof(g_legalTo));
  memset(g_captureTo, false, sizeof(g_captureTo));
  updateChessLEDs();
}

void handleSquarePress(int sq, bool playerIsWhite, String &outMove) {
  bool isMyPiece = playerIsWhite ? (pole[sq] > 0) : (pole[sq] < 0);
  if (g_selSq == -1) {
    if (isMyPiece) {
      g_selSq = sq;
      computeLegalMovesFrom(sq);
      showGameLEDs(true);
    }
  } else if (sq == g_selSq) {
    clearGameSelection();
  } else if (g_legalTo[sq] || g_captureTo[sq]) {
    outMove = squareToAlg(g_selSq) + squareToAlg(sq);
    clearGameSelection();
  } else if (isMyPiece) {
    // 다른 자기 기물 누르면 재선택
    g_selSq = sq;
    computeLegalMovesFrom(sq);
    showGameLEDs(true);
  } else {
    clearGameSelection();
  }
}

void updateBoardDiff() {
  String diffCommand = "";
  bool changed = false;

  for (int i = 0; i < 64; i++) {
    if (pole[i] != prev_pole[i]) {
      String coord = getCoordString(i);
      int id = convertPieceToID(pole[i]);
      if (diffCommand.length() > 0) diffCommand += ";";
      diffCommand += coord + "=" + String(id);
      prev_pole[i] = pole[i];
      changed = true;
    }
  }

  if (changed) {
    g_expectedMask = processSmartCommand(diffCommand);
    Serial.printf("[DIAG] diff=%s  expectedMask=0x%02X\n", diffCommand.c_str(), g_expectedMask);  // ← 임시
    updateChessLEDs();
  } else {
    Serial.println("[DIAG] no diff, expectedMask unchanged");  // ← 임시
  }
}

void sendKingStatus(bool isWhiteKing, int statusID) {
  int kingPos = -1;
  int targetVal = isWhiteKing ? fk : -fk;

  for (int i = 0; i < 64; i++) {
    if (pole[i] == targetVal) {
      kingPos = i;
      break;
    }
  }

  if (kingPos != -1) {
    String coord = getCoordString(kingPos);
    String cmd = coord + "=" + String(statusID);
    processSmartCommand(cmd);
    Serial.print("[STATUS] King Update: ");
    Serial.println(cmd);
  }
}

void forceFullSync() {
  String fullCommand = "";
  for (int i = 0; i < 64; i++) {
    prev_pole[i] = pole[i];
    if (pole[i] == 0) continue;
    String coord = getCoordString(i);
    int id = convertPieceToID(pole[i]);
    if (fullCommand.length() > 0) fullCommand += ";";
    fullCommand += coord + "=" + String(id);
  }
  if (fullCommand.length() > 0) {
    g_expectedMask = processSmartCommand(fullCommand);
    waitMoveSettle(8000);
  } else {
    g_expectedMask = 0;
  }
  updateChessLEDs();
}

// ---- (3) 유틸리티 함수 ----

int strToPos(String s) {
  if (s.length() < 2) return -1;
  int col = s[0] - 'a';
  int row = 8 - (s[1] - '0');
  return row * 8 + col;
}

String get_time(long tim) {
  char sz[10];
  if (tim > 360000) tim = 0;
  sprintf(sz, "%02d:%02d:%02d", tim / 3600, (tim % 3600) / 60, tim % 60);
  return String(sz);
}

// ---- (4) 체스 엔진 로직 ----

void init_filesystem() {
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS Mount Failed");
    return;
  }
  if (!LittleFS.exists(BOOK_FILE)) {
    File file = LittleFS.open(BOOK_FILE, "w");
    if (file) {
      file.println("FEN|MOVE");
      file.close();
    }
  }
}

String readMoveFromFlash(String currentFen) {
  File file = LittleFS.open(BOOK_FILE, "r");
  if (!file) return "";
  int spaceIdx = currentFen.indexOf(' ', currentFen.indexOf(' ') + 1);
  String shortFen = (spaceIdx > 0) ? currentFen.substring(0, spaceIdx) : currentFen;

  String candidates[16];  // 한 포지션당 최대 후보 수
  int count = 0;
  while (file.available() && count < 16) {
    String line = file.readStringUntil('\n');
    line.trim();
    int separator = line.lastIndexOf('|');
    if (separator > 0) {
      String storedFen = line.substring(0, separator);
      if (storedFen.equals(shortFen)) {
        String mv = line.substring(separator + 1);
        // 같은 수 중복 방지
        bool dup = false;
        for (int i = 0; i < count; i++)
          if (candidates[i] == mv) {
            dup = true;
            break;
          }
        if (!dup) candidates[count++] = mv;
      }
    }
  }
  file.close();

  if (count == 0) return "";
  return candidates[random(count)];  // 모인 후보 중 무작위 선택
}

void saveMoveToFlash(String currentFen, String move) {
  int spaceIdx = currentFen.indexOf(' ', currentFen.indexOf(' ') + 1);
  String shortFen = (spaceIdx > 0) ? currentFen.substring(0, spaceIdx) : currentFen;
  String target = shortFen + "|" + move;

  // 동일한 FEN+move 줄이 이미 있으면 저장 생략 (다른 수는 통과)
  File rf = LittleFS.open(BOOK_FILE, "r");
  if (rf) {
    while (rf.available()) {
      String line = rf.readStringUntil('\n');
      line.trim();
      if (line == target) {
        rf.close();
        return;
      }
    }
    rf.close();
  }

  File file = LittleFS.open(BOOK_FILE, "a");
  if (!file) return;
  file.println(target);
  file.close();
}

String getLichessMove(String fenStr) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("[WiFi Fail]");
    return "";
  }

  fenStr.replace(" ", "%20");  // URL Encoding
  String url = "https://explorer.lichess.ovh/masters?fen=" + fenStr;

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, url);
  http.addHeader("Authorization", "Bearer " + String(LICHESS_TOKEN));
  int code = http.GET();
  Serial.print("\n[HTTP ");
  Serial.print(code);
  Serial.print("] ");
  String bestMove = "";

  if (code == 200) {
    String payload = http.getString();
    DynamicJsonDocument doc(8192);
    DeserializationError error = deserializeJson(doc, payload);
    if (error) {
      Serial.print("[JSON err: ");
      Serial.print(error.c_str());
      Serial.print("] ");
    } else {
      JsonArray moves = doc["moves"];
      Serial.print("[moves=");
      Serial.print(moves.size());
      Serial.print("] ");
      if (moves.size() > 0) {
        long totalWeight = 0;
        int candidates = moves.size() > 5 ? 5 : moves.size();
        for (int i = 0; i < candidates; i++) totalWeight += (long)moves[i]["white"] + (long)moves[i]["black"] + (long)moves[i]["draws"];
        if (totalWeight > 0) {
          long rnd = random(totalWeight);
          long currentWeight = 0;
          for (int i = 0; i < candidates; i++) {
            currentWeight += (long)moves[i]["white"] + (long)moves[i]["black"] + (long)moves[i]["draws"];
            if (rnd < currentWeight) {
              bestMove = moves[i]["uci"].as<String>();
              break;
            }
          }
        }
      }
    }
  }
  http.end();
  return bestMove;
}

String str_pole(int i) {
  return String(char('a' + i % 8) + String(8 - i / 8));
}

void kingpositions() {
  for (int i = 0; i < 64; i++) {
    if (pole[i] == fk) poswk = i;
    if (pole[i] == -fk) posbk = i;
  }
}

String str_step(step_t st) {
  String s = "";
  if (st.f1 == 0) return s;
  if (st.type == 2) s = "0-0";
  else if (st.type == 3) s = "0-0-0";
  else {
    if (abs(st.f1) > 1) s = s + fig_symb[abs(st.f1)];
    s = s + str_pole(st.c1);
    if (st.f2 == 0) s = s + "-";
    if (st.f2 != 0) s = s + "x";
    s = s + str_pole(st.c2);
  }
  if (st.type > 3) s = s + "=" + fig_symb[st.type - 2];
  if (st.check == 1) s = s + "+";
  else if (st.check == 2) s = s + "#";
  return s;
}

String fenout(int l) {
  String s = "";
  for (int r = 0; r < 8; r++) {
    if (r > 0) s = s + "/";
    int empty = 0;
    for (int c = 0; c < 8; c++) {
      int f = pole[c + r * 8];
      if (f == 0) empty++;
      if (f != 0 || c == 7)
        if (empty > 0) {
          s = s + String(empty);
          empty = 0;
        }
      switch (f) {
        case fp: s = s + "P"; break;
        case -fp: s = s + "p"; break;
        case fn: s = s + "N"; break;
        case -fn: s = s + "n"; break;
        case fb: s = s + "B"; break;
        case -fb: s = s + "b"; break;
        case fr: s = s + "R"; break;
        case -fr: s = s + "r"; break;
        case fq: s = s + "Q"; break;
        case -fq: s = s + "q"; break;
        case fk: s = s + "K"; break;
        case -fk: s = s + "k"; break;
      }
    }
  }
  if (pos[l].w == 1) s = s + " w ";
  else s = s + " b ";
  if (pos[l].wrk + pos[l].wrq + pos[l].brk + pos[l].brq == 0) s = s + "-";
  else {
    if (pos[l].wrk) s = s + "K";
    if (pos[l].wrq) s = s + "Q";
    if (pos[l].brk) s = s + "k";
    if (pos[l].brq) s = s + "q";
  }
  if (pos[l].pp != 0) s = s + " " + str_pole(pos[l].pp);
  else s = s + " -";
  return s;
}

// [수정됨] 캐슬링 파싱 버그 수정 완료
boolean fen(String ss) {
  char s = 'x', i = 0, j = 0;
  boolean load = false;
  for (int k = 0; k < 64; k++) pole[k] = 0;
  pos[0].w = 1;
  pos[0].wrk = 0;
  pos[0].wrq = 0;
  pos[0].brk = 0;
  pos[0].brq = 0;
  pos[0].pp = 0;
  pos[0].cur_step = 0;
  pos[0].n_steps = 0;
  int spaces = 0;
  for (int c = 0; c < ss.length(); c++) {
    s = ss[c];
    if (spaces == 0 && i > 7) {
      i = 0;
      j++;
    }

    if (s == ' ') {
      spaces++;
      continue;
    }

    if (spaces == 3) {
      if (int(s) >= 'a' && int(s) <= 'h') {
        char s1 = ss[c + 1];
        if (int(s1) >= '1' && int(s1) <= '8') {
          pos[0].pp = 8 * (7 - (int(s1) - int('1'))) + int(s) - int('a');
          c++;
        }
      }
      continue;
    }

    if (spaces == 2) {
      switch (s) {
        case 'K': pos[0].wrk = 1; break;
        case 'Q': pos[0].wrq = 1; break;
        case 'k': pos[0].brk = 1; break;
        case 'q': pos[0].brq = 1; break;
      }
      continue;
    }

    if (spaces == 1) {
      if (s == 'w') {
        pos[0].w = 1;
        load = true;
      } else if (s == 'b') {
        pos[0].w = 0;
        load = true;
      }
      continue;
    }

    if (spaces == 0) {
      int l = j * 8 + i;
      if (l >= 64) continue;
      switch (s) {
        case '/': i = 0; break;
        case 'p':
          pole[l] = -fp;
          i++;
          break;
        case 'P':
          pole[l] = fp;
          i++;
          break;
        case 'n':
          pole[l] = -fn;
          i++;
          break;
        case 'N':
          pole[l] = fn;
          i++;
          break;
        case 'b':
          pole[l] = -fb;
          i++;
          break;
        case 'B':
          pole[l] = fb;
          i++;
          break;
        case 'r':
          pole[l] = -fr;
          i++;
          break;
        case 'R':
          pole[l] = fr;
          i++;
          break;
        case 'q':
          pole[l] = -fq;
          i++;
          break;
        case 'Q':
          pole[l] = fq;
          i++;
          break;
        case 'k':
          pole[l] = -fk;
          i++;
          break;
        case 'K':
          pole[l] = fk;
          i++;
          break;
        case '1': i++; break;
        case '2': i += 2; break;
        case '3': i += 3; break;
        case '4': i += 4; break;
        case '5': i += 5; break;
        case '6': i += 6; break;
        case '7': i += 7; break;
        case '8':
          i = 0;
          j++;
          break;
      }
    }
    if (spaces >= 4) break;
  }
  if (!load) Serial.println(F("Error: FEN parsing"));
  else { fenstr = ss; }
  return load;
}

void getbm(int n, String ep) {
  ep.trim();
  if (ep == "0-0") {
    for (int i = 0; i < pos[0].n_steps; i++)
      if (pos[0].steps[i].type == 2) {
        bestmove[n] = pos[0].steps[i];
        return;
      }
  } else if (ep == "0-0-0") {
    for (int i = 0; i < pos[0].n_steps; i++)
      if (pos[0].steps[i].type == 3) {
        bestmove[n] = pos[0].steps[i];
        return;
      }
  } else if (ep.indexOf("=") > -1) {
    char fi = ep.indexOf(ep.length() - 1);
    int type = 7;
    if (fi == 'N') type = 4;
    else if (fi == 'B') type = 5;
    else if (fi == 'R') type = 6;
    for (int i = 0; i < pos[0].n_steps; i++)
      if (pos[0].steps[i].type == type) {
        bestmove[n] = pos[0].steps[i];
        return;
      }
  } else {
    int posp1 = ep.indexOf("+");
    int posp2 = ep.indexOf("#");
    if (posp1 >= 0 || posp2 >= 0) ep = ep.substring(0, ep.length() - 1);
    char co = ep.charAt(ep.length() - 2);
    char ro = ep.charAt(ep.length() - 1);
    int c2 = 8 * (7 - (int(ro) - int('1'))) + int(co) - int('a');
    char fi = ep.charAt(0);
    for (int i = 0; i < pos[0].n_steps; i++) {
      if (pos[0].steps[i].c2 == c2) {
        if (fig_symb1[abs(pos[0].steps[i].f1)] == fi) {
          if (ep.length() == 3 || (ep.length() == 4 && ep.charAt(1) == 'x')) {
            bestmove[n] = pos[0].steps[i];
            return;
          } else if (int(ep.charAt(1)) - int('a') == column[pos[0].steps[i].c1] - 1) {
            bestmove[n] = pos[0].steps[i];
            return;
          }
        } else if (ep.length() == 2 && abs(pos[0].steps[i].f1) == fp) {
          bestmove[n] = pos[0].steps[i];
          return;
        }
        if (str_step(pos[0].steps[i]) == ep) bestmove[n] = pos[0].steps[i];
        else {
          String st = str_step(pos[0].steps[i]);
          st = st.substring(0, 1) + st.substring(2, st.length());
          if (pos[0].steps[i].f2 != 0 && ep.charAt(1) == 'x' && st == ep) bestmove[n] = pos[0].steps[i];
        }
      }
    }
  }
}

void show_position() {
  for (int i = 0; i < 8; i++) {
    Serial.println("-----------------------------------------");
    Serial.print("|");
    for (int j = 0; j < 8; j++) {
      signed char f = pole[i * 8 + j];
      if (f >= 0) {
        Serial.print("  ");
        Serial.print(fig_symb1[f]);
        Serial.print(" |");
      } else {
        Serial.print(" -");
        Serial.print(fig_symb1[-f]);
        Serial.print(" |");
      }
    }
    Serial.print("  ");
    Serial.println(8 - i);
  }
  Serial.println("-----------------------------------------");
  Serial.println("    a    b    c    d    e    f    g    h");
  if (pos[0].w == 0) Serial.print("Black move");
  else Serial.print("White move");
  Serial.println();
}

void movepos(int l, step_t &s) {
  pos[l + 1].wrk = pos[l].wrk;
  pos[l + 1].wrq = pos[l].wrq;
  pos[l + 1].brk = pos[l].brk;
  pos[l + 1].brq = pos[l].brq;
  pos[l + 1].pp = 0;
  if (s.f2 != 0 || abs(s.f1) == fp) pos[l + 1].halfmove_clock = 0;
  else pos[l + 1].halfmove_clock = pos[l].halfmove_clock + 1;
  uint64_t new_h = pos[l].hash;
  int old_c_idx = (pos[l].wrk << 3) | (pos[l].wrq << 2) | (pos[l].brk << 1) | pos[l].brq;
  new_h ^= zobrist_castling[old_c_idx];
  if (pos[l].pp != 0) new_h ^= zobrist_ep[pos[l].pp % 8];
  if (pos[l].w) {
    if (pos[l].wrk || pos[l].wrq) {
      if (s.c1 == 60) {
        pos[l + 1].wrk = 0;
        pos[l + 1].wrq = 0;
      } else if (s.c1 == 63) pos[l + 1].wrk = 0;
      else if (s.c1 == 56) pos[l + 1].wrq = 0;
    }
    if (s.type == 0 && s.f1 == fp && s.c2 == s.c1 - 16)
      if ((column[s.c2] > 1 && pole[s.c2 - 1] == -fp) || (column[s.c2] < 8 && pole[s.c2 + 1] == -fp)) pos[l + 1].pp = s.c1 - 8;
    pos[l + 1].weight_w = pos[l].weight_w;
    pos[l + 1].weight_b = pos[l].weight_b;
    if (s.f2 != 0) pos[l + 1].weight_b -= fig_weight[-s.f2];
    if (s.type > 3) pos[l + 1].weight_w += fig_weight[s.type - 2] - 100;
    if (stats) {
      if (s.f1 == fk && endspiel) pos[l + 1].weight_s = pos[l].weight_s + stat_weightw[6][s.c2] - stat_weightw[6][s.c1];
      else pos[l + 1].weight_s = pos[l].weight_s + stat_weightw[s.f1 - 1][s.c2] - stat_weightw[s.f1 - 1][s.c1];
      if (s.f2 != 0) pos[l + 1].weight_s += stat_weightb[-s.f2 - 1][s.c2];
    }
  } else {
    if (pos[l].brk || pos[l].brq) {
      if (s.c1 == 4) {
        pos[l + 1].brk = 0;
        pos[l + 1].brq = 0;
      } else if (s.c1 == 7) pos[l + 1].brk = 0;
      else if (s.c1 == 0) pos[l + 1].brq = 0;
    }
    if (s.type == 0 && s.f1 == -fp && s.c2 == s.c1 + 16)
      if ((column[s.c2] > 1 && pole[s.c2 - 1] == fp) || (column[s.c2] < 8 && pole[s.c2 + 1] == fp)) pos[l + 1].pp = s.c1 + 8;
    pos[l + 1].weight_w = pos[l].weight_w;
    pos[l + 1].weight_b = pos[l].weight_b;
    if (s.f2 != 0) pos[l + 1].weight_w -= fig_weight[s.f2];
    if (s.type > 3) pos[l + 1].weight_b += fig_weight[s.type - 2] - 100;
    if (stats) {
      if (s.f1 == -fk && endspiel) pos[l + 1].weight_s = pos[l].weight_s - stat_weightb[6][s.c2] + stat_weightb[6][s.c1];
      else pos[l + 1].weight_s = pos[l].weight_s - stat_weightb[-s.f1 - 1][s.c2] + stat_weightb[-s.f1 - 1][s.c1];
      if (s.f2 != 0) pos[l + 1].weight_s -= stat_weightw[s.f2 - 1][s.c2];
    }
  }
  int p1_idx = (s.f1 > 0) ? (s.f1 - 1) : (abs(s.f1) + 5);
  new_h ^= zobrist_table[p1_idx][s.c1];
  new_h ^= zobrist_table[p1_idx][s.c2];
  if (s.f2 != 0) {
    int p2_idx = (s.f2 > 0) ? (s.f2 - 1) : (abs(s.f2) + 5);
    new_h ^= zobrist_table[p2_idx][s.c2];
  }
  new_h ^= zobrist_side;
  int new_c_idx = (pos[l + 1].wrk << 3) | (pos[l + 1].wrq << 2) | (pos[l + 1].brk << 1) | pos[l + 1].brq;
  new_h ^= zobrist_castling[new_c_idx];
  if (pos[l + 1].pp != 0) new_h ^= zobrist_ep[pos[l + 1].pp % 8];
  pos[l + 1].hash = new_h;
  count++;
}

void movestep(int l, step_t &s) {
  pole[s.c1] = 0;
  pole[s.c2] = s.f1;
  if (pos[l].w) {
    if (s.f1 == fk) poswk = s.c2;
    switch (s.type) {
      case 0: return;
      case 1: pole[s.c2 + 8] = 0; break;
      case 2:
        pole[60] = 0;
        pole[61] = fr;
        pole[62] = fk;
        pole[63] = 0;
        poswk = 62;
        break;
      case 3:
        pole[60] = 0;
        pole[59] = fr;
        pole[58] = fk;
        pole[57] = 0;
        pole[56] = 0;
        poswk = 58;
        break;
      default: pole[s.c2] = s.type - 2;
    }
  } else {
    if (s.f1 == -fk) posbk = s.c2;
    switch (s.type) {
      case 0: return;
      case 1: pole[s.c2 - 8] = 0; break;
      case 2:
        pole[4] = 0;
        pole[5] = -fr;
        pole[6] = -fk;
        pole[7] = 0;
        posbk = 6;
        break;
      case 3:
        pole[4] = 0;
        pole[3] = -fr;
        pole[2] = -fk;
        pole[1] = 0;
        pole[0] = 0;
        posbk = 2;
        break;
      default: pole[s.c2] = 2 - s.type;
    }
  }
}

void backstep(int l, step_t &s) {
  pole[s.c1] = s.f1;
  pole[s.c2] = s.f2;
  if (pos[l].w) {
    if (s.f1 == fk) poswk = s.c1;
    switch (s.type) {
      case 0: return;
      case 1:
        pole[s.c2] = 0;
        pole[s.c2 + 8] = -fp;
        break;
      case 2:
        pole[60] = fk;
        pole[61] = 0;
        pole[62] = 0;
        pole[63] = fr;
        poswk = 60;
        break;
      case 3:
        pole[60] = fk;
        pole[59] = 0;
        pole[58] = 0;
        pole[57] = 0;
        pole[56] = fr;
        poswk = 60;
        break;
    }
  } else {
    if (s.f1 == -fk) posbk = s.c1;
    switch (s.type) {
      case 0: return;
      case 1:
        pole[s.c2] = 0;
        pole[s.c2 - 8] = fp;
        break;
      case 2:
        pole[4] = -fk;
        pole[5] = 0;
        pole[6] = 0;
        pole[7] = -fr;
        posbk = 4;
        break;
      case 3:
        pole[4] = -fk;
        pole[3] = 0;
        pole[2] = 0;
        pole[1] = 0;
        pole[0] = -fr;
        posbk = 4;
        break;
    }
  }
}

void add_king2(int l, int i) {
  signed char f1 = pole[i];
  signed char f2;
  int j = 0;
  while (king_step[i][j] != 99) {
    f2 = pole[king_step[i][j]];
    if (f2 == 0 || (f2 < 0 && f1 > 0) || (f2 > 0 && f1 < 0)) {
      steps2[n_steps2].type = 0;
      steps2[n_steps2].c1 = i;
      steps2[n_steps2].c2 = king_step[i][j];
      steps2[n_steps2].f1 = f1;
      steps2[n_steps2].f2 = f2;
      n_steps2++;
    }
    j++;
  }
}
void add_king(int l, int i) {
  signed char f1 = pole[i];
  signed char f2;
  int j = 0;
  while (king_step[i][j] != 99) {
    f2 = pole[king_step[i][j]];
    if (f2 == 0 || (f2 < 0 && f1 > 0) || (f2 > 0 && f1 < 0)) {
      pos[l].steps[pos[l].n_steps].type = 0;
      pos[l].steps[pos[l].n_steps].c1 = i;
      pos[l].steps[pos[l].n_steps].c2 = king_step[i][j];
      pos[l].steps[pos[l].n_steps].f1 = f1;
      pos[l].steps[pos[l].n_steps].f2 = f2;
      pos[l].n_steps++;
    }
    j++;
  }
}
void add_knight(int l, int i) {
  signed char f1 = pole[i];
  signed char f2;
  int j = 0;
  while (knight_step[i][j] != 99) {
    f2 = pole[knight_step[i][j]];
    if (f2 == 0 || (f2 < 0 && f1 > 0) || (f2 > 0 && f1 < 0)) {
      pos[l].steps[pos[l].n_steps].type = 0;
      pos[l].steps[pos[l].n_steps].c1 = i;
      pos[l].steps[pos[l].n_steps].c2 = knight_step[i][j];
      pos[l].steps[pos[l].n_steps].f1 = f1;
      pos[l].steps[pos[l].n_steps].f2 = f2;
      pos[l].n_steps++;
    }
    j++;
  }
}
void add_stra(int l, int i) {
  signed char f1 = pole[i];
  signed char f2 = 0;
  int j = 0;
  while (stra_step[i][j] != 99) {
    if (stra_step[i][j] == 88) f2 = 0;
    else if (f2 == 0) {
      f2 = pole[stra_step[i][j]];
      if (f2 == 0 || (f2 < 0 && f1 > 0) || (f2 > 0 && f1 < 0)) {
        pos[l].steps[pos[l].n_steps].type = 0;
        pos[l].steps[pos[l].n_steps].c1 = i;
        pos[l].steps[pos[l].n_steps].c2 = stra_step[i][j];
        pos[l].steps[pos[l].n_steps].f1 = f1;
        pos[l].steps[pos[l].n_steps].f2 = f2;
        pos[l].n_steps++;
      }
    }
    j++;
  }
}
void add_diag(int l, int i) {
  signed char f1 = pole[i];
  signed char f2 = 0;
  int j = 0;
  while (diag_step[i][j] != 99) {
    if (diag_step[i][j] == 88) f2 = 0;
    else if (f2 == 0) {
      f2 = pole[diag_step[i][j]];
      if (f2 == 0 || (f2 < 0 && f1 > 0) || (f2 > 0 && f1 < 0)) {
        pos[l].steps[pos[l].n_steps].type = 0;
        pos[l].steps[pos[l].n_steps].c1 = i;
        pos[l].steps[pos[l].n_steps].c2 = diag_step[i][j];
        pos[l].steps[pos[l].n_steps].f1 = f1;
        pos[l].steps[pos[l].n_steps].f2 = f2;
        pos[l].n_steps++;
      }
    }
    j++;
  }
}
void add_one(int l, int c1, int c2) {
  steps2[n_steps2].type = 0;
  steps2[n_steps2].c1 = c1;
  steps2[n_steps2].c2 = c2;
  steps2[n_steps2].f1 = pole[c1];
  steps2[n_steps2].f2 = pole[c2];
  n_steps2++;
}

boolean checkd_w() {
  signed char f2 = 0;
  int j = 0;
  while (diag_step[poswk][j] != 99) {
    if (diag_step[poswk][j] == 88) f2 = 0;
    else if (f2 == 0) {
      f2 = pole[diag_step[poswk][j]];
      if (f2 == -fb || f2 == -fq) return (true);
    }
    j++;
  }
  f2 = 0;
  j = 0;
  while (stra_step[poswk][j] != 99) {
    if (stra_step[poswk][j] == 88) f2 = 0;
    else if (f2 == 0) {
      f2 = pole[stra_step[poswk][j]];
      if (f2 == -fr || f2 == -fq) return (true);
      if (j == 0 || stra_step[poswk][j] == 88)
        if (f2 == -fk) return (true);
    }
    j++;
  }
  return (false);
}
boolean checkd_b() {
  signed char f2 = 0;
  int j = 0;
  while (diag_step[posbk][j] != 99) {
    if (diag_step[posbk][j] == 88) f2 = 0;
    else if (f2 == 0) {
      f2 = pole[diag_step[posbk][j]];
      if (f2 == fb || f2 == fq) return (true);
    }
    j++;
  }
  f2 = 0;
  j = 0;
  while (stra_step[posbk][j] != 99) {
    if (stra_step[posbk][j] == 88) f2 = 0;
    else if (f2 == 0) {
      f2 = pole[stra_step[posbk][j]];
      if (f2 == fr || f2 == fq) return (true);
      if (j == 0 || stra_step[posbk][j] == 88)
        if (f2 == fk) return (true);
    }
    j++;
  }
  return (false);
}
boolean check_w() {
  signed char f2 = 0;
  int j = 0;
  if (pole[poswk] != fk) {
    for (int i = 0; i < 64; i++)
      if (pole[i] == fk) {
        poswk = i;
        break;
      }
  }
  while (diag_step[poswk][j] != 99) {
    if (diag_step[poswk][j] == 88) f2 = 0;
    else if (f2 == 0) {
      f2 = pole[diag_step[poswk][j]];
      if (f2 == -fb || f2 == -fq) return (true);
    }
    j++;
  }
  f2 = 0;
  j = 0;
  while (stra_step[poswk][j] != 99) {
    if (stra_step[poswk][j] == 88) f2 = 0;
    else if (f2 == 0) {
      f2 = pole[stra_step[poswk][j]];
      if (f2 == -fr || f2 == -fq) return (true);
      if (j == 0 || stra_step[poswk][j] == 88)
        if (f2 == -fk) return (true);
    }
    j++;
  }
  j = 0;
  while (knight_step[poswk][j] != 99) {
    if (pole[knight_step[poswk][j]] == -fn) return (true);
    j++;
  }
  if (row[poswk] < 7) {
    if (column[poswk] > 1 && pole[poswk - 9] == -fp) return (true);
    if (column[poswk] < 8 && pole[poswk - 7] == -fp) return (true);
  }
  j = 0;
  while (king_step[poswk][j] != 99) {
    if (pole[king_step[poswk][j]] == -fk) return (true);
    j++;
  }
  return (false);
}
boolean check_b() {
  signed char f2 = 0;
  int j = 0;
  if (pole[posbk] != -fk) {
    for (int i = 0; i < 64; i++)
      if (pole[i] == -fk) {
        posbk = i;
        break;
      }
  }
  while (diag_step[posbk][j] != 99) {
    if (diag_step[posbk][j] == 88) f2 = 0;
    else if (f2 == 0) {
      f2 = pole[diag_step[posbk][j]];
      if (f2 == fb || f2 == fq) return (true);
    }
    j++;
  }
  f2 = 0;
  j = 0;
  while (stra_step[posbk][j] != 99) {
    if (stra_step[posbk][j] == 88) f2 = 0;
    else if (f2 == 0) {
      f2 = pole[stra_step[posbk][j]];
      if (f2 == fr || f2 == fq) return (true);
      if (j == 0 || stra_step[posbk][j] == 88)
        if (f2 == fk) return (true);
    }
    j++;
  }
  j = 0;
  while (knight_step[posbk][j] != 99) {
    if (pole[knight_step[posbk][j]] == fn) return (true);
    j++;
  }
  if (row[posbk] > 2) {
    if (column[posbk] > 1 && pole[posbk + 7] == fp) return (true);
    if (column[posbk] < 8 && pole[posbk + 9] == fp) return (true);
  }
  j = 0;
  while (king_step[posbk][j] != 99) {
    if (pole[king_step[posbk][j]] == fk) return (true);
    j++;
  }
  return (false);
}

void sort_steps(int l) {
  step_t buf;
  for (int i = 0; i < pos[l].n_steps - 1; i++) {
    int maxweight = pos[l].steps[i].weight;
    int maxj = i;
    for (int j = i + 1; j < pos[l].n_steps; j++)
      if (pos[l].steps[j].weight > maxweight) {
        maxweight = pos[l].steps[j].weight;
        maxj = j;
      }
    if (maxweight == 0 && l > 0) return;
    if (maxj == i) continue;
    buf = pos[l].steps[i];
    pos[l].steps[i] = pos[l].steps[maxj];
    pos[l].steps[maxj] = buf;
  }
}

// [수정됨] 통과한 폰 로직 적용 평가 함수
int evaluate(int l) {
  int score_mat = 0;
  if (pos[l].w) score_mat = pos[l].weight_w - pos[l].weight_b;
  else score_mat = pos[l].weight_b - pos[l].weight_w;
  int score_pos = 0;
  if (stats) {
    if (pos[l].w) score_pos = pos[l].weight_s;
    else score_pos = -pos[l].weight_s;
  }
  int bonus_passed = 0;
  const int pp_score[9] = { 0, 0, 5, 10, 20, 40, 80, 150, 0 };
  int w_passed = 0;
  int b_passed = 0;
  for (int i = 0; i < 64; i++) {
    int p = pole[i];
    if (abs(p) != fp) continue;
    int r = row[i];
    int c = column[i];
    boolean is_passed = true;
    boolean is_isolated = true;
    boolean is_blocked = false;
    if (p == fp) {
      if (r < 7) {
        for (int step = 1; step <= (7 - r); step++) {
          int t_idx = i - (8 * step);
          if (pole[t_idx] == -fp) {
            is_passed = false;
            break;
          }
          if (c > 1 && pole[t_idx - 1] == -fp) {
            is_passed = false;
            break;
          }
          if (c < 8 && pole[t_idx + 1] == -fp) {
            is_passed = false;
            break;
          }
        }
      }
      if (is_passed) {
        int current_score = pp_score[r];
        if (c > 1)
          if (pole[i - 1] == fp || pole[i + 7] == fp || pole[i + 15] == fp) is_isolated = false;
        if (c < 8)
          if (pole[i + 1] == fp || pole[i + 9] == fp || pole[i + 17] == fp) is_isolated = false;
        if (r < 8 && pole[i - 8] < 0) is_blocked = true;
        if (is_isolated) current_score /= 2;
        else current_score = (current_score * 3) / 2;
        if (is_blocked) current_score /= 2;
        w_passed += current_score;
      }
    } else {
      if (r > 2) {
        for (int step = 1; step <= (r - 2); step++) {
          int t_idx = i + (8 * step);
          if (pole[t_idx] == fp) {
            is_passed = false;
            break;
          }
          if (c > 1 && pole[t_idx - 1] == fp) {
            is_passed = false;
            break;
          }
          if (c < 8 && pole[t_idx + 1] == fp) {
            is_passed = false;
            break;
          }
        }
      }
      if (is_passed) {
        int current_score = pp_score[9 - r];
        if (c > 1)
          if (pole[i - 1] == -fp || pole[i - 9] == -fp || pole[i - 17] == -fp) is_isolated = false;
        if (c < 8)
          if (pole[i + 1] == -fp || pole[i - 7] == -fp || pole[i - 15] == -fp) is_isolated = false;
        if (r > 1 && pole[i + 8] > 0) is_blocked = true;
        if (is_isolated) current_score /= 2;
        else current_score = (current_score * 3) / 2;
        if (is_blocked) current_score /= 2;
        b_passed += current_score;
      }
    }
  }
  if (pos[l].w) bonus_passed = w_passed - b_passed;
  else bonus_passed = b_passed - w_passed;
  if (endspiel) bonus_passed = (bonus_passed * 5) / 4;
  if (!stats) return score_mat + bonus_passed;
  else {
    int total_mat = pos[l].weight_w + pos[l].weight_b + 2000;
    int divider = (total_mat < 3000) ? 3000 : total_mat;
    long final_score = (long)(score_mat + score_pos + bonus_passed) * 5000 / divider;
    return (int)final_score;
  }
}

void generate_steps(int l) {
  pos[l].cur_step = 0;
  pos[l].n_steps = 0;
  int check;
  signed char f;
  task_l = l;
  task_start = 2;
  for (int ii = 0; ii < 64; ii++) {
    int i;
    if (pos[l].w) i = ii;
    else i = 63 - ii;
    f = pole[i];
    if (f == 0 || (f < 0 && pos[l].w) || (f > 0 && !pos[l].w)) continue;
    switch (abs(f)) {
      case fn: add_knight(l, i); break;
      case fb: add_diag(l, i); break;
      case fr: add_stra(l, i); break;
      case fq:
        add_stra(l, i);
        add_diag(l, i);
        break;
      case fk:
        if (endspiel) add_king(l, i);
        break;
    }
  }
  while (task_start == 2) { delayMicroseconds(0); }
  if (pos[l].w == 1 && !pos[l].check_on_table) {
    if (pos[l].wrk)
      if (pole[60] == fk && pole[61] == 0 && pole[62] == 0 && pole[63] == fr) {
        pole[60] = 0;
        pole[61] = fk;
        poswk = 61;
        check = check_w();
        pole[60] = fk;
        poswk = 60;
        pole[61] = 0;
        if (!check) {
          pos[l].steps[pos[l].n_steps].type = 2;
          pos[l].steps[pos[l].n_steps].c1 = 60;
          pos[l].steps[pos[l].n_steps].c2 = 62;
          pos[l].steps[pos[l].n_steps].f1 = fk;
          pos[l].steps[pos[l].n_steps].f2 = 0;
          pos[l].n_steps++;
        }
      }
    if (pos[l].wrq)
      if (pole[60] == fk && pole[59] == 0 && pole[58] == 0 && pole[57] == 0 && pole[56] == fr) {
        pole[60] = 0;
        pole[59] = fk;
        poswk = 59;
        check = check_w();
        pole[60] = fk;
        poswk = 60;
        pole[59] = 0;
        if (!check) {
          pos[l].steps[pos[l].n_steps].type = 3;
          pos[l].steps[pos[l].n_steps].c1 = 60;
          pos[l].steps[pos[l].n_steps].c2 = 58;
          pos[l].steps[pos[l].n_steps].f1 = fk;
          pos[l].steps[pos[l].n_steps].f2 = 0;
          pos[l].n_steps++;
        }
      }
  } else if (pos[l].w == 0 && !pos[l].check_on_table) {
    if (pos[l].brk)
      if (pole[4] == -fk && pole[5] == 0 && pole[6] == 0 && pole[7] == -fr) {
        pole[4] = 0;
        pole[5] = -fk;
        posbk = 5;
        check = check_b();
        pole[4] = -fk;
        posbk = 4;
        pole[5] = 0;
        if (!check) {
          pos[l].steps[pos[l].n_steps].type = 2;
          pos[l].steps[pos[l].n_steps].c1 = 4;
          pos[l].steps[pos[l].n_steps].c2 = 6;
          pos[l].steps[pos[l].n_steps].f1 = -fk;
          pos[l].steps[pos[l].n_steps].f2 = 0;
          pos[l].n_steps++;
        }
      }
    if (pos[l].brq)
      if (pole[4] == -fk && pole[3] == 0 && pole[2] == 0 && pole[1] == 0 && pole[0] == -fr) {
        pole[4] = 0;
        pole[3] = -fk;
        posbk = 3;
        check = check_b();
        pole[4] = -fk;
        posbk = 4;
        pole[3] = 0;
        if (!check) {
          pos[l].steps[pos[l].n_steps].type = 3;
          pos[l].steps[pos[l].n_steps].c1 = 4;
          pos[l].steps[pos[l].n_steps].c2 = 2;
          pos[l].steps[pos[l].n_steps].f1 = -fk;
          pos[l].steps[pos[l].n_steps].f2 = 0;
          pos[l].n_steps++;
        }
      }
  }
  for (int i = 0; i < n_steps2; i++) {
    pos[l].steps[pos[l].n_steps] = steps2[i];
    pos[l].n_steps++;
  }
  for (int i = 0; i < pos[l].n_steps; i++) {
    pos[l].steps[i].check = 0;
    pos[l].steps[i].weight = abs(pos[l].steps[i].f2);
    if (pos[l].steps[i].type > 3) pos[l].steps[i].weight += fig_weight[pos[l].steps[i].type - 2];
    pos[l].steps[i].weight <<= 2;
    if (l > 0) {
      if (pos[l].best.c2 == pos[l].steps[i].c2 && pos[l].best.c1 == pos[l].steps[i].c1) pos[l].steps[i].weight += 5;
      if (pos[l].steps[i].c2 == pos[l - 1].steps[pos[l - 1].cur_step].c2) pos[l].steps[i].weight += 8;
    }
  }
  sort_steps(l);
}

void epd() {
  for (int i = 0; i < MAXEPD; i++) {
    bestmove[i].c1 = -1;
    bestmove[i].c2 = -1;
    bestmove[i].type = -1;
  }
  int posbm = fenstr.indexOf("bm");
  if (posbm > -1) {
    int posp = fenstr.indexOf(";", posbm + 2);
    if (posp == -1) posp = fenstr.length();
    String ep = fenstr.substring(posbm + 3, posp);
    kingpositions();
    generate_steps(0);
    posp = ep.indexOf(" ");
    if (posp > -1) {
      getbm(0, ep.substring(0, posp));
      ep = ep.substring(posp + 1);
      posp = ep.indexOf(" ");
      if (posp > -1) {
        getbm(1, ep.substring(0, posp));
        ep = ep.substring(posp + 1);
        posp = ep.indexOf(" ");
        if (posp > -1) {
          getbm(2, ep.substring(0, posp));
          ep = ep.substring(posp + 1);
          posp = ep.indexOf(" ");
          if (posp > -1) {
            getbm(3, ep.substring(0, posp));
            ep = ep.substring(posp + 1);
            posp = ep.indexOf(" ");
            if (posp > -1) getbm(4, ep.substring(0, posp));
            else getbm(4, ep);
          } else getbm(3, ep);
        } else getbm(2, ep);
      } else getbm(1, ep);
    } else getbm(0, ep);
  }
  bestcount = 0;
  for (int i = 0; i < MAXEPD; i++)
    if (bestmove[i].c1 != -1) bestcount++;
}

void taskOne(void *parameter) {
  task_time = millis();
  int l, f;
  unsigned long task_count = 0;
  do {
    if (task_start == 2) {
      l = task_l;
      if (pole[poswk] != fk) {
        for (int i = 0; i < 64; i++)
          if (pole[i] == fk) {
            poswk = i;
            break;
          }
      }
      if (pole[posbk] != -fk) {
        for (int i = 0; i < 64; i++)
          if (pole[i] == -fk) {
            posbk = i;
            break;
          }
      }
      if (l > 0) {
        if (pos[l - 1].steps[pos[l - 1].cur_step].check == 0) {
          if (pos[l].w) pos[l].check_on_table = check_w();
          else pos[l].check_on_table = check_b();
          pos[l - 1].steps[pos[l - 1].cur_step].check = pos[l].check_on_table;
        } else pos[l].check_on_table = 1;
      } else if (pos[0].w) pos[0].check_on_table = check_w();
      else pos[0].check_on_table = check_b();
      n_steps2 = 0;
      for (int ii = 0; ii < 64; ii++) {
        int i;
        if (pos[l].w) i = ii;
        else i = 63 - ii;
        f = pole[i];
        if (f == 0 || (f < 0 && pos[l].w) || (f > 0 && !pos[l].w)) continue;
        if (f == fp) {
          if (row[i] < 7 && pole[i - 8] == 0) add_one(l, i, i - 8);
          if (row[i] == 2 && pole[i - 8] == 0 && pole[i - 16] == 0) add_one(l, i, i - 16);
          if (row[i] == 7) {
            if (pole[i - 8] == 0) {
              add_one(l, i, i - 8);
              steps2[n_steps2 - 1].type = 4;
              add_one(l, i, i - 8);
              steps2[n_steps2 - 1].type = 5;
              add_one(l, i, i - 8);
              steps2[n_steps2 - 1].type = 6;
              add_one(l, i, i - 8);
              steps2[n_steps2 - 1].type = 7;
            }
            if (column[i] > 1 && pole[i - 9] < 0) {
              add_one(l, i, i - 9);
              steps2[n_steps2 - 1].type = 4;
              add_one(l, i, i - 9);
              steps2[n_steps2 - 1].type = 5;
              add_one(l, i, i - 9);
              steps2[n_steps2 - 1].type = 6;
              add_one(l, i, i - 9);
              steps2[n_steps2 - 1].type = 7;
            }
            if (column[i] < 8 && pole[i - 7] < 0) {
              add_one(l, i, i - 7);
              steps2[n_steps2 - 1].type = 4;
              add_one(l, i, i - 7);
              steps2[n_steps2 - 1].type = 5;
              add_one(l, i, i - 7);
              steps2[n_steps2 - 1].type = 6;
              add_one(l, i, i - 7);
              steps2[n_steps2 - 1].type = 7;
            }
          } else {
            if (column[i] > 1 && pole[i - 9] < 0) add_one(l, i, i - 9);
            if (column[i] < 8 && pole[i - 7] < 0) add_one(l, i, i - 7);
          }
        } else if (f == -fp) {
          if (row[i] > 2 && pole[i + 8] == 0) add_one(l, i, i + 8);
          if (row[i] == 7 && pole[i + 8] == 0 && pole[i + 16] == 0) add_one(l, i, i + 16);
          if (row[i] == 2) {
            if (pole[i + 8] == 0) {
              add_one(l, i, i + 8);
              steps2[n_steps2 - 1].type = 4;
              add_one(l, i, i + 8);
              steps2[n_steps2 - 1].type = 5;
              add_one(l, i, i + 8);
              steps2[n_steps2 - 1].type = 6;
              add_one(l, i, i + 8);
              steps2[n_steps2 - 1].type = 7;
            }
            if (column[i] > 1 && pole[i + 7] > 0) {
              add_one(l, i, i + 7);
              steps2[n_steps2 - 1].type = 4;
              add_one(l, i, i + 7);
              steps2[n_steps2 - 1].type = 5;
              add_one(l, i, i + 7);
              steps2[n_steps2 - 1].type = 6;
              add_one(l, i, i + 7);
              steps2[n_steps2 - 1].type = 7;
            }
            if (column[i] < 8 && pole[i + 9] > 0) {
              add_one(l, i, i + 9);
              steps2[n_steps2 - 1].type = 4;
              add_one(l, i, i + 9);
              steps2[n_steps2 - 1].type = 5;
              add_one(l, i, i + 9);
              steps2[n_steps2 - 1].type = 6;
              add_one(l, i, i + 9);
              steps2[n_steps2 - 1].type = 7;
            }
          } else {
            if (column[i] > 1 && pole[i + 7] > 0) add_one(l, i, i + 7);
            if (column[i] < 8 && pole[i + 9] > 0) add_one(l, i, i + 9);
          }
        } else if (!endspiel && abs(f) == fk) add_king2(l, i);
      }
      if (pos[l].pp != 0 && pole[pos[l].pp] == 0) {
        if (pos[l].w == 1) {
          if (column[pos[l].pp] > 1 && pole[pos[l].pp + 7] == fp) {
            add_one(l, pos[l].pp + 7, pos[l].pp);
            steps2[n_steps2 - 1].type = 1;
            steps2[n_steps2 - 1].f2 = -fp;
          }
          if (column[pos[l].pp] < 8 && pole[pos[l].pp + 9] == fp) {
            add_one(l, pos[l].pp + 9, pos[l].pp);
            steps2[n_steps2 - 1].type = 1;
            steps2[n_steps2 - 1].f2 = -fp;
          }
        } else {
          if (column[pos[l].pp] > 1 && pole[pos[l].pp - 9] == -fp) {
            add_one(l, pos[l].pp - 9, pos[l].pp);
            steps2[n_steps2 - 1].type = 1;
            steps2[n_steps2 - 1].f2 = fp;
          }
          if (column[pos[l].pp] < 8 && pole[pos[l].pp - 7] == -fp) {
            add_one(l, pos[l].pp - 7, pos[l].pp);
            steps2[n_steps2 - 1].type = 1;
            steps2[n_steps2 - 1].f2 = fp;
          }
        }
      }
      task_start = 0;
    }
    task_count++;
    if (task_count > 7000000) {
      task_count = 0;
      delay(1);  // 워치독 양보 (시리얼 읽기는 pollThinkAbort로 일원화)
    }
  } while (1);
}

int draw_repeat(int l) {
  if (l <= 12 || zero) return 0;
  for (int i = 0; i < 4; i++) {
    int li = l - i;
    if (pos[li].steps[pos[li].cur_step].c1 != pos[li - 4].steps[pos[li - 4].cur_step].c1 || pos[li].steps[pos[li].cur_step].c1 != pos[li - 8].steps[pos[li - 8].cur_step].c1 || pos[li].steps[pos[li].cur_step].c2 != pos[li - 4].steps[pos[li - 4].cur_step].c2 || pos[li].steps[pos[li].cur_step].c2 != pos[li - 8].steps[pos[li - 8].cur_step].c2) return 0;
  }
  return 1;
}

int is_repetition(int l) {
  uint64_t current_h = pos[l].hash;
  int count = 0;
  for (int i = 0; i < game_history_count; i++)
    if (game_history_hash[i] == current_h) count++;
  for (int i = 0; i < l; i++)
    if (pos[i].hash == current_h) count++;
  return count;
}

int active(step_t &s) {
  int j;
  if (s.f2 != 0 || s.type > 3) return 1;
  if (abs(s.f2) == fk) return -1;
  switch (s.f1) {
    case fp:
      if (row[s.c2] > 5) return 1;
      if ((column[s.c2] > 1 && posbk == s.c2 - 9) || (column[s.c2] < 8 && posbk == s.c2 - 7)) return 1;
      return -1;
    case -fp:
      if (row[s.c2] < 4) return 1;
      if ((column[s.c2] > 1 && poswk == s.c2 + 7) || (column[s.c2] < 8 && posbk == s.c2 + 9)) return 1;
      return -1;
    case fn:
      j = 0;
      while (knight_step[s.c2][j] != 99) {
        if (pole[knight_step[s.c2][j]] == -fk) return 1;
        j++;
      }
      return 0;
    case -fn:
      j = 0;
      while (knight_step[s.c2][j] != 99) {
        if (pole[knight_step[s.c2][j]] == fk) return 1;
        j++;
      }
      return 0;
    case fb:
      if (diag1[s.c2] != diag1[posbk] && diag2[s.c2] != diag2[posbk]) return -1;
      return 0;
    case -fb:
      if (diag1[s.c2] != diag1[poswk] && diag2[s.c2] != diag2[poswk]) return -1;
      return 0;
    case fr:
      if (row[s.c2] != row[posbk] && column[s.c2] != column[posbk]) return -1;
      return 0;
    case -fr:
      if (row[s.c2] != row[poswk] && column[s.c2] != column[poswk]) return -1;
      return 0;
    case fq:
      if (diag1[s.c2] != diag1[posbk] && diag2[s.c2] != diag2[posbk] && row[s.c2] != row[posbk] && column[s.c2] != column[posbk]) return -1;
      return 0;
    case -fq:
      if (diag1[s.c2] != diag1[poswk] && diag2[s.c2] != diag2[poswk] && row[s.c2] != row[poswk] && column[s.c2] != column[poswk]) return -1;
      return 0;
  }
  return 0;
}

int quiescence(int l, int alpha, int beta, int depthleft) {
  yield();
  clockUpdate();  // [시계] 탐색 노드마다 갱신하여 규칙적으로 감소
  if (depthleft <= 0) {
    if (l > depth) depth = l;
    return evaluate(l);
  }
  int score = -20000;
  generate_steps(l);
  if (!pos[l].check_on_table) {
    int weight = evaluate(l);
    if (weight >= score) score = weight;
    if (score > alpha) alpha = score;
    if (alpha >= beta) return alpha;
  }
  int check, checked, act;
  for (int i = 0; i < pos[l].n_steps; i++) {
    act = 1;
    if (!pos[l].check_on_table) {
      act = active(pos[l].steps[i]);
      if (act == -1) continue;
    }
    movestep(l, pos[l].steps[i]);
    check = 0;
    if (act == 0) {
      if (pos[l].w) check = checkd_b();
      else check = checkd_w();
      pos[l].steps[i].check = check;
      if (!check) {
        backstep(l, pos[l].steps[i]);
        continue;
      }
    }
    if (pos[l].w) checked = check_w();
    else checked = check_b();
    if (checked) {
      backstep(l, pos[l].steps[i]);
      continue;
    }
    if (check && depthleft == 1 && l < MAXDEPTH - 1) depthleft++;
    pos[l].cur_step = i;
    movepos(l, pos[l].steps[i]);
    int tmp = -quiescence(l + 1, -beta, -alpha, depthleft - 1);
    backstep(l, pos[l].steps[i]);
    if (draw_repeat(l)) tmp = 0;
    if (tmp > score) score = tmp;
    if (score > alpha) {
      alpha = score;
      pos[l].best = pos[l].steps[i];
    }
    if (alpha >= beta) return alpha;
  }
  if (score == -20000) {
    if (pos[l].check_on_table) {
      score = -10000 + l;
      pos[l - 1].steps[pos[l - 1].cur_step].check = 2;
    }
  }
  return score;
}

void pollThinkAbort() {
  clockUpdate();  // AI 생각 중에도 시계가 줄어들도록 매 폴링마다 갱신

  // 기권 버튼 스캔 (20ms 간격)
  static unsigned long lastBtnScan = 0;
  static bool prevResignBtn = false;
  unsigned long now = millis();
  if (now - lastBtnScan >= 20) {
    lastBtnScan = now;
    scanMatrix();
    debounceMatrix();
    bool cur = debouncedState[BTN_RESIGN_R][BTN_RESIGN_C];
    if (cur && !prevResignBtn) {
      if (!g_resignPending) {
        g_resignPending = true;
        g_resignBlinkOn = true;
        g_resignBlinkLast = now;
        strip.setPixelColor(LED_RESIGN, strip.Color(255, 0, 0));
        strip.show();
      } else {
        g_resignPending = false;
        g_resignBlinkOn = false;
        strip.setPixelColor(LED_RESIGN, 0);
        strip.show();
        g_abortCmd = "resign";
        g_abortThink = true;
        halt = 1;
      }
    }
    prevResignBtn = cur;
  }

  // 기권 LED 깜빡임
  if (g_resignPending && now - g_resignBlinkLast >= 300) {
    g_resignBlinkOn = !g_resignBlinkOn;
    strip.setPixelColor(LED_RESIGN, g_resignBlinkOn ? strip.Color(255, 0, 0) : 0);
    strip.show();
    g_resignBlinkLast = now;
  }

  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    s.toLowerCase();
    if (s == "resign" || s == "exit" || s == "quit" || s == "stop") {
      g_abortCmd = s;
      g_abortThink = true;
      halt = 1;  // alphaBeta가 halt 체크로 즉시 탈출
    }
    // 그 외 입력(좌표 등)은 AI 턴이므로 무시
  }
}

boolean print_best(int dep) {
  pollThinkAbort();  // [추가] AI 생각 중 입력 체크
  if (halt || millis() - starttime > timelimith) return false;
  if (lastbestdepth == dep && pos[0].best.type == lastbeststep.type && pos[0].best.c1 == lastbeststep.c1 && pos[0].best.c2 == lastbeststep.c2) return false;
  boolean ret = false;
  if (pos[0].best.type == lastbeststep.type && pos[0].best.c1 == lastbeststep.c1 && pos[0].best.c2 == lastbeststep.c2) {
    for (int i = 0; i < MAXEPD; i++) {
      if (lastbeststep.c2 == bestmove[i].c2 && lastbeststep.c1 == bestmove[i].c1 && lastbeststep.type == bestmove[i].type) {
        ret = true;
        bestsolved = 1;
        break;
      }
    }
  }
  lastbestdepth = dep;
  lastbeststep = pos[0].best;
  if (pos[0].w == 0) Serial.print("1...");
  else Serial.print("1.");
  String st = str_step(pos[0].best);
  Serial.print(st);
  for (int i = 0; i < 10 - st.length(); i++) Serial.print(" ");
  String depf = "/" + String(depth + 1) + " ";
  long tim = (millis() - starttime) / 1000;
  String wei = String(pos[0].best.weight / 100., 2);
  if (pos[0].best.weight > 9000) wei = "+M" + String((10001 - pos[0].best.weight) / 2);
  Serial.println("(" + wei + ") Depth: " + String(dep + depf) + get_time(tim) + " " + String(count / 1000) + "kN");
  return ret;
}

int alphaBeta(int l, int alpha, int beta, int depthleft) {
  yield();
  clockUpdate();  // [시계] 탐색 노드마다 갱신하여 규칙적으로 감소
  if (l > 0 && is_repetition(l) >= 2) return 0;
  if (pos[l].halfmove_clock >= 100) return 0;
  int score = -20000, check, ext, tmp;
  if (depthleft <= 0) {
    int fd = fdepth;
    if (pos[l - 1].steps[pos[l - 1].cur_step].f2 != 0) fd += 2;
    return quiescence(l, alpha, beta, fd);
  }
  if (l > 0) generate_steps(l);
  if (l >= nulldepth && zero == 0 && depthleft > 2)
    if (!pos[l].check_on_table && pos[l - 1].steps[pos[l - 1].cur_step].f2 == 0) {
      zero = 1;
      pos[l + 1].wrk = pos[l].wrk;
      pos[l + 1].wrq = pos[l].wrq;
      pos[l + 1].brk = pos[l].brk;
      pos[l + 1].brq = pos[l].brq;
      pos[l + 1].weight_w = pos[l].weight_w;
      pos[l + 1].weight_b = pos[l].weight_b;
      pos[l + 1].weight_s = pos[l].weight_s;
      pos[l + 1].pp = 0;
      pos[l].cur_step = MAXSTEPS;
      pos[l].steps[MAXSTEPS].f2 = 0;
      int tmpz = -alphaBeta(l + 1, -beta, -beta + 1, depthleft - 3);
      zero = 0;
      if (tmpz >= beta) return beta;
    }
  if (l > 4 && !zero && depthleft <= 2 && futility && !pos[l].check_on_table && pos[l - 1].steps[pos[l - 1].cur_step].f2 == 0) {
    int weight = evaluate(l);
    if (weight - 200 >= beta) return beta;
  }
  for (int i = 0; i < pos[l].n_steps; i++) {
    ext = 0;
    if (l <= 1) {          // l=0,1에서 폴링 → 서브트리가 길어도 20ms마다 버튼 감지
      pollThinkAbort();
      if (halt) return score;
    }
    if (l == 0) {
      depth = depthleft;
      if (level < 7)
        if (pos[0].steps[pos[0].cur_step].check) ext = 2;
    }
    movestep(l, pos[l].steps[i]);
    if (pos[l].w) check = check_w();
    else check = check_b();
    if (check) {
      backstep(l, pos[l].steps[i]);
      continue;
    }
    pos[l].cur_step = i;
    movepos(l, pos[l].steps[i]);
    if (l > 2 && !lazy && !zero && lazyeval && !pos[l].steps[i].f2 && !pos[0].steps[pos[0].cur_step].check && evaluate(l + 1) + 100 <= alpha && ((pos[l].w && !check_b()) || (!pos[l].w && !check_w()))) {
      lazy = 1;
      if (-alphaBeta(l + 1, -beta, -alpha, depthleft - 3) <= alpha) tmp = alpha;
      else {
        lazy = 0;
        tmp = -alphaBeta(l + 1, -beta, -alpha, depthleft - 1 + ext);
      }
      lazy = 0;
    } else tmp = -alphaBeta(l + 1, -beta, -alpha, depthleft - 1 + ext);
    backstep(l, pos[l].steps[i]);
    if (draw_repeat(l)) tmp = 0;
    if (tmp > score) score = tmp;
    pos[l].steps[i].weight = tmp;
    if (score > alpha) {
      alpha = score;
      pos[l].best = pos[l].steps[i];
      if (l == 0 && level > 3)
        if (print_best(depthleft)) return alpha;
    }
    if (alpha >= beta) return alpha;
    if (halt || (l < 3 && millis() - starttime > timelimith)) return score;
  }
  if (score == -20000) {
    if (pos[l].check_on_table) {
      score = -10000 + l;
      pos[l - 1].steps[pos[l - 1].cur_step].check = 2;
    } else score = 0;
  }
  return score;
}

boolean is_draw() {
  int cn = 0, cbw = 0, cbb = 0, co = 0, cb = 0, cw = 0;
  for (int i = 0; i < 64; i++) {
    if (abs(pole[i]) == 1) co++;
    if (abs(pole[i]) > 3 && abs(pole[i]) < 6) co++;
    if (abs(pole[i]) == 6) continue;
    if (abs(pole[i]) == 2) cn++;
    if (abs(pole[i]) == 3 && (column[i] + row[i]) % 2 == 0) cbb++;
    if (abs(pole[i]) == 3 && (column[i] + row[i]) % 2 == 1) cbw++;
    if (pole[i] == 3) cw++;
    if (pole[i] == -3) cb++;
  }
  if (cn == 1 && co + cbb + cbw == 0) return true;
  if (cbb + cbw == 1 && co + cn == 0) return true;
  if (co + cn + cbb == 0 || co + cn + cbw == 0) return true;
  if (co + cn == 0 && cb == 1 && cw == 1) return true;
  return false;
}

int solve_step() {
  pollThinkAbort();            // [추가] 책수로 즉시 끝나기 전에 입력 체크
  if (g_abortThink) return 1;  // [추가] 중단 요청이면 탐색 없이 반환

  String currentFen = fenout(0);
  String bookMove = readMoveFromFlash(currentFen);
  // 대국 중 실시간 Lichess 조회 제거 — 학습으로 모은 북만 사용
  if (bookMove != "") {
    int u1 = strToPos(bookMove.substring(0, 2));
    int u2 = strToPos(bookMove.substring(2, 4));
    int promoType = 0;
    if (bookMove.length() == 5) {
      char p = bookMove.charAt(4);
      if (p == 'n') promoType = 4;
      else if (p == 'b') promoType = 5;
      else if (p == 'r') promoType = 6;
      else promoType = 7;
    }
    generate_steps(0);
    for (int i = 0; i < pos[0].n_steps; i++) {
      if (pos[0].steps[i].c1 == u1 && pos[0].steps[i].c2 == u2) {
        if (promoType == 0 || pos[0].steps[i].type == promoType) {
          pos[0].best = pos[0].steps[i];
          Serial.println("Book: " + bookMove);
          return 1;
        }
      }
    }
  }
  int score;
  count = 0;
  zero = 0;
  lazy = 0;
  for (int i = 1; i < MAXDEPTH; i++) {
    if (i % 2) pos[i].w = !pos[0].w;
    else pos[i].w = pos[0].w;
    pos[i].pp = 0;
  }
  starttime = millis();
  if (is_draw()) {
    Serial.println(" DRAW!");
    return 1;
  }
  lastbestdepth = -1;
  lastbeststep.c1 = -1;
  lastbeststep.c2 = -1;
  bestsolved = 0;
  pos[0].weight_b = 0;
  pos[0].weight_w = 0;
  for (int i = 0; i < 64; i++) {
    if (pole[i] < 0) pos[0].weight_b += fig_weight[-pole[i]];
    else if (pole[i] > 0) pos[0].weight_w += fig_weight[pole[i]];
  }
  if (pos[0].weight_w + pos[0].weight_b < 3500) endspiel = true;
  else endspiel = false;
  pos[0].weight_s = 0;
  for (int i = 0; i < 64; i++) {
    int f = pole[i];
    if (!f) continue;
    if (abs(f) == fk && endspiel) {
      if (f < 0) pos[0].weight_s -= stat_weightb[6][i];
      else pos[0].weight_s += stat_weightw[6][i];
    } else {
      if (f < 0) pos[0].weight_s -= stat_weightb[-f - 1][i];
      else pos[0].weight_s += stat_weightw[f - 1][i];
    }
  }
  kingpositions();
  generate_steps(0);
  int legal = 0;
  int check;
  for (int i = 0; i < pos[0].n_steps; i++) {
    movestep(0, pos[0].steps[i]);
    if (pos[0].w) check = check_w();
    else check = check_b();
    pos[0].check_on_table = check;
    if (!check) legal++;
    if (!check) pos[0].steps[i].weight = 0;
    else pos[0].steps[i].weight = -30000;
    backstep(0, pos[0].steps[i]);
  }
  if (legal == 0) {
    if (pos[0].check_on_table) {
      Serial.println(" CHECKMATE!");
      return 1;
    } else {
      Serial.println(" PAT!");
      return 1;
    }
  }
  sort_steps(0);
  pos[0].n_steps = legal;
  int ALPHA = -20000;
  int BETA = 20000;
  level = 2;
  if (timelimith > 300000) level = 4;
  for (int x = 0; x < MAXDEPTH; x++) {
    pos[x].best.f1 = 0;
    pos[x].best.c2 = -1;
  }
  stats = 1;
  step_t safe_best_move = pos[0].steps[0];
  while (level <= 20) {
    for (int x = 1; x < MAXDEPTH; x++) {
      pos[x].best.f1 = 0;
      pos[x].best.c2 = -1;
    }
    for (int i = 0; i < pos[0].n_steps; i++) {
      movestep(0, pos[0].steps[i]);
      if (pos[0].w) pos[0].steps[i].check = check_b();
      else pos[0].steps[i].check = check_w();
      pos[0].steps[i].weight += evaluate(0) + pos[0].steps[i].check * 500;
      if (pos[0].steps[i].f2 != 0) pos[0].steps[i].weight -= pos[0].steps[i].f1;
      backstep(0, pos[0].steps[i]);
    }
    pos[0].steps[0].weight += 10000;
    sort_steps(0);
    for (int i = 0; i < pos[0].n_steps; i++) pos[0].steps[i].weight = -8000;
    if (nullmove) nulldepth = 3;
    else nulldepth = 93;
    fdepth = 4;
    score = alphaBeta(0, ALPHA, BETA, level);
    unsigned long tim = millis() - starttime;
    if (score >= BETA) {
      ALPHA = -20000;
      BETA = 20000;
    } else {
      ALPHA = score - 100;
      BETA = score + 100;
    }
    sort_steps(0);
    if (score > 9900) return 1;  // 메이트 발견: 시간 초과와 무관하게 즉시 사용
    if (halt || tim > timelimith) {
      pos[0].best = safe_best_move;
      break;
    }
    safe_best_move = pos[0].best;
    if (print_best(level) || bestsolved) return 1;
    level++;
  }
  if (pos[0].best.f1 == 0 && legal > 0) pos[0].best = safe_best_move;
  return 1;
}

int countLegalMoves() {
  int legalCount = 0;
  generate_steps(0);
  int tempSteps = pos[0].n_steps;
  step_t buf[MAXSTEPS];
  for (int i = 0; i < tempSteps; i++) buf[i] = pos[0].steps[i];
  for (int i = 0; i < tempSteps; i++) {
    movestep(0, buf[i]);
    boolean safe = (pos[0].w == 1) ? !check_w() : !check_b();
    if (safe) legalCount++;
    backstep(0, buf[i]);
  }
  return legalCount;
}

void init_zobrist() {
  randomSeed(micros());
  for (int i = 0; i < 13; i++)
    for (int j = 0; j < 64; j++) zobrist_table[i][j] = ((uint64_t)random(0xFFFF) << 48) | ((uint64_t)random(0xFFFF) << 32) | ((uint64_t)random(0xFFFF) << 16) | (uint64_t)random(0xFFFF);
  zobrist_side = ((uint64_t)random(0xFFFF) << 48) | ((uint64_t)random(0xFFFF) << 32) | ((uint64_t)random(0xFFFF) << 16) | (uint64_t)random(0xFFFF);
  for (int i = 0; i < 16; i++) zobrist_castling[i] = ((uint64_t)random(0xFFFF) << 48) | ((uint64_t)random(0xFFFF) << 32) | ((uint64_t)random(0xFFFF) << 16) | (uint64_t)random(0xFFFF);
  for (int i = 0; i < 8; i++) zobrist_ep[i] = ((uint64_t)random(0xFFFF) << 48) | ((uint64_t)random(0xFFFF) << 32) | ((uint64_t)random(0xFFFF) << 16) | (uint64_t)random(0xFFFF);
}

uint64_t calculate_full_hash(int l) {
  uint64_t h = 0;
  for (int i = 0; i < 64; i++) {
    int p = pole[i];
    if (p != 0) {
      int p_idx = (p > 0) ? (p - 1) : (abs(p) + 5);
      h ^= zobrist_table[p_idx][i];
    }
  }
  if (pos[l].w == 0) h ^= zobrist_side;
  int c_idx = (pos[l].wrk << 3) | (pos[l].wrq << 2) | (pos[l].brk << 1) | pos[l].brq;
  h ^= zobrist_castling[c_idx];
  if (pos[l].pp != 0) h ^= zobrist_ep[pos[l].pp % 8];
  return h;
}

// [추가] 자율 학습 관련 (링 호환성 포함)
boolean applyInternalMove(String uciMove) {
  if (uciMove == "e1h1") uciMove = "e1g1";
  else if (uciMove == "e1a1") uciMove = "e1c1";
  else if (uciMove == "e8h8") uciMove = "e8g8";
  else if (uciMove == "e8a8") uciMove = "e8c8";

  int u1 = strToPos(uciMove.substring(0, 2));
  int u2 = strToPos(uciMove.substring(2, 4));
  int promoType = 0;
  if (uciMove.length() == 5) {
    char p = uciMove.charAt(4);
    if (p == 'n') promoType = 4;
    else if (p == 'b') promoType = 5;
    else if (p == 'r') promoType = 6;
    else promoType = 7;
  }

  generate_steps(0);
  for (int i = 0; i < pos[0].n_steps; i++) {
    if (pos[0].steps[i].c1 == u1 && pos[0].steps[i].c2 == u2) {
      if (promoType > 0 && pos[0].steps[i].type != promoType) continue;
      step_t move = pos[0].steps[i];
      movestep(0, move);
      movepos(0, move);
      pos[1].w = !pos[0].w;
      pos[0] = pos[1];
      return true;
    }
  }
  return false;
}

void startSelfStudy(int minutes) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi Disconnected! Reconnecting...");
    scanAndConnectWifi();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] Reconnect failed. Aborting self study.");
      return;
    }
  }
  Serial.println("=== SELF STUDY START ===");
  unsigned long endTime = millis() + (minutes * 60 * 1000UL);
  int gamesCount = 0;
  const String forcedOpenings[] = { "e2e4", "d2d4", "c2c4", "g1f3" };
  const int openingCount = 4;

  while (millis() < endTime) {
    handleRingNetwork();
    fen("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
    int depth = 0;
    String forcedMove = "";
    int mode = gamesCount % (openingCount + 1);
    if (mode < openingCount) forcedMove = forcedOpenings[mode];
    Serial.print("Game ");
    Serial.print(gamesCount + 1);
    if (forcedMove != "") { Serial.print(" [Force: " + forcedMove + "]"); }
    Serial.print(": ");

    while (depth < 12) {
      handleRingNetwork();
      String currentFen = fenout(0);
      String uciMove = "";
      if (depth == 0 && forcedMove != "") {
        uciMove = forcedMove;
        saveMoveToFlash(currentFen, uciMove);  // 항상 호출, 중복은 함수가 거름
        Serial.print("!");
      } else {
        uciMove = getLichessMove(currentFen);
        if (uciMove == "") {
          Serial.print("X");
          break;
        }
        saveMoveToFlash(currentFen, uciMove);  // 항상 호출, 중복은 함수가 거름
        Serial.print("!");
      }
      if (!applyInternalMove(uciMove)) {
        Serial.print("Err");
        break;
      }
      depth++;
      delay(50);
    }
    Serial.println();
    gamesCount++;
  }
  Serial.println("Study Finished.");
  fen("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
  forceFullSync();
}

void replayPGN(String pgn) {
  Serial.println(F("\n=== PGN REPLAY START ==="));

  // 원본과 완벽히 동일한 50ms 간격의 초기 호밍 시퀀스 실행
  executeInitialSequence();
  

  // 초기 보드 세팅 및 물리 보드 강제 동기화
  fen("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
  forceFullSync();

  // 첫 수 시작 전 대기 (2.5초)
  unsigned long initialWait = millis();
  while (millis() - initialWait < 2500) {
    handleRingNetwork();
    delay(10);
  }

  // [수정됨] PGN 재생 중 킹 상태 복구를 위한 추적 변수 추가
  bool wasWhiteCheck = false;
  bool wasBlackCheck = false;

  // 줄바꿈 문자를 공백으로 치환하여 평탄화
  pgn.replace("\n", " ");
  pgn.replace("\r", " ");

  int startPos = 0;
  while (startPos < pgn.length()) {
    int endPos = pgn.indexOf(' ', startPos);
    String token;

    if (endPos == -1) {
      token = pgn.substring(startPos);
      startPos = pgn.length();
    } else {
      token = pgn.substring(startPos, endPos);
      startPos = endPos + 1;
    }

    token.trim();
    if (token.length() == 0) continue;

    // 1. 수 번호(1., 2...) 필터링
    if (token.indexOf('.') > -1) continue;

    // 2. 게임 결과 토큰 처리 (기권 및 합의 무승부)
    if (token == "1-0") {
      Serial.println(F("Result: White Wins (1-0/Resign)"));
      sendKingStatus(true, ID_W_WIN);      // 백 승리
      sendKingStatus(false, ID_B_RESIGN);  // 흑 기권 패배
      break;
    }
    if (token == "0-1") {
      Serial.println(F("Result: Black Wins (0-1/Resign)"));
      sendKingStatus(false, ID_B_WIN);    // 흑 승리
      sendKingStatus(true, ID_W_RESIGN);  // 백 기권 패배
      break;
    }
    if (token == "1/2-1/2") {
      Serial.println(F("Result: Draw (1/2-1/2)"));
      sendKingStatus(true, ID_W_DRAW);   // 백 무승부
      sendKingStatus(false, ID_B_DRAW);  // 흑 무승부
      break;
    }
    if (token == "*") break;

    // 3. 기보 토큰 전처리 (캐슬링 기호 대응)
    token.replace("O-O-O", "0-0-0");
    token.replace("O-O", "0-0");

    Serial.print(F("Playing: "));
    Serial.println(token);

    generate_steps(0);
    bestmove[0].c1 = -1;

    // SAN 파서로 수 해석
    getbm(0, token);

    if (bestmove[0].c1 != -1) {
      step_t move = bestmove[0];

      // 엔진 내부 보드 업데이트
      movestep(0, move);
      movepos(0, move);
      pos[1].w = !pos[0].w;
      pos[0] = pos[1];

      // 물리 보드 이동 명령 전송 (시점 대칭 자동 적용)
      updateBoardDiff();

      // ==============================================================
      // [수정됨] 이전 턴 플레이어의 체크 해소까지 완벽하게 감지하는 로직
      // ==============================================================
      bool currentWhiteCheck = check_w();
      bool currentBlackCheck = check_b();

      // 백킹 체크 상태 갱신 및 해소 처리
      if (currentWhiteCheck && !wasWhiteCheck) {
        sendKingStatus(true, ID_W_CHECK);
        wasWhiteCheck = true;
      } else if (!currentWhiteCheck && wasWhiteCheck) {
        sendKingStatus(true, ID_W_KING);
        wasWhiteCheck = false;
      }

      // 흑킹 체크 상태 갱신 및 해소 처리
      if (currentBlackCheck && !wasBlackCheck) {
        sendKingStatus(false, ID_B_CHECK);
        wasBlackCheck = true;
      } else if (!currentBlackCheck && wasBlackCheck) {
        sendKingStatus(false, ID_B_KING);
        wasBlackCheck = false;
      }

      // 게임 종료 상태 판정을 위한 현재 턴 확인
      boolean isWhiteTurn = (pos[0].w == 1);
      bool isCheck = isWhiteTurn ? currentWhiteCheck : currentBlackCheck;
      int legalMoves = countLegalMoves();

      if (legalMoves == 0) {
        if (isCheck) {
          if (isWhiteTurn) {
            Serial.println(F("CHECKMATE! Black Wins."));
            sendKingStatus(true, ID_W_MATE);
            sendKingStatus(false, ID_B_WIN);
          } else {
            Serial.println(F("CHECKMATE! White Wins."));
            sendKingStatus(false, ID_B_MATE);
            sendKingStatus(true, ID_W_WIN);
          }
        } else {
          Serial.println(F("STALEMATE! DRAW."));
          sendKingStatus(true, ID_W_DRAW);
          sendKingStatus(false, ID_B_DRAW);
        }
        break;
      }

      // 수 간격 유지: 2.5초 (하트비트 유지 루프)
      unsigned long waitStart = millis();
      while (millis() - waitStart < 2500) {
        handleRingNetwork();
        delay(10);
      }
    } else {
      Serial.print(F("Failed to parse PGN move: "));
      Serial.println(token);
      break;
    }
  }
  Serial.println(F("=== PGN REPLAY FINISHED ==="));
}

// 부팅 시 1회: 전 노드 기동 확인(TOKAWAKE) → 리부트+호밍 시퀀스
void bootInitOnce() {
  if (g_bootInitDone) return;

  const uint8_t ALL_MASK = (uint8_t)((1 << TOTAL_SLAVES) - 1);

  // 1) 모든 노드가 깰 때까지 계속 확인 (성공할 때까지 반복)
  Serial.println(F("[BOOT] Waiting for all nodes (TOKAWAKE)..."));
  while (!waitSimpleToken("TOKAWAKE", ALL_MASK, 3000)) {
    Serial.printf("[BOOT] awake=0x%02X missing=0x%02X retry...\n",
                  g_gotMask, (uint8_t)(ALL_MASK & ~g_gotMask));
    delay(200);
  }
  Serial.println(F("[BOOT] All nodes awake."));

  // 2) 리부트 + 호밍 시퀀스 (내부에서 TOKHOME 대기)
  executeInitialSequence();

  VFD_print("SELECT MODE");

  g_bootInitDone = true;  // 이후 다시 안 함
}

// =============================================================
// 슬레이브 완료 보고(REBOOT_COMPLETE) 기반의 초기 호밍 시퀀스
//  - 호밍 중에는 TOKHOME 재발행 간격을 길게(reissueMs) 잡아
//    8축 풀회전과 토큰 릴레이가 겹치는 빈도를 낮춘다.
// =============================================================

// TOKHOME / TOKAWAKE 공용(재발행 간격을 인자로 받는 버전)
bool waitSimpleTokenSlow(const char *kind, uint8_t expectedMask,
                         unsigned long timeoutMs, unsigned long reissueMs) {
  g_tokenSeq++;
  g_gotMask = 0;
  RING_SERIAL.println(String(kind) + ":" + String(g_tokenSeq) + ":0");
  unsigned long tokenSent = millis(), startAll = millis();
  int retry = 0;

  while (true) {
    handleRingNetwork();
    if ((g_gotMask & expectedMask) == expectedMask) return true;

    if (millis() - tokenSent > reissueMs) {
      if (++retry > TOKEN_MAX_RETRY || millis() - startAll > timeoutMs) {
        Serial.printf("[WARN] %s timeout. missing=0x%02X\n",
                      kind, (uint8_t)(expectedMask & ~g_gotMask));
        return false;
      }
      g_tokenSeq++;
      g_gotMask = 0;
      RING_SERIAL.println(String(kind) + ":" + String(g_tokenSeq) + ":0");
      tokenSent = millis();
    }
    delay(5);
  }
}

void executeInitialSequence() {
  Serial.println(F("[SYSTEM] Starting Initial Sequence."));
  VFD_print("HOMING...");

  int rebootIndex = 1;
  unsigned long lastSendTime = 0;

  // 50ms 간격 순차 리부트 명령 전송
  while (rebootIndex <= TOTAL_SLAVES) {
    unsigned long now = millis();
    if (now - lastSendTime >= REBOOT_INTERVAL) {
      RING_SERIAL.printf("%d:reboot\n", rebootIndex);
      Serial.printf("[TX] Reboot Node %d\n", rebootIndex);
      lastSendTime = now;
      rebootIndex++;
    }
    handleRingNetwork();
    delay(1);
  }

  // ───────────────────────────────────────────────
  // [핵심] 호밍으로 8축이 풀회전하는 동안에는 토큰을 아예 쏘지 않는다.
  //   호밍 소요 ~2.2~2.5초로 일정하므로, 그 구간엔 토큰 릴레이가
  //   슬레이브에 끼지 않도록 무폴링으로 가만히 대기한다.
  //   (하트비트 등 다른 통신은 handleRingNetwork로 계속 유지)
  // ───────────────────────────────────────────────
  const unsigned long HOMING_QUIET_MS = 2500;  // reboot 다 쏜 시점 기준 무폴링 대기
  Serial.println(F("[SYSTEM] Commands sent. Quiet wait during homing..."));
  unsigned long quietStart = millis();
  while (millis() - quietStart < HOMING_QUIET_MS) {
    handleRingNetwork();  // 토큰은 안 쏘고 수신/하트비트만 유지
    delay(1);
  }

  // 무폴링 대기 후, 막바지(거의 멈춤)에만 TOKHOME 폴링 시작.
  //  - 이 시점엔 모터가 정지했거나 정지 직전이라 토큰이 끼어도 탈조 위험이 작음.
  Serial.println(F("[SYSTEM] Polling homing via TOKHOME..."));
  const uint8_t ALL_MASK = (uint8_t)((1 << TOTAL_SLAVES) - 1);
  const unsigned long HOMING_REISSUE_MS = 1000;  // 이미 멈춘 뒤라 기존 간격 그대로
  const unsigned long HOMING_TIMEOUT_MS = 10000;

  if (waitSimpleTokenSlow("TOKHOME", ALL_MASK, HOMING_TIMEOUT_MS, HOMING_REISSUE_MS)) {
    Serial.println(F("[SYSTEM] All slaves homed successfully."));
  } else {
    g_homingTimeoutCount++;
    Serial.printf("[SYSTEM] WARNING: Homing timeout (TOKHOME). total=%lu\n",
                  g_homingTimeoutCount);
  }
  // VFD는 여기서 끄지 않음 — 호출부에서 다음 화면으로 바로 전환하거나 필요 시 VFD_off() 호출
  // (꺼졌다 켜지는 깜빡임 방지)
}

// [추가] 체크/체크해소 표시를 '수'와 동시에 내보내는 헬퍼
//  - game() 루프 상단의 체크 블록과 완전히 동일한 로직
//  - was*Check 플래그를 갱신하므로 상단 블록이 같은 전환을 중복 발송하지 않음
void updateCheckDisplay(bool &wasWhiteCheck, bool &wasBlackCheck) {
  bool currentWhiteCheck = check_w();
  bool currentBlackCheck = check_b();
  if (currentWhiteCheck && !wasWhiteCheck) {
    sendKingStatus(true, ID_W_CHECK);  wasWhiteCheck = true;
  } else if (!currentWhiteCheck && wasWhiteCheck) {
    sendKingStatus(true, ID_W_KING);   wasWhiteCheck = false;
  }
  if (currentBlackCheck && !wasBlackCheck) {
    sendKingStatus(false, ID_B_CHECK); wasBlackCheck = true;
  } else if (!currentBlackCheck && wasBlackCheck) {
    sendKingStatus(false, ID_B_KING);  wasBlackCheck = false;
  }
}

// =============================================================
// [6] 게임 및 Setup/Loop
// =============================================================

void game(boolean playerIsWhite, bool doInit) {
  g_playerIsWhite = playerIsWhite;

  String s = "";
  boolean gameover = 0;

  if (doInit) {
    executeInitialSequence();   // 내부에서 HOMING... 표시

    fen("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
    kingpositions();

    game_history_count = 0;
    pos[0].hash = calculate_full_hash(0);
    pos[0].halfmove_clock = 0;
    game_history_hash[game_history_count++] = pos[0].hash;

    clocksInit();
    VFD_print("SETTING UP...");
    clocksPower(g_timeControl != TC_UNLIMITED);
    forceFullSync();
  }

  Serial.println(F("\n=== CHESS GAME START ==="));
  Serial.print("FEN_DATA:");
  Serial.println(fenout(0));

  bool wasWhiteCheck = false;
  bool wasBlackCheck = false;

  while (!gameover) {
    handleRingNetwork();
    clockUpdate();  // [시계] 진행 중인 쪽 시간 갱신 및 타임아웃 감지
    boolean isWhiteTurn = (pos[0].w == 1);
    boolean isUserTurn = (isWhiteTurn == playerIsWhite);

    bool currentWhiteCheck = check_w();
    bool currentBlackCheck = check_b();

    if (currentWhiteCheck && !wasWhiteCheck) {
      sendKingStatus(true, ID_W_CHECK);
      wasWhiteCheck = true;
    } else if (!currentWhiteCheck && wasWhiteCheck) {
      sendKingStatus(true, ID_W_KING);
      wasWhiteCheck = false;
    }

    if (currentBlackCheck && !wasBlackCheck) {
      sendKingStatus(false, ID_B_CHECK);
      wasBlackCheck = true;
    } else if (!currentBlackCheck && wasBlackCheck) {
      sendKingStatus(false, ID_B_KING);
      wasBlackCheck = false;
    }

    bool isCheck = isWhiteTurn ? currentWhiteCheck : currentBlackCheck;

    int legalMoves = countLegalMoves();
    if (legalMoves == 0) {
      if (isCheck) {
        if (isWhiteTurn) {
          Serial.println(F("CHECKMATE! Black Wins."));
          sendKingStatus(true, ID_W_MATE);
          sendKingStatus(false, ID_B_WIN);
          VFD_print(playerIsWhite ? "YOU LOSE" : "YOU WIN");   // [VFD]
        } else {
          Serial.println(F("CHECKMATE! White Wins."));
          sendKingStatus(false, ID_B_MATE);
          sendKingStatus(true, ID_W_WIN);
          VFD_print(playerIsWhite ? "YOU WIN" : "YOU LOSE");   // [VFD]
        }
      } else {
        Serial.println(F("STALEMATE! DRAW."));
        sendKingStatus(true, ID_W_DRAW);
        sendKingStatus(false, ID_B_DRAW);
        VFD_print("DRAW");   // [VFD]
      }
      gameover = 1;
      break;
    }

    if (is_repetition(0) >= 3) {
      Serial.println(F("DRAW by repetition!"));
      sendKingStatus(true, ID_W_DRAW);
      sendKingStatus(false, ID_B_DRAW);
      VFD_print("DRAW");
      clockPause();
      gameover = 1;
      break;
    }
    if (pos[0].halfmove_clock >= 100) {
      Serial.println(F("DRAW by 50-move rule!"));
      sendKingStatus(true, ID_W_DRAW);
      sendKingStatus(false, ID_B_DRAW);
      VFD_print("DRAW");
      clockPause();
      gameover = 1;
      break;
    }

    if (is_draw()) {
      Serial.println(F("DRAW by insufficient material!"));
      sendKingStatus(true, ID_W_DRAW);
      sendKingStatus(false, ID_B_DRAW);
      VFD_print("DRAW");
      clockPause();
      gameover = 1;
      break;
    }

    if (isUserTurn) {
      Serial.println(F("\n>>> YOUR TURN!"));
      VFD_print("YOUR TURN");   // [VFD]
      clockResume(isWhiteTurn);  // [시계] 내 턴 시간 진행 시작
      s = "";

      while (Serial.available() > 0) Serial.read();
      clearGameSelection();  // 이전 턴 선택 상태 초기화
      g_resignPending = false;
      g_resignBlinkOn = false;

      // 키 입력 이전 상태 초기화 (이전 턴에서 눌린 키 오인식 방지)
      static bool prevGame[MATRIX_ROWS][MATRIX_COLS];
      scanMatrix(); debounceMatrix();
      memcpy(prevGame, debouncedState, sizeof(prevGame));

      static unsigned long lastBlink = 0;
      static bool blinkOn = true;
      lastBlink = millis(); blinkOn = true;

      while (s == "") {
        handleRingNetwork();
        clockUpdate();  // [시계] 입력 대기 중 시간 갱신/타임아웃 감지
        if (g_timeOver) break;

        // 매트릭스 스캔 & 기물 선택 처리
        scanMatrix();
        debounceMatrix();

        // 기물 선택 깜빡임
        if (g_selSq != -1 && millis() - lastBlink > 300) {
          blinkOn = !blinkOn;
          showGameLEDs(blinkOn);
          if (g_resignPending && g_resignBlinkOn) { strip.setPixelColor(LED_RESIGN, strip.Color(255, 0, 0)); strip.show(); }
          lastBlink = millis();
        }
        // 기권 LED 깜빡임
        if (g_resignPending && millis() - g_resignBlinkLast >= 300) {
          g_resignBlinkOn = !g_resignBlinkOn;
          strip.setPixelColor(LED_RESIGN, g_resignBlinkOn ? strip.Color(255, 0, 0) : 0);
          strip.show();
          g_resignBlinkLast = millis();
        }

        // 키 rising edge 감지
        for (int r = 0; r < MATRIX_ROWS; r++) {
          for (int c = 0; c < MATRIX_COLS; c++) {
            if (debouncedState[r][c] && !prevGame[r][c]) {
              if (r == BTN_RESIGN_R && c == BTN_RESIGN_C) {
                if (!g_resignPending) {
                  g_resignPending = true;
                  g_resignBlinkOn = true;
                  g_resignBlinkLast = millis();
                  strip.setPixelColor(LED_RESIGN, strip.Color(255, 0, 0));
                  strip.show();
                } else {
                  g_resignPending = false;
                  g_resignBlinkOn = false;
                  strip.setPixelColor(LED_RESIGN, 0);
                  strip.show();
                  if (playerIsWhite) {
                    Serial.println(F("\n>>> You (White) Resigned! AI (Black) Wins."));
                    sendKingStatus(true, ID_W_RESIGN);
                    sendKingStatus(false, ID_B_WIN);
                  } else {
                    Serial.println(F("\n>>> You (Black) Resigned! AI (White) Wins."));
                    sendKingStatus(false, ID_B_RESIGN);
                    sendKingStatus(true, ID_W_WIN);
                  }
                  VFD_print("YOU LOSE");
                  gameover = 1;
                  s = "resign";
                }
              } else {
                if (g_resignPending) {
                  g_resignPending = false;
                  g_resignBlinkOn = false;
                  strip.setPixelColor(LED_RESIGN, 0);
                  strip.show();
                }
                int sq = keyToSquare(r, c);
                if (sq >= 0) handleSquarePress(sq, playerIsWhite, s);
              }
            }
            prevGame[r][c] = debouncedState[r][c];
          }
        }
        if (s != "") break;

        if (Serial.available() > 0) {
          s = Serial.readStringUntil('\n');
          s.trim();

          if (s == "exit" || s == "quit") {
            gameover = 1;
            break;
          } else if (s == "resign") {
            if (playerIsWhite) {
              Serial.println(F("\n>>> You (White) Resigned! AI (Black) Wins."));
              sendKingStatus(true, ID_W_RESIGN);
              sendKingStatus(false, ID_B_WIN);
            } else {
              Serial.println(F("\n>>> You (Black) Resigned! AI (White) Wins."));
              sendKingStatus(false, ID_B_RESIGN);
              sendKingStatus(true, ID_W_WIN);
            }
            VFD_print("YOU LOSE");   // [VFD]
            gameover = 1;
            break;
          }
        }
        delay(10);
      }

      if (g_resignPending) {
        g_resignPending = false;
        g_resignBlinkOn = false;
        strip.setPixelColor(LED_RESIGN, 0);
        strip.show();
      }

      if (g_timeOver) {
        clockPause();  // [시계] 타임아웃 시 정지
        if (g_timeOverIsWhite) {
          Serial.println(F("TIME OVER! Black Wins."));
          sendKingStatus(true, ID_W_TIME_OVER);
          sendKingStatus(false, ID_B_WIN);
          VFD_print(playerIsWhite ? "YOU LOSE" : "YOU WIN");   // [VFD]
        } else {
          Serial.println(F("TIME OVER! White Wins."));
          sendKingStatus(false, ID_B_TIME_OVER);
          sendKingStatus(true, ID_W_WIN);
          VFD_print(playerIsWhite ? "YOU WIN" : "YOU LOSE");   // [VFD]
        }
        gameover = 1;
      }
      if (gameover) {
        clockPause();  // [시계] exit/resign 등으로 게임 종료 시 정지
        break;
      }

      if (s.length() >= 4) {
        int u1 = strToPos(s.substring(0, 2));
        int u2 = strToPos(s.substring(2, 4));
        bool handled = false;

        // 프로모션: 폰을 물리적으로 끝 랭크에 이동시킨 뒤 기물 선택
        if (abs(pole[u1]) == fp && (row[u2] == 8 || row[u2] == 1)) {
          handled = true;
          generate_steps(0);
          step_t qStep; bool qFound = false;
          for (int i = 0; i < pos[0].n_steps; i++) {
            if (pos[0].steps[i].c1 == u1 && pos[0].steps[i].c2 == u2 && pos[0].steps[i].type == 7) {
              qStep = pos[0].steps[i]; qFound = true; break;
            }
          }
          if (!qFound) {
            Serial.println(F("Illegal Move!"));
            Serial.print("FEN_DATA:"); Serial.println(fenout(0));
          } else {
            movestep(0, qStep);
            kingpositions();
            if (playerIsWhite ? check_w() : check_b()) {
              backstep(0, qStep);
              Serial.println(F("Illegal: King under attack!"));
              Serial.print("FEN_DATA:"); Serial.println(fenout(0));
            } else {
              // 1) pole[u2]를 폰으로 임시 설정 → 물리 보드에 폰이 끝 랭크까지 이동
              int savedQ = pole[u2];
              pole[u2] = (playerIsWhite ? fp : -fp);
              clockPause();
              VFD_print("MOVING...");
              updateBoardDiff();   // 폰이 u1→u2로 물리 이동 (모터 명령 전송)
              pole[u2] = savedQ;   // backstep을 위해 내부 상태 복원
              waitMoveSettle(8000);

              // 2) 기물 선택 (시계 재개하여 시간 소모)
              clockResume(isWhiteTurn);
              int selectedType = waitPromoChoice();
              clockPause();
              clockAddIncrement(isWhiteTurn);

              // 3) 선택된 타입으로 엔진 상태 확정
              backstep(0, qStep);
              generate_steps(0);
              int finalIdx = -1;
              for (int i = 0; i < pos[0].n_steps; i++) {
                if (pos[0].steps[i].c1 == u1 && pos[0].steps[i].c2 == u2 && pos[0].steps[i].type == selectedType) {
                  finalIdx = i; break;
                }
              }
              if (finalIdx != -1) {
                step_t finalStep = pos[0].steps[finalIdx];
                movestep(0, finalStep);
                movepos(0, finalStep);
                pos[1].w = !pos[0].w;
                pos[0] = pos[1];
                game_history_hash[game_history_count++] = pos[0].hash;
                Serial.print(F("You played: ")); Serial.println(str_step(finalStep));
                Serial.print("FEN_DATA:"); Serial.println(fenout(0));
                VFD_print("MOVING...");
                updateBoardDiff();  // 프로모션 기물 교체 명령 슬레이브에 전송
                updateCheckDisplay(wasWhiteCheck, wasBlackCheck);
                waitMoveSettle(8000);
              }
            }
          }
        }

        // 일반 수 처리
        if (!handled) {
          int moveIdx = -1;
          generate_steps(0);
          for (int i = 0; i < pos[0].n_steps; i++) {
            if (pos[0].steps[i].c1 == u1 && pos[0].steps[i].c2 == u2 && pos[0].steps[i].type <= 3) {
              moveIdx = i; break;
            }
          }
          if (moveIdx != -1) {
            step_t userStep = pos[0].steps[moveIdx];
            movestep(0, userStep);
            kingpositions();
            if (playerIsWhite ? check_w() : check_b()) {
              backstep(0, userStep);
              Serial.println(F("Illegal: King under attack!"));
              Serial.print("FEN_DATA:"); Serial.println(fenout(0));
            } else {
              movepos(0, userStep);
              pos[1].w = !pos[0].w;
              pos[0] = pos[1];
              game_history_hash[game_history_count++] = pos[0].hash;
              Serial.print(F("You played: ")); Serial.println(str_step(userStep));
              Serial.print("FEN_DATA:"); Serial.println(fenout(0));
              clockPause();
              clockAddIncrement(isWhiteTurn);
              VFD_print("MOVING...");
              updateBoardDiff();
              updateCheckDisplay(wasWhiteCheck, wasBlackCheck);
              waitMoveSettle(8000);
            }
          } else {
            Serial.println(F("Illegal Move!"));
            Serial.print("FEN_DATA:"); Serial.println(fenout(0));
          }
        }
      }
    } else {
      Serial.println(F("\n>>> AI IS THINKING..."));
      VFD_print("AI THINKING...");   // [VFD]
      halt = 0;
      g_abortThink = false;
      g_abortCmd = "";
      kingpositions();

      clockResume(isWhiteTurn);  // [시계] AI 생각 시간도 소모 시작

      waitingForMoveComplete = true;

      solve_step();

      if (g_timeOver) {
        clockPause();  // [시계] 타임아웃 -> 정지
        if (g_timeOverIsWhite) {
          Serial.println(F("TIME OVER! Black Wins."));
          sendKingStatus(true, ID_W_TIME_OVER);
          sendKingStatus(false, ID_B_WIN);
          VFD_print(playerIsWhite ? "YOU LOSE" : "YOU WIN");   // [VFD]
        } else {
          Serial.println(F("TIME OVER! White Wins."));
          sendKingStatus(false, ID_B_TIME_OVER);
          sendKingStatus(true, ID_W_WIN);
          VFD_print(playerIsWhite ? "YOU WIN" : "YOU LOSE");   // [VFD]
        }
        waitingForMoveComplete = false;
        pendingCompleteCount = 0;
        gameover = 1;
        break;
      }

      if (g_abortThink) {
        clockPause();  // [시계] 게임 종료(abort/resign) -> 정지
        String cmd = g_abortCmd;
        g_abortThink = false;
        g_abortCmd = "";
        waitingForMoveComplete = false;
        pendingCompleteCount = 0;
        if (cmd == "resign") {
          if (playerIsWhite) {
            Serial.println(F("\n>>> You (White) Resigned! AI (Black) Wins."));
            sendKingStatus(true, ID_W_RESIGN);
            sendKingStatus(false, ID_B_WIN);
          } else {
            Serial.println(F("\n>>> You (Black) Resigned! AI (White) Wins."));
            sendKingStatus(false, ID_B_RESIGN);
            sendKingStatus(true, ID_W_WIN);
          }
          VFD_print("YOU LOSE");   // [VFD]
        } else {
          Serial.println(F("\n>>> Game aborted."));
        }
        gameover = 1;
        break;
      }

      if (pos[0].best.f1 != 0) {
        Serial.println(F(">>> Waiting for board to settle..."));
        waitMoveSettle(8000);   // (기존) 직전 수 정착 대기 — 그대로 유지

        unsigned long marginStart = millis();
        while (millis() - marginStart < 1000) {
          handleRingNetwork();
          pollThinkAbort();
          if (g_abortThink) break;
          delay(10);
        }

        if (g_abortThink) {
          clockPause();  // [시계] 게임 종료(abort/resign) -> 정지
          String cmd = g_abortCmd;
          g_abortThink = false;
          g_abortCmd = "";
          waitingForMoveComplete = false;
          pendingCompleteCount = 0;
          if (cmd == "resign") {
            if (playerIsWhite) {
              Serial.println(F("\n>>> You (White) Resigned! AI (Black) Wins."));
              sendKingStatus(true, ID_W_RESIGN);
              sendKingStatus(false, ID_B_WIN);
            } else {
              Serial.println(F("\n>>> You (Black) Resigned! AI (White) Wins."));
              sendKingStatus(false, ID_B_RESIGN);
              sendKingStatus(true, ID_W_WIN);
            }
            VFD_print("YOU LOSE");   // [VFD]
          } else {
            Serial.println(F("\n>>> Game aborted."));
          }
          gameover = 1;
          break;
        }

        step_t aiMove = pos[0].best;
        movestep(0, aiMove);
        movepos(0, aiMove);
        pos[1].w = !pos[0].w;
        pos[0] = pos[1];
        game_history_hash[game_history_count++] = pos[0].hash;

        Serial.print(F("AI played: "));
        Serial.println(str_step(aiMove));
        Serial.print("FEN_DATA:");
        Serial.println(fenout(0));

        clockPause();                                      // [시계] MOVING...과 동시에 정지
        clockAddIncrement(isWhiteTurn);                    // [시계] AI가 둔 수의 증가시간 적용 (정지 시점에 적용)
        VFD_print("MOVING...");                            // [VFD] AI 수 모터 표시
        updateBoardDiff();                                 // AI 수 전송
        updateCheckDisplay(wasWhiteCheck, wasBlackCheck);  // [VFD] 체크/해소 '수와 동시' 전송
        waitMoveSettle(8000);                              // [추가] AI 수 모든 모터 완료 대기
      } else {
        clockPause();
        waitingForMoveComplete = false;
        pendingCompleteCount = 0;
        kingpositions();
        bool aiInCheck = isWhiteTurn ? check_w() : check_b();
        if (aiInCheck) {
          if (isWhiteTurn) {
            Serial.println(F("CHECKMATE! Black Wins."));
            sendKingStatus(true, ID_W_MATE);
            sendKingStatus(false, ID_B_WIN);
          } else {
            Serial.println(F("CHECKMATE! White Wins."));
            sendKingStatus(false, ID_B_MATE);
            sendKingStatus(true, ID_W_WIN);
          }
          VFD_print("YOU WIN");
        } else {
          Serial.println(F("STALEMATE! Draw."));
          sendKingStatus(true, ID_W_DRAW);
          sendKingStatus(false, ID_B_DRAW);
          VFD_print("DRAW");
        }
        gameover = 1;
      }
    }
    delay(10);
  }
}

void game_pvp() {
  // 2인용 모드에서는 물리 보드의 기준 시점을 항상 백(White)을 아래로 둡니다.
  g_playerIsWhite = true;

  String s = "";
  boolean gameover = 0;

  // 원본과 완벽히 동일한 50ms 간격의 초기 호밍 시퀀스 실행
  executeInitialSequence();
  

  // 초기 보드 상태 세팅 (FEN)
  fen("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
  kingpositions();

  // 해시 및 히스토리 초기화
  game_history_count = 0;
  pos[0].hash = calculate_full_hash(0);
  pos[0].halfmove_clock = 0;
  game_history_hash[game_history_count++] = pos[0].hash;

  Serial.println(F("\n=== CHESS GAME START (PLAYER vs PLAYER) ==="));
  Serial.print("FEN_DATA:");
  Serial.println(fenout(0));

  // 호밍 완료 후 초기 보드 상태 물리적 전송
  forceFullSync();

  bool wasWhiteCheck = false;
  bool wasBlackCheck = false;

  while (!gameover) {
    handleRingNetwork();
    boolean isWhiteTurn = (pos[0].w == 1);

    // 1. 양 진영 체크 상태 확인 및 물리 보드 전송
    bool currentWhiteCheck = check_w();
    bool currentBlackCheck = check_b();

    if (currentWhiteCheck && !wasWhiteCheck) {
      sendKingStatus(true, ID_W_CHECK);
      wasWhiteCheck = true;
    } else if (!currentWhiteCheck && wasWhiteCheck) {
      sendKingStatus(true, ID_W_KING);
      wasWhiteCheck = false;
    }

    if (currentBlackCheck && !wasBlackCheck) {
      sendKingStatus(false, ID_B_CHECK);
      wasBlackCheck = true;
    } else if (!currentBlackCheck && wasBlackCheck) {
      sendKingStatus(false, ID_B_KING);
      wasBlackCheck = false;
    }

    bool isCheck = isWhiteTurn ? currentWhiteCheck : currentBlackCheck;

    // 2. 합법적인 수 계산 및 종료(체크메이트/스테일메이트) 판정
    int legalMoves = countLegalMoves();
    if (legalMoves == 0) {
      if (isCheck) {
        if (isWhiteTurn) {
          Serial.println(F("CHECKMATE! Black Wins."));
          sendKingStatus(true, ID_W_MATE);
          sendKingStatus(false, ID_B_WIN);
        } else {
          Serial.println(F("CHECKMATE! White Wins."));
          sendKingStatus(false, ID_B_MATE);
          sendKingStatus(true, ID_W_WIN);
        }
      } else {
        Serial.println(F("STALEMATE! DRAW."));
        sendKingStatus(true, ID_W_DRAW);
        sendKingStatus(false, ID_B_DRAW);
      }
      gameover = 1;
      break;
    }

    if (is_repetition(0) >= 3) {
      Serial.println(F("DRAW by repetition!"));
      sendKingStatus(true, ID_W_DRAW);
      sendKingStatus(false, ID_B_DRAW);
      VFD_print("DRAW");
      clockPause();
      gameover = 1;
      break;
    }
    if (pos[0].halfmove_clock >= 100) {
      Serial.println(F("DRAW by 50-move rule!"));
      sendKingStatus(true, ID_W_DRAW);
      sendKingStatus(false, ID_B_DRAW);
      VFD_print("DRAW");
      clockPause();
      gameover = 1;
      break;
    }

    // 3. 턴 안내 메시지 출력
    if (isWhiteTurn) {
      Serial.println(F("\n>>> WHITE'S TURN!"));
    } else {
      Serial.println(F("\n>>> BLACK'S TURN!"));
    }

    s = "";

    // 시리얼 버퍼 비우기
    while (Serial.available() > 0) Serial.read();

    // 4. 무조건 사용자의 입력을 대기함 (AI 로직 없음)
    while (s == "") {
      handleRingNetwork();
      if (Serial.available() > 0) {
        s = Serial.readStringUntil('\n');
        s.trim();

        if (s == "exit" || s == "quit") {
          gameover = 1;
          break;
        } else if (s == "resign") {
          if (isWhiteTurn) {
            Serial.println(F("\n>>> White Resigned! Black Wins."));
            sendKingStatus(true, ID_W_RESIGN);
            sendKingStatus(false, ID_B_WIN);
          } else {
            Serial.println(F("\n>>> Black Resigned! White Wins."));
            sendKingStatus(false, ID_B_RESIGN);
            sendKingStatus(true, ID_W_WIN);
          }
          gameover = 1;
          break;
        }
      }
      delay(10);
    }
    if (gameover) break;

    // 5. 이동 좌표 파싱 및 기물 이동 검증
    if (s.length() >= 4) {
      int u1 = strToPos(s.substring(0, 2));
      int u2 = strToPos(s.substring(2, 4));
      int targetType = 0;

      // 프로모션 상황 처리
      if (abs(pole[u1]) == fp && (row[u2] == 8 || row[u2] == 1)) {
        targetType = waitPromoChoice();
      }

      int moveIdx = -1;
      generate_steps(0);

      for (int i = 0; i < pos[0].n_steps; i++) {
        if (pos[0].steps[i].c1 == u1 && pos[0].steps[i].c2 == u2) {
          if (targetType != 0) {
            if (pos[0].steps[i].type == targetType) {
              moveIdx = i;
              break;
            }
          } else {
            if (pos[0].steps[i].type <= 3) {
              moveIdx = i;
              break;
            }
          }
        }
      }

      if (moveIdx != -1) {
        step_t userStep = pos[0].steps[moveIdx];
        movestep(0, userStep);
        kingpositions();

        // 이동 후 킹이 공격받는지 확인
        if (isWhiteTurn ? check_w() : check_b()) {
          backstep(0, userStep);
          Serial.println(F("Illegal: King under attack!"));
          Serial.print("FEN_DATA:");
          Serial.println(fenout(0));
        } else {
          movepos(0, userStep);
          pos[1].w = !pos[0].w;
          pos[0] = pos[1];
          game_history_hash[game_history_count++] = pos[0].hash;

          Serial.print(F("Played: "));
          Serial.println(str_step(userStep));
          Serial.print("FEN_DATA:");
          Serial.println(fenout(0));

          // 물리 보드 이동 명령 전송
          updateBoardDiff();
        }
      } else {
        Serial.println(F("Illegal Move!"));
        Serial.print("FEN_DATA:");
        Serial.println(fenout(0));
      }
    }
  }
}

// =============================================================
// [추가] FEN 불러오기 및 자유 대국 (Sandbox) 모드
// =============================================================
void game_fen_sandbox(String customFen) {
  // 물리 보드의 시점을 백(White)을 아래로 고정
  g_playerIsWhite = true;

  String s = "";
  boolean gameover = 0;

  // 1. 호밍 시퀀스 실행
  executeInitialSequence();

  // 2. FEN 파싱 및 보드 상태 세팅
  if (!fen(customFen)) {
    Serial.println(F("[ERROR] Invalid FEN String. Returning to idle."));
    return;
  }

  // 킹 위치 추적 및 엔진 해시 초기화
  kingpositions();
  game_history_count = 0;
  pos[0].hash = calculate_full_hash(0);
  pos[0].halfmove_clock = 0;
  game_history_hash[game_history_count++] = pos[0].hash;

  Serial.println(F("\n=== FEN SANDBOX MODE START ==="));
  Serial.print("FEN_DATA:");
  Serial.println(fenout(0));

  // 3. 불러온 FEN 상태를 물리 보드에 강제 동기화
  forceFullSync();

  bool wasWhiteCheck = false;
  bool wasBlackCheck = false;

  // 4. 자유 대국 루프 (엔진 룰 및 체크/메이트 검증 포함)
  while (!gameover) {
    handleRingNetwork();
    boolean isWhiteTurn = (pos[0].w == 1);

    // 양 진영 체크 상태 확인 및 물리 보드 전송
    bool currentWhiteCheck = check_w();
    bool currentBlackCheck = check_b();

    if (currentWhiteCheck && !wasWhiteCheck) {
      sendKingStatus(true, ID_W_CHECK);
      wasWhiteCheck = true;
    } else if (!currentWhiteCheck && wasWhiteCheck) {
      sendKingStatus(true, ID_W_KING);
      wasWhiteCheck = false;
    }

    if (currentBlackCheck && !wasBlackCheck) {
      sendKingStatus(false, ID_B_CHECK);
      wasBlackCheck = true;
    } else if (!currentBlackCheck && wasBlackCheck) {
      sendKingStatus(false, ID_B_KING);
      wasBlackCheck = false;
    }

    bool isCheck = isWhiteTurn ? currentWhiteCheck : currentBlackCheck;

    // 합법적인 수 계산 및 종료(체크메이트/스테일메이트) 판정
    int legalMoves = countLegalMoves();
    if (legalMoves == 0) {
      if (isCheck) {
        if (isWhiteTurn) {
          Serial.println(F("CHECKMATE! Black Wins."));
          sendKingStatus(true, ID_W_MATE);
          sendKingStatus(false, ID_B_WIN);
        } else {
          Serial.println(F("CHECKMATE! White Wins."));
          sendKingStatus(false, ID_B_MATE);
          sendKingStatus(true, ID_W_WIN);
        }
      } else {
        Serial.println(F("STALEMATE! DRAW."));
        sendKingStatus(true, ID_W_DRAW);
        sendKingStatus(false, ID_B_DRAW);
      }
      gameover = 1;
      break;
    }

    // 3회 동형 반복 및 50수 무승부 룰 검사
    if (is_repetition(0) >= 3) {
      Serial.println(F("DRAW by repetition!"));
      sendKingStatus(true, ID_W_DRAW);
      sendKingStatus(false, ID_B_DRAW);
      VFD_print("DRAW");
      clockPause();
      gameover = 1;
      break;
    }
    if (pos[0].halfmove_clock >= 100) {
      Serial.println(F("DRAW by 50-move rule!"));
      sendKingStatus(true, ID_W_DRAW);
      sendKingStatus(false, ID_B_DRAW);
      VFD_print("DRAW");
      clockPause();
      gameover = 1;
      break;
    }

    // 턴 안내 메시지
    if (isWhiteTurn) {
      Serial.println(F("\n>>> [SANDBOX] WHITE'S TURN!"));
    } else {
      Serial.println(F("\n>>> [SANDBOX] BLACK'S TURN!"));
    }

    s = "";

    while (Serial.available() > 0) Serial.read();

    // 사용자 입력 대기
    while (s == "") {
      handleRingNetwork();
      if (Serial.available() > 0) {
        s = Serial.readStringUntil('\n');
        s.trim();

        if (s == "exit" || s == "quit") {
          gameover = 1;
          break;
        }
      }
      delay(10);
    }
    if (gameover) break;

    // 이동 좌표 파싱 및 기물 이동 검증
    if (s.length() >= 4) {
      int u1 = strToPos(s.substring(0, 2));
      int u2 = strToPos(s.substring(2, 4));
      int targetType = 0;

      // 프로모션 처리
      if (abs(pole[u1]) == fp && (row[u2] == 8 || row[u2] == 1)) {
        targetType = waitPromoChoice();
      }

      int moveIdx = -1;
      generate_steps(0);  // 현재 FEN 기준 합법적인 수 생성

      for (int i = 0; i < pos[0].n_steps; i++) {
        if (pos[0].steps[i].c1 == u1 && pos[0].steps[i].c2 == u2) {
          if (targetType != 0) {
            if (pos[0].steps[i].type == targetType) {
              moveIdx = i;
              break;
            }
          } else {
            if (pos[0].steps[i].type <= 3) {
              moveIdx = i;
              break;
            }
          }
        }
      }

      if (moveIdx != -1) {
        step_t userStep = pos[0].steps[moveIdx];
        movestep(0, userStep);
        kingpositions();

        // 이동 후 스스로 체크에 걸리는 자살수 방지
        if (isWhiteTurn ? check_w() : check_b()) {
          backstep(0, userStep);
          Serial.println(F("Illegal: King under attack!"));
          Serial.print("FEN_DATA:");
          Serial.println(fenout(0));
        } else {
          // 정상적인 수 처리
          movepos(0, userStep);
          pos[1].w = !pos[0].w;
          pos[0] = pos[1];
          game_history_hash[game_history_count++] = pos[0].hash;

          Serial.print(F("Played: "));
          Serial.println(str_step(userStep));
          Serial.print("FEN_DATA:");
          Serial.println(fenout(0));

          // 물리 보드로 변경된 기물 위치 전송
          updateBoardDiff();
        }
      } else {
        Serial.println(F("Illegal Move!"));
        Serial.print("FEN_DATA:");
        Serial.println(fenout(0));
      }
    }
  }
}

// 프로모션 UI 임시 테스트: 백킹 a1, 흑킹 h8, 백폰 e7 → 바로 e8 프로모션 가능
void promoTest() {
  Serial.println(F("[PROMO TEST] Position: K at a1, P at e7, k at h8"));
  Serial.println(F("[PROMO TEST] Press e7 key -> e8 key -> choose promotion piece. 'exit' to quit."));

  g_playerIsWhite = true;
  fen("7k/4P3/8/8/8/8/8/K7 w - - 0 1");
  kingpositions();
  game_history_count = 0;
  pos[0].hash = calculate_full_hash(0);
  game_history_hash[game_history_count++] = pos[0].hash;
  bool wasWhiteCheck = false;
  bool wasBlackCheck = false;

  VFD_print("SETTING UP...");
  forceFullSync();       // 물리 보드 초기 배치
  waitMoveSettle(8000);  // 기물 이동 완료 대기

  clocksInit();
  clocksPower(g_timeControl != TC_UNLIMITED);
  clockResume(true);  // 백 턴: 시계 시작

  clearGameSelection();
  VFD_print("PROMO TEST");

  // 이전 키 상태 초기화
  scanMatrix(); debounceMatrix();
  bool prevT[MATRIX_ROWS][MATRIX_COLS];
  memcpy(prevT, debouncedState, sizeof(prevT));
  unsigned long lastBlink = 0;
  bool blinkOn = true;

  String s = "";
  while (s == "") {
    handleRingNetwork();
    clockUpdate();
    scanMatrix();
    debounceMatrix();

    if (g_selSq != -1 && millis() - lastBlink > 300) {
      blinkOn = !blinkOn;
      showGameLEDs(blinkOn);
      lastBlink = millis();
    }

    for (int r = 0; r < MATRIX_ROWS; r++) {
      for (int c = 0; c < MATRIX_COLS; c++) {
        if (debouncedState[r][c] && !prevT[r][c]) {
          int sq = keyToSquare(r, c);
          if (sq >= 0) handleSquarePress(sq, true, s);
        }
        prevT[r][c] = debouncedState[r][c];
      }
    }
    if (s != "") break;

    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd == "exit" || cmd == "quit") { clearGameSelection(); return; }
      s = cmd;
    }
    delay(10);
  }

  if (s.length() >= 4) {
    int u1 = strToPos(s.substring(0, 2));
    int u2 = strToPos(s.substring(2, 4));

    if (abs(pole[u1]) == fp && (row[u2] == 8 || row[u2] == 1)) {
      generate_steps(0);
      step_t qStep; bool qFound = false;
      for (int i = 0; i < pos[0].n_steps; i++) {
        if (pos[0].steps[i].c1 == u1 && pos[0].steps[i].c2 == u2 && pos[0].steps[i].type == 7) {
          qStep = pos[0].steps[i]; qFound = true; break;
        }
      }
      if (!qFound) {
        Serial.println(F("[PROMO TEST] Invalid move."));
      } else {
        movestep(0, qStep);
        if (check_w()) {
          backstep(0, qStep);
          Serial.println(F("[PROMO TEST] Illegal (king in check)"));
        } else {
          // 1) 폰이 끝 랭크까지 물리 이동
          int savedQ = pole[u2];
          pole[u2] = fp;
          clockPause();
          VFD_print("MOVING...");
          updateBoardDiff();
          pole[u2] = savedQ;
          waitMoveSettle(8000);

          // 2) 기물 선택 (시계 재개)
          clockResume(true);
          int selectedType = waitPromoChoice();
          clockPause();
          clockAddIncrement(true);

          // 3) 선택된 타입으로 확정
          backstep(0, qStep);
          generate_steps(0);
          int finalIdx = -1;
          for (int i = 0; i < pos[0].n_steps; i++) {
            if (pos[0].steps[i].c1 == u1 && pos[0].steps[i].c2 == u2 && pos[0].steps[i].type == selectedType) {
              finalIdx = i; break;
            }
          }
          if (finalIdx != -1) {
            step_t finalStep = pos[0].steps[finalIdx];
            movestep(0, finalStep);
            movepos(0, finalStep);
            pos[1].w = !pos[0].w;
            pos[0] = pos[1];
            game_history_hash[game_history_count++] = pos[0].hash;
            Serial.printf("[PROMO TEST] OK: %s (type=%d)\n", str_step(finalStep).c_str(), finalStep.type);
            VFD_print("MOVING...");
            updateBoardDiff();  // 프로모션 기물 교체 명령 슬레이브에 전송
            updateCheckDisplay(wasWhiteCheck, wasBlackCheck);
            waitMoveSettle(8000);
          }
          clearGameSelection();
          game(true, false);
          return;
        }
      }
    } else {
      Serial.println(F("[PROMO TEST] Not a promotion move."));
    }
  }

  clearGameSelection();
  enterSelectMode();
}

void setup() {
  Serial.begin(115200);
  RING_SERIAL.setRxBufferSize(2048);  // [추가] begin() 전에 호출해야 적용됨
  RING_SERIAL.begin(115200, SERIAL_8N1, RING_RX, RING_TX);

  pinMode(vfd_clk, OUTPUT);
  pinMode(vfd_din, OUTPUT);
  pinMode(vfd_cs, OUTPUT);
  VFD_init();

  clocksInit();
  clocksPower(false);  // [시계] 부팅 시에는 꺼둠

  init_filesystem();
  init_zobrist();

  initWifi();  // 기존 WiFi.begin() 블록 전체 교체

  strip.begin();
  strip.setBrightness(50);
  strip.show();

  for (int r = 0; r < MATRIX_ROWS; r++) pinMode(rowPins[r], INPUT);
  for (int c = 0; c < MATRIX_COLS; c++) pinMode(colPins[c], INPUT_PULLUP);
  enterSelectMode();

  for (int i = 0; i < 64; i++) {
    pole[i] = 0;
    prev_pole[i] = 0;
  }

  Serial.println(F("Start"));
  xTaskCreate(taskOne, "TaskOne", 10000, NULL, 1, NULL);
}

void loop() {
  bootInitOnce();  // 부팅 1회 게이트(완료되면 즉시 통과)
  handleRingNetwork();
  scanMatrix();
  debounceMatrix();
  processSelectButtons();

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "reset") {
      Serial.println("\n>>> [COMMAND] Restarting ESP32-S3...");
      delay(100);
      ESP.restart();
    } else if (input == "r_test") {
      Serial.println("\n>>> [COMMAND] Start Loop Test (TOKHOME polling)");
      isLoopTestMode = true;
    } else if (input == "stop") {
      Serial.println("\n>>> [COMMAND] Stop Test -> Normal Mode");
      isLoopTestMode = false;
    } else if (input == "wifi_reset") {
      Serial.println("[WiFi] Clearing saved config...");
      LittleFS.remove(WIFI_FILE);
      WiFi.disconnect();
      scanAndConnectWifi();
    } else if (input.startsWith("tc:")) {
      String tcArg = input.substring(3);
      tcArg.trim();
      tcArg.toLowerCase();
      if (tcArg == "5") {
        g_timeControl = TC_5_0;
      } else if (tcArg == "10") {
        g_timeControl = TC_10_0;
      } else if (tcArg == "15") {
        g_timeControl = TC_15_10;
      } else if (tcArg == "30") {
        g_timeControl = TC_30_0;
      } else if (tcArg == "unlimited") {
        g_timeControl = TC_UNLIMITED;
      } else {
        Serial.print(F(">>> Unknown time control: "));
        Serial.println(tcArg);
      }
      Serial.print(F(">>> Time control set to: "));
      Serial.println(tcArg);
    } else if (input == "promo_test") {
      promoTest();
    } else if (input == "game_w") {
      game(true);
    } else if (input == "game_b") {
      game(false);
    } else if (input == "game_pvp") {
      game_pvp();
    } else if (input.startsWith("pgn:")) {
      String pgnData = input.substring(4);
      replayPGN(pgnData);
    } else if (input.startsWith("fen:")) {
      String customFen = input.substring(4);
      game_fen_sandbox(customFen);
    } else if (input.startsWith("study:")) {
      int minutes = input.substring(6).toInt();
      if (minutes <= 0) minutes = 10;  // 기본 10분
      startSelfStudy(minutes);
    } else if (input.indexOf('=') != -1) {
      processSmartCommand(input);
    } else {
      if (input.length() > 0) {
        Serial.print("[RAW TX to RING] ");
        Serial.println(input);
        RING_SERIAL.println(input);
      }
    }
  }

  // ── r_test: 폴링(TOKHOME) 기반 리부트+호밍 루프 테스트 ──
  if (isLoopTestMode) {
    static int testCycle = 0;
    Serial.printf("\n[TEST] === Cycle %d : reboot + homing (TOKHOME polling) ===\n", ++testCycle);

    executeInitialSequence();

    // 누적 통계 출력: 사이클 수 / 타임아웃 횟수 / 성공률
    unsigned long ok = (unsigned long)testCycle - g_homingTimeoutCount;
    Serial.printf("[TEST] Cycle %d done. timeouts=%lu / %d  (success=%lu)\n",
                  testCycle, g_homingTimeoutCount, testCycle, ok);

    Serial.println("[TEST] Waiting 1s... (type 'stop' to end)");
    unsigned long t = millis();
    while (millis() - t < 1000) {
      handleRingNetwork();
      if (Serial.available()) {
        String c = Serial.readStringUntil('\n');
        c.trim();
        if (c == "stop") {
          isLoopTestMode = false;
          Serial.printf(">>> [COMMAND] Stop Test. total cycles=%d, timeouts=%lu\n",
                        testCycle, g_homingTimeoutCount);
          testCycle = 0;
          g_homingTimeoutCount = 0;  // 다음 테스트 위해 리셋(원치 않으면 이 줄 제거)
          break;
        }
      }
      delay(5);
    }
  }
}