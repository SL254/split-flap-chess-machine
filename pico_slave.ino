#include <Arduino.h>
#include <EEPROM.h>
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"

// ==========================================
// [설정] 내 노드 ID
// ==========================================
const int MY_NODE_ID = 7;
#define RING_SERIAL Serial1
const long BAUD_RATE = 115200;

// ==========================================
// [설정] 하드웨어 - 8개 모터 핀맵
// ==========================================
const int NUM_MOTORS = 8;
const byte ENABLE_PINS[NUM_MOTORS] = { 21, 22, 26, 27, 28, 12, 11, 10 };
const byte STEP_PINS[NUM_MOTORS] = { 9, 8, 7, 6, 5, 4, 3, 2 };
const byte HALL_PINS[NUM_MOTORS] = { 20, 19, 18, 17, 16, 15, 14, 13 };
const byte COMMON_DIR_PIN = 25;

// ==========================================
// [시스템 상수 및 저장소]
// ==========================================
const uint32_t OLD_FLASH_MAGIC = 0xABC12349;  // 이전 버전 매직넘버
const uint32_t NEW_FLASH_MAGIC = 0xABC12350;  // 새 버전 매직넘버
const int TOTAL_PIECES = 27;
const int MAX_POSITIONS = 32;

struct StepperSettings {
  uint32_t magic;
  long globalOffsets[NUM_MOTORS];
  long slotOffsets[NUM_MOTORS][MAX_POSITIONS];
  float commonSpeed;
  float commonAccel;
  long stepLossComp;
};

StepperSettings globalSettings;
const long STEPS_PER_REV = 24576;

// PIO 프로그램
const uint16_t stepper_pio_instructions[] = { 0x80a0, 0xa027, 0xb042, 0xa042, 0x0044 };
const struct pio_program stepper_program = { .instructions = stepper_pio_instructions, .length = 5, .origin = -1 };

// ==========================================
// [멀티코어 큐 및 상태]
// ==========================================
enum CommandType { CMD_NONE,
                   CMD_REBOOT,
                   CMD_STOP,
                   CMD_MOVE_BATCH,
                   CMD_REHOME_SINGLE,
                   CMD_REHOME_AND_MOVE };
struct MotorCommand {
  CommandType type;
  int32_t targets[NUM_MOTORS];
  int motorId;
  int32_t extraParam;
};
queue_t commandQueue;

volatile long extraHomeOffsets[NUM_MOTORS] = { 0 };
volatile long cachedSlotOffsets[NUM_MOTORS][MAX_POSITIONS] = { 0 };
volatile bool statusSystemHomed = false;
volatile bool statusMoveComplete = false;

// === 토큰 프로토콜 ===
volatile unsigned long myMoveGen    = 0;   // 받은 이동 명령 누적 세대
volatile unsigned long completedGen = 0;   // 완료까지 끝낸 이동 세대

bool isTestMode = false;
bool isManualMode = false;
bool waitingForSetComplete = false;
int simpleTestId = -1;
int simpleTestCount = 0;

const float DECEL_TIMING_FACTOR = 0.6;
unsigned long lastMoveTimes[NUM_MOTORS] = { 0 };
const int HOLD_DELAY = 0;
const uint32_t REBOOT_STAGGER_MS = 50;
const int STATE_IDLE = 999;

// ==========================================
// [통신]
// ==========================================
void sendToRing(String msg) {
  RING_SERIAL.println(msg);
}
void logMsg(String msg) {
  Serial.println(msg);
  sendToRing("MASTER:LOG:" + String(MY_NODE_ID) + ":" + msg);
}

// ==========================================
// [클래스] PioStepper
// ==========================================
class PioStepper {
private:
  PIO _pio;
  uint _sm, _stepPin, _dirPin, _enPin;
  long _currentPos = 0, _targetPos = 0;
  long _pulsesRemaining = 0;
  float _currentSpeed = 0, _targetSpeed = 0, _acceleration = 0, _deceleration = 0;
  bool _isRunning = false, _isHomingMode = false, _isPending = false;
  int _dir = 1;
  unsigned long _scheduleTime = 0, _lastUpdateMicros = 0;

public:
  void attach(PIO pio, uint sm, uint stepPin, uint dirPin, uint enPin) {
    _pio = pio;
    _sm = sm;
    _stepPin = stepPin;
    _dirPin = dirPin;
    _enPin = enPin;
    pinMode(_dirPin, OUTPUT);
    pinMode(_enPin, OUTPUT);
    digitalWrite(_enPin, HIGH);
    int loaded_offset = pio_add_program(_pio, &stepper_program);
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_sideset(&c, 1, false, false);
    sm_config_set_wrap(&c, loaded_offset, loaded_offset + 4);
    sm_config_set_sideset_pins(&c, _stepPin);
    sm_config_set_clkdiv(&c, clock_get_hz(clk_sys) / 1000000.0);
    pio_gpio_init(_pio, _stepPin);
    pio_sm_set_consecutive_pindirs(_pio, _sm, _stepPin, 1, true);
    pio_sm_init(_pio, _sm, loaded_offset, &c);
    pio_sm_set_enabled(_pio, _sm, true);
  }
  void enable() {
    digitalWrite(_enPin, LOW);
  }
  void disable() {
    digitalWrite(_enPin, HIGH);
  }
  void setZero() {
    _currentPos = 0;
    _targetPos = 0;
  }
  long getCurrentPos() {
    return _currentPos;
  }
  bool isHomingMode() {
    return _isHomingMode;
  }
  void stop() {
    _targetSpeed = 0;
    _currentSpeed = 0;
    _isRunning = false;
    _isPending = false;
    _targetPos = _currentPos;
    _pulsesRemaining = 0;
    pio_sm_clear_fifos(_pio, _sm);
  }
  void rotate(int dir, float speed, float accel, float decel, uint32_t delayMs) {
    _isHomingMode = true;
    _dir = dir;
    _targetSpeed = speed;
    _acceleration = accel;
    _deceleration = decel;
    _scheduleTime = millis() + delayMs;
    _isPending = true;
    _isRunning = false;
    digitalWrite(COMMON_DIR_PIN, HIGH);
  }
  void move(long steps, float speed, float accel, float decel, uint32_t delayMs, long extraPulses = 0) {
    if (steps <= 0) return;
    _isHomingMode = false;
    _targetPos = _currentPos + steps;
    _pulsesRemaining = steps + extraPulses;
    _targetSpeed = speed;
    _acceleration = accel;
    _deceleration = decel;
    _dir = 1;
    digitalWrite(COMMON_DIR_PIN, HIGH);
    _scheduleTime = millis() + delayMs;
    _isPending = true;
    _isRunning = false;
  }
  bool moving() {
    return _isRunning || _isPending || (_pulsesRemaining > 0 && !_isHomingMode);
  }
  void finishHoming(long extraSteps) {
    _isHomingMode = false;
    _currentPos = 0;
    long dist = extraSteps;
    while (dist <= 0) dist += STEPS_PER_REV;
    _targetPos = dist;
    _pulsesRemaining = dist;
    _dir = 1;
    digitalWrite(COMMON_DIR_PIN, HIGH);
    _isRunning = true;
    _lastUpdateMicros = micros();
  }
  void update() {
    if (_isPending) {
      if (millis() >= _scheduleTime) {
        _isPending = false;
        _isRunning = true;
        _lastUpdateMicros = micros();
        _currentSpeed = 400.0;
        enable();
      } else return;
    }
    if (!_isRunning && !_isHomingMode && _pulsesRemaining <= 0) return;
    unsigned long now = micros();
    float dt = (now - _lastUpdateMicros) / 1000000.0;
    _lastUpdateMicros = now;
    if (dt > 0.05) dt = 0.0;
    long dist = (_isHomingMode) ? 100000 : _pulsesRemaining;
    if (dist < 0) dist = 0;
    float stopDist = ((_currentSpeed * _currentSpeed) / (2.0 * _deceleration)) * DECEL_TIMING_FACTOR;
    if (!_isHomingMode && dist <= stopDist) {
      _currentSpeed -= _deceleration * dt;
      if (_currentSpeed < 5000.0) _currentSpeed = 5000.0;
    } else if (_currentSpeed < _targetSpeed) {
      _currentSpeed += _acceleration * dt;
      if (_currentSpeed > _targetSpeed) _currentSpeed = _targetSpeed;
    }
    if (!_isHomingMode && dist == 0) {
      _isRunning = false;
      _currentSpeed = 0;
      return;
    }
    if (!pio_sm_is_tx_fifo_full(_pio, _sm) && _currentSpeed > 1.0) {
      uint32_t delay_cycles = (uint32_t)(1000000.0 / _currentSpeed);
      pio_sm_put_blocking(_pio, _sm, (delay_cycles < 2) ? 2 : delay_cycles);
      if (_currentPos != _targetPos) _currentPos += _dir;
      if (!_isHomingMode && _pulsesRemaining > 0) _pulsesRemaining--;
    }
  }
};

PioStepper steppers[NUM_MOTORS];
int states[NUM_MOTORS] = { 0 }, positions[NUM_MOTORS] = { 0 };
int core1_pendingTargets[NUM_MOTORS] = { 0 };

// ==========================================
// [물리 제어 함수] - Core 1
// ==========================================
bool isSensorActive(int id) {
  return (digitalRead(HALL_PINS[id]) == LOW);
}

void startSingleHoming(int id) {
  steppers[id].stop();
  steppers[id].setZero();
  states[id] = isSensorActive(id) ? -1 : 0;
  positions[id] = 1;
  steppers[id].rotate(1, globalSettings.commonSpeed, globalSettings.commonAccel, globalSettings.commonAccel, 0);
  steppers[id].enable();
  statusSystemHomed = false;
}

void moveToIndex(int id, int targetPos, uint32_t delayMs) {

  if (targetPos == 1) {
    if (positions[id] == 1 && states[id] == 100 && !steppers[id].moving()) return;
    startSingleHoming(id);
    return;
  }

  if (targetPos == positions[id] && !steppers[id].moving()) return;

  if (targetPos < 0 || targetPos >= MAX_POSITIONS) return;

  long targetOffset = cachedSlotOffsets[id][targetPos];

  long currentAbsPos = steppers[id].getCurrentPos();

  long currentRelPos = currentAbsPos % STEPS_PER_REV;

  if (currentRelPos < 0) currentRelPos += STEPS_PER_REV;

  long targetRelPos = ((long)STEPS_PER_REV * (targetPos - 1)) / TOTAL_PIECES - 8;

  long stepsToMove = targetRelPos - currentRelPos;

  if (stepsToMove <= 0) stepsToMove += STEPS_PER_REV;

  if (stepsToMove < 16) stepsToMove += STEPS_PER_REV;

  stepsToMove += targetOffset;

  steppers[id].move(stepsToMove, globalSettings.commonSpeed, globalSettings.commonAccel, globalSettings.commonAccel, delayMs, globalSettings.stepLossComp);

  positions[id] = targetPos;
}

bool checkHoming(int id) {
  if (states[id] == 100) return true;
  if (states[id] == -1) {
    if (!isSensorActive(id)) states[id] = 0;
  } else if (states[id] == 0) {
    if (isSensorActive(id)) {
      steppers[id].finishHoming(extraHomeOffsets[id]);
      states[id] = 1;
    }
  } else if (states[id] == 1) {
    if (!steppers[id].moving()) {
      steppers[id].setZero();
      positions[id] = 1;
      states[id] = 100;
      if (core1_pendingTargets[id] != 0) {
        moveToIndex(id, core1_pendingTargets[id], 0);
        core1_pendingTargets[id] = 0;
      }
      return true;
    }
  }
  return false;
}

void performSystemRebootLogic() {
  statusSystemHomed = false;
  for (int i = 0; i < NUM_MOTORS; i++) {
    core1_pendingTargets[i] = 0;
    steppers[i].stop();
    steppers[i].setZero();
    states[i] = isSensorActive(i) ? -1 : 0;
    positions[i] = 1;
    uint32_t startDelay = i * REBOOT_STAGGER_MS;
    steppers[i].rotate(1, globalSettings.commonSpeed, globalSettings.commonAccel, globalSettings.commonAccel, startDelay);
    steppers[i].enable();
  }
}

void core1_entry() {
  multicore_lockout_victim_init();
  while (true) {
    MotorCommand cmd;
    if (queue_try_remove(&commandQueue, &cmd)) {
      switch (cmd.type) {
        case CMD_REBOOT: performSystemRebootLogic(); break;
        case CMD_STOP:
          for (int i = 0; i < NUM_MOTORS; i++) {
            steppers[i].stop();
            steppers[i].disable();
            states[i] = STATE_IDLE;
          }
          statusSystemHomed = false;
          break;
        case CMD_MOVE_BATCH:
          for (int i = 0; i < NUM_MOTORS; i++) {
            if (cmd.targets[i] >= 1) moveToIndex(i, cmd.targets[i], 0);
          }
          statusMoveComplete = false;
          break;
        case CMD_REHOME_SINGLE:
          if (cmd.motorId >= 0 && cmd.motorId < NUM_MOTORS) startSingleHoming(cmd.motorId);
          break;
        case CMD_REHOME_AND_MOVE:
          if (cmd.motorId >= 0 && cmd.motorId < NUM_MOTORS) {
            core1_pendingTargets[cmd.motorId] = cmd.extraParam;
            startSingleHoming(cmd.motorId);
          }
          break;
        default: break;
      }
    }
    for (int i = 0; i < NUM_MOTORS; i++) steppers[i].update();
    if (!statusSystemHomed) {
      bool allDone = true;
      for (int i = 0; i < NUM_MOTORS; i++) {
        if (states[i] != STATE_IDLE) {
          if (!checkHoming(i)) allDone = false;
        }
      }
      if (states[0] != STATE_IDLE && allDone) statusSystemHomed = true;
    } else {
      bool anyMoving = false;
      for (int i = 0; i < NUM_MOTORS; i++)
        if (steppers[i].moving()) anyMoving = true;
      statusMoveComplete = !anyMoving;
      for (int i = 0; i < NUM_MOTORS; i++) {
        if (states[i] != 100 && states[i] != STATE_IDLE) checkHoming(i);
      }
    }
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (steppers[i].moving()) {
        steppers[i].enable();
        lastMoveTimes[i] = millis();
      } else {
        if (millis() - lastMoveTimes[i] < HOLD_DELAY) steppers[i].enable();
        else steppers[i].disable();
      }
    }
  }
}

// ==========================================
// [EEPROM 제어] - Core 0
// ==========================================
void commitSettingsToFlash() {
  multicore_lockout_start_blocking();
  EEPROM.put(0, globalSettings);
  bool success = EEPROM.commit();
  multicore_lockout_end_blocking();
  if (!success) logMsg("FLASH_ERROR: Commit Failed");
}

void saveOffset(int id, long newVal) {
  if (id < 0 || id >= NUM_MOTORS) return;
  globalSettings.globalOffsets[id] = newVal;
  extraHomeOffsets[id] = newVal;
  commitSettingsToFlash();
}
void saveSlotOffset(int id, int posIndex, long offsetVal) {
  if (id < 0 || id >= NUM_MOTORS || posIndex < 0 || posIndex >= MAX_POSITIONS) return;
  globalSettings.slotOffsets[id][posIndex] = offsetVal;
  cachedSlotOffsets[id][posIndex] = offsetVal;
  commitSettingsToFlash();
}

void sendRebootCmd() {
  MotorCommand cmd;
  cmd.type = CMD_REBOOT;
  queue_try_add(&commandQueue, &cmd);
}
void sendStopCmd() {
  MotorCommand cmd;
  cmd.type = CMD_STOP;
  queue_try_add(&commandQueue, &cmd);
}
void sendMoveBatchCmd(int* targets) {
  MotorCommand cmd;
  cmd.type = CMD_MOVE_BATCH;
  for (int i = 0; i < NUM_MOTORS; i++) cmd.targets[i] = targets[i];
  queue_try_add(&commandQueue, &cmd);
}
void sendRehomeSingleCmd(int id) {
  MotorCommand cmd;
  cmd.type = CMD_REHOME_SINGLE;
  cmd.motorId = id;
  queue_try_add(&commandQueue, &cmd);
}
void sendRehomeAndMoveCmd(int id, int targetPos) {
  MotorCommand cmd;
  cmd.type = CMD_REHOME_AND_MOVE;
  cmd.motorId = id;
  cmd.extraParam = targetPos;
  queue_try_add(&commandQueue, &cmd);
}

// ==========================================
// [명령 처리]
// ==========================================
void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd.equalsIgnoreCase("reboot")) {
    sendRebootCmd();
  } else if (cmd.startsWith("setloss")) {
    int firstParen = cmd.indexOf('(');
    int lastParen = cmd.indexOf(')');
    if (firstParen != -1 && lastParen != -1) {
      String content = cmd.substring(firstParen + 1, lastParen);
      long val = content.toInt();
      if (val >= -1000 && val <= 1000 && val != -1) {
        globalSettings.stepLossComp = val;
        commitSettingsToFlash();
        logMsg("CONFIG: StepLossComp=" + String(globalSettings.stepLossComp));
      }
    }
  } else if (cmd.startsWith("speed")) {
    int firstParen = cmd.indexOf('(');
    int lastParen = cmd.indexOf(')');
    if (firstParen != -1 && lastParen != -1) {
      String content = cmd.substring(firstParen + 1, lastParen);
      int commaIndex = content.indexOf(',');
      if (commaIndex != -1) {
        float s = content.substring(0, commaIndex).toFloat();
        float a = content.substring(commaIndex + 1).toFloat();
        if (s > 0 && a > 0) {
          globalSettings.commonSpeed = s;
          globalSettings.commonAccel = a;
          commitSettingsToFlash();
          logMsg("CONFIG: Speed=" + String(s) + ", Accel=" + String(a));
        }
      }
    }
  } else if (cmd.equalsIgnoreCase("resetall")) {
    globalSettings.magic = NEW_FLASH_MAGIC;
    globalSettings.commonSpeed = 17000.0;
    globalSettings.commonAccel = 90000.0;
    globalSettings.stepLossComp = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
      globalSettings.globalOffsets[i] = 0;
      extraHomeOffsets[i] = 0;
      for (int j = 0; j < MAX_POSITIONS; j++) {
        globalSettings.slotOffsets[i][j] = 0;
        cachedSlotOffsets[i][j] = 0;
      }
    }
    commitSettingsToFlash();
    logMsg("CMD: ALL_RESET -> REBOOTING");
    sendRebootCmd();
  } else if (cmd.equalsIgnoreCase("stop")) {
    isTestMode = false;
    isManualMode = true;
    simpleTestId = -1;
    sendStopCmd();
  } else if (cmd.startsWith("move")) {
    int firstParen = cmd.indexOf('(');
    int lastParen = cmd.indexOf(')');
    if (firstParen != -1 && lastParen != -1) {
      String content = cmd.substring(firstParen + 1, lastParen);
      int commaIndex = content.indexOf(',');
      if (commaIndex != -1) {
        int mask = content.substring(0, commaIndex).toInt();
        int targetPos = content.substring(commaIndex + 1).toInt();
        if (targetPos >= 1 && targetPos <= TOTAL_PIECES) {
          isManualMode = true;
          isTestMode = false;
          int targets[NUM_MOTORS];
          bool anyMoved = false;
          for (int i = 0; i < NUM_MOTORS; i++) {
            if ((mask >> i) & 1) {
              targets[i] = targetPos;
              anyMoved = true;
            } else {
              targets[i] = -1;
            }
          }
          if (anyMoved) {
            myMoveGen++;                  // 이동 세대 증가
            statusMoveComplete = false;   // core0에서 즉시 닫아 토큰 레이스 차단
            waitingForSetComplete = true;
            sendMoveBatchCmd(targets);
          }
        }
      }
    }
  } else if (cmd.startsWith("addindex")) {
    int firstParen = cmd.indexOf('(');
    int lastParen = cmd.indexOf(')');
    if (firstParen != -1 && lastParen != -1) {
      String content = cmd.substring(firstParen + 1, lastParen);
      int firstComma = content.indexOf(',');
      int secondComma = content.indexOf(',', firstComma + 1);
      if (firstComma != -1 && secondComma != -1) {
        int id = content.substring(0, firstComma).toInt();
        int target = content.substring(firstComma + 1, secondComma).toInt();
        long delta = content.substring(secondComma + 1).toInt();
        if (id >= 0 && id < NUM_MOTORS && target >= 1 && target < MAX_POSITIONS) {
          isManualMode = true;
          isTestMode = false;
          long newVal = cachedSlotOffsets[id][target] + delta;
          saveSlotOffset(id, target, newVal);
          sendRehomeAndMoveCmd(id, target);
        }
      }
    }
  } else if (cmd.startsWith("setindex")) {
    int firstParen = cmd.indexOf('(');
    int lastParen = cmd.indexOf(')');
    if (firstParen != -1 && lastParen != -1) {
      String content = cmd.substring(firstParen + 1, lastParen);
      int firstComma = content.indexOf(',');
      int secondComma = content.indexOf(',', firstComma + 1);
      if (firstComma != -1 && secondComma != -1) {
        int id = content.substring(0, firstComma).toInt();
        int target = content.substring(firstComma + 1, secondComma).toInt();
        long offset = content.substring(secondComma + 1).toInt();
        if (id >= 0 && id < NUM_MOTORS && target >= 1 && target < MAX_POSITIONS) {
          isManualMode = true;
          isTestMode = false;
          saveSlotOffset(id, target, offset);
          int targets[NUM_MOTORS];
          for (int i = 0; i < NUM_MOTORS; i++) targets[i] = -1;
          targets[id] = target;
          sendMoveBatchCmd(targets);
        }
      }
    }
  } else if (cmd.equalsIgnoreCase("set")) {
    isTestMode = false;
    int pattern[NUM_MOTORS] = { 3, 3, 3, 3, 3, 3, 3, 3 };
    isManualMode = true;
    waitingForSetComplete = true;
    sendMoveBatchCmd(pattern);
  } else if (cmd.startsWith("test")) {
    int firstParen = cmd.indexOf('(');
    int lastParen = cmd.indexOf(')');
    if (firstParen != -1 && lastParen != -1) {
      String content = cmd.substring(firstParen + 1, lastParen);
      int commaIndex = content.indexOf(',');
      if (commaIndex != -1) {
        int id = content.substring(0, commaIndex).toInt();
        int count = content.substring(commaIndex + 1).toInt();
        if (id >= 0 && id < NUM_MOTORS) {
          simpleTestId = id;
          simpleTestCount = count;
          isManualMode = true;
          sendRehomeSingleCmd(id);
        }
      }
    } else if (cmd.equalsIgnoreCase("test")) {
      isTestMode = true;
      int target;
      do { target = random(2, TOTAL_PIECES + 1); } while (target == positions[0]);
      int targets[NUM_MOTORS];
      for (int i = 0; i < NUM_MOTORS; i++) targets[i] = -1;
      targets[0] = target;
      waitingForSetComplete = true;
      sendMoveBatchCmd(targets);
    }
  } else if (cmd.equalsIgnoreCase("offset")) {
    String msg = "OFFSETS >> ";
    for (int i = 0; i < NUM_MOTORS; i++) {
      msg += "[" + String(i) + "]=" + String(extraHomeOffsets[i]);
      if (i < NUM_MOTORS - 1) msg += ", ";
    }
    logMsg(msg);
  } else if (cmd.startsWith("setoffset")) {
    int openBracket = cmd.indexOf('(');
    int closeBracket = cmd.indexOf(')');
    if (openBracket != -1 && closeBracket != -1) {
      String content = cmd.substring(openBracket + 1, closeBracket);
      int commaIndex = content.indexOf(',');
      if (commaIndex != -1) {
        int id = content.substring(0, commaIndex).toInt();
        long val = content.substring(commaIndex + 1).toInt();
        if (id >= 0 && id < NUM_MOTORS) {
          saveOffset(id, val);
          sendRebootCmd();
        }
      }
    }
  } else if (cmd.startsWith("addoffset")) {
    int openBracket = cmd.indexOf('(');
    int closeBracket = cmd.indexOf(')');
    if (openBracket != -1 && closeBracket != -1) {
      String content = cmd.substring(openBracket + 1, closeBracket);
      int commaIndex = content.indexOf(',');
      if (commaIndex != -1) {
        String idStr = content.substring(0, commaIndex);
        long delta = content.substring(commaIndex + 1).toInt();
        if (idStr.equalsIgnoreCase("all")) {
          for (int i = 0; i < NUM_MOTORS; i++) {
            globalSettings.globalOffsets[i] += delta;
            extraHomeOffsets[i] = globalSettings.globalOffsets[i];
          }
          commitSettingsToFlash();
          sendRebootCmd();
        } else {
          int id = idStr.toInt();
          if (id >= 0 && id < NUM_MOTORS) {
            saveOffset(id, globalSettings.globalOffsets[id] + delta);
            sendRebootCmd();
          }
        }
      }
    }
  } else if (isDigit(cmd.charAt(0))) {
    int target = cmd.toInt();
    if (target >= 1 && target <= TOTAL_PIECES) {
      int targets[NUM_MOTORS];
      for (int i = 0; i < NUM_MOTORS; i++) targets[i] = target;
      isManualMode = true;
      waitingForSetComplete = true;
      sendMoveBatchCmd(targets);
    }
  }
}

// ==========================================
// [통신 핸들링]
// ==========================================
String serialBuffer = "", usbBuffer = "";

void handleMultiCommand(String fullBuffer) {
  sendToRing(fullBuffer);
  int startIndex = 0;
  int semiIndex = fullBuffer.indexOf(';', startIndex);
  while (startIndex < fullBuffer.length()) {
    int endIndex = (semiIndex == -1) ? fullBuffer.length() : semiIndex;
    String segment = fullBuffer.substring(startIndex, endIndex);
    segment.trim();
    if (segment.length() > 0) {
      int separator = segment.indexOf(':');
      if (separator != -1) {
        String targetIDStr = segment.substring(0, separator);
        String command = segment.substring(separator + 1);
        if (targetIDStr.equalsIgnoreCase("all") || targetIDStr.toInt() == MY_NODE_ID) processCommand(command);
      }
    }
    if (semiIndex == -1) break;
    startIndex = semiIndex + 1;
    semiIndex = fullBuffer.indexOf(';', startIndex);
  }
}

// 토큰을 소비하지 않고 자기 비트만 OR해서 다음 노드로 흘림
void relayToken(String tok) {
  bool isMove = tok.startsWith("TOKMOVE:");
  bool isHome = tok.startsWith("TOKHOME:");
  bool isAwk  = tok.startsWith("TOKAWAKE:");
  if (!isMove && !isHome && !isAwk) { sendToRing(tok); return; }

  int bit = (1 << (MY_NODE_ID - 1));

  if (isMove) {
    // TOKMOVE:seq:g1,...,g8:mask
    int c1 = tok.indexOf(':');
    int c2 = tok.indexOf(':', c1 + 1);
    int c3 = tok.lastIndexOf(':');
    if (c2 == -1 || c3 <= c2) { sendToRing(tok); return; }

    String gensStr = tok.substring(c2 + 1, c3);
    int mask = tok.substring(c3 + 1).toInt();

    // 쉼표로 나뉜 8개 세대 중 MY_NODE_ID 번째
    unsigned long reqGen = 0;
    int idx = 0, start = 0;
    for (int i = 0; i <= (int)gensStr.length(); i++) {
      if (i == (int)gensStr.length() || gensStr.charAt(i) == ',') {
        idx++;
        if (idx == MY_NODE_ID) { reqGen = (unsigned long)gensStr.substring(start, i).toInt(); break; }
        start = i + 1;
      }
    }
    if (reqGen != 0 && completedGen >= reqGen) mask |= bit;
    sendToRing(tok.substring(0, c3 + 1) + String(mask));
    return;
  }

  // TOKHOME / TOKAWAKE : 형식 TOKxxx:seq:mask
  int c1 = tok.indexOf(':');
  int c2 = tok.lastIndexOf(':');
  if (c2 <= c1) { sendToRing(tok); return; }
  int mask = tok.substring(c2 + 1).toInt();
  if (isHome) { if (statusSystemHomed) mask |= bit; }
  else        { mask |= bit; }   // AWAKE: 살아서 중계 가능하면 1
  sendToRing(tok.substring(0, c2 + 1) + String(mask));
}

void checkSerialCommand() {
  while (RING_SERIAL.available() > 0) {
    char c = RING_SERIAL.read();
    if (c == '\n') {
      serialBuffer.trim();
      if (serialBuffer.length() > 0) {
        if (serialBuffer.startsWith("TOK")) relayToken(serialBuffer);  // OR 후 중계
        else handleMultiCommand(serialBuffer);
      }
      serialBuffer = "";
    } else serialBuffer += c;
  }
}

void checkUSBInput() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      usbBuffer.trim();
      if (usbBuffer.length() > 0) {
        int separator = usbBuffer.indexOf(':');
        if (separator != -1) {
          String targetIDStr = usbBuffer.substring(0, separator);
          String command = usbBuffer.substring(separator + 1);
          if (targetIDStr.equalsIgnoreCase("all") || targetIDStr.toInt() == MY_NODE_ID) processCommand(command);
          sendToRing(usbBuffer);
        } else {
          processCommand(usbBuffer);
        }
      }
      usbBuffer = "";
    } else usbBuffer += c;
  }
}

// ==========================================
// [Setup & Loop]
// ==========================================
void setup() {
  pinMode(COMMON_DIR_PIN, OUTPUT); digitalWrite(COMMON_DIR_PIN, HIGH);
  RING_SERIAL.setFIFOSize(256);                 // [추가] begin() 전에 호출
  RING_SERIAL.setTX(0); RING_SERIAL.setRX(1); RING_SERIAL.begin(BAUD_RATE);
  Serial.begin(115200); 
  EEPROM.begin(4096); 
  randomSeed(analogRead(26));
  queue_init(&commandQueue, sizeof(MotorCommand), 32);

  // 1. 모터 하드웨어 및 PIO 초기화 (Core 1이 접근해도 터지지 않도록 최우선 세팅)
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(HALL_PINS[i], INPUT_PULLUP);
    PIO targetPio = (i < 4) ? pio0 : pio1;
    uint targetSm = i % 4;
    steppers[i].attach(targetPio, targetSm, STEP_PINS[i], COMMON_DIR_PIN, ENABLE_PINS[i]);
    steppers[i].disable(); 
    states[i] = STATE_IDLE; 
  }

  // 2. Core 1 실행 및 Lockout 수신 대기
  multicore_launch_core1(core1_entry);
  delay(10); // Core 1이 multicore_lockout_victim_init()을 완전히 세팅할 수 있도록 10ms 여유 부여

  // 3. 플래시 메모리 마이그레이션 및 초기화 (이제 안전하게 Lockout 가능)
  EEPROM.get(0, globalSettings);

  if (globalSettings.magic == OLD_FLASH_MAGIC) {
    logMsg("MIGRATION: Old settings detected. Preserving offsets...");
    globalSettings.commonSpeed = 17000.0f;
    globalSettings.commonAccel = 90000.0f;
    globalSettings.stepLossComp = 0;
    globalSettings.magic = NEW_FLASH_MAGIC;
    commitSettingsToFlash();
  }
  else if (globalSettings.magic != NEW_FLASH_MAGIC) {
    globalSettings.magic = NEW_FLASH_MAGIC;
    globalSettings.commonSpeed = 17000.0f;
    globalSettings.commonAccel = 90000.0f;
    globalSettings.stepLossComp = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
      globalSettings.globalOffsets[i] = 0;
      for (int j = 0; j < MAX_POSITIONS; j++) globalSettings.slotOffsets[i][j] = 0;
    }
    commitSettingsToFlash();
  }
  else if (globalSettings.stepLossComp == -1 || globalSettings.stepLossComp < -1000 || globalSettings.stepLossComp > 1000) {
    // 기존(필드 추가 전) EEPROM에는 이 영역이 미기록(0xFF=-1) 상태라 비정상 값일 수 있음 -> 기본값으로 채움
    globalSettings.stepLossComp = 0;
    commitSettingsToFlash();
  }

  // 4. 캐시 데이터 업데이트
  for (int i = 0; i < NUM_MOTORS; i++) {
    extraHomeOffsets[i] = globalSettings.globalOffsets[i];
    for (int j = 0; j < MAX_POSITIONS; j++) cachedSlotOffsets[i][j] = globalSettings.slotOffsets[i][j];
  }

  logMsg("SYSTEM_READY");
}

void loop() {
  checkSerialCommand();
  checkUSBInput();

  // [제거됨] AWAKE / REBOOT_COMPLETE / SET_COMPLETE 자발 송신 → 토큰 폴링으로 대체

// 완료 세대 갱신: 모터가 멎어 있으면 현재 이동 세대를 완료로 승격
if (statusMoveComplete) completedGen = myMoveGen;

  static bool prevMoveComplete = false;
  if (statusMoveComplete && !prevMoveComplete) {
    waitingForSetComplete = false;   // 플래그만 정리(송신 없음)

    if (isTestMode) {
      delay(1000);
      int target;
      do { target = random(2, TOTAL_PIECES + 1); } while (target == positions[0]);
      int targets[NUM_MOTORS];
      for (int i = 0; i < NUM_MOTORS; i++) targets[i] = -1;
      targets[0] = target;
      sendMoveBatchCmd(targets);
    }
  }
  prevMoveComplete = statusMoveComplete;

  if (simpleTestId != -1 && simpleTestCount > 0) {
    if (states[simpleTestId] == 100) {
      simpleTestCount--;
      if (simpleTestCount > 0) {
        logMsg("TEST_RUNNING: ID=" + String(simpleTestId) + ", Left=" + String(simpleTestCount));
        delay(200);
        sendRehomeSingleCmd(simpleTestId);
      } else {
        logMsg("TEST_COMPLETE: ID=" + String(simpleTestId));
        simpleTestId = -1;
      }
    }
  }
}