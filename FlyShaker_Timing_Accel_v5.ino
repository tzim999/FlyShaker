/*
  Fly Shaker ESP32 Firmware
  Version 5: Stable Timing Engine with Optional Three-Axis ADXL345 Measurement

  Hardware:
    Original ESP32 with built-in 8-bit DAC
    DAC output: GPIO25
    Communication LED: GPIO26 (blue, active HIGH)
    Stimulus LED: GPIO27 (red, active HIGH)
    ADXL345 SPI: CS GPIO5, SCK GPIO18, MISO GPIO19, MOSI GPIO23

  Implements:
    - Serial command parsing and validation
    - START, PAUSE, STOP, STATUS
    - DAC square-wave generation using FREQ, AMP, DUTY
    - BURST and BSILENCE timing
    - SEQ and SSILENCE timing
    - SESSION timing with automatic stop
    - REPORT(NONE), REPORT(BRIEF), REPORT(FULL)
    - Eight-dash response/event terminator
    - Announces each selected phase duration before that phase begins
    - Completes and flushes START reporting before enabling the DAC
    - Reports aborted active phases when STOP or session completion interrupts them
    - Blue communication LED on GPIO26 pulses for serial transactions
    - Red stimulus LED on GPIO27 is on during active bursts
    - Exactly one eight-dash terminator per command or asynchronous event
    - Millisecond-resolution duration reporting
    - Optional ADXL345 initialization and X/Y/Z burst measurement
    - Fixed ADXL345 full-resolution range of +/-16 g at 400 samples/second
    - STATUS reports ACCEL OK or ACCEL NOT FOUND
    - Missing accelerometer never prevents normal shaker operation
    - Three-line per-burst acceleration report for X, Y, and Z

  Notes:
    - Z is assumed to be perpendicular to the shaker plate.
    - X and Y reveal lateral coupling, rocking, and asymmetric resonances.
    - Acceleration values are reported in g, with units omitted by protocol design.
    - The ADXL345 remains in full-resolution mode at a fixed +/-16 g range.
    - If the ADXL345 is absent, no acceleration reports are produced.
*/

#include <Arduino.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <SPI.h>
#include "esp_timer.h"

const float MAX_AMP = 30.0f;   // Maximum allowed DAC amplitude

const uint32_t SERIAL_BAUD = 115200;
const uint8_t DAC_PIN = 25;
const uint8_t DAC_BASELINE = 127;
const uint8_t COMM_LED_PIN = 26;
const uint8_t STIM_LED_PIN = 27;
const uint32_t COMM_LED_PULSE_MS = 40;
const size_t INPUT_BUFFER_SIZE = 256;

const uint8_t ACCEL_CS_PIN = 5;
const uint8_t ACCEL_SCK_PIN = 18;
const uint8_t ACCEL_MISO_PIN = 19;
const uint8_t ACCEL_MOSI_PIN = 23;
const uint32_t ACCEL_SPI_HZ = 5000000;
const uint16_t ACCEL_SAMPLE_RATE_HZ = 400;
const uint32_t ACCEL_SAMPLE_INTERVAL_US = 1000000UL / ACCEL_SAMPLE_RATE_HZ;
const float ACCEL_G_PER_LSB = 0.0039f;

const uint8_t ADXL345_REG_DEVID = 0x00;
const uint8_t ADXL345_REG_BW_RATE = 0x2C;
const uint8_t ADXL345_REG_POWER_CTL = 0x2D;
const uint8_t ADXL345_REG_DATA_FORMAT = 0x31;
const uint8_t ADXL345_REG_DATAX0 = 0x32;
const uint8_t ADXL345_DEVICE_ID = 0xE5;

char inputBuffer[INPUT_BUFFER_SIZE];
size_t inputLength = 0;
bool discardUntilNewline = false;
bool responseEndAlreadyPrinted = false;

struct RangeValue {
  float minimum;
  float maximum;
  bool isSet;
};

enum ReportMode { REPORT_NONE, REPORT_BRIEF, REPORT_FULL };
enum SystemState { STATE_STOPPED, STATE_RUNNING, STATE_PAUSED };
enum SequencePhase { PHASE_SEQUENCE_ACTIVE, PHASE_SEQUENCE_SILENCE };
enum BurstPhase { PHASE_BURST_ACTIVE, PHASE_BURST_SILENCE };

struct ProtocolConfig {
  RangeValue frequency;
  RangeValue amplitude;
  RangeValue duty;
  RangeValue burst;
  RangeValue burstSilence;
  RangeValue sequence;
  RangeValue sequenceSilence;
  float session;
  bool sessionSet;
  ReportMode reportMode;
};

ProtocolConfig config;
SystemState systemState = STATE_STOPPED;
SequencePhase sequencePhase = PHASE_SEQUENCE_ACTIVE;
BurstPhase burstPhase = PHASE_BURST_ACTIVE;

float activeFrequencyHz = 0.0f;
float activeAmplitude = 0.0f;
float activeDutyPercent = 0.0f;
float activeBurstSeconds = 0.0f;
float activeBurstSilenceSeconds = 0.0f;
float activeSequenceSeconds = 0.0f;
float activeSequenceSilenceSeconds = 0.0f;
uint8_t activeDacValue = DAC_BASELINE;

uint32_t activePhaseDurationUs = 0;
uint32_t baselinePhaseDurationUs = 0;
uint32_t nextWaveformPhaseUs = 0;
bool waveformActivePhase = false;
bool waveformEnabled = false;
uint32_t communicationLedOffAtMs = 0;

bool accelerometerPresent = false;
bool accelerometerCollecting = false;
uint32_t nextAccelSampleUs = 0;
uint32_t accelSamples = 0;
double accelSum[3] = {0.0, 0.0, 0.0};
double accelSumSquares[3] = {0.0, 0.0, 0.0};
float accelMinimum[3] = {0.0f, 0.0f, 0.0f};
float accelMaximum[3] = {0.0f, 0.0f, 0.0f};
float accelZeroG[3] = {0.0f, 0.0f, 0.0f};

uint64_t sessionDeadlineUs = 0;
uint64_t sequenceDeadlineUs = 0;
uint64_t burstDeadlineUs = 0;
uint64_t pauseStartedUs = 0;

void initializeConfiguration();
void serviceSerial();
void processCommandLine(char* line);
bool processCommand(char* command);
bool processParameter(const char* name, char* argumentText);
bool processControlCommand(const char* command);
bool parseRange(char* text, float& minimum, float& maximum);
bool parseSingleValue(char* text, float& value);
bool parseReportMode(char* text, ReportMode& mode);
bool validateRangeParameter(const char* name, float minimum, float maximum, float allowedMinimum, float allowedMaximum);
bool validateConfiguration();
void startExperiment();
void resumeExperiment();
void pauseExperiment();
void stopExperiment(bool sessionComplete = false);
void printAbortMessages();
void serviceExperiment();
void beginSequenceActive(uint64_t now);
void beginSequenceSilence(uint64_t now);
void beginBurstActive(uint64_t now);
void beginBurstSilence(uint64_t now);
void startWaveform();
void stopWaveform();
void serviceWaveform();
void serviceLeds();
void pulseCommunicationLed();
bool initializeAccelerometer();
void configureAccelerometerRange(uint8_t rangeG);
uint8_t adxlReadRegister(uint8_t reg);
void adxlWriteRegister(uint8_t reg, uint8_t value);
bool readAccelRaw(int16_t& xRaw, int16_t& yRaw, int16_t& zRaw);
void readAccelG(float& xG, float& yG, float& zG);
void calibrateAccelZero();
void beginAccelCapture();
void serviceAccelerometer();
void finishAccelCapture(bool report);
void printAccelReport();
void selectWaveformValues();
void setDacBaseline();
float randomFloat(float minimum, float maximum);
uint64_t secondsToUs(float seconds);
uint64_t nowUs();
void printStatus();
void printError(const char* code, const char* detail = nullptr);
void printAcknowledgment(const char* command);
void printState();
void printWaveformSelection();
void printTimingSelection();
void printEvent(const char* eventText);
void printResponseEnd();
char* trimWhitespace(char* text);
void uppercaseInPlace(char* text);
const char* reportModeName(ReportMode mode);
const char* stateName(SystemState state);
const char* sequencePhaseName(SequencePhase phase);
const char* burstPhaseName(BurstPhase phase);
void printNumber(float value);
void printDuration(float seconds);
void printRemaining(const char* label, uint64_t deadline, uint64_t now);

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(COMM_LED_PIN, OUTPUT);
  pinMode(STIM_LED_PIN, OUTPUT);
  digitalWrite(COMM_LED_PIN, LOW);
  digitalWrite(STIM_LED_PIN, LOW);
  initializeConfiguration();
  SPI.begin(ACCEL_SCK_PIN, ACCEL_MISO_PIN, ACCEL_MOSI_PIN, ACCEL_CS_PIN);
  pinMode(ACCEL_CS_PIN, OUTPUT);
  digitalWrite(ACCEL_CS_PIN, HIGH);
  accelerometerPresent = initializeAccelerometer();
  setDacBaseline();
  randomSeed(esp_random());
  delay(250);
  Serial.println("READY FLY_SHAKER_TIMING_ACCEL_V4_2");
  printResponseEnd();
}

void loop() {
  serviceSerial();
  serviceExperiment();
  serviceWaveform();
  serviceAccelerometer();
  serviceLeds();
}

void initializeConfiguration() {
  config.frequency = {0.0f, 0.0f, false};
  config.amplitude = {0.0f, 0.0f, false};
  config.duty = {0.0f, 0.0f, false};
  config.burst = {0.0f, 0.0f, false};
  config.burstSilence = {0.0f, 0.0f, false};
  config.sequence = {0.0f, 0.0f, false};
  config.sequenceSilence = {0.0f, 0.0f, false};
  config.session = 0.0f;
  config.sessionSet = false;
  config.reportMode = REPORT_FULL;
}

void serviceSerial() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());

    if (discardUntilNewline) {
      if (c == '\n' || c == '\r') {
        discardUntilNewline = false;
        inputLength = 0;
      }
      continue;
    }

    if (c == '\n' || c == '\r') {
      if (inputLength > 0) {
        inputBuffer[inputLength] = '\0';
        pulseCommunicationLed();
        processCommandLine(inputBuffer);
        inputLength = 0;
      }
      continue;
    }

    if (inputLength < INPUT_BUFFER_SIZE - 1) {
      inputBuffer[inputLength++] = c;
    } else {
      inputLength = 0;
      discardUntilNewline = true;
      printError("INPUT_TOO_LONG");
      printResponseEnd();
    }
  }
}

void processCommandLine(char* line) {
  char* cursor = trimWhitespace(line);

  while (*cursor != '\0') {
    while (isspace(static_cast<unsigned char>(*cursor))) cursor++;
    if (*cursor == '\0') break;

    char* commandStart = cursor;
    int parenthesisDepth = 0;

    while (*cursor != '\0') {
      if (*cursor == '(') parenthesisDepth++;
      else if (*cursor == ')') {
        parenthesisDepth--;
        if (parenthesisDepth < 0) {
          printError("SYNTAX", "UNEXPECTED_RIGHT_PARENTHESIS");
          printResponseEnd();
          return;
        }
      } else if (isspace(static_cast<unsigned char>(*cursor)) && parenthesisDepth == 0) {
        break;
      }
      cursor++;
    }

    if (parenthesisDepth != 0) {
      printError("SYNTAX", "UNMATCHED_PARENTHESIS");
      printResponseEnd();
      return;
    }

    char savedCharacter = *cursor;
    *cursor = '\0';
    char* command = trimWhitespace(commandStart);
    if (*command != '\0') {
      responseEndAlreadyPrinted = false;
      processCommand(command);
      if (!responseEndAlreadyPrinted) printResponseEnd();
    }

    if (savedCharacter == '\0') break;
    *cursor = savedCharacter;
    cursor++;
  }
}

bool processCommand(char* command) {
  command = trimWhitespace(command);
  if (*command == '\0') return true;

  char* openParenthesis = strchr(command, '(');
  if (openParenthesis == nullptr) {
    uppercaseInPlace(command);
    if (strcmp(command, "STATUS") == 0) {
      printStatus();
      return true;
    }
    if (processControlCommand(command)) return true;
    printError("UNKNOWN_COMMAND", command);
    return false;
  }

  char* closeParenthesis = strrchr(command, ')');
  if (closeParenthesis == nullptr || closeParenthesis < openParenthesis) {
    printError("SYNTAX", "MISSING_RIGHT_PARENTHESIS");
    return false;
  }

  char* trailing = trimWhitespace(closeParenthesis + 1);
  if (*trailing != '\0') {
    printError("SYNTAX", "TEXT_AFTER_RIGHT_PARENTHESIS");
    return false;
  }

  *openParenthesis = '\0';
  *closeParenthesis = '\0';
  char* name = trimWhitespace(command);
  char* argumentText = trimWhitespace(openParenthesis + 1);
  uppercaseInPlace(name);

  if (*name == '\0') {
    printError("SYNTAX", "MISSING_PARAMETER_NAME");
    return false;
  }
  return processParameter(name, argumentText);
}

bool processControlCommand(const char* command) {
  if (strcmp(command, "START") == 0) {
    if (!validateConfiguration()) {
      printError("CANNOT_START", "CONFIG_INCOMPLETE");
      return true;
    }
    if (systemState == STATE_PAUSED) resumeExperiment();
    else startExperiment();
    return true;
  }

  if (strcmp(command, "PAUSE") == 0) {
    pauseExperiment();
    return true;
  }

  if (strcmp(command, "STOP") == 0) {
    stopExperiment(false);
    return true;
  }
  return false;
}

bool processParameter(const char* name, char* argumentText) {
  float minimum = 0.0f, maximum = 0.0f, value = 0.0f;

  if (strcmp(name, "FREQ") == 0) {
    if (!parseRange(argumentText, minimum, maximum)) { printError("BAD_ARGUMENTS", "FREQ_REQUIRES_MIN_MAX"); return false; }
    if (!validateRangeParameter("FREQ", minimum, maximum, 1.0f, 500.0f)) return false;
    config.frequency = {minimum, maximum, true}; printAcknowledgment("FREQ"); return true;
  }
  if (strcmp(name, "AMP") == 0) {
    if (!parseRange(argumentText, minimum, maximum)) { printError("BAD_ARGUMENTS", "AMP_REQUIRES_MIN_MAX"); return false; }
    if (!validateRangeParameter("AMP", minimum, maximum, -127.0f, 127.0f)) return false;
    config.amplitude = {minimum, maximum, true}; printAcknowledgment("AMP"); return true;
  }
  if (strcmp(name, "DUTY") == 0) {
    if (!parseRange(argumentText, minimum, maximum)) { printError("BAD_ARGUMENTS", "DUTY_REQUIRES_MIN_MAX"); return false; }
    if (!validateRangeParameter("DUTY", minimum, maximum, 1.0f, 99.0f)) return false;
    config.duty = {minimum, maximum, true}; printAcknowledgment("DUTY"); return true;
  }
  if (strcmp(name, "BURST") == 0) {
    if (!parseRange(argumentText, minimum, maximum)) { printError("BAD_ARGUMENTS", "BURST_REQUIRES_MIN_MAX"); return false; }
    if (!validateRangeParameter("BURST", minimum, maximum, 0.010f, 3600.0f)) return false;
    config.burst = {minimum, maximum, true}; printAcknowledgment("BURST"); return true;
  }
  if (strcmp(name, "BSILENCE") == 0) {
    if (!parseRange(argumentText, minimum, maximum)) { printError("BAD_ARGUMENTS", "BSILENCE_REQUIRES_MIN_MAX"); return false; }
    if (!validateRangeParameter("BSILENCE", minimum, maximum, 0.0f, 86400.0f)) return false;
    config.burstSilence = {minimum, maximum, true}; printAcknowledgment("BSILENCE"); return true;
  }
  if (strcmp(name, "SEQ") == 0) {
    if (!parseRange(argumentText, minimum, maximum)) { printError("BAD_ARGUMENTS", "SEQ_REQUIRES_MIN_MAX"); return false; }
    if (!validateRangeParameter("SEQ", minimum, maximum, 0.010f, 604800.0f)) return false;
    config.sequence = {minimum, maximum, true}; printAcknowledgment("SEQ"); return true;
  }
  if (strcmp(name, "SSILENCE") == 0) {
    if (!parseRange(argumentText, minimum, maximum)) { printError("BAD_ARGUMENTS", "SSILENCE_REQUIRES_MIN_MAX"); return false; }
    if (!validateRangeParameter("SSILENCE", minimum, maximum, 0.0f, 604800.0f)) return false;
    config.sequenceSilence = {minimum, maximum, true}; printAcknowledgment("SSILENCE"); return true;
  }
  if (strcmp(name, "SESSION") == 0) {
    if (!parseSingleValue(argumentText, value)) { printError("BAD_ARGUMENTS", "SESSION_REQUIRES_ONE_VALUE"); return false; }
    if (!isfinite(value) || value < 0.010f || value > 2592000.0f) {
      printError("OUT_OF_RANGE", "SESSION_ALLOWED_0.010_TO_2592000"); return false;
    }
    config.session = value; config.sessionSet = true; printAcknowledgment("SESSION"); return true;
  }
  if (strcmp(name, "REPORT") == 0) {
    ReportMode newMode;
    if (!parseReportMode(argumentText, newMode)) { printError("BAD_ARGUMENTS", "REPORT_REQUIRES_NONE_BRIEF_OR_FULL"); return false; }
    config.reportMode = newMode; printAcknowledgment("REPORT"); return true;
  }

  printError("UNKNOWN_PARAMETER", name);
  return false;
}

bool parseRange(char* text, float& minimum, float& maximum) {
  char* comma = strchr(text, ',');
  if (comma == nullptr || strchr(comma + 1, ',') != nullptr) return false;
  *comma = '\0';
  char* minimumText = trimWhitespace(text);
  char* maximumText = trimWhitespace(comma + 1);
  if (*minimumText == '\0' || *maximumText == '\0') return false;

  char* endPointer = nullptr;
  minimum = strtof(minimumText, &endPointer);
  if (endPointer == minimumText) return false;
  endPointer = trimWhitespace(endPointer);
  if (*endPointer != '\0') return false;

  maximum = strtof(maximumText, &endPointer);
  if (endPointer == maximumText) return false;
  endPointer = trimWhitespace(endPointer);
  if (*endPointer != '\0') return false;
  return isfinite(minimum) && isfinite(maximum);
}

bool parseSingleValue(char* text, float& value) {
  text = trimWhitespace(text);
  if (*text == '\0' || strchr(text, ',') != nullptr) return false;
  char* endPointer = nullptr;
  value = strtof(text, &endPointer);
  if (endPointer == text) return false;
  endPointer = trimWhitespace(endPointer);
  return *endPointer == '\0' && isfinite(value);
}

bool parseReportMode(char* text, ReportMode& mode) {
  text = trimWhitespace(text);
  uppercaseInPlace(text);
  if (strcmp(text, "NONE") == 0) { mode = REPORT_NONE; return true; }
  if (strcmp(text, "BRIEF") == 0) { mode = REPORT_BRIEF; return true; }
  if (strcmp(text, "FULL") == 0) { mode = REPORT_FULL; return true; }
  return false;
}

bool validateRangeParameter(const char* name, float minimum, float maximum, float allowedMinimum, float allowedMaximum) {
  if (!isfinite(minimum) || !isfinite(maximum)) { printError("INVALID_NUMBER", name); return false; }
  if (minimum > maximum) { printError("MIN_GREATER_THAN_MAX", name); return false; }
  if (minimum < allowedMinimum || maximum > allowedMaximum) {
    Serial.print("ERROR OUT_OF_RANGE "); Serial.print(name); Serial.print(" ALLOWED ");
    printNumber(allowedMinimum); Serial.print(" TO "); printNumber(allowedMaximum); Serial.println();
    return false;
  }
  return true;
}

bool validateConfiguration() {
  return config.frequency.isSet && config.amplitude.isSet && config.duty.isSet &&
         config.burst.isSet && config.burstSilence.isSet && config.sequence.isSet &&
         config.sequenceSilence.isSet && config.sessionSet;
}

void startExperiment() {
  uint64_t now = nowUs();

  systemState = STATE_RUNNING;
  sequencePhase = PHASE_SEQUENCE_ACTIVE;
  burstPhase = PHASE_BURST_ACTIVE;

  activeSequenceSeconds = randomFloat(config.sequence.minimum, config.sequence.maximum);
  activeBurstSeconds = randomFloat(config.burst.minimum, config.burst.maximum);
  selectWaveformValues();

  sessionDeadlineUs = now + secondsToUs(config.session);
  sequenceDeadlineUs = now + secondsToUs(activeSequenceSeconds);
  burstDeadlineUs = now + secondsToUs(activeBurstSeconds);
  if (burstDeadlineUs > sequenceDeadlineUs) {
    burstDeadlineUs = sequenceDeadlineUs;
    activeBurstSeconds = static_cast<float>(burstDeadlineUs - now) / 1000000.0f;
  }

  if (config.reportMode != REPORT_NONE) {
    printAcknowledgment("START");
    printState();
  }
  if (config.reportMode == REPORT_FULL) {
    Serial.print("SESSION DURATION "); printDuration(config.session); Serial.println();
    Serial.println("SEQUENCE ACTIVE START");
    Serial.print("SEQUENCE ACTIVE DURATION "); printDuration(activeSequenceSeconds); Serial.println();
    Serial.println("BURST START");
    Serial.print("BURST DURATION "); printDuration(activeBurstSeconds); Serial.println();
    printWaveformSelection();
  }
  if (config.reportMode != REPORT_NONE) {
    printResponseEnd();
  }

  // All START text is transmitted before acceleration capture and GPIO25 switching.
  beginAccelCapture();
  startWaveform();
}

void resumeExperiment() {
  uint64_t now = nowUs();
  uint64_t pausedDuration = now - pauseStartedUs;
  sessionDeadlineUs += pausedDuration;
  sequenceDeadlineUs += pausedDuration;
  burstDeadlineUs += pausedDuration;
  systemState = STATE_RUNNING;

  printAcknowledgment("START");
  if (config.reportMode != REPORT_NONE) {
    printState();
    if (config.reportMode == REPORT_FULL) {
      Serial.println("SESSION RESUMED");
      printRemaining("SESSION REMAINING", sessionDeadlineUs, now);
      printRemaining("SEQUENCE PHASE REMAINING", sequenceDeadlineUs, now);
      if (sequencePhase == PHASE_SEQUENCE_ACTIVE) {
        printRemaining("BURST PHASE REMAINING", burstDeadlineUs, now);
      }
    }
    printResponseEnd();
  }

  if (sequencePhase == PHASE_SEQUENCE_ACTIVE && burstPhase == PHASE_BURST_ACTIVE) {
    beginAccelCapture();
    startWaveform();
  } else stopWaveform();
}

void pauseExperiment() {
  if (systemState == STATE_RUNNING) {
    pauseStartedUs = nowUs();
    systemState = STATE_PAUSED;
    stopWaveform();
    finishAccelCapture(false);
  }
  printAcknowledgment("PAUSE");
  if (config.reportMode != REPORT_NONE) printState();
}

void stopExperiment(bool sessionComplete) {
  bool wasActive = (systemState == STATE_RUNNING || systemState == STATE_PAUSED);
  stopWaveform();
  finishAccelCapture(false);

  if (config.reportMode != REPORT_NONE) {
    if (!sessionComplete) printAcknowledgment("STOP");
    if (wasActive) printAbortMessages();
    if (sessionComplete) Serial.println("SESSION COMPLETE");
  }

  systemState = STATE_STOPPED;
  if (config.reportMode != REPORT_NONE) {
    printState();
    if (sessionComplete) printResponseEnd();
  }
}

void printAbortMessages() {
  if (sequencePhase == PHASE_SEQUENCE_ACTIVE) {
    if (burstPhase == PHASE_BURST_ACTIVE) Serial.println("BURST ABORTED");
    Serial.println("SEQUENCE ACTIVE ABORTED");
  } else {
    Serial.println("SEQUENCE SILENCE ABORTED");
  }
}

void serviceExperiment() {
  if (systemState != STATE_RUNNING) return;
  uint64_t now = nowUs();

  if (now >= sessionDeadlineUs) {
    stopExperiment(true);
    return;
  }

  if (now >= sequenceDeadlineUs) {
    if (sequencePhase == PHASE_SEQUENCE_ACTIVE) beginSequenceSilence(now);
    else beginSequenceActive(now);
    return;
  }

  if (sequencePhase == PHASE_SEQUENCE_ACTIVE && now >= burstDeadlineUs) {
    if (burstPhase == PHASE_BURST_ACTIVE) beginBurstSilence(now);
    else beginBurstActive(now);
  }
}

void beginSequenceActive(uint64_t now) {
  sequencePhase = PHASE_SEQUENCE_ACTIVE;
  activeSequenceSeconds = randomFloat(config.sequence.minimum, config.sequence.maximum);
  sequenceDeadlineUs = now + secondsToUs(activeSequenceSeconds);

  if (config.reportMode == REPORT_FULL) {
    Serial.println("SEQUENCE ACTIVE START");
    Serial.print("SEQUENCE ACTIVE DURATION "); printDuration(activeSequenceSeconds); Serial.println();
    printResponseEnd();
  }

  beginBurstActive(now);
}

void beginSequenceSilence(uint64_t now) {
  sequencePhase = PHASE_SEQUENCE_SILENCE;
  activeSequenceSilenceSeconds = randomFloat(config.sequenceSilence.minimum, config.sequenceSilence.maximum);
  sequenceDeadlineUs = now + secondsToUs(activeSequenceSilenceSeconds);
  stopWaveform();
  finishAccelCapture(config.reportMode == REPORT_FULL);

  if (config.reportMode == REPORT_FULL) {
    Serial.println("SEQUENCE SILENCE START");
    Serial.print("SEQUENCE SILENCE DURATION "); printDuration(activeSequenceSilenceSeconds); Serial.println();
    printResponseEnd();
  }
}

void beginBurstActive(uint64_t now) {
  burstPhase = PHASE_BURST_ACTIVE;
  activeBurstSeconds = randomFloat(config.burst.minimum, config.burst.maximum);
  burstDeadlineUs = now + secondsToUs(activeBurstSeconds);
  if (burstDeadlineUs > sequenceDeadlineUs) {
    burstDeadlineUs = sequenceDeadlineUs;
    activeBurstSeconds = static_cast<float>(burstDeadlineUs - now) / 1000000.0f;
  }
  selectWaveformValues();

  if (config.reportMode == REPORT_FULL) {
    Serial.println("BURST START");
    Serial.print("BURST DURATION "); printDuration(activeBurstSeconds); Serial.println();
    printWaveformSelection();
    printResponseEnd();
  }

  // Capture begins immediately before GPIO25 starts switching.
  beginAccelCapture();
  startWaveform();
}

void beginBurstSilence(uint64_t now) {
  burstPhase = PHASE_BURST_SILENCE;
  activeBurstSilenceSeconds = randomFloat(config.burstSilence.minimum, config.burstSilence.maximum);
  burstDeadlineUs = now + secondsToUs(activeBurstSilenceSeconds);
  if (burstDeadlineUs > sequenceDeadlineUs) {
    burstDeadlineUs = sequenceDeadlineUs;
    activeBurstSilenceSeconds = static_cast<float>(burstDeadlineUs - now) / 1000000.0f;
  }
  stopWaveform();
  finishAccelCapture(config.reportMode == REPORT_FULL);

  if (config.reportMode == REPORT_FULL) {
    Serial.println("BURST SILENCE START");
    Serial.print("BURST SILENCE DURATION "); printDuration(activeBurstSilenceSeconds); Serial.println();
    printResponseEnd();
  }
}

void startWaveform() {
  waveformEnabled = true;
  digitalWrite(STIM_LED_PIN, HIGH);
  waveformActivePhase = true;
  dacWrite(DAC_PIN, activeDacValue);
  nextWaveformPhaseUs = micros() + activePhaseDurationUs;
}

void stopWaveform() {
  waveformEnabled = false;
  digitalWrite(STIM_LED_PIN, LOW);
  setDacBaseline();
}

void serviceWaveform() {
  if (!waveformEnabled || systemState != STATE_RUNNING) return;
  uint32_t now = micros();
  if (static_cast<int32_t>(now - nextWaveformPhaseUs) < 0) return;

  if (waveformActivePhase) {
    waveformActivePhase = false;
    dacWrite(DAC_PIN, DAC_BASELINE);
    nextWaveformPhaseUs += baselinePhaseDurationUs;
  } else {
    waveformActivePhase = true;
    dacWrite(DAC_PIN, activeDacValue);
    nextWaveformPhaseUs += activePhaseDurationUs;
  }

  uint32_t fullPeriodUs = activePhaseDurationUs + baselinePhaseDurationUs;
  if (static_cast<int32_t>(now - nextWaveformPhaseUs) >= static_cast<int32_t>(fullPeriodUs)) {
    nextWaveformPhaseUs = now + (waveformActivePhase ? activePhaseDurationUs : baselinePhaseDurationUs);
  }
}

void selectWaveformValues() {
  activeFrequencyHz = randomFloat(config.frequency.minimum, config.frequency.maximum);
  activeAmplitude = randomFloat(config.amplitude.minimum, config.amplitude.maximum);
  activeDutyPercent = randomFloat(config.duty.minimum, config.duty.maximum);

  int dacValue = static_cast<int>(roundf(static_cast<float>(DAC_BASELINE) + activeAmplitude));
  activeDacValue = static_cast<uint8_t>(constrain(dacValue, 0, 255));
  float periodUs = 1000000.0f / activeFrequencyHz;
  activePhaseDurationUs = static_cast<uint32_t>(roundf(periodUs * activeDutyPercent / 100.0f));
  baselinePhaseDurationUs = static_cast<uint32_t>(roundf(periodUs - activePhaseDurationUs));
  if (activePhaseDurationUs < 1) activePhaseDurationUs = 1;
  if (baselinePhaseDurationUs < 1) baselinePhaseDurationUs = 1;
}

void setDacBaseline() {
  dacWrite(DAC_PIN, DAC_BASELINE);
  waveformActivePhase = false;
}

float randomFloat(float minimum, float maximum) {
  if (minimum == maximum) return minimum;
  uint32_t randomValue = esp_random();
  float fraction = static_cast<float>(randomValue) / 4294967295.0f;
  return minimum + fraction * (maximum - minimum);
}

uint64_t secondsToUs(float seconds) {
  return static_cast<uint64_t>(llround(static_cast<double>(seconds) * 1000000.0));
}

uint64_t nowUs() {
  return static_cast<uint64_t>(esp_timer_get_time());
}

void printStatus() {
  uint64_t now = nowUs();
  Serial.println("STATUS BEGIN");
  Serial.print("STATE "); Serial.println(stateName(systemState));

  if (validateConfiguration()) Serial.println("CONFIG COMPLETE");
  else {
    Serial.println("CONFIG INCOMPLETE");
    Serial.print("MISSING");
    if (!config.frequency.isSet) Serial.print(" FREQ");
    if (!config.amplitude.isSet) Serial.print(" AMP");
    if (!config.duty.isSet) Serial.print(" DUTY");
    if (!config.burst.isSet) Serial.print(" BURST");
    if (!config.burstSilence.isSet) Serial.print(" BSILENCE");
    if (!config.sequence.isSet) Serial.print(" SEQ");
    if (!config.sequenceSilence.isSet) Serial.print(" SSILENCE");
    if (!config.sessionSet) Serial.print(" SESSION");
    Serial.println();
  }

  auto printRange = [](const char* name, const RangeValue& value) {
    Serial.print(name); Serial.print(" ");
    if (value.isSet) {
      Serial.print("("); printNumber(value.minimum); Serial.print(","); printNumber(value.maximum); Serial.println(")");
    } else Serial.println("NOT_SET");
  };

  printRange("FREQ", config.frequency);
  printRange("AMP", config.amplitude);
  printRange("DUTY", config.duty);
  printRange("BURST", config.burst);
  printRange("BSILENCE", config.burstSilence);
  printRange("SEQ", config.sequence);
  printRange("SSILENCE", config.sequenceSilence);

  Serial.print("SESSION ");
  if (config.sessionSet) { Serial.print("("); printNumber(config.session); Serial.println(")"); }
  else Serial.println("NOT_SET");

  Serial.print("REPORT ("); Serial.print(reportModeName(config.reportMode)); Serial.println(")");
  Serial.print("ACCEL "); Serial.println(accelerometerPresent ? "OK" : "NOT FOUND");
  Serial.print("DAC_PIN "); Serial.println(DAC_PIN);

  if (systemState != STATE_STOPPED) {
    Serial.print("SEQUENCE_PHASE "); Serial.println(sequencePhaseName(sequencePhase));
    if (sequencePhase == PHASE_SEQUENCE_ACTIVE) {
      Serial.print("BURST_PHASE "); Serial.println(burstPhaseName(burstPhase));
    }
    uint64_t effectiveNow = (systemState == STATE_PAUSED) ? pauseStartedUs : now;
    printRemaining("SESSION_REMAINING", sessionDeadlineUs, effectiveNow);
    printRemaining("SEQUENCE_PHASE_REMAINING", sequenceDeadlineUs, effectiveNow);
    if (sequencePhase == PHASE_SEQUENCE_ACTIVE) printRemaining("BURST_PHASE_REMAINING", burstDeadlineUs, effectiveNow);
    if (waveformEnabled) printWaveformSelection();
  }

  Serial.println("STATUS END");
}

void printError(const char* code, const char* detail) {
  Serial.print("ERROR "); Serial.print(code);
  if (detail != nullptr && *detail != '\0') { Serial.print(" "); Serial.print(detail); }
  Serial.println();
}

void printAcknowledgment(const char* command) {
  if (config.reportMode == REPORT_NONE) return;
  Serial.print("OK "); Serial.println(command);
}

void printState() {
  Serial.print("STATE "); Serial.println(stateName(systemState));
}

void printWaveformSelection() {
  Serial.print("FREQ "); printNumber(activeFrequencyHz);
  Serial.print(" AMP "); printNumber(activeAmplitude);
  Serial.print(" DUTY "); printNumber(activeDutyPercent);
  Serial.println();
}

void printTimingSelection() {
  Serial.print("SESSION DURATION "); printDuration(config.session); Serial.println();
  Serial.print("SEQUENCE ACTIVE DURATION "); printDuration(activeSequenceSeconds); Serial.println();
}

void printEvent(const char* eventText) {
  Serial.println(eventText);
  printResponseEnd();
}

void printResponseEnd() {
  pulseCommunicationLed();
  Serial.println("--------");
  Serial.flush();
  responseEndAlreadyPrinted = true;
}

void printRemaining(const char* label, uint64_t deadline, uint64_t now) {
  double remaining = deadline > now ? static_cast<double>(deadline - now) / 1000000.0 : 0.0;
  Serial.print(label); Serial.print(" "); printDuration(static_cast<float>(remaining)); Serial.println();
}

char* trimWhitespace(char* text) {
  while (isspace(static_cast<unsigned char>(*text))) text++;
  if (*text == '\0') return text;
  char* end = text + strlen(text) - 1;
  while (end >= text && isspace(static_cast<unsigned char>(*end))) { *end = '\0'; end--; }
  return text;
}

void uppercaseInPlace(char* text) {
  while (*text != '\0') {
    *text = static_cast<char>(toupper(static_cast<unsigned char>(*text)));
    text++;
  }
}

const char* reportModeName(ReportMode mode) {
  switch (mode) {
    case REPORT_NONE: return "NONE";
    case REPORT_BRIEF: return "BRIEF";
    case REPORT_FULL: return "FULL";
    default: return "UNKNOWN";
  }
}

const char* stateName(SystemState state) {
  switch (state) {
    case STATE_STOPPED: return "STOPPED";
    case STATE_RUNNING: return "RUNNING";
    case STATE_PAUSED: return "PAUSED";
    default: return "UNKNOWN";
  }
}

const char* sequencePhaseName(SequencePhase phase) {
  return phase == PHASE_SEQUENCE_ACTIVE ? "ACTIVE" : "SILENCE";
}

const char* burstPhaseName(BurstPhase phase) {
  return phase == PHASE_BURST_ACTIVE ? "ACTIVE" : "SILENCE";
}

void serviceLeds() {
  if (communicationLedOffAtMs != 0 && static_cast<int32_t>(millis() - communicationLedOffAtMs) >= 0) {
    digitalWrite(COMM_LED_PIN, LOW);
    communicationLedOffAtMs = 0;
  }
}

void pulseCommunicationLed() {
  digitalWrite(COMM_LED_PIN, HIGH);
  communicationLedOffAtMs = millis() + COMM_LED_PULSE_MS;
}


bool initializeAccelerometer() {
  delay(10);
  if (adxlReadRegister(ADXL345_REG_DEVID) != ADXL345_DEVICE_ID) return false;

  adxlWriteRegister(ADXL345_REG_POWER_CTL, 0x00);
  adxlWriteRegister(ADXL345_REG_BW_RATE, 0x0C);  // 400 Hz output data rate.
  configureAccelerometerRange(16);
  adxlWriteRegister(ADXL345_REG_POWER_CTL, 0x08);  // Measurement mode.
  delay(20);
  calibrateAccelZero();
  return true;
}

void configureAccelerometerRange(uint8_t rangeG) {
  uint8_t rangeBits = 0x03;
  if (rangeG == 2) rangeBits = 0x00;
  else if (rangeG == 4) rangeBits = 0x01;
  else if (rangeG == 8) rangeBits = 0x02;
  adxlWriteRegister(ADXL345_REG_DATA_FORMAT, 0x08 | rangeBits);  // FULL_RES plus range.
}

uint8_t adxlReadRegister(uint8_t reg) {
  SPI.beginTransaction(SPISettings(ACCEL_SPI_HZ, MSBFIRST, SPI_MODE3));
  digitalWrite(ACCEL_CS_PIN, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(ACCEL_CS_PIN, HIGH);
  SPI.endTransaction();
  return value;
}

void adxlWriteRegister(uint8_t reg, uint8_t value) {
  SPI.beginTransaction(SPISettings(ACCEL_SPI_HZ, MSBFIRST, SPI_MODE3));
  digitalWrite(ACCEL_CS_PIN, LOW);
  SPI.transfer(reg & 0x3F);
  SPI.transfer(value);
  digitalWrite(ACCEL_CS_PIN, HIGH);
  SPI.endTransaction();
}

bool readAccelRaw(int16_t& xRaw, int16_t& yRaw, int16_t& zRaw) {
  uint8_t data[6];
  SPI.beginTransaction(SPISettings(ACCEL_SPI_HZ, MSBFIRST, SPI_MODE3));
  digitalWrite(ACCEL_CS_PIN, LOW);
  SPI.transfer(ADXL345_REG_DATAX0 | 0xC0);  // Read plus multi-byte.
  for (uint8_t i = 0; i < 6; ++i) data[i] = SPI.transfer(0x00);
  digitalWrite(ACCEL_CS_PIN, HIGH);
  SPI.endTransaction();

  xRaw = static_cast<int16_t>((static_cast<uint16_t>(data[1]) << 8) | data[0]);
  yRaw = static_cast<int16_t>((static_cast<uint16_t>(data[3]) << 8) | data[2]);
  zRaw = static_cast<int16_t>((static_cast<uint16_t>(data[5]) << 8) | data[4]);
  return true;
}

void readAccelG(float& xG, float& yG, float& zG) {
  int16_t xRaw = 0;
  int16_t yRaw = 0;
  int16_t zRaw = 0;
  if (!readAccelRaw(xRaw, yRaw, zRaw)) {
    xG = 0.0f;
    yG = 0.0f;
    zG = 0.0f;
    return;
  }
  xG = static_cast<float>(xRaw) * ACCEL_G_PER_LSB;
  yG = static_cast<float>(yRaw) * ACCEL_G_PER_LSB;
  zG = static_cast<float>(zRaw) * ACCEL_G_PER_LSB;
}

void calibrateAccelZero() {
  const uint16_t calibrationSamples = 32;
  double sums[3] = {0.0, 0.0, 0.0};
  for (uint16_t i = 0; i < calibrationSamples; ++i) {
    float values[3];
    readAccelG(values[0], values[1], values[2]);
    for (uint8_t axis = 0; axis < 3; ++axis) sums[axis] += values[axis];
    delayMicroseconds(2500);
  }
  for (uint8_t axis = 0; axis < 3; ++axis) {
    accelZeroG[axis] = static_cast<float>(sums[axis] / calibrationSamples);
  }
}

void beginAccelCapture() {
  accelSamples = 0;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    accelSum[axis] = 0.0;
    accelSumSquares[axis] = 0.0;
    accelMinimum[axis] = 0.0f;
    accelMaximum[axis] = 0.0f;
  }
  accelerometerCollecting = accelerometerPresent;
  nextAccelSampleUs = micros();
}

void serviceAccelerometer() {
  if (!accelerometerCollecting || systemState != STATE_RUNNING) return;
  uint32_t now = micros();
  if (static_cast<int32_t>(now - nextAccelSampleUs) < 0) return;

  float values[3];
  readAccelG(values[0], values[1], values[2]);
  for (uint8_t axis = 0; axis < 3; ++axis) {
    values[axis] -= accelZeroG[axis];
    if (accelSamples == 0) {
      accelMinimum[axis] = values[axis];
      accelMaximum[axis] = values[axis];
    } else {
      if (values[axis] < accelMinimum[axis]) accelMinimum[axis] = values[axis];
      if (values[axis] > accelMaximum[axis]) accelMaximum[axis] = values[axis];
    }
    accelSum[axis] += values[axis];
    accelSumSquares[axis] += static_cast<double>(values[axis]) * values[axis];
  }
  accelSamples++;

  nextAccelSampleUs += ACCEL_SAMPLE_INTERVAL_US;
  if (static_cast<int32_t>(now - nextAccelSampleUs) >= 0) {
    nextAccelSampleUs = now + ACCEL_SAMPLE_INTERVAL_US;
  }
}

void finishAccelCapture(bool report) {
  if (!accelerometerCollecting) return;
  accelerometerCollecting = false;
  if (report) {
    printAccelReport();
    printResponseEnd();
  }
}

void printAccelReport() {
  const char axisNames[3] = {'X', 'Y', 'Z'};
  for (uint8_t axis = 0; axis < 3; ++axis) {
    Serial.print("ACCEL SAMPLES "); Serial.print(accelSamples);
    Serial.print(" "); Serial.print(axisNames[axis]); Serial.print("_MEAN ");
    if (accelSamples == 0) {
      Serial.print("0.000 "); Serial.print(axisNames[axis]); Serial.print("_MIN 0.000 ");
      Serial.print(axisNames[axis]); Serial.print("_MAX 0.000 ");
      Serial.print(axisNames[axis]); Serial.print("_P2P 0.000 ");
      Serial.print(axisNames[axis]); Serial.println("_RMS 0.000");
      continue;
    }

    float mean = static_cast<float>(accelSum[axis] / accelSamples);
    float rms = sqrtf(static_cast<float>(accelSumSquares[axis] / accelSamples));
    float p2p = accelMaximum[axis] - accelMinimum[axis];
    Serial.print(mean, 3);
    Serial.print(" "); Serial.print(axisNames[axis]); Serial.print("_MIN "); Serial.print(accelMinimum[axis], 3);
    Serial.print(" "); Serial.print(axisNames[axis]); Serial.print("_MAX "); Serial.print(accelMaximum[axis], 3);
    Serial.print(" "); Serial.print(axisNames[axis]); Serial.print("_P2P "); Serial.print(p2p, 3);
    Serial.print(" "); Serial.print(axisNames[axis]); Serial.print("_RMS "); Serial.println(rms, 3);
  }
}

void printDuration(float seconds) {
  Serial.print(seconds, 3);
}

void printNumber(float value) {
  float rounded = roundf(value);
  if (fabsf(value - rounded) < 0.000001f) {
    Serial.print(static_cast<long>(rounded));
    return;
  }
  char buffer[24];
  snprintf(buffer, sizeof(buffer), "%.3f", value);
  char* end = buffer + strlen(buffer) - 1;
  while (end > buffer && *end == '0') { *end = '\0'; end--; }
  if (end > buffer && *end == '.') *end = '\0';
  Serial.print(buffer);
}
