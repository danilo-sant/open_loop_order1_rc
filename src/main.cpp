#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Settings
static const TickType_t samplingInterval = 50 / portTICK_PERIOD_MS; 
static const TickType_t timeToStartInterval = 4000 / portTICK_PERIOD_MS; 

static const uint8_t PV_PIN = 14;
static const uint8_t MV_PIN = 26;
volatile float VCC = 0.0;
volatile uint16_t sensorReadingInt;
volatile float sensorReadingVoltage;
// Globals
static TimerHandle_t getSensorReadingTimer = NULL;
static TimerHandle_t stepInputStartTimer = NULL;

//static TaskHandle_t taskZeroOutputHandler = NULL; 

//*****************************************************************************
// Callbacks

// get a new sample
void getSensorReadingCallback(TimerHandle_t xTimer) {
  Serial.print(xTaskGetTickCount() / 1000., 1);
  sensorReadingInt = analogRead(PV_PIN);
  sensorReadingVoltage = (sensorReadingInt) * (VCC/4096.);
  Serial.print(",");
  Serial.print(VCC);
  Serial.print(",");
  Serial.println(sensorReadingVoltage, 4);
}

// send step after <timeToStartInterval> seconds
void setStepInputReadingCallback(TimerHandle_t xTimer) {
  dacWrite(MV_PIN, 77); 
  VCC = 1.0;
}


//*****************************************************************************

void setup() {
  // Configure Serial
  Serial.begin(115200);
  // Configure I/O direction
  pinMode(MV_PIN, OUTPUT);
  pinMode(PV_PIN, INPUT);
  
  // Create a auto-reload timer for sensor readings
  getSensorReadingTimer = xTimerCreate(
                      "getSensorReadingTimer",     // Name of timer
                      samplingInterval,            // Period of timer (in ticks)
                      pdTRUE,              // Auto-reload TRUE, one_shot FALSE
                      (void *)0,            // Timer ID
                      getSensorReadingCallback);  // Callback function
  // Create a one shot timer for step output
  stepInputStartTimer  = xTimerCreate(
                      "stepInputStartTimer ",     // Name of timer
                      timeToStartInterval,            // Period of timer (in ticks)
                      pdFALSE,              // Auto-reload TRUE, one_shot FALSE
                      (void *)1,            // Timer ID
                      setStepInputReadingCallback);  // Callback function

  xTimerStart(getSensorReadingTimer, 0);
  xTimerStart(stepInputStartTimer, 0);          
}

void loop() {
  vTaskSuspend(NULL); // Execution should never get here
}