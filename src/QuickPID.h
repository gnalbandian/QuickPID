#pragma once
#ifndef QuickPID_h
#define QuickPID_h

#include <RunningAverage.h>

class AutoTunePID {
    public:
        AutoTunePID();
        AutoTunePID(float *input, float *output, uint8_t tuningRule, bool printOrPlotter = false); // Should't also consider direction?
        ~AutoTunePID(){};

        void reset();
        void autoTuneConfig(const byte outputStep, const byte hysteresis, const int setpoint, const int output);
        bool autoTuneLoop();
        void setAutoTuneConstants(float* kp, float* ki, float* kd);
    
    private:

        float *_input = nullptr;     // Pointers to the Input, Output, and Setpoint variables. This creates a
        float *_output = nullptr;    // hard link between the variables and the PID, freeing the user from having
        // float *mySetpoint = nullptr;  // to constantly tell us what these values are. With pointers we'll just know.

        uint8_t _autoTuneStage = 0;
        uint8_t _tuningRule = 0;
        byte _outputStep;
        byte _hysteresis;
        int _atSetpoint;  // 1/3 of 10-bit ADC matches 8-bit PWM value of 85 for symetrical waveform
        int _atOutput;

        bool _printOrPlotter = false;

        uint32_t _t0, _t1, _t2, _t3;
        float _Ku, _Tu, _td, _kp, _ki, _kd, _rdAvg, _peakHigh, _peakLow;

         const uint16_t RulesContants[10][3] =
                                { //ckp,  cki, ckd x 1000
                                    { 450,  540,   0 },  // ZIEGLER_NICHOLS_PI
                                    { 600, 176,  75 },  // ZIEGLER_NICHOLS_PID
                                    { 313,  142,   0 },  // TYREUS_LUYBEN_PI
                                    { 454,  206,  72 },  // TYREUS_LUYBEN_PID
                                    { 303, 1212,   0 },  // CIANCONE_MARLIN_PI
                                    { 303, 1333,  37 },  // CIANCONE_MARLIN_PID
                                    {   0,    0,   0 },  // AMIGOF_PID
                                    { 700, 1750, 105 },  // PESSEN_INTEGRAL_PID
                                    { 333,  667, 111 },  // SOME_OVERSHOOT_PID
                                    { 200,  400,  67 },  // NO_OVERSHOOT_PID
                                };
        
        RunningAverage<float, 5> inputAvg;

};

class QuickPID {

  public:

    // enum tuningMethod : uint8_t
    // {
    //     ZIEGLER_NICHOLS_PI,
    //     ZIEGLER_NICHOLS_PID,
    //     TYREUS_LUYBEN_PI,
    //     TYREUS_LUYBEN_PID,
    //     CIANCONE_MARLIN_PI,
    //     CIANCONE_MARLIN_PID,
    //     AMIGOF_PID,
    //     PESSEN_INTEGRAL_PID,
    //     SOME_OVERSHOOT_PID,
    //     NO_OVERSHOOT_PID
    // };

    // controller mode
    typedef enum { MANUAL = 0, AUTOMATIC = 1 } mode_t;

    // DIRECT: intput increases when the error is positive. REVERSE: intput decreases when the error is positive.
    typedef enum { DIRECT = 0, REVERSE = 1 } direction_t;

    // commonly used functions ************************************************************************************

    // Constructor. Links the PID to Input, Output, Setpoint and initial Tuning Parameters.
    QuickPID(float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd, float POn, direction_t ControllerDirection);

    // Overload constructor with proportional mode. Links the PID to Input, Output, Setpoint and Tuning Parameters.
    QuickPID(float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd, direction_t ControllerDirection);

    // Sets PID to either Manual (0) or Auto (non-0).
    void SetMode(mode_t Mode);

    // Performs the PID calculation. It should be called every time loop() cycles. ON/OFF and calculation frequency
    // can be set using SetMode and SetSampleTime respectively.
    bool Compute();

    // Automatically determines and sets the tuning parameters Kp, Ki and Kd. Use this in the setup loop.
    // void AutoTune(int inputPin, int outputPin, int tuningRule, int Print, uint32_t timeout);
    void AutoTune(uint8_t tuningRule);
    void clearAutoTune();

    // Sets and clamps the output to a specific range (0-255 by default).
    void SetOutputLimits(int Min, int Max);

    // available but not commonly used functions ******************************************************************

    // While most users will set the tunings once in the constructor, this function gives the user the option of
    // changing tunings during runtime for Adaptive control.
    void SetTunings(float Kp, float Ki, float Kd);

    // Overload for specifying proportional mode.
    void SetTunings(float Kp, float Ki, float Kd, float POn);

    // Sets the controller Direction or Action. DIRECT means the output will increase when the error is positive.
    // REVERSE means the output will decrease when the error is positive.
    void SetControllerDirection(direction_t ControllerDirection);

    // Sets the sample time in microseconds with which each PID calculation is performed. Default is 100000 µs.
    void SetSampleTimeUs(uint32_t NewSampleTimeUs);

    //Display functions ******************************************************************************************
    float GetKp();         // These functions query the pid for interal values. They were created mainly for
    float GetKi();         // the pid front-end, where it's important to know what is actually inside the PID.
    float GetKd();
    mode_t GetMode();
    direction_t GetDirection();
    int analogReadFast(int ADCpin);

    AutoTunePID *autoTune;

  private:

    void Initialize();

    float dispKp;       // tuning parameters for display purposes.
    float dispKi;
    float dispKd;

    float pOn;          // proportional mode (0-1) default = 1 (100% Proportional on Error)
    float kp;           // (P)roportional Tuning Parameter
    float ki;           // (I)ntegral Tuning Parameter
    float kd;           // (D)erivative Tuning Parameter
    float kpi;          // proportional on error amount
    float kpd;          // proportional on measurement amount

    float *myInput;     // Pointers to the Input, Output, and Setpoint variables. This creates a
    float *myOutput;    // hard link between the variables and the PID, freeing the user from having
    float *mySetpoint;  // to constantly tell us what these values are. With pointers we'll just know.

    direction_t controllerDirection;
    uint32_t sampleTimeUs, lastTime;
    int outMin, outMax, error;
    int lastInput, outputSum;
    bool inAuto;

    inline int32_t FL_FX(float a) {
      return (a * 256.0); // float to fixed point
    }
    inline int32_t FX_MUL(int32_t a, int32_t b) {
      return ((a * b) >> 8); // fixed point multiply
    }
    inline int32_t CONSTRAIN(int32_t x, int32_t lower, int32_t upper) {
      return ((x) < (lower) ? (lower) : ((x) > (upper) ? (upper) : (x)));
    }

}; // class QuickPID

#if (defined(ESP32) || defined(ARDUINO_ARCH_ESP32))
#include "analogWrite.h"
#endif
#endif // QuickPID.h
