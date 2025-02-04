# QuickPID   [![arduino-library-badge](https://www.ardu-badge.com/badge/QuickPID.svg?)](https://www.ardu-badge.com/QuickPID)

QuickPID is a fast fixed/floating point implementation of the Arduino PID library with built-in [AutoTune](https://github.com/Dlloydev/QuickPID/wiki/AutoTune) function. This controller can automatically determine and set parameters (Kp, Ki, Kd). Additionally the Ultimate Gain `Ku`, Ultimate Period `Tu`, Dead Time `td` and controllability of the process are determined. There are 9 tuning rules available to choose from. Also available is a POn setting that controls the mix of Proportional on Error to Proportional on Measurement.  

### About

This PID controller provides a faster *read-compute-write* cycle than alternatives as it has a more efficient and reduced algorithm that avoids time consuming partial calculations, it takes advantage of fixed point math and has a faster analog read function. The `Ki` and `Kd` are scaled by time (µs) and the new `kpi` and `kpd` parameters are calculated in the `SetTunings()` function. This results in a simple and fast algorithm with only two multiply operations required in `Compute()`.

### Features

Development began with a fork of the Arduino PID Library. Modifications and new features have been added as described in the change log and below:

- Quicker hybrid fixed/floating point math in compute function
- Built-in `AutoTune()` function automatically determines and sets `Kp`, `Ki` and `Kd`. and also ultimate gain `Ku` and ultimate period `Tu` of the control system. There are 9 tuning rules to choose from
- [AutoTune](https://github.com/Dlloydev/QuickPID/wiki/AutoTune) uses a precise and low control effort sequence that gets results quickly. It also determines how difficult the process is to control.
- `POn` parameter controls the setpoint weighting and mix of Proportional on Error to Proportional on Measurement
- Reorganized and more efficient PID algorithm, faster analog read function, micros() timing resolution
- Runs a complete PID cycle (*read-compute-write*) faster than just an `analogRead()` command  in Arduino
- Includes  a complete [analogWrite function for ESP32](https://github.com/Dlloydev/ESP32-ESP32S2-AnalogWrite) boards. This controls up to 8 independent PWM pins and 2 DAC pins.

### Performance

| `Compute()`                                                  | Kp   | Ki   | Kd   | Step Time (µS) |
| :----------------------------------------------------------- | ---- | ---- | ---- | -------------- |
| QuickPID                                                     | 2.0  | 15.0 | 0.05 | 72             |
| Arduino PID                                                  | 2.0  | 15.0 | 0.05 | 104            |
| **Full PID cycle:** **`analogRead(), Compute(), analogWrite()`** |      |      |      |                |
| QuickPID using `analogReadFast()`                            | 2.0  | 5.0  | 0.2  | 96             |
| Arduino PID using `analogRead()`                             | 2.0  | 5.0  | 0.2  | 224            |

### [AutoTune RC Filter](https://github.com/Dlloydev/QuickPID/wiki/AutoTune_RC_Filter)

This example allows you to experiment with the AutoTune, various tuning rules and the POn control on an RC filter. It automatically determines and sets the tuning parameters.

#### [QuickPID WiKi ...](https://github.com/Dlloydev/QuickPID/wiki)

### Simplified PID Algorithm

```c++
outputSum += (kpi * error) - (kpd * dInput);
```

The new `kpi` and `kpd` parameters are calculated in the `SetTunings()` function. This results in a simple and fast algorithm with only two multiply operations required. The pOn variable controls the setpoint weighting of Proportional on Error and Proportional on Measurement. The gains for `error` (`kpi`) and measurement `dInput` (`kpd`) are calculated as follows:

```c++
 kpi = kp * pOn + ki;
 kpd = kp * (1 - pOn) + kd;
```

### Controller Action

If a positive error increases the controller's output, the controller is said to be direct acting (i.e. heating process). When a positive error decreases the controller's output, the controller is said to be reverse acting (i.e. cooling process). Since the PWM and ADC peripherals on microcontrollers only operate with positive values, QuickPID only uses positive values for Input, Output and Setpoint. When the controller is set to `REVERSE` acting, the sign of the `error` and `dInput` (derivative of Input) is internally changed. All operating ranges and limits remain the same. To simulate a `REVERSE` acting process from a process that's  `DIRECT` acting, the Input value needs to be "flipped". That is, if your reading from a 10-bit ADC with 0-1023 range, the input value used is (1023 - reading). See the example [AutoTune_RC_Filter.ino](https://github.com/Dlloydev/QuickPID/blob/master/examples/AutoTune_RC_Filter/AutoTune_RC_Filter.ino) for details.

### Functions

#### QuickPID_Constructor

```c++
QuickPID::QuickPID(float* Input, float* Output, float* Setpoint,
                   float Kp, float Ki, float Kd, float POn, uint8_t ControllerDirection);
```

- `Input`, `Output`, and `Setpoint` are pointers to the variables holding these values.
- `Kp`, `Ki`, and `Kd` are the PID proportional, integral, and derivative gains.
- `POn` is the Proportional on Error weighting value (0.0-1.0). This controls the mix of Proportional on Error (PonE) and  Proportional on Measurement (PonM) that's used in the compute algorithm. Note that POn controls the PonE amount, where the remainder (1-PonE) is the PonM amount. Also, the default POn is 1

<img src="https://user-images.githubusercontent.com/63488701/104958919-fe3c4680-599e-11eb-851e-73f26291d3e5.gif" alt="POn" style="zoom: 67%;" />

- `ControllerDirection` Either DIRECT or REVERSE determines which direction the output will move for a given error. DIRECT is most common.

```c++
QuickPID::QuickPID(float* Input, float* Output, float* Setpoint,
                   float Kp, float Ki, float Kd, uint8_t ControllerDirection);
```

This allows you to use Proportional on Error without explicitly saying so.

#### Compute

```c++
bool QuickPID::Compute();
```

This function contains the PID algorithm and it should be called once every loop(). Most of the time it will just return false without doing anything. However, at a  frequency specified by `SetSampleTime` it will calculate a new Output and return true.

#### [AutoTune](https://github.com/Dlloydev/QuickPID/wiki/AutoTune)

```c++
void QuickPID::AutoTune(int inputPin, int outputPin, int tuningRule, int Print = 0, uint32_t timeout = 30);
```

The `AutoTune()` function automatically determines and sets `Kp`, `Ki` and `Kd`. It also determines the critical gain `Ku` and critical period `Tu` of the control system. 

`int Print = 0; // on(1), off(0)`

When using Serial Monitor, turn on serial print output to view the AutoTune status and results. When using the Serial Plotter, turn off the AutoTune print output to prevent plot labels from being overwritten.

`uint32_t timeout = 120`

Sets the AutoTune timeout where the default is 120 seconds.

#### SetTunings

```c++
void QuickPID::SetTunings(float Kp, float Ki, float Kd, float POn);
```

This function allows the controller's dynamic performance to be adjusted. It's called automatically from the constructor, but tunings can also be adjusted on the fly during normal operation. The parameters are as described in the constructor.

```c++
void QuickPID::SetTunings(float Kp, float Ki, float Kd);
```

Set Tunings using the last remembered POn setting.

#### SetSampleTime

```c++
void QuickPID::SetSampleTimeUs(uint32_t NewSampleTimeUs);
```

Sets the period, in microseconds, at which the calculation is performed. The default is 100ms.

#### SetOutputLimits

```c++
void QuickPID::SetOutputLimits(int Min, int Max);
```

The PID controller is designed to vary its output within a given range.  By default this range is 0-255, the Arduino PWM range.

#### SetMode

```c++
void QuickPID::SetMode(uint8_t Mode);
```

Allows the controller Mode to be set to `MANUAL` (0) or `AUTOMATIC` (non-zero). when the transition from manual to automatic occurs, the controller is automatically initialized.

#### Initialize

```c++
void QuickPID::Initialize();
```

Does all the things that need to happen to ensure a bump-less transfer from manual to automatic mode.

#### SetControllerDirection

```c++
void QuickPID::SetControllerDirection(uint8_t Direction)
```

The PID will either be connected to a DIRECT acting process (+Output leads to +Input) or a REVERSE acting process (+Output leads to -Input.) We need to know which one, because otherwise we may increase the output when we should be decreasing. This is called from the constructor.

#### Display_Functions

```c++
float QuickPID::GetKp();
float QuickPID::GetKi();
float QuickPID::GetKd();
float QuickPID::GetKu();
float QuickPID::GetTu();
float QuickPID::GetTd();
uint8_t QuickPID::GetMode();
uint8_t QuickPID::GetDirection();
```

These functions query the internal state of the PID. They're here for display purposes.

#### Utility_Functions

```c++
int QuickPID::analogReadFast(int ADCpin)
```

A faster configuration of `analogRead()`where a preset of 32 is used.  If the architecture definition isn't found, normal `analogRead()`is used to return a value.

#### [AnalogWrite (PWM and DAC) for ESP32](https://github.com/Dlloydev/ESP32-ESP32S2-AnalogWrite)

Use this link for reference. Note that if you're using QuickPID, there's no need to install the AnalogWrite library as this feature is already included.

#### Change Log

#### [![arduino-library-badge](https://www.ardu-badge.com/badge/QuickPID.svg?)](https://www.ardu-badge.com/QuickPID)

#### Version 2.2.8

- AutoTune is now independent of the QuickPID library and can be run from a sketch.  AutoTune is now non-blocking, no timeouts are required and it uses Input and Output variables directly. See the example [AutoTune_RC_Filter.ino](https://github.com/Dlloydev/QuickPID/blob/master/examples/AutoTune_RC_Filter/AutoTune_RC_Filter.ino) for details.

#### Version 2.2.7

- Fixed REVERSE acting controller mode.
- now using src folder for source code 
- replaced defines with enumerated types and inline functions

#### Version 2.2.6

- Changed Input, Output and Setpoint parameters to float.
- Updated compatibility with the ESP32 AnalogWrite 

#### Version 2.2.2

- Added compatibility with the ESP32 Arduino framework 
- Added full featured AnalogWrite methods for ESP32 and ESP32S2

#### Version 2.2.1

- Even faster AutoTune function
- AutoTune  now determines the controllability of the process
- Added AMIGO_PID tuning rule
- Added `GetTd()` function to display dead time

#### Version 2.2.0

- Improved AutoTune function
- Added 8 tuning rules
- Added `GetKu()` function to display ultimate gain
- Added `GetTu()` function to display ultimate period (time constant)
- Updated example and documentation

#### Version 2.1.0

- Added AutoTune function and documentation

- Added AutoTune_RC_Filter example and documentation

#### Version 2.0.5

- Added MIT license text file
- POn defaults to 1

#### Version 2.0.4

- Added `QuickPID_AdaptiveTunings.ino`, `QuickPID_Basic.ino`, `QuickPID_PonM.ino` and `QuickPID_RelayOutput.ino` to the examples folder.
- `QuickPID_RelayOutput.ino` has the added feature of `minWindow` setting that sets the minimum on time for the relay.

#### Version 2.0.3

- Initial Version with modifications as listed in [features.](#Features) 

------

### Original README (Arduino PID)

```
***************************************************************
* Arduino PID Library - Version 1.2.1
* by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
*
* This Library is licensed under the MIT License
***************************************************************
```

 - For an ultra-detailed explanation of why the code is the way it is, please visit:
   http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

 - For function documentation see:  http://playground.arduino.cc/Code/PIDLibrary

------

