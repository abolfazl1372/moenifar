(******************************************************************************
* نمونه استفاده از فانکشن‌بلاک کنترل سرعت خط / Line Speed Control Usage Example
* ===========================================================================
* این برنامه نحوه استفاده از FB_LineSpeedControl را نشان می‌دهد
* This program demonstrates how to use FB_LineSpeedControl
* 
* سناریوهای پوشش داده شده / Covered Scenarios:
* 1. کنترل دستی با دکمه‌ها / Manual control with buttons
* 2. کنترل از HMI / HMI control
* 3. کنترل اتوماتیک / Automatic control
* 4. توقف اضطراری و ریست / Emergency stop and reset
* 5. حرکت آهسته (Jog) / Jog operation
* 6. نمایش اطلاعات تشخیصی / Display diagnostic information
*******************************************************************************)

PROGRAM Example_LineSpeedControl

(* ===== متغیرهای ورودی از سیستم / Input Variables from System ===== *)
VAR
    (* دکمه‌های کنترل / Control Buttons *)
    btnEnable           : BOOL;             // دکمه فعال‌سازی / Enable button
    btnReset            : BOOL;             // دکمه ریست / Reset button
    btnEmergencyStop    : BOOL;             // دکمه توقف اضطراری / Emergency stop button
    btnSpeedUp          : BOOL;             // دکمه افزایش سرعت / Speed up button
    btnSpeedDown        : BOOL;             // دکمه کاهش سرعت / Speed down button
    btnJogForward       : BOOL;             // دکمه حرکت آهسته جلو / Jog forward button
    btnJogReverse       : BOOL;             // دکمه حرکت آهسته عقب / Jog reverse button
    
    (* انتخابگر حالت / Mode Selector *)
    modeSelector        : INT := 1;         // 0=Manual, 1=HMI, 2=Auto
    
    (* ورودی‌های HMI / HMI Inputs *)
    hmiSpeedSetpoint    : REAL := 50.0;    // نقطه تنظیم سرعت از HMI / Speed setpoint from HMI (m/min)
    
    (* ورودی‌های اتوماتیک / Automatic Inputs *)
    autoSpeedCommand    : REAL := 75.0;    // فرمان سرعت از سیستم بالادست / Speed command from higher system (m/min)
    
    (* سنسورها و فیدبک / Sensors & Feedback *)
    safetyGatesClosed   : BOOL := TRUE;    // وضعیت درب‌های ایمنی / Safety gates status
    motorTempOK         : BOOL := TRUE;    // دمای موتور نرمال / Motor temperature OK
    
    (* پارامترهای خط تولید / Production Line Parameters *)
    productType         : INT := 1;         // نوع محصول / Product type (1-5)
    lineLength          : REAL := 100.0;    // طول خط تولید / Production line length (m)
END_VAR

(* ===== پارامترهای قابل تنظیم / Configurable Parameters ===== *)
VAR
    (* تنظیمات برای انواع محصول / Settings for Product Types *)
    maxLineSpeed        : REAL := 120.0;    // حداکثر سرعت خط / Maximum line speed (m/min)
    minLineSpeed        : REAL := 5.0;      // حداقل سرعت خط / Minimum line speed (m/min)
    jogSpeedSetting     : REAL := 3.0;      // سرعت حرکت آهسته / Jog speed (m/min)
    
    (* زمان‌های رمپ / Ramp Times *)
    normalRampUp        : REAL := 10.0;     // زمان رمپ بالا عادی / Normal ramp up time (s)
    normalRampDown      : REAL := 8.0;      // زمان رمپ پایین عادی / Normal ramp down time (s)
    emergencyRampTime   : REAL := 3.0;      // زمان رمپ اضطراری / Emergency ramp time (s)
    manualRampTime      : REAL := 0.5;      // زمان رمپ دستی / Manual ramp time (s)
    
    (* تنظیمات سیستم / System Settings *)
    plcScanTime         : REAL := 20.0;     // زمان اسکن PLC / PLC scan time (ms)
    speedStep           : REAL := 2.0;      // قدم تغییر سرعت دستی / Manual speed step (m/min)
    speedToleranceBand  : REAL := 1.0;      // باند تلرانس سرعت / Speed tolerance band (m/min)
    highSpeedWarning    : REAL := 85.0;     // آستانه هشدار سرعت بالا / High speed warning threshold (%)
END_VAR

(* ===== اینستنس فانکشن‌بلاک / Function Block Instance ===== *)
VAR
    lineSpeedController : FB_LineSpeedControl;  // کنترلر سرعت خط / Line speed controller
END_VAR

(* ===== متغیرهای خروجی برای نمایش / Output Variables for Display ===== *)
VAR
    (* اطلاعات سرعت / Speed Information *)
    actualSpeed         : REAL;             // سرعت واقعی / Actual speed (m/min)
    speedPercentage     : REAL;             // سرعت به درصد / Speed percentage (%)
    currentAcceleration : REAL;             // شتاب فعلی / Current acceleration (m/s²)
    
    (* وضعیت‌ها / Status *)
    lineRunning         : BOOL;             // خط در حال کار / Line running
    speedReached        : BOOL;             // رسیدن به سرعت هدف / Target speed reached
    emergencyActive     : BOOL;             // توقف اضطراری فعال / Emergency stop active
    rampInProgress      : BOOL;             // رمپ در حال اجرا / Ramp in progress
    highSpeedAlarm      : BOOL;             // آلارم سرعت بالا / High speed alarm
    
    (* اطلاعات عملیاتی / Operational Information *)
    operatingMode       : STRING[20];       // حالت عملیاتی / Operating mode
    totalRunTime        : REAL;             // زمان کل کارکرد / Total runtime (hours)
    totalDistance       : REAL;             // مسافت کل / Total distance (km)
    
    (* محاسبات کارایی / Efficiency Calculations *)
    averageSpeed        : REAL;             // سرعت متوسط / Average speed (m/min)
    productionRate      : REAL;             // نرخ تولید / Production rate (units/hour)
END_VAR

(* ===== منطق برای تعیین اینترلاک ایمنی / Safety Interlock Logic ===== *)
VAR
    safetyInterlock     : BOOL;             // اینترلاک ایمنی کلی / Overall safety interlock
END_VAR

(* ========================================================================= *)
(* بدنه اصلی برنامه / MAIN PROGRAM BODY *)
(* ========================================================================= *)

(* ----- تنظیم پارامترها بر اساس نوع محصول / Set Parameters Based on Product Type ----- *)
CASE productType OF
    1:  (* محصول نوع 1 - سرعت پایین / Product Type 1 - Low Speed *)
        maxLineSpeed := 60.0;
        normalRampUp := 5.0;
        normalRampDown := 4.0;
        
    2:  (* محصول نوع 2 - سرعت متوسط / Product Type 2 - Medium Speed *)
        maxLineSpeed := 100.0;
        normalRampUp := 8.0;
        normalRampDown := 6.0;
        
    3:  (* محصول نوع 3 - سرعت بالا / Product Type 3 - High Speed *)
        maxLineSpeed := 150.0;
        normalRampUp := 12.0;
        normalRampDown := 10.0;
        
    4:  (* محصول نوع 4 - محصول حساس / Product Type 4 - Sensitive Product *)
        maxLineSpeed := 80.0;
        normalRampUp := 15.0;
        normalRampDown := 12.0;
        emergencyRampTime := 5.0;
        
    5:  (* محصول نوع 5 - تولید سریع / Product Type 5 - Fast Production *)
        maxLineSpeed := 200.0;
        normalRampUp := 10.0;
        normalRampDown := 8.0;
        speedStep := 5.0;
        
    ELSE  (* پیش‌فرض / Default *)
        maxLineSpeed := 100.0;
        normalRampUp := 10.0;
        normalRampDown := 8.0;
END_CASE;

(* ----- محاسبه اینترلاک ایمنی / Calculate Safety Interlock ----- *)
safetyInterlock := safetyGatesClosed AND motorTempOK;

(* ----- فراخوانی فانکشن‌بلاک کنترل سرعت / Call Line Speed Control Function Block ----- *)
lineSpeedController(
    (* ورودی‌های کنترل اصلی / Main Control Inputs *)
    enable              := btnEnable,
    reset               := btnReset,
    mode                := modeSelector,
    
    (* نقاط تنظیم / Setpoints *)
    setpointHMI         := hmiSpeedSetpoint,
    autoSetpoint        := autoSpeedCommand,
    
    (* کنترل دستی / Manual Control *)
    speedUp             := btnSpeedUp,
    speedDown           := btnSpeedDown,
    jogForward          := btnJogForward,
    jogReverse          := btnJogReverse,
    
    (* ایمنی / Safety *)
    stopEmergency       := btnEmergencyStop,
    safetyInterlockOK   := safetyInterlock,
    
    (* محدودیت‌ها / Limits *)
    maxSpeed            := maxLineSpeed,
    minSpeed            := minLineSpeed,
    jogSpeed            := jogSpeedSetting,
    
    (* پارامترهای رمپ / Ramp Parameters *)
    rampUpTimeNormal_s  := normalRampUp,
    rampDownTimeNormal_s:= normalRampDown,
    rampDownTimeEmergency_s := emergencyRampTime,
    manualRampTime_s    := manualRampTime,
    rampStep            := speedStep,
    
    (* تنظیمات سیستم / System Settings *)
    scanCycle_ms        := plcScanTime,
    speedTolerance      := speedToleranceBand,
    warningThreshold    := highSpeedWarning
);

(* ----- خواندن خروجی‌ها / Read Outputs ----- *)
(* اطلاعات سرعت / Speed Information *)
actualSpeed         := lineSpeedController.speedOut_mpm;
speedPercentage     := lineSpeedController.speedOut_percent;
currentAcceleration := lineSpeedController.acceleration_mps2;

(* وضعیت‌ها / Status *)
lineRunning         := lineSpeedController.isRunning;
speedReached        := lineSpeedController.atSetpoint;
emergencyActive     := lineSpeedController.emergencyLatched;
rampInProgress      := lineSpeedController.rampActive;
highSpeedAlarm      := lineSpeedController.speedWarning;

(* اطلاعات عملیاتی / Operational Information *)
operatingMode       := lineSpeedController.currentMode;
totalRunTime        := lineSpeedController.runTime_hours;
totalDistance       := lineSpeedController.totalDistance_km;

(* ----- محاسبات اضافی / Additional Calculations ----- *)
(* محاسبه سرعت متوسط / Calculate Average Speed *)
IF totalRunTime > 0 THEN
    averageSpeed := (totalDistance * 1000.0) / (totalRunTime * 60.0);  // m/min
ELSE
    averageSpeed := 0.0;
END_IF;

(* محاسبه نرخ تولید تخمینی / Calculate Estimated Production Rate *)
(* فرض: هر 10 متر خط = 1 واحد محصول / Assumption: Every 10m of line = 1 product unit *)
IF lineRunning AND actualSpeed > 0 THEN
    productionRate := (actualSpeed * 60.0) / 10.0;  // units/hour
ELSE
    productionRate := 0.0;
END_IF;

(* ----- نمایش اطلاعات تشخیصی در HMI / Display Diagnostic Info on HMI ----- *)
(* این اطلاعات می‌تواند در HMI نمایش داده شود *)
(* This information can be displayed on HMI *)

(* 
مثال نمایش در HMI / HMI Display Example:
=====================================
سرعت فعلی / Current Speed: [actualSpeed] m/min ([speedPercentage]%)
شتاب / Acceleration: [currentAcceleration] m/s²
حالت عملیاتی / Operating Mode: [operatingMode]
وضعیت خط / Line Status: [IF lineRunning THEN "در حال کار/Running" ELSE "متوقف/Stopped"]
زمان کارکرد / Runtime: [totalRunTime] hours
مسافت طی شده / Distance: [totalDistance] km
سرعت متوسط / Avg Speed: [averageSpeed] m/min
نرخ تولید / Production Rate: [productionRate] units/hr

هشدارها / Warnings:
- سرعت بالا / High Speed: [highSpeedAlarm]
- توقف اضطراری / E-Stop: [emergencyActive]
- رمپ فعال / Ramp Active: [rampInProgress]
*)

END_PROGRAM