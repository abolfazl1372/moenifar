(******************************************************************************
* فانکشن‌بلاک کنترل سرعت خط تولید / Line Speed Control Function Block
* ===========================================================================
* نام / Name: FB_LineSpeedControl
* نسخه / Version: 2.0 (بهینه‌سازی شده / Optimized)
* تاریخ / Date: 2025/01/07
* 
* توضیحات / Description:
* این فانکشن‌بلاک برای کنترل دقیق سرعت خطوط تولید صنعتی طراحی شده است
* This function block is designed for precise control of industrial production line speeds
* 
* کاربردها / Applications:
* - خطوط لمینیت / Laminate production lines
* - ماشین‌های چاپ / Printing machines  
* - خطوط بسته‌بندی / Packaging lines
* - سیستم‌های نوار نقاله / Conveyor systems
* - خطوط نساجی / Textile production lines
* 
* ویژگی‌های بهبود یافته / Enhanced Features:
* - حافظه سرعت قبل از توقف / Speed memory before stop
* - محاسبه شتاب لحظه‌ای / Instantaneous acceleration calculation
* - شمارنده زمان کارکرد / Runtime counter
* - هشدارهای سرعت / Speed warnings
* - محافظت از تغییرات ناگهانی / Sudden change protection
*******************************************************************************)

FUNCTION_BLOCK FB_LineSpeedControl

(* ===== ورودی‌ها / INPUTS ===== *)
VAR_INPUT
    (* کنترل اصلی / Main Control *)
    enable              : BOOL;             // فعال‌سازی فانکشن‌بلاک / Enable function block
    reset               : BOOL;             // ریست خطاها و قفل‌ها / Reset errors and latches
    
    (* انتخاب حالت / Mode Selection *)
    mode                : INT;              // حالت کار / Operating mode: 0=Manual, 1=HMI, 2=Auto
    
    (* نقاط تنظیم / Setpoints *)
    setpointHMI         : REAL;             // نقطه تنظیم از HMI / HMI setpoint (m/min)
    autoSetpoint        : REAL;             // نقطه تنظیم اتوماتیک / Automatic setpoint (m/min)
    
    (* کنترل دستی / Manual Control *)
    speedUp             : BOOL;             // دکمه افزایش سرعت / Speed up button
    speedDown           : BOOL;             // دکمه کاهش سرعت / Speed down button
    jogForward          : BOOL;             // حرکت آهسته رو به جلو / Jog forward
    jogReverse          : BOOL;             // حرکت آهسته رو به عقب / Jog reverse
    
    (* ایمنی / Safety *)
    stopEmergency       : BOOL;             // توقف اضطراری / Emergency stop
    safetyInterlockOK   : BOOL := TRUE;    // اینترلاک ایمنی / Safety interlock
    
    (* محدودیت‌ها / Limits *)
    maxSpeed            : REAL := 100.0;    // حداکثر سرعت مجاز / Maximum allowed speed (m/min)
    minSpeed            : REAL := 0.0;      // حداقل سرعت مجاز / Minimum allowed speed (m/min)
    jogSpeed            : REAL := 5.0;      // سرعت حرکت آهسته / Jog speed (m/min)
    
    (* پارامترهای رمپ / Ramp Parameters *)
    rampUpTimeNormal_s  : REAL := 5.0;     // زمان افزایش سرعت عادی / Normal ramp up time (s)
    rampDownTimeNormal_s: REAL := 5.0;     // زمان کاهش سرعت عادی / Normal ramp down time (s)
    rampDownTimeEmergency_s : REAL := 2.0; // زمان توقف اضطراری / Emergency stop time (s)
    manualRampTime_s    : REAL := 1.0;     // زمان رمپ دستی / Manual ramp time (s)
    rampStep            : REAL := 1.0;      // قدم تغییر سرعت دستی / Manual speed step (m/min)
    
    (* تنظیمات سیستم / System Settings *)
    scanCycle_ms        : REAL := 10.0;    // زمان چرخه اسکن PLC / PLC scan cycle time (ms)
    speedTolerance      : REAL := 0.5;      // تلرانس سرعت / Speed tolerance (m/min)
    warningThreshold    : REAL := 90.0;     // آستانه هشدار سرعت / Speed warning threshold (%)
END_VAR

(* ===== خروجی‌ها / OUTPUTS ===== *)
VAR_OUTPUT
    (* خروجی‌های اصلی / Main Outputs *)
    speedOut_mpm        : REAL;             // سرعت خروجی / Output speed (m/min)
    speedOut_percent    : REAL;             // سرعت خروجی درصدی / Output speed percentage (%)
    acceleration_mps2   : REAL;             // شتاب لحظه‌ای / Instantaneous acceleration (m/s²)
    
    (* وضعیت‌ها / Status *)
    isRunning           : BOOL;             // در حال کار / Running status
    atSetpoint          : BOOL;             // رسیدن به نقطه تنظیم / At setpoint
    emergencyLatched    : BOOL;             // قفل توقف اضطراری / Emergency stop latched
    
    (* هشدارها / Warnings *)
    speedWarning        : BOOL;             // هشدار سرعت بالا / High speed warning
    rampActive          : BOOL;             // رمپ فعال است / Ramp is active
    
    (* اطلاعات تشخیصی / Diagnostic Info *)
    currentMode         : STRING[20];       // نمایش حالت فعلی / Current mode display
    runTime_hours       : REAL;             // زمان کارکرد / Runtime hours
    totalDistance_km    : REAL;             // مسافت کل طی شده / Total distance traveled (km)
END_VAR

(* ===== متغیرهای داخلی / INTERNAL VARIABLES ===== *)
VAR
    (* متغیرهای کنترل / Control Variables *)
    targetSpeed         : REAL;             // سرعت هدف / Target speed
    previousSpeed       : REAL;             // سرعت قبلی / Previous speed
    speedBeforeStop     : REAL;             // سرعت قبل از توقف / Speed before stop
    manualSetpoint      : REAL;             // نقطه تنظیم دستی / Manual setpoint
    
    (* محاسبات رمپ / Ramp Calculations *)
    delta               : REAL;             // تفاوت سرعت / Speed difference
    step                : REAL;             // قدم رمپ / Ramp step
    rampTime            : REAL;             // زمان رمپ فعلی / Current ramp time
    actualRampRate      : REAL;             // نرخ واقعی رمپ / Actual ramp rate
    
    (* تایمرها و شمارنده‌ها / Timers & Counters *)
    runTimer            : TON;              // تایمر زمان کارکرد / Runtime timer
    distanceAccumulator : REAL;             // انباشتگر مسافت / Distance accumulator
    
    (* فلگ‌های کنترل / Control Flags *)
    firstScan           : BOOL := TRUE;     // اولین اسکن / First scan flag
    modeChanged         : BOOL;             // تغییر حالت / Mode changed flag
    previousMode        : INT;              // حالت قبلی / Previous mode
END_VAR

(* ===== کنستانت‌ها / CONSTANTS ===== *)
VAR CONSTANT
    MODE_MANUAL         : INT := 0;         // کد حالت دستی / Manual mode code
    MODE_HMI            : INT := 1;         // کد حالت HMI / HMI mode code  
    MODE_AUTO           : INT := 2;         // کد حالت اتوماتیک / Auto mode code
    MIN_RAMP_TIME       : REAL := 0.1;      // حداقل زمان رمپ / Minimum ramp time (s)
    SPEED_ZERO_THRESHOLD: REAL := 0.1;      // آستانه سرعت صفر / Zero speed threshold
END_VAR

(* ========================================================================= *)
(* بدنه اصلی برنامه / MAIN PROGRAM BODY *)
(* ========================================================================= *)

(* ----- مقداردهی اولیه / Initialization ----- *)
IF firstScan THEN
    manualSetpoint := 0.0;
    speedOut_mpm := 0.0;
    previousSpeed := 0.0;
    runTime_hours := 0.0;
    totalDistance_km := 0.0;
    firstScan := FALSE;
END_IF;

(* ----- ریست خطاها / Reset Errors ----- *)
IF reset AND emergencyLatched THEN
    emergencyLatched := FALSE;
    speedBeforeStop := 0.0;
END_IF;

(* ----- بررسی فعال‌سازی و اینترلاک ایمنی / Check Enable & Safety Interlock ----- *)
IF NOT enable OR NOT safetyInterlockOK THEN
    (* کاهش تدریجی سرعت تا صفر / Gradual speed reduction to zero *)
    IF speedOut_mpm > SPEED_ZERO_THRESHOLD THEN
        speedOut_mpm := speedOut_mpm - SafeDivide(speedOut_mpm, rampDownTimeNormal_s * 1000.0 / scanCycle_ms);
    ELSE
        speedOut_mpm := 0.0;
    END_IF;
    isRunning := FALSE;
    currentMode := 'DISABLED';
    RETURN;
END_IF;

(* ----- مدیریت توقف اضطراری / Emergency Stop Management ----- *)
IF stopEmergency THEN
    (* ذخیره سرعت قبل از توقف / Save speed before stop *)
    IF NOT emergencyLatched AND speedOut_mpm > SPEED_ZERO_THRESHOLD THEN
        speedBeforeStop := speedOut_mpm;
    END_IF;
    
    (* کاهش سریع سرعت با رمپ اضطراری / Rapid speed reduction with emergency ramp *)
    speedOut_mpm := speedOut_mpm - SafeDivide(speedOut_mpm, rampDownTimeEmergency_s * 1000.0 / scanCycle_ms);
    
    (* قفل کردن سیستم پس از توقف کامل / Lock system after complete stop *)
    IF speedOut_mpm < SPEED_ZERO_THRESHOLD THEN
        speedOut_mpm := 0.0;
        emergencyLatched := TRUE;
        isRunning := FALSE;
    END_IF;
    
    currentMode := 'EMERGENCY STOP';
    RETURN;
END_IF;

(* ----- بررسی قفل اضطراری / Check Emergency Latch ----- *)
IF emergencyLatched THEN
    speedOut_mpm := 0.0;
    isRunning := FALSE;
    currentMode := 'E-STOP LATCHED';
    RETURN;
END_IF;

(* ----- مدیریت حرکت آهسته (Jog) / Jog Management ----- *)
IF jogForward OR jogReverse THEN
    IF jogForward AND NOT jogReverse THEN
        targetSpeed := jogSpeed;
        currentMode := 'JOG FORWARD';
    ELSIF jogReverse AND NOT jogForward THEN
        targetSpeed := -jogSpeed;
        currentMode := 'JOG REVERSE';
    ELSE
        (* هر دو دکمه فشرده شده - توقف / Both buttons pressed - stop *)
        targetSpeed := 0.0;
        currentMode := 'JOG CONFLICT';
    END_IF;
    rampTime := manualRampTime_s;
    
(* ----- انتخاب حالت عملکرد / Operating Mode Selection ----- *)
ELSE
    (* تشخیص تغییر حالت / Detect mode change *)
    IF mode <> previousMode THEN
        modeChanged := TRUE;
        previousMode := mode;
    ELSE
        modeChanged := FALSE;
    END_IF;
    
    (* تغییر خودکار به حالت دستی با فشردن دکمه‌ها / Auto switch to manual with button press *)
    IF (speedUp OR speedDown) AND (mode <> MODE_MANUAL) THEN
        mode := MODE_MANUAL;
        manualSetpoint := speedOut_mpm;
        modeChanged := TRUE;
    END_IF;
    
    (* پردازش حالت‌های مختلف / Process different modes *)
    CASE mode OF
        MODE_MANUAL:  (* حالت دستی / Manual Mode *)
            (* مدیریت دکمه‌های افزایش/کاهش سرعت / Handle speed up/down buttons *)
            IF speedUp AND NOT speedDown THEN
                manualSetpoint := manualSetpoint + rampStep;
            ELSIF speedDown AND NOT speedUp THEN
                manualSetpoint := manualSetpoint - rampStep;
            END_IF;
            
            (* اعمال محدودیت‌ها به setpoint دستی / Apply limits to manual setpoint *)
            manualSetpoint := LIMIT(minSpeed, manualSetpoint, maxSpeed);
            
            targetSpeed := manualSetpoint;
            rampTime := manualRampTime_s;
            currentMode := 'MANUAL';
            
        MODE_HMI:  (* حالت HMI / HMI Mode *)
            targetSpeed := LIMIT(minSpeed, setpointHMI, maxSpeed);
            rampTime := rampUpTimeNormal_s;
            currentMode := 'HMI CONTROL';
            
        MODE_AUTO:  (* حالت اتوماتیک / Auto Mode *)
            targetSpeed := LIMIT(minSpeed, autoSetpoint, maxSpeed);
            rampTime := rampUpTimeNormal_s;
            currentMode := 'AUTOMATIC';
            
        ELSE  (* حالت نامعتبر / Invalid mode *)
            targetSpeed := 0.0;
            currentMode := 'INVALID MODE';
    END_CASE;
END_IF;

(* ----- محاسبه و اعمال رمپ سرعت / Calculate and Apply Speed Ramp ----- *)
(* محاسبه اختلاف سرعت / Calculate speed difference *)
delta := targetSpeed - speedOut_mpm;

(* انتخاب زمان رمپ بر اساس جهت تغییر / Select ramp time based on direction *)
IF delta > 0 THEN
    (* افزایش سرعت / Speed increase *)
    rampTime := MAX(MIN_RAMP_TIME, rampUpTimeNormal_s);
ELSIF delta < 0 THEN
    (* کاهش سرعت / Speed decrease *)
    rampTime := MAX(MIN_RAMP_TIME, rampDownTimeNormal_s);
END_IF;

(* محاسبه قدم رمپ / Calculate ramp step *)
IF ABS(delta) > speedTolerance THEN
    step := SafeDivide(delta, rampTime * 1000.0 / scanCycle_ms);
    speedOut_mpm := speedOut_mpm + step;
    rampActive := TRUE;
ELSE
    (* رسیدن به نقطه تنظیم / Reached setpoint *)
    speedOut_mpm := targetSpeed;
    rampActive := FALSE;
END_IF;

(* ----- اعمال محدودیت‌های نهایی / Apply Final Limits ----- *)
speedOut_mpm := LIMIT(minSpeed, speedOut_mpm, maxSpeed);

(* ----- محاسبات خروجی / Output Calculations ----- *)
(* محاسبه درصد سرعت / Calculate speed percentage *)
IF maxSpeed > 0 THEN
    speedOut_percent := (speedOut_mpm / maxSpeed) * 100.0;
ELSE
    speedOut_percent := 0.0;
END_IF;

(* محاسبه شتاب / Calculate acceleration *)
(* شتاب = تغییر سرعت / زمان *)
(* Acceleration = speed change / time *)
IF scanCycle_ms > 0 THEN
    acceleration_mps2 := ((speedOut_mpm - previousSpeed) / 60.0) / (scanCycle_ms / 1000.0);
ELSE
    acceleration_mps2 := 0.0;
END_IF;

(* به‌روزرسانی سرعت قبلی / Update previous speed *)
previousSpeed := speedOut_mpm;

(* ----- تنظیم وضعیت‌ها / Set Status Flags ----- *)
(* وضعیت در حال کار / Running status *)
isRunning := ABS(speedOut_mpm) > SPEED_ZERO_THRESHOLD;

(* رسیدن به نقطه تنظیم / At setpoint *)
atSetpoint := ABS(speedOut_mpm - targetSpeed) <= speedTolerance;

(* هشدار سرعت بالا / High speed warning *)
speedWarning := speedOut_percent >= warningThreshold;

(* ----- محاسبات زمان کارکرد و مسافت / Runtime and Distance Calculations ----- *)
(* تایمر زمان کارکرد / Runtime timer *)
runTimer(IN := isRunning, PT := T#1H);
IF runTimer.Q THEN
    runTime_hours := runTime_hours + 1.0;
    runTimer(IN := FALSE);  (* ریست تایمر / Reset timer *)
END_IF;

(* محاسبه مسافت طی شده / Calculate distance traveled *)
(* مسافت = سرعت × زمان *)
(* Distance = speed × time *)
IF isRunning THEN
    distanceAccumulator := distanceAccumulator + (ABS(speedOut_mpm) * scanCycle_ms / 60000.0);
    IF distanceAccumulator >= 1000.0 THEN
        totalDistance_km := totalDistance_km + 1.0;
        distanceAccumulator := distanceAccumulator - 1000.0;
    END_IF;
END_IF;

END_FUNCTION_BLOCK

(******************************************************************************
* تابع تقسیم ایمن / Safe Division Function
* ===========================================================================
* این تابع از خطای تقسیم بر صفر جلوگیری می‌کند
* This function prevents division by zero errors
*******************************************************************************)
FUNCTION SafeDivide : REAL
VAR_INPUT
    numerator   : REAL;     // صورت کسر / Numerator
    denominator : REAL;     // مخرج کسر / Denominator
END_VAR

VAR
    ZERO_THRESHOLD : REAL := 0.0001;  // آستانه صفر / Zero threshold
END_VAR

(* بررسی مخرج برای جلوگیری از تقسیم بر صفر / Check denominator to prevent division by zero *)
IF ABS(denominator) < ZERO_THRESHOLD THEN
    (* بازگشت صفر در صورت مخرج صفر / Return zero if denominator is zero *)
    SafeDivide := 0.0;
ELSE
    (* انجام تقسیم عادی / Perform normal division *)
    SafeDivide := numerator / denominator;
END_IF;

END_FUNCTION