(**************************************************************************
* مثال استفاده از فانکشن‌بلاک FB_LineSpeedControl
* نحوه پیاده‌سازی در برنامه اصلی PLC
**************************************************************************)

PROGRAM Main
VAR
    // Instance از فانکشن‌بلاک کنترل سرعت
    lineSpeedController : FB_LineSpeedControl;
    
    // ورودی‌های سیستم
    systemEnable        : BOOL := TRUE;
    operationMode       : INT := 1;        // 0=Manual, 1=HMI, 2=Auto
    hmiSpeedSetpoint    : REAL := 50.0;    // نقطه تنظیم از HMI
    autoSpeedSetpoint   : REAL := 60.0;    // نقطه تنظیم اتوماتیک
    manualSpeedUp       : BOOL;            // از دکمه‌های اپراتور
    manualSpeedDown     : BOOL;            // از دکمه‌های اپراتور
    emergencyStop       : BOOL;            // از سیستم ایمنی
    
    // خروجی‌های سیستم
    finalSpeedOutput    : REAL;            // سرعت نهایی برای محرک
    emergencyActive     : BOOL;            // وضعیت قفل اضطراری
    
    // تنظیمات سیستم
    maxLineSpeed        : REAL := 120.0;   // حداکثر سرعت خط (m/min)
    minLineSpeed        : REAL := 5.0;     // حداقل سرعت خط (m/min)
    normalRampTime      : REAL := 8.0;     // زمان رمپ عادی (ثانیه)
    emergencyRampTime   : REAL := 3.0;     // زمان رمپ اضطراری (ثانیه)
    manualRampTime      : REAL := 2.0;     // زمان رمپ دستی (ثانیه)
    manualStepSize      : REAL := 2.0;     // قدم تغییر دستی (m/min)
    plcScanTime         : REAL := 10.0;    // زمان چرخه PLC (ms)
END_VAR

// =============================
// فراخوانی فانکشن‌بلاک کنترل سرعت
// =============================
lineSpeedController(
    enable                  := systemEnable,
    mode                    := operationMode,
    setpointHMI             := hmiSpeedSetpoint,
    autoSetpoint            := autoSpeedSetpoint,
    speedUp                 := manualSpeedUp,
    speedDown               := manualSpeedDown,
    stopEmergency           := emergencyStop,
    maxSpeed                := maxLineSpeed,
    minSpeed                := minLineSpeed,
    rampUpTimeNormal_s      := normalRampTime,
    rampDownTimeEmergency_s := emergencyRampTime,
    manualRampTime_s        := manualRampTime,
    rampStep                := manualStepSize,
    scanCycle_ms            := plcScanTime,
    speedOut_mpm            => finalSpeedOutput,
    emergencyLatched        => emergencyActive
);

// =============================
// پردازش خروجی‌ها
// =============================
// ارسال سرعت به محرک اصلی
// DriveSpeed := finalSpeedOutput;

// نمایش وضعیت در HMI
// HMI_ActualSpeed := finalSpeedOutput;
// HMI_EmergencyStatus := emergencyActive;

// =============================
// مدیریت حالت‌های مختلف
// =============================
CASE operationMode OF
    0: // Manual Mode
        // در حالت دستی، اپراتور با دکمه‌ها کنترل می‌کند
        // دکمه‌های speedUp و speedDown فعال هستند
        ;
    
    1: // HMI Mode  
        // در حالت HMI، سرعت از رابط کاربری تنظیم می‌شود
        // hmiSpeedSetpoint از HMI دریافت می‌شود
        ;
    
    2: // Auto Mode
        // در حالت اتوماتیک، سرعت از سیستم کنترل بالاتر می‌آید
        // autoSpeedSetpoint از PLC/SCADA دریافت می‌شود
        ;
END_CASE;

END_PROGRAM