(**************************************************************************
* فانکشن‌بلاک کنترل سرعت خط (FB_LineSpeedControl)
* شامل مدیریت حالت دستی/اتوماتیک، توقف اضطراری، رمپ نرم، و جلوگیری از تقسیم بر صفر
* مناسب برای ماشین‌آلات صنعتی مانند لمینیت، چاپ، و بسته‌بندی
* تاریخ: ۲۰۲۵/۰۷/۰۲ - نسخه کامل صنعتی با قابلیت‌های پیشرفته
**************************************************************************)

FUNCTION_BLOCK FB_LineSpeedControl

VAR_INPUT
    enable              : BOOL;           // فعال‌سازی فانکشن‌بلاک
    mode                : INT;            // حالت کار: 0=Manual, 1=HMI, 2=Auto
    setpointHMI         : REAL;           // نقطه تنظیم از HMI (m/min)
    autoSetpoint        : REAL;           // نقطه تنظیم اتوماتیک (m/min)
    speedUp             : BOOL;           // دکمه افزایش سرعت دستی
    speedDown           : BOOL;           // دکمه کاهش سرعت دستی
    stopEmergency       : BOOL;           // توقف اضطراری
    maxSpeed            : REAL := 100.0;  // حداکثر سرعت مجاز (m/min)
    minSpeed            : REAL := 0.0;    // حداقل سرعت مجاز (m/min)
    rampUpTimeNormal_s  : REAL := 5.0;    // زمان رمپ عادی (ثانیه)
    rampDownTimeEmergency_s : REAL := 2.0; // زمان رمپ توقف اضطراری (ثانیه)
    manualRampTime_s    : REAL := 1.0;    // زمان رمپ حالت دستی (ثانیه)
    rampStep            : REAL := 1.0;    // قدم تغییر سرعت دستی (m/min)
    scanCycle_ms        : REAL := 10.0;   // زمان چرخه اسکن (میلی‌ثانیه)
END_VAR

VAR_OUTPUT
    speedOut_mpm        : REAL;           // سرعت خروجی (m/min)
    emergencyLatched    : BOOL;           // وضعیت قفل توقف اضطراری
END_VAR

VAR
    targetSpeed         : REAL;           // سرعت هدف
    manualSetpoint      : REAL;           // نقطه تنظیم دستی
    delta               : REAL;           // تفاوت سرعت
    step                : REAL;           // قدم رمپ
    rampTime            : REAL;           // زمان رمپ فعلی
END_VAR

// =============================
// فعال‌سازی و شرایط اولیه
// =============================
IF NOT enable THEN
    // اگر غیرفعال است، خروجی باید به صفر برسد
    speedOut_mpm := 0.0;
    RETURN;
END_IF;

// =============================
// توقف اضطراری (Emergency Stop)
// =============================
IF stopEmergency THEN
    speedOut_mpm := speedOut_mpm - SafeDivide(speedOut_mpm, rampDownTimeEmergency_s * 1000.0 / scanCycle_ms);
    IF speedOut_mpm < 5.0 THEN
        speedOut_mpm := 0.0;
        emergencyLatched := TRUE;
        enable := FALSE;
    END_IF;
    RETURN;
END_IF;

// =============================
// قفل شدن پس از توقف اضطراری
// =============================
IF emergencyLatched THEN
    speedOut_mpm := 0.0;
    RETURN;
END_IF;

// =============================
// تغییر مد به دستی با فشردن کلید
// =============================
IF (speedUp OR speedDown) AND (mode <> 0) THEN
    mode := 0; // Manual Mode
    manualSetpoint := speedOut_mpm;
END_IF;

// =============================
// حالت مد دستی (Manual Mode)
// =============================
IF mode = 0 THEN
    IF speedUp THEN
        manualSetpoint := manualSetpoint + rampStep;
    ELSIF speedDown THEN
        manualSetpoint := manualSetpoint - rampStep;
    END_IF;
    targetSpeed := manualSetpoint;
    rampTime := manualRampTime_s;

// =============================
// حالت مد HMI (Mode 1)
// =============================
ELSIF mode = 1 THEN
    targetSpeed := setpointHMI;
    rampTime := rampUpTimeNormal_s;

// =============================
// حالت مد AUTO (Mode 2)
// =============================
ELSIF mode = 2 THEN
    targetSpeed := autoSetpoint;
    rampTime := rampUpTimeNormal_s;
END_IF;

// =============================
// محاسبه سرعت خروجی با رمپ نرم
// =============================
delta := targetSpeed - speedOut_mpm;
step := SafeDivide(delta, rampTime * 1000.0 / scanCycle_ms);

speedOut_mpm := speedOut_mpm + step;

// محدود کردن خروجی به min/max
IF speedOut_mpm > maxSpeed THEN
    speedOut_mpm := maxSpeed;
ELSIF speedOut_mpm < minSpeed THEN
    speedOut_mpm := minSpeed;
END_IF;

END_FUNCTION_BLOCK

// =============================
// تابع SafeDivide برای تقسیم امن
// =============================
FUNCTION SafeDivide : REAL
VAR_INPUT
    numerator   : REAL;
    denominator : REAL;
END_VAR

// تقسیم امن برای جلوگیری از خطای تقسیم بر صفر
IF denominator > -0.0001 AND denominator < 0.0001 THEN
    SafeDivide := 0.0;
ELSE
    SafeDivide := numerator / denominator;
END_IF;

END_FUNCTION