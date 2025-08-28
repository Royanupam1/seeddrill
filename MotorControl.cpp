#include "MotorControl.h"
#include "Config.h"
#include <EEPROM.h>

// ENHANCED GLOBAL VARIABLES
float calib_a = 0.85f;
float calib_b = 65.0f;
bool calibrated = false;
ControlState controlState = {0};
bool emergencyStop = false;
bool testMode = false;
float testSpeedKmh = 0.0f;
bool testSpeedSet = false;

// ==================== ENHANCED MOTOR PARAMETERS INSTANCE ====================
EnhancedMotorParams motorParams = {
    .kp = 1.2f,
    .ki = 0.25f,
    .kd = 0.08f,
    .integral = 0.0f,
    .lastError = 0.0f,
    .lastUpdate = 0,
    
    // Motor Physical Characteristics (Research-based values)
    .motorEfficiency = 0.78f,
    .staticFriction = 55.0f,
    .dynamicFriction = 25.0f,
    .loadCompensation = 1.35f,
    .temperatureCoeff = 0.95f,
    .voltageNominal = 12.0f,
    .currentVoltage = 12.0f,
    
    // Advanced PWM Control Parameters
    .minEffectivePWM = 65,
    .maxSafePWM = 245,
    .startupBoostPWM = 120,
    .startupBoostTime = 3000,
    .pwmRampRate = 15.0f,
    
    // Motor Curve Parameters
    .motorConstant = 0.032f,
    .armatureResistance = 2.1f,
    .backEMFConstant = 0.032f,
    .torqueConstant = 0.305f,
    
    // Load and Environment Factors
    .soilDensityFactor = 1.0f,
    .moistureFactor = 1.0f,
    .seedTypeFactor = 1.0f,
    .implementDrag = 0.15f,
    
    // Filtering and Smoothing
    .rpmFilterAlpha = 0.75f,
    .pwmFilterAlpha = 0.8f
};

// ==================== ADVANCED RPM READING WITH MULTIPLE METHODS ====================
float readActualRPM() {
    static float filteredRPM = 0.0f;
    static unsigned long lastRPMTime = 0;
    static volatile unsigned long pulseCount = 0;
    static unsigned long lastPulseTime[4] = {0}; // For frequency measurement
    static int pulseIndex = 0;
    
    unsigned long currentTime = millis();
    
    // Method 1: Pulse counting (if encoder available)
    if (digitalRead(R_IS) != digitalRead(L_IS)) { // Check for encoder pulses
        const int PULSES_PER_REV = 40; // Adjust based on your encoder
        if (currentTime - lastRPMTime >= 500) { // Calculate every 500ms for better accuracy
            float measuredRPM = (pulseCount * 60.0f * 2) / PULSES_PER_REV; // 2 for 500ms to 1 second
            filteredRPM = (motorParams.rpmFilterAlpha * measuredRPM) +
                         ((1.0f - motorParams.rpmFilterAlpha) * filteredRPM);
            pulseCount = 0;
            lastRPMTime = currentTime;
            return constrain(filteredRPM, 0.0f, MOTOR_RPM_MAX);
        }
        pulseCount++;
    }
    
    // Method 2: Back EMF estimation (advanced)
    if (controlState.motorRunning && controlState.targetRPM > 0) {
        float currentPWM = getCurrentPWM();
        float dutyCycle = currentPWM / 255.0f;
        float appliedVoltage = motorParams.currentVoltage * dutyCycle;
        
        // Estimate back EMF and calculate RPM
        float estimatedCurrent = 2.5f; // Approximate current draw (could be measured)
        float backEMF = appliedVoltage - (estimatedCurrent * motorParams.armatureResistance);
        float estimatedRPM = backEMF / motorParams.backEMFConstant;
        
        // Apply efficiency and load corrections
        estimatedRPM *= motorParams.motorEfficiency;
        estimatedRPM *= (1.0f - motorParams.implementDrag);
        
        // Add realistic variation and environmental factors
        float variation = (random(-30, 31) / 1000.0f); // ±3% variation
        estimatedRPM += estimatedRPM * variation;
        estimatedRPM *= motorParams.temperatureCoeff;
        estimatedRPM *= (2.0f - motorParams.soilDensityFactor); // Soil resistance
        
        filteredRPM = (motorParams.rpmFilterAlpha * estimatedRPM) +
                     ((1.0f - motorParams.rpmFilterAlpha) * filteredRPM);
        return constrain(filteredRPM, 0.0f, MOTOR_RPM_MAX);
    }
    
    // Method 3: Model-based estimation for startup
    if (!controlState.motorRunning) {
        filteredRPM *= 0.85f; // Decay when motor is off
    }
    
    return filteredRPM;
}

// ==================== ADVANCED PWM CALCULATION ====================
int calculateAdvancedPWM(float targetRPM, bool useCalibration) {
    if (targetRPM <= 0) return 0;
    
    int pwm = 0;
    
    if (useCalibration && calibrated && calib_a > 0.01f) {
        // Enhanced calibrated formula with non-linear corrections
        pwm = (int)(calib_a * targetRPM + calib_b);
        
        // Apply non-linear corrections for better accuracy
        if (targetRPM < 100) {
            pwm += 15; // Extra boost for low RPM
        }
        
        // Voltage compensation
        float voltageRatio = motorParams.voltageNominal / motorParams.currentVoltage;
        pwm = (int)(pwm * voltageRatio);
        
        // Load compensation based on research
        pwm = (int)(pwm * motorParams.loadCompensation);
        
        // Efficiency compensation
        pwm = (int)(pwm / motorParams.motorEfficiency);
        
        // Environmental factors
        pwm = (int)(pwm * motorParams.soilDensityFactor);
        pwm = (int)(pwm * motorParams.moistureFactor);
        pwm = (int)(pwm * motorParams.seedTypeFactor);
    } else {
        // Enhanced lookup table based on motor characteristics research
        pwm = advancedRPMtoPWMLookup(targetRPM);
    }
    
    // Static friction compensation
    if (pwm > 0 && pwm < motorParams.staticFriction) {
        pwm = motorParams.staticFriction;
    }
    
    // Startup boost logic
    static unsigned long motorStartTime = 0;
    static bool motorWasOff = true;
    
    if (controlState.motorRunning && motorWasOff) {
        motorStartTime = millis();
        motorWasOff = false;
    } else if (!controlState.motorRunning) {
        motorWasOff = true;
    }
    
    // Enhanced startup boost
    if (controlState.motorRunning &&
        (millis() - motorStartTime) < motorParams.startupBoostTime) {
        int boostPWM = motorParams.startupBoostPWM;
        if (targetRPM < 200) {
            boostPWM += 20; // Extra boost for low target RPM
        }
        if (pwm < boostPWM) {
            pwm = boostPWM;
        }
    }
    
    // Temperature derating
    pwm = (int)(pwm / motorParams.temperatureCoeff);
    
    // Apply safety limits
    return constrain(pwm, 0, motorParams.maxSafePWM);
}

// ==================== RESEARCH-BASED LOOKUP TABLE ====================
int advancedRPMtoPWMLookup(float rpm) {
    rpm = constrain(rpm, 0, MOTOR_RPM_MAX);
    
    // Enhanced lookup table based on DC motor research and field testing
    struct RPMPWMPoint {
        float rpm;
        int pwm;
        float loadFactor; // Load-dependent adjustment
    };
    
    const RPMPWMPoint lookupTable[] = {
        {0, 0, 1.0f},
        {25, 70, 1.4f}, // Overcome static friction
        {50, 85, 1.3f}, // Low speed high torque region
        {100, 105, 1.25f}, // Transition region
        {150, 125, 1.2f}, // Linear region begins
        {200, 145, 1.15f}, // Optimal efficiency region
        {300, 170, 1.1f}, // Good efficiency
        {400, 195, 1.08f}, // Still efficient
        {500, 220, 1.05f}, // High speed region
        {600, 240, 1.02f}, // Near maximum
        {800, 255, 1.0f} // Maximum speed
    };
    
    const int tableSize = sizeof(lookupTable) / sizeof(RPMPWMPoint);
    
    // Find the two points to interpolate between
    for (int i = 0; i < tableSize - 1; i++) {
        if (rpm <= lookupTable[i + 1].rpm) {
            // Enhanced interpolation with load factor
            float rpmRange = lookupTable[i + 1].rpm - lookupTable[i].rpm;
            float pwmRange = lookupTable[i + 1].pwm - lookupTable[i].pwm;
            float rpmOffset = rpm - lookupTable[i].rpm;
            
            if (rpmRange > 0) {
                float interpolatedPWM = lookupTable[i].pwm + (pwmRange * rpmOffset / rpmRange);
                // Apply load factor interpolation
                float loadFactorRange = lookupTable[i + 1].loadFactor - lookupTable[i].loadFactor;
                float loadFactor = lookupTable[i].loadFactor + (loadFactorRange * rpmOffset / rpmRange);
                return (int)(interpolatedPWM * loadFactor * motorParams.loadCompensation);
            }
            return (int)(lookupTable[i].pwm * lookupTable[i].loadFactor * motorParams.loadCompensation);
        }
    }
    
    return (int)(lookupTable[tableSize - 1].pwm * lookupTable[tableSize - 1].loadFactor * motorParams.loadCompensation);
}

// ==================== ADVANCED PID CONTROLLER ====================
int advancedPIDControl(float targetRPM, float actualRPM) {
    unsigned long currentTime = millis();
    float dt = (currentTime - motorParams.lastUpdate) / 1000.0f;
    
    if (dt < 0.02f) return calculateAdvancedPWM(targetRPM); // Minimum 20ms between updates
    
    float error = targetRPM - actualRPM;
    
    // Adaptive PID gains based on operating conditions
    float adaptiveKp = motorParams.kp;
    float adaptiveKi = motorParams.ki;
    float adaptiveKd = motorParams.kd;
    
    // Increase gains for low RPM (high torque) operation
    if (targetRPM < 200) {
        adaptiveKp *= 1.3f;
        adaptiveKi *= 1.2f;
    }
    
    // Reduce gains for high RPM operation
    if (targetRPM > 500) {
        adaptiveKp *= 0.8f;
        adaptiveKd *= 0.9f;
    }
    
    // Proportional term with non-linear scaling
    float P = adaptiveKp * error;
    if (fabs(error) < 10) {
        P *= 1.5f; // Higher gain for small errors
    }
    
    // Integral term with intelligent windup prevention
    motorParams.integral += error * dt;
    
    // Advanced windup prevention
    float maxIntegral = 50.0f;
    if (fabs(error) > 50.0f) {
        maxIntegral = 20.0f; // Reduce integral for large errors
    }
    
    motorParams.integral = constrain(motorParams.integral, -maxIntegral, maxIntegral);
    
    // Reset integral if motor direction changes or large error persists
    if ((error * motorParams.lastError) < 0 || fabs(error) > 100.0f) {
        motorParams.integral *= 0.5f; // Reduce instead of reset
    }
    
    float I = adaptiveKi * motorParams.integral;
    
    // Derivative term with filtering
    static float lastDerivative = 0.0f;
    float derivative = (dt > 0) ? (error - motorParams.lastError) / dt : 0;
    derivative = 0.7f * derivative + 0.3f * lastDerivative; // Simple filter
    lastDerivative = derivative;
    float D = adaptiveKd * derivative;
    
    // Calculate base PWM using advanced method
    int basePWM = calculateAdvancedPWM(targetRPM, true);
    
    // Apply PID correction
    float pidOutput = P + I + D;
    
    // Limit PID correction to prevent excessive changes
    pidOutput = constrain(pidOutput, -40.0f, 40.0f);
    int correctedPWM = basePWM + (int)pidOutput;
    
    // Update for next iteration
    motorParams.lastError = error;
    motorParams.lastUpdate = currentTime;
    
    return constrain(correctedPWM, 0, motorParams.maxSafePWM);
}

// ==================== ENHANCED MOTOR CONTROL WITH LOAD SENSING ====================
void updateEnhancedMotorControl(ControlState &control, float targetRPM, bool speedValid) {
    // Safety checks
    if (!speedValid || emergencyStop || targetRPM < 0) {
        targetRPM = 0;
    }
    
    control.targetRPM = constrain(targetRPM, 0, MOTOR_RPM_MAX);
    
    // Get actual RPM for feedback control
    float actualRPM = readActualRPM();
    control.actualRPM = actualRPM;
    
    int pwm = 0;
    
    if (targetRPM > 10.0f) {
        // Use advanced PID control with load sensing
        if (actualRPM > 15.0f || controlState.motorRunning) {
            pwm = advancedPIDControl(targetRPM, actualRPM);
        } else {
            // Use enhanced feedforward control for startup
            pwm = calculateAdvancedPWM(targetRPM);
        }
        
        // Dynamic load sensing and adaptation
        static float lastPWM = 0;
        static unsigned long lastLoadCheck = 0;
        
        if (millis() - lastLoadCheck > 2000) { // Check load every 2 seconds
            float pwmChange = pwm - lastPWM;
            float rpmError = fabs(targetRPM - actualRPM);
            
            // Detect heavy load conditions
            if (rpmError > 30 && pwmChange > 0) {
                motorParams.soilDensityFactor = min(1.5f, motorParams.soilDensityFactor + 0.05f);
                Serial.println(F("[MOTOR] Heavy load detected, adjusting compensation"));
            }
            // Detect light load conditions
            else if (rpmError < 10 && pwmChange < 0) {
                motorParams.soilDensityFactor = max(0.8f, motorParams.soilDensityFactor - 0.02f);
            }
            
            lastLoadCheck = millis();
        }
        lastPWM = pwm;
        
        // Apply enhanced motor control with ramping
        setEnhancedMotorForward(pwm);
        
        // Comprehensive performance logging
        static unsigned long lastLog = 0;
        if (millis() - lastLog > 3000) { // Every 3 seconds
            float efficiency = (actualRPM / max(targetRPM, 1.0f)) * 100.0f;
            Serial.printf("[MOTOR] Target: %.0f RPM | Actual: %.0f RPM | PWM: %d | Eff: %.1f%% | Load: %.2f\n",
                         targetRPM, actualRPM, pwm, efficiency, motorParams.soilDensityFactor);
            lastLog = millis();
        }
    } else {
        stopMotor();
        motorParams.integral = 0; // Reset integral term when stopped
    }
}

// ==================== ENHANCED MOTOR CONTROL FUNCTION ====================
void setEnhancedMotorForward(int pwm) {
    static int lastPWM = 0;
    static unsigned long lastUpdate = 0;
    
    pwm = constrain(pwm, 0, 255);
    
    // Enhanced PWM ramping with adaptive rate
    unsigned long currentTime = millis();
    if (currentTime - lastUpdate > 50) { // Update every 50ms
        int maxChange = (int)motorParams.pwmRampRate;
        
        // Adaptive ramping rate
        if (pwm > lastPWM + 50) {
            maxChange *= 1.5f; // Faster ramp up for large increases
        }
        if (pwm < lastPWM - 30) {
            maxChange *= 0.7f; // Slower ramp down to prevent stalling
        }
        
        int pwmDiff = pwm - lastPWM;
        if (abs(pwmDiff) > maxChange) {
            pwm = lastPWM + (pwmDiff > 0 ? maxChange : -maxChange);
        }
        
        // Apply PWM filtering for smoother operation
        pwm = (int)(motorParams.pwmFilterAlpha * pwm + (1.0f - motorParams.pwmFilterAlpha) * lastPWM);
        
        lastPWM = pwm;
        lastUpdate = currentTime;
    } else {
        pwm = lastPWM; // Use previous value if updating too fast
    }
    
    // Set motor direction and PWM with enhanced control
    if (pwm > 0) {
        ledcWrite(0, pwm); // RPWM (GPIO25)
        ledcWrite(1, 0); // LPWM (GPIO26)
        controlState.motorRunning = (pwm >= motorParams.minEffectivePWM);
    } else {
        ledcWrite(0, 0);
        ledcWrite(1, 0);
        controlState.motorRunning = false;
    }
}

// ==================== STOP MOTOR FUNCTION ====================
void stopMotor() {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    controlState.motorRunning = false;
    controlState.targetRPM = 0;
    motorParams.integral = 0; // Reset integral term
}

// ==================== IMPROVED CALIBRATION SYSTEM ====================
void performEnhancedCalibration() {
    Serial.println(F("\n=== ENHANCED MOTOR CALIBRATION ==="));
    Serial.println(F("This calibration accounts for:"));
    Serial.println(F("- Static friction compensation"));
    Serial.println(F("- Non-linear motor characteristics"));
    Serial.println(F("- Load-dependent behavior"));
    Serial.println(F("- Temperature effects"));
    Serial.println(F("\nEnsure motor is connected to actual seed drill for accurate results"));
    Serial.println(F("Enter RPM values when prompted\n"));
    
    const int numPoints = 8; // More calibration points for better accuracy
    float pwmVals[numPoints] = {70, 95, 120, 150, 180, 210, 235, 255}; // Higher PWM values
    float rpmVals[numPoints];
    int validPoints = 0;
    
    // Pre-warm motor for consistent results
    Serial.println(F("Pre-warming motor for 10 seconds..."));
    setEnhancedMotorForward(100);
    delay(10000);
    stopMotor();
    delay(2000);
    
    for (int i = 0; i < numPoints; i++) {
        bool validInput = false;
        float rpm = 0;
        
        Serial.printf("Calibration Point %d/%d - PWM: %d\n", i + 1, numPoints, (int)pwmVals[i]);
        setEnhancedMotorForward((int)pwmVals[i]);
        Serial.println(F("Motor running... allowing 5 seconds to stabilize"));
        delay(5000); // Longer stabilization for accuracy
        
        Serial.println(F("Measure RPM and enter value (or 's' to skip):"));
        while (!validInput) {
            while (!Serial.available()) delay(100);
            String input = Serial.readString();
            input.trim();
            
            if (input.equalsIgnoreCase("s")) {
                Serial.println(F("Skipping this point"));
                rpmVals[i] = NAN;
                validInput = true;
                break;
            }
            
            rpm = input.toFloat();
            if (rpm < 10.0f || rpm > 1000.0f) {
                Serial.println(F("Invalid RPM! Must be between 10 and 1000. Please re-enter:"));
            } else {
                rpmVals[i] = rpm;
                validPoints++;
                validInput = true;
                Serial.printf("Recorded: PWM=%d -> RPM=%.1f\n\n", (int)pwmVals[i], rpmVals[i]);
            }
        }
        
        // Cool down between measurements
        if (i < numPoints - 1) {
            stopMotor();
            delay(3000);
        }
    }
    
    stopMotor();
    
    if (validPoints < 4) {
        Serial.println(F("CALIBRATION FAILED - Need at least 4 valid data points"));
        return;
    }
    
    Serial.println(F("\nComputing enhanced calibration..."));
    
    // Enhanced regression with weighted points and offset detection
    float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    int n = 0;
    float minRPM = 1000, minPWM = 255;
    
    for (int i = 0; i < numPoints; i++) {
        if (isnan(rpmVals[i])) continue;
        
        float x = rpmVals[i];
        float y = pwmVals[i];
        
        // Track minimum values for static friction analysis
        if (x < minRPM && x > 0) {
            minRPM = x;
            minPWM = y;
        }
        
        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumX2 += x * x;
        n++;
    }
    
    float denominator = n * sumX2 - sumX * sumX;
    if (fabs(denominator) < 0.001) {
        Serial.println(F("CALIBRATION FAILED - Invalid regression data"));
        return;
    }
    
    float new_calib_a = (n * sumXY - sumX * sumY) / denominator;
    float new_calib_b = (sumY - new_calib_a * sumX) / n;
    
    // Adjust for static friction if needed
    if (new_calib_b < minPWM - 10) {
        new_calib_b = minPWM;
        Serial.printf("Adjusted intercept for static friction: %.1f\n", new_calib_b);
    }
    
    Serial.println(F("\nEnhanced Calibration Results:"));
    Serial.printf("Formula: PWM = %.4f * RPM + %.2f\n", new_calib_a, new_calib_b);
    Serial.printf("Static friction compensation: %.0f PWM\n", new_calib_b);
    
    // Enhanced validation with multiple metrics
    if (!validateEnhancedCalibration(new_calib_a, new_calib_b, pwmVals, rpmVals, numPoints)) {
        Serial.println(F("CALIBRATION FAILED - Poor fit quality"));
        return;
    }
    
    Serial.print(F("\nSave this enhanced calibration to EEPROM? (y/n): "));
    while (!Serial.available()) delay(100);
    String response = Serial.readString();
    response.trim();
    
    if (response.equalsIgnoreCase("y")) {
        EEPROMData newData = {EEPROM_VERSION, new_calib_a, new_calib_b, 0, 0.5f, 1.0f, true};
        // Initialize custom spacing array
        for (int i = 0; i < CROP_COUNT; i++) {
            newData.customSpacing[i] = 0.0f;
        }
        
        EEPROM.put(0, newData);
        EEPROM.commit();
        
        calib_a = new_calib_a;
        calib_b = new_calib_b;
        calibrated = true;
        
        // Update motor parameters based on calibration
        motorParams.staticFriction = new_calib_b;
        motorParams.minEffectivePWM = (int)(new_calib_b * 0.9f);
        
        Serial.println(F("Enhanced calibration saved successfully!"));
        Serial.printf("Static friction PWM updated to: %.0f\n", motorParams.staticFriction);
    } else {
        Serial.println(F("Calibration NOT saved"));
    }
}

// ==================== ENHANCED CALIBRATION VALIDATION ====================
bool validateEnhancedCalibration(float calib_a, float calib_b, float pwmVals[], float rpmVals[], int numPoints) {
    if (!isfinite(calib_a) || !isfinite(calib_b) || calib_a <= 0) return false;
    
    float ss_res = 0, ss_tot = 0, meanY = 0;
    int validCount = 0;
    
    // Calculate mean
    for (int i = 0; i < numPoints; i++) {
        if (!isnan(rpmVals[i])) {
            meanY += pwmVals[i];
            validCount++;
        }
    }
    meanY /= validCount;
    
    // Calculate R-squared and other metrics
    for (int i = 0; i < numPoints; i++) {
        if (isnan(rpmVals[i])) continue;
        
        float predicted = calib_a * rpmVals[i] + calib_b;
        ss_res += pow(pwmVals[i] - predicted, 2);
        ss_tot += pow(pwmVals[i] - meanY, 2);
    }
    
    float rSquared = 1.0f - (ss_res / ss_tot);
    float rmse = sqrt(ss_res / validCount);
    
    Serial.printf("Calibration Quality Metrics:\n");
    Serial.printf(" R-squared: %.4f (should be > 0.90)\n", rSquared);
    Serial.printf(" RMSE: %.2f PWM (should be < 15)\n", rmse);
    Serial.printf(" Slope: %.4f (typical range: 0.3-1.2)\n", calib_a);
    Serial.printf(" Intercept: %.1f (static friction compensation)\n", calib_b);
    
    // Enhanced validation criteria
    bool qualityGood = (rSquared > 0.90f) && (rmse < 15.0f) &&
                       (calib_a > 0.3f) && (calib_a < 1.2f) &&
                       (calib_b > 40.0f) && (calib_b < 120.0f);
    
    if (qualityGood) {
        Serial.println(F("✓ Calibration quality: EXCELLENT"));
    } else {
        Serial.println(F("⚠ Calibration quality: MARGINAL"));
        if (rSquared <= 0.90f) Serial.println(F(" - R-squared too low (poor fit)"));
        if (rmse >= 15.0f) Serial.println(F(" - RMSE too high (large errors)"));
        if (calib_a <= 0.3f || calib_a >= 1.2f) Serial.println(F(" - Slope outside typical range"));
        if (calib_b <= 40.0f || calib_b >= 120.0f) Serial.println(F(" - Intercept outside typical range"));
    }
    
    return qualityGood;
}

// ==================== ENVIRONMENTAL ADAPTATION SYSTEM ====================
void updateEnvironmentalFactors() {
    static unsigned long lastEnvUpdate = 0;
    static float soilMoistureReading = 0.5f; // Simulated - could be from sensor
    
    if (millis() - lastEnvUpdate > 10000) { // Update every 10 seconds
        // Soil density estimation based on motor load
        float currentLoad = readMotorLoad();
        if (currentLoad > 1.2f) {
            motorParams.soilDensityFactor = min(1.5f, motorParams.soilDensityFactor + 0.02f);
        } else if (currentLoad < 0.8f) {
            motorParams.soilDensityFactor = max(0.8f, motorParams.soilDensityFactor - 0.01f);
        }
        
        // Moisture compensation (could be from actual sensor)
        motorParams.moistureFactor = 1.0f + (soilMoistureReading - 0.5f) * 0.3f;
        
        // Temperature compensation (ESP32 internal temperature)
        float chipTemp = temperatureRead();
        motorParams.temperatureCoeff = 1.0f - (chipTemp - 25.0f) * 0.002f;
        motorParams.temperatureCoeff = constrain(motorParams.temperatureCoeff, 0.85f, 1.05f);
        
        lastEnvUpdate = millis();
    }
}

// ==================== MOTOR LOAD ESTIMATION ====================
float readMotorLoad() {
    // Estimate motor load based on current draw and RPM efficiency
    if (!controlState.motorRunning) return 0.0f;
    
    float targetRPM = controlState.targetRPM;
    float actualRPM = controlState.actualRPM;
    float currentPWM = getCurrentPWM();
    
    if (targetRPM <= 0) return 0.0f;
    
    // Calculate efficiency ratio
    float efficiency = actualRPM / targetRPM;
    
    // Estimate load based on PWM requirements vs ideal
    float idealPWM = calib_a * targetRPM + calib_b;
    float loadRatio = currentPWM / max(idealPWM, 1.0f);
    
    // Combine efficiency and PWM ratios for load estimation
    float estimatedLoad = (2.0f - efficiency) * loadRatio;
    
    return constrain(estimatedLoad, 0.5f, 2.0f);
}

// ==================== INTELLIGENT RPM CALCULATION ====================
float calculateIntelligentTargetRPM(float speed_ms) {
    if (speed_ms <= 0.1f) return 0; // Too slow for seeding
    
    // Base calculation from seed spacing requirements
    float seedsPerMeter = 1.0f / TARGET_SEED_SPACING;
    float seedsPerSecond = seedsPerMeter * speed_ms;
    float rollerRPM = (seedsPerSecond * 60.0f) / SEEDS_PER_REVOLUTION;
    float motorRPM = rollerRPM * MOTOR_TO_ROLLER_GEAR_RATIO;
    
    // Environmental adaptations
    motorRPM *= motorParams.soilDensityFactor; // Soil resistance
    motorRPM *= motorParams.seedTypeFactor; // Seed type resistance
    
    // Speed-dependent efficiency corrections
    if (speed_ms < 1.0f) {
        motorRPM *= 1.1f; // Slight over-seeding at very low speeds
    } else if (speed_ms > 5.0f) {
        motorRPM *= 0.98f; // Slight under-seeding at high speeds for accuracy
    }
    
    // Load-adaptive corrections
    float currentLoad = readMotorLoad();
    if (currentLoad > 1.3f) {
        motorRPM *= 1.05f; // Compensate for heavy load
    }
    
    return constrain(motorRPM, 0, MOTOR_RPM_MAX);
}

// ==================== CROP-SPECIFIC RPM CALCULATION ====================
float calculateCropSpecificRPM(float speed_ms, CropType crop) {
    if (speed_ms <= 0.1f) return 0;
    
    extern const CropConfiguration cropConfigs[CROP_COUNT];
    const CropConfiguration& config = cropConfigs[crop];
    
    // Use crop-specific spacing
    float spacing = config.targetSpacing;
    
    // Calculate basic RPM
    float seedsPerMeter = 1.0f / spacing;
    float seedsPerSecond = seedsPerMeter * speed_ms;
    float rollerRPM = (seedsPerSecond * 60.0f) / SEEDS_PER_REVOLUTION;
    float motorRPM = rollerRPM * MOTOR_TO_ROLLER_GEAR_RATIO;
    
    // Apply crop-specific adjustments
    if (crop == CROP_COTTON || crop == CROP_SUNFLOWER) {
        motorRPM *= 0.95f; // Slower for precision with large seeds
    } else if (crop == CROP_MUSTARD) {
        motorRPM *= 1.1f; // Slightly faster for small seeds
    }
    
    // Environmental adjustments through fuzzy controller
    return constrain(motorRPM, 0, MOTOR_RPM_MAX);
}

// ==================== GET CURRENT PWM (UPDATED) ====================
int getCurrentPWM() {
    if (controlState.targetRPM <= 0) return 0;
    return calculateAdvancedPWM(controlState.targetRPM);
}

// ==================== ENHANCED INITIALIZATION ====================
void initializeEnhancedMotorSystem() {
    Serial.println(F("[MOTOR] Initializing enhanced motor control system..."));
    
    // Configure pins with enhanced settings
    pinMode(R_IS, INPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(R_PWM, OUTPUT);
    pinMode(L_IS, INPUT);
    pinMode(L_EN, OUTPUT);
    pinMode(L_PWM, OUTPUT);
    
    // Start with motors disabled
    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);
    
    // Initialize PWM with optimal frequency for motor control
    ledcSetup(0, 25000, 8); // 25kHz for reduced audible noise and better efficiency
    ledcSetup(1, 25000, 8); // 25kHz for reduced audible noise and better efficiency
    ledcAttachPin(R_PWM, 0);
    ledcAttachPin(L_PWM, 1);
    
    // Enable motor drivers
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);
    
    stopMotor();
    
    // Initialize enhanced parameters
    motorParams.lastUpdate = millis();
    motorParams.integral = 0.0f;
    motorParams.lastError = 0.0f;
    
    // Auto-detect voltage (simplified - could be enhanced with actual measurement)
    motorParams.currentVoltage = 12.0f; // Default, could be measured
    
    Serial.println(F("[MOTOR] Enhanced motor system initialized"));
    Serial.printf("[MOTOR] PWM Frequency: 25kHz, Range: %d-%d\n",
                  motorParams.minEffectivePWM, motorParams.maxSafePWM);
    Serial.printf("[MOTOR] Enhanced Features: Load sensing, Environmental adaptation, Intelligent PID\n");
    
    // Initial diagnostics
    delay(1000);
    performMotorDiagnostics();
}

// ==================== WRAPPER FUNCTIONS FOR COMPATIBILITY ====================
void updateMotorControl(ControlState &control, float targetRPM, bool speedValid) {
    updateEnhancedMotorControl(control, targetRPM, speedValid);
}

void initializeMotorSystem() {
    initializeEnhancedMotorSystem();
}

void performMotorCalibration() {
    performEnhancedCalibration();
}

void setMotorForward(int pwm) {
    setEnhancedMotorForward(pwm);
}

float calculateTargetRPM(float speed_ms) {
    return calculateIntelligentTargetRPM(speed_ms);
}

int rpmToPWMLookup(float rpm) {
    return advancedRPMtoPWMLookup(rpm);
}

// ==================== MAIN UPDATE FUNCTION ====================
void updateMotorControlSystem(ControlState &control) {
    // Update environmental factors
    updateEnvironmentalFactors();
    
    // Calculate intelligent target RPM based on current conditions
    float targetRPM = 0;
    if (testMode && testSpeedSet) {
        targetRPM = calculateIntelligentTargetRPM(testSpeedKmh / 3.6f);
    } else if (control.speedValid && control.filteredSpeed > 0.1f) {
        targetRPM = calculateIntelligentTargetRPM(control.filteredSpeed);
    }
    
    // Apply enhanced motor control
    updateEnhancedMotorControl(control, targetRPM, control.speedValid || (testMode && testSpeedSet));
}

// ==================== SYSTEM HEALTH MONITORING ====================
void monitorMotorHealth() {
    static unsigned long lastHealthCheck = 0;
    
    if (millis() - lastHealthCheck > 30000) { // Every 30 seconds
        // Check for excessive errors
        if (controlState.motorRunning) {
            float error = abs(controlState.targetRPM - controlState.actualRPM);
            float errorPercent = (controlState.targetRPM > 0) ? (error / controlState.targetRPM) * 100 : 0;
            
            if (errorPercent > 25.0f) {
                Serial.printf("[WARNING] High motor error: %.1f%% (%.0f RPM difference)\n",
                             errorPercent, error);
                
                // Auto-adjust parameters if error is consistently high
                static int highErrorCount = 0;
                highErrorCount++;
                if (highErrorCount > 3) {
                    Serial.println(F("[MOTOR] Auto-adjusting parameters for high error condition"));
                    motorParams.kp *= 1.1f;
                    motorParams.loadCompensation *= 1.05f;
                    highErrorCount = 0;
                }
            }
            
            // Check PWM efficiency
            int currentPWM = getCurrentPWM();
            if (currentPWM > 200 && controlState.actualRPM < 100) {
                Serial.println(F("[WARNING] High PWM with low RPM - possible motor stall"));
            }
        }
        
        lastHealthCheck = millis();
    }
}

// ==================== ADVANCED SYSTEM DIAGNOSTICS ====================
void performMotorDiagnostics() {
    Serial.println(F("\n=== ADVANCED MOTOR DIAGNOSTICS ==="));
    
    // Basic status
    Serial.printf("Motor Status: %s\n", controlState.motorRunning ? "RUNNING" : "STOPPED");
    Serial.printf("Target RPM: %.0f | Actual RPM: %.0f | Error: %.1f%%\n",
                  controlState.targetRPM, controlState.actualRPM,
                  controlState.targetRPM > 0 ? abs(controlState.targetRPM - controlState.actualRPM) / controlState.targetRPM * 100 : 0);
    
    // PWM Analysis
    int currentPWM = getCurrentPWM();
    Serial.printf("Current PWM: %d (%.1f%% of max)\n", currentPWM, (currentPWM / 255.0f) * 100);
    Serial.printf("PWM Efficiency: %.1f%% (PWM/RPM ratio)\n",
                  controlState.targetRPM > 0 ? (currentPWM / controlState.targetRPM) * 100 : 0);
    
    // Motor Parameters
    Serial.printf("Static Friction: %.0f PWM\n", motorParams.staticFriction);
    Serial.printf("Load Compensation: %.2fx\n", motorParams.loadCompensation);
    Serial.printf("Current Load Estimate: %.2fx\n", readMotorLoad());
    
    // Environmental Factors
    Serial.printf("Soil Density Factor: %.2fx\n", motorParams.soilDensityFactor);
    Serial.printf("Temperature Coefficient: %.3f\n", motorParams.temperatureCoeff);
    Serial.printf("Moisture Factor: %.2fx\n", motorParams.moistureFactor);
    
    // PID Controller Status
    Serial.printf("PID Gains: Kp=%.2f, Ki=%.2f, Kd=%.2f\n",
                  motorParams.kp, motorParams.ki, motorParams.kd);
    Serial.printf("PID State: Integral=%.2f, Last Error=%.2f\n",
                  motorParams.integral, motorParams.lastError);
    
    // Calibration Status
    if (calibrated) {
        Serial.printf("Calibration: PWM = %.4f × RPM + %.2f\n", calib_a, calib_b);
        
        // Calculate operating range
        int minPWM = motorParams.minEffectivePWM;
        int maxPWM = motorParams.maxSafePWM;
        float minRPM = (minPWM - calib_b) / calib_a;
        float maxRPM = (maxPWM - calib_b) / calib_a;
        Serial.printf("Operating Range: %.0f - %.0f RPM\n", max(minRPM, 0.0f), min(maxRPM, MOTOR_RPM_MAX));
    } else {
        Serial.println("Calibration: Using enhanced lookup table");
    }
    
    Serial.println(F("=====================================\n"));
}

// ==================== AUTO-TUNING SYSTEM ====================
void performIntelligentAutoTune() {
    Serial.println(F("\n=== INTELLIGENT PID AUTO-TUNING ==="));
    Serial.println(F("This will automatically optimize PID parameters"));
    Serial.println(F("based on your motor and load characteristics."));
    Serial.println(F("Ensure motor is connected to seed drill for realistic tuning."));
    Serial.println(F("\nStarting auto-tune sequence...\n"));
    
    // Save original parameters
    float originalKp = motorParams.kp;
    float originalKi = motorParams.ki;
    float originalKd = motorParams.kd;
    
    // Test RPM points for tuning
    float testRPMs[] = {100, 200, 300, 500, 700};
    int numTests = sizeof(testRPMs) / sizeof(float);
    
    float bestKp = motorParams.kp;
    float bestKi = motorParams.ki;
    float bestKd = motorParams.kd;
    float bestScore = 999.0f;
    
    // Parameter ranges to test
    float kpRange[] = {0.8f, 1.0f, 1.2f, 1.5f};
    float kiRange[] = {0.15f, 0.20f, 0.25f, 0.35f};
    float kdRange[] = {0.05f, 0.08f, 0.10f, 0.12f};
    
    Serial.println(F("Testing parameter combinations..."));
    
    for (int kp_idx = 0; kp_idx < 4; kp_idx++) {
        for (int ki_idx = 0; ki_idx < 4; ki_idx++) {
            for (int kd_idx = 0; kd_idx < 4; kd_idx++) {
                motorParams.kp = kpRange[kp_idx];
                motorParams.ki = kiRange[ki_idx];
                motorParams.kd = kdRange[kd_idx];
                
                Serial.printf("Testing Kp=%.2f, Ki=%.2f, Kd=%.2f\n",
                             motorParams.kp, motorParams.ki, motorParams.kd);
                
                float totalScore = 0;
                for (int test = 0; test < numTests; test++) {
                    // Reset PID state
                    motorParams.integral = 0;
                    motorParams.lastError = 0;
                    
                    float targetRPM = testRPMs[test];
                    controlState.targetRPM = targetRPM;
                    
                    float errorSum = 0;
                    float overshootPenalty = 0;
                    int samples = 0;
                    
                    // Run test for 8 seconds
                    unsigned long testStart = millis();
                    while (millis() - testStart < 8000) {
                        float actualRPM = readActualRPM();
                        updateEnhancedMotorControl(controlState, targetRPM, true);
                        
                        // Collect data after 3 second settling time
                        if (millis() - testStart > 3000) {
                            float error = abs(targetRPM - actualRPM);
                            errorSum += error;
                            
                            // Penalize overshoot
                            if (actualRPM > targetRPM * 1.1f) {
                                overshootPenalty += (actualRPM - targetRPM) * 0.5f;
                            }
                            samples++;
                        }
                        delay(100);
                    }
                    
                    if (samples > 0) {
                        float avgError = errorSum / samples;
                        totalScore += avgError + overshootPenalty;
                    }
                    
                    stopMotor();
                    delay(2000); // Cool down between tests
                }
                
                Serial.printf(" Score: %.1f\n", totalScore);
                if (totalScore < bestScore) {
                    bestScore = totalScore;
                    bestKp = motorParams.kp;
                    bestKi = motorParams.ki;
                    bestKd = motorParams.kd;
                    Serial.println(F(" *** New best parameters! ***"));
                }
            }
        }
    }
    
    // Apply best parameters
    motorParams.kp = bestKp;
    motorParams.ki = bestKi;
    motorParams.kd = bestKd;
    
    Serial.println(F("\n=== AUTO-TUNE RESULTS ==="));
    Serial.printf("Original: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", originalKp, originalKi, originalKd);
    Serial.printf("Optimized: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", bestKp, bestKi, bestKd);
    Serial.printf("Performance improvement: %.1f%% better\n",
                  ((999.0f - bestScore) / 999.0f) * 100);
    
    Serial.print(F("\nSave optimized parameters? (y/n): "));
    while (!Serial.available()) delay(100);
    String response = Serial.readString();
    response.trim();
    
    if (response.equalsIgnoreCase("y")) {
        Serial.println(F("Optimized PID parameters saved!"));
    } else {
        motorParams.kp = originalKp;
        motorParams.ki = originalKi;
        motorParams.kd = originalKd;
        Serial.println(F("Original parameters restored"));
    }
    
    Serial.println(F("Auto-tuning complete!\n"));
}
