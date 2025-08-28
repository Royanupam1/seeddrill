#include "Calibration.h"
#include <EEPROM.h>
#include <Arduino.h>
#include <math.h>

void loadCalibration(EEPROMData &eepromData, float &calib_a, float &calib_b, bool &calibrated) {
    EEPROM.get(0, eepromData);
    if (eepromData.version == EEPROM_VERSION) {
        calib_a = eepromData.a;
        calib_b = eepromData.b;
        calibrated = isfinite(calib_a) && isfinite(calib_b) && 
                    fabs(calib_a) > 0.01 && calib_a > 0;
    } else {
        calib_a = 1.0;
        calib_b = 0.0;
        calibrated = false;
    }
}

void printStartupBanner(float calib_a, float calib_b, bool calibrated) {
    Serial.println(F("==============================================="));
    Serial.println(F(" GPS-Based Seed Drill"));
    Serial.println(F("==============================================="));
    Serial.println();
    Serial.println(F("CALIBRATION STATUS:"));
    
    if (calibrated) {
        Serial.println(F("LOADED FROM EEPROM"));
        Serial.print(F("Formula: PWM = "));
        Serial.print(calib_a, 4);
        Serial.print(F(" * RPM + "));
        Serial.println(calib_b, 4);
        
        // Calculate RPM range using constants
        const int minPWM = 30;
        const int maxPWM = 250;
        float minRPM = (minPWM - calib_b) / calib_a;
        float maxRPM = (maxPWM - calib_b) / calib_a;
        
        Serial.print(F("Valid RPM range: "));
        Serial.print(minRPM, 0);
        Serial.print(F(" - "));
        Serial.print(maxRPM, 0);
        Serial.println(F(" RPM"));
    } else {
        Serial.println(F("USING LOOKUP TABLE"));
    }

    Serial.println();
    printPinConfiguration();
    Serial.println();
}

// Split into separate function for better organization
void printPinConfiguration() {
    Serial.println(F("ESP32 Pin Configuration:"));
    Serial.print(F("Motor R_PWM: GPIO")); Serial.println(R_PWM);
    Serial.print(F("Motor L_PWM: GPIO")); Serial.println(L_PWM);
    Serial.print(F("GPS RX: GPIO")); Serial.println(GPS_RX_PIN);
    Serial.print(F("GPS TX: GPIO")); Serial.println(GPS_TX_PIN);
    Serial.print(F("Calib Button: GPIO")); Serial.println(CALIB_BUTTON_PIN);
    Serial.print(F("LCD SDA: GPIO")); Serial.println(LCD_SDA);
    Serial.print(F("LCD SCL: GPIO")); Serial.println(LCD_SCL);
}

bool validateCalibration(float calib_a, float calib_b, float pwmVals[], float rpmVals[], int numPoints) {
    if (!isfinite(calib_a) || !isfinite(calib_b) || calib_a <= 0) return false;
    
    float ss_res = 0, ss_tot = 0, meanY = 0;
    
    // Calculate mean
    for (int i = 0; i < numPoints; i++) {
        meanY += pwmVals[i];
    }
    meanY /= numPoints;
    
    // Calculate R-squared
    for (int i = 0; i < numPoints; i++) {
        float predicted = calib_a * rpmVals[i] + calib_b;
        ss_res += pow(pwmVals[i] - predicted, 2);
        ss_tot += pow(pwmVals[i] - meanY, 2);
    }
    
    float rSquared = 1.0f - (ss_res / ss_tot);
    Serial.print(F("R-squared: "));
    Serial.println(rSquared, 4);
    
    return rSquared > MIN_R_SQUARED;
}

// Split calibration into smaller functions for better maintainability
bool collectCalibrationData(float pwmVals[], float rpmVals[], int numPoints, int &validPoints) {
    validPoints = 0;
    
    for (int i = 0; i < numPoints; i++) {
        if (!collectSingleCalibrationPoint(i, numPoints, pwmVals[i], rpmVals[i])) {
            continue;
        }
        validPoints++;
    }
    
    return validPoints >= 3;
}

bool collectSingleCalibrationPoint(int pointIndex, int totalPoints, float pwmVal, float &rpmVal) {
    bool validInput = false;
    float rpm = 0;
    
    Serial.print(F("Test "));
    Serial.print(pointIndex + 1);
    Serial.print(F("/"));
    Serial.print(totalPoints);
    Serial.print(F(" - PWM: "));
    Serial.println(pwmVal);
    
    setMotorForward(pwmVal);
    Serial.println(F("Motor running... measure RPM and enter value:"));
    Serial.println(F("(Enter 's' to skip this point)"));
    
    delay(CALIBRATION_STABILIZATION_DELAY);
    
    while (!validInput) {
        while (!Serial.available()) delay(SERIAL_CHECK_DELAY);
        
        String input = Serial.readString();
        input.trim();
        
        if (input.equalsIgnoreCase("s")) {
            Serial.println(F("Skipping this point"));
            rpmVal = NAN;
            validInput = true;
            return false;
        }
        
        rpm = input.toFloat();
        if (rpm < MIN_VALID_RPM || rpm > MAX_VALID_RPM) {
            Serial.print(F("Invalid RPM! Must be between "));
            Serial.print(MIN_VALID_RPM);
            Serial.print(F(" and "));
            Serial.print(MAX_VALID_RPM);
            Serial.println(F(". Please re-enter:"));
        } else {
            rpmVal = rpm;
            validInput = true;
            Serial.print(F("Recorded: PWM="));
            Serial.print(pwmVal);
            Serial.print(F(" -> RPM="));
            Serial.println(rpmVal);
            Serial.println();
            return true;
        }
    }
    return false;
}

bool computeCalibrationCoefficients(float pwmVals[], float rpmVals[], int numPoints, float &new_calib_a, float &new_calib_b) {
    float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    int n = 0;
    
    for (int i = 0; i < numPoints; i++) {
        if (isnan(rpmVals[i])) continue;
        
        float x = rpmVals[i];
        float y = pwmVals[i];
        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumX2 += x * x;
        n++;
    }
    
    float denominator = n * sumX2 - sumX * sumX;
    if (fabs(denominator) < 0.001) {
        Serial.println(F("CALIBRATION FAILED - Invalid data"));
        return false;
    }
    
    new_calib_a = (n * sumXY - sumX * sumY) / denominator;
    new_calib_b = (sumY - new_calib_a * sumX) / n;
    
    return true;
}

void showCalibrationResults(float new_calib_a, float new_calib_b, float pwmVals[], float rpmVals[], int numPoints) {
    Serial.println(F("\nCalibration Results:"));
    Serial.print(F("Formula: PWM = "));
    Serial.print(new_calib_a, 4);
    Serial.print(F(" * RPM + "));
    Serial.println(new_calib_b, 4);
    
    Serial.println(F("\nCalibration Test:"));
    Serial.println(F("RPM\tActual PWM\tPredicted PWM\tError"));
    
    for (int i = 0; i < numPoints; i++) {
        if (isnan(rpmVals[i])) continue;
        
        int predictedPWM = (int)(new_calib_a * rpmVals[i] + new_calib_b);
        int error = abs(predictedPWM - pwmVals[i]);
        
        Serial.print(rpmVals[i], 1);
        Serial.print(F("\t"));
        Serial.print(pwmVals[i]);
        Serial.print(F("\t\t"));
        Serial.print(predictedPWM);
        Serial.print(F("\t\t"));
        Serial.println(error);
    }
}

bool saveCalibrationToEEPROM(float new_calib_a, float new_calib_b) {
    Serial.print(F("\nSave this calibration to EEPROM? (y/n): "));
    
    while (!Serial.available()) delay(SERIAL_CHECK_DELAY);
    
    String response = Serial.readString();
    response.trim();
    
    if (response.equalsIgnoreCase("y")) {
        EEPROMData newData = {EEPROM_VERSION, new_calib_a, new_calib_b};
        EEPROM.put(0, newData);
        EEPROM.commit();
        
        // Update global variables
        calib_a = new_calib_a;
        calib_b = new_calib_b;
        calibrated = true;
        
        Serial.println(F("Calibration saved to ESP32 EEPROM"));
        return true;
    } else {
        Serial.println(F("Calibration NOT saved"));
        return false;
    }
}

// Simplified main calibration function
void calibrateMotor() {
    Serial.println(F("\n=== ESP32 MOTOR CALIBRATION ==="));
    Serial.println(F("Ensure motor is unloaded and free to rotate"));
    Serial.println(F("Enter RPM values when prompted\n"));

    const int numPoints = NUM_CALIB_POINTS;
    const float pwmVals[numPoints] = {50, 80, 110, 140, 180, 220};  // const array to save RAM
    float rpmVals[numPoints];
    int validPoints = 0;

    // Collect calibration data
    if (!collectCalibrationData((float*)pwmVals, rpmVals, numPoints, validPoints)) {
        Serial.println(F("CALIBRATION FAILED - Insufficient valid data points"));
        stopMotor();
        return;
    }

    stopMotor();
    Serial.println(F("Computing calibration..."));

    // Compute coefficients
    float new_calib_a, new_calib_b;
    if (!computeCalibrationCoefficients((float*)pwmVals, rpmVals, numPoints, new_calib_a, new_calib_b)) {
        return;
    }

    // Validate calibration
    if (!validateCalibration(new_calib_a, new_calib_b, (float*)pwmVals, rpmVals, numPoints)) {
        Serial.println(F("CALIBRATION FAILED - Poor fit (R-squared too low)"));
        return;
    }

    // Show results
    showCalibrationResults(new_calib_a, new_calib_b, (float*)pwmVals, rpmVals, numPoints);
    
    // Save to EEPROM
    saveCalibrationToEEPROM(new_calib_a, new_calib_b);
}

