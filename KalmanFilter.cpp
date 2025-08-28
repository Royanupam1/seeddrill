#include "KalmanFilter.h"
#include "Config.h"
#include <Arduino.h>
#include <math.h>

bool isValidTractorGPSData(float speed_kmh, int satellites, float hdop,
                           bool hasValidFix, unsigned long dataAge) {
    if (!hasValidFix || satellites < GPS_MIN_SATELLITES ||
        hdop <= 0 || hdop > GPS_MAX_HDOP)
        return false;
    if (dataAge > GPS_DATA_MAX_AGE)
        return false;
    if (speed_kmh < 0 || speed_kmh > 20.0f)
        return false;
    if (speed_kmh < TRACTOR_CRAWL_THRESHOLD && satellites < 6)
        return false;
    return true;
}

void initTractorKalman(KalmanState &kalman) {
    kalman.speed = 0;
    kalman.accel = 0;
    kalman.P[0][0] = 0.5f;
    kalman.P[0][1] = 0;
    kalman.P[1][0] = 0;
    kalman.P[1][1] = 0.05f;
    kalman.lastUpdate = millis();
    kalman.initialized = true;
    for (int i = 0; i < 5; i++) kalman.speedBuffer[i] = 0;
    kalman.bufferIndex = 0;
    kalman.vibrationLevel = 0;
    kalman.inTurn = false;
    kalman.avgAccel = 0;
}

void detectTractorConditions(KalmanState &kalman, float currentSpeed) {
    kalman.speedBuffer[kalman.bufferIndex] = currentSpeed;
    kalman.bufferIndex = (kalman.bufferIndex + 1) % 5;

    float meanSpeed = 0;
    for (int i = 0; i < 5; i++) meanSpeed += kalman.speedBuffer[i];
    meanSpeed /= 5.0f;

    float variance = 0;
    for (int i = 0; i < 5; i++) variance += pow(kalman.speedBuffer[i] - meanSpeed, 2);
    kalman.vibrationLevel = sqrt(variance / 5.0f);

    kalman.inTurn = (fabs(kalman.accel) > TRACTOR_TURN_DETECTION);
    kalman.avgAccel = kalman.avgAccel * 0.9f + kalman.accel * 0.1f;
}

void tractorKalmanPredict(KalmanState &kalman, float dt) {
    if (!kalman.initialized || dt <= 0.001f || dt > 2.0f) return;

    float speedPred = kalman.speed + kalman.accel * dt;
    float accelDecay = max(0.98f, 1.0f - dt * 0.05f);
    float accelPred = kalman.accel * accelDecay;
    accelPred = constrain(accelPred, -2.0f, 2.0f);

    float P00 = kalman.P[0][0];
    float P01 = kalman.P[0][1];
    float P10 = kalman.P[1][0];
    float P11 = kalman.P[1][1];

    float currentSpeedKmh = kalman.speed * 3.6f;
    float adaptiveQ_speed = TRACTOR_KALMAN_Q_SPEED;
    float adaptiveQ_accel = TRACTOR_KALMAN_Q_ACCEL;

    if (currentSpeedKmh < TRACTOR_IDLE_THRESHOLD) {
        adaptiveQ_speed *= 0.3f;
        adaptiveQ_accel *= 0.2f;
    } else if (currentSpeedKmh < TRACTOR_CRAWL_THRESHOLD) {
        adaptiveQ_speed *= 0.5f;
        adaptiveQ_accel *= 0.4f;
    } else if (currentSpeedKmh < TRACTOR_WORKING_THRESHOLD) {
        adaptiveQ_speed *= 0.8f;
        adaptiveQ_accel *= 0.7f;
    }

    if (kalman.vibrationLevel > TRACTOR_VIBRATION_FILTER)
        adaptiveQ_speed *= 0.6f;
    if (kalman.inTurn)
        adaptiveQ_accel *= 2.0f;

    kalman.P[0][0] = P00 + dt * (P10 + P01) + dt * dt * P11 + adaptiveQ_speed;
    kalman.P[0][1] = P01 + dt * P11;
    kalman.P[1][0] = P10 + dt * P11;
    kalman.P[1][1] = P11 + adaptiveQ_accel;

    kalman.P[0][0] = max(kalman.P[0][0], 0.001f);
    kalman.P[1][1] = max(kalman.P[1][1], 0.001f);

    kalman.speed = constrain(speedPred, 0.0f, 20.0f / 3.6f);
    kalman.accel = accelPred;

    detectTractorConditions(kalman, kalman.speed);
}

void tractorKalmanUpdate(KalmanState &kalman, float measurement, float R_variance, float speedKmh) {
    if (!kalman.initialized) {
        kalman.speed = measurement;
        kalman.accel = 0;
        kalman.P[0][0] = 0.5f;
        kalman.P[0][1] = 0;
        kalman.P[1][0] = 0;
        kalman.P[1][1] = 0.05f;
        kalman.initialized = true;
        return;
    }

    float innovation = measurement - kalman.speed;
    float S = kalman.P[0][0] + R_variance;
    if (S < 1e-6f) S = 1e-6f;

    float K0 = kalman.P[0][0] / S;
    float K1 = kalman.P[1][0] / S;

    if (speedKmh < TRACTOR_IDLE_THRESHOLD) {
        K0 = constrain(K0 * 1.2f, 0.0f, 0.8f);
        K1 = constrain(K1 * 0.8f, -1.0f, 1.0f);
    } else if (speedKmh < TRACTOR_CRAWL_THRESHOLD) {
        K0 = constrain(K0, 0.0f, 0.7f);
        K1 = constrain(K1, -1.5f, 1.5f);
    } else {
        K0 = constrain(K0, 0.0f, 0.6f);
        K1 = constrain(K1, -1.0f, 1.0f);
    }

    if (kalman.vibrationLevel > TRACTOR_VIBRATION_FILTER) {
        K0 *= 0.8f;
        K1 *= 0.7f;
    }

    kalman.speed += K0 * innovation;
    kalman.accel += K1 * innovation;

    float P00 = kalman.P[0][0];
    float P01 = kalman.P[0][1];
    float P10 = kalman.P[1][0];
    float P11 = kalman.P[1][1];

    kalman.P[0][0] = (1.0f - K0) * P00;
    kalman.P[0][1] = (1.0f - K0) * P01;
    kalman.P[1][0] = P10 - K1 * P00;
    kalman.P[1][1] = P11 - K1 * P01;

    kalman.P[0][0] = max(kalman.P[0][0], 0.001f);
    kalman.P[1][1] = max(kalman.P[1][1], 0.001f);

    kalman.speed = constrain(kalman.speed, 0.0f, 20.0f / 3.6f);
    kalman.accel = constrain(kalman.accel, -2.0f, 2.0f);
}

float calculateTractorMeasurementNoise(int satellites, float hdop, float speedKmh, unsigned long dataAge, bool inField) {
    float R = TRACTOR_KALMAN_R_BASE;
    if (hdop > 0 && hdop < 10.0f) {
        R += (hdop - 1.0f) * 0.03f;
    }

    if (satellites < 6)
        R *= 1.8f;
    else if (satellites >= 8)
        R *= 0.7f;

    if (speedKmh < TRACTOR_IDLE_THRESHOLD)
        R *= 3.0f;
    else if (speedKmh < TRACTOR_CRAWL_THRESHOLD)
        R *= 1.5f;
    else if (speedKmh < TRACTOR_WORKING_THRESHOLD)
        R *= 0.9f;
    else
        R *= 1.1f;

    if (dataAge > 500)
        R *= (1.0f + dataAge / 3000.0f);

    if (inField)
        R += TRACTOR_IMPLEMENT_NOISE;

    return constrain(R, 0.02f, 2.0f);
}

void processTractorGPSMeasurement(KalmanState &kalman, ControlState &control,
                                  float rawSpeedKmh, int satellites, float hdop,
                                  bool hasValidFix, unsigned long dataAge, bool inField) {
    static unsigned long lastProcessTime = 0;
    unsigned long currentTime = millis();
    control.rawSpeedKmh = rawSpeedKmh;

    if (!isValidTractorGPSData(rawSpeedKmh, satellites, hdop, hasValidFix, dataAge)) {
        control.gpsLocked = false;
        return;
    }

    float rawSpeedMs = rawSpeedKmh / 3.6f;
    float dt = (currentTime - lastProcessTime) / 1000.0f;
    
    // FIXED: Remove recursive call and add proper condition
    if (dt > 0.01f && dt < 2.0f) {
        tractorKalmanPredict(kalman, dt);
    }

    float measurementNoise = calculateTractorMeasurementNoise(satellites, hdop,
                                                              rawSpeedKmh, dataAge, inField);
    tractorKalmanUpdate(kalman, rawSpeedMs, measurementNoise * measurementNoise, rawSpeedKmh);

    control.gpsLocked = true;
    control.satelliteCount = satellites;
    control.gpsHDOP = hdop;
    control.lastGPSUpdate = currentTime;
    kalman.lastUpdate = currentTime;
    lastProcessTime = currentTime;
}

void applyTractorPostProcessing(KalmanState &kalman, ControlState &control) {
    float speedKmh = kalman.speed * 3.6f;
    
    // FIXED: Reset speed to 0 only if very low, not just below threshold
    if (speedKmh < 0.5f) {  // Changed from TRACTOR_IDLE_THRESHOLD (1.0f)
        kalman.speed = 0;
        kalman.accel = 0;
        speedKmh = 0;
    }

    // FIXED: Simplified smoothing without sticky static variable
    static float lastFilteredSpeed = 0;
    static bool firstRun = true;
    
    if (firstRun) {
        lastFilteredSpeed = kalman.speed;
        firstRun = false;
    }
    
    // FIXED: Reduced alpha value for more responsive filtering
    float alpha = 0.3f;  // Changed from 0.8f to be more responsive
    if (speedKmh < TRACTOR_CRAWL_THRESHOLD) {
        alpha = 0.2f;  // Even more responsive for low speeds
    }
    
    // FIXED: Only apply smoothing if speed is above minimum threshold
    float smoothedSpeed;
    if (speedKmh > 0.1f) {
        smoothedSpeed = alpha * lastFilteredSpeed + (1.0f - alpha) * kalman.speed;
        lastFilteredSpeed = smoothedSpeed;
    } else {
        smoothedSpeed = 0;
        lastFilteredSpeed = 0;  // Reset the filter when stopped
    }

    control.filteredSpeed = smoothedSpeed;
    control.acceleration = kalman.accel;
    control.speedValid = true;

    static unsigned long lastDiagnostic = 0;
    if (millis() - lastDiagnostic > 12000) {
        Serial.print(F("ESP32 Tractor Filter - Vib: "));
        Serial.print(kalman.vibrationLevel, 3);
        Serial.print(F(", Turn: "));
        Serial.print(kalman.inTurn ? F("Y") : F("N"));
        Serial.print(F(", AvgAcc: "));
        Serial.print(kalman.avgAccel, 3);
        Serial.print(F(", Heap: "));
        Serial.println(ESP.getFreeHeap());
        lastDiagnostic = millis();
    }
}

// ADDED: Function to reset filter state (call this when you want to clear stuck values)
void resetKalmanFilter(KalmanState &kalman) {
    kalman.speed = 0;
    kalman.accel = 0;
    kalman.initialized = false;
    for (int i = 0; i < 5; i++) kalman.speedBuffer[i] = 0;
    kalman.bufferIndex = 0;
    kalman.vibrationLevel = 0;
    kalman.inTurn = false;
    kalman.avgAccel = 0;
    
   
    Serial.println("Kalman filter reset - clearing stuck values");
}