#include "FuzzyPID.h"
#include "Config.h"
#include <Arduino.h>

// Global instances
FuzzyPIDController fuzzyController;
AgriculturalOptimizer agriOptimizer;

// ==================== FUZZY PID CONTROLLER IMPLEMENTATION ====================

FuzzyPIDController::FuzzyPIDController() {
    // Initialize with default Indian agriculture values
    params.kp_base = 1.2f;
    params.ki_base = 0.25f;
    params.kd_base = 0.08f;
    params.last_error = 0.0f;
    params.error_sum = 0.0f;
    params.last_time = 0;
    
    // Environmental factors for Indian conditions
    params.soil_resistance = 1.0f;
    params.moisture_level = 1.0f;
    params.temperature_factor = 1.0f;
    params.crop_load_factor = 1.0f;
    params.seasonal_factor = 1.0f;
    
    // Adaptive learning
    params.adaptation_rate = 0.1f;
    for (int i = 0; i < 10; i++) {
        params.performance_history[i] = 0;
    }
    params.history_index = 0;
    params.learning_enabled = true;
}

float FuzzyPIDController::triangular_membership(float x, float a, float b, float c) {
    if (x <= a || x >= c) return 0.0f;
    if (x == b) return 1.0f;
    if (x < b) return (x - a) / (b - a);
    return (c - x) / (c - b);
}

float FuzzyPIDController::trapezoidal_membership(float x, float a, float b, float c, float d) {
    if (x <= a || x >= d) return 0.0f;
    if (x >= b && x <= c) return 1.0f;
    if (x < b) return (x - a) / (b - a);
    return (d - x) / (d - c);
}

FuzzyLevel FuzzyPIDController::classify_error(float error) {
    float abs_error = fabs(error);
    if (abs_error < 5.0f) return VERY_LOW;
    else if (abs_error < 15.0f) return FUZZY_LOW;
    else if (abs_error < 30.0f) return MEDIUM;
    else if (abs_error < 50.0f) return FUZZY_HIGH;
    else return VERY_HIGH;
}

FuzzyLevel FuzzyPIDController::classify_error_rate(float error_rate) {
    float abs_rate = fabs(error_rate);
    if (abs_rate < 2.0f) return VERY_LOW;
    else if (abs_rate < 8.0f) return FUZZY_LOW;
    else if (abs_rate < 20.0f) return MEDIUM;
    else if (abs_rate < 40.0f) return FUZZY_HIGH;
    else return VERY_HIGH;
}

FuzzyLevel FuzzyPIDController::classify_load(float load) {
    if (load < 0.7f) return VERY_LOW;
    else if (load < 0.9f) return FUZZY_LOW;
    else if (load < 1.1f) return MEDIUM;
    else if (load < 1.3f) return FUZZY_HIGH;
    else return VERY_HIGH;
}

void FuzzyPIDController::apply_fuzzy_rules(FuzzyLevel error_level, FuzzyLevel error_rate_level,
                                          FuzzyLevel load_level, SoilCondition soil_condition,
                                          float& kp_adj, float& ki_adj, float& kd_adj) {
    // Initialize adjustments
    kp_adj = 1.0f;
    ki_adj = 1.0f;
    kd_adj = 1.0f;
    
    // Basic fuzzy rules for PID tuning
    // Rule 1: If error is VERY_HIGH, increase Kp
    if (error_level == VERY_HIGH) {
        kp_adj *= 1.5f;
        ki_adj *= 0.8f;
    }
    
    // Rule 2: If error is VERY_LOW, reduce Kp
    if (error_level == VERY_LOW) {
        kp_adj *= 0.8f;
        ki_adj *= 1.2f;
    }
    
    // Rule 3: If error_rate is HIGH, increase Kd
    if (error_rate_level == FUZZY_HIGH || error_rate_level == VERY_HIGH) {
        kd_adj *= 1.3f;
    }
    
    // Rule 4: Soil-specific adjustments
    switch (soil_condition) {
        case SOIL_CLAY:
            kp_adj *= 1.1f; // Clay needs more aggressive control
            ki_adj *= 0.9f;
            break;
        case SOIL_SANDY:
            kp_adj *= 0.9f; // Sandy soil responds quickly
            ki_adj *= 1.1f;
            break;
        case SOIL_BLACK_COTTON:
            kp_adj *= 1.2f; // Black cotton soil is heavy
            ki_adj *= 0.8f;
            kd_adj *= 1.1f;
            break;
        default:
            break;
    }
    
    // Rule 5: Load-based adjustments
    if (load_level == FUZZY_HIGH || load_level == VERY_HIGH) {
        kp_adj *= 1.2f;
        ki_adj *= 1.1f;
    }
    
    // Constrain adjustments to reasonable ranges
    kp_adj = constrain(kp_adj, 0.5f, 2.0f);
    ki_adj = constrain(ki_adj, 0.5f, 2.0f);
    kd_adj = constrain(kd_adj, 0.5f, 2.0f);
}

float FuzzyPIDController::calculate(float setpoint, float actual_value, float dt) {
    if (dt <= 0.001f) dt = 0.02f; // Default 20ms
    
    float error = setpoint - actual_value;
    float error_rate = (error - params.last_error) / dt;
    
    // Classify inputs
    FuzzyLevel error_level = classify_error(error);
    FuzzyLevel error_rate_level = classify_error_rate(error_rate);
    FuzzyLevel load_level = classify_load(params.crop_load_factor);
    
    // Get fuzzy adjustments
    float kp_adj, ki_adj, kd_adj;
    apply_fuzzy_rules(error_level, error_rate_level, load_level, SOIL_LOAMY,
                     kp_adj, ki_adj, kd_adj);
    
    // Apply environmental factors
    float kp = params.kp_base * kp_adj * params.temperature_factor * params.seasonal_factor;
    float ki = params.ki_base * ki_adj * params.moisture_level;
    float kd = params.kd_base * kd_adj * params.soil_resistance;
    
    // PID calculation
    params.error_sum += error * dt;
    params.error_sum = constrain(params.error_sum, -100.0f, 100.0f); // Windup prevention
    
    float P = kp * error;
    float I = ki * params.error_sum;
    float D = kd * error_rate;
    
    float output = P + I + D;
    
    // Update for next iteration
    params.last_error = error;
    params.last_time = millis();
    
    // Update performance history for learning
    update_performance_history(fabs(error));
    
    return constrain(output, 0, 255);
}

void FuzzyPIDController::configure_for_crop_type(CropType crop) {
    switch (crop) {
        case CROP_WHEAT:
            params.crop_load_factor = 1.0f;
            params.kp_base = 1.2f;
            break;
        case CROP_COTTON:
            params.crop_load_factor = 1.3f; // Cotton requires precision
            params.kp_base = 1.4f;
            break;
        case CROP_MUSTARD:
            params.crop_load_factor = 0.8f; // Small seeds, lighter load
            params.kp_base = 1.0f;
            break;
        case CROP_SOYBEAN:
            params.crop_load_factor = 1.1f; // Medium precision needed
            params.kp_base = 1.2f;
            break;
        case CROP_GROUNDNUT:
            params.crop_load_factor = 1.2f; // Larger seeds
            params.kp_base = 1.3f;
            break;
        case CROP_SUNFLOWER:
            params.crop_load_factor = 1.1f; // Large seeds
            params.kp_base = 1.2f;
            break;
        case CROP_MAIZE:
            params.crop_load_factor = 1.4f; // Large seeds, high precision
            params.kp_base = 1.4f;
            break;
        case CROP_CHICKPEA:
            params.crop_load_factor = 1.0f; // Medium seeds
            params.kp_base = 1.1f;
            break;
        default:
            params.crop_load_factor = 1.0f;
            params.kp_base = 1.2f;
            break;
    }
}

void FuzzyPIDController::configure_for_soil_type(SoilCondition soil) {
    switch (soil) {
        case SOIL_CLAY:
            params.soil_resistance = 1.3f;
            break;
        case SOIL_SANDY:
            params.soil_resistance = 0.8f;
            break;
        case SOIL_BLACK_COTTON:
            params.soil_resistance = 1.5f;
            break;
        case SOIL_ALLUVIAL:
            params.soil_resistance = 1.1f;
            break;
        case SOIL_RED_LATERITE:
            params.soil_resistance = 1.2f;
            break;
        default: // SOIL_LOAMY
            params.soil_resistance = 1.0f;
            break;
    }
}

void FuzzyPIDController::configure_for_season(SeasonType season) {
    switch (season) {
        case SEASON_KHARIF:
            params.seasonal_factor = 1.1f; // Monsoon conditions
            params.moisture_level = 1.2f;
            break;
        case SEASON_RABI:
            params.seasonal_factor = 1.0f; // Optimal conditions
            params.moisture_level = 1.0f;
            break;
        case SEASON_ZAID:
            params.seasonal_factor = 0.9f; // Summer, dry conditions
            params.moisture_level = 0.8f;
            break;
    }
}

void FuzzyPIDController::set_environmental_sensors(float moisture, float temperature) {
    params.moisture_level = 1.0f + (moisture - 50.0f) / 100.0f; // Normalize around 50%
    params.moisture_level = constrain(params.moisture_level, 0.7f, 1.3f);
    
    params.temperature_factor = 1.0f - (temperature - 25.0f) / 50.0f; // Normalize around 25°C
    params.temperature_factor = constrain(params.temperature_factor, 0.9f, 1.1f);
}

void FuzzyPIDController::auto_tune(float target_rpm, int duration_ms) {
    Serial.println(F("[FUZZY] Starting auto-tune process..."));
    // Simplified auto-tune - could be enhanced
    Serial.printf("[FUZZY] Auto-tuning for %.0f RPM over %d ms\n", target_rpm, duration_ms);
}

void FuzzyPIDController::reset_learning() {
    for (int i = 0; i < 10; i++) {
        params.performance_history[i] = 0;
    }
    params.history_index = 0;
}

void FuzzyPIDController::enable_learning(bool enable) {
    params.learning_enabled = enable;
}

void FuzzyPIDController::print_diagnostics() {
    Serial.println(F("\n=== FUZZY PID DIAGNOSTICS ==="));
    Serial.printf("Base Parameters: Kp=%.3f, Ki=%.3f, Kd=%.3f\n",
                 params.kp_base, params.ki_base, params.kd_base);
    Serial.printf("Environmental Factors:\n");
    Serial.printf(" Soil Resistance: %.2f\n", params.soil_resistance);
    Serial.printf(" Moisture Level: %.2f\n", params.moisture_level);
    Serial.printf(" Temperature Factor: %.2f\n", params.temperature_factor);
    Serial.printf(" Crop Load Factor: %.2f\n", params.crop_load_factor);
    Serial.printf(" Seasonal Factor: %.2f\n", params.seasonal_factor);
    Serial.printf("Learning: %s\n", params.learning_enabled ? "ENABLED" : "DISABLED");
    Serial.println(F("==============================\n"));
}

void FuzzyPIDController::update_performance_history(float error) {
    if (!params.learning_enabled) return;
    
    params.performance_history[params.history_index] = error;
    params.history_index = (params.history_index + 1) % 10;
}

// Regional preset implementations
void FuzzyPIDController::load_punjab_preset() {
    // Wheat belt - heavy soils, high moisture
    configure_for_soil_type(SOIL_CLAY);
    configure_for_season(SEASON_RABI);
    params.moisture_level = 1.1f;
    params.soil_resistance = 1.3f;
}

void FuzzyPIDController::load_maharashtra_preset() {
    // Cotton region - black cotton soil
    configure_for_soil_type(SOIL_BLACK_COTTON);
    configure_for_season(SEASON_KHARIF);
    params.moisture_level = 1.0f;
    params.soil_resistance = 1.5f;
}

void FuzzyPIDController::load_up_preset() {
    // Mixed farming - alluvial soil
    configure_for_soil_type(SOIL_ALLUVIAL);
    configure_for_season(SEASON_RABI);
    params.moisture_level = 1.0f;
    params.soil_resistance = 1.1f;
}

void FuzzyPIDController::load_rajasthan_preset() {
    // Arid region - sandy soil
    configure_for_soil_type(SOIL_SANDY);
    configure_for_season(SEASON_ZAID);
    params.moisture_level = 0.8f;
    params.soil_resistance = 0.8f;
}

void FuzzyPIDController::load_karnataka_preset() {
    // Deccan plateau - red laterite
    configure_for_soil_type(SOIL_RED_LATERITE);
    configure_for_season(SEASON_KHARIF);
    params.moisture_level = 0.9f;
    params.soil_resistance = 1.2f;
}

void FuzzyPIDController::load_wb_preset() {
    // Rice belt - clayey soil
    configure_for_soil_type(SOIL_CLAY);
    configure_for_season(SEASON_KHARIF);
    params.moisture_level = 1.3f;
    params.soil_resistance = 1.3f;
}

// ==================== AGRICULTURAL OPTIMIZER IMPLEMENTATION ====================

float AgriculturalOptimizer::calculate_operating_cost(float speed_kmh, float spacing_accuracy,
                                                     float fuel_rate, CropType crop) {
    float base_cost = opt_params.fuel_cost_per_liter * fuel_rate;
    float labor_cost = opt_params.labor_cost_per_hour * (10.0f / speed_kmh); // 10 ha field
    float machinery_cost = opt_params.machinery_cost_per_hour * (10.0f / speed_kmh);
    
    // Penalty for poor spacing accuracy
    float spacing_penalty = (100.0f - spacing_accuracy) * opt_params.seed_wastage_penalty;
    
    return base_cost + labor_cost + machinery_cost + spacing_penalty;
}

float AgriculturalOptimizer::calculate_roi_per_hectare(CropType crop, float yield_efficiency) {
    extern const CropConfiguration cropConfigs[CROP_COUNT];
    float expected_yield = cropConfigs[crop].expectedYield * (yield_efficiency / 100.0f);
    
    // Simplified price per kg (INR)
    float price_per_kg = 20.0f; // Default price
    switch (crop) {
        case CROP_WHEAT: price_per_kg = 25.0f; break;
        case CROP_COTTON: price_per_kg = 60.0f; break;
        case CROP_SOYBEAN: price_per_kg = 40.0f; break;
        case CROP_GROUNDNUT: price_per_kg = 55.0f; break;
        case CROP_MUSTARD: price_per_kg = 50.0f; break;
        case CROP_SUNFLOWER: price_per_kg = 45.0f; break;
        case CROP_MAIZE: price_per_kg = 22.0f; break;
        case CROP_CHICKPEA: price_per_kg = 70.0f; break;
        default: break;
    }
    
    return expected_yield * price_per_kg;
}

float AgriculturalOptimizer::optimal_speed_for_crop(CropType crop, SoilCondition soil, float moisture) {
    // Define optimal speeds based on crop characteristics and target spacing
    // Smaller spacing requires slower speed for precision
    float base_speed;
    
    switch (crop) {
        case CROP_WHEAT:
            base_speed = 6.0f; // Medium precision needed
            break;
        case CROP_COTTON:
            base_speed = 4.0f; // Wide spacing, high precision needed
            break;
        case CROP_MUSTARD:
            base_speed = 5.0f; // Small seeds, close spacing
            break;
        case CROP_SOYBEAN:
            base_speed = 5.5f; // Medium spacing
            break;
        case CROP_GROUNDNUT:
            base_speed = 5.0f; // Medium-close spacing
            break;
        case CROP_SUNFLOWER:
            base_speed = 4.5f; // Wide spacing, precision needed
            break;
        case CROP_MAIZE:
            base_speed = 4.0f; // Wide spacing, large seeds
            break;
        case CROP_CHICKPEA:
            base_speed = 5.5f; // Medium spacing
            break;
        default:
            base_speed = 5.0f;
            break;
    }
    
    // Adjust for soil conditions
    switch (soil) {
        case SOIL_CLAY:
            base_speed *= 0.9f; // Clay needs slower operation
            break;
        case SOIL_SANDY:
            base_speed *= 1.1f; // Sandy soil allows faster operation
            break;
        case SOIL_BLACK_COTTON:
            base_speed *= 0.8f; // Heavy soil needs slow operation
            break;
        case SOIL_ALLUVIAL:
            base_speed *= 1.0f; // Balanced soil
            break;
        case SOIL_RED_LATERITE:
            base_speed *= 0.95f; // Slightly slower for laterite
            break;
        default:
            break;
    }
    
    // Adjust for moisture
    if (moisture > 70.0f) {
        base_speed *= 0.85f; // Wet conditions - slower operation
    } else if (moisture < 30.0f) {
        base_speed *= 1.05f; // Dry conditions - can go slightly faster
    }
    
    return constrain(base_speed, MIN_OPERATING_SPEED, MAX_OPERATING_SPEED);
}

float AgriculturalOptimizer::optimal_spacing_for_yield(CropType crop, float soil_fertility) {
    extern const CropConfiguration cropConfigs[CROP_COUNT];
    float base_spacing = cropConfigs[crop].targetSpacing;
    
    // Adjust for soil fertility (0-100%)
    if (soil_fertility > 80.0f) {
        base_spacing *= 1.05f; // Fertile soil - can space wider
    } else if (soil_fertility < 40.0f) {
        base_spacing *= 0.95f; // Poor soil - space closer
    }
    
    return base_spacing;
}

void AgriculturalOptimizer::adapt_for_slope(float slope_degrees) {
    // Implementation for slope adaptation
    // Adjust speed and spacing based on field slope
    if (slope_degrees > 5.0f) {
        // Steeper slopes need slower operation
        opt_params.time_penalty *= 1.2f;
    }
}

void AgriculturalOptimizer::adapt_for_residue_cover(float residue_percentage) {
    // Implementation for residue cover adaptation
    // Higher residue cover affects seeding depth and speed
    if (residue_percentage > 50.0f) {
        opt_params.seed_wastage_penalty *= 1.1f;
    }
}

void AgriculturalOptimizer::adapt_for_soil_compaction(float compaction_level) {
   
    if (compaction_level > 70.0f) {
        opt_params.machinery_cost_per_hour *= 1.15f;
    }
}

void AgriculturalOptimizer::optimize_for_region(const char* state_name) {
    // Regional optimization based on local conditions
    if (strcmp(state_name, "punjab") == 0) {
        opt_params.fuel_cost_per_liter = 95.0f;
        opt_params.labor_cost_per_hour = 350.0f;
    } else if (strcmp(state_name, "maharashtra") == 0) {
        opt_params.fuel_cost_per_liter = 105.0f;
        opt_params.labor_cost_per_hour = 400.0f;
    } else if (strcmp(state_name, "rajasthan") == 0) {
        opt_params.fuel_cost_per_liter = 98.0f;
        opt_params.labor_cost_per_hour = 280.0f;
    } else if (strcmp(state_name, "karnataka") == 0) {
        opt_params.fuel_cost_per_liter = 102.0f;
        opt_params.labor_cost_per_hour = 320.0f;
    } else if (strcmp(state_name, "up") == 0) {
        opt_params.fuel_cost_per_liter = 92.0f;
        opt_params.labor_cost_per_hour = 300.0f;
    } else if (strcmp(state_name, "wb") == 0) {
        opt_params.fuel_cost_per_liter = 96.0f;
        opt_params.labor_cost_per_hour = 280.0f;
    }
}