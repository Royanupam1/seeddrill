#ifndef FUZZYPID_H
#define FUZZYPID_H

#include "Config.h"

// ==================== FUZZY SETS DEFINITIONS ====================
enum FuzzyLevel {
    VERY_LOW = 0,
    FUZZY_LOW = 1,      // Changed from LOW to FUZZY_LOW
    MEDIUM = 2,
    FUZZY_HIGH = 3,     // Changed from HIGH to FUZZY_HIGH
    VERY_HIGH = 4
};

enum SoilCondition {
    SOIL_SANDY = 0,
    SOIL_LOAMY = 1,
    SOIL_CLAY = 2,
    SOIL_BLACK_COTTON = 3,
    SOIL_ALLUVIAL = 4,
    SOIL_RED_LATERITE = 5
};

enum SeasonType {
    SEASON_KHARIF = 0, // Monsoon (June-October)
    SEASON_RABI = 1,   // Winter (November-April)
    SEASON_ZAID = 2    // Summer (April-June)
};

// ==================== FUZZY PID CONTROLLER CLASS ====================
class FuzzyPIDController {
private:
    struct FuzzyParams {
        float kp_base = 1.2f;
        float ki_base = 0.25f;
        float kd_base = 0.08f;
        float last_error = 0.0f;
        float error_sum = 0.0f;
        unsigned long last_time = 0;
        
        // Environmental factors for Indian agriculture
        float soil_resistance = 1.0f; // 0.8-1.5 based on soil type
        float moisture_level = 1.0f; // 0.7-1.3 based on soil moisture
        float temperature_factor = 1.0f; // 0.9-1.1 based on ambient temp
        float crop_load_factor = 1.0f; // Crop-specific load adjustment
        float seasonal_factor = 1.0f; // Season-based adjustment
        
        // Adaptive learning parameters
        float adaptation_rate = 0.1f;
        float performance_history[10] = {0};
        int history_index = 0;
        bool learning_enabled = true;
    } params;
    
    // Fuzzy membership functions
    float triangular_membership(float x, float a, float b, float c);
    float trapezoidal_membership(float x, float a, float b, float c, float d);
    
    // Input classification
    FuzzyLevel classify_error(float error);
    FuzzyLevel classify_error_rate(float error_rate);
    FuzzyLevel classify_load(float load);
    SoilCondition classify_soil_condition();
    
    // Fuzzy rules for Indian agricultural conditions
    void apply_fuzzy_rules(FuzzyLevel error_level, FuzzyLevel error_rate_level,
                          FuzzyLevel load_level, SoilCondition soil_condition,
                          float& kp_adj, float& ki_adj, float& kd_adj);
    
    // Environmental adaptation
    void update_environmental_factors();
    void adapt_to_crop_type(CropType crop);
    void seasonal_adaptation(SeasonType season);
    
    // Performance monitoring and learning
    void update_performance_history(float error);
    void adaptive_learning();
    
public:
    FuzzyPIDController();
    
    // Main control function
    float calculate(float setpoint, float actual_value, float dt);
    
    // Configuration functions
    void configure_for_crop_type(CropType crop);
    void configure_for_soil_type(SoilCondition soil);
    void configure_for_season(SeasonType season);
    void set_environmental_sensors(float moisture, float temperature);
    
    // Tuning and diagnostics
    void auto_tune(float target_rpm, int duration_ms = 30000);
    void reset_learning();
    void enable_learning(bool enable);
    void print_diagnostics();
    
    // Preset configurations for Indian regions
    void load_punjab_preset(); // Wheat belt - heavy soils
    void load_maharashtra_preset(); // Cotton region - black cotton soil
    void load_up_preset(); // Mixed farming - alluvial soil
    void load_rajasthan_preset(); // Arid region - sandy soil
    void load_karnataka_preset(); // Deccan plateau - red laterite
    void load_wb_preset(); // Rice belt - clayey soil
};

// ==================== AGRICULTURAL OPTIMIZATION FUNCTIONS ====================
class AgriculturalOptimizer {
private:
    struct OptimizationParams {
        float fuel_cost_per_liter = 100.0f; // ₹/liter
        float labor_cost_per_hour = 300.0f; // ₹/hour
        float machinery_cost_per_hour = 500.0f; // ₹/hour
        float seed_wastage_penalty = 2.0f; // Multiplier for wasted seeds
        float time_penalty = 1.5f; // Penalty for slow operation
    } opt_params;
    
public:
    // Economic optimization
    float calculate_operating_cost(float speed_kmh, float spacing_accuracy,
                                  float fuel_rate, CropType crop);
    float calculate_roi_per_hectare(CropType crop, float yield_efficiency);
    
    // Agronomic optimization
    float optimal_speed_for_crop(CropType crop, SoilCondition soil, float moisture);
    float optimal_spacing_for_yield(CropType crop, float soil_fertility);
    
    // Field condition adaptations
    void adapt_for_slope(float slope_degrees);
    void adapt_for_residue_cover(float residue_percentage);
    void adapt_for_soil_compaction(float compaction_level);
    
    // Regional presets
    void optimize_for_region(const char* state_name);
};

// Global instances
extern FuzzyPIDController fuzzyController;
extern AgriculturalOptimizer agriOptimizer;

#endif // FUZZYPID_H
