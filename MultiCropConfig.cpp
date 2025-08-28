
#include "Config.h"


const CropConfiguration cropConfigs[CROP_COUNT] = {
    // WHEAT
    {
        .name = "Wheat",
        .targetSpacing = 0.15f,    
        .seedsPerKg = 25000,
        .season = "R",             
        .expectedYield = 4500.0f,
        .icon = "W"                
    },
    
    // SOYBEAN
    {
        .name = "Soybean", 
        .targetSpacing = 0.20f,
        .seedsPerKg = 6000,
        .season = "K",
        .expectedYield = 2500.0f,
        .icon = "S"
    },
    
    // COTTON
    {
        .name = "Cotton",
        .targetSpacing = 0.30f,
        .seedsPerKg = 8000,
        .season = "K", 
        .expectedYield = 1800.0f,
        .icon = "C"
    },
    
    // GROUNDNUT
    {
        .name = "Groundnut",
        .targetSpacing = 0.18f,
        .seedsPerKg = 2200,
        .season = "K",
        .expectedYield = 2200.0f,
        .icon = "G"
    },
    
    // MUSTARD
    {
        .name = "Mustard",
        .targetSpacing = 0.12f,
        .seedsPerKg = 500000,
        .season = "R",
        .expectedYield = 1500.0f,
        .icon = "M"
    },
    
    // SUNFLOWER
    {
        .name = "Sunflower",
        .targetSpacing = 0.25f,
        .seedsPerKg = 25000,
        .season = "B",             // B=Both seasons
        .expectedYield = 2000.0f,
        .icon = "F"
    },
    
    // MAIZE
    {
        .name = "Maize",
        .targetSpacing = 0.22f,
        .seedsPerKg = 3500,
        .season = "K",
        .expectedYield = 6000.0f,
        .icon = "Z"
    },
    
    // CHICKPEA
    {
        .name = "Chickpea",
        .targetSpacing = 0.16f,
        .seedsPerKg = 2000,
        .season = "R",
        .expectedYield = 2800.0f,
        .icon = "P"
    }
};