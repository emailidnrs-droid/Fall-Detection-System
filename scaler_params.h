// ============================================
// SCALER PARAMETERS FOR FEATURE NORMALIZATION
// ============================================

#ifndef SCALER_PARAMS_H
#define SCALER_PARAMS_H

#include <Arduino.h>

class ScalerParams {
public:
    static const int NUM_FEATURES = 6;

    // Mean values for each feature
    static const float MEAN[];

    // Standard deviation values for each feature
    static const float STD[];

    // Normalize feature array
    static void normalize(float features[], float normalized[]) {
        for(int i = 0; i < NUM_FEATURES; i++) {
            normalized[i] = (features[i] - MEAN[i]) / STD[i];
        }
    }
};


const float ScalerParams::MEAN[6] = {0.985466, -0.063380, 0.064714, -1.896920, 0.863010, -1.089128};

const float ScalerParams::STD[6] = {0.055729, 0.146012, 0.160347, 2.069820, 2.814199, 2.484260};

#endif