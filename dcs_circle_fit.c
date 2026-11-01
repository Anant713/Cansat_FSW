#include "dcs_circle_fit.h"

#include <math.h>
#include <stddef.h>

#define DCS_CIRCLE_PI_F 3.14159265358979323846f
#define DCS_CIRCLE_TWO_PI_F (2.0f * DCS_CIRCLE_PI_F)

bool dcs_circle_fit_config_valid(const DcsCircleFitConfig *config)
{
    return config != NULL && config->minimum_samples >= 6u &&
        config->maximum_samples >= config->minimum_samples &&
        config->maximum_samples <= DCS_CIRCLE_FIT_MAX_SAMPLES &&
        isfinite(config->minimum_direction_coverage_rad) &&
        config->minimum_direction_coverage_rad > 0.0f &&
        config->minimum_direction_coverage_rad <= DCS_CIRCLE_TWO_PI_F &&
        isfinite(config->minimum_velocity_spread_mps) &&
        config->minimum_velocity_spread_mps > 0.0f &&
        isfinite(config->maximum_rms_fraction) &&
        config->maximum_rms_fraction > 0.0f &&
        isfinite(config->minimum_airspeed_mps) &&
        isfinite(config->maximum_airspeed_mps) &&
        config->minimum_airspeed_mps > 0.0f &&
        config->maximum_airspeed_mps > config->minimum_airspeed_mps;
}

bool dcs_circle_fit_init(DcsCircleFitState *state,
                         const DcsCircleFitConfig *config)
{
    if (state == NULL || !dcs_circle_fit_config_valid(config)) {
        return false;
    }
    *state = (DcsCircleFitState){0};
    return true;
}

bool dcs_circle_fit_add(DcsCircleFitState *state,
                        const DcsCircleFitConfig *config,
                        uint32_t gnss_epoch_ms,
                        float ground_v_north_mps,
                        float ground_v_east_mps)
{
    if (state == NULL || !dcs_circle_fit_config_valid(config) ||
        !isfinite(ground_v_north_mps) || !isfinite(ground_v_east_mps) ||
        state->sample_count >= config->maximum_samples ||
        (state->has_epoch && gnss_epoch_ms == state->last_epoch_ms)) {
        return false;
    }
    const uint8_t index = state->sample_count;
    state->north_mps[index] = ground_v_north_mps;
    state->east_mps[index] = ground_v_east_mps;
    state->sample_count++;
    state->last_epoch_ms = gnss_epoch_ms;
    state->has_epoch = true;
    return true;
}

static bool solve_3x3(float matrix[3][4], float answer[3])
{
    for (unsigned int column = 0u; column < 3u; ++column) {
        unsigned int pivot = column;
        for (unsigned int row = column + 1u; row < 3u; ++row) {
            if (fabsf(matrix[row][column]) > fabsf(matrix[pivot][column])) {
                pivot = row;
            }
        }
        if (!isfinite(matrix[pivot][column]) || fabsf(matrix[pivot][column]) < 1.0e-6f) {
            return false;
        }
        if (pivot != column) {
            for (unsigned int j = column; j < 4u; ++j) {
                const float temporary = matrix[column][j];
                matrix[column][j] = matrix[pivot][j];
                matrix[pivot][j] = temporary;
            }
        }
        const float divisor = matrix[column][column];
        for (unsigned int j = column; j < 4u; ++j) {
            matrix[column][j] /= divisor;
        }
        for (unsigned int row = 0u; row < 3u; ++row) {
            if (row == column) {
                continue;
            }
            const float factor = matrix[row][column];
            for (unsigned int j = column; j < 4u; ++j) {
                matrix[row][j] -= factor * matrix[column][j];
            }
        }
    }
    for (unsigned int i = 0u; i < 3u; ++i) {
        answer[i] = matrix[i][3];
    }
    return true;
}

static void sort_angles(float *angles, uint8_t count)
{
    for (uint8_t i = 1u; i < count; ++i) {
        const float value = angles[i];
        uint8_t j = i;
        while (j > 0u && angles[j - 1u] > value) {
            angles[j] = angles[j - 1u];
            --j;
        }
        angles[j] = value;
    }
}

bool dcs_circle_fit_solve(const DcsCircleFitState *state,
                          const DcsCircleFitConfig *config,
                          DcsCircleFitResult *result)
{
    if (state == NULL || result == NULL || !dcs_circle_fit_config_valid(config) ||
        state->sample_count < config->minimum_samples) {
        return false;
    }

    const uint8_t count = state->sample_count;
    float mean_n = 0.0f, mean_e = 0.0f;
    for (uint8_t i = 0u; i < count; ++i) {
        mean_n += state->north_mps[i];
        mean_e += state->east_mps[i];
    }
    mean_n /= (float)count;
    mean_e /= (float)count;

    float matrix[3][4] = {{0}};
    float spread_squared_sum = 0.0f;
    for (uint8_t i = 0u; i < count; ++i) {
        const float n = state->north_mps[i] - mean_n;
        const float e = state->east_mps[i] - mean_e;
        const float squared = n * n + e * e;
        const float row[3] = {n, e, 1.0f};
        spread_squared_sum += squared;
        for (unsigned int r = 0u; r < 3u; ++r) {
            for (unsigned int c = 0u; c < 3u; ++c) {
                matrix[r][c] += row[r] * row[c];
            }
            matrix[r][3] += row[r] * squared;
        }
    }
    const float spread = sqrtf(spread_squared_sum / (float)count);
    if (!isfinite(spread) || spread < config->minimum_velocity_spread_mps) {
        return false;
    }

    float coefficients[3];
    if (!solve_3x3(matrix, coefficients)) {
        return false;
    }
    const float centre_n_local = 0.5f * coefficients[0];
    const float centre_e_local = 0.5f * coefficients[1];
    const float radius_squared = coefficients[2] +
        centre_n_local * centre_n_local + centre_e_local * centre_e_local;
    if (!isfinite(radius_squared) || radius_squared <= 0.0f) {
        return false;
    }
    const float radius = sqrtf(radius_squared);
    const float wind_n = mean_n + centre_n_local;
    const float wind_e = mean_e + centre_e_local;
    if (!isfinite(wind_n) || !isfinite(wind_e) ||
        radius < config->minimum_airspeed_mps ||
        radius > config->maximum_airspeed_mps) {
        return false;
    }

    float residual_sum = 0.0f;
    float angles[DCS_CIRCLE_FIT_MAX_SAMPLES];
    for (uint8_t i = 0u; i < count; ++i) {
        const float dn = state->north_mps[i] - wind_n;
        const float de = state->east_mps[i] - wind_e;
        const float residual = hypotf(dn, de) - radius;
        residual_sum += residual * residual;
        float angle = atan2f(de, dn);
        if (angle < 0.0f) {
            angle += DCS_CIRCLE_TWO_PI_F;
        }
        angles[i] = angle;
    }
    sort_angles(angles, count);
    float largest_gap = angles[0] + DCS_CIRCLE_TWO_PI_F - angles[count - 1u];
    for (uint8_t i = 1u; i < count; ++i) {
        largest_gap = fmaxf(largest_gap, angles[i] - angles[i - 1u]);
    }
    const float coverage = DCS_CIRCLE_TWO_PI_F - largest_gap;
    const float rms = sqrtf(residual_sum / (float)count);
    if (!isfinite(coverage) || !isfinite(rms) ||
        coverage < config->minimum_direction_coverage_rad ||
        rms > config->maximum_rms_fraction * radius) {
        return false;
    }

    *result = (DcsCircleFitResult){
        .wind_north_mps = wind_n,
        .wind_east_mps = wind_e,
        .maneuver_airspeed_mps = radius,
        .radial_rms_mps = rms,
        .direction_coverage_rad = coverage,
        .sample_count = count,
    };
    return true;
}
