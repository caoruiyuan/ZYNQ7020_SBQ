#include "q3_analysis.h"
#include "q3_calibration.h"

#include <float.h>
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define Q3_ANALYSIS_MAX_FFT_SLOTS       6650U
#define Q3_ANALYSIS_MAX_DISPLAY_POINTS  128U
#define Q3_ANALYSIS_MAX_CANDIDATE_PEAKS 24U
#define Q3_ANALYSIS_PEAK_GUARD_BINS     8U
#define Q3_ANALYSIS_NOISE_MAG_RATIO     12U
#define Q3_ANALYSIS_FUNDAMENTAL_POWER_GAP 400U
#define Q3_ANALYSIS_HARMONIC_TOLERANCE_BINS 2.0
#define Q3_ANALYSIS_MAX_HARMONIC_ORDER  64U
#define Q3_ANALYSIS_PHASE_BLOCK         4096U
#define Q3_ANALYSIS_PHASE_ITERATIONS    2U
#define Q3_ANALYSIS_RECONSTRUCT_POINTS  8192U
#define Q3_ANALYSIS_MIN_PEAK_VOLTS      0.0005
#define Q3_ANALYSIS_RAW_SAMPLE_RATE_HZ  65000000.0
#define Q3_ANALYSIS_DECIMATION          8.0
#define Q3_ANALYSIS_MAX_MATRIX          \
    (1U + 2U * Q3_ANALYSIS_MAX_COMPONENTS)
#define Q3_ANALYSIS_PI                  3.14159265358979323846
#define Q3_ANALYSIS_TWO_PI              6.28318530717958647692

typedef struct {
    uint32_t slot;
    uint64_t power;
    double bin;
} q3_peak_t;

typedef struct {
    double cosine[Q3_ANALYSIS_MAX_COMPONENTS];
    double sine[Q3_ANALYSIS_MAX_COMPONENTS];
    double amplitude[Q3_ANALYSIS_MAX_COMPONENTS];
    double phase[Q3_ANALYSIS_MAX_COMPONENTS];
} q3_fit_t;

typedef struct {
    double cosine;
    double sine;
    double step_cosine;
    double step_sine;
    double omega;
} q3_oscillator_t;

/*
 * Core 1 calls the analysis synchronously, so a static work area avoids
 * placing two 32 KiB arrays on its 8 KiB stack.
 */
static uint64_t SpectrumPower[Q3_ANALYSIS_MAX_FFT_SLOTS];
static uint64_t MedianScratch[Q3_ANALYSIS_MAX_FFT_SLOTS];
static uint32_t ComponentThresholdTenthsPercent =
    Q3_ANALYSIS_COMPONENT_THRESHOLD_DEFAULT_TENTHS_PERCENT;

int q3_analysis_set_component_threshold_tenths_percent(
    uint32_t tenths_percent)
{
    if (!Q3_ANALYSIS_COMPONENT_THRESHOLD_TENTHS_PERCENT_IS_VALID(
            tenths_percent)) {
        return -1;
    }
    ComponentThresholdTenthsPercent = tenths_percent;
    return 0;
}

uint32_t q3_analysis_get_component_threshold_tenths_percent(void)
{
    return ComponentThresholdTenthsPercent;
}

/* The FIR coefficient table must match the regenerated q3_fir_0 IP. */
static const double FirHalfCoefficients[81] = {
    -8.8203359691335000123e-07,
    -3.0544597292882801119e-06,
    -7.0879791223294001182e-06,
    -1.3093148306798189375e-05,
    -2.0393245354074628507e-05,
    -2.7287568737497880719e-05,
    -3.1116150903397936725e-05,
    -2.8756009167905838584e-05,
    -1.7562087835787619494e-05,
    3.4061605144928899679e-06,
    3.2212108144388432564e-05,
    6.3384373930618417977e-05,
    8.8230024176383714594e-05,
    9.6397996616819282183e-05,
    7.8630342319393979459e-05,
    3.0198854265829009187e-05,
    -4.58892232639610372e-05,
    -0.00013706668296734877749,
    -0.0002218299816355498574,
    -0.00027340894866271261672,
    -0.00026626954711780030519,
    -0.00018425526264812238449,
    -2.8296635736663519324e-05,
    0.00017883714345358882,
    0.00039304549349818276383,
    0.00055595115626704699491,
    0.00060811135181691484009,
    0.00050600921494787208261,
    0.00023870371168419071369,
    -0.00016089214791651198274,
    -0.00061473159160930373503,
    -0.0010110883584329530705,
    -0.0012283423927164816188,
    -0.0011669818873472801671,
    -0.00078240709301011397963,
    -0.00010930158967589397262,
    0.00073116277449670954037,
    0.0015468819484093763745,
    0.0021133515406815267523,
    0.0022288987212192722283,
    0.0017744010635017102868,
    0.00076170198383066324913,
    -0.00064483501230738868477,
    -0.0021445367938944066497,
    -0.0033567665367307962694,
    -0.0039090989192982076098,
    -0.0035388227116730313314,
    -0.002182470640526586788,
    -2.7107647025127899531e-05,
    0.0024968366502250752791,
    0.0047888887196888448028,
    0.0062091425840290646529,
    0.0062405741515847972661,
    0.0046450615939388987391,
    0.0015709727672268670139,
    -0.00242341420062779523,
    -0.0064496597466952134206,
    -0.0094670107335204957411,
    -0.010527163923732056963,
    -0.0090331125868753636698,
    -0.0049482044687076780221,
    0.0011047420350563156265,
    0.0078950843206841234828,
    0.013795651123438657654,
    0.017124552857810625833,
    0.01655298826630903844,
    0.011490733725370301521,
    0.0023557648002908945263,
    -0.0093491343300929272142,
    -0.021193755849713301342,
    -0.030203705679879466056,
    -0.03340968750069520099,
    -0.028460621020488706401,
    -0.014175669654109513199,
    0.0090862539185235721706,
    0.039330411796490827303,
    0.073142295740141582971,
    0.10619118552449691162,
    0.13393566632491976653,
    0.15240054322580448565,
    0.15887581178456486319
};

static double FilteredChainMagnitude(double frequency_hz)
{
    double cic_magnitude = 1.0;
    double fir_magnitude;
    const double decimated_omega =
        Q3_ANALYSIS_TWO_PI * frequency_hz /
        (Q3_ANALYSIS_RAW_SAMPLE_RATE_HZ /
         Q3_ANALYSIS_DECIMATION);
    uint32_t coefficient;

    if (frequency_hz > 0.0) {
        const double raw_omega =
            Q3_ANALYSIS_TWO_PI * frequency_hz /
            Q3_ANALYSIS_RAW_SAMPLE_RATE_HZ;
        const double denominator =
            Q3_ANALYSIS_DECIMATION * sin(raw_omega / 2.0);

        if (fabs(denominator) > DBL_EPSILON) {
            const double stage_magnitude =
                fabs(sin(raw_omega *
                         Q3_ANALYSIS_DECIMATION / 2.0) /
                     denominator);

            cic_magnitude =
                stage_magnitude * stage_magnitude *
                stage_magnitude * stage_magnitude;
        }
    }

    fir_magnitude = FirHalfCoefficients[80];
    for (coefficient = 0U; coefficient < 80U; ++coefficient) {
        fir_magnitude +=
            2.0 * FirHalfCoefficients[coefficient] *
            cos(decimated_omega *
                (double)(80U - coefficient));
    }
    fir_magnitude = fabs(fir_magnitude);

    {
        const double magnitude =
            cic_magnitude * fir_magnitude;

        return (magnitude > 1.0e-12) ? magnitude : 1.0;
    }
}

static uint64_t PowerAt(const uint32_t *fft_words, uint32_t slot)
{
    const int32_t real = (int32_t)fft_words[slot * 2U];
    const int32_t imaginary = (int32_t)fft_words[slot * 2U + 1U];

    return (uint64_t)((int64_t)real * real) +
           (uint64_t)((int64_t)imaginary * imaginary);
}

static uint64_t SelectMedian(const uint64_t *values, uint32_t count)
{
    int32_t left = 0;
    int32_t right = (int32_t)count - 1;
    const int32_t wanted = (int32_t)(count / 2U);

    (void)memcpy(MedianScratch, values,
                 (size_t)count * sizeof(MedianScratch[0]));
    while (left < right) {
        int32_t lower = left;
        int32_t upper = right;
        const uint64_t pivot =
            MedianScratch[(left + right) / 2];

        while (lower <= upper) {
            while (MedianScratch[lower] < pivot) {
                ++lower;
            }
            while (MedianScratch[upper] > pivot) {
                --upper;
            }
            if (lower <= upper) {
                const uint64_t temporary = MedianScratch[lower];

                MedianScratch[lower] = MedianScratch[upper];
                MedianScratch[upper] = temporary;
                ++lower;
                --upper;
            }
        }
        if (wanted <= upper) {
            right = upper;
        } else if (wanted >= lower) {
            left = lower;
        } else {
            break;
        }
    }
    return MedianScratch[wanted];
}

static int IsSeparated(uint32_t slot, const q3_peak_t *selected,
                       uint32_t selected_count)
{
    uint32_t index;

    for (index = 0U; index < selected_count; ++index) {
        const uint32_t distance =
            (slot > selected[index].slot) ?
                (slot - selected[index].slot) :
                (selected[index].slot - slot);

        if (distance <= Q3_ANALYSIS_PEAK_GUARD_BINS) {
            return 0;
        }
    }
    return 1;
}

static double LogParabolicOffset(uint64_t previous, uint64_t peak,
                                 uint64_t next)
{
    const double previous_log = log((double)previous + 1.0);
    const double peak_log = log((double)peak + 1.0);
    const double next_log = log((double)next + 1.0);
    const double denominator =
        previous_log - 2.0 * peak_log + next_log;
    double offset = 0.0;

    if (fabs(denominator) > DBL_EPSILON) {
        offset =
            0.5 * (previous_log - next_log) / denominator;
    }
    if (offset > 0.5) {
        offset = 0.5;
    } else if (offset < -0.5) {
        offset = -0.5;
    }
    return offset;
}

static void SortComponentsByHarmonic(q3_peak_t *peaks,
                                     uint32_t *harmonics,
                                     uint32_t count)
{
    uint32_t first;

    for (first = 0U; first < count; ++first) {
        uint32_t second;

        for (second = first + 1U; second < count; ++second) {
            if (harmonics[second] < harmonics[first]) {
                const q3_peak_t temporary_peak = peaks[first];
                const uint32_t temporary_harmonic = harmonics[first];

                peaks[first] = peaks[second];
                peaks[second] = temporary_peak;
                harmonics[first] = harmonics[second];
                harmonics[second] = temporary_harmonic;
            }
        }
    }
}

/*
 * Candidate peaks are already in descending power order. Compare every peak
 * with the same strongest peak so successively weaker spurs cannot walk below
 * the selected amplitude ratio. The threshold uses tenths of one percent, so
 * 15 means 1.5%. A zero setting bypasses this relative filter.
 */
static uint32_t RetainSignificantPeaks(q3_peak_t *peaks,
                                      uint32_t count,
                                      uint32_t threshold_tenths_percent)
{
    uint64_t strongest_power;
    double minimum_power;
    uint32_t kept = 1U;
    uint32_t candidate;

    if (count == 0U) {
        return 0U;
    }
    if (threshold_tenths_percent ==
        Q3_ANALYSIS_COMPONENT_THRESHOLD_DISABLED_TENTHS_PERCENT) {
        return count;
    }
    strongest_power = peaks[0].power;
    minimum_power =
        (double)strongest_power *
        (double)threshold_tenths_percent *
        (double)threshold_tenths_percent /
        1000000.0;
    for (candidate = 1U; candidate < count; ++candidate) {
        if ((double)peaks[candidate].power <= minimum_power) {
            break;
        }
        peaks[kept] = peaks[candidate];
        ++kept;
    }
    return kept;
}

static int HarmonicAlreadySelected(const uint32_t *harmonics,
                                   uint32_t count,
                                   uint32_t harmonic)
{
    uint32_t component;

    for (component = 0U; component < count; ++component) {
        if (harmonics[component] == harmonic) {
            return 1;
        }
    }
    return 0;
}

/*
 * The measurement contract always contains a real fundamental, but it can be
 * weaker than one of its harmonics. A fixed board spur must therefore not be
 * promoted merely because it is the lowest retained frequency.
 *
 * Treat each sufficiently strong candidate as an H1 hypothesis, then retain
 * only candidates that fall on integer multiples of that frequency. Prefer
 * the model with the most matched components, followed by matched power and
 * frequency residual. The five-percent H1 qualification rejects weak
 * subharmonic board spurs while the two-percent component pool still retains
 * legitimate low-level higher harmonics such as the H15 regression case.
 */
static uint32_t SelectHarmonicModel(
    const q3_peak_t *candidates,
    uint32_t candidate_count,
    q3_peak_t *selected,
    uint32_t *selected_harmonics)
{
    q3_peak_t best_peaks[Q3_ANALYSIS_MAX_COMPONENTS];
    uint32_t best_harmonics[Q3_ANALYSIS_MAX_COMPONENTS];
    uint32_t best_count = 0U;
    double best_power = -1.0;
    double best_error = DBL_MAX;
    const uint64_t strongest_power =
        candidate_count > 0U ? candidates[0].power : 0U;
    uint32_t fundamental;

    for (fundamental = 0U;
         fundamental < candidate_count; ++fundamental) {
        q3_peak_t model_peaks[Q3_ANALYSIS_MAX_COMPONENTS];
        uint32_t model_harmonics[Q3_ANALYSIS_MAX_COMPONENTS];
        uint32_t model_count = 1U;
        double model_power = (double)candidates[fundamental].power;
        double model_error = 0.0;
        const double fundamental_bin = candidates[fundamental].bin;
        uint32_t candidate;

        if ((fundamental_bin <= 0.0) ||
            (candidates[fundamental].power <=
             strongest_power / Q3_ANALYSIS_FUNDAMENTAL_POWER_GAP)) {
            continue;
        }

        model_peaks[0] = candidates[fundamental];
        model_harmonics[0] = 1U;
        for (candidate = 0U;
             candidate < candidate_count &&
             model_count < Q3_ANALYSIS_MAX_COMPONENTS;
             ++candidate) {
            const double ratio =
                candidates[candidate].bin / fundamental_bin;
            uint32_t harmonic;
            double residual;

            if (candidate == fundamental || ratio < 1.5) {
                continue;
            }
            harmonic = (uint32_t)(ratio + 0.5);
            if ((harmonic < 2U) ||
                (harmonic > Q3_ANALYSIS_MAX_HARMONIC_ORDER) ||
                HarmonicAlreadySelected(
                    model_harmonics, model_count, harmonic)) {
                continue;
            }
            residual = fabs(
                candidates[candidate].bin -
                fundamental_bin * (double)harmonic);
            if (residual > Q3_ANALYSIS_HARMONIC_TOLERANCE_BINS) {
                continue;
            }

            model_peaks[model_count] = candidates[candidate];
            model_harmonics[model_count] = harmonic;
            model_power += (double)candidates[candidate].power;
            model_error += residual;
            ++model_count;
        }

        if ((model_count > best_count) ||
            ((model_count == best_count) &&
             ((model_power > best_power) ||
              ((model_power == best_power) &&
               (model_error < best_error))))) {
            best_count = model_count;
            best_power = model_power;
            best_error = model_error;
            (void)memcpy(best_peaks, model_peaks,
                         model_count * sizeof(best_peaks[0]));
            (void)memcpy(best_harmonics, model_harmonics,
                         model_count * sizeof(best_harmonics[0]));
        }
    }

    if (best_count == 0U) {
        return 0U;
    }
    (void)memcpy(selected, best_peaks,
                 best_count * sizeof(selected[0]));
    (void)memcpy(selected_harmonics, best_harmonics,
                 best_count * sizeof(selected_harmonics[0]));
    SortComponentsByHarmonic(
        selected, selected_harmonics, best_count);
    return best_count;
}

static double WrapPhase(double phase)
{
    while (phase > Q3_ANALYSIS_PI) {
        phase -= Q3_ANALYSIS_TWO_PI;
    }
    while (phase < -Q3_ANALYSIS_PI) {
        phase += Q3_ANALYSIS_TWO_PI;
    }
    return phase;
}

static int SolveLinearSystem(
    double matrix[Q3_ANALYSIS_MAX_MATRIX][Q3_ANALYSIS_MAX_MATRIX],
    double right_hand[Q3_ANALYSIS_MAX_MATRIX],
    uint32_t dimension,
    double solution[Q3_ANALYSIS_MAX_MATRIX])
{
    double augmented[Q3_ANALYSIS_MAX_MATRIX]
                    [Q3_ANALYSIS_MAX_MATRIX + 1U];
    uint32_t row;
    uint32_t column;

    for (row = 0U; row < dimension; ++row) {
        for (column = 0U; column < dimension; ++column) {
            augmented[row][column] = matrix[row][column];
        }
        augmented[row][dimension] = right_hand[row];
    }

    for (column = 0U; column < dimension; ++column) {
        uint32_t pivot_row = column;
        double pivot_magnitude = fabs(augmented[column][column]);

        for (row = column + 1U; row < dimension; ++row) {
            const double magnitude = fabs(augmented[row][column]);

            if (magnitude > pivot_magnitude) {
                pivot_magnitude = magnitude;
                pivot_row = row;
            }
        }
        if (pivot_magnitude < 1.0e-12) {
            return -1;
        }
        if (pivot_row != column) {
            for (row = column; row <= dimension; ++row) {
                const double temporary = augmented[column][row];

                augmented[column][row] = augmented[pivot_row][row];
                augmented[pivot_row][row] = temporary;
            }
        }

        {
            const double pivot = augmented[column][column];

            for (row = column; row <= dimension; ++row) {
                augmented[column][row] /= pivot;
            }
        }
        for (row = 0U; row < dimension; ++row) {
            double factor;
            uint32_t item;

            if (row == column) {
                continue;
            }
            factor = augmented[row][column];
            for (item = column; item <= dimension; ++item) {
                augmented[row][item] -=
                    factor * augmented[column][item];
            }
        }
    }

    for (row = 0U; row < dimension; ++row) {
        solution[row] = augmented[row][dimension];
    }
    return 0;
}

static int FitSinusoids(const int32_t *samples,
                        uint32_t first_sample,
                        uint32_t sample_count,
                        double sample_rate_hz,
                        double fundamental_hz,
                        const uint32_t *harmonics,
                        uint32_t component_count,
                        q3_fit_t *fit)
{
    double normal[Q3_ANALYSIS_MAX_MATRIX][Q3_ANALYSIS_MAX_MATRIX];
    double right_hand[Q3_ANALYSIS_MAX_MATRIX];
    double solution[Q3_ANALYSIS_MAX_MATRIX];
    q3_oscillator_t oscillator[Q3_ANALYSIS_MAX_COMPONENTS];
    const uint32_t dimension = 1U + 2U * component_count;
    uint32_t row;
    uint32_t column;
    uint32_t component;
    uint32_t sample_index;
    uint32_t sample_step = 1U;
    double highest_frequency_hz = 0.0;

    for (component = 0U; component < component_count; ++component) {
        double frequency_hz = fundamental_hz * (double)harmonics[component];
        if (frequency_hz > highest_frequency_hz)
            highest_frequency_hz = frequency_hz;
    }
    if (sample_count >= 8192U) {
        if (highest_frequency_hz < sample_rate_hz / 16.0)
            sample_step = 4U;
        else if (highest_frequency_hz < sample_rate_hz / 8.0)
            sample_step = 2U;
    }

    (void)memset(normal, 0, sizeof(normal));
    (void)memset(right_hand, 0, sizeof(right_hand));
    (void)memset(fit, 0, sizeof(*fit));

    for (component = 0U; component < component_count; ++component) {
        const double omega =
            Q3_ANALYSIS_TWO_PI * fundamental_hz *
            (double)harmonics[component] / sample_rate_hz;
        const double initial_phase =
            omega * (double)first_sample;

        oscillator[component].omega = omega;
        oscillator[component].cosine = cos(initial_phase);
        oscillator[component].sine = sin(initial_phase);
        oscillator[component].step_cosine = cos(omega * sample_step);
        oscillator[component].step_sine = sin(omega * sample_step);
    }

    for (sample_index = 0U; sample_index < sample_count;
         sample_index += sample_step) {
        double basis[Q3_ANALYSIS_MAX_MATRIX];
        const double sample =
            (double)samples[first_sample + sample_index];

        basis[0] = 1.0;
        for (component = 0U; component < component_count;
             ++component) {
            basis[1U + 2U * component] =
                oscillator[component].cosine;
            basis[2U + 2U * component] =
                oscillator[component].sine;
        }

        for (row = 0U; row < dimension; ++row) {
            right_hand[row] += basis[row] * sample;
            for (column = 0U; column <= row; ++column) {
                normal[row][column] += basis[row] * basis[column];
            }
        }

        for (component = 0U; component < component_count;
             ++component) {
            double next_cosine =
                oscillator[component].cosine *
                    oscillator[component].step_cosine -
                oscillator[component].sine *
                    oscillator[component].step_sine;
            double next_sine =
                oscillator[component].sine *
                    oscillator[component].step_cosine +
                oscillator[component].cosine *
                    oscillator[component].step_sine;

            if (((sample_index + sample_step) & 1023U) == 0U) {
                const double exact_phase =
                    oscillator[component].omega *
                    (double)(first_sample + sample_index + sample_step);

                next_cosine = cos(exact_phase);
                next_sine = sin(exact_phase);
            }
            oscillator[component].cosine = next_cosine;
            oscillator[component].sine = next_sine;
        }
    }

    for (row = 0U; row < dimension; ++row) {
        for (column = 0U; column < row; ++column) {
            normal[column][row] = normal[row][column];
        }
    }
    if (SolveLinearSystem(normal, right_hand, dimension,
                          solution) != 0) {
        return -1;
    }

    for (component = 0U; component < component_count; ++component) {
        const double cosine_coefficient =
            solution[1U + 2U * component];
        const double sine_coefficient =
            solution[2U + 2U * component];

        fit->cosine[component] = cosine_coefficient;
        fit->sine[component] = sine_coefficient;
        fit->amplitude[component] =
            hypot(cosine_coefficient, sine_coefficient);
        /*
         * MATLAB stores sineCoefficient + j*cosineCoefficient so that
         * angle() is the phase of A*sin(wt + phase).
         */
        fit->phase[component] =
            atan2(cosine_coefficient, sine_coefficient);
    }
    return 0;
}

static double RefineFundamental(const int32_t *samples,
                                uint32_t sample_count,
                                double sample_rate_hz,
                                double fundamental_hz,
                                const uint32_t *harmonics,
                                uint32_t component_count)
{
    uint32_t block_length = Q3_ANALYSIS_PHASE_BLOCK;
    uint32_t iteration;

    if (block_length > sample_count / 4U) {
        block_length = sample_count / 4U;
    }
    if (block_length < 16U) {
        return fundamental_hz;
    }

    for (iteration = 0U;
         iteration < Q3_ANALYSIS_PHASE_ITERATIONS; ++iteration) {
        q3_fit_t first_fit;
        q3_fit_t second_fit;
        const uint32_t second_start = sample_count - block_length;
        const double delta_time =
            (double)(sample_count - block_length) / sample_rate_hz;
        double weighted_correction = 0.0;
        double total_weight = 0.0;
        uint32_t component;

        if ((FitSinusoids(samples, 0U, block_length,
                          sample_rate_hz, fundamental_hz,
                          harmonics, component_count,
                          &first_fit) != 0) ||
            (FitSinusoids(samples, second_start, block_length,
                          sample_rate_hz, fundamental_hz,
                          harmonics, component_count,
                          &second_fit) != 0)) {
            break;
        }

        for (component = 0U; component < component_count;
             ++component) {
            const double phase_change = WrapPhase(
                second_fit.phase[component] -
                first_fit.phase[component]);
            const double correction =
                phase_change /
                (Q3_ANALYSIS_TWO_PI *
                 (double)harmonics[component] * delta_time);
            const double weight =
                first_fit.amplitude[component] *
                second_fit.amplitude[component];

            weighted_correction += weight * correction;
            total_weight += weight;
        }
        if (total_weight <= DBL_EPSILON) {
            break;
        }
        fundamental_hz += weighted_correction / total_weight;
    }
    return fundamental_hz;
}

static double ReconstructAtPhase(const q3_analysis_result_t *result,
                                 double phase)
{
    double value = 0.0;
    uint32_t component;

    for (component = 0U; component < result->component_count;
         ++component) {
        value +=
            (double)result->component_amplitude_codes[component] *
            sin((double)result->harmonics[component] * phase +
                (double)result->
                    component_relative_phase_rad[component]);
    }
    return value;
}

int q3_analysis_run(const int32_t *time_samples,
                    uint32_t time_sample_count,
                    const uint32_t *fft_words,
                    uint32_t fft_slot_count,
                    uint32_t fft_first_bin,
                    uint32_t fft_length,
                    uint32_t fft_block_exponent,
                    float sample_rate_hz,
                    uint16_t *display_spectrum,
                    uint32_t display_point_count,
                    q3_analysis_result_t *result)
{
    q3_peak_t candidates[Q3_ANALYSIS_MAX_CANDIDATE_PEAKS];
    q3_peak_t selected[Q3_ANALYSIS_MAX_COMPONENTS];
    uint64_t median_power;
    uint64_t threshold_power;
    uint32_t candidate_count = 0U;
    uint32_t slot;
    uint32_t bucket;
    uint32_t selection;
    uint32_t component_count = 0U;
    uint32_t component;
    uint32_t harmonics[Q3_ANALYSIS_MAX_COMPONENTS];
    const uint32_t component_threshold_tenths_percent =
        ComponentThresholdTenthsPercent;
    double weighted_frequency = 0.0;
    double weighted_harmonic = 0.0;
    double fundamental_bin;
    double fundamental_hz;
    q3_fit_t full_fit;
    double base_phase = 0.0;
    double rms_squared = 0.0;
    double waveform_min = DBL_MAX;
    double waveform_max = -DBL_MAX;

    if ((time_samples == NULL) || (fft_words == NULL) ||
        (display_spectrum == NULL) || (result == NULL) ||
        (time_sample_count == 0U) || (fft_slot_count < 3U) ||
        (fft_slot_count > Q3_ANALYSIS_MAX_FFT_SLOTS) ||
        (fft_length == 0U) || (sample_rate_hz <= 0.0f) ||
        (display_point_count == 0U) ||
        (display_point_count > Q3_ANALYSIS_MAX_DISPLAY_POINTS)) {
        return -1;
    }

    (void)memset(result, 0, sizeof(*result));
    /*
     * The display contract is component-only. Clearing it before analysis also
     * guarantees that noise-only frames and every early-exit path cannot leak
     * raw Hann-window sidelobes into the UI.
     */
    (void)memset(display_spectrum, 0,
                 display_point_count * sizeof(*display_spectrum));
    for (slot = 0U; slot < fft_slot_count; ++slot) {
        SpectrumPower[slot] = PowerAt(fft_words, slot);
    }

    median_power = SelectMedian(SpectrumPower, fft_slot_count);
    {
        /*
         * A perfectly clean host vector can quantize more than half the FFT
         * bins to zero. Treat one FFT power count as the minimum measurable
         * noise reference so threshold-off mode does not promote isolated
         * one-LSB bins into extra harmonics.
         */
        const uint64_t noise_reference_power =
            (median_power == 0U) ? 1U : median_power;

        if (noise_reference_power >
            UINT64_MAX /
            (Q3_ANALYSIS_NOISE_MAG_RATIO *
             Q3_ANALYSIS_NOISE_MAG_RATIO)) {
            threshold_power = UINT64_MAX;
        } else {
            threshold_power =
                noise_reference_power *
                Q3_ANALYSIS_NOISE_MAG_RATIO *
                Q3_ANALYSIS_NOISE_MAG_RATIO;
        }
    }
    if (component_threshold_tenths_percent !=
        Q3_ANALYSIS_COMPONENT_THRESHOLD_DISABLED_TENTHS_PERCENT) {
        uint64_t absolute_threshold_power;

        /*
         * MATLAB: magnitude >= 0.5 mV * sum(Hann) / 2.
         * FIR samples retain the AN9238 integer scale. The block-floating
         * FFT reports X / 2^BLK_EXP and a periodic Hann has sum = N/2.
         * This is an AC peak-amplitude threshold, so it must not include the
         * signed-sample DC zero-offset correction.
         *
         * Zero-percent is the explicit all-fixed-threshold-off mode. In that
         * mode keep only the adaptive noise gate above; otherwise a 0.9 mVpp
         * component (0.45 mV peak) can never be retained even after the user
         * switches the threshold off.
         */
        const double minimum_peak_codes =
            (double)q3_calibration_peak_mv_to_amplitude_codes(
                (float)(Q3_ANALYSIS_MIN_PEAK_VOLTS * 1000.0));
        const double fft_scale =
            ldexp(1.0, (int)fft_block_exponent);
        const double minimum_fft_magnitude =
            minimum_peak_codes * (double)fft_length /
            (4.0 * fft_scale);
        const double minimum_fft_power =
            minimum_fft_magnitude * minimum_fft_magnitude;

        if (minimum_fft_power >= (double)UINT64_MAX) {
            absolute_threshold_power = UINT64_MAX;
        } else {
            absolute_threshold_power =
                (uint64_t)ceil(minimum_fft_power);
        }
        if (absolute_threshold_power > threshold_power) {
            threshold_power = absolute_threshold_power;
        }
    }

    /*
     * Repeated maximum scans build a small power-ordered candidate pool while
     * enforcing peakGuardBins. The later harmonic-model stage chooses the
     * components published to the UI.
     */
    for (selection = 0U;
         selection < Q3_ANALYSIS_MAX_CANDIDATE_PEAKS; ++selection) {
        uint32_t best_slot = 0U;
        uint64_t best_power = 0U;

        for (slot = 1U; slot + 1U < fft_slot_count; ++slot) {
            const uint64_t power = SpectrumPower[slot];

            if ((power == 0U) || (power < threshold_power) ||
                (power <= SpectrumPower[slot - 1U]) ||
                (power < SpectrumPower[slot + 1U]) ||
                !IsSeparated(slot, candidates, candidate_count)) {
                continue;
            }
            if (power > best_power) {
                best_power = power;
                best_slot = slot;
            }
        }
        if (best_power == 0U) {
            break;
        }
        candidates[candidate_count].slot = best_slot;
        candidates[candidate_count].power = best_power;
        candidates[candidate_count].bin =
            (double)(fft_first_bin + best_slot) +
            LogParabolicOffset(SpectrumPower[best_slot - 1U],
                               best_power,
                               SpectrumPower[best_slot + 1U]);
        ++candidate_count;
    }
    if (candidate_count == 0U) {
        return 0;
    }

    candidate_count =
        RetainSignificantPeaks(
            candidates, candidate_count,
            component_threshold_tenths_percent);
    component_count = SelectHarmonicModel(
        candidates, candidate_count, selected, harmonics);
    if (component_count == 0U) {
        return 0;
    }

    for (component = 0U; component < component_count; ++component) {
        const double weight = (double)selected[component].power;
        const double harmonic = (double)harmonics[component];

        weighted_frequency +=
            weight * harmonic * selected[component].bin;
        weighted_harmonic += weight * harmonic * harmonic;
    }
    if (weighted_harmonic <= DBL_EPSILON) {
        return 0;
    }

    fundamental_bin = weighted_frequency / weighted_harmonic;
    fundamental_hz =
        fundamental_bin * (double)sample_rate_hz /
        (double)fft_length;
    fundamental_hz = RefineFundamental(
        time_samples, time_sample_count, (double)sample_rate_hz,
        fundamental_hz, harmonics, component_count);

    if (FitSinusoids(time_samples, 0U, time_sample_count,
                     (double)sample_rate_hz, fundamental_hz,
                     harmonics, component_count, &full_fit) != 0) {
        return 0;
    }

    result->component_count = component_count;
    for (component = 0U; component < component_count; ++component) {
        if (harmonics[component] == 1U) {
            base_phase = full_fit.phase[component];
            break;
        }
    }
    for (component = 0U; component < component_count; ++component) {
        const double component_frequency =
            fundamental_hz * (double)harmonics[component];
        const double corrected_amplitude =
            full_fit.amplitude[component] /
            FilteredChainMagnitude(component_frequency);

        result->harmonics[component] = harmonics[component];
        result->component_frequency_hz[component] =
            (float)component_frequency;
        result->component_amplitude_codes[component] =
            (float)corrected_amplitude;
        result->component_phase_rad[component] =
            (float)full_fit.phase[component];
        result->component_relative_phase_rad[component] =
            (float)WrapPhase(
                full_fit.phase[component] -
                (double)harmonics[component] * base_phase);
        rms_squared +=
            corrected_amplitude * corrected_amplitude;

        if (harmonics[component] == 1U) {
            result->fundamental_phase_rad =
                result->component_phase_rad[component];
            result->fundamental_amplitude_codes =
                result->component_amplitude_codes[component];
        }
    }

    result->fundamental_hz = (float)fundamental_hz;
    result->fundamental_bin =
        (float)(fundamental_hz * (double)fft_length /
                (double)sample_rate_hz);
    result->rms_codes = (float)sqrt(rms_squared / 2.0);

    for (selection = 0U;
         selection < Q3_ANALYSIS_RECONSTRUCT_POINTS; ++selection) {
        const double phase =
            Q3_ANALYSIS_TWO_PI * (double)selection /
            (double)Q3_ANALYSIS_RECONSTRUCT_POINTS;
        const double value = ReconstructAtPhase(result, phase);

        if (value < waveform_min) {
            waveform_min = value;
        }
        if (value > waveform_max) {
            waveform_max = value;
        }
    }
    result->peak_to_peak_codes =
        (float)(waveform_max - waveform_min);
    result->waveform_max_abs_codes =
        (float)fmax(fabs(waveform_min), fabs(waveform_max));
    result->valid = 1U;

    /*
     * Raw Hann-window FFT powers are used only for detection. The public
     * display spectrum contains only retained components and uses their final
     * unwindowed least-squares amplitudes, including CIC/FIR/analog correction.
     * Rejected peaks and neighboring window sidelobes therefore remain zero.
     */
    {
        double maximum_component_amplitude = 0.0;

        for (component = 0U; component < component_count; ++component) {
            const double amplitude =
                (double)result->component_amplitude_codes[component];

            if (amplitude > maximum_component_amplitude) {
                maximum_component_amplitude = amplitude;
            }
        }

        if (maximum_component_amplitude > DBL_EPSILON) {
            for (component = 0U;
                 component < component_count; ++component) {
                uint32_t display_value;

                if (result->component_frequency_hz[component] > 200000.0f)
                    continue;
                bucket = (uint32_t)(
                    result->component_frequency_hz[component] *
                    (float)(display_point_count - 1U) / 200000.0f);
                if (bucket >= display_point_count) {
                    bucket = display_point_count - 1U;
                }
                display_value = (uint32_t)
                    ((double)result->component_amplitude_codes[component] *
                     100.0 / maximum_component_amplitude + 0.5);
                if (display_value > 100U) {
                    display_value = 100U;
                }
                display_spectrum[bucket] = (uint16_t)display_value;
            }
        }
    }

    return 0;
}

float q3_analysis_waveform_at_phase(
    const q3_analysis_result_t *result, float phase_rad)
{
    if ((result == NULL) || (result->valid == 0U)) {
        return 0.0f;
    }
    return (float)ReconstructAtPhase(result, (double)phase_rad);
}
