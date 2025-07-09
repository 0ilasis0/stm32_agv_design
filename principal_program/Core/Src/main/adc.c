#include "main/adc.h"
#include "adc.h"
#include "motor/main.h"
#include "main/config.h"

#define SWAP(a,b)       \
do {                    \
    uint16_t temp = a;  \
    a = b;              \
    b = temp;           \
} while(0)

uint16_t hall_max[4] = {0};
uint16_t hall_min[4] = {2000, 2000, 2000, 2000};
AdcHall adc_hall;

static uint16_t ADC_Values[ADC_COUNT * ADC_NEED_LEN] = {0};                                 // adc儲存位置

// PB12(R16)     PB1(R24)     PB11(R18)      PB0(L34)
static AdcHall adc_hall_init (void)
{
    AdcHall adc_hall_new;
    adc_hall_new.sensor_track_right     = 0;
    adc_hall_new.sensor_track_left      = 0;
    adc_hall_new.sensor_node            = 0;
    adc_hall_new.sensor_direction       = 0;
    adc_hall_new.magnetic_stripe_value  = ADC_MAGNETIC_STRIPE_VALUE;
    adc_hall_new.strong_magnet_value    = ADC_STRONG_MAGNET_VALUE;
    return adc_hall_new;
}

static void max_min (uint16_t temp, uint16_t *max, uint16_t *min)
{
    if(temp > *max) *max = temp;
    if(temp < *min && temp > 1000) *min = temp;
}

static void quick_sort(uint16_t* arr, int left, int right) {
    if (left >= right) return;
    uint16_t pivot = arr[right];
    int i = left - 1;
    for (int j = left; j < right; j++) {
        if (arr[j] <= pivot) {
            i++;
            SWAP(arr[i], arr[j]);
        }
    }
    SWAP(arr[i + 1], arr[right]);
    quick_sort(arr, left, i);
    quick_sort(arr, i + 2, right);
}

static void sort(uint16_t* values, size_t count) {
    if (count > 1) {
        quick_sort(values, 0, count - 1);
    }
}


/*text*/
static KalmanFilter kf[4];

static void Kalman_Init(KalmanFilter *kf, float Q, float R) {
    kf->x_est = 0;
    kf->P = 1;
    kf->Q = Q;
    kf->R = R;
}

static float Kalman_Update(KalmanFilter *kf, float z) {
    kf->P += kf->Q;
    float K = kf->P / (kf->P + kf->R);
    kf->x_est += K * (z - kf->x_est);
    kf->P *= (1 - K);
    return kf->x_est;
}

static void Kalman_set(void)
{
    for(int i = 0; i < 4; i++)
    {
        Kalman_Init(&kf[i], 0.01f, 5.0f);
    }
}
/*text*/


void adc_setup (void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Values, ADC_COUNT * ADC_NEED_LEN);
    adc_hall = adc_hall_init();
    Kalman_set();
}

// renew adc senser
void adc_renew (void)
{
    if (!runtime_switch.adc) return;

    uint16_t adc[ADC_COUNT][ADC_NEED_LEN] = {0};

    // for (int i = 0; i < ADC_COUNT; i++)
    // {
    //     for (int j = 0; j < ADC_NEED_LEN; j++)
    //     {
    //         adc[i][j] = ADC_Values[j * ADC_COUNT + i];
    //     }
    //     sort(adc[i], ADC_NEED_LEN);
    // }

    // adc_hall.sensor_track_right   = adc[0][ADC_NEED_LEN/2];
    // adc_hall.sensor_track_left    = adc[1][ADC_NEED_LEN/2];
    // adc_hall.sensor_node          = adc[2][ADC_NEED_LEN/2];
    // adc_hall.sensor_direction     = adc[3][ADC_NEED_LEN/2];

    adc_hall.sensor_track_right  = Kalman_Update(&kf[0], adc[0][ADC_NEED_LEN - 1]);
    adc_hall.sensor_track_left   = Kalman_Update(&kf[1], adc[1][ADC_NEED_LEN - 1]);
    adc_hall.sensor_node         = Kalman_Update(&kf[2], adc[2][ADC_NEED_LEN - 1]);
    adc_hall.sensor_direction    = Kalman_Update(&kf[3], adc[3][ADC_NEED_LEN - 1]);

    if (HAL_GetTick() < 3000) return;
    max_min(adc_hall.sensor_track_right,   &hall_max[0], &hall_min[0]);
    max_min(adc_hall.sensor_track_left,    &hall_max[1], &hall_min[1]);
    max_min(adc_hall.sensor_node,          &hall_max[2], &hall_min[2]);
    max_min(adc_hall.sensor_direction,     &hall_max[3], &hall_min[3]);
}


