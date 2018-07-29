#include "Wheel_speed.h"

static ECUB_Wheelspeed_t Wheel_can; //can structure
int32_t i, key, j; //for sortInsertion function
uint32_t *temp; //only used in compute_average fuction
double average; //only used in compute_average fuction
static const float PI = 3.14159265359; //pi constant

#define CLOCK_PRESCALER 72.0 //vytahneš z inicializace TIM
#define INT_CLOCK 72000000.0
static const float CLOCK = (INT_CLOCK / CLOCK_PRESCALER);// Final timer clock ticking 
static const float WHEEL_RADIUS = 0.195;
static const float WHEEL_PERIMETER = (WHEEL_RADIUS * 2.0 * PI);
static const float SENS_POLES = (48.0); 
static const float WHL_FRQ_CONST = (CLOCK);  // Divide by counter period to get  frequency
static const float WHL_SPD_CONST = ((WHEEL_PERIMETER/SENS_POLES) * CLOCK); // Divide by counter period to get wheel perimeter speed m/s
static const float WHL_SPD_CONST_MMS = (1000.0 * (float)WHL_SPD_CONST); // Divide by counter period to get wheel perimeter speed mm/s
static const float WHL_A_SPD_CONST = ((1.0/SENS_POLES)*CLOCK); // Divide by counter period to get angular speed RPS
static const float WHL_A_SPD_CONST_mRPS = (int32_t)(1000.0 * (float)WHL_A_SPD_CONST); // Divide by counter period to get angular speed mRPS 
static const float WHL_A_SPD_CONST_dRPM = (int32_t)(600.0 * (float)WHL_A_SPD_CONST); // deka RPM  
static const float WHL_ZERO_SPD_THRESHOLD = 200; // If calculated speed is smaller than 250mm/s we say we are standstill  

SPEED_SENS_TypeDef sens_rr_speed;
SPEED_SENS_TypeDef sens_rl_speed;

uint64_t timer_functions_countdown;
static const uint64_t timer_functions_timeout = 10; //ms

uint64_t counter_left_overflow_countdown;
uint64_t counter_right_overflow_countdown;
uint8_t counter_left_overflow;
uint8_t counter_right_overflow;

static const uint64_t counter_overflow_timeout = 12;

void sortInsertion(int32_t *array, int32_t size)
{
   int i, key, j;

   for (i = 1; i < size; i++)
   {
       key = array[i];
       j = i-1;

       /* Move elements of arr[0..i-1], that are
          greater than key, to one position ahead
          of their current position */
       while (j >= 0 && array[j] > key)
       {
           array[j+1] = array[j];
           j = j-1;
       }
       array[j+1] = key;
   }
}
/** Function initialize defaults for speed sensor
 *  arguments:
 *  counter - used to know the timer tick when function is called
 */
void initSens(SPEED_SENS_TypeDef *sens_spd)
{
		timer_functions_countdown = 0;
    uint8_t i;

    sens_spd->data.counter = 0;
    sens_spd->data.last_counter = 0;
    sens_spd->data.flag_overflow = 0;
    sens_spd->data.input_prescale = 1;

    sens_spd->data.filt_period= 0;
    sens_spd->data.filter.index = 0;
    sens_spd->data.filter.len = PERIOD_BUF_LEN;
    sens_spd->data.filter.win_len = -1;

    for(i=0; i<PERIOD_BUF_LEN; i++)
    {
        sens_spd->data.filter.arr[i] = 0;
    }

    sens_spd->zerospeed = 0;
    sens_spd->freq = 0;
    sens_spd->speed = 0;
    sens_spd->speed_mms = 0;
    sens_spd->rot_speed = 0;
    sens_spd->rot_speed_mrps = 0;
		
		counter_left_overflow = 1;
		counter_right_overflow = 1;
}

/** Function detects zero speed
 *  Comment:
 *  Function assume that counter is reset by sensor signal edge
 *
 *  arguments:
 *  sens_spd - basic sensor data structure with speeds etc.
 *  counter - used to know the timer tick when function is called
 *
 *  return:
 *  1 - zero speed detected
 */
int checkZeroSpeed(SPEED_SENS_TypeDef *sens_spd, int32_t counter)
{
    if(((WHL_SPD_CONST_MMS/counter) < WHL_ZERO_SPD_THRESHOLD) || ((WHL_SPD_CONST_MMS/sens_spd->data.counter) < WHL_ZERO_SPD_THRESHOLD))
    {
        return 1;
    }
    return 0;
}

void start_WS_measure(TIM_HandleTypeDef *tim_WSR,TIM_HandleTypeDef *tim_WSL) //should start dma measurement of wheel speed
{ 
	//HAL_TIM_Base_Start_IT(tim_WSR); //configuration for wheelspeed right
	//HAL_TIM_Base_Start_IT(tim_WSL); //configuration for wheelspeed left	
	HAL_TIM_IC_Init(tim_WSR);
	HAL_TIM_IC_Init(tim_WSL);
	HAL_TIM_IC_Start_IT(tim_WSR,TIM_CHANNEL_1); //configuration for wheelspeed left
	HAL_TIM_IC_Start_IT(tim_WSL,TIM_CHANNEL_1); //configuration for wheelspeed left
	initSens(&sens_rr_speed);
	initSens(&sens_rl_speed);
}

/** Function saves wheel pulse period
 *  Comment:
 *  This function should be called from interrupt of specific sensor
 *  Function is able to detect zero speed (spd < thrshld) when called
 *
 *  arguments:
 *  sens_spd - basic sensor data structure with speeds etc.
 *
 *  return:
 *  1 - zero speed detected
 *  0 - speed is calculated properly
 */
int saveSensPeriod(SPEED_SENS_TypeDef *sens_spd)
{

    ///Calculate new buffer index
    sens_spd->data.filter.index = (sens_spd->data.filter.index + 1) % sens_spd->data.filter.len ;
    if(++sens_spd->data.filter.win_len > sens_spd->data.filter.len) sens_spd->data.filter.win_len = sens_spd->data.filter.len;

    /// Calculate period
 //   if(sens_spd->data.flag_overflow)
 //   {
 //       sens_spd->data.filter.arr[sens_spd->data.filter.index] = 0xFFFFFFFF; // Use maximum period if overflow occurs
 //   }
 //   else
 //   {
        sens_spd->data.filter.arr[sens_spd->data.filter.index] = sens_spd->data.counter - sens_spd->data.last_counter;
 //   }

    //sens_spd->zerospeed = checkZeroSpeed(sens_spd, sens_spd->data.counter);
		
    return 0;
}

int32_t medianFilt(MEDIAN_TypeDef *median)
{
    /// Current status needs to be saved (because interrupt is overwriting)
    int32_t len = median->len;
    int32_t win_len = median->win_len;
    int32_t index = median->index;

    median->win_len = -1;
    if(win_len <= 0) win_len = 1;

    int32_t temp[win_len];
    int32_t i, j;

    // Find the corresponding data
    for(i=0; i < win_len; i++)
    {
        j = index - i;
        if(j < 0) j = len + j;

        temp[i] = median->arr[j];
    }

    //sortBubble(temp, win_len);
    sortInsertion(temp, win_len);

    // Median filter calculation
    if(win_len % 2)
    {
        return temp[win_len >> 1];
    }
    else
    {
        return ( ( temp[win_len>>1] + temp[(win_len>>1)-1] ) >> 1);
    }
}


/** Function calculate wheel speed
 *  Comment:
 *  This function should be called at some fixed frequency
 *  It calculates speed based on saved period data
 *
 *  arguments:
 *  sens_spd - basic sensor data structure with speeds etc.
 *
 *  return:
 *  1 - zero speed detected
 *  0 - speed is calculated properly
 */
int calcSens(SPEED_SENS_TypeDef *sens_spd, int32_t counter)
{
    /// Calculate filtering frequency in 100s of Hz as data update send frequency must be > 100Hz
    //if(sens_spd->data.filter.win_len < 1) sens_spd->data.filter.win_len = 1;
    //else if(sens_spd->data.filter.win_len > PERIOD_BUF_LEN) sens_spd->data.filter.win_len = PERIOD_BUF_LEN;

    sens_spd->data.filt_period = medianFilt(&sens_spd->data.filter);

    /// Calculate input frequency
    sens_spd->freq = (int32_t)(CLOCK / (float)sens_spd->data.filt_period);

    ///Calculate speeds only if zero speed not detected
    sens_spd->speed = (WHL_SPD_CONST / (float)sens_spd->data.filt_period);
    sens_spd->speed_mms = (int16_t)(WHL_SPD_CONST_MMS / (float)sens_spd->data.filt_period);
    sens_spd->rot_speed = (WHL_A_SPD_CONST / (float)sens_spd->data.filt_period);
    sens_spd->rot_speed_mrps = (int16_t)(WHL_A_SPD_CONST_mRPS / (float)sens_spd->data.filt_period);
    sens_spd->rot_speed_drpm = (int16_t)(WHL_A_SPD_CONST_dRPM / (float)sens_spd->data.filt_period);

    sens_spd->zerospeed = checkZeroSpeed(sens_spd, counter);  // Reset speed to 0 if counter is overflowing

    if(sens_spd->zerospeed)
    {
        //sens_spd->freq = 0;
        sens_spd->speed = 0;
        sens_spd->speed_mms = 0;
        sens_spd->rot_speed = 0;
        sens_spd->rot_speed_mrps = 0;
        sens_spd->rot_speed_drpm = 0;

        return 1;
    }

    return 0;
}




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM3)
	{
		TIM3->CCR2 = 1;
	}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin);
	if(htim->Instance==TIM3)
	{
		if (counter_right_overflow_countdown < HAL_GetTick()){
			counter_right_overflow = 1;
		}else{
			counter_right_overflow = 0;
		}
		sens_rr_speed.data.counter = TIM3->CCR1;
		TIM3->CNT = 0;
		saveSensPeriod(&sens_rr_speed); // Save period //zde to padá do pice
    /// Check if we are above zero speed threshold
    sens_rr_speed.zerospeed = checkZeroSpeed(&sens_rr_speed, sens_rr_speed.data.counter);
		counter_right_overflow_countdown = HAL_GetTick() + counter_overflow_timeout;
	}
	if(htim->Instance==TIM5)
	{
		if (counter_left_overflow_countdown < HAL_GetTick()){
			counter_left_overflow = 1;
		}else{
			counter_left_overflow = 0;
		}
		sens_rl_speed.data.counter = TIM5->CCR1;
		TIM5->CNT = 0;
		saveSensPeriod(&sens_rl_speed); // Save period
		/// Check if we are above zero speed threshold
    sens_rl_speed.zerospeed = checkZeroSpeed(&sens_rl_speed, sens_rl_speed.data.counter);
		counter_left_overflow_countdown = HAL_GetTick() + counter_overflow_timeout;

	}
}


void stop_WS_measure(TIM_HandleTypeDef *WSR,TIM_HandleTypeDef *WSL) //should stop dma measurement of wheel speed
{ 
	//HAL_TIM_Encoder_Stop_DMA(WSR,TIM_CHANNEL_ALL); //configuration for wheelspeed right
	//HAL_TIM_Encoder_Stop_DMA(WSL,TIM_CHANNEL_ALL); //configuration for wheelspeed left
}


void Can_WheelSpeed(CAN_HandleTypeDef *hcan,TIM_HandleTypeDef *htim3,TIM_HandleTypeDef *htim5)
{
	if (timer_functions_countdown < HAL_GetTick()){
		timer_functions_countdown = HAL_GetTick() + timer_functions_timeout;
		calcSens(&sens_rr_speed, TIM3->CCR1);
    calcSens(&sens_rl_speed, TIM5->CCR1);
		Wheel_can.WhL = sens_rr_speed.rot_speed_drpm;
		Wheel_can.WhR = sens_rl_speed.rot_speed_drpm;
		
		Wheel_can.FT_WhL = counter_right_overflow;
		Wheel_can.FT_WhR = counter_left_overflow;
	}
	if(ECUB_Wheelspeed_need_to_send()){
		ECUB_send_Wheelspeed_s(&Wheel_can);
		Wheel_can.SEQ++;
	}
	
}

