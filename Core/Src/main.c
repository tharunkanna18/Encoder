/* USER CODE BEGIN Header */
	/**
	  ******************************************************************************
	  * @file           : main.c
	  * @brief          : Main program body
	  ******************************************************************************
	  * @attention
	  *
	  * Copyright (c) 2026 STMicroelectronics.
	  * All rights reserved.
	  *
	  * This software is licensed under terms that can be found in the LICENSE file
	  * in the root directory of this software component.
	  * If no LICENSE file comes with this software, it is provided AS-IS.
	  *
	  ******************************************************************************
	  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "core_cm4.h"
#include <stdio.h>
#include <stdlib.h>
#include "telnet_server.h"
#include "bno055.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEAD_BAND 3

#define CENTRE_TOLERANCE 1
#define APPROACH_DISTANCE 20

#define RUN_PWM 400
#define CENTRE_PWM 70
#define MAX_PWM 400
#define MIN_PWM 100
#define KP 8

#define CAL_MOVE_THRESHOLD    30     // min counts of real travel before "stopped" = end-stop
#define CAL_STATE_TIMEOUT_MS  5000   // hard timeout, same for left and right


#define MAIN_MIN_PWM    250   // lowest PWM that still overcomes friction near centre
#define MAIN_MAX_PWM    400   // top speed, same as your old MAX_PWM
#define MAIN_PROP_BAND  80   // counts over which PWM ramps down as we approach centre

#define SHAKE_DURATION_MS   800   // Total time for engine shake effect
#define SHAKE_PWM           350   // Intensity of the shake
#define SHAKE_FREQUENCY_MS  50   // Pulse switch speed (50ms = 20Hz vibration)

#define PWM_TIMER_CLK_HZ   1000000UL  // TIM5 tick rate with current Prescaler=83 on 84MHz APB1 timer clock
#define PWM_FREQ_MIN_HZ    100
#define PWM_FREQ_MAX_HZ    20000

#define SETTINGS_FLASH_SECTOR   FLASH_SECTOR_11
#define SETTINGS_FLASH_ADDR     0x080E0000UL
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
I2C_HandleTypeDef hi2c1;   /* I2C1 for BNO055 IMU on PB6(SCL)/PB7(SDA) */

/* USER CODE BEGIN PV */
  volatile int32_t encoder_count = 0;
  volatile uint8_t centre_reached = 0;
  //volatile int32_t enc_count = 0;
  int32_t left_position = 0;
  int32_t right_position = 0;
  int32_t centre_position = 0;
  uint8_t calibration_state = 0;
  uint8_t calibration_done = 0;

  int32_t last_encoder = 0;
  uint32_t stable_timer = 0;
  uint32_t state_entry_timer = 0;
  int32_t  state_entry_encoder = 0;
  uint8_t  calibration_error = 0;
  uint8_t engine_started = 0;

  volatile uint32_t pwm_frequency_hz = 1000;   // matches original ARR=999 -> 1kHz
  volatile uint8_t  pwm_duty_percent = 0;      // 0-100%
  volatile uint8_t  pwm_direction    = 0;      // 0 = left channel, 1 = right channel
  volatile uint8_t  pwm_manual_mode  = 0;      // 0 = steering loop drives PWM, 1 = Hercules command does

  /* BNO055 IMU state */
  BNO055_Euler_t   imu_euler   = {0.0f, 0.0f, 0.0f};  /* X=Yaw, Y=Roll, Z=Pitch */
  uint8_t          imu_ok      = 0;                     /* 1 = sensor found and running */
  BNO055_CalibStatus_t imu_cal = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
void Steering_Control_Decel(void);

void PWM_Set_Frequency(uint32_t freq_hz);
void PWM_Set_Duty(uint8_t duty_percent);
void Telnet_Process_Command(char *cmd);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Motor_Left(void)
{
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, RUN_PWM);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
}

void Motor_Right(void)
{
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, RUN_PWM);
}

void Motor_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
}

void Motor_Left_pwm(uint16_t pwm)
{
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pwm);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
}

void Motor_Right_pwm(uint16_t pwm)
{
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, pwm);
}

void Move_To_Zero(void)
{
    int32_t error = encoder_count;
    uint16_t pwm;

    /* Exactly at centre */
    if(abs(error) <= CENTRE_TOLERANCE)
    {
        Motor_Stop();

        __disable_irq();
        encoder_count = 0;
        __enable_irq();

        centre_reached = 1;
        return;
    }

    /* Linear PWM */
    pwm = abs(error) * KP;

    if(pwm > RUN_PWM)
        pwm = RUN_PWM;

    if(abs(error) < APPROACH_DISTANCE)
    {
        if(pwm < CENTRE_PWM)
            pwm = CENTRE_PWM;
    }
    else
    {
        if(pwm < MIN_PWM)
            pwm = MIN_PWM;
    }

    if(error > 0)
        Motor_Left_pwm(pwm);
    else
        Motor_Right_pwm(pwm);
}

void Steering_Calibration(void)
{
    switch(calibration_state)
    {

    case 0: // Move LEFT
        Motor_Left();

        if(last_encoder != encoder_count)
        {
            last_encoder = encoder_count;
            stable_timer = HAL_GetTick();
        }

        if(abs(encoder_count - state_entry_encoder) > CAL_MOVE_THRESHOLD &&
           (HAL_GetTick() - stable_timer) > 1500)
        {
            left_position = encoder_count;          // stopped -> treat as left end
            Motor_Stop();
            HAL_Delay(1000);

            last_encoder = encoder_count;
            stable_timer = HAL_GetTick();
            state_entry_timer   = HAL_GetTick();
            state_entry_encoder = encoder_count;
            calibration_state = 1;
        }
        else if((HAL_GetTick() - state_entry_timer) > CAL_STATE_TIMEOUT_MS)
        {
            left_position = encoder_count;          // timeout -> treat as left end too
            calibration_error = 1;
            Motor_Stop();
            HAL_Delay(1000);

            last_encoder = encoder_count;
            stable_timer = HAL_GetTick();
            state_entry_timer   = HAL_GetTick();
            state_entry_encoder = encoder_count;
            calibration_state = 1;
        }
        break;

    case 1: // Move RIGHT
        Motor_Right();

        if(last_encoder != encoder_count)
        {
            last_encoder = encoder_count;
            stable_timer = HAL_GetTick();
        }

        if(abs(encoder_count - state_entry_encoder) > CAL_MOVE_THRESHOLD &&
           (HAL_GetTick() - stable_timer) > 1500)
        {
            right_position = encoder_count;
            centre_position = (left_position + right_position) / 2;
            Motor_Stop();
            HAL_Delay(1000);

            last_encoder = encoder_count;
            stable_timer = HAL_GetTick();
            calibration_state = 2;
        }
        else if((HAL_GetTick() - state_entry_timer) > CAL_STATE_TIMEOUT_MS)
        {
            right_position = encoder_count;
            centre_position = (left_position + right_position) / 2;
            calibration_error = 1;
            Motor_Stop();
            HAL_Delay(1000);

            last_encoder = encoder_count;
            stable_timer = HAL_GetTick();
            calibration_state = 2;
        }
        break;

        //------------------------------------
        // Move to Centre
        //------------------------------------
        case 2:

            if(encoder_count > centre_position + DEAD_BAND)
            {
                Motor_Left();
            }
            else if(encoder_count < centre_position - DEAD_BAND)
            {
                Motor_Right();
            }
            else
            {
                Motor_Stop();

                HAL_Delay(500);

                __disable_irq();

                encoder_count = encoder_count - centre_position;

               __enable_irq();

                calibration_done = 1;
                calibration_state = 3;
            }

            break;

        //------------------------------------
        // Calibration Complete
        //------------------------------------
        case 3:

            Motor_Stop();

            break;
    }
}


void Steering_Control_Decel(void)
{
    int32_t error = encoder_count;
    uint16_t pwm;

    /* Inside dead band: fully stop, no PWM at all */
    if(abs(error) <= DEAD_BAND)
    {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
        return;
    }

    /* Far from centre: full speed */
    if(abs(error) >= MAIN_PROP_BAND)
    {
        pwm = MAIN_MAX_PWM;
    }
    else
    {
        /* Linear ramp: MIN_PWM right at the dead band edge,
           scaling up to MAX_PWM at MAIN_PROP_BAND counts out */
        uint32_t span  = MAIN_PROP_BAND - DEAD_BAND;
        uint32_t dist  = abs(error) - DEAD_BAND;
        pwm = MAIN_MIN_PWM + (uint16_t)((dist * (MAIN_MAX_PWM - MAIN_MIN_PWM)) / span);
    }

    if(error > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pwm);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, pwm);
    }
}

void Engine_Startup_Shake(void)
{
    uint32_t start_tick = HAL_GetTick();
    uint32_t last_toggle = 0;
    uint8_t direction = 0;

    while ((HAL_GetTick() - start_tick) < SHAKE_DURATION_MS)
    {
        // Service LwIP/Ethernet during blocking animation loop
        MX_LWIP_Process();

        if (HAL_GetTick() - last_toggle >= SHAKE_FREQUENCY_MS)
        {
            last_toggle = HAL_GetTick();
            direction = !direction;

            if (direction)
            {
                Motor_Left_pwm(SHAKE_PWM);
            }
            else
            {
                Motor_Right_pwm(SHAKE_PWM);
            }
        }
    }

    /* Stop motor and re-center encoder offset after shaking */
    Motor_Stop();
    HAL_Delay(100);

    __disable_irq();
    encoder_count = 0; // Re-align center after physical vibration
    __enable_irq();

    engine_started = 1;
}

#define SETTINGS_FLASH_SECTOR   FLASH_SECTOR_11
#define SETTINGS_FLASH_ADDR     0x080E0000UL   // start of sector 11 (last 128KB) - verify your .map file doesn't reach this high
#define SETTINGS_MAGIC          0xC0FFEE01UL

typedef struct {
    uint32_t magic;
    uint32_t pwm_frequency_hz;
    uint32_t pwm_duty_percent;
    uint32_t pwm_direction;
    uint32_t pwm_manual_mode;
} settings_t;


static uint8_t Settings_Save(void)
{
    settings_t s;
    s.magic            = SETTINGS_MAGIC;
    s.pwm_frequency_hz = pwm_frequency_hz;
    s.pwm_duty_percent = pwm_duty_percent;
    s.pwm_direction    = pwm_direction;
    s.pwm_manual_mode  = pwm_manual_mode;

    HAL_FLASH_Unlock();

    /* Clear Flash flags */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                            FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_error = 0;
    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 2.7V to 3.6V supply
    erase.Sector       = SETTINGS_FLASH_SECTOR;
    erase.NbSectors    = 1;

    if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return 0; // Erase failed
    }

    uint32_t *src = (uint32_t *)&s;
    uint32_t addr = SETTINGS_FLASH_ADDR;
    for (uint32_t i = 0; i < (sizeof(s) / sizeof(uint32_t)); i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, src[i]) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return 0; // Write failed
        }
        addr += 4;
    }

    HAL_FLASH_Lock();
    return 1; // Success
}

static void Settings_Load(void)
{
    settings_t *saved = (settings_t *)SETTINGS_FLASH_ADDR;

    if (saved->magic == SETTINGS_MAGIC)
    {
        pwm_frequency_hz = saved->pwm_frequency_hz;
        pwm_duty_percent = (uint8_t)saved->pwm_duty_percent;
        pwm_direction    = (uint8_t)saved->pwm_direction;
        pwm_manual_mode  = (uint8_t)saved->pwm_manual_mode;

       /*  Apply immediately so the timer reflects the restored
           settings before the calibration/control loop starts*/
        PWM_Set_Frequency(pwm_frequency_hz);
        PWM_Set_Duty(pwm_duty_percent);
    }
   /*  else: nothing saved yet -- keep the compiled-in defaults */
}


void PWM_Set_Frequency(uint32_t freq_hz)
{
    if (freq_hz < PWM_FREQ_MIN_HZ) freq_hz = PWM_FREQ_MIN_HZ;
    if (freq_hz > PWM_FREQ_MAX_HZ) freq_hz = PWM_FREQ_MAX_HZ;

    uint32_t new_arr = (PWM_TIMER_CLK_HZ / freq_hz) - 1;

    __HAL_TIM_SET_AUTORELOAD(&htim5, new_arr);

    pwm_frequency_hz = freq_hz;

    /* Re-apply duty against the new ARR so pulse width tracks correctly */
    PWM_Set_Duty(pwm_duty_percent);
}

void PWM_Set_Duty(uint8_t duty_percent)
{
    if (duty_percent > 100) duty_percent = 100;

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim5);
    uint32_t ccr = (arr * duty_percent) / 100;

    if (pwm_direction == 0)
    {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, ccr);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, ccr);
    }

    pwm_duty_percent = duty_percent;
}

void Telnet_Process_Command(char *cmd)
{
    char response[96];

    while (*cmd == ' ') cmd++;   // trim leading spaces

    /* --- Combined Command: PWM=<freq>,<duty> (e.g., PWM=1500,50) --- */
    if (strncmp(cmd, "PWM=", 4) == 0)
    {
        uint32_t freq = 0;
        uint32_t duty = 0;

        // Parse frequency and duty cycle separated by a comma
        if (sscanf(cmd + 4, "%lu,%lu", &freq, &duty) == 2)
        {
            pwm_manual_mode = 1;

            // Set frequency first (updates ARR)
            PWM_Set_Frequency(freq);

            // Set duty cycle (calculates CCR against new ARR)
            PWM_Set_Duty((uint8_t)duty);

            snprintf(response, sizeof(response),
                     "OK PWM FREQ=%luHz DUTY=%u%% DIR=%c\r\n",
                     pwm_frequency_hz, pwm_duty_percent, pwm_direction ? 'R' : 'L');
            Telnet_Send(response);
        }
        else
        {
            Telnet_Send("ERR Invalid format. Use PWM=<freq>,<duty> (e.g., PWM=1500,50)\r\n");
        }
    }
    /* --- Individual Frequency Command --- */
    else if (strncmp(cmd, "FREQ=", 5) == 0)
    {
        PWM_Set_Frequency((uint32_t)atoi(cmd + 5));
        pwm_manual_mode = 1;

        snprintf(response, sizeof(response),
                 "OK FREQ=%luHz DUTY=%u%% DIR=%c\r\n",
                 pwm_frequency_hz, pwm_duty_percent, pwm_direction ? 'R' : 'L');
        Telnet_Send(response);
    }
    /* --- Individual Duty Command --- */
    else if (strncmp(cmd, "DUTY=", 5) == 0)
    {
        PWM_Set_Duty((uint8_t)atoi(cmd + 5));
        pwm_manual_mode = 1;

        snprintf(response, sizeof(response),
                 "OK FREQ=%luHz DUTY=%u%% DIR=%c\r\n",
                 pwm_frequency_hz, pwm_duty_percent, pwm_direction ? 'R' : 'L');
        Telnet_Send(response);
    }
    else if (strncmp(cmd, "DIR=L", 5) == 0)
    {
        pwm_direction = 0;
        PWM_Set_Duty(pwm_duty_percent);   // re-apply to new channel
        Telnet_Send("OK DIR=L\r\n");
    }
    else if (strncmp(cmd, "DIR=R", 5) == 0)
    {
        pwm_direction = 1;
        PWM_Set_Duty(pwm_duty_percent);
        Telnet_Send("OK DIR=R\r\n");
    }
    else if (strncmp(cmd, "AUTO", 4) == 0)
    {
        pwm_manual_mode = 0;
        Motor_Stop(); // Zero out manual override compares
        Telnet_Send("OK AUTO mode resumed\r\n");
    }
    else if (strncmp(cmd, "SAVE", 4) == 0)
    {
        if (Settings_Save())
            Telnet_Send("OK Settings saved to flash\r\n");
        else
            Telnet_Send("ERR Flash save failed\r\n");
    }
    /* --- IMU instant query: IMU=? --- */
    else if (strncmp(cmd, "IMU", 3) == 0)
    {
        if (imu_ok)
        {
            /* Force a fresh read and send current XYZ */
            if (BNO055_Read_Euler(&hi2c1, &imu_euler) == HAL_OK)
            {
                /* Format without %f using integer math (avoids newlib-nano float) */
                int16_t x_int  = (int16_t)imu_euler.x;
                int16_t x_dec  = (int16_t)((imu_euler.x  - (float)x_int) * 10.0f);
                if (x_dec < 0) x_dec = -x_dec;

                int16_t y_int  = (int16_t)imu_euler.y;
                int16_t y_dec  = (int16_t)((imu_euler.y  - (float)y_int) * 10.0f);
                if (y_dec < 0) y_dec = -y_dec;

                int16_t z_int  = (int16_t)imu_euler.z;
                int16_t z_dec  = (int16_t)((imu_euler.z  - (float)z_int) * 10.0f);
                if (z_dec < 0) z_dec = -z_dec;

                snprintf(response, sizeof(response),
                         "IMU X=%d.%d Y=%d.%d Z=%d.%d (Yaw/Roll/Pitch deg)\r\n",
                         x_int, x_dec,
                         y_int, y_dec,
                         z_int, z_dec);
                Telnet_Send(response);
            }
            else
            {
                Telnet_Send("ERR IMU read failed\r\n");
            }
        }
        else
        {
            Telnet_Send("ERR IMU not initialised (check BNO055 wiring)\r\n");
        }
    }
    /* --- IMU calibration status: IMUCAL --- */
    else if (strncmp(cmd, "IMUCAL", 6) == 0)
    {
        if (imu_ok)
        {
            if (BNO055_Read_CalibStatus(&hi2c1, &imu_cal) == HAL_OK)
            {
                snprintf(response, sizeof(response),
                         "IMUCAL SYS=%d GYR=%d ACC=%d MAG=%d (0=uncal,3=full)\r\n",
                         imu_cal.sys, imu_cal.gyro, imu_cal.accel, imu_cal.mag);
                Telnet_Send(response);
            }
            else
            {
                Telnet_Send("ERR IMU calibration read failed\r\n");
            }
        }
        else
        {
            Telnet_Send("ERR IMU not initialised\r\n");
        }
    }
    else
    {
        Telnet_Send("ERR Unknown command. Use PWM=<Hz>,<Duty%>, FREQ=<Hz>, DUTY=<0-100>, DIR=L|R, AUTO, SAVE, IMU, IMUCAL\r\n");
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  HAL_Delay(200);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); // Assert LAN_RST (Low)
  HAL_Delay(50);                                        // Hold in reset
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);   // Release LAN_RST (High)
  HAL_Delay(100);                                       // Wait for PHY clock to stabilize
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();           /* I2C1 for BNO055 IMU */
  MX_LWIP_Init();
  Telnet_Server_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE BEGIN 2 */
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);

    // Load saved configuration parameters into global variables
    Settings_Load();

    /* --- I2C Bus Scan: find all devices, report over Telnet --- */
    {
        char scan_msg[96];
        uint8_t found_any = 0;

        HAL_Delay(800);   /* Let BNO055 boot before scanning */

        Telnet_Send("I2C SCAN start (0x01-0x7F)...\r\n");
        HAL_Delay(50);   /* Give TCP time to flush */

        for (uint8_t addr = 1; addr < 128; addr++)
        {
            /* HAL uses 8-bit shifted address */
            if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 2, 20) == HAL_OK)
            {
                snprintf(scan_msg, sizeof(scan_msg),
                         "I2C FOUND device at 0x%02X\r\n", addr);
                Telnet_Send(scan_msg);
                HAL_Delay(10);
                found_any = 1;
            }
        }

        if (!found_any)
        {
            Telnet_Send("I2C SCAN: NO devices found - check SDA/SCL wiring!\r\n");
        }
        else
        {
            Telnet_Send("I2C SCAN complete.\r\n");
        }
        HAL_Delay(50);
    }

    /* --- BNO055 IMU Initialisation --- */
    if (BNO055_Init(&hi2c1) == HAL_OK)
    {
        imu_ok = 1;
        Telnet_Send("IMU BNO055 OK - NDOF fusion active\r\n");
    }
    else
    {
        imu_ok = 0;
        Telnet_Send("IMU BNO055 FAIL - sensor not responding\r\n");
        /* Not fatal - encoder/PWM control continues without IMU */
    }

    // Calibration initialization
    stable_timer = HAL_GetTick();
    last_encoder = encoder_count;
    calibration_state = 0;
    state_entry_timer   = HAL_GetTick();
    state_entry_encoder = encoder_count;

    uint32_t telnet_timer = 0;
    char telnet_msg[192];

    // Run calibration sequence
    /*while(calibration_state != 3)
    {
        MX_LWIP_Process();
        Steering_Calibration();

        if(HAL_GetTick() - telnet_timer >= 100)
        {
            telnet_timer = HAL_GetTick();

            snprintf(telnet_msg, sizeof(telnet_msg),
                     "CAL State=%d Enc=%ld Left=%ld Right=%ld Centre=%ld\r\n",
                     calibration_state,
                     encoder_count,
                     left_position,
                     right_position,
                     centre_position);

            Telnet_Send(telnet_msg);
        }
    }

    // Calibration finished
    Motor_Stop();
    HAL_Delay(1000);

    Engine_Startup_Shake();*/
    encoder_count = 0;

    /* ====================================================================
     * CRITICAL FIX: Re-apply Flash PWM settings to hardware registers
     * after Calibration and Engine Shake have finished zeroing the motors!
     * ==================================================================== */
    PWM_Set_Frequency(pwm_frequency_hz);
    if (pwm_manual_mode)
    {
        PWM_Set_Duty(pwm_duty_percent);
    }

    telnet_timer = 0;
    /* USER CODE END 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	/*  if(encoder_count > DEAD_BAND)
	  {
	   	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);
	   	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_SET);
	  }
	  else if(encoder_count < -DEAD_BAND)
	  {
	   	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);
	   	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_RESET);
	  }
	  else
	  {
	   	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);
	   	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_RESET);
	  } */

	  MX_LWIP_Process();
	  	  if(HAL_GetTick() - telnet_timer >= 100)
	  	  {
	  	      telnet_timer = HAL_GetTick();

	  	      /* Read real IMU or generate simulated values */
	  	      if (imu_ok)
	  	      {
	  	          BNO055_Read_Euler(&hi2c1, &imu_euler);
	  	      }
	  	      else
	  	      {
	  	          /* Simulated X/Y/Z — static counters, clearly sweep full range */
	  	          static int16_t sim_x = 0;      /* 0 to 3599  → 0.0 to 359.9 deg */
	  	          static int16_t sim_y = -1800;  /* -1800 to 1799 → -180.0 to 179.9 deg */
	  	          static int16_t sim_z = -900;   /* -900 to 899  → -90.0 to 89.9 deg */

	  	          sim_x += 5;   if (sim_x >= 3600) sim_x = 0;
	  	          sim_y += 7;   if (sim_y >= 1800) sim_y = -1800;
	  	          sim_z += 3;   if (sim_z >= 900)  sim_z = -900;

	  	          imu_euler.x = (float)sim_x / 10.0f;
	  	          imu_euler.y = (float)sim_y / 10.0f;
	  	          imu_euler.z = (float)sim_z / 10.0f;
	  	      }

	  	      /* Format X,Y,Z using integer math (no %f needed) */
	  	      int16_t ix  = (int16_t)imu_euler.x;
	  	      int16_t ixd = (int16_t)((imu_euler.x - (float)ix) * 10.0f);
	  	      if (ixd < 0) ixd = -ixd;
	  	      int16_t iy  = (int16_t)imu_euler.y;
	  	      int16_t iyd = (int16_t)((imu_euler.y - (float)iy) * 10.0f);
	  	      if (iyd < 0) iyd = -iyd;
	  	      int16_t iz  = (int16_t)imu_euler.z;
	  	      int16_t izd = (int16_t)((imu_euler.z - (float)iz) * 10.0f);
	  	      if (izd < 0) izd = -izd;

	  	      snprintf(telnet_msg, sizeof(telnet_msg),
	  	               "State=%d Enc=%ld Left=%ld Right=%ld Centre=%ld Freq=%luHz Duty=%u%% X=%d.%d Y=%d.%d Z=%d.%d\r\n",
	  	               calibration_state,
	  	               encoder_count,
	  	               left_position,
	  	               right_position,
	  	               centre_position,
	  	               pwm_frequency_hz,
	  	               pwm_duty_percent,
	  	               ix, ixd,
	  	               iy, iyd,
	  	               iz, izd);

	  	      Telnet_Send(telnet_msg);
	  	  }
	  	  //TIM4_CH3,CH4 CODE
	  	  // Hysteresis motor control

	  	  if(!pwm_manual_mode)
	  	  {
	  	      if(!centre_reached)
	  	      {
	  	          Move_To_Zero();
	  	      }
	  	      else
	  	      {
	  	          Steering_Control_Decel();

	  	          if(abs(encoder_count) > DEAD_BAND)
	  	          {
	  	              centre_reached = 3;
	  	          }
	  	          else if(abs(encoder_count) > -DEAD_BAND)
	  	          {
	  	              centre_reached = -3;
	  	          }
	  	      }              // closes "else" (centre_reached branch)
	  	  }                  // closes "if(!pwm_manual_mode)"
    }                      // closes "while(1)"
	    /* USER CODE END 3 */
}                        // closes "int main(void)"/* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 5;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}


/**
  * @brief I2C1 Initialization for BNO055 IMU
  *        SCL = PB6, SDA = PB7 (configured in HAL_I2C_MspInit in stm32f4xx_hal_msp.c)
  *        Speed: 400 kHz Fast Mode
  *        APB1 clock = 42 MHz
  * @retval None
  */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 400000;              /* 400 kHz Fast Mode */
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLED;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;

  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  // --- new R_EN / L_EN pins for BTS7960 ---
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;   // pick your actual pins
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);          // use whatever port these pins are on
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_SET);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_14)
    {
    	irq_count++;
        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15) == GPIO_PIN_SET)
            encoder_count++;
        else
            encoder_count--;
    }
}*/

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
		if(htim->Instance == TIM4)
		{
			if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
			{
				if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_15)==GPIO_PIN_SET)
				{
					encoder_count++;
				}
				else
				{
					encoder_count--;
				}
			}
		}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	  /* User can add his own implementation to report the HAL error return state */
	  __disable_irq();
	  while (1)
	  {
	  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	  /* User can add his own implementation to report the file name and line number,
		 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
