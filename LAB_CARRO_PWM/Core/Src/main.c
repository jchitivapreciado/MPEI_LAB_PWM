/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Navegador autónomo de polígonos regulares
  *                   Avance por ENCODER + corrección MPU + Giro MPU
  *                   ANTICIPO ajustado para 90° exactos.
  *                   Encoder verifica el giro como sanity check.
  * Microcontrolador: STM32F103C8T6 @ 72 MHz
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ============================================================================
// ★★★ PARÁMETROS CONFIGURABLES ★★★
// ============================================================================
#define N_LADOS          3          /* 3=triángulo, 4=cuadrado, 5=pentágono */
#define DISTANCIA_CM     50.0f      /* Longitud de cada lado en cm */
#define N_VUELTAS        2          /* Repeticiones del polígono */

// ============================================================================
// CALIBRACIÓN DEL ENCODER  →  ¡AJUSTAR ESTA LÍNEA!
// ============================================================================
// Pulsos que cuenta una rueda por cada centímetro recorrido.
// Calibración: corre el robot 1 vez. En la LCD verás "Pulsos: NNN" al terminar.
// Luego:  PULSOS_POR_CM = pulsos_contados / DISTANCIA_CM
#define PULSOS_POR_CM    2.0f

// ============================================================================
// PARÁMETROS INTERNOS
// ============================================================================
#define ANGULO_GIRO      (360.0f / (float)N_LADOS)

// --- Avance ---
#define VELOCIDAD_BASE   500
#define MAX_PWM          799
#define RAMPA_PASO_PWM   25
#define PAUSA_ENTRE_ACCIONES_MS  300U

// --- Corrección de rumbo durante el avance (MPU) ---
#define KP_LINEA_RECTA       3.0f
#define CORRECCION_MAX       80

// --- Safety: si el encoder no cuenta nada, salir por tiempo ---
#define TIMEOUT_AVANCE_FACTOR  1.5f
#define MS_POR_CM_REF          40U

// --- Giro (ambas ruedas opuestas + kick) ---
#define PWM_GIRO         240
#define PWM_GIRO_KICK    350
#define MS_GIRO_KICK     80U
#define ANTICIPO_GIRO_DEG 3.0f
#define TIMEOUT_GIRO_MS  5000U

// --- Verificación del giro con encoder ---
#define MIN_PULSOS_GIRO  8

// --- MPU6050 ---
#define MPU6050_ADDR            (0x68 << 1)
#define MPU6050_REG_WHOAMI      0x75
#define MPU6050_REG_PWR_MGMT1   0x6B
#define MPU6050_REG_CONFIG      0x1A
#define MPU6050_REG_GYRO_CFG    0x1B
#define MPU6050_REG_GYRO_ZOUT_H 0x47
#define GYRO_SENS_131           131.0f

#define LCD_REFRESH_MS   250U
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
// Encoders
volatile int32_t enc_izq = 0;
volatile int32_t enc_der = 0;

// MPU
float gyro_z_offset     = 0.0f;
float yaw_actual        = 0.0f;
uint32_t ultimo_tiempo_imu = 0;
uint8_t mpu_ok          = 0;

static int pwm_actual_izq = 0;
static int pwm_actual_der = 0;

// Bandera de fallo del giro (para reportar en LCD)
uint8_t giro_fallo_encoder = 0;
/* USER CODE END PV */

/* Prototipos ----------------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
void LCD_Init(void);
void LCD_SendCommand(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_PrintString(char *str);
void LCD_Clear(void);

static int  Acercar_Valor(int actual, int objetivo, int paso);
static void Motores_SetRaw(int izq, int der);
static void Motores_AplicarConRampa(int izq, int der);
void Motores_Frenar(void);

uint8_t MPU6050_Init(void);
void    MPU6050_Calibrar(void);
void    MPU6050_ActualizarYaw(void);

void Avanzar_Distancia(float cm);
void Girar_Grados(float angulo_deg);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// ============================================================================
// 1. DRIVER LCD 16x2
// ============================================================================
static void LCD_EnablePulse(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
    for (volatile int i = 0; i < 200; i++);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    for (volatile int i = 0; i < 200; i++);
}

static void LCD_Write4Bits(uint8_t value)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,  (value & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,  (value & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, (value & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, (value & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    LCD_EnablePulse();
}

static void LCD_WriteByte(uint8_t value, uint8_t isData)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, isData ? GPIO_PIN_SET : GPIO_PIN_RESET);
    LCD_Write4Bits(value >> 4);
    LCD_Write4Bits(value & 0x0F);
}

void LCD_SendCommand(uint8_t cmd) { LCD_WriteByte(cmd, 0); }
void LCD_SendData(uint8_t data)  { LCD_WriteByte(data, 1); }

void LCD_Init(void)
{
    HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_Delay(50);

    LCD_Write4Bits(0x03); HAL_Delay(15);
    LCD_Write4Bits(0x03); HAL_Delay(15);
    LCD_Write4Bits(0x03); HAL_Delay(15);
    LCD_Write4Bits(0x02); HAL_Delay(15);

    LCD_SendCommand(0x28); HAL_Delay(10);
    LCD_SendCommand(0x28); HAL_Delay(10);
    LCD_SendCommand(0x08); HAL_Delay(10);
    LCD_SendCommand(0x01); HAL_Delay(15);
    LCD_SendCommand(0x06); HAL_Delay(10);
    LCD_SendCommand(0x0C); HAL_Delay(10);
}

void LCD_Clear(void) { LCD_SendCommand(0x01); HAL_Delay(10); }

void LCD_SetCursor(uint8_t row, uint8_t col)
{
    uint8_t addr = (row == 0) ? (0x80 | col) : (0xC0 | col);
    LCD_SendCommand(addr);
}

void LCD_PrintString(char *str)
{
    while (*str) { LCD_SendData((uint8_t)(*str)); str++; }
}

// ============================================================================
// 2. MOTORES
// ============================================================================
static void Motores_SetRaw(int izq, int der)
{
    if (izq >  MAX_PWM) izq =  MAX_PWM;
    if (izq < -MAX_PWM) izq = -MAX_PWM;
    if (der >  MAX_PWM) der =  MAX_PWM;
    if (der < -MAX_PWM) der = -MAX_PWM;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    int i = izq;
    if (i >= 0)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        i = -i;
    }

    int d = der;
    if (d >= 0)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
        d = -d;
    }

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, i);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, d);
}

void Motores_Frenar(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

    pwm_actual_izq = 0;
    pwm_actual_der = 0;
}

static int Acercar_Valor(int actual, int objetivo, int paso)
{
    if (actual < objetivo) { actual += paso; if (actual > objetivo) actual = objetivo; }
    else if (actual > objetivo) { actual -= paso; if (actual < objetivo) actual = objetivo; }
    return actual;
}

static void Motores_AplicarConRampa(int izq_obj, int der_obj)
{
    pwm_actual_izq = Acercar_Valor(pwm_actual_izq, izq_obj, RAMPA_PASO_PWM);
    pwm_actual_der = Acercar_Valor(pwm_actual_der, der_obj, RAMPA_PASO_PWM);
    Motores_SetRaw(pwm_actual_izq, pwm_actual_der);
}

// ============================================================================
// 3. INTERRUPCIÓN DE ENCODERS
// ============================================================================
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t t_izq = 0, t_der = 0;
    uint32_t ahora = HAL_GetTick();

    if (GPIO_Pin == GPIO_PIN_12)
    {
        if ((ahora - t_izq) >= 3U) { enc_izq++; t_izq = ahora; }
    }
    if (GPIO_Pin == GPIO_PIN_13)
    {
        if ((ahora - t_der) >= 3U) { enc_der++; t_der = ahora; }
    }
}

// ============================================================================
// 4. MPU6050
// ============================================================================
uint8_t MPU6050_Init(void)
{
    uint8_t check = 0, data;
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_REG_WHOAMI,
                         1, &check, 1, 200) != HAL_OK) return 0;
    if (check != 0x68) return 0;

    data = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_REG_PWR_MGMT1, 1, &data, 1, 1000);
    data = 0x03;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_REG_CONFIG, 1, &data, 1, 1000);
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_REG_GYRO_CFG, 1, &data, 1, 1000);

    return 1;
}

void MPU6050_Calibrar(void)
{
    int32_t suma_gz = 0;
    uint8_t buffer[2];
    int16_t raw_gz;
    int validas = 0;

    for (int i = 0; i < 500; i++)
    {
        if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_REG_GYRO_ZOUT_H,
                             1, buffer, 2, 50) == HAL_OK)
        {
            raw_gz = (int16_t)((buffer[0] << 8) | buffer[1]);
            suma_gz += raw_gz;
            validas++;
        }
        HAL_Delay(4);
    }
    if (validas > 0) gyro_z_offset = (float)suma_gz / (float)validas;
}

void MPU6050_ActualizarYaw(void)
{
    if (!mpu_ok) return;

    uint8_t buffer[2];
    int16_t raw_gz;
    uint32_t ahora = HAL_GetTick();
    float dt = (float)(ahora - ultimo_tiempo_imu) / 1000.0f;
    ultimo_tiempo_imu = ahora;

    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_REG_GYRO_ZOUT_H,
                         1, buffer, 2, 50) == HAL_OK)
    {
        raw_gz = (int16_t)((buffer[0] << 8) | buffer[1]);
        float gz = ((float)raw_gz - gyro_z_offset) / GYRO_SENS_131;
        if (gz > -0.5f && gz < 0.5f) gz = 0.0f;
        yaw_actual += gz * dt;
    }
}

// ============================================================================
// 5. AVANCE — ENCODER + CORRECCIÓN MPU
// ============================================================================
void Avanzar_Distancia(float cm)
{
    pwm_actual_izq = 0;
    pwm_actual_der = 0;
    enc_izq = 0;
    enc_der = 0;
    yaw_actual = 0.0f;
    ultimo_tiempo_imu = HAL_GetTick();

    int32_t pulsos_objetivo = (int32_t)(cm * PULSOS_POR_CM + 0.5f);
    uint32_t tiempo_max_ms = (uint32_t)(cm * (float)MS_POR_CM_REF * TIMEOUT_AVANCE_FACTOR);
    uint32_t t0 = HAL_GetTick();

    while (1)
    {
        uint32_t transcurrido = HAL_GetTick() - t0;

        int32_t promedio = (enc_izq + enc_der) / 2;
        if (promedio >= pulsos_objetivo) break;
        if (transcurrido > tiempo_max_ms) break;

        MPU6050_ActualizarYaw();

        float error = 0.0f - yaw_actual;
        float correccion = error * KP_LINEA_RECTA;
        if (correccion >  (float)CORRECCION_MAX) correccion =  (float)CORRECCION_MAX;
        if (correccion < -(float)CORRECCION_MAX) correccion = -(float)CORRECCION_MAX;

        int pwm_izq_obj = VELOCIDAD_BASE - (int)correccion;
        int pwm_der_obj = VELOCIDAD_BASE + (int)correccion;

        Motores_AplicarConRampa(pwm_izq_obj, pwm_der_obj);
        HAL_Delay(10);
    }

    Motores_Frenar();
    HAL_Delay(PAUSA_ENTRE_ACCIONES_MS);
}

// ============================================================================
// 6. GIRO — AMBAS RUEDAS OPUESTAS + KICK + VERIFICACIÓN CON ENCODER
// ============================================================================
void Girar_Grados(float angulo_deg)
{
    yaw_actual = 0.0f;
    ultimo_tiempo_imu = HAL_GetTick();
    pwm_actual_izq = 0;
    pwm_actual_der = 0;

    // Reset de encoders para medir cuánto giraron las ruedas
    enc_izq = 0;
    enc_der = 0;

    float angulo_destino = -angulo_deg;
    uint32_t t0 = HAL_GetTick();

    // Kick inicial: ambos motores arrancan al mismo tiempo
    Motores_SetRaw(PWM_GIRO_KICK, -PWM_GIRO_KICK);
    HAL_Delay(MS_GIRO_KICK);

    while (yaw_actual > (angulo_destino + ANTICIPO_GIRO_DEG))
    {
        if ((HAL_GetTick() - t0) > TIMEOUT_GIRO_MS) break;

        MPU6050_ActualizarYaw();
        Motores_SetRaw(PWM_GIRO, -PWM_GIRO);
        HAL_Delay(10);
    }

    Motores_Frenar();
    HAL_Delay(PAUSA_ENTRE_ACCIONES_MS);
}
/* USER CODE END 0 */

// ============================================================================
// MAIN
// ============================================================================
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  Motores_Frenar();
  HAL_Delay(500);

  LCD_Init();
  LCD_Clear();
  LCD_SetCursor(0, 0); LCD_PrintString("Poligono + Enc");
  LCD_SetCursor(1, 0); LCD_PrintString("Iniciando...");
  HAL_Delay(2000);

  LCD_Clear();
  LCD_SetCursor(0, 0); LCD_PrintString("Init MPU6050");

  mpu_ok = MPU6050_Init();

  if (!mpu_ok)
  {
      LCD_SetCursor(1, 0); LCD_PrintString("NO detectado!");
      HAL_Delay(2500);
  }
  else
  {
      LCD_SetCursor(1, 0); LCD_PrintString("Calibrando...");
      HAL_Delay(2500);
      MPU6050_Calibrar();
      HAL_Delay(500);
  }

  LCD_Clear();
  LCD_SetCursor(0, 0); LCD_PrintString("Iniciando en");
  for (int i = 3; i > 0; i--)
  {
      char buf[6];
      snprintf(buf, sizeof(buf), "%d...", i);
      LCD_SetCursor(1, 0); LCD_PrintString(buf);
      HAL_Delay(1000);
  }
  LCD_Clear();

  uint32_t total_pasos = (uint32_t)N_LADOS * (uint32_t)N_VUELTAS;

  for (uint32_t paso = 0; paso < total_pasos; paso++)
  {
      char l1[17], l2[17];
      int vuelta_actual = (int)(paso / (uint32_t)N_LADOS) + 1;
      int lado_actual   = (int)(paso % (uint32_t)N_LADOS) + 1;
      snprintf(l1, sizeof(l1), "Vuelta %d/%d", vuelta_actual, N_VUELTAS);
      snprintf(l2, sizeof(l2), "Lado   %d/%d", lado_actual,   N_LADOS);
      LCD_SetCursor(0, 0); LCD_PrintString(l1);
      LCD_SetCursor(1, 0); LCD_PrintString(l2);

      // Avanza por encoder
      Avanzar_Distancia(DISTANCIA_CM);

      // Muestra el conteo de pulsos del tramo (para calibrar)
      snprintf(l1, sizeof(l1), "Pulsos: %ld", (long)((enc_izq + enc_der) / 2));
      LCD_SetCursor(0, 0); LCD_PrintString(l1);

      // Gira solo si no es el último paso
      if (paso < (total_pasos - 1))
      {
          // Guardamos los encoders antes del giro para verificar después
          int32_t enc_izq_antes = enc_izq;
          int32_t enc_der_antes = enc_der;

          Girar_Grados(ANGULO_GIRO);

          // Verificación: ¿ambas ruedas giraron?
          int32_t d_izq = enc_izq - enc_izq_antes;
          int32_t d_der = enc_der - enc_der_antes;
          if (d_izq < MIN_PULSOS_GIRO || d_der < MIN_PULSOS_GIRO)
          {
              giro_fallo_encoder = 1;
              snprintf(l2, sizeof(l2), "Giro fallo !");
              LCD_SetCursor(1, 0); LCD_PrintString(l2);
              HAL_Delay(800);
          }
      }
  }

  Motores_Frenar();

  LCD_Clear();
  LCD_SetCursor(0, 0);
  LCD_PrintString("Recorrido");

  LCD_SetCursor(1, 0);
  if (giro_fallo_encoder)
  {
      LCD_PrintString("Rev. giros !");
  }
  else
  {
      LCD_PrintString("Completado!");
  }
  /* USER CODE END 2 */

  while (1)
  {
  }
}

// ============================================================================
// CONFIGURACIÓN DE PERIFÉRICOS
// ============================================================================
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 799;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
  HAL_TIM_MspPostInit(&htim2);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif
