#include "stm32f4xx_hal.h"              // Device:STM32Cube HAL:Common

// Handle structures for Timer and ADC
TIM_HandleTypeDef htim2;
ADC_HandleTypeDef hadc1;
uint16_t adcValue;
uint16_t servoValue;

// GPIO Configuration for Servo Control
// PA2 is configured as TIM2_CH3 for PWM output
void gpioConfig()
{
   __HAL_RCC_GPIOA_CLK_ENABLE();      // Enable GPIOA clock
   
   GPIO_InitTypeDef gpioStructA;       // Define GPIO structure
   
   gpioStructA.Mode=GPIO_MODE_AF_PP;   // Alternate function push-pull mode for PWM
   gpioStructA.Pin=GPIO_PIN_2;         // Select PA2 pin
   gpioStructA.Alternate=GPIO_AF1_TIM2; // TIM2 alternate function
   gpioStructA.Speed=GPIO_SPEED_FREQ_MEDIUM;  // Medium speed is sufficient for servo
   gpioStructA.Pull=GPIO_NOPULL;       // No pull-up/down resistor
   
   HAL_GPIO_Init(GPIOA, &gpioStructA); // Initialize GPIOA with these settings
}

// Timer Configuration for Servo PWM
// Generates 50Hz PWM signal (20ms period)
// Pulse width: 0.5ms (0°) to 2.5ms (180°)
void timConfig()
{
   __HAL_RCC_TIM2_CLK_ENABLE();        // Enable TIM2 clock
   
   TIM_OC_InitTypeDef sConfigOC = {0}; // PWM configuration structure
   
   htim2.Instance=TIM2;                // Use TIM2
   htim2.Init.Prescaler=47;            // Set prescaler: 48MHz/48 = 1MHz timer clock
   htim2.Init.Period=19999;            // 50Hz PWM (1MHz/50Hz = 20000)
   htim2.Init.CounterMode= TIM_COUNTERMODE_UP;  // Up-counting mode
   htim2.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;  // No clock division
   htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  // Disable auto-reload preload
   
   HAL_TIM_PWM_Init(&htim2);          // Initialize timer for PWM
   
   //TIM PWM Configuration
   sConfigOC.OCMode = TIM_OCMODE_PWM1; // PWM mode 1
   sConfigOC.Pulse = 1500;             // 1.5ms pulse width (90° position)
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;  // High polarity
   
   HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);  // Apply settings to Channel 3
}

// Error Handler function
void Error_Handler(void)
{
    while(1)
    {
    }
}
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 192;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 8;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    {
        Error_Handler();
    }
}

int main()
{
	HAL_Init();
	SystemClock_Config();
	timConfig();
	gpioConfig();
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	
	while(1)
	{
        // Sweep from 0° to 180° in small steps
        // 500µs (0°) to 2500µs (180°) in 50µs steps
		for(int i=500;i<=2500;i+=50)
		{
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, i);
			HAL_Delay(200);
		}
		
        // Sweep from 180° back to 0° in small steps
		for(int i=2500;i>=500;i-=50)
		{
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, i);
			HAL_Delay(200);
		}
		
        // Test specific positions:
        // Move to 90° (1.5ms pulse)
	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1500);
		HAL_Delay(2000);
        // Move to 0° (0.5ms pulse)
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 500);
		HAL_Delay(2000);
        // Move to 180° (2.5ms pulse)
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 2500);
		HAL_Delay(2000);
	}	
	
}

// System tick interrupt handler
// Required for HAL_Delay function
void SysTick_Handler(void) {
    HAL_IncTick();
}