/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 *
 * Copyright (c) 2024 Jan Zwiener (jan@zwiener.org)
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "stm32f429i_discovery_lcd.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/

#include "../../Peanut-GB/peanut_gb.h"

// Generate by xxd -i gameboy_rom.gb > gameboy_rom.h
//
// Adding static const in front so that the ROM data
// ends up read-only in flash memory.
static const
#include "gameboy_rom.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define WIDTH 240 /**< framebuffer width in pixel */
#define HEIGHT 320 /**< framebuffer height in pixel */
#define BPP 4 /**< bytes/pixel */
#define COLOR(r,g,b)   ((uint32_t)(0xff000000 | ((r) << 16) | ((g) << 8) | (b)))

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart1;
DMA2D_HandleTypeDef hdma2d;
LTDC_HandleTypeDef hltdc;
SDRAM_HandleTypeDef hsdram1;
RNG_HandleTypeDef hrng;

static int LCD_LAYER_FRONT; // active display layer (front buffer)
static int LCD_LAYER_BACK;
static uint32_t* g_fb[2];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_USART1_UART_Init(void);

void mainTask(void);
void render(const float dt_sec, float forward, float left);
void clearFrameBuffer(void);

/* Private user code ---------------------------------------------------------*/

int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
    SystemClock_Config();

    /* Serial output (UART)  */
    // MX_DMA_Init();
    MX_USART1_UART_Init();
    /* Setup hardware random number generator */
    hrng.Instance = RNG;
    HAL_RNG_Init(&hrng);

    /* Display setup */
    BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

    BSP_LCD_Init();
    LCD_LAYER_FRONT = 1;
    LCD_LAYER_BACK = 0;
    g_fb[0] = (uint32_t*)LCD_FRAME_BUFFER;
    g_fb[1] = (uint32_t*)(LCD_FRAME_BUFFER + WIDTH * HEIGHT * BPP);
    BSP_LCD_LayerDefaultInit(0, (uint32_t)g_fb[0]);
    BSP_LCD_LayerDefaultInit(1, (uint32_t)g_fb[1]);
    BSP_LCD_SetLayerVisible(0, DISABLE);
    BSP_LCD_SetLayerVisible(1, ENABLE);
    BSP_LCD_SelectLayer(LCD_LAYER_BACK);

    /* ChromART (DMA2D) setup */
    hdma2d.Init.Mode         = DMA2D_M2M; // convert 8bit palette colors to 32bit ARGB888
    hdma2d.Init.ColorMode    = DMA2D_ARGB8888; // destination color format
    hdma2d.Init.OutputOffset = 0;
    hdma2d.Instance = DMA2D;
    hdma2d.LayerCfg[LCD_LAYER_FRONT].AlphaMode = DMA2D_NO_MODIF_ALPHA;
    hdma2d.LayerCfg[LCD_LAYER_FRONT].InputAlpha = 0xFF; // N/A only for A8 or A4
    hdma2d.LayerCfg[LCD_LAYER_FRONT].InputColorMode = DMA2D_INPUT_ARGB8888; // source format
    hdma2d.LayerCfg[LCD_LAYER_FRONT].InputOffset = 0;
    HAL_DMA2D_Init(&hdma2d);
    HAL_DMA2D_ConfigLayer(&hdma2d, LCD_LAYER_FRONT);

    /* Enable CPU cycle counter */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    // access the cycle counter at: DWT->CYCCNT

    /* Setup Game Boy Emulator */

    /* Run Main task */
    mainTask();

    while (1) {} /* should never end up here */
}

void screen_flip_buffers(void)
{
  // wait for VSYNC
    while (!(LTDC->CDSR & LTDC_CDSR_VSYNCS)) {}

    BSP_LCD_SetLayerVisible(LCD_LAYER_FRONT, DISABLE);
    LCD_LAYER_BACK = LCD_LAYER_FRONT;
    LCD_LAYER_FRONT ^= 1;
    BSP_LCD_SetLayerVisible(LCD_LAYER_FRONT, ENABLE);
    BSP_LCD_SelectLayer(LCD_LAYER_BACK);

}
/* Using the Systick 1000 Hz millisecond timer to sleep */
static void sleep(uint32_t delayMs)
{
    const uint32_t tickstart = HAL_GetTick();
    while((HAL_GetTick() - tickstart) < delayMs)
    {
        __WFE(); // save a bit of power while we are waiting
    }
}

static void clearBuffer(uint32_t* fb)
{
    int y;
    for (y = 0; y < HEIGHT; y++)
    {
        for (int x = 0; x < WIDTH; x++)
        {
            fb[y * WIDTH + x] = COLOR(10, 169, 216);
        }
    }
}

// -----------------------------------------------------------------------
// GB specific
// -----------------------------------------------------------------------

struct priv_t
{
    uint8_t *rom; /* Pointer to allocated memory holding GB file. */
    uint8_t *cart_ram; /* Pointer to allocated memory holding save file. */
};

/**
 * Returns a byte from the ROM file at the given address.
 */
uint8_t gb_rom_read(struct gb_s *gb, const uint_fast32_t addr)
{
    const struct priv_t * const p = gb->direct.priv;
    return p->rom[addr];
}

/**
 * Returns a byte from the cartridge RAM at the given address.
 */
uint8_t gb_cart_ram_read(struct gb_s *gb, const uint_fast32_t addr)
{
    const struct priv_t * const p = gb->direct.priv;
    return p->cart_ram[addr];
}

/**
 * Writes a given byte to the cartridge RAM at the given address.
 */
void gb_cart_ram_write(struct gb_s *gb, const uint_fast32_t addr,
               const uint8_t val)
{
    const struct priv_t * const p = gb->direct.priv;
    p->cart_ram[addr] = val;
}

/**
 * Ignore all errors.
 */
void gb_error(struct gb_s *gb, const enum gb_error_e gb_err, const uint16_t val)
{
    assert(0);
}

/**
 * Draws scanline into framebuffer.
 */
void lcd_draw_line(struct gb_s *gb, const uint8_t pixels[160],
           const uint_fast8_t line)
{
    const uint32_t palette[] = { COLOR(255,255,255), COLOR(0xA5,0xA5,0xA5), COLOR(0x52,0x52,0x52), COLOR(0,0,0) };
    uint32_t* fb = g_fb[LCD_LAYER_BACK];

    const int sx = 40;
    const int sy = 88;
    for(unsigned int x = 0; x < LCD_WIDTH; x++)
        fb[(line+sy) * WIDTH + x + sx] = palette[pixels[x] & 3];
}

void mainTask(void)
{
    int frameTimeMs = 0; // current frametime in ms
    const int setpointframeTimeMs = 16;

    // setup game boy
    static struct gb_s gb;
    static struct priv_t priv; // pass-through data for peanut
    static uint8_t g_ram[0x20000]; // excessive, but should work in every case

    priv.rom = (uint8_t*)gameboy_rom_gb;
    priv.cart_ram = g_ram;

    gb_init(&gb, &gb_rom_read, &gb_cart_ram_read,
            &gb_cart_ram_write, &gb_error, &priv);

    clearBuffer(g_fb[LCD_LAYER_FRONT]);
    clearBuffer(g_fb[LCD_LAYER_BACK]);

    gb_init_lcd(&gb, &lcd_draw_line);

    for(uint32_t epoch=0;;epoch++)
    {
        uint32_t tickStart = HAL_GetTick();

        /*
        kb[SDL_SCANCODE_W] = (BSP_PB_GetState(BUTTON_KEY) != RESET);
        */

        gb_run_frame(&gb);
        screen_flip_buffers();

        frameTimeMs = (int)(HAL_GetTick() - tickStart);
        const int timeleftMs = setpointframeTimeMs - frameTimeMs;
        if (timeleftMs > 0)
        {
            sleep(timeleftMs);
        }

        // Send the frametime in milliseconds via ASCII over UART to a host PC for debugging/optimization
        /*
        uint8_t uartAsciiOutput[128]; // debug ASCII output buffer for UART sending
        if (epoch % 60 == 0 && HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY)
        {
            const int bytesInBuffer =
                    snprintf((char*)uartAsciiOutput, sizeof(uartAsciiOutput), "%i ms\r\n", frameTimeMs);
            HAL_UART_Transmit(&huart1, uartAsciiOutput, bytesInBuffer, 32);
        }
        */
    }
}

// -----------------------------------------------------------------------
// STM32 stuff
// -----------------------------------------------------------------------

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Enable HSE Oscillator and activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 360;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /* Activate the Over-Drive mode */
    HAL_PWREx_EnableOverDrive();

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        HAL_IncTick();
    }
}

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
}

void HAL_RNG_MspInit(RNG_HandleTypeDef *hrng)
{
    __RNG_CLK_ENABLE(); /* RNG Peripheral clock enable */
}

#ifdef  USE_FULL_ASSERT
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

