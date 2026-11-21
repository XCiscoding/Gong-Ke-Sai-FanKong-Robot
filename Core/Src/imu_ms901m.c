#include "imu_ms901m.h"
#include "main.h"
#include <stdio.h>

volatile ImuMs901mDiagnostics imu_ms901m_diag;
static ImuMs901mParser parser;
static volatile uint32_t uart_errors;
static uint32_t started_ms, last_log_ms;

#if IMU_ONLY_TEST_ACTIVE && IMU_DIAGNOSTIC_LOG_ENABLED
static void LogLine(const char *line)
{
    uint32_t started = HAL_GetTick();
    while (*line) {
        while (!(UART4->SR & USART_SR_TXE)) {
            if ((uint32_t)(HAL_GetTick() - started) > 25U) return;
        }
        UART4->DR = (uint8_t)*line++;
    }
}
#endif
void ImuMs901m_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    const ImuMs901mDiagnostics initial = {0};
    imu_ms901m_diag = initial;
    ImuMs901m_ParserInit(&parser);
    uart_errors = 0U;
    started_ms = last_log_ms = HAL_GetTick();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_UART5_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD, &gpio);
    /* Receive only: PC12 and the UART5 transmitter remain unconfigured. */
    UART5->CR1 = UART5->CR2 = UART5->CR3 = 0U;
    UART5->BRR = (HAL_RCC_GetPCLK1Freq() + IMU_MS901M_BAUDRATE / 2U) / IMU_MS901M_BAUDRATE;
    UART5->CR1 = USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
    HAL_NVIC_SetPriority(UART5_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(UART5_IRQn);

#if IMU_ONLY_TEST_ACTIVE && IMU_DIAGNOSTIC_LOG_ENABLED
    /* F407 emits parsed/solved verification data on a separate TX-only port. */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_UART4_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_10;
    gpio.Pull = GPIO_NOPULL;
    gpio.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &gpio);
    /* TX only: PC11 and the UART4 receiver remain unconfigured. */
    UART4->CR1 = UART4->CR2 = UART4->CR3 = 0U;
    UART4->BRR = (HAL_RCC_GetPCLK1Freq() + 57600U) / 115200U;
    UART4->CR1 = USART_CR1_TE | USART_CR1_UE;
    LogLine("F407 MS901M solve; MOTOR OFF; PD2/UART5 RX; PC10/UART4 TX; keep still 2s\r\n");
#endif
}
void UART5_IRQHandler(void)
{
    uint32_t status = UART5->SR;
    if (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        uint8_t byte = (uint8_t)UART5->DR;
        if (status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
            uart_errors++;
            parser.length = 0U;
        } else if (status & USART_SR_RXNE) {
            ImuMs901m_Feed(&parser, byte, HAL_GetTick());
        }
    }
}
void ImuMs901m_Service(void)
{
    uint32_t now, primask = __get_PRIMASK();
    ImuMs901mSnapshot data;
    __disable_irq();
    data = parser.data;
    imu_ms901m_diag.uart_errors = uart_errors;
    if (primask == 0U) __enable_irq();
    now = HAL_GetTick();
    imu_ms901m_diag.data = data;
    imu_ms901m_diag.age_ms = data.angle_frames ? (uint32_t)(now - data.last_angle_ms) : UINT32_MAX;
    imu_ms901m_diag.state = !data.angle_frames ? 0U : ImuMs901m_IsFresh(&data, now) ? 1U : 2U;
    if (!imu_ms901m_diag.zero_ready && (uint32_t)(now - started_ms) >= 2000U &&
        data.angle_frames >= 5U && imu_ms901m_diag.state == 1U) {
        imu_ms901m_diag.zero_yaw_tenths = data.yaw_tenths;
        imu_ms901m_diag.zero_ready = 1U;
    }
    if (imu_ms901m_diag.zero_ready) {
        imu_ms901m_diag.relative_yaw_tenths = ImuMs901m_AngleDifference(data.yaw_tenths,
                                                   imu_ms901m_diag.zero_yaw_tenths);
    }
#if IMU_ONLY_TEST_ACTIVE && IMU_DIAGNOSTIC_LOG_ENABLED
    if ((uint32_t)(now - last_log_ms) >= 250U) {
        char line[160];
        last_log_ms = now;
        (void)snprintf(line, sizeof(line),
            "IMU state=%lu bytes=%lu frames=%lu age=%lu yaw10=%ld delta10=%ld zero=%lu err=%lu/%lu\r\n",
            (unsigned long)imu_ms901m_diag.state, (unsigned long)data.raw_bytes,
            (unsigned long)data.angle_frames, (unsigned long)imu_ms901m_diag.age_ms,
            (long)data.yaw_tenths, (long)imu_ms901m_diag.relative_yaw_tenths,
            (unsigned long)imu_ms901m_diag.zero_ready,
            (unsigned long)data.checksum_errors, (unsigned long)imu_ms901m_diag.uart_errors);
        LogLine(line);
    }
#endif
}
