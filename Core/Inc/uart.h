
#include "stm32f1xx_hal.h"

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

//void print_u1(char *, unsigned int);
int _write(int file, char *data, int len);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
