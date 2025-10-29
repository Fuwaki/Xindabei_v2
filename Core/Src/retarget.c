// #include "usart.h"
// #include <sys/stat.h>
// #include <errno.h>

// int __io_putchar(int ch)
// {
//     HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
//     return ch;
// }

// int _write(int file, char *ptr, int len)
// {
//     (void)file;
//     HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
//     return len;
// }