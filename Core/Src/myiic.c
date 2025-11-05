#include "myiic.h"

#include <stdbool.h>
#include <stdio.h>
/******************************************************************
 * 函 数 名 称：I2C_GPIO_Init
 * 函 数 说 明：I2C的引脚初始化
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
void i2c_gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(I2C_PORT, SCL_PIN|SDA_PIN, GPIO_PIN_SET);

  /*Configure GPIO pins : SCL_PIN SDA_PIN */
  GPIO_InitStruct.Pin = SCL_PIN|SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(I2C_PORT, &GPIO_InitStruct);
}



/*********************************************************
* 函数名称：i2c_delay
 * 函 数 说 明：I2C总线延时函数
* 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
*********************************************************/
void i2c_delay()
{
  uint32_t delay = 200;

  while (delay--)
  {
    __NOP();
  }
}
/*********************************************************

 * 函 数 名 称：i2c_start
 * 函 数 说 明：产生I2C起始信号
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
*********************************************************/
static void i2c_start()
{
  HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_SET); // 确保数据线为高电平
  HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_SET); // 确保时钟线为高电平
  i2c_delay();
  HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_RESET); // 拉低数据线，产生起始信号
  i2c_delay();
  HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_RESET); // 拉低时钟线，准备发送数据
  i2c_delay();
}
/*********************************************************
 * 函 数 名 称：i2c_stop
 * 函 数 说 明：产生I2C停止信号
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
*********************************************************/
static void i2c_stop()
{
  HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_RESET); // 确保数据线为低电平
  HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_SET); // 确保时钟线为高电平
  i2c_delay();
  HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_SET); // 拉高数据线，产生停止信号
  i2c_delay();
}
/*********************************************************
 * 函 数 名 称：i2c_write_byte
 * 函 数 说 明：向I2C总线写入一个字节，并读取ACK信号
 * 函 数 形 参：data - 要写入的数据字节
 * 函 数 返 回：uint8_t - ACK信号，0表示收到ACK，1表示收到NACK
 * 作       者：LC
 * 备       注：无
*********************************************************/
static bool i2c_write_byte(uint8_t data)
{
  uint8_t i;

  for (i = 0; i < 8; i++)
  {
    if (data & 0x80) // 检查数据的最高位
    {
      HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_SET); // 将数据线拉高，写入1
    }
    else
    {
      HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_RESET); // 将数据线拉低，写入0
    }

    HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_SET);  // 拉高时钟线，发送数据位
    i2c_delay();
    HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_RESET);  // 拉低时钟线，准备发送下一位数据
    i2c_delay();
    data <<= 1;  // 左移一位，准备发送下一位数据
  }

  HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_SET); // 拉高数据线，准备接收ACK信号
  HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_SET); // 拉高时钟线
  i2c_delay();
  bool ack = (HAL_GPIO_ReadPin(I2C_PORT, SDA_PIN) == GPIO_PIN_RESET); // 读取ACK信号
  HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_RESET); // 拉低时钟线
  i2c_delay();
  return ack;
}


/*********************************************************
 * 函 数 名 称：i2c_read_byte
 * 函 数 说 明：从I2C总线读取一个字节，并发送ACK或NACK信号
 * 函 数 形 参：ack - ACK信号，0表示发送ACK，1表示发送NACK
 * 函 数 返 回：uint8_t - 读取到的数据字节
 * 作       者：LC
 * 备       注：无
*********************************************************/
static uint8_t i2c_read_byte(bool ack)
{
  uint8_t i, data = 0;
  HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_SET); // 确保数据线为高电平

  for (i = 0; i < 8; i++)
  {
    data <<= 1; // 左移一位，准备接收数据位
    HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_SET); // 拉高时钟线，准备接收数据
    i2c_delay();


    if (HAL_GPIO_ReadPin(I2C_PORT, SDA_PIN))  // 检查数据线的状态
    {
      data |= 0x01;  // 如果数据线为高电平，将最低位设置为1
    }

    HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_RESET);  // 拉低时钟线，准备接收下一位数据
    i2c_delay();
  }

  HAL_GPIO_WritePin(I2C_PORT, SDA_PIN,
                    ack ? GPIO_PIN_RESET : GPIO_PIN_SET); // 发送ACK/NACK信号

  HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_SET); // 拉高时钟线，准备发送ACK或NACK信号
  i2c_delay();
  HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_RESET); // 拉低时钟线
  i2c_delay();
  return data; // 返回读取到的数据字节
}


/*********************************************************
 * 函 数 名 称：i2c_write_data
 * 函 数 说 明：向I2C设备写入数据
 * 函 数 形 参：dev_addr - 设备地址
  reg_addr - 寄存器地址
  data - 要写入的数据数组指针
  len - 数据长度
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
*********************************************************/
bool i2c_write_data(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint8_t len)
{
  bool success = true;
  i2c_start();
  if (!i2c_write_byte((dev_addr << 1) & 0xFE))  // 写入设备地址（写）
  {
    printf("I2C: device 0x%02X write-addr NACK\r\n", dev_addr);
    success = false;
    goto stop;
  }
  if (!i2c_write_byte(reg_addr))                // 写入寄存器地址
  {
    printf("I2C: device 0x%02X reg 0x%02X NACK\r\n", dev_addr, reg_addr);
    success = false;
    goto stop;
  }

  for (uint8_t i = 0; i < len; i++)
  {
    if (!i2c_write_byte(data[i]))              // 写入数据
    {
      printf("I2C: device 0x%02X data[%d]=0x%02X NACK\r\n", dev_addr, i, data[i]);
      success = false;
      goto stop;
    }
  }

stop:
  i2c_stop();
  return success;
}

/*********************************************************
 * 函 数 名 称：i2c_read_data
 * 函 数 说 明：从I2C设备读取数据
 * 函 数 形 参：dev_addr - 设备地址
  reg_addr - 寄存器地址
  data - 存储读取数据的数组指针
  len - 数据长度
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
*********************************************************/
bool i2c_read_data(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
  bool success = true;
  i2c_start();
  if (!i2c_write_byte((dev_addr << 1) & 0xFE))  // 写入设备地址
  {
    printf("I2C: device 0x%02X write-addr NACK\r\n", dev_addr);
    success = false;
    goto stop;
  }
  if (!i2c_write_byte(reg_addr))                // 写入寄存器地址
  {
    printf("I2C: device 0x%02X reg 0x%02X NACK\r\n", dev_addr, reg_addr);
    success = false;
    goto stop;
  }
  i2c_start();
  if (!i2c_write_byte((dev_addr << 1) | 0x01))  // 写入设备地址，读模式
  {
    printf("I2C: device 0x%02X read-addr NACK\r\n", dev_addr);
    success = false;
    goto stop;
  }

  for (uint8_t i = 0; i < len; i++)
  {
    bool send_ack = (i < (len - 1));
    data[i] = i2c_read_byte(send_ack);       // 读取字节
  }

stop:
  i2c_stop();
  return success;
}

/*********************************************************
 * 函 数 名 称：i2c_scan
 * 函 数 说 明：扫描I2C总线，记录应答设备地址
 * 函 数 形 参：start_addr/end_addr 扫描范围（7位地址）、found_list 保存数组、max_found 数组容量
 * 函 数 返 回：发现的设备数量
 * 作       者：ChatGPT
 * 备       注：found_list 只保存前max_found个地址
*********************************************************/
uint8_t i2c_scan(uint8_t start_addr, uint8_t end_addr, uint8_t *found_list, uint8_t max_found)
{
  if (start_addr < 0x03)
  {
    start_addr = 0x03;
  }
  if (end_addr > 0x77)
  {
    end_addr = 0x77;
  }
  if (start_addr > end_addr)
  {
    return 0;
  }

  uint8_t count = 0;

  for (uint8_t addr = start_addr; addr <= end_addr; addr++)
  {
    i2c_start();
    bool ack = i2c_write_byte((addr << 1) & 0xFE);
    i2c_stop();

    if (ack)
    {
      if (count < max_found && found_list != NULL)
      {
        found_list[count] = addr;
      }
      count++;
    }
  }

  return count;
}