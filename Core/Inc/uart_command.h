#pragma once
#include <stdint.h>
#include <stddef.h>

/* 可调节的命令解析参数 */
#define CMD_MAX_LEN         16      /* 单个命令名称最大长度（不含终止符） */
#define CMD_MAX_ARGS        8       /* 参数最大数量（含命令本身位置0） */
#define CMD_ARG_MAX_LEN     16      /* 每个参数最大长度（不含终止符） */
#define CMD_LINE_MAX        128     /* 输入一行最大长度（含终止符） */

typedef void (*CommandHandler)(int argc, char *argv[]);

typedef struct {
  const char *name;              /* 命令名（大写推荐） */
  CommandHandler handler;        /* 处理函数 */
} CommandEntry;



typedef struct {
  const char *name;
  float * value;
} Value;

/* 主解析入口：data 可以包含尾部的 \r 或 \n，会被自动裁剪 */
void handle_command(const uint8_t *data, uint16_t len);

void add_mutable_value(const char *name, float *value);

void print_handler();

void add_printable_value(const char *name, float *value);
void add_mutable_value(const char *name, float *value);

