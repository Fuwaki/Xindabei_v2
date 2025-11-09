#include "uart_command.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/_intsup.h>

static void set_speed_handler(int argc, char *argv[]) {
  if (argc < 2) {
    printf("ERR\n");
    return;
  }
  int value = atoi(argv[1]);
  printf("Speed set to %d\n", value);
}
float a=1.0;
static unsigned short mutable_value_count = 1;
Value muttable_values[16] = {
    {"a", &a}

};

static unsigned short print_value_count = 1;
float *printable_values[16] = {
    &a

};
void print_handler() {
  for (size_t i = 0; i < print_value_count; ++i) {
    printf("%f", *printable_values[i]);
    if (i < print_value_count - 1) {
      printf(",");
    } else {
      printf("\n");
    }
  }
}

void add_printable_value(const char *name, float *value) {
  if (print_value_count < 16) {
    printable_values[print_value_count++] = value;
  }
}


void add_mutable_value(const char *name, float *value) {
  if (mutable_value_count < 16) {
    muttable_values[mutable_value_count].name = name;
    muttable_values[mutable_value_count].value = value;
    mutable_value_count++;
  }
}

void var_command_handler(int argc, char *argv[]) {
  if (argc == 1) {
    // 列出所有可变变量
    for (size_t i = 0; i < mutable_value_count; i++) {
      printf("%s = %f\n", muttable_values[i].name, *(muttable_values[i].value));
    }
    return;
  }
  const char *var_name = argv[1];
  for (size_t i = 0; i < mutable_value_count; i++) {
    if (strcmp(var_name, muttable_values[i].name) == 0) {
      if (argc == 2) {
        // 查询变量值
        printf("%s = %f\n", var_name, *(muttable_values[i].value));
      } else if (argc == 3) {
        // 设置变量值
        float new_value = atof(argv[2]);
        *(muttable_values[i].value) = new_value;
        printf("%s set to %f\n", var_name, new_value);
      } else {
        printf("ERR\n");
      }
      return;
    }
  }
  printf("Unknown variable: %s\n", var_name);
}

static const unsigned short command_table_count = 2;
const CommandEntry command_table[16] = {
    {"set_speed", set_speed_handler},
    {"var", var_command_handler},
};

void handle_command(const uint8_t *data, uint16_t len) {
  if (!data || len == 0)
    return;

  /* 复制并裁剪长度 */
  if (len >= CMD_LINE_MAX)
    len = CMD_LINE_MAX - 1; /* 预留终止符 */
  char line[CMD_LINE_MAX];
  for (uint16_t i = 0; i < len; ++i)
    line[i] = (char)data[i];
  line[len] = '\0';

  /* 去掉末尾的 \r 和 \n */
  while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
    line[--len] = '\0';
  }
  if (len == 0)
    return; /* 空行 */

  /* 原地解析：用空白分割 */
  char *argv[CMD_MAX_ARGS];
  int argc = 0;
  char *p = line;
  while (*p && argc < CMD_MAX_ARGS) {
    /* 跳过前导空白 */
    while (*p && (*p == ' ' || *p == '\t'))
      ++p;
    if (!*p)
      break;
    argv[argc++] = p;
    while (*p && *p != ' ' && *p != '\t')
      ++p;
    if (*p) {
      *p++ = '\0'; /* 截断 token */
    }
  }
  if (argc == 0)
    return;

  for (size_t i = 0; i < command_table_count; ++i) {
    if (strcasecmp(argv[0], command_table[i].name) == 0) {
      command_table[i].handler(argc, argv);
      return;
    }
  }
  printf("Unknown command: %s\n", argv[0]);
}