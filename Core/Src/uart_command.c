#include "uart_command.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/_intsup.h>
#include "common.h"
#include "param_server.h"

static void set_speed_handler(int argc, char *argv[]) {
  if (argc < 2) {
    printf("ERR\n");
    return;
  }
  int value = atoi(argv[1]);
  printf("Speed set to %d\n", value);
}

void print_handler() {
  int count = ParamServer_GetCount();
  int first = 1;
  for (int i = 0; i < count; ++i) {
    const ParamDesc *p = ParamServer_GetByIndex(i);
    if (!p || !(p->mask & PARAM_MASK_SERIAL)) {
      continue;
    }

    if (!first) {
      printf(",");
    }
    first = 0;

    if (p->type == PARAM_TYPE_FLOAT) {
      float v = ParamServer_GetValueFloat(p);
      printf(FLOAT_FMT, FLOAT_TO_INT(v));
    } else {
      int v = ParamServer_GetValueInt(p);
      printf("%d", v);
    }
  }
  printf("\n");
}

void add_printable_value(const char *name, float *value) {
  (void)name;
  (void)value;
  /* 保留空实现，兼容旧接口；用户应直接用 ParamServer_Register 配置 printable 标志 */
}

void add_mutable_value(const char *name, float *value) {
  (void)name;
  (void)value;
  /* 保留空实现，兼容旧接口；用户应直接用 ParamServer_Register 配置 read_only=0 */
}

void var_command_handler(int argc, char *argv[]) {
  if (argc == 1) {
    int count = ParamServer_GetCount();
    for (int i = 0; i < count; ++i) {
      const ParamDesc *p = ParamServer_GetByIndex(i);
      if (!p) continue;

      if (p->type == PARAM_TYPE_FLOAT) {
        float v = ParamServer_GetValueFloat(p);
        printf("%s = " FLOAT_FMT "\n", p->name, FLOAT_TO_INT(v));
      } else {
        int v = ParamServer_GetValueInt(p);
        printf("%s = %d\n", p->name, v);
      }
    }
    return;
  }

  const char *var_name = argv[1];
  const ParamDesc *p = ParamServer_FindByName(var_name);
  if (!p) {
    printf("Unknown variable: %s\n", var_name);
    return;
  }

  if (argc == 2) {
    if (p->type == PARAM_TYPE_FLOAT) {
      float v = ParamServer_GetValueFloat(p);
      printf("%s = " FLOAT_FMT "\n", p->name, FLOAT_TO_INT(v));
    } else {
      int v = ParamServer_GetValueInt(p);
      printf("%s = %d\n", p->name, v);
    }
  } else if (argc == 3) {
    if (p->read_only) {
      printf("%s is read-only\n", p->name);
      return;
    }

    if (p->type == PARAM_TYPE_FLOAT) {
      float v = atof(argv[2]);
      if (ParamServer_SetValueFloat(p, v)) {
        printf("%s set to " FLOAT_FMT "\n", p->name, FLOAT_TO_INT(v));
      } else {
        printf("ERR\n");
      }
    } else {
      int v = atoi(argv[2]);
      if (ParamServer_SetValueInt(p, v)) {
        printf("%s set to %d\n", p->name, v);
      } else {
        printf("ERR\n");
      }
    }
  } else {
    printf("ERR\n");
  }
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