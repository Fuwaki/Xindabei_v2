#ifndef PARAM_SERVER_H
#define PARAM_SERVER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PARAM_TYPE_FLOAT = 0,
    PARAM_TYPE_INT   = 1,
} ParamType;

/* 回调函数定义 */
typedef float (*ParamGetFloatCb)(void);
typedef void  (*ParamSetFloatCb)(float val);
typedef int   (*ParamGetIntCb)(void);
typedef void  (*ParamSetIntCb)(int val);

typedef struct {
    const char *name;      /* 参数名字 */
    ParamType   type;      /* 类型 */
    
    /* 回调函数联合体 */
    union {
        struct {
            ParamGetFloatCb get;
            ParamSetFloatCb set;
        } f;
        struct {
            ParamGetIntCb get;
            ParamSetIntCb set;
        } i;
    } ops;

    float       step;      /* 调参步长 */
    uint8_t     read_only; /* 1 = 只读 */
    uint8_t     mask;      /* 参见 ParamMask 枚举 */
} ParamDesc;

/* 打印/显示掩码位 */
typedef enum {
    PARAM_MASK_SERIAL = 0x01,  /* 串口打印 */
    PARAM_MASK_OLED   = 0x02,  /* OLED 显示 */
} ParamMask;

/* 注册一个参数 */
bool ParamServer_Register(const ParamDesc *desc);

/* 通过名字查找参数 */
const ParamDesc *ParamServer_FindByName(const char *name);

/* 遍历用接口 */
int ParamServer_GetCount(void);
const ParamDesc *ParamServer_GetByIndex(int idx);

/* 读写接口（不做复杂检查，调用前确保类型匹配） */
bool  ParamServer_SetValueFloat(const ParamDesc *p, float v);
bool  ParamServer_SetValueInt(const ParamDesc *p, int v);
float ParamServer_GetValueFloat(const ParamDesc *p);
int   ParamServer_GetValueInt(const ParamDesc *p);

#ifdef __cplusplus
}
#endif

#endif /* PARAM_SERVER_H */
