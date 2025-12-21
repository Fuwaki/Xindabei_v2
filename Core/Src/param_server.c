#include "param_server.h"
#include <string.h>

#define MAX_PARAMS 64

static ParamDesc s_params[MAX_PARAMS];
static uint8_t   s_paramCount = 0;

bool ParamServer_Register(const ParamDesc *desc)
{
    if (desc == NULL || desc->name == NULL) {
        return false;
    }
    if (s_paramCount >= MAX_PARAMS) {
        return false;
    }

    /* 简单防止重名：如果已存在同名，则覆盖旧的 */
    for (uint8_t i = 0; i < s_paramCount; ++i) {
        if (strcmp(s_params[i].name, desc->name) == 0) {
            s_params[i] = *desc;
            return true;
        }
    }

    s_params[s_paramCount] = *desc;
    s_paramCount++;
    return true;
}

const ParamDesc *ParamServer_FindByName(const char *name)
{
    if (name == NULL) {
        return NULL;
    }

    for (uint8_t i = 0; i < s_paramCount; ++i) {
        if (strcmp(s_params[i].name, name) == 0) {
            return &s_params[i];
        }
    }
    return NULL;
}

int ParamServer_GetCount(void)
{
    return (int)s_paramCount;
}

const ParamDesc *ParamServer_GetByIndex(int idx)
{
    if (idx < 0 || idx >= (int)s_paramCount) {
        return NULL;
    }
    return &s_params[idx];
}

bool ParamServer_SetValueFloat(const ParamDesc *p, float v)
{
    if (p == NULL) {
        return false;
    }
    if (p->read_only) {
        return false;
    }
    if (p->type != PARAM_TYPE_FLOAT) {
        return false;
    }
    if (p->ops.f.set) {
        p->ops.f.set(v);
        return true;
    }
    return false;
}

bool ParamServer_SetValueInt(const ParamDesc *p, int v)
{
    if (p == NULL) {
        return false;
    }
    if (p->read_only) {
        return false;
    }
    if (p->type != PARAM_TYPE_INT) {
        return false;
    }
    if (p->ops.i.set) {
        p->ops.i.set(v);
        return true;
    }
    return false;
}

float ParamServer_GetValueFloat(const ParamDesc *p)
{
    if (p == NULL) {
        return 0.0f;
    }
    if (p->type != PARAM_TYPE_FLOAT) {
        return 0.0f;
    }
    if (p->ops.f.get) {
        return p->ops.f.get();
    }
    return 0.0f;
}

int ParamServer_GetValueInt(const ParamDesc *p)
{
    if (p == NULL) {
        return 0;
    }
    if (p->type != PARAM_TYPE_INT) {
        return 0;
    }
    if (p->ops.i.get) {
        return p->ops.i.get();
    }
    return 0;
}

const char* ParamServer_GetString(const ParamDesc *p)
{
    if (p == NULL) {
        return "?";
    }
    if (p->type == PARAM_TYPE_ENUM && p->ops.e.getString) {
        return p->ops.e.getString();
    }
    return "?";
}
