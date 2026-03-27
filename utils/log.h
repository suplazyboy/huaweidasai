/*
* Copyright (c) Huawei Technologies Co., Ltd. 2026-2026. All rights reserved.
* Description: 2026 code craft log
* Author: anzhiwu a00494813
* Create: 2026/3/4
 */

#ifndef INC_2026_CODE_CRAFT_LOG_H
#define INC_2026_CODE_CRAFT_LOG_H

#include <cstdio>

int GetFrameID();

enum emLogLevel {
    LOG_LEVEL_NO_LOG = 0,
    LOG_LEVEL_ASSERT,
    LOG_LEVEL_ERR,
    LOG_LEVEL_WARN,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DBG,
};

extern int g_logLevel;

#ifdef _MSC_VER
#define LOG(level, prefix, fmt, ...) 	do{ if (g_logLevel >= level) fprintf(stderr, "[" prefix "]" __FILE__ ":%d|%s|frame[%d]|" fmt "\n", __LINE__, __FUNCTION__, GetFrameID(), ##__VA_ARGS__);}while(0)
#define LOG_DBG(fmt, ...) LOG(LOG_LEVEL_DBG, "DBG", fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...) LOG(LOG_LEVEL_INFO, "INFO", fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...) LOG(LOG_LEVEL_WARN, "WARN", fmt, ##__VA_ARGS__)
#define LOG_ERR(fmt, ...) LOG(LOG_LEVEL_ERR, "ERR", fmt, ##__VA_ARGS__)
#define LOG_ASSERT(fmt, ...) LOG(LOG_LEVEL_ASSERT, "ASSERT", fmt, ##__VA_ARGS__)
#define CHECK_RET(expr, ...) do {if (!(expr)) {LOG_ERR("check \"%s\" failed", #expr); return __VA_ARGS__;} } while (0)
#define ASSERT(expr) do {if (!(expr)) {LOG_ASSERT("assert \"%s\" failed", #expr);} } while (0)
#define ASSERT_RET(expr, ...) do {if (!(expr)) {LOG_ASSERT("assert \"%s\" failed", #expr); return __VA_ARGS__;} } while (0)
#else
#define LOG(level, prefix, fmt, args...) 	do{ if (g_logLevel >= level) fprintf(stderr, "[" prefix "]" __FILE__ ":%d|%s|" fmt "\n", __LINE__, __FUNCTION__, ##args);}while(0)
#define LOG_DBG(fmt, args...) LOG(LOG_LEVEL_DBG, "DBG", fmt, ##args)
#define LOG_INFO(fmt, args...) LOG(LOG_LEVEL_INFO, "INFO", fmt, ##args)
#define LOG_WARN(fmt, args...) LOG(LOG_LEVEL_WARN, "WARN", fmt, ##args)
#define LOG_ERR(fmt, args...) LOG(LOG_LEVEL_ERR, "ERR", fmt, ##args)
#define LOG_ASSERT(fmt, args...) LOG(LOG_LEVEL_ASSERT, "ASSERT", fmt, ##args)
#define CHECK_RET(expr, args...) do {if (!(expr)) {LOG_ERR("check \"%s\" failed", #expr); return args;} } while (0)
#define ASSERT(expr) do {if (!(expr)) {LOG_ASSERT("assert \"%s\" failed", #expr);} } while (0)
#define ASSERT_RET(expr, args...) do {if (!(expr)) {LOG_ASSERT("assert \"%s\" failed", #expr); return args;} } while (0)

#endif

#endif  // INC_2026_CODE_CRAFT_LOG_H
