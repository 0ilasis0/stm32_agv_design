/*
#include "main/fn_state.h"
*/
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "main/config.h"

typedef int FnState;
extern FnState last_error;
#define FNS_INVALID         -1
#define FNS_OK              0
#define FNS_FAIL            1
#define FNS_TIMEOUT         2
#define FNS_BUF_EMPTY       3
#define FNS_BUF_OVERFLOW    4
#define FNS_NO_MATCH        5
#define FNS_NOT_MOVE        6
#define FNS_ERR_OOM         7

#define ERROR_CHECK_FNS_RETURN(expr)    \
    do {                                \
        FnState _err = (expr);          \
        if (_err != FNS_OK)             \
        {                               \
            last_error = _err;          \
            return _err;                \
        }                               \
    } while (0)

#define ERROR_CHECK_FNS_VOID(expr)  \
    do {                            \
        FnState _err = (expr);      \
        if (_err != FNS_OK)         \
        {                           \
            last_error = _err;      \
            return;                 \
        }                           \
    } while (0)

#define ERROR_CHECK_FNS_CLEAN(expr, cleanup)    \
    do {                                        \
        FnState _err = (expr);                  \
        if (_err != FNS_OK)                     \
        {                                       \
            cleanup;                            \
            last_error = _err;                  \
            return _err;                        \
        }                                       \
    } while (0)

#define ERROR_CHECK_FNS_HANDLE(expr)    \
    do {                                \
        FnState _err = (expr);          \
        if (_err != FNS_OK)             \
        {                               \
            last_error = _err;          \
            Error_Handler();            \
        }                               \
    } while (0)

#define ERROR_CHECK_HAL_HANDLE(expr)        \
    do {                                    \
        HAL_StatusTypeDef _err = (expr);    \
        if (_err != HAL_OK)                 \
        {                                   \
            Error_Handler();                \
        }                                   \
    } while (0)
