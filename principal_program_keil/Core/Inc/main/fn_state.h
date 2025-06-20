#pragma once

typedef enum FnState
{
    FNS_OK,
    FNS_FAIL,
    FNS_BUF_EMPTY,
    FNS_BUF_OVERFLOW,
    FNS_NO_MATCH,
} FnState;

#define FNS_ERROR_CHECK(expr)   \
    do {                        \
        FnState _err = (expr);  \
        if (_err != FNS_OK)     \
            return _err;        \
    } while (0)
