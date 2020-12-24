/* Copyright (c) 2020 Andrei Gramakov. All rights reserved.
 *
 * This file is licensed under the terms of the MIT license.
 * For a copy, see: https://opensource.org/licenses/MIT
 *
 * site:    https://agramakov.me
 * e-mail:  mail@agramakov.me
 */

#pragma once

#include "common.h"
#include "SharedVirtualRegisters.h"
#include "svr_types.h"

extern SharedVirtualRegisters_t regs;

inline esp_err_t regs_init()
{
    bool res = SVR_Init(&regs, REGS_AMOUNT);
    if (res) {
        return ESP_OK;
    }
    return ESP_FAIL;
}
