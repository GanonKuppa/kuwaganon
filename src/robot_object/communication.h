/*
 * communication.h
 *
 *  Created on: 2017/08/13
 *      Author: ryota
 */

#pragma once

#include <stdint.h>

namespace umouse{
    int printfSync(const char* fmt, ...);
    int printfAsync(const char* fmt, ...);
    void fetchCommand();
    void sendPeriodicMsg();
    uint8_t* getPointerOfPeriodicMsg();
}
