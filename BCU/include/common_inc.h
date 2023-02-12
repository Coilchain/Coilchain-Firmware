#ifndef COMMON_INC_H
#define COMMON_INC_H

// C++ scope
#include <Arduino.h>
#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <iterator>
#include <algorithm>
#include <ctime>

// C scope
#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
}
#endif