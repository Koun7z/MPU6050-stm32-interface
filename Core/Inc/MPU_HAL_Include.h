//
// Created by pwoli on 17.04.2025.
//

#ifndef CRSF_HAL_INCLUDE_H
#define CRSF_HAL_INCLUDE_H

#if defined(STM32F3)
#  include "stm32f3xx_hal.h"
#elif defined(STM32F4)
#  include "stm32f4xx_hal.h"
#elif defined(STM32F7)
#  include "stm32f7xx_hal.h"
#elif defined(STM32L4)
#  include "stm32l4xx_hal.h"
#elif defined(STM32L5)
#  include "stm32l5xx_hal.h"
#elif defined(STM32H7)
#  include "stm32h7xx_hal.h"
#elif defined(STM32G4)
#  include "stm32g4xx_hal.h"
#else
#error "Define STM32F3, STM32F4, STM32F7, STM32L4, STM32L5, STM32H7 or STM32G4 symbol based on your MCU type"
#endif

#endif //CRSF_HAL_INCLUDE_H
