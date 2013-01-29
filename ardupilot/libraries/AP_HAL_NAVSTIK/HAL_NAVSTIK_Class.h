
#ifndef __AP_HAL_NAVSTIK_CLASS_H__
#define __AP_HAL_NAVSTIK_CLASS_H__

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_NAVSTIK

#include <AP_HAL_NAVSTIK.h>
#include "AP_HAL_NAVSTIK_Namespace.h"
#include <systemlib/perf_counter.h>

class HAL_Navstik : public AP_HAL::HAL {
public:
    HAL_Navstik();
    void init(int argc, char * const argv[]) const;
};

extern const HAL_Navstik AP_HAL_Navstik;

#endif // CONFIG_HAL_BOARD == HAL_BOARD_NAVSTIK
#endif // __AP_HAL_NAVSTIK_CLASS_H__
