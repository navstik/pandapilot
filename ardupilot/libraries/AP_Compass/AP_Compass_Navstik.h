/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef AP_Compass_NAVSTIK_H
#define AP_Compass_NAVSTIK_H

#include "Compass.h"

class AP_Compass_Navstik : public Compass
{
public:
    AP_Compass_Navstik() : Compass() {
        product_id = AP_COMPASS_TYPE_NAVSTIK;
    }
    bool        init(void);
    bool        read(void);
    void        accumulate(void);

private:
    int _mag_fd;
    Vector3f _sum;
    uint32_t _count;
    uint64_t _last_timestamp;
};

#endif // AP_Compass_NAVSTIK_H

