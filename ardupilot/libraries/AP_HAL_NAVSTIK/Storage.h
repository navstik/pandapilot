

#ifndef __AP_HAL_NAVSTIK_STORAGE_H__
#define __AP_HAL_NAVSTIK_STORAGE_H__

#include <AP_HAL.h>
#include "AP_HAL_NAVSTIK_Namespace.h"
#include <systemlib/perf_counter.h>

#define NAVSTIK_STORAGE_SIZE 4096
#define NAVSTIK_STORAGE_MAX_WRITE 512
#define NAVSTIK_STORAGE_LINE_SHIFT 9
#define NAVSTIK_STORAGE_LINE_SIZE (1<<NAVSTIK_STORAGE_LINE_SHIFT)
#define NAVSTIK_STORAGE_NUM_LINES (NAVSTIK_STORAGE_SIZE/NAVSTIK_STORAGE_LINE_SIZE)

class Navstik::NavstikStorage : public AP_HAL::Storage {
public:
    NavstikStorage() :
	_fd(-1),
	_dirty_mask(0),
	_perf_storage(perf_alloc(PC_ELAPSED, "APM_storage")),
	_perf_errors(perf_alloc(PC_COUNT, "APM_storage_errors"))
	{}
    void init(void* machtnichts) {}
    uint8_t  read_byte(uint16_t loc);
    uint16_t read_word(uint16_t loc);
    uint32_t read_dword(uint16_t loc);
    void     read_block(void *dst, uint16_t src, size_t n);

    void write_byte(uint16_t loc, uint8_t value);
    void write_word(uint16_t loc, uint16_t value);
    void write_dword(uint16_t loc, uint32_t value);
    void write_block(uint16_t dst, void* src, size_t n);

    void _timer_tick(void);

private:
    int _fd;
    void _storage_create(void);
    void _storage_open(void);
    void _mark_dirty(uint16_t loc, uint16_t length);
    uint8_t _buffer[NAVSTIK_STORAGE_SIZE];
    volatile uint32_t _dirty_mask;
    perf_counter_t  _perf_storage;
    perf_counter_t  _perf_errors;
};

#endif // __AP_HAL_NAVSTIK_STORAGE_H__
