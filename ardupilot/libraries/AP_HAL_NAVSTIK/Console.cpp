#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_NAVSTIK
#include <stdarg.h>
#include "Console.h"
#include "UARTDriver.h"
#include <stdio.h>

using namespace Navstik;

NavstikConsoleDriver::NavstikConsoleDriver() {}

void NavstikConsoleDriver::init(void* uart)
{
	_uart = (NavstikUARTDriver *)uart;
}

void NavstikConsoleDriver::backend_open()
{}

void NavstikConsoleDriver::backend_close()
{}

size_t NavstikConsoleDriver::backend_read(uint8_t *data, size_t len) {
    return 0;
}

size_t NavstikConsoleDriver::backend_write(const uint8_t *data, size_t len) {
    return 0;
}

void NavstikConsoleDriver::print_P(const prog_char_t *pstr) {
	print(pstr);
}

void NavstikConsoleDriver::println_P(const prog_char_t *pstr) {
	println(pstr);
}

void NavstikConsoleDriver::printf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    _uart->vprintf(fmt, ap);
    va_end(ap);
}

void NavstikConsoleDriver::_printf_P(const prog_char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    _uart->vprintf(fmt, ap);
    va_end(ap);
}

void NavstikConsoleDriver::vprintf(const char *fmt, va_list ap) {
	_uart->vprintf(fmt, ap);
}

void NavstikConsoleDriver::vprintf_P(const prog_char *fmt, va_list ap) {
	_uart->vprintf(fmt, ap);
}

int16_t NavstikConsoleDriver::available() {
	return _uart->available();
}

int16_t NavstikConsoleDriver::txspace() {
	return _uart->txspace();
}

int16_t NavstikConsoleDriver::read() {
	return _uart->read();
}

size_t NavstikConsoleDriver::write(uint8_t c) {
	return _uart->write(c);
}

#endif
