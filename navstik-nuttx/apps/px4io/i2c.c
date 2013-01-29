/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file i2c.c
 *
 * I2C communication for the PX4IO module.
 */

#include <stdint.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <stm32_i2c.h>
#include <stm32_dma.h>
#include <stdio.h>

#define DEBUG
#include "px4io.h"

/*
 * I2C register definitions.
 */
#define I2C_BASE	STM32_I2C1_BASE

#define REG(_reg)	(*(volatile uint32_t *)(I2C_BASE + _reg))

#define rCR1		REG(STM32_I2C_CR1_OFFSET)
#define rCR2		REG(STM32_I2C_CR2_OFFSET)
#define rOAR1		REG(STM32_I2C_OAR1_OFFSET)
#define rOAR2		REG(STM32_I2C_OAR2_OFFSET)
#define rDR		REG(STM32_I2C_DR_OFFSET)
#define rSR1		REG(STM32_I2C_SR1_OFFSET)
#define rSR2		REG(STM32_I2C_SR2_OFFSET)
#define rCCR		REG(STM32_I2C_CCR_OFFSET)
#define rTRISE		REG(STM32_I2C_TRISE_OFFSET)

static int		i2c_interrupt(int irq, void *context);
#ifdef DEBUG
void		i2c_dump(void);
#endif

static void		i2c_rx_setup(void);
static void		i2c_tx_setup(void);
static void		i2c_rx_complete(void);
static void		i2c_tx_complete(void);

static DMA_HANDLE	rx_dma;
static DMA_HANDLE	tx_dma;

static uint8_t		rx_buf[64];
static unsigned		rx_len;

static const uint8_t	junk_buf[] = { 0xff, 0xff, 0xff, 0xff };

static uint8_t		*tx_buf = junk_buf;
static unsigned		tx_len = sizeof(junk_buf);

static uint8_t		selected_page;
static uint8_t		selected_offset;

enum {
	DIR_NONE = 0,
	DIR_TX = 1,
	DIR_RX = 2
} direction;

void
i2c_init(void)
{
	debug("i2c init");

	/* allocate DMA handles and initialise DMA */
	rx_dma = stm32_dmachannel(DMACHAN_I2C1_RX);
	i2c_rx_setup();
	tx_dma = stm32_dmachannel(DMACHAN_I2C1_TX);
	i2c_tx_setup();

	/* enable the i2c block clock and reset it */
	modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_I2C1EN);
	modifyreg32(STM32_RCC_APB1RSTR, 0, RCC_APB1RSTR_I2C1RST);
	modifyreg32(STM32_RCC_APB1RSTR, RCC_APB1RSTR_I2C1RST, 0);

	/* configure the i2c GPIOs */
	stm32_configgpio(GPIO_I2C1_SCL);
	stm32_configgpio(GPIO_I2C1_SDA);

	/* soft-reset the block */
	rCR1 |= I2C_CR1_SWRST;
	rCR1 = 0;

	/* set for DMA operation */
	rCR2 |= I2C_CR2_ITEVFEN |I2C_CR2_ITERREN | I2C_CR2_DMAEN;

	/* set the frequency value in CR2 */
	rCR2 &= ~I2C_CR2_FREQ_MASK;
	rCR2 |= STM32_PCLK1_FREQUENCY / 1000000;

	/* set divisor and risetime for fast mode */
	uint16_t result = STM32_PCLK1_FREQUENCY / (400000 * 25);
	if (result < 1)
		result = 1;
	result = 3;
	rCCR &= ~I2C_CCR_CCR_MASK;
	rCCR |= I2C_CCR_DUTY | I2C_CCR_FS | result;
	rTRISE = (uint16_t)((((STM32_PCLK1_FREQUENCY / 1000000) * 300) / 1000) + 1);

	/* set our device address */
	rOAR1 = 0x1a << 1;

	/* enable event interrupts */
	irq_attach(STM32_IRQ_I2C1EV, i2c_interrupt);
	irq_attach(STM32_IRQ_I2C1ER, i2c_interrupt);
	up_enable_irq(STM32_IRQ_I2C1EV);
	up_enable_irq(STM32_IRQ_I2C1ER);

	/* and enable the I2C port */
	rCR1 |= I2C_CR1_ACK | I2C_CR1_PE;

#ifdef DEBUG
	i2c_dump();
#endif
}


/*
  reset the I2C bus
  used to recover from lockups
 */
void i2c_reset(void)
{
	rCR1 |= I2C_CR1_SWRST;
	rCR1 = 0;

	/* set for DMA operation */
	rCR2 |= I2C_CR2_ITEVFEN |I2C_CR2_ITERREN | I2C_CR2_DMAEN;

	/* set the frequency value in CR2 */
	rCR2 &= ~I2C_CR2_FREQ_MASK;
	rCR2 |= STM32_PCLK1_FREQUENCY / 1000000;

	/* set divisor and risetime for fast mode */
	uint16_t result = STM32_PCLK1_FREQUENCY / (400000 * 25);
	if (result < 1)
		result = 1;
	result = 3;
	rCCR &= ~I2C_CCR_CCR_MASK;
	rCCR |= I2C_CCR_DUTY | I2C_CCR_FS | result;
	rTRISE = (uint16_t)((((STM32_PCLK1_FREQUENCY / 1000000) * 300) / 1000) + 1);

	/* set our device address */
	rOAR1 = 0x1a << 1;

	/* and enable the I2C port */
	rCR1 |= I2C_CR1_ACK | I2C_CR1_PE;
}

/*
line=247
CR1   0x00000401  CR2   0x00000b18
OAR1  0x00000034  OAR2  0x00000000
CCR   0x0000c003  TRISE 0x00000008
SR1   0x00000044  SR2   0x00000002
 */

static int
i2c_interrupt(int irq, FAR void *context)
{
	uint16_t sr1 = rSR1;

	debug_line = __LINE__;
	isr_debug(3, "sr1=%u", (unsigned)sr1);
	debug_line = __LINE__;

	if (sr1 & I2C_SR1_ADDR) {

		/* clear ADDR to ack our selection and get direction */
		(void)rSR1;		/* as recommended, re-read SR1 */
		uint16_t sr2 = rSR2;

		debug_line = __LINE__;
		isr_debug(3, "sr2=%u", (unsigned)sr2);
		debug_line = __LINE__;

		if (sr2 & I2C_SR2_TRA) {
			/* we are the transmitter */
			isr_debug(3, "tx isr");

			if (direction == DIR_RX) {
				/*
				  we seem to have missed a receive
				  interrupt. This will be coming from
				  the junk_buf we sent at the end of
				  the transaction. We know we should
				  have had a receive interrupt, so
				  just call the rx complete routine
				  now. Perhaps the DMA complete
				  interrupt is coming in on a vector
				  we're not configured for?
				 */
				isr_debug(3, "missed rx interrupt");
				debug_line = __LINE__;
				i2c_rx_complete();
				debug_line = __LINE__;
			}
			direction = DIR_TX;
			debug_line = __LINE__;
			stm32_dmastart(tx_dma, NULL, NULL, false);
			debug_line = __LINE__;
		} else {
			/* we are the receiver */
			isr_debug(3, "recv isr");

			direction = DIR_RX;
			debug_line = __LINE__;
			stm32_dmastart(rx_dma, NULL, NULL, false);			
			debug_line = __LINE__;
		}
	}

	if (sr1 & (I2C_SR1_STOPF | I2C_SR1_AF)) {

		debug_line = __LINE__;

		if (sr1 & I2C_SR1_STOPF) {
			isr_debug(3, "STOPF direction=%d", (int)direction);
			/* write to CR1 to clear STOPF */
			(void)rSR1;		/* as recommended, re-read SR1 */
			rCR1 |= I2C_CR1_PE;
		} else {
			isr_debug(3, "AF direction=%d", (int)direction);			
		}

		/* it's likely that the DMA hasn't stopped, so we have to do it here */
		switch (direction) {
		case DIR_TX:
			debug_line = __LINE__;
			i2c_tx_complete();
			debug_line = __LINE__;
			break;
		case DIR_RX:
			debug_line = __LINE__;
			i2c_rx_complete();
			debug_line = __LINE__;
			break;
		default:
			/* spurious stop, ignore */
			isr_debug(1, "bad direction=%d", (int)direction);			
			break;
		}
		direction = DIR_NONE;
	}

	debug_line = __LINE__;
	/* clear any errors that might need it (this handles AF as well */
	if (sr1 & I2C_SR1_ERRORMASK)
		rSR1 = 0;
	debug_line = __LINE__;

	if (sr1 == (I2C_SR1_BTF | I2C_SR1_RXNE)) {
		/* 
		   this happens every 10-20 minutes. 
		 */
		i2c_reset();
		i2c_resets++;
	}

	return 0;
}

static void
i2c_rx_setup(void)
{
	/*
	 * Note that we configure DMA in circular mode; this means that a too-long
	 * transfer will overwrite the buffer, but that avoids us having to deal with
	 * bailing out of a transaction while the master is still babbling at us.
	 */
	rx_len = 0;
	debug_line = __LINE__;
	stm32_dmasetup(rx_dma, (uintptr_t)&rDR, (uintptr_t)&rx_buf[0], sizeof(rx_buf),
		DMA_CCR_CIRC |
		DMA_CCR_MINC |
		DMA_CCR_PSIZE_32BITS |
		DMA_CCR_MSIZE_8BITS |
		DMA_CCR_PRIMED);
	debug_line = __LINE__;
}

static void
i2c_rx_complete(void)
{
	rx_len = sizeof(rx_buf) - stm32_dmaresidual(rx_dma);
	stm32_dmastop(rx_dma);

	isr_debug(3, "rx_complete rx_len=%d", (int)rx_len);

	if (rx_len >= 2) {
		selected_page = rx_buf[0];
		selected_offset = rx_buf[1];

		isr_debug(2, "rx_complete page=%u offset=%u len=%u", 
			  (unsigned)selected_page,
			  (unsigned)selected_offset,
			  (unsigned)rx_len);

		/* work out how many registers are being written */
		unsigned count = (rx_len - 2) / 2;
		if (count > 0) {
			registers_set(selected_page, selected_offset, (const uint16_t *)&rx_buf[2], count);
		} else {
			/* no registers written, must be an address cycle */
			uint16_t *regs;
			unsigned reg_count;

			/* work out which registers are being addressed */
			debug_line = __LINE__;
			int ret = registers_get(selected_page, selected_offset, &regs, &reg_count);
			debug_line = __LINE__;
			isr_debug(3, "registers_ret->%d", ret);
			if (ret == 0) {
				tx_buf = (uint8_t *)regs;
				tx_len = reg_count *2;
			} else {
				tx_buf = junk_buf;
				tx_len = sizeof(junk_buf);
			}

			/* disable interrupts while reconfiguring DMA for the selected registers */
			irqstate_t flags = irqsave();

			debug_line = __LINE__;
			stm32_dmastop(tx_dma);
			debug_line = __LINE__;
			i2c_tx_setup();
			debug_line = __LINE__;

			irqrestore(flags);
			debug_line = __LINE__;
		}
	}

	/* prepare for the next transaction */
	debug_line = __LINE__;
	i2c_rx_setup();
	debug_line = __LINE__;
}

static void
i2c_tx_setup(void)
{
	/*
	 * Note that we configure DMA in circular mode; this means that a too-long
	 * transfer will copy the buffer more than once, but that avoids us having
	 * to deal with bailing out of a transaction while the master is still 
	 * babbling at us.
	 */
	stm32_dmasetup(tx_dma, (uintptr_t)&rDR, (uintptr_t)&tx_buf[0], tx_len,
		DMA_CCR_DIR |
		DMA_CCR_CIRC |
		DMA_CCR_MINC |
		DMA_CCR_PSIZE_8BITS |
		DMA_CCR_MSIZE_8BITS |
		DMA_CCR_PRIMED);
}

static void
i2c_tx_complete(void)
{
	debug_line = __LINE__;
	stm32_dmastop(tx_dma);
	debug_line = __LINE__;

	tx_buf = junk_buf;
	tx_len = sizeof(junk_buf);

	isr_debug(3, "tx_complete");

	/* prepare for the next transaction */
	debug_line = __LINE__;
	i2c_tx_setup();
	debug_line = __LINE__;
}

#ifdef DEBUG
void i2c_dump(void)
{
	debug("CR1   0x%08x  CR2   0x%08x", rCR1,  rCR2);
	debug("OAR1  0x%08x  OAR2  0x%08x", rOAR1, rOAR2);
	debug("CCR   0x%08x  TRISE 0x%08x", rCCR,  rTRISE);
	debug("SR1   0x%08x  SR2   0x%08x", rSR1,  rSR2);
}

#endif
