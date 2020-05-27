/*
 * Copyright (c) 2013-2014, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <arch.h>
#include <arch_helpers.h>
#include <pll.h>
#include <stdio.h>
#include <saradc.h>
#include <secure_apb.h>
#include "meson_i2c.h"
#include <timer.h>

/*******************************************************************************
 * Place holder function to perform any S-EL1 specific architectural setup. At
 * the moment there is nothing to do.
 ******************************************************************************/
extern void power_init(int mode);

/* GPIOM_12/GPIOM_13 i2c2 i2cc*/
void set_i2c2_pinmux(void)
{
       unsigned int val;

       /* set pinmux */
       val = readl(PADCTRL_PIN_MUX_REGA);
       val &= ~(0xff < 16);
       val |= (0x1 << 16) | (0x1 << 20);
       writel(val, PADCTRL_PIN_MUX_REGA);
       /* set ds */
       val = readl(PADCTRL_GPIOM_DS);
       val &= ~(0xf << 24);
       val |= 0xf << 24;
       writel(val, PADCTRL_GPIOM_DS);
       /* disable pull up */
       val = readl(PADCTRL_GPIOM_PULL_EN);
       val &= ~(0x3 << 12);
       writel(val, PADCTRL_GPIOM_PULL_EN);

       serial_puts("i2c: pinmux regA = 0x");
       serial_put_hex(readl(PADCTRL_PIN_MUX_REGA), 32);
       serial_puts("\n");

       serial_puts("i2c: ds = 0x");
       serial_put_hex(readl(PADCTRL_GPIOM_DS), 32);
       serial_puts("\n");

       serial_puts("i2c: pullup en(disbale pull up) = 0x");
       serial_put_hex(readl(PADCTRL_GPIOM_PULL_EN), 32);
       serial_puts("\n");
}

void fifo_write_8_pages_once_new()
{
        uint32_t i;
        uint8_t read_tab[64];

        uint8_t buf[64 + 7] = {1, 2, 3, 4, 5, 6, 7, 8,
                        0x8, 10, 20, 30, 40, 50, 60, 70, 80,
                        0x10,12, 23, 34, 45, 56, 67, 78, 89,
                        0x18, 31, 32, 33, 34, 35, 36, 37, 38,
                        0x20, 41, 42, 43, 44, 45, 46, 47, 48,
                        0x28, 11, 22, 33, 44, 55, 66, 77, 88,
                        0x30, 100, 110, 120, 130, 140, 150, 160, 170, 
                        0x38, 80, 70, 60, 50, 40, 30, 20, 10
        };   

        /* write page 0 - 7 */
        meson_i2c_fifo_write_interval(9);
        meson_i2c_fifo_write(0x50, 0, buf, 64 + 7);     
        printf("meson i2c fifo write 8 pages over\n");
     
        /*single read 64 bytes*/ 
        for (i = 0; i < 64; i++) {
                 meson_i2c_read(0x50, i, &read_tab[i], 1);
                 printf("Single-Read %d th, val = %d\n", i, read_tab[i]);
         }
}

void one_page_write(uint8_t offset, uint8_t *buf, uint8_t size)
{
        meson_i2c_write(0x50, offset, buf, size);
	_udelay(5 * 1000);
}

void one_page_read(uint8_t page_offset)
{
	int32_t i;
        uint8_t read_tab[8];

        meson_i2c_read(0x50, page_offset, read_tab, 8);
        for (i = 0; i < 8; i++)
                printf("sequence read: read the 8 page write: the %d th val = %d\n", i, read_tab[i]);           
}

/* page write */
/*
 * page0: 0x0 - 0x7
 * page1: 0x8 - 0xf
 * page2: 0x10 - 0x17
 * page3: 0x18 - 0x1f
 * page4: 0x20 - 0x27
 * page5: 0x28 - 0x2f
 * page6: 0x30 - 0x37
 * page7: 0x38 - 0x3f
 */
void write_8_page(void)
{
        uint32_t i;
        uint8_t read_tab[64];

        uint8_t buf0[] = {1, 2, 3, 4, 5, 6, 7, 8};
        uint8_t buf1[] = {10, 20, 30, 40, 50, 60, 70, 80};
        uint8_t buf2[] = {12, 23, 34, 45, 56, 67, 78, 89};
        uint8_t buf3[] = {31, 32, 33, 34, 35, 36, 37, 38};

        uint8_t buf4[] = {41, 42, 43, 44, 45, 46, 47, 48};
        uint8_t buf5[] = {11, 22, 33, 44, 55, 66, 77, 88};
        uint8_t buf6[] = {100, 110, 120, 130, 140, 150, 160, 170};
        uint8_t buf7[] = {80, 70, 60, 50, 40, 30, 20, 10};

        /* page 0 */
	printf("writing page 0\n"); 
        one_page_write(0, buf0, sizeof(buf0));
        /* page 1 */
	printf("writing page 1\n"); 
        one_page_write(0x8, buf1, sizeof(buf1));
        /* page 2 */    
	printf("writing page 2\n"); 
        one_page_write(0x10, buf2, sizeof(buf2));
        /* page 3 */    
	printf("writing page 3\n"); 
        one_page_write(0x18, buf3, sizeof(buf3));               
        /* page 4 */
	printf("writing page 4\n"); 
        one_page_write(0x20, buf4, sizeof(buf4));
        /* page 5 */    
	printf("writing page 5\n"); 
        one_page_write(0x28, buf5, sizeof(buf5));
        /* page 6*/     
	printf("writing page 6\n"); 
        one_page_write(0x30, buf6, sizeof(buf6));
        /* page 7*/
	printf("writing page 7\n"); 
        one_page_write(0x38, buf7, sizeof(buf7));

        meson_i2c_read(0x50, 0, read_tab, sizeof(buf0) * 8);
        /* print the read data */
        for (i = 0; i < sizeof(buf0) * 8; i++)
                printf("sequence read: read the 8 page write: the %d th val = %d\n", i, read_tab[i]);           
}

void write_1357_page(void)
{
        uint8_t buf1[] = {10, 20, 30, 40, 50, 60, 70, 80};
        uint8_t buf3[] = {31, 32, 33, 34, 35, 36, 37, 38};

        uint8_t buf5[] = {11, 22, 33, 44, 55, 66, 77, 88};
        uint8_t buf7[] = {80, 70, 60, 50, 40, 30, 20, 10};

        /* page 1 */
	printf("writing page 1\n"); 
        one_page_write(0x8, buf1, sizeof(buf1));
	one_page_read(0x8);
        /* page 3 */    
	printf("writing page 3\n"); 
        one_page_write(0x18, buf3, sizeof(buf3));               
	one_page_read(0x18);
        /* page 5 */    
	printf("writing page 5\n"); 
        one_page_write(0x28, buf5, sizeof(buf5));
	one_page_read(0x28);
        /* page 7*/
	printf("writing page 7\n"); 
        one_page_write(0x38, buf7, sizeof(buf7));
	one_page_read(0x38);
}

void i2c_test(void)
{
	set_i2c2_pinmux();
	meson_i2c_port_init(I2C_M2);

	uint8_t val1 = 0x58;
	uint8_t val2 = 0x67;
	uint8_t val3 = 0x12;
        uint8_t r_val;

	meson_i2c_write(0x50, 0x2, &val1, 1);
        serial_puts("write 0x58 to 0x2 register done\n");
        meson_i2c_read(0x50, 0x2, &r_val, 1);
        serial_puts("read one byte done--------------0x2 = ");
        serial_put_hex(r_val, 16); 
        serial_puts("\n");

	meson_i2c_write(0x50, 0x2, &val1, 1);
        serial_puts("write 0x58 to 0x2 register done\n");
        meson_i2c_read(0x50, 0x2, &r_val, 1);
        serial_puts("read one byte done--------------3 = ");
        serial_put_hex(r_val, 16); 
        serial_puts("\n");

	meson_i2c_write(0x50, 0x8, &val3, 1);
	printf("write 0x12 to 0x8 done\n");
        meson_i2c_read(0x50, 0x8, &r_val, 1);
	printf("read 0x8 value = 0x%x\n", r_val);

	write_8_page();
}

void bl2_arch_setup(void)
{
	/* Give access to FP/SIMD registers */
	write_cpacr(CPACR_EL1_FPEN(CPACR_EL1_FP_TRAP_NONE));

	/* get board id */
	saradc_ch1_get();

	/* init power for each domain */
	//power_init(0);

	
	/* Init plls */
	pll_init();
        serial_puts("PLL init done\n");
	
	/*init c2 i2c*/
	i2c_test();
}
