/*
 * Copyright (C)2018 Amlogic, Inc. All rights reserved.
 *
 * All information contained herein is Amlogic confidential.
 *
 * This software is provided to you pursuant to Software License Agreement
 * (SLA) with Amlogic Inc ("Amlogic"). This software may be used
 * only in accordance with the terms of this agreement.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification is strictly prohibited without prior written permission from
 * Amlogic.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * Author:  Jian Hu <jian.hu@amlogic.com>
 */
#include <secure_apb.h>
#include <stdio.h>
#include <serial.h>
#include <meson_i2c.h>
#include <io.h>
#include <timer.h>
#include <string.h>

#define MESON_I2C_CLK_RATE  		167000000
#define BIT(nr)         		(1UL << (nr))
#define GENMASK(h, l) \
(((~0UL) << (l)) & (~0UL >> (32 - 1 - (h))))

#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#define I2C_TIMEOUT_MS			1000*100
#define MAX_TOKEN			16
#define MAX_WDDATA			8

/* Control register fields */
#define REG_CTRL_START			BIT(0)
#define REG_CTRL_ACK_IGNORE		BIT(1)
#define REG_CTRL_STATUS			BIT(2)
#define REG_CTRL_ERROR			BIT(3)
#define REG_CTRL_CLKDIV_SHIFT		12
#define REG_CTRL_CLKDIV_MASK		GENMASK(21, 12)
#define REG_CTRL_CLKDIVEXT_SHIFT 	28
#define REG_CTRL_CLKDIVEXT_MASK		GENMASK(29, 28)

/* C2 FIFO MODE */
#define REG_FIFO_MODE			BIT(30)
#define REG_FIFO_TIMEOUT_STOP		BIT(24)
#define REG_FIFO_TIMEOUT_CLR		BIT(23)
#define REG_FIFO_TIMEOUT_EN		BIT(22)
#define REG_FIFO_IRQ_MODE		BIT(21) /*0 pulse 1 level */
#define REG_FIFO_ERR_STOP		BIT(20)
#define REG_FIFO_RD_RST			BIT(18)
#define REG_FIFO_WD_RST			BIT(17)
#define REG_FIFO_TK_RST			BIT(16)
#define REG_FIFO_TO_THD_MASK		GENMASK(16, 31)
#define REG_FIFO_RD_THD_MASK		GENMASK(8, 11)
#define REG_FIFO_WD_THD_MASK		GENMASK(4, 7)
#define REG_FIFO_TK_THD_MASK		GENMASK(0, 2)

#define I2C_DEBUG			0
#define I2C_FIFO_WRITE			0

uint32_t wfifo[110];
uint32_t tfifo[110];

typedef enum { false, true }bool;

enum {
	TOKEN_END = 0,
	TOKEN_START,
	TOKEN_SLAVE_ADDR_WRITE,
	TOKEN_SLAVE_ADDR_READ,
	TOKEN_DATA,
	TOKEN_DATA_LAST,
	TOKEN_STOP,
};

struct i2cregs {
	volatile uint32_t ctrl;
	volatile uint32_t slave_addr;
	volatile uint32_t tok_list0;
	volatile uint32_t tok_list1;
	volatile uint32_t tok_wdata0;
	volatile uint32_t tok_wdata1;
	volatile uint32_t tok_rdata0;
	volatile uint32_t tok_rdata1;
	volatile uint32_t timeout_th; /* for irq working mode timeout */
	/* C2 timing */
	volatile uint32_t cntl_dely1;
	volatile uint32_t cntl_dely2;
	volatile uint32_t low_dely;
	volatile uint32_t high_dely;
	/* C2 fifo */
	volatile uint32_t fifo_ctrl0;
	volatile uint32_t fifo_ctrl1;
	volatile uint32_t fifo_pending;
	volatile uint32_t fifo_pending_mask;
	volatile uint32_t fifo_st0;
};

struct meson_i2c {
	struct	i2cregs *regs;
	struct	i2c_msg *msg;	/* Current I2C message */
	bool	last;		/* Whether the message is the last */
	uint32_t count;		/* Number of bytes in the current transfer */
	uint32_t pos;		/* Position of current transfer in message */
	uint32_t tokens[2];		/* Sequence of tokens to be written */
	uint32_t num_tokens;	/* Number of tokens to be written */
	uint32_t clock_frequency;
	uint32_t div_factor;
	uint32_t delay_ajust;
	uint32_t clkin_rate;
	uint32_t time_count;
	uint8_t mode;
	uint8_t fifo_count;
	uint8_t wf_pos;
	uint8_t interval;
};

struct meson_i2c i2cs[5];

uint32_t current_id;

/* C2 i2c data */
struct meson_i2c_platdata c2_i2c_data[] = {
		{0, 0xfe001400, 3, 15, 100000, MESON_I2C_CLK_RATE, FIFO_POLLING}, /* i2c A */
		{1, 0xfe005c00, 3, 15, 100000, MESON_I2C_CLK_RATE, FIFO_POLLING}, /* i2c B */
		{2, 0xfe006800, 3, 15, 400000, MESON_I2C_CLK_RATE, FIFO_POLLING}, /* i2c C */
		{3, 0xfe006c00, 3, 15, 100000, MESON_I2C_CLK_RATE, FIFO_POLLING}, /* i2c D */
		{4, 0xfe00b000, 3, 15, 100000, MESON_I2C_CLK_RATE, FIFO_POLLING}, /* i2c E */
};

void meson_i2c_dump_regs(void)
{
	uint32_t i;

	for (i = 0; i < 110;i++)
		printf("wf[%d] = 0x%08x\n", i, wfifo[i]);

	for (i = 0; i < 110;i++)
		printf("tf[%d] = 0x%08x\n", i, tfifo[i]);

	printf("i2c reg : 0x%x = 0x%x\n", &i2cs[current_id].regs->ctrl, i2cs[current_id].regs->ctrl);
	printf("i2c reg : 0x%x = 0x%x\n", &i2cs[current_id].regs->slave_addr, i2cs[current_id].regs->slave_addr);
	printf("i2c reg : 0x%x = 0x%x\n", &i2cs[current_id].regs->tok_list0, i2cs[current_id].regs->tok_list0);
	printf("i2c reg : 0x%x = 0x%x\n", &i2cs[current_id].regs->tok_list1, i2cs[current_id].regs->tok_list1);
	printf("i2c reg : 0x%x = 0x%x\n", &i2cs[current_id].regs->tok_wdata0, i2cs[current_id].regs->tok_wdata0);
	printf("i2c reg : 0x%x = 0x%x\n", &i2cs[current_id].regs->tok_wdata1, i2cs[current_id].regs->tok_wdata1);
	printf("i2c reg : 0x%x = 0x%x\n", &i2cs[current_id].regs->tok_rdata0, i2cs[current_id].regs->tok_rdata0);
	printf("i2c reg : 0x%x = 0x%x\n", &i2cs[current_id].regs->tok_rdata1, i2cs[current_id].regs->tok_rdata1);
	printf("i2c reg : 0x%x = 0x%x\n", &i2cs[current_id].regs->fifo_ctrl0, i2cs[current_id].regs->fifo_ctrl0);
	printf("i2c reg : 0x%x = 0x%x\n", &i2cs[current_id].regs->fifo_ctrl1, i2cs[current_id].regs->fifo_ctrl1);
	printf("i2c reg : 0x%x = 0x%x\n", &i2cs[current_id].regs->fifo_pending, i2cs[current_id].regs->fifo_pending);
	printf("i2c reg : 0x%x = 0x%x\n", &i2cs[current_id].regs->fifo_pending_mask, i2cs[current_id].regs->fifo_pending_mask);
	printf("i2c reg : 0x%x = 0x%x\n", &i2cs[current_id].regs->fifo_st0, i2cs[current_id].regs->fifo_st0);
}

static void meson_i2c_regs_init(void)
{
	uint32_t i;

	/* reset ctrl and slave registers */
	for (i = 0; i < 2; i++)
		writel(0, &i2cs[current_id].regs->ctrl + i);

	/* reset timing and fifo registers */
	for (i = 9 ;i < 18 ;i ++)
		writel(0, &i2cs[current_id].regs->ctrl + i);
}

static void meson_i2c_reset_tokens(void)
{
	i2cs[current_id].tokens[0] = 0;
	i2cs[current_id].tokens[1] = 0;
	i2cs[current_id].num_tokens = 0;
}

static void meson_i2c_add_token(uint32_t token)
{
	if (i2cs[current_id].num_tokens < 8)
		i2cs[current_id].tokens[0] |= (token & 0xf) << (i2cs[current_id].num_tokens * 4);
	else
		i2cs[current_id].tokens[1] |= (token & 0xf) << ((i2cs[current_id].num_tokens % 8) * 4);

	i2cs[current_id].num_tokens++;
	if (i2cs[current_id].num_tokens == 16)
		i2cs[current_id].num_tokens = 0;
}

/*
 * Retrieve data for the current transfer (which can be at most 8
 * bytes) from the device internal buffer.
 */
static void meson_i2c_get_data(uint8_t *buf, uint32_t len)
{
	uint32_t rdata0, rdata1;
	uint32_t i;

	rdata0 = i2cs[current_id].regs->tok_rdata0;
	rdata1 = i2cs[current_id].regs->tok_rdata1;

#if I2C_DEBUG
	printf("meson i2c: read data0 0x%x, data1 0x%x, len %d\n",
		rdata0, rdata1, len);
#endif

	for (i = 0; i < MIN((uint32_t)4, len); i++)
		*buf++ = (rdata0 >> i * 8) & 0xff;

	for (i = 4; i < MIN((uint32_t)8, len); i++)
		*buf++ = (rdata1 >> (i - 4) * 8) & 0xff;
}

static void meson_i2c_fifo_polling_get_data(uint8_t *buf, uint32_t len)
{
	uint32_t rdata0, rdata1;
	uint32_t i, j, tmp_floor, left_count;
	uint32_t r_fifo_count;

	tmp_floor = len/8;
	left_count = len%8;
	r_fifo_count = (i2cs[current_id].regs->fifo_st0) & 0xf;

	if (r_fifo_count < 4) {
		for (j = 0; j < MIN(tmp_floor,(uint32_t)4); j++) {
				rdata0 = i2cs[current_id].regs->tok_rdata0;
				rdata1 = i2cs[current_id].regs->tok_rdata1;
#if I2C_DEBUG
				printf("meson i2c fifo polling: read data0 0x%x, data1 0x%x, len %d, r_fifo_cout=%d\n",
					rdata0, rdata1, len, r_fifo_count);
#endif
				for (i = 0; i < 4; i++)
					*buf++ = (rdata0 >> i * 8) & 0xff;

				for (i = 4; i < 8; i++)
					*buf++ = (rdata1 >> (i - 4) * 8) & 0xff;
		}
	}
	/* deal the left_count = 1, 2, 3, 4, 5, 6, 7*/
	if (r_fifo_count < 4) {
		if (tmp_floor < 4) {
			rdata0 = i2cs[current_id].regs->tok_rdata0;
			rdata1 = i2cs[current_id].regs->tok_rdata1;
#if I2C_DEBUG
			printf("meson i2c fifo polling left data: read data0 0x%x, data1 0x%x, len %d r_fifo_count = %d\n",
				rdata0, rdata1, len, r_fifo_count);
#endif
			for (i = 0; i < MIN((uint32_t)4, left_count); i++)
				*buf++ = (rdata0 >> i * 8) & 0xff;

			for (i = 4; i < MIN((uint32_t)8, len); i++)
				*buf++ = (rdata1 >> (i - 4) * 8) & 0xff;
		}
	}
}

uint32_t wn = 0;
/*
 * Write data for the current transfer (which can be at most 8 bytes)
 * to the device internal buffer.
 */
static void meson_i2c_put_data(uint8_t *buf, uint32_t len)
{
	uint32_t wdata0 = 0, wdata1 = 0;
	uint32_t i;

	for (i = 0; i < MIN((uint32_t)4, len); i++)
		wdata0 |= *buf++ << (i * 8);

	for (i = 4; i < MIN((uint32_t)8, len); i++)
		wdata1 |= *buf++ << ((i - 4) * 8);

	i2cs[current_id].regs->tok_wdata0 = wdata0;
	i2cs[current_id].regs->tok_wdata1 = wdata1;

	wfifo[wn] = i2cs[current_id].regs->tok_wdata0;
	wn++;
	wfifo[wn] = i2cs[current_id].regs->tok_wdata1;
	wn++;
#if I2C_FIFO_WRITE
	printf("meson i2c: write data wdata0: 0x%x, wdata1: 0x%x, len: %d. Wf cnt = %d\n",
		wdata0, wdata1, len, readl(&i2cs[current_id].regs->fifo_st0) >> 4 & 0xf);
#endif
}

/*
 * Prepare the next transfer: pick the next 8 bytes in the remaining
 * part of message and write tokens and data (if needed) to the
 * device.
 */
static void meson_i2c_prepare_xfer(void)
{
	bool write = !(i2cs[current_id].msg->flags & I2C_M_RD);
	uint32_t i;

	i2cs[current_id].count = MIN(i2cs[current_id].msg->len - i2cs[current_id].pos, 8u);

	for (i = 0; i + 1 < i2cs[current_id].count; i++)
		meson_i2c_add_token(TOKEN_DATA);

	if (i2cs[current_id].count) {
		if (write || i2cs[current_id].pos + i2cs[current_id].count < i2cs[current_id].msg->len)
			meson_i2c_add_token(TOKEN_DATA);
		else
			meson_i2c_add_token(TOKEN_DATA_LAST);
	}

	if (write)
		meson_i2c_put_data(i2cs[current_id].msg->buf + i2cs[current_id].pos, i2cs[current_id].count);

	if (i2cs[current_id].last && i2cs[current_id].pos + i2cs[current_id].count >= i2cs[current_id].msg->len)
		meson_i2c_add_token(TOKEN_STOP);

	i2cs[current_id].regs->tok_list0 = i2cs[current_id].tokens[0];
	i2cs[current_id].regs->tok_list1 = i2cs[current_id].tokens[1];
}

static void meson_i2c_do_start(struct i2c_msg *msg)
{
	uint32_t token;

	token = (msg->flags & I2C_M_RD) ? TOKEN_SLAVE_ADDR_READ :
		TOKEN_SLAVE_ADDR_WRITE;

	/* change it if duty change */
	i2cs[current_id].regs->slave_addr = msg->addr << 1;
	meson_i2c_add_token(TOKEN_START);
	meson_i2c_add_token(token);
}

static int32_t meson_i2c_xfer_msg(struct i2c_msg *msg, uint32_t last)
{
	uint32_t time_count = 0;
	volatile uint32_t *ctrl = &i2cs[current_id].regs->ctrl;

#if I2C_DEBUG
	serial_puts("meson i2c:");
	if (msg->flags & I2C_M_RD)
		serial_puts("read ");
	else
		serial_puts("write ");
	serial_puts("addr 0x");
	serial_put_hex(msg->addr, 32);
	serial_puts(" len 0x");
	serial_put_hex(msg->len, 32);
	serial_puts("\n");
#endif
	i2cs[current_id].msg = msg;
	i2cs[current_id].last = last;
	i2cs[current_id].pos = 0;
	i2cs[current_id].count = 0;

	meson_i2c_reset_tokens();
	meson_i2c_do_start(msg);

	do {
		meson_i2c_prepare_xfer();
		/* start the transfer */
		setbits_le32((&i2cs[current_id].regs->ctrl), REG_CTRL_START);
		while (readl(ctrl) & REG_CTRL_STATUS) {
			if (time_count > I2C_TIMEOUT_MS) {
				clrbits_le32(&i2cs[current_id].regs->ctrl, REG_CTRL_START);
				serial_puts("meson i2c time out\n");
				return -1;
			}
			_udelay(1);
			time_count++;
		}
		meson_i2c_reset_tokens();
		clrbits_le32(&i2cs[current_id].regs->ctrl, REG_CTRL_START);

		if (readl(ctrl) & REG_CTRL_ERROR) {
			serial_puts("meson i2c: error ctrl reg = 0x");
			serial_put_hex(readl(ctrl), 32);
			serial_puts("\n");
			return -1;
		}

		if ((msg->flags & I2C_M_RD) && i2cs[current_id].count) {
			meson_i2c_get_data(i2cs[current_id].msg->buf + i2cs[current_id].pos,
					   i2cs[current_id].count);
		}
		i2cs[current_id].pos += i2cs[current_id].count;
	} while (i2cs[current_id].pos < msg->len);

	return 0;
}

uint32_t  tn = 0;
static void meson_i2c_fill_token_list(void)
{
	i2cs[current_id].regs->tok_list0 = i2cs[current_id].tokens[0];
	i2cs[current_id].regs->tok_list1 = i2cs[current_id].tokens[1];

	tfifo[tn] = i2cs[current_id].regs->tok_list0;
	tn++;
	tfifo[tn] = i2cs[current_id].regs->tok_list1;
	tn++;
#if I2C_FIFO_WRITE
	printf("If it is right, Disable I2C_FIFO_WRITE. list0 = 0x%x, list1 = 0x%x, tf_cnt =%d\n", i2cs[current_id].regs->tok_list0, i2cs[current_id].regs->tok_list1, (readl(&i2cs[current_id].regs->fifo_st0) >> 8) & 0x7);
#endif
	meson_i2c_reset_tokens();
}

static void meson_i2c_fifo_polling_prepare_xfer(void)
{
	bool write = !(i2cs[current_id].msg->flags & I2C_M_RD);
	uint32_t i, j, tmp_floor, left_count, token_left_count;
	uint32_t w_fifo_count, token_fifo_count;

	/* write fifo total size is 8*8 = 64 bytes , max fifo fill 32bytes data once */
	i2cs[current_id].count = MIN(i2cs[current_id].msg->len - i2cs[current_id].pos, 32u);
	tmp_floor = (i2cs[current_id].count)/8;
	left_count = (i2cs[current_id].count)%8;
	token_left_count = (i2cs[current_id].count)%16;
	w_fifo_count = ((i2cs[current_id].regs->fifo_st0) << 4) & 0xf;
	token_fifo_count = ((i2cs[current_id].regs->fifo_st0) << 8) & 0x7;
#if I2C_DEBUG
	printf("pre xfer: count = %d, tmp_floor = %d, left_count = %d, token_left_count = %d\n", i2cs[current_id].count, tmp_floor, left_count, token_left_count);
	printf("pre xfer: before add fifos-, i2c reg : 0x%x = 0x%x, w_fifo_count = %d, token_fifo_count = %d\n",
	&i2cs[current_id].regs->fifo_st0, i2cs[current_id].regs->fifo_st0, w_fifo_count, token_fifo_count);
#endif

	/* deal 4 floor wdata fifo */
	if (write && w_fifo_count < 4 ) {
		for (j = 0; j < MIN(tmp_floor,(uint32_t)4); j++) {
			/* deal one floor write fifo */
			meson_i2c_put_data(i2cs[current_id].msg->buf + i2cs[current_id].pos, 8);
			/* Update position, if not, the write fifo data is wrong */
			i2cs[current_id].pos += 8;
#if I2C_DEBUG
		printf("pre xfer: 1fifo wdata over, i2c reg : 0x%x = 0x%x\n", &i2cs[current_id].regs->fifo_st0, i2cs[current_id].regs->fifo_st0);
#endif
		}
		if (left_count) {
			meson_i2c_put_data(i2cs[current_id].msg->buf + i2cs[current_id].pos, left_count);
			i2cs[current_id].pos += left_count;
#if I2C_DEBUG
		printf("pre xfer: 2fifo wdata over, i2c reg : 0x%x = 0x%x\n", &i2cs[current_id].regs->fifo_st0, i2cs[current_id].regs->fifo_st0);
#endif
		}
	}

	/* deal 2 floor token list fifo */
	if (token_fifo_count < 2) {
		for (j = 0; j < MIN(tmp_floor/2,(uint32_t)2); j++) {
			for (i = 0; i + 1 < MAX_TOKEN; i++) {
				meson_i2c_add_token(TOKEN_DATA);
				if (write) {
					if (i2cs[current_id].num_tokens %(11-1) == 0);
						meson_i2c_add_token(TOKEN_START);
						meson_i2c_add_token(TOKEN_SLAVE_ADDR_WRITE);
				}
			}

			if (write || i2cs[current_id].pos + MAX_TOKEN < i2cs[current_id].msg->len) {
				 meson_i2c_add_token(TOKEN_DATA);
				if (write) {
					if (i2cs[current_id].num_tokens %(11-1) == 0);
						meson_i2c_add_token(TOKEN_START);
						meson_i2c_add_token(TOKEN_SLAVE_ADDR_WRITE);
				}
			} else
				 meson_i2c_add_token(TOKEN_DATA_LAST);

			i2cs[current_id].regs->tok_list0 = i2cs[current_id].tokens[0];
			i2cs[current_id].regs->tok_list1 = i2cs[current_id].tokens[1];
			/* prepare for next floor token list */
			meson_i2c_reset_tokens();
#if I2C_DEBUG
			printf("pre xfer: fill token fifo over, i2c reg : 0x%x = 0x%x\n", &i2cs[current_id].regs->fifo_st0, i2cs[current_id].regs->fifo_st0);
			printf("pre xfer:floor %d list0 = 0x%x, list1 = 0x%x\n", i, i2cs[current_id].regs->tok_list0, i2cs[current_id].regs->tok_list1);
#endif
		}
	}

	/*
	 * TOKEN_DATA_LAST is used for a read
	 */
	if (write) {
		for (i = 0; i < token_left_count; i++)
			meson_i2c_add_token(TOKEN_DATA);
	}

	if (!write) {
		/* write fifo fill one floor to around read failed
		 * avoid reading over one byte error
		 */
		i2cs[current_id].regs->tok_wdata0 = 0;
		i2cs[current_id].regs->tok_wdata1 = 0;
		/* the last data is different for a read, skip the last */
		for (i = 0; i + 1 < left_count; i++)
			meson_i2c_add_token(TOKEN_DATA);
		if (left_count) {
			if (i2cs[current_id].pos + left_count < i2cs[current_id].msg->len)
				meson_i2c_add_token(TOKEN_DATA);
			else
				meson_i2c_add_token(TOKEN_DATA_LAST);
		}
	}

	if (i2cs[current_id].last && i2cs[current_id].pos + i2cs[current_id].count >= i2cs[current_id].msg->len)
		meson_i2c_add_token(TOKEN_STOP);

	if (token_left_count) {
		i2cs[current_id].regs->tok_list0 = i2cs[current_id].tokens[0];
		i2cs[current_id].regs->tok_list1 = i2cs[current_id].tokens[1];
#if I2C_DEBUG
		printf("pre xfer: add left, fifo token over, i2c reg : 0x%x = 0x%x\n", &i2cs[current_id].regs->fifo_st0, i2cs[current_id].regs->fifo_st0);
#endif
	}
}

static void meson_i2c_fifo_polling_init(void)
{
	setbits_le32(&i2cs[current_id].regs->fifo_ctrl0, REG_FIFO_RD_RST);
	setbits_le32(&i2cs[current_id].regs->fifo_ctrl0, REG_FIFO_WD_RST);
	setbits_le32(&i2cs[current_id].regs->fifo_ctrl0, REG_FIFO_TK_RST);
	_udelay(2);
	clrbits_le32(&i2cs[current_id].regs->fifo_ctrl0, REG_FIFO_RD_RST);
	clrbits_le32(&i2cs[current_id].regs->fifo_ctrl0, REG_FIFO_WD_RST);
	clrbits_le32(&i2cs[current_id].regs->fifo_ctrl0, REG_FIFO_TK_RST);
	clrbits_le32(&i2cs[current_id].regs->fifo_ctrl0, REG_FIFO_TIMEOUT_STOP);
	clrbits_le32(&i2cs[current_id].regs->fifo_ctrl0, REG_FIFO_TIMEOUT_CLR);
	clrbits_le32(&i2cs[current_id].regs->fifo_ctrl0, REG_FIFO_TIMEOUT_EN);
	clrbits_le32(&i2cs[current_id].regs->fifo_ctrl0, REG_FIFO_IRQ_MODE);
	clrbits_le32(&i2cs[current_id].regs->fifo_ctrl0, REG_FIFO_IRQ_MODE);
	setbits_le32(&i2cs[current_id].regs->fifo_ctrl0, REG_FIFO_MODE);

	/* set the initial threshold */
	clrsetbits_le32(&i2cs[current_id].regs->fifo_ctrl1, REG_FIFO_TK_THD_MASK, 0x2 << 0);
	clrsetbits_le32(&i2cs[current_id].regs->fifo_ctrl1, REG_FIFO_WD_THD_MASK, 0x4 << 4);
	clrsetbits_le32(&i2cs[current_id].regs->fifo_ctrl1, REG_FIFO_RD_THD_MASK, 0x4 << 8);
	clrsetbits_le32(&i2cs[current_id].regs->fifo_ctrl1, REG_FIFO_TO_THD_MASK, 0x0);
}

static int32_t meson_i2c_xfer_fifo_polling_msg(struct i2c_msg *msg, uint32_t last)
{
	uint32_t time_count = 0;
	volatile uint32_t *ctrl = &i2cs[current_id].regs->ctrl;

#if I2C_DEBUG
	serial_puts("meson i2c fifo polling:");
	if (msg->flags & I2C_M_RD)
		serial_puts("read ");
	else
		serial_puts("write ");
	printf("addr 0x%x, len %d\n", msg->addr, msg->len);
#endif
	i2cs[current_id].msg = msg;
	i2cs[current_id].last = last;
	i2cs[current_id].pos = 0;
	i2cs[current_id].count = 0;

	meson_i2c_reset_tokens();
	meson_i2c_do_start(msg);
	meson_i2c_fifo_polling_init();

	do {
		meson_i2c_fifo_polling_prepare_xfer();
		/* start the transfer */
		setbits_le32((&i2cs[current_id].regs->ctrl), REG_CTRL_START);

		while (readl(ctrl) & REG_CTRL_STATUS) {
			if (time_count > I2C_TIMEOUT_MS) {
				clrbits_le32(&i2cs[current_id].regs->ctrl, REG_CTRL_START);
				printf("meson i2c time out, ctrl = 0x%x\n", readl(ctrl));
				return -1;
			}
			_udelay(1);
			time_count++;
		}

		meson_i2c_reset_tokens();

		clrbits_le32(&i2cs[current_id].regs->ctrl, REG_CTRL_START);

		if (readl(ctrl) & REG_CTRL_ERROR) {
			printf("meson i2c: error ctrl = 0x%x\n", readl(ctrl));
			return -1;
		}

		if ((msg->flags & I2C_M_RD) && i2cs[current_id].count) {
			meson_i2c_fifo_polling_get_data(i2cs[current_id].msg->buf + i2cs[current_id].pos,
							i2cs[current_id].count);
		}
		if (msg->flags & I2C_M_RD)
			i2cs[current_id].pos += i2cs[current_id].count;
#if I2C_DEBUG
		printf("One msg transfer over, compelte %d bytes data, prepare the left\n", i2cs[current_id].pos);
#endif
	} while (i2cs[current_id].pos < msg->len);

	return 0;
}

int32_t meson_i2c_xfer(struct i2c_msg *msg, uint32_t nmsgs)
{
	uint32_t i;
	int32_t	ret;

	for (i = 0; i < nmsgs; i++) {
		if (i2cs[current_id].mode == NORMAL_MODE)
			ret = meson_i2c_xfer_msg(msg + i, i == (nmsgs - 1));
		else if (i2cs[current_id].mode == FIFO_POLLING)
			ret = meson_i2c_xfer_fifo_polling_msg( msg + i, i == (nmsgs - 1));
		else {
			ret = -1;
			serial_puts("Not support fifo irq mode\n");
		}
		if (ret) {
			serial_puts("meson_i2c_xfer error\n");
			return ret;
		}
	}

	return 0;
}

/*
 * need Bytes of Writing one slave register
 */
void meson_i2c_fifo_write_interval(uint32_t val)
{
	i2cs[current_id].interval = val;
}

static void meson_i2c_fifo_add_token(uint32_t token)
{
	if (i2cs[current_id].num_tokens < 8)
		i2cs[current_id].tokens[0] |= (token & 0xf) << (i2cs[current_id].num_tokens * 4);
	else
		i2cs[current_id].tokens[1] |= (token & 0xf) << ((i2cs[current_id].num_tokens % 8) * 4);

	i2cs[current_id].num_tokens++;
	i2cs[current_id].fifo_count++;
	/*token list is full, fill it in tfifo, and reset tokens */
	if (i2cs[current_id].num_tokens == 16)
		meson_i2c_fill_token_list();
}

uint32_t reg_list[6] = {1,2,4,4,4,6};

uint32_t check_tfifo_is_full(struct meson_i2c *i2c)
{
	if (((readl(&i2cs[current_id].regs->fifo_st0) >> 8) & 0x7) > 2)
		return 1;
	else
		return 0;
}

uint32_t check_wfifo_is_full(struct meson_i2c *i2c)
{
	if (((readl(&i2cs[current_id].regs->fifo_st0) >> 4) & 0xf) > 4)
		return 1;
	else
		return 0;
}

static int32_t meson_i2c_fifo_write_msg(struct i2c_msg *msg)
{
	volatile uint32_t *ctrl = &i2cs[current_id].regs->ctrl;
	uint32_t len, cur_len, i, token_idx = 0, wtimes = 0, ttimes = 0, total, tcnt;
	uint32_t token_cell[16];
	static uint32_t data_pos = 0;
	struct meson_i2c *i2c = &i2cs[current_id];
	/* calc tfifo cnt */
	total = (msg->len)/6 * 16;
	tcnt = total;
	len = msg->len;

	i2c->msg = msg;
	i2c->fifo_count = 0;
	i2cs->wf_pos = 0;

	clrbits_le32(&i2cs[current_id].regs->ctrl, REG_CTRL_START);
	meson_i2c_reset_tokens();
	i2c->regs->slave_addr = msg->addr << 1;
	meson_i2c_fifo_polling_init();

#if I2C_FIFO_WRITE
	/* cacl wfifo floor, left */
	uint32_t wf_floor, wf_left;
	wf_floor = (msg->len)/8;
	wf_left = msg->len%8;
	for (i = 0; i < msg->len;i++)
		printf("i2c buf[%d] = 0x%02x\n",i, msg->buf[i]);
	printf("fifo write: addr 0x%x, len %d\n", msg->addr, msg->len);
	printf("%d token waiting to transfer, include %d floor, %d left\n", total, total/16, total%16);
	printf("%d data waiting to transfer, include %d floor, %d left\n", msg->len, wf_floor, wf_left);
	printf("Wf cnt = %d, Tf_cnt = %d\n",
		readl(&i2cs[current_id].regs->fifo_st0) >> 4 & 0xf,
		readl(&i2cs[current_id].regs->fifo_st0) >> 8 & 0x7);
#endif
	do {
		/*Fill token fifo, You'd better fill in datas with multiple of 6 */
		while (!check_tfifo_is_full(i2c) && total) {
			ttimes++;
			cur_len = MIN(total, 16);
			for (i = 0; i < cur_len;i++) {
				if (i >= 12) {
					meson_i2c_fifo_add_token(0);
					break;
				}
				token_cell[i] = reg_list[token_idx++];
				meson_i2c_fifo_add_token(token_cell[i]);
				if (token_idx == (i2c->interval + 3))
					token_idx = 0;
			}
			total -= cur_len;
			if (ttimes == 4 || tcnt < 64)
				setbits_le32((&i2cs[current_id].regs->ctrl), REG_CTRL_START);

			if (i2cs[current_id].num_tokens)
				meson_i2c_fill_token_list();
#if I2C_FIFO_WRITE
			printf("T:%d,%d,%d\n", ttimes, total, cur_len);
#endif
		}
		while (!check_wfifo_is_full(i2c) && len) {
			wtimes++;
			/* Keep the same with Tokne
			 * One tfifo format: SWDDDP SWDDDP E E E E
			 * One wfifo format: DDDDDD00
			 */
			cur_len = MIN(len, 6);
			meson_i2c_put_data(&(msg->buf[data_pos]), cur_len);
			i2cs[current_id].wf_pos += cur_len;
			data_pos += cur_len;
			len -= cur_len;
#if I2C_FIFO_WRITE
			printf("W:%d,%d,%d,%d\n",len, wtimes, i2cs[current_id].wf_pos, data_pos);
#endif
		}

		/* Make sure the regs writing in */
		_udelay(50);
#if I2C_FIFO_WRITE
		i2cs[current_id].time_count++;
		if (i2cs[current_id].time_count%4000 == 0) {
			printf("data left = %d, token left = %d, wtimes = %d, ttimes = %d\n", len, total,
				wtimes, ttimes);
			printf("Wf cnt = %d, Tf_cnt = %d\n",
			readl(&i2cs[current_id].regs->fifo_st0) >> 4 & 0xf,
			readl(&i2cs[current_id].regs->fifo_st0) >> 8 & 0x7);
			meson_i2c_dump_regs();
		}
#endif
	} while ((total != 0) || (len != 0));

	if (readl(ctrl) & REG_CTRL_ERROR) {
		printf("meson i2c fifo write: i2c error = 0x%x\n", readl(ctrl));
		return -1;
	}
	while (readl(ctrl) & REG_CTRL_STATUS) {
		if (i2cs[current_id].time_count > I2C_TIMEOUT_MS) {
			clrbits_le32(&i2cs[current_id].regs->ctrl, REG_CTRL_START);
			printf("meson i2c fifo write: i2c time out ctrl reg = 0x%x\n", readl(ctrl));
			meson_i2c_dump_regs();
			return -1;
		}
		_udelay(1);
		i2cs[current_id].time_count++;
	}
	/* Transfer is over */
	clrbits_le32(&i2cs[current_id].regs->ctrl, REG_CTRL_START);
#if I2C_FIFO_WRITE
	printf("This transfer cost %d us, pos = %d \n", i2cs[current_id].time_count, i2cs[current_id].pos);
#endif
	return 0;
}

int32_t meson_i2c_fifo_write(uint32_t addr, uint8_t offset,
			uint8_t *buffer, uint32_t len)
{
	struct i2c_msg msg[1];
	int32_t ret = 0;
	uint8_t buf[len + 1];

	buf[0] = offset;

	msg->addr = addr;
	msg->len = len + 1;/* addr's length + len */
	msg->buf = buf;
	msg->flags= 0;

	memcpy(&buf[1], buffer, len);

	ret = meson_i2c_fifo_write_msg(msg);
	if (ret < 0) {
		serial_puts("meson i2c: fifo write failed\n");
		return ret;
	}

	return 0;
}

int32_t meson_i2c_set_speed(uint32_t speed)
{
	uint32_t clk_rate = i2cs[current_id].clkin_rate;
	uint32_t div;

	div = clk_rate/(speed * i2cs[current_id].div_factor);

	/* clock divider has 12 bits */
	if (div >= (1 << 12)) {
		serial_puts("meson i2c: requested bus frequency too low\n");
		div = (1 << 12) - 1;
	}
	clrsetbits_le32(&i2cs[current_id].regs->ctrl, REG_CTRL_CLKDIV_MASK, (div & GENMASK(9, 0)) << REG_CTRL_CLKDIV_SHIFT);

	clrsetbits_le32(&i2cs[current_id].regs->ctrl, REG_CTRL_CLKDIVEXT_MASK,
			(div >> 10) << REG_CTRL_CLKDIVEXT_SHIFT);

	return 0;
}

int32_t meson_i2c_read(uint32_t addr, uint8_t offset,
		       uint8_t *buffer, uint32_t len)
{
	struct i2c_msg msg[2], *ptr;
	uint32_t msg_count;
	int32_t ret;

	if (!len) {
		serial_puts("invalid length\n");
		return -1;
	}

	ptr = msg;

	ptr->addr = addr;
	ptr->flags = 0;
	ptr->len = 1;
	ptr->buf = &offset;
	ptr++;

	ptr->addr = addr;
	ptr->flags = I2C_M_RD;
	ptr->len = len;
	ptr->buf = buffer;
	ptr++;

	msg_count = ptr - msg;
	ret = meson_i2c_xfer(msg, msg_count);
	if (ret < 0) {
		serial_puts("meson i2c: read failed\n");
		return ret;
	}

	return 0;
}


int32_t meson_i2c_write(uint32_t addr, uint8_t offset,
			uint8_t *buffer, uint32_t len)
{
	struct i2c_msg msg[1];
	int32_t ret = 0;
	uint8_t buf[len + 1];

	buf[0] = offset;

	msg->addr = addr;
	msg->len = len + 1;/* addr's length + len */
	msg->buf = buf;
	msg->flags= 0;

	memcpy(&buf[1], buffer, len);
	ret = meson_i2c_xfer(msg, 1);
	if (ret < 0) {
		serial_puts("meson i2c: write failed\n");
		return ret;
	}

	return 0;
}

/*
 *i2c master platform data init
 */
int32_t meson_i2c_port_init(uint32_t id)
{
	struct meson_i2c_platdata *plat = NULL;

	plat = &c2_i2c_data[id];

	i2cs[id].regs = (struct i2cregs *)plat->reg;
	i2cs[id].div_factor = plat->div_factor;
	i2cs[id].delay_ajust = plat->delay_ajust;
	i2cs[id].clock_frequency = plat->clock_frequency;
	i2cs[id].clkin_rate = plat->clkin_rate;
	i2cs[id].mode = plat->mode;

	current_id = id;

#if I2C_DEBUG
	printf("index = %d, reg = 0x%x, div = %d, delay = %d ,clock-frequency = %d, mode = %d\n",
	plat->bus_num, i2cs[id].regs, i2cs[id].div_factor, i2cs[id].delay_ajust, i2cs[id].clock_frequency,
	i2cs[id].mode);
#endif
	meson_i2c_regs_init();

	meson_i2c_set_speed(i2cs[id].clock_frequency);/*init i2c work speed*/

	return 0;
}
