#ifndef _RZG2L_VIDEO_H_
#define _RZG2L_VIDEO_H_

/* U-boot Header*/
#include <dm/uclass.h>
#include <dm/device_compat.h>
#include <cpu_func.h>
#include <common.h>
#include <asm/io.h>
#include <i2c.h>
#include <linux/delay.h>
#include <string.h>

/* Register Define */
#include "dsi_reg.h"
#include "rzg2l_def.h"

/* Screen Header */
// #include "dsi_define_720P.h"
//#include "dsi_define_1080P.h"

#define ADV7535_SADDR                   0x7A
#define ADV7535_DSI_CEC_ADDR            0x78

/* Du Define */
#if 1
#define DU_MCR0                         0x00
#define DU_MSR0                         0x04
#define DU_DITR0                        0x10
#define DEMD(n)                         (n << 8)
#define FIXED_TO_LOWD                   0
#define DATA_ENABLE                     3
#define VSPOL(n)                        (n << 16)
#define HSPOL(n)                        (n << 17)
#define HIG_ACT                         1
#define LOW_ACT                         0

#define DU_DITR1                        0x14
#define DU_DITR1_VSA(x)                 ((x) << 0)
#define DU_DITR1_VACTIVE(x)             ((x) << 16)

#define DU_DITR2                        0x18
#define DU_DITR2_VBP(x)                 ((x) << 0)
#define DU_DITR2_VFP(x)                 ((x) << 16)

#define DU_DITR3                        0x1C
#define DU_DITR3_HSA(x)                 ((x) << 0)
#define DU_DITR3_HACTIVE(x)             ((x) << 16)

#define DU_DITR4                        0x20
#define DU_DITR4_HBP(x)                 ((x) << 0)
#define DU_DITR4_HFP(x)                 ((x) << 16)

#define DU_DITR5                        0x24
#define DU_DITR5_VSFT(x)                ((x) << 0)
#define DU_DITR5_HSFT(x)                ((x) << 16)

#define DU_MCR1                         0x40
#define OPMD(n)                         (n << 0)
#define OPMD0                           0
#define OPMD1                           1
#define OPMD2                           2
#define OPMD3                           3
#define PB_AUTOCLR                      (1 << 16)

#define DU_PBCR0                        0x4C
#define DU_PBCR0_PB_DEP(x)              ((x) << 0)

#define DU_PBCR1                        0x50
#define PB_RUFOP                        (1 << 0)

#define DU_PBCR2                        0x54
#define PB_RUFDAT(n)                    (n << 0)
#endif

/* DPHY Registers */
#define DSIDPHYCTRL0			0x00
#define DSIDPHYCTRL0_CAL_EN_HSRX_OFS	(1 << 16)
#define DSIDPHYCTRL0_CMN_MASTER_EN	(1 << 8)
#define DSIDPHYCTRL0_RE_VDD_DETVCCQLV18	(1 << 2)
#define DSIDPHYCTRL0_EN_LDO1200		(1 << 1)
#define DSIDPHYCTRL0_EN_BGR		(1 << 0)

#define DSIDPHYTIM0			0x04
#define DSIDPHYTIM0_TCLK_MISS(x)	((x) << 24)
#define DSIDPHYTIM0_T_INIT(x)		((x) << 0)

#define DSIDPHYTIM1			0x08
#define DSIDPHYTIM1_THS_PREPARE(x)	((x) << 24)
#define DSIDPHYTIM1_TCLK_PREPARE(x)	((x) << 16)
#define DSIDPHYTIM1_THS_SETTLE(x)	((x) << 8)
#define DSIDPHYTIM1_TCLK_SETTLE(x)	((x) << 0)

#define DSIDPHYTIM2			0x0C
#define DSIDPHYTIM2_TCLK_TRAIL(x)	((x) << 24)
#define DSIDPHYTIM2_TCLK_POST(x)	((x) << 16)
#define DSIDPHYTIM2_TCLK_PRE(x)		((x) << 8)
#define DSIDPHYTIM2_TCLK_ZERO(x)	((x) << 0)

#define DSIDPHYTIM3			0x10
#define DSIDPHYTIM3_TLPX(x)		((x) << 24)
#define DSIDPHYTIM3_THS_EXIT(x)		((x) << 16)
#define DSIDPHYTIM3_THS_TRAIL(x)	((x) << 8)
#define DSIDPHYTIM3_THS_ZERO(x)		((x) << 0)

#define DSIDPHYCTRL1			0x40
#define DSIDPHYCTRL1_TRIM_REGSEL	(1 << 0)

/* DSI Link*/

/* Clock Lane Stop Time Set Register */
#define CLSTPTSETR			0x314
#define CLSTPTSETR_CLKKPT(x)		((x) << 24)
#define CLSTPTSETR_CLKBFHT(x)		((x) << 16)
#define CLSTPTSETR_CLKSTPT(x)		((x) << 2)

/* LP Transition Time Set Register */
#define LPTRNSTSETR			0x318
#define LPTRNSTSETR_GOLPBKT(x)		((x) << 0)

/* Video-Input Channel 1 Set 1 Register */
#define VICH1SET1R			0x404
#define VICH1SET1R_DLY(x)		(((x) & 0xfff) << 2)

/* Video-Input Channel 1 Vertical Size Set Register */
#define VICH1VSSETR			0x428
#define VICH1VSSETR_VACTIVE(x)		(((x) & 0x7fff) << 16)
#define VICH1VSSETR_VSPOL_LOW		(1 << 15)
#define VICH1VSSETR_VSPOL_HIGH		(0 << 15)
#define VICH1VSSETR_VSA(x)		(((x) & 0xfff) << 0)

/* Video-Input Channel 1 Vertical Porch Set Register */
#define VICH1VPSETR			0x42C
#define VICH1VPSETR_VFP(x)		(((x) & 0x1fff) << 16)
#define VICH1VPSETR_VBP(x)		(((x) & 0x1fff) << 0)

/* Video-Input Channel 1 Horizontal Size Set Register */
#define VICH1HSSETR			0x430
#define VICH1HSSETR_HACTIVE(x)		(((x) & 0x7fff) << 16)
#define VICH1HSSETR_HSPOL_LOW		(1 << 15)
#define VICH1HSSETR_HSPOL_HIGH		(0 << 15)
#define VICH1HSSETR_HSA(x)		(((x) & 0xfff) << 0)

/* Video-Input Channel 1 Horizontal Porch Set Register */
#define VICH1HPSETR			0x434
#define VICH1HPSETR_HFP(x)		(((x) & 0x1fff) << 16)
#define VICH1HPSETR_HBP(x)		(((x) & 0x1fff) << 0)


int rzg2l_video_init(void);

#endif /*_RZG2L_VIDEO_H_*/
