#include "rzg2l_video.h"
#include <common.h>
#include <command.h>

//#include "lcdc_iodefine.h"
//#include "rzg2l_display.h"

/***********************************************************************
 * GPIO PINMUX Macro definitions
 **********************************************************************/
/* 寄存器读写宏定义 */
#define BIT(nr)         (1UL << (nr))

/* PFC寄存器偏移定义 */
#define PMC(off)        (0x0200 + (off))      // 端口模式控制
#define PFC(off)        (0x0400 + (off) * 4)  // 端口功能控制
#define PWPR            (0x3014)              // 端口写保护寄存器
#define PWPR_G3S        (0x3000)
#define IOLH(off)       (0x1000 + (off) * 8)  // IO电平控制
#define PUPD(off)       (0x1C00 + (off) * 8)  // 上下拉控制
#define SR(off)         (0x1400 + (off) * 8)  // 转换速率控制
#define PM(off)         (0x0100 + (off) * 2)  // 端口模式控制

#define PWPR_B0WI       BIT(7)  /* Bit Write Disable */
#define PWPR_PFCWE      BIT(6) /* PFC Register Write Enable */

#define PM_MASK         0x03
#define PFC_MASK        0x7

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/


typedef struct
{
    uint32_t set_address;
    uint32_t set_data;
} set_address_data_t;

#define COMPUTE_X(index, width) ((LCD_HACTIVE - width) * 3 / 2 + index % (width * 3))
#define COMPUTE_Y(index, width, height) ((LCD_VACTIVE - height) / 2 + height - i / 3 / width)

#ifdef DSI_PANEL
static void rzg2l_cpg_dsi_rst_init(void);
static void adv7535_init(void);
static void adv7535_deinit(void);
static void rzg2l_dsi_phy_init(void);
static void rzg2l_dsi_link_init(void);
#endif
static void rzg2l_fpvcg_init(void);
static void rzg2l_du_init(void);
static void rzg2l_vcpd_init(void);
static void rzg2l_lcdc_start(void);
static void rzg2l_lcdc_stop(void);
static void rzg2l_dpi_pin_init(void);
static void rzg2l_dpi_cpg_init(void);

/* BITS_PER_PIXEL */
#define DISPLAY_BPP_INPUT0      (24)


// DU timing parameters for 480x272@60Hz (根据数据表调整)
#define LCD_HACTIVE             480  // 水平有效像素
#define LCD_VACTIVE             272  // 垂直有效行数
#define LCD_HFRONT              2    // 水平前肩 (Thfp) - 从2+41+2变为2
#define LCD_HSYNC               41   // 水平同步 (Thbp) - 保持41
#define LCD_HBACK               2    // 水平后肩 (Thbp) - 从40变为2
#define LCD_VFRONT              2    // 垂直前肩 (Tvfp) - 从8变为2
#define LCD_VSYNC               10   // 垂直同步 (Tvb) - 从8变为10
#define LCD_VBACK               2    // 垂直后肩 (Tvb) - 从8变为2
#define LCD_VSPOL               1    // 垂直同步极性
#define LCD_HSPOL               1    // 水平同步极性
#define LCD_DEMD                0x3  // 数据使能模式

// VSPD parameters (保持不变)
#define LCD_BPP                 24   // 像素位深
#define LCD_VIR                 0    // 虚拟输入使能
#define LCD_RDFMT               0x18 // 读格式
#define LCD_RDCSC               0    // 色彩空间转换使能
#define LCD_SRCM_ADDR           0x58000000  // 源内存地址
#define LCD_WRFMT               0x18 // 写格式
#define LCD_WRCSC               0    // 写色彩空间转换
#define LCD_ODE                 0    // 输出数据使能
#define LCD_CFMT                0    // 色彩格式

#define FB_ADDR                 LCD_SRCM_ADDR

volatile uint32_t   *g_framebuffer = (volatile uint32_t *)FB_ADDR;

/***********************************************************************************************************************
 * GPIO API definitions
 **********************************************************************************************************************/

static void rzg2l_set_iohl(u32 off, u8 pin, u8 strangth)
{
    void __iomem *addr = RZG2L_GPIO_BASE + IOLH(off);
    unsigned long flags;
    u32 reg;

    /* handle _L/_H for 32-bit register read/write */
    if (pin >= 4) {
        pin -= 4;
        addr += 4;
    }

    reg = readl(addr) & ~(0x03 << (pin * 8));
    writel(reg | (strangth << (pin * 8)), addr);
}

static void rzg2l_set_sr(u32 off, u8 pin, u8 mode)
{
    void __iomem *addr = RZG2L_GPIO_BASE + SR(off);
    unsigned long flags;
    u32 reg;

    /* handle _L/_H for 32-bit register read/write */
    if (pin >= 4) {
        pin -= 4;
        addr += 4;
    }

    reg = readl(addr) & ~(0x03 << (pin * 8));
    writel(reg | (mode << (pin * 8)), addr);
}

static void rzg2l_set_pupd(u32 off, u8 pin, u8 mode)
{
    void __iomem *addr = RZG2L_GPIO_BASE + PUPD(off);
    unsigned long flags;
    u32 reg;

    /* handle _L/_H for 32-bit register read/write */
    if (pin >= 4) {
        pin -= 4;
        addr += 4;
    }

    reg = readl(addr) & ~(0x03 << (pin * 8));
    writel(reg | (mode << (pin * 8)), addr);
}

static void rzg2l_set_gpio(u8 off, u8 pin, u8 func, u8 set_strangth)
{
    u32 reg;

    printf("Setting GPIO: port=%d, pin=%d, func=%d\n", off, pin, func);

    if (set_strangth != 0)
        rzg2l_set_iohl(off, pin, set_strangth);

    rzg2l_set_sr(off, pin, 1);

    /* Set pin to 'Non-use (Hi-Z input protection)'  */
    reg = readw(RZG2L_GPIO_BASE + PM(off));
    reg &= ~(PM_MASK << (pin * 2));
    writew(reg, RZG2L_GPIO_BASE + PM(off));

    /* Temporarily switch to GPIO mode with PMC register */
    reg = readb(RZG2L_GPIO_BASE + PMC(off));
    writeb(reg & ~BIT(pin), RZG2L_GPIO_BASE + PMC(off));

    /* Set the PWPR register to allow PFC register to write */
    writel(0x0, RZG2L_GPIO_BASE + PWPR);                /* B0WI=0, PFCWE=0 */
    writel(PWPR_PFCWE,RZG2L_GPIO_BASE + PWPR);          /* B0WI=0, PFCWE=1 */

    /* Select Pin function mode with PFC register */
    reg = readl(RZG2L_GPIO_BASE + PFC(off));
    reg &= ~(PFC_MASK << (pin * 4));
    writel(reg | (func << (pin * 4)), RZG2L_GPIO_BASE + PFC(off));

    /* Set the PWPR register to be write-protected */
    writel(0x0, RZG2L_GPIO_BASE + PWPR);                /* B0WI=0, PFCWE=0 */
    writel(PWPR_B0WI, RZG2L_GPIO_BASE + PWPR);          /* B0WI=1, PFCWE=0 */

    /* Switch to Peripheral pin function with PMC register */
    reg = readb(RZG2L_GPIO_BASE + PMC(off));
    writeb(reg | BIT(pin), RZG2L_GPIO_BASE + PMC(off));
}

static void set_white_screen_display(uint32_t   *framebuffer, uint8_t color)
{
    uint32_t size = LCD_HACTIVE * LCD_VACTIVE * (DISPLAY_BPP_INPUT0 >> 3);

    printf("%s: start\r\n", __func__);
    dcache_disable();
    printf("%s: start: framebuffer[Addr:%8X] set to %x\r\n",
            __func__, framebuffer, color);
    memset( framebuffer, color, size);
    dcache_enable();
}

static const uint32_t dpi_pin_register_values[][2] = { //0x11030000
//    {0x11033014,0x00000000},    //Diable write protect (PWPR) for PFC
//    {0x11033014,0x00000040},

    {0x11030214,0x07030301},    //PMC set VSYNC/HSYNC/CLK/DE/Data0

    {0x11030218,0x03030307},    //PMC set  Data 1/2/3/4/5/6/7/8/9
    {0x1103021C,0x03030703},    //PMC set Data 10/11/12/13/14/15/16/17/18
    {0x11030220,0x03030703},    //PMC set Data 19/20/21/22/23
    {0x11030458,0x00000011},    //PFC set CLK
    {0x1103045C,0x00000111},    //PFC set VSYNC/DE/Data0
    {0x11030460,0x00000111},    //PFC set Data1/2/3
    {0x11030464,0x00000011},    //PFC set Data4/5
    {0x11030468,0x00000011},    //PFC set Data6/7
    {0x1103046C,0x00000011},    //PFC set Data8/9
    {0x11030470,0x00000011},    //PFC set Data10/11
    {0x11030474,0x00000111},    //PFC set Data12/13/14
    {0x11030478,0x00000011},    //PFC set Data15/16
    {0x1103047C,0x00000011},    //PFC set Data17/18
    {0x11030480,0x00000011},    //PFC set Data19/2
    {0x11030484,0x00000111},    //PFC set Data21/22/23
#if 0
    {0x11030813,0x00000001},    // ???

    {0x11030013,0x00000002},
    {0x11030128,0x00000008},

    {0x11030014,0x00000002},
    {0x11030126,0x00000008},

    {0x11030039,0x00000002},
    {0x11030172,0x00000008},
#endif
};

// 为480x272@60Hz配置PLL5
// 目标像素时钟为9MHz (根据数据表)
#define PL5_REFDIV      2
#define PL5_INTIN       37      // 修改PLL5倍频系数
#define PL5_FRACIN      4404019 // 修改PLL5小数部分
#define PL5_POSTDIV1    8
#define PL5_POSTDIV2    1
#define PL5_DIVVAL      0
#define PL5_SPREAD      0x16

// vclk1 = 113.3 MHz / 12 = 9.44 MHz
#define CPG_DSI_DIV_A       2
#define CPG_DSI_DIV_B       1

static const uint32_t cpg_dpi_reg_init[][2] = {
    {RZG2L_CPG_BASE + CPG_SIPLL5_CLK3, (PL5_DIVVAL << 0) |
                                       (PL5_FRACIN << 8)},

    {RZG2L_CPG_BASE + CPG_SIPLL5_CLK4, PL5_INTIN << 16},

    {RZG2L_CPG_BASE + CPG_SIPLL5_CLK1, POSTDIV1(PL5_POSTDIV1) |
                                       POSTDIV2(PL5_POSTDIV2) |
                                       REFDIV(PL5_REFDIV)},

    {RZG2L_CPG_BASE + CPG_PL5_SDIV, DSI_CLK_DIVA_VAL(DSI_DIV_A) |
                                    DSI_CLK_DIVB_VAL(DSI_DIV_B)},

    {RZG2L_CPG_BASE + CPG_SIPLL5_STBY, DOWNSPREAD | RESETB | SSCG_EN},
    {RZG2L_CPG_BASE + CPG_CLKON_LCDC, LCD_CLK_EN},
    {RZG2L_CPG_BASE + CPG_CLKON_GPIO, GPIO_CLK_EN},
};

static const uint32_t cpg_dpi_rst_init[][2] = {
    {RZG2L_CPG_BASE + CPG_RST_LCDC, UNIT0_RST_WEN | UNIT0_RST_STOP},
    {RZG2L_CPG_BASE + CPG_OTHERFUNC1_REG, RES0_ON_W_EN | RES0_SET},
};

static const uint32_t fcpvd_register_values[][2] = {
    {RZG2L_FCPVD_BASE, 0x00000109},
};

#ifdef DSI_PANEL
/* step2 */
static const uint32_t cpg_dsi_init[][2] = {
 /* {RZG2L_CPG_BASE + CPG_PL1_DDIV, 0x10000000},*/

 {RZG2L_CPG_BASE + CPG_PL2_DDIV, DIV_DSI_LPCLK_SET},
 {RZG2L_CPG_BASE + CPG_PL5_SDIV, DSI_CLK_DIVA_VAL(1) | DSI_CLK_DIVB_VAL(1)},
 {RZG2L_CPG_BASE + CPG_SIPLL5_CLK1, POSTDIV1(PL5_POSTDIV1)      |
                                        POSTDIV2(PL5_POSTDIV2)  |
                                        REFDIV(PL5_REFDIV)},

 /* {RZG2L_CPG_BASE + CPG_SIPLL5_CLK2, FOUTVCOPD_WEN(1) | FOUTVCOPD(1)},*/

 {RZG2L_CPG_BASE + CPG_SIPLL5_CLK3, DIVVAL(PL5_DIVVAL) | FRACIN(PL5_FRACIN)},
 {RZG2L_CPG_BASE + CPG_SIPLL5_CLK4, INTIN(CPG_PL5_INTIN)},
 {RZG2L_CPG_BASE + CPG_SIPLL5_CLK5, SPREAD(CPG_PL5_SPREAD)},
 {RZG2L_CPG_BASE + CPG_SIPLL5_STBY, RESETB | DOWNSPREAD | SSCG_EN},
 {RZG2L_CPG_BASE + CPG_CLKON_MIPI_DSI, CLK_ONEN | CLK_ON},
 {RZG2L_CPG_BASE + CPG_CLKON_LCDC, CLK1_ONWEN | CLK0_ONWEN | CL1_ON | CL0_ON},
 {RZG2L_CPG_BASE + CPG_CLKON_I2C, I2C_CLK_ON | I2C_CLK_ONWEN},
};

#define CPG_RST_MIPI_DSI        0x0868
#define CPG_RST_LCDC            0x086C
#define CPG_RST_I2C             0x0880
#define CPG_OTHERFUNC1_REG      0x0BE8

static const uint32_t cpg_dsi_rst_init[][2] = {
    {RZG2L_CPG_BASE + CPG_RST_MIPI_DSI,   0x00060006},
    {RZG2L_CPG_BASE + CPG_RST_LCDC,       0x00010001},
    {RZG2L_CPG_BASE + CPG_RST_I2C,        0x000f000f},
    {RZG2L_CPG_BASE + CPG_OTHERFUNC1_REG, 0x00010001},
};

#define DSI_DPHY        0x10850000
#define DSIDPHYCTRL0    0x00
#define DSIDPHYTIM0     0x04
#define DSIDPHYTIM1     0x08
#define DSIDPHYTIM2     0x0C
#define DSIDPHYTIM3     0x10
#define DPHYCTRL1       0x40
#define DPHYTRIM0       0x44
static const uint32_t dsi_phy_register_values[][2] = {
    {0x00000000, 1},/* wait us */
    {DSI_DPHY + DSIDPHYTIM0, DSIDPHYTIM0_TCLK_MISS(1) | DSIDPHYTIM0_T_INIT(DSI_T_INIT)},
    {DSI_DPHY + DSIDPHYTIM1, DSIDPHYTIM1_THS_PREPARE(DSI_THS_PREPARE)     |
                                DSIDPHYTIM1_TCLK_PREPARE(DSI_TCLK_SETTLE) |
                                DSIDPHYTIM1_THS_SETTLE(DSI_THS_SETTLE)    |
                                DSIDPHYTIM1_TCLK_SETTLE(DSI_TCLK_SETTLE)},

    {DSI_DPHY  + DSIDPHYTIM2, DSIDPHYTIM2_TCLK_TRAIL(DSI_TCLK_TRAIL) |
                                DSIDPHYTIM2_TCLK_POST(DSI_TCLK_POST) |
                                DSIDPHYTIM2_TCLK_PRE(DSI_TCLK_PRE)   |
                                DSIDPHYTIM2_TCLK_ZERO(DSI_TCLK_ZERO)},

    {DSI_DPHY  + DSIDPHYTIM3, DSIDPHYTIM3_TLPX(DSI_TLPX)             |
                                DSIDPHYTIM3_THS_EXIT(DSI_THS_EXIT)   |
                                DSIDPHYTIM3_THS_TRAIL(DSI_THS_TRAIL) |
                                DSIDPHYTIM3_THS_ZERO(DSI_THS_ZERO)},

    {DSI_DPHY + DPHYCTRL1, 0x00000001},/* DPHYCTRL1 //linux:no this reg */
    {DSI_DPHY + DPHYTRIM0, 0x5A8BBBBB},/* DPHYTRIM0 //linux:no this reg */
    {DSI_DPHY + DSIDPHYCTRL0, 0x00010105},
    {0x00000000, 20},// wait us
    {DSI_DPHY + DSIDPHYCTRL0, 0x00010107},
    {0x00000000, 10},// wait us
    {DSI_DPHY + DSIDPHYCTRL0, 0x00010001},
};

#define ISRR                    0x0000

#define LINKSR                  0x0010
#define HSBUSY                  (0x1 << 12)
#define VICHRUN1                (0x1 << 8)

#define TXSETR                  0x0100
#define DSI_LANE(n)            (n << 0)
#define NUM_LANE_1              0
#define NUM_LANE_2              1
#define NUM_LANE_3              2
#define NUM_LANE_4              3
#define LANE_CLK_EN             (1 << 8)
#define LANE_DATA_EN            (1 << 9)
#define NUM_LANE_CA             (3 << 16) /* fixed value */

#define ULPSSETR                0x0108
#define ULPSWKUP(n)             (n << 0)

#define ULPSCR                  0x010C
#define RSTCR                   0x0110
#define RSTSR                   0x0114

#define DSISETR                 0x0120
#define MRPSZ(n)                (n << 0)
#define ECCEN(n)                (n << 16)
#define CRCEN(n)                (n << 20)
#define EOTPEN(n)               (n << 31)


#define RXBUFSZR                0x0124
#define TXPPD0R                 0x0160
#define TXPPD1R                 0x0164
#define TXPPD2R                 0x0168
#define TXPPD3R                 0x016C
#define RXSR                    0x0200
#define RXSCR                   0x0204
#define PRESPTOBTASETR          0x0210
#define PRESPTOLPSETR           0x0214
#define PRESPTOHSSETR           0x0218
#define AKEPLATIR               0x0220
#define AKEPACMSR               0x0224
#define AKEPSCR                 0x0228
#define RXRSSR                  0x0230
#define RXRSSCR                 0x0234
#define RXRINFOOWSR             0x0238
#define RXRINFOOWSCR            0x023C
#define RXRSS0R                 0x0240
#define RXRSS1R                 0x0244
#define RXRSS2R                 0x0248
#define RXRSS3R                 0x024c
#define RXPPD0R                 0x02C0
#define RXPPD1R                 0x02C4
#define RXPPD2R                 0x02C8
#define RXPPD3R                 0x02CC

static const uint32_t dsi_link_register_values[][2] = {
    /* {0x00000000,1000},// wait us */

    /* {RZG2L_DSI_LINK_BASE + LINKSR, 0x00001100}, */

    {RZG2L_DSI_LINK_BASE + TXSETR, NUM_LANE_CA |
                                   LANE_DATA_EN |
                                   LANE_CLK_EN |
                                   DSI_LANE(NUM_LANE_4)},

    {RZG2L_DSI_LINK_BASE + ULPSSETR, ULPSWKUP(0xA0)},
    {RZG2L_DSI_LINK_BASE + DSISETR, EOTPEN(1) | CRCEN(0x0F) | ECCEN(1) | MRPSZ(1)},

    /* {0x10860310,0x000000FF},//OPMSWTMR  //doc: no defination */

    {RZG2L_DSI_LINK_BASE + CLSTPTSETR, CLSTPTSETR_CLKKPT(DSI_CLKKPT)    |
                                        CLSTPTSETR_CLKBFHT(DSI_CLKBFHT) |
                                        CLSTPTSETR_CLKSTPT(DSI_CLKSTPT)},

    {RZG2L_DSI_LINK_BASE + LPTRNSTSETR, LPTRNSTSETR_GOLPBKT(DSI_GOLPBKT)},
    /* {RZG2L_DSI_LINK_BASE + PLSR, 0x040000F1},        */
    /* {RZG2L_DSI_LINK_BASE + SQCH0SET0R,0x00800000},   */
    /* {RZG2L_DSI_LINK_BASE + SQCH0SET1R,0x08000000},   */
    /* {RZG2L_DSI_LINK_BASE + SQCH1SET0R,0x00800000},   */
    /* {RZG2L_DSI_LINK_BASE + SQCH1SET1R,0x08300000},   */
    {RZG2L_DSI_LINK_BASE + RXIER,       0x17F7FF01},
    {RZG2L_DSI_LINK_BASE + FERRIER,     0x001F0007},
    {RZG2L_DSI_LINK_BASE + PLIER,       0x3F003000},
    {RZG2L_DSI_LINK_BASE + VICH1IER,    0x7CF0000B},
    {RZG2L_DSI_LINK_BASE + SQCH0IER,    0x7D490110},
    {RZG2L_DSI_LINK_BASE + SQCH1IER,    0x7D490110},
    {RZG2L_DSI_LINK_BASE + HSCLKSETR,   0x00000003},
    {RZG2L_DSI_LINK_BASE + VICH1PPSETR, 0x003E8000},

    {RZG2L_DSI_LINK_BASE + VICH1VSSETR, VICH1VSSETR_VACTIVE(LCD_VACTIVE) |
                                        VICH1VSSETR_VSA(LCD_VSYNC)},

     /* {RZG2L_DSI_LINK_BASE + VICH1VSSETR, (LCD_VSYNC << 0)  |
      *                                     (LCD_VSPOL << 15) |
      *                                     (LCD_VACTIVE << 16)},
      */

    {RZG2L_DSI_LINK_BASE + VICH1VPSETR, VICH1VPSETR_VFP(LCD_VFRONT) |
                                        VICH1VPSETR_VBP(LCD_VBACK)},

    {RZG2L_DSI_LINK_BASE + VICH1HSSETR, VICH1HSSETR_HACTIVE(LCD_HACTIVE) |
                                        VICH1HSSETR_HSA(LCD_HSYNC)},

    /* {RZG2L_DSI_LINK_BASE + VICH1HSSETR, (LCD_HSYNC << 0)  |
     *                                     (LCD_HSPOL << 15) |
     *                                     (LCD_HACTIVE << 16)},
     */

    {RZG2L_DSI_LINK_BASE + VICH1HPSETR, VICH1HPSETR_HFP(LCD_HFRONT) |
                                        VICH1HPSETR_HBP(LCD_HBACK)},

    {RZG2L_DSI_LINK_BASE + VICH1SET1R, VICH1SET1R_DLY(DSI_DLY)},

    /* last bit is Video-Input Operation Start */
    {RZG2L_DSI_LINK_BASE + VICH1SET0R, HFPNOLP | HBPNOLP | HSANOLP | VSTART},
};
#endif

static const uint32_t du_reg_init[][2] = {
    {RZG2L_DU_BASE + DU_IMR0, 0x00000000},
    {RZG2L_DU_BASE + DU_DITR0, DEMD(DATA_ENABLE) | VSPOL(HIG_ACT) |
                               HSPOL(HIG_ACT)},

    {RZG2L_DU_BASE + DU_DITR1, DU_DITR1_VSA(LCD_VSYNC) | DU_DITR1_VACTIVE(LCD_VACTIVE)},
    {RZG2L_DU_BASE + DU_DITR2, DU_DITR2_VBP(LCD_VBACK) | DU_DITR2_VFP(LCD_VFRONT)},
    {RZG2L_DU_BASE + DU_DITR3, DU_DITR3_HSA(LCD_HSYNC) | DU_DITR3_HACTIVE(LCD_HACTIVE)},
    {RZG2L_DU_BASE + DU_DITR4, DU_DITR4_HBP(LCD_HBACK) | DU_DITR4_HFP(LCD_HFRONT)},
    {RZG2L_DU_BASE + DU_DITR5, 0x00000000},
    {RZG2L_DU_BASE + DU_MCR1,  PB_AUTOCLR | OPMD(OPMD0)},
    {RZG2L_DU_BASE + DU_PBCR0, DU_PBCR0_PB_DEP(0x1F)},
    {RZG2L_DU_BASE + DU_PBCR1, PB_RUFOP},
    {RZG2L_DU_BASE + DU_PBCR2, PB_RUFDAT(0x00ff00ff)},
};



static const uint32_t vcpd_register_values[][2] = {//0x10870000  //still picture output
#ifdef DSI_PANEL
    {RZG2L_VCPD_BASE + VI6_CMD0, 0x00000000},
    {RZG2L_VCPD_BASE + VI6_CLK_CTRL0, 0x10010F1F},
    {RZG2L_VCPD_BASE + VI6_CLK_CTRL1, 0xFF10FFFF},
    {RZG2L_VCPD_BASE + VI6_CLK_DCSWT, 0x10870018, 0x00000808},
    {RZG2L_VCPD_BASE + VI6_SRESET, 0x00000001},
    /* {RZG2L_VCPD_BASE + VI6_STATUS, 0x00000000}, cmp32 = 0x00000000 */
    {RZG2L_VCPD_BASE + VI6_WPF0_IRQ_ENB, 0x00000000},
    /* {RZG2L_VCPD_BASE + VI6_WPF0_IRQ_ENB, 0x00011003}, */
//	{0x1087004C, 0x00000000}, // VI6_WPF0_IRQ_STA: cmp32 = 0x00000000
    {0x10870100, 0x00000007}, // VI6_DL_CTRL
    {0x10870114, 0x00000000}, // VI6_DL_SWAP0
    {0x10870120, 0x00000002}, // VI6_DL_BODY_SIZE0
//	{0x10870130, 0x00000000}, 				// VI6_DL_HDR_REF_ADDR0: cmp32 = 0x00000000
    {0x10870158, 0x00000000}, // VI6_DL_WUPCNT0
    {0x10870300, (LCD_VACTIVE << 0) | (LCD_HACTIVE << 16)}, // VI6_RPF_SRC_BSIZE: BVSIZE, BHSIZE
    {0x10870304, (LCD_VACTIVE << 0) | (LCD_HACTIVE << 16)}, // VI6_RPF_SRC_DSIZE: EVSIZE, EHSIZE
    {0x10870308, (LCD_RDFMT << 0) | (LCD_RDCSC << 8) | (LCD_VIR<<28)}, // VI6_RPF_INFMT: RDFMT, CSC
//	{0x10870308, 0x00000018}, // VI6_RPF_INFMT: RDFMT, CSC
    {0x10870310, 0x00000000}, // VI6_RPF_LOC
    {0x1087030C, 0x00000F0F}, // VI6_RPF_DSWAP
    {0x10870318, 0xFF0000FF}, // VI6_RPFn_VRTCOL_SET
    {0x10870334, ((LCD_HACTIVE * 1) << 0) | ((LCD_HACTIVE * 3) << 16)}, // VI6_RPF_SRCM_STRIDE: PICT_STRD_C, PICT_STRD_Y
    {0x1087033C, LCD_SRCM_ADDR}, // VI6_RPF_SRCM_ADDR_Y: SRCM_ADDR_Y=DDR Address
    {0x10871000, 0x00000002}, // VI6_WPF0_SRCRPF: RPF0 is master
//	{0x1087100C, 0xFF000018},// VI6_WPF0_OUTFMT: WRFMT, CSC, ODE
    {0x1087100C, (LCD_WRFMT << 0) | (LCD_WRCSC << 8) | (LCD_ODE << 22) | (0xFF << 24)},// VI6_WPF0_OUTFMT: WRFMT, CSC, ODE
    {0x10871010, 0x0000000F}, // VI6_WPF_DSWAP
    {0x10872000, (56 << 0) }, // VI6_DPR_RPF0_ROUTE: RT_RPF0=WPF0
    {0x10872004, (0x3F << 0)},// VI6_DPR_RPF1_ROUTE: RT_RPF1=UNUSED
    {0x10872014, 0x00000500}, // VI6_DPR_WPF0_FPORCH
    {0x10872050, (0x3F << 0) | (0 << 8) | (1 << 28)}, // VI6_DPR_ILV_BRS_ROUTE: RT=UNUSED, FP=0, IIFSEL=BRS
    {0x10873B00, (1 << 0) | (1 << 1) | (LCD_CFMT << 4) | (1500 << 16)}, // VI6_LIF0_CTRL: LIF_EN=1, REQSEL=1, CFMT, OBTH=1500
    {0x10873B04, 0x00000000 | (1500 << 16)}, // VI6_LIF0_CSBTH
    {0x10873B0C, (1536 << 16) | (1 << 31)}, // VI6_LIF0_LBA: LBA1=1536, LBA0=1
    {0x10873900, 0x00000000}, // VI6_BRS_INCTRL
    {0x10873910, (0x0 << 0) | (0x0 << 4) | (0x1 << 16) | (0x0 << 20) | (1 << 31)},  // VI6_BRSA_CTRL
    {0x10873914, (0x00 << 0) | (0x00 << 8) | (0x4 << 16) | (0x4 << 20) | (0 << 23) | (0x3 << 24) | (0x2 << 28) | (0 << 31)}, // VI6_BRSA_BLD
    {0x10873918, (0x0 << 0) | (0x0 << 4) | (0 << 31)}, // VI6_BRSB_CTRL
    {0x10870104, 0x00000000}, // VI6_DL_HDR_ADDR0
#else
    // 0x10870000  //still picture output
    {0x10870000, 0x00000000}, // VI6_CMD0
    {0x10870000, 0x00000000}, // VI6_CLK_CTRL0
    {0x10870000, 0x00000000}, // VI6_CLK_CTRL1
    {0x10870018, 0x00000808}, // VI6_CLK_DCSWT
    {0x1087001C, 0x00000000}, // VI6_CLK_DCSM0
    {0x10870020, 0x00000000}, // VI6_CLK_DCSM1
    {0x10870028, 0x00000001}, // VI6_SRESET
                              //    {0x10870038, 0x00000000}, // VI6_STATUS: cmp32 = 0x00000000
    {0x10870048, 0x00000000}, // VI6_WPF0_IRQ_ENB
    // {0x10870048, 0x00011003}, // VI6_WPF0_IRQ_ENB
    //  {0x1087004C, 0x00000000}, // VI6_WPF0_IRQ_STA: cmp32 = 0x00000000
    {0x10870100, 0x00000007},                                                                                                // VI6_DL_CTRL
    {0x10870114, 0x00000000},                                                                                                // VI6_DL_SWAP0
    {0x10870120, 0x00000002},                                                                                                // VI6_DL_BODY_SIZE0
                                                                                                                             // {0x10870130, 0x00000000},               // VI6_DL_HDR_REF_ADDR0: cmp32 = 0x00000000
    {0x10870158, 0x00000000},                                                                                                // VI6_DL_WUPCNT0
    {0x10870300, (LCD_VACTIVE << 0) | (LCD_HACTIVE << 16)},                                                                  // VI6_RPF_SRC_BSIZE: BVSIZE, BHSIZE
    {0x10870304, (LCD_VACTIVE << 0) | (LCD_HACTIVE << 16)},                                                                  // VI6_RPF_SRC_DSIZE: EVSIZE, EHSIZE
    {0x10870308, (LCD_RDFMT << 0) | (LCD_RDCSC << 8) | (LCD_VIR << 28)},                                                     // VI6_RPF_INFMT: RDFMT, CSC
                                                                                                                             // {0x10870308, 0x00000018}, // VI6_RPF_INFMT: RDFMT, CSC
    {0x10870310, 0x00000000},                                                                                                // VI6_RPF_LOC
    {0x1087030C, 0x00000F0F},                                                                                                // VI6_RPF_DSWAP
    {0x10870318, 0xFF0000FF},                                                                                                // VI6_RPFn_VRTCOL_SET
    {0x10870334, ((LCD_HACTIVE * 1) << 0) | ((LCD_HACTIVE * 3) << 16)},                                                      // VI6_RPF_SRCM_STRIDE: PICT_STRD_C, PICT_STRD_Y
    {0x1087033C, LCD_SRCM_ADDR},                                                                                             // VI6_RPF_SRCM_ADDR_Y: SRCM_ADDR_Y=DDR Address
    {0x10871000, 0x00000002},                                                                                                // VI6_WPF0_SRCRPF: RPF0 is master
                                                                                                                             // {0x1087100C, 0xFF000018},// VI6_WPF0_OUTFMT: WRFMT, CSC, ODE
    {0x1087100C, (LCD_WRFMT << 0) | (LCD_WRCSC << 8) | (LCD_ODE << 22) | (0xFF << 24)},                                      // VI6_WPF0_OUTFMT: WRFMT, CSC, ODE
    {0x10871010, 0x0000000F},                                                                                                // VI6_WPF_DSWAP
    {0x10872000, (56 << 0)},                                                                                                 // VI6_DPR_RPF0_ROUTE: RT_RPF0=WPF0
    {0x10872004, (0x3F << 0)},                                                                                               // VI6_DPR_RPF1_ROUTE: RT_RPF1=UNUSED
    {0x10872014, 0x00000500},                                                                                                // VI6_DPR_WPF0_FPORCH
    {0x10872050, (0x3F << 0) | (0 << 8) | (1 << 28)},                                                                        // VI6_DPR_ILV_BRS_ROUTE: RT=UNUSED, FP=0, IIFSEL=BRS
    {0x10873B00, (1 << 0) | (1 << 1) | (LCD_CFMT << 4) | (1500 << 16)},                                                      // VI6_LIF0_CTRL: LIF_EN=1, REQSEL=1, CFMT, OBTH=1500
    {0x10873B04, 0x00000000 | (1500 << 16)},                                                                                 // VI6_LIF0_CSBTH
    {0x10873B0C, (1536 << 16) | (1 << 31)},                                                                                  // VI6_LIF0_LBA: LBA1=1536, LBA0=1
    {0x10873900, 0x00000000},                                                                                                // VI6_BRS_INCTRL
    {0x10873910, (0x0 << 0) | (0x0 << 4) | (0x1 << 16) | (0x0 << 20) | (1 << 31)},                                           // VI6_BRSA_CTRL
    {0x10873914, (0x00 << 0) | (0x00 << 8) | (0x4 << 16) | (0x4 << 20) | (0 << 23) | (0x3 << 24) | (0x2 << 28) | (0 << 31)}, // VI6_BRSA_BLD
    {0x10873918, (0x0 << 0) | (0x0 << 4) | (0 << 31)},                                                                       // VI6_BRSB_CTRL
    {0x10870104, 0x00000000},
#endif

};


/* Write Read Register */
#define reg_write(addr,val)     writel((val),((uint32_t)addr))
#define reg_read(addr)          readl(addr)

/* Write Block */
static void rzg2l_registers_set(const uint32_t (*arr)[2], uint32_t len)
{
    int i;

    for(i = 0; i < len; i++) {
        if (arr[i][0] == 0) udelay(arr[i][1]);
        else                writel(arr[i][1],((uint32_t)(arr[i][0])));
    }
}

static void rzg2l_dpi_pin_init(void)
{
    int ret;
    printf("%s: start\r\n", __func__);

    // Set LCD PW, BL PIN as output
    rzg2l_set_gpio(0x10 + 3, 1, 0, 2);
    rzg2l_set_gpio(0x10 + 4, 1, 0, 2);
    rzg2l_set_gpio(0x10 + 41, 1, 0, 2);
    rzg2l_set_gpio(0x10 + 45, 2, 0, 2);

    rzg2l_set_gpio(0x10 + 7, 2, 1, 2);
    rzg2l_set_gpio(0x10 + 8, 0, 1, 2);
    rzg2l_set_gpio(0x10 + 8, 1, 1, 2);
    rzg2l_set_gpio(0x10 + 8, 2, 1, 2);
    rzg2l_set_gpio(0x10 + 9, 0, 1, 2);
    rzg2l_set_gpio(0x10 + 9, 1, 1, 2);
    rzg2l_set_gpio(0x10 + 10, 0, 1, 2);
    rzg2l_set_gpio(0x10 + 10, 1, 1, 2);
    rzg2l_set_gpio(0x10 + 11, 0, 1, 2);
    rzg2l_set_gpio(0x10 + 11, 1, 1, 2);
    rzg2l_set_gpio(0x10 + 12, 0, 1, 2);
    rzg2l_set_gpio(0x10 + 12, 1, 1, 2);
    rzg2l_set_gpio(0x10 + 13, 0, 1, 2);
    rzg2l_set_gpio(0x10 + 13, 1, 1, 2);
    rzg2l_set_gpio(0x10 + 13, 2, 1, 2);
    rzg2l_set_gpio(0x10 + 14, 0, 1, 2);
    rzg2l_set_gpio(0x10 + 14, 1, 1, 2);
    rzg2l_set_gpio(0x10 + 15, 0, 1, 2);
    rzg2l_set_gpio(0x10 + 15, 1, 1, 2);
    rzg2l_set_gpio(0x10 + 16, 0, 1, 2);
    rzg2l_set_gpio(0x10 + 16, 1, 1, 2);
    rzg2l_set_gpio(0x10 + 17, 0, 1, 2);
    rzg2l_set_gpio(0x10 + 17, 1, 1, 2);
    rzg2l_set_gpio(0x10 + 17, 2, 1, 2);

    rzg2l_set_gpio(0x10 + 6, 1, 1,2);
    rzg2l_set_gpio(0x10 + 7, 0, 1,2);
    rzg2l_set_gpio(0x10 + 7, 1, 1,2);
    rzg2l_set_gpio(0x10 + 6, 0, 1,0);

    // LCD PW, BL ON
    ret = run_command("gpio set 7",0);
    ret = run_command("gpio set 9",0);
    ret = run_command("gpio set 92",0);
    ret = run_command("gpio set 108",0);

}

static void rzg2l_dpi_cpg_init(void)
{
        uint32_t clk_mon, clk_mon_lcd, rst_mon_lcd;

        printf("%s: start\r\n", __func__);

        clk_mon = CLK1_MON_CLK_SUPPLIED | CLK0_MON_CLK_SUPPLIED;
        rzg2l_registers_set(cpg_dpi_reg_init, ARRAY_SIZE(cpg_dpi_reg_init));

        /* CPG_CLKMON_LCDC, wait clock on */
        clk_mon_lcd = reg_read(RZG2L_CPG_BASE + CPG_CLKMON_LCDC) & clk_mon;
        while (clk_mon_lcd != clk_mon);

        /* LCD reset & SSCG CLK */
        rzg2l_registers_set(cpg_dpi_rst_init, ARRAY_SIZE(cpg_dpi_rst_init));

        /* wait reset on */
        rst_mon_lcd = (reg_read(RZG2L_CPG_BASE + CPG_RSTMON_LCDC) & RST0_MON);
        while (rst_mon_lcd & RST0_MON);
}

#ifdef DSI_PANEL
static void rzg2l_cpg_dsi_rst_init(void)
{
    rzg2l_registers_set(cpg_dsi_init, ARRAY_SIZE(cpg_dsi_init));

    // wait clock on
    while ((reg_read(RZG2L_CPG_BASE + CPG_CLKMON_MIPI_DSI) & DSI_CLK_SUPPLY) != DSI_CLK_SUPPLY);
    while ((reg_read(RZG2L_CPG_BASE + CPG_CLKMON_LCDC) & LCD_CLK_SUPPLY) != LCD_CLK_SUPPLY);

    rzg2l_registers_set(cpg_dsi_rst_init, ARRAY_SIZE(cpg_dsi_rst_init));

    // wait reset on
    while ((reg_read(RZG2L_CPG_BASE + CPG_RSTMON_MIPI_DSI) & DSI_RST_SUPPLY));
    while ((reg_read(RZG2L_CPG_BASE + CPG_RSTMON_LCDC) & LCD_RST_SUPPLY));
}
#endif

#ifdef DSI_HDMI_BRIDGE
/* I2C Write Register */
static int adv7535_i2c_reg_write(struct udevice *dev, uint addr, uint mask, uint data)
{
    uint8_t valb;
    int err;

    if (mask != 0xff) {
        err = dm_i2c_read(dev, addr, &valb, 1);
        if (err)
            return err;

        valb &= ~mask;
        valb |= data;
    } else {
        valb = data;
    }

    err = dm_i2c_write(dev, addr, &valb, 1);
    return err;
}

/* I2C Read Register */
static int adv7535_i2c_reg_read(struct udevice *dev, uint8_t addr, uint8_t *data)
{
    uint8_t valb;
    int err;

    err = dm_i2c_read(dev, addr, &valb, 1);
    if (err)
        return err;

    *data = (int)valb;
    return 0;
}

/* ADV7537 Fix Register Set */
static void adv7535_setup_cfg(struct udevice *dev, struct udevice *dev_cec)
{
    /* Initialisation (Fixed) Registers */
    adv7535_i2c_reg_write(dev, 0x16,0xff, 0x20);
    adv7535_i2c_reg_write(dev, 0x9A,0xff, 0xE0);
    adv7535_i2c_reg_write(dev, 0xBA,0xff, 0x70);
    adv7535_i2c_reg_write(dev, 0xDE,0xff, 0x82);
    adv7535_i2c_reg_write(dev, 0xE4,0xff, 0x40);
    adv7535_i2c_reg_write(dev, 0xE5,0xff, 0x80);

    adv7535_i2c_reg_write(dev, 0xE1, 0xff, ADV7535_DSI_CEC_ADDR);       // 设置cec地址  //等于默认值

    adv7535_i2c_reg_write(dev_cec, 0x15, 0xff,0xD0);
    adv7535_i2c_reg_write(dev_cec, 0x17, 0xff,0xD0);
    adv7535_i2c_reg_write(dev_cec, 0x24, 0xff,0x20);
    adv7535_i2c_reg_write(dev_cec, 0x57, 0xff,0x11);
    adv7535_i2c_reg_write(dev_cec, 0x05, 0xff,0xc8);
}

/* ADV7537 Screen Param Register Set */
static void adv7535_vmode_cfg(struct udevice *dev, struct udevice *dev_cec)
{
    uint32_t low, high;
    uint32_t line_length, frame_height;

    line_length = xres + left_margin + right_margin + hsync_len;
    frame_height = yres + upper_margin + lower_margin + vsync_len;

    adv7535_i2c_reg_write(dev_cec, 0x1C, 0xff, 0x40); /* 4 Data Lanes */
    adv7535_i2c_reg_write(dev_cec, 0x16, 0xff, 0x18); /* Pixel Clock  */ // div 3
    adv7535_i2c_reg_write(dev_cec, 0x27, 0xff, 0xCB); /* INT_TIMING_GEN */

    /* video mode settings */
    low = (line_length << 4);
    high = (line_length >> 4);
    adv7535_i2c_reg_write(dev_cec, 0x28, 0xff, high); /* Total Line Length */
    adv7535_i2c_reg_write(dev_cec, 0x29, 0xff, low);

    low = (hsync_len << 4);
    high = (hsync_len >> 4);
    adv7535_i2c_reg_write(dev_cec, 0x2A, 0xff, high); /* Hsync Active Width */
    adv7535_i2c_reg_write(dev_cec, 0x2B, 0xff, low);

    low = (right_margin << 4);
    high = (right_margin >> 4);
    adv7535_i2c_reg_write(dev_cec, 0x2C,0xff, high); /* Horizontal FP Width */
    adv7535_i2c_reg_write(dev_cec, 0x2D,0xff, low);

    low = (left_margin << 4);
    high = (left_margin >> 4);
    adv7535_i2c_reg_write(dev_cec, 0x2E, 0xff, high); /* Horizontal BP Width */
    adv7535_i2c_reg_write(dev_cec, 0x2F, 0xff, low);

    low = (frame_height << 4);
    high = (frame_height >> 4);
    adv7535_i2c_reg_write(dev_cec, 0x30, 0xff, high); /* Total Frame Height */
    adv7535_i2c_reg_write(dev_cec, 0x31, 0xff, low);

    low = (vsync_len << 4);
    high = (vsync_len >> 4);
    adv7535_i2c_reg_write(dev_cec, 0x32,0xff, high); /* Vsync Active Height */
    adv7535_i2c_reg_write(dev_cec, 0x33,0xff, low);

    low = (lower_margin << 4);
    high = (lower_margin >> 4);
    adv7535_i2c_reg_write(dev_cec, 0x34,0xff, high); /* Vertical FP Height */
    adv7535_i2c_reg_write(dev_cec, 0x35,0xff, low);

    low = (upper_margin << 4);
    high = (upper_margin >> 4);
    adv7535_i2c_reg_write(dev_cec, 0x36, 0xff, high); /* Vertical BP Height */
    adv7535_i2c_reg_write(dev_cec, 0x37, 0xff, low);

    /* Reset Internal Timing Generator */
    adv7535_i2c_reg_write(dev_cec, 0x27, 0xff, 0xCB);
    adv7535_i2c_reg_write(dev_cec, 0x27, 0xff, 0x8B);
    adv7535_i2c_reg_write(dev_cec, 0x27, 0xff, 0xCB);        // 重置时序

    adv7535_i2c_reg_write(dev_cec, 0x03, 0xff, 0x89);        // HDMI 输出使能
    adv7535_i2c_reg_write(dev_cec, 0x55, 0xff, 0x00);        // 关闭测试图案

    adv7535_i2c_reg_write(dev, 0xAF, 0xff, 0x16); /* HDMI Output */ // 设置为HDMI 输出
    adv7535_i2c_reg_write(dev, 0x55, 0xff, 0x02); /* AVI Info-frame */
    adv7535_i2c_reg_write(dev, 0x56, 0xff, 0x28); /* 16:9 */
    adv7535_i2c_reg_write(dev, 0x40, 0xff, 0x00); /* GCP Enable */
    adv7535_i2c_reg_write(dev, 0x4C, 0xff, 0x00); /* 24bpp */

    adv7535_i2c_reg_write(dev, 0x49, 0xff, 0x00); // 8 deth

    /* low refresh rate */
    if (refresh < 50)
        adv7535_i2c_reg_write(dev, 0x4A, 0xff, 0x8C);

    /*TODO Audio Setup */
    adv7535_i2c_reg_write(dev_cec, 0xBE, 0xff, 0x3D); /* CEC Power Mode */

    /* Power */
    adv7535_i2c_reg_write(dev, 0x41, 0xff, 0x10);
    adv7535_i2c_reg_write(dev, 0xd6, 0xff, 0xC8);

    adv7535_i2c_reg_write(dev, 0xAF, 0xff, 0x06); /* HDMI Output */ // 设置为HDMI 输出

    adv7535_i2c_reg_write(dev_cec, 0x11, 0xff, 0x09);
    adv7535_i2c_reg_write(dev_cec, 0x81, 0xff, 0x06);//CEC_TX_RETRY

    adv7535_i2c_reg_write(dev_cec, 0x03, 0xff, 0x89);
    adv7535_i2c_reg_write(dev_cec, 0x55, 0xff, 0x00);

    adv7535_i2c_reg_write(dev_cec, 0x05, 0xff, 0xC8);

    adv7535_i2c_reg_write(dev, 0xE2,0xff, 0x00);
    adv7535_i2c_reg_write(dev, 0xD6,0xff, 0x48);
    adv7535_i2c_reg_write(dev, 0xC9,0xff, 0x13);
}

static void adv7535_power_off(struct udevice *dev, struct udevice *dev_cec)
{
    adv7535_i2c_reg_write(dev, 0x50, 0xff, 0x10);
}

/* Init ADV7535 */
static void adv7535_init(void)
{
    struct udevice *bus = NULL, *dev = NULL, *dev_cec = NULL;
    uint8_t val = 0;

    /* Get I2C Bus */
    if (uclass_get_device_by_name(UCLASS_I2C, "i2c@10058400", &bus)) {
        puts("Cannot find MCU bus! Can not disable MCU WDT.\n");
        return;
    }

    /* Get CEC Device */
    val = i2c_get_chip(bus, 0x3C, 1, &dev_cec);
    if (val) {
        puts("Cannot get 0x3C!\n");
        return;
    }

    /* Get Device */
    val = i2c_get_chip(bus, 0x3D, 1, &dev);
    if (val) {
        puts("Cannot get 0x3D!\n");
        return;
    }

    adv7535_setup_cfg(dev, dev_cec);
    adv7535_vmode_cfg(dev, dev_cec);
}

/* Init ADV7535 */
static void adv7535_deinit(void)
{
    struct udevice *bus = NULL, *dev = NULL, *dev_cec = NULL;
    uint8_t val = 0;

    /* Get I2C Bus */
    if (uclass_get_device_by_name(UCLASS_I2C, "i2c@10058400", &bus)) {
        puts("Cannot find MCU bus! Can not disable MCU WDT.\n");
        return;
    }

    /* Get CEC Device */
    val = i2c_get_chip(bus, 0x3C, 1, &dev_cec);
    if (val) {
        puts("Cannot get 0x3C!\n");
        return;
    }

    /* Get Device */
    val = i2c_get_chip(bus, 0x3D, 1, &dev);
    if (val) {
        puts("Cannot get 0x3D!\n");
        return;
    }

    adv7535_power_off(dev, dev_cec);
}
#endif

#ifdef DSI_PANEL
/* Init DSI PHY */
static void rzg2l_dsi_phy_init(void)
{
    rzg2l_registers_set(dsi_phy_register_values,ARRAY_SIZE(dsi_phy_register_values));

    // CPG_RSTMON_MIPI_DSI: MIPI_DSI_CMN_RSTB=1, MIPI_DSI_ARESET_N=1, MIPI_DSI_PRESET_N=1
    while ((reg_read(RZG2L_CPG_BASE + 0x09E8) & 0x00000007) != 0x00000000);
}

/* Init DSI Link */
static void rzg2l_dsi_link_init(void)
{
    rzg2l_registers_set(dsi_link_register_values,ARRAY_SIZE(dsi_link_register_values));
    while ((reg_read(DSI_LINK_base_addr + VICH1SR) & 0x00000008) != 0x00000008);
}
#endif // DSI_PANEL

/* Init FCPVD */
static void rzg2l_fpvcg_init(void)
{
    printf("FCPVD Init\r\n");
    rzg2l_registers_set(fcpvd_register_values, ARRAY_SIZE(fcpvd_register_values));
}

/* Init DU */
static void rzg2l_du_init(void)
{
    printf("DU Init\r\n");
    rzg2l_registers_set(du_reg_init, ARRAY_SIZE(du_reg_init));
}

/* Init VPCD */
static void rzg2l_vcpd_init(void)
{
    printf("VCPD Init\r\n");
    rzg2l_registers_set(vcpd_register_values,ARRAY_SIZE(vcpd_register_values));
}


/* Start LCDC */
static void rzg2l_lcdc_start(void)
{
    /* Disable all due to auto start at rzg2l video init */
    printf("%s: start\r\n", __func__);
#if 1
    /*----- Start Video Output ----- */

    //VI6_CMD0: STRCMD=1
    reg_write(VSPD_base_addr + 0x0000, 0x00000001);

    //Wait until VI6_DISP0_IRQ_STA.DST=1
    while ((reg_read(VSPD_base_addr + 0x007C) & 0x00000100) != 0x00000100);

    //DU_MCR0 : DI_EN=1
    reg_write(DU_base_addr + 0x0000, 0x00000100);

    //ISR.VIN1
    //while ((reg_read(DSI_LINK_base_addr + ISR) & 0x00000100) != 0x00000100);
#endif
}

/* Stop LCDC */
static void rzg2l_lcdc_stop(void)
{
    printf("%s: start\r\n", __func__);

}

int rzg2l_video_init(void)
{
    printf("%s: start\r\n", __func__);
    /* adv7535_deinit(); */
    /* rzg2l_cpg_dsi_rst_init(); */

    /* Pinmux setup for DPI panel */
    rzg2l_dpi_pin_init();

    /* CPG setup for DPI panel */
    rzg2l_dpi_cpg_init();

    /* Initialize display buffer */
    set_white_screen_display(g_framebuffer, 0xff);

    rzg2l_du_init();
    rzg2l_vcpd_init();
    /* rzg2l_fpvcg_init(); */
    rzg2l_lcdc_start();

    return 0;
}


void do_rzg2l_video_start(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
    return rzg2l_lcdc_start();
}

void do_rzg2l_video_stop(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
    return rzg2l_lcdc_stop();
}

int do_rzg2l_video_load_img(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
    dcache_disable();
    return run_command("fatload mmc 1:1 0x58000000 boot_img.bin",0);
}

U_BOOT_CMD(
    rzg2l_video_start, 1, 0, do_rzg2l_video_start,
    "Start Rzg2l",
    ""
);

U_BOOT_CMD(
    rzg2l_video_stop, 1, 0, do_rzg2l_video_stop,
    "Stop Rzg2l LCDC",
    ""
);

U_BOOT_CMD(
    rzg2l_video_load_img, 1, 0, do_rzg2l_video_load_img,
    "Load Rzg2l",
    ""
);
