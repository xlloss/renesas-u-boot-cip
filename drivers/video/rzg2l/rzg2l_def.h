/*
 * Copyright (c) 2020, Renesas Electronics Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __RZG2L_DEF_H__
#define __RZG2L_DEF_H__

#define RZG2L_SRAM_BASE                 (0x00010000)
#define RZG2L_DEVICE_BASE               (0x10000000)
#define RZG2L_SCIF0_BASE                (0x1004B800)
#define RZG2L_SPIMULT_BASE              (0x10060000)
#define RZG2L_SPIMULT_WBUF_BASE         (0x10070000)
#define RZG2L_SYC_BASE                  (0x11000000)
#define RZG2L_CPG_BASE                  (0x11010000)
#define RZG2L_SYSC_BASE                 (0x11020000)
#define RZG2L_GPIO_BASE                 (0x11030000)
#define RZG2L_TZC_SPI_BASE              (0x11060000)
#define RZG2L_TZC_DDR_BASE              (0x11070000)
#define RZG2L_DDR_PHY_BASE              (0x11400000)
#define RZG2L_DDR_MEMC_BASE             (0x11410000)
#define RZG2L_GIC_BASE                  (0x11900000)
#define RZG2L_SD0_BASE                  (0x11C00000)
#define RZG2L_SPIROM_BASE               (0x20000000)
#define RZG2L_DDR1_BASE                 (0x40000000)
#define RZG2L_DDR2_BASE                 (0x80000000)
#define RZG2L_DDR3_BASE                 (0x100000000)

#define RZG2L_GICD_BASE                 (RZG2L_GIC_BASE)
#define RZG2L_GICR_BASE                 (RZG2L_GIC_BASE + 0x00040000)

#define RZG2L_SRAM_SIZE                 (0x00030000 - RZG2L_SRAM_BASE)
#define RZG2L_DEVICE_SIZE               (0x15000000 - RZG2L_DEVICE_BASE)
#define RZG2L_SPIROM_SIZE               (0x30000000 - RZG2L_SPIROM_BASE)
#define RZG2L_DDR1_SIZE                 (RZG2L_DDR2_BASE - RZG2L_DDR1_BASE)
#define RZG2L_DDR2_SIZE                 (RZG2L_DDR3_BASE - RZG2L_DDR2_BASE)

#define RZG2L_SPIROM_FIP_BASE           (RZG2L_SPIROM_BASE + 0x0001D200)
#define RZG2L_SPIROM_FIP_SIZE           (0x30000000 - RZG2L_SPIROM_FIP_BASE)

#define RZG2L_SYC_INCK_HZ               (24000000)
#define RZG2L_UART_INCK_HZ              (100000000)
#define RZG2L_UART_BARDRATE             (115200)

#define CPG_CLKMON_MIPI_DSI             0x06E8
#define DSI_CLK_SUPPLY                  0x3F

#define CPG_CLKMON_MIPI_DSI             0x06E8
#define LCD_CLK_SUPPLY                  0x03

#define CPG_RSTMON_MIPI_DSI             0x09E8
#define DSI_PRESET_N                    (1 << 2)
#define DSI_ARESET_N                    (1 << 1)
#define DSI_CMN_RSTB                    (1 << 0)
#define DSI_RST_SUPPLY (DSI_PRESET_N | DSI_ARESET_N)

#define CPG_RSTMON_LCDC
#define LCD_RST_SUPPLY                  0x01

/* Boot Info base address */
#define RZG2L_BOOTINFO_BASE             (RZG2L_SRAM_BASE)

/* Base address where parameters to BL31 are stored */
#define PARAMS_BASE                     (RZG2L_SRAM_BASE + 0x0001F000)
#define PARAMS_SIZE                     (0x1000)

#define RIIC1_BASE_ADDR                 0x10058400UL
#define CPG_base_addr                   0x11010000UL
#define DSI_PHY_base_addr               0x10850000UL
#define DSI_LINK_base_addr              0x10860000UL
#define VSPD_base_addr                  0x10870000UL
#define FCPVD_base_addr                 0x10880000UL
#define DU_base_addr                    0x10890000UL
#define USB_Ch1_Host_base_addr          0x11C70000UL

#define CPG_CLKMON_LCDC                 0x06EC
#define CLK1_MON_CLK_SUPPLIED           (1 << 1)
#define CLK0_MON_CLK_SUPPLIED           (1 << 0)

#define CPG_RSTMON_LCDC                 0x09EC
#define RST0_MON                        (1 << 0)

#define CPG_RST_LCDC                    0x086C
#define UNIT0_RST_WEN                   (1 << 16)
#define UNIT0_RST_STOP                  (1 << 0)

#define CPG_OTHERFUNC1_REG              0x0BE8
#define RES0_ON_W_EN                    (1 << 16)
#define RES0_SET                        (1 << 0)

#define CPG_PL1_DDIV                    0x0200

#define CPG_PL5_SDIV                    0x0420
#define DSI_CLK_DIVB_WEN                (1 << 24)
#define DSI_CLK_DIVA_WEN                (1 << 16)
#define DSI_CLK_DIVA_VAL(n)             (n << 0 | DSI_CLK_DIVA_WEN)
#define DSI_CLK_DIVB_VAL(n)             (n << 8 | DSI_CLK_DIVB_WEN)

#define CPG_PL2_DDIV                    0x0204
#define DIV_DSI_LPCLK_WEN               (1 << 28)
#define DSI_LPCLK_SET(DIV_DSI_LPCLK_WEN | DSI_LPCLK_M4PHI_VAL)

#define CPG_SIPLL5_CLK1                 0x0144
#define POSTDIV1(n)                     (n << 0)
#define POSTDIV2(n)                     (n << 4)
#define REFDIV(n)                       (n << 8)

#define CPG_SIPLL5_CLK2                 0x0148
#define FOUTVCOPD_WEN(n)                (n << 24)
#define FOUTVCOPD(n)                    (n << 8)

#define CPG_SIPLL5_CLK3                 0x014C
#define FRACIN(n)                       (n << 8)
#define DIVVAL(n)                       (n << 0)

#define CPG_SIPLL5_CLK4                 0x0150
#define INTIN(n)                        (n << 16)

#define CPG_SIPLL5_CLK5                 0x0154
#define SPREAD(n)                       (n << 0)

#define CPG_SIPLL5_STBY                 0x0140
#define DOWNSPRE_AD_WREN                (1 << 20)
#define DOWNSPREAD                      (DOWNSPRE_AD_WREN | 1 << 4)
#define RESETB_WEN                      (1 << 16)
#define RESETB                          (RESETB_WEN | 1 << 0)
#define SSCG_EN_WEN                     (1 << 18)
#define SSCG_EN                         (SSCG_EN_WEN | 1 << 0)

#define CPG_CLKON_MIPI_DSI              0x0568
#define DSI_PLLCLK_ON                   (1 << 0)
#define DSI_SYSCLK_ON                   (1 << 1)
#define DSI_ACLK_ON                     (1 << 2)
#define DSI_PCLK_ON                     (1 << 3)
#define DSI_VCLK_ON                     (1 << 4)
#define DSI_LPCLK_ON                    (1 << 5)

#define DSI_PLLCLK_ONEN                 (1 << 16)
#define DSI_SYSCLK_ONEN                 (1 << 17)
#define DSI_ACLK_ONEN                   (1 << 18)
#define DSI_PCLK_ONEN                   (1 << 19)
#define DSI_VCLK_ONEN                   (1 << 20)
#define DSI_LPCLK_ONEN                  (1 << 21)

#define CPG_CLKON_LCDC                  0x056C
#define CLK0_ON                         (1 << 0)
#define CLK1_ON                         (1 << 1)
#define CLK0_ONWEN                      (1 << 16)
#define CLK1_ONWEN                      (1 << 17)
#define CLK_EN (CLK0_ON | CLK1_ON | CLK2_ON | CLK3_ON | CLK4_ON | CLK5_ON)
#define CLK_ONEN (CLK0_ONWEN | CLK1_ONWEN | CLK2_ONWEN | CLK3_ONWEN |
CLK4_ONWEN | CLK5_ONWEN)

#define CPG_CLKON_I2C                   0x0580
#define I2C0_PCLK_ON                    (1 << 0)
#define I2C1_PCLK_ON                    (1 << 1)
#define I2C2_PCLK_ON                    (1 << 2)
#define I2C3_PCLK_ON                    (1 << 3)
#define I2C0_PCLK_ONWEN                 (1 << 16)
#define I2C1_PCLK_ONWEN                 (1 << 17)
#define I2C2_PCLK_ONWEN                 (1 << 18)
#define I2C3_PCLK_ONWEN                 (1 << 19)
#define I2C_CLK_ON                      (I2C0_PCLK_ON | I2C1_PCLK_ON |
                                         I2C2_PCLK_ON | I2C3_PCLK_ON)

#define I2C_CLK_ONWEN                   (I2C0_PCLK_ONWEN | I2C1_PCLK_ONWEN |
                                         I2C2_PCLK_ONWEN | I2C3_PCLK_ONWEN)

#define ICCR1                           (RIIC1_BASE_ADDR + 0x000)
#define ICCR2                           (RIIC1_BASE_ADDR + 0x004)
#define ICMR1                           (RIIC1_BASE_ADDR + 0x008)
#define ICMR2                           (RIIC1_BASE_ADDR + 0x00C)
#define ICMR3                           (RIIC1_BASE_ADDR + 0x010)
#define ICFER                           (RIIC1_BASE_ADDR + 0x014)
#define ICSER                           (RIIC1_BASE_ADDR + 0x018)
#define ICIER                           (RIIC1_BASE_ADDR + 0x01C)
#define ICSR1                           (RIIC1_BASE_ADDR + 0x020)
#define ICSR2                           (RIIC1_BASE_ADDR + 0x024)
#define ICSAR0                          (RIIC1_BASE_ADDR + 0x028)
#define ICSAR1                          (RIIC1_BASE_ADDR + 0x02C)
#define ICSAR2                          (RIIC1_BASE_ADDR + 0x030)
#define ICBRL                           (RIIC1_BASE_ADDR + 0x034)
#define ICBRH                           (RIIC1_BASE_ADDR + 0x038)
#define ICDRT                           (RIIC1_BASE_ADDR + 0x03C)
#define ICDRR                           (RIIC1_BASE_ADDR + 0x040)

#endif /* __RZG2L_DEF_H__ */
