#ifndef __DSI_DEFINE__
#define __DSI_DEFINE__

//PLL2.FOUT3=533.333MHz, lpcalk=533.333/2/16=16.666MHz
#define CPG_LPCLK_DIV       0 // CPG_PL2_DDIV

/*
 * Ref.
 *     HW_Man 7. Clock Pulse Generator (CPG) page 266
 *
 * PLL5.FOUTPOSTDIV =
 *     24MHz / 2 * (75 + (8808038 / (2 ^ 24))) / 3 / 1 = 302.1MHz
 *
 * SSC = 31.25kHz / 0.5%
 */

 /* hsclk = 302.1MHz / 16 = 18.88125MHz */
#define PL5_REFDIV      2          /* CLK1             */
#define PL5_INTIN       74         /* CPG_SIPLL5_CLK4  */
#define PL5_FRACIN      4194304    /* CPG_SIPLL5_CLK3  */
#define PL5_POSTDIV1    1          /* CLK1             */
#define PL5_POSTDIV2    1          /* CLK1             */
#define PL5_SSC_EN      1
#define PL5_DOWNSPREAD  1
#define PL5_DIVVAL      0          /* CPG_SIPLL5_CLK3  */
#define PL5_SPREAD      0x16       /* CPG_SIPLL5_CLK5  */

/* vclk1 = 302.1MHz / 4 / 3 = 25.175MHz */
/* vclk1 = 302.1MHz / 2 / 2 = 75MHz     */
#define CPG_DSI_DIV_A       1
#define CPG_DSI_DIV_B       2

/* DPHY 250M */
#define DSI_T_INIT          79801
#define DSI_TCLK_PREPARE    8
#define DSI_THS_PREPARE     9
#define DSI_TCLK_SETTLE     9
#define DSI_THS_SETTLE      9
#define DSI_TCLK_ZERO       33
#define DSI_TCLK_PRE        4
#define DSI_TCLK_POST       35
#define DSI_TCLK_TRAIL      7
#define DSI_THS_ZERO        16
#define DSI_THS_TRAIL       9
#define DSI_THS_EXIT        13
#define DSI_TLPX            6

/* DSI-LINK */
#define DSI_CLKKPT          7
#define DSI_CLKBFHT         8
#define DSI_CLKSTPT         27
#define DSI_GOLPBKT         40
#define DSI_DLY             212
#define DSI_TXESYNC         0
#define DSI_VIDT            0x3E
#define DSI_VIVC            0
#define DSI_HSANOLP         0
#define DSI_HBPNOLP         0
#define DSI_HFPNOLP         0
#define DSI_VSEN            0

/* DU */
#define LCD_HACTIVE         1280   /* resolution x   */
#define LCD_VACTIVE         720    /* resolution y   */
#define LCD_HFRONT          110    /* Hfront         */
#define LCD_HSYNC           40     /* Hsync          */
#define LCD_HBACK           220    /* Hback          */
#define LCD_VFRONT          5      /* Vfront         */
#define LCD_VSYNC           5      /* Vsync          */
#define LCD_VBACK           20     /* Vback          */
#define LCD_VSPOL           1      /* vsync polarity */
#define LCD_HSPOL           1      /* hsync polarity */
#define LCD_DEMD            0x3

/* Define For ADV7535 */
#define refresh      60
#define xres         1280          /* resolution x */
#define yres         720           /* resolution y */
#define left_margin  220           /* Hback        */
#define right_margin 110           /* Hfront       */
#define upper_margin 20            /* Vback        */
#define lower_margin 5             /* Vfront       */
#define hsync_len    40            /* Hsync        */
#define vsync_len    5             /* Vsync        */

/* VSPD */
#define LCD_BPP             24
#define LCD_VIR             0      /* Virtual Input Enable */
#define LCD_RDFMT           0x18
#define LCD_RDCSC           0      /* Color Space Conversion Enable */
#define LCD_SRCM_ADDR       0x58000000
#define LCD_WRFMT           0x18
#define LCD_WRCSC           0
#define LCD_ODE             0
#define LCD_CFMT            0

#endif //!__DSI_DEFINE__

