#ifndef __DSI_DEFINE__
#define __DSI_DEFINE__

/* Link/Phy CLK M4φ */
/*
 00b: 1/16  (16.656 MHz)
 01b: 1/32  (8.328  MHz)
 10b: 1/64  (4.164  MHz)
 11b: 1/128 (2.082  MHz)
 */
#define DSI_LPCLK_M4PHI_VAL (0 << 12)

/* 1080p 148.5MHz */
#define PL5_REFDIV      2
#define PL5_INTIN       148
#define PL5_FRACIN      8388608
#define PL5_POSTDIV1    1
#define PL5_POSTDIV2    1
#define PL5_SSC_EN      1
#define PL5_DOWNSPREAD  1
#define PL5_DIVVAL      0
#define PL5_SPREAD      0x16

#define DSI_DIV_A       1
#define DSI_DIV_B       2

//DPHY 250M
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

//DSI-LINK
#define DSI_CLKKPT          12
#define DSI_CLKBFHT         15
#define DSI_CLKSTPT         48
#define DSI_GOLPBKT         75
#define DSI_DLY             212
#define DSI_TXESYNC         0
#define DSI_VIDT            0x3E
#define DSI_VIVC            0
#define DSI_HSANOLP         0
#define DSI_HBPNOLP         0
#define DSI_HFPNOLP         0
#define DSI_VSEN            0

// Define For ADV7535
#define refresh 60         // Frame rate
#define xres 1920          // resolution x
#define yres 1080          // resolution y
#define left_margin 148    // Hback
#define right_margin 88    // Hfront
#define upper_margin 36    // Vback
#define lower_margin 4     // Vfront
#define hsync_len 44       // Hsync
#define vsync_len 5        // Vsync

//DU
#define LCD_H         1920   // resolution x
#define LCD_V         1080   // resolution y
#define LCD_HFRONT          88     // Hfront
#define LCD_HSYNC           44     // Hsync
#define LCD_HBACK           148    // Hback
#define LCD_VFRONT          4      // Vfront
#define LCD_VSYNC           5      // Vsync
#define LCD_VBACK           36     // Vback
#define LCD_VSPOL           1      // vsync polarity
#define LCD_HSPOL           1      // hsync polarity
#define LCD_DEMD            0x3

// .name = "1920x1080",
// .status = 0,
// .type = ((1<<6)),
// .clock = (148500),
// .hdisplay = (1920),
// .hsync_start = (2008),
// .hsync_end = (2052),
// .htotal = (2200),
// .hskew = (0),
// .vdisplay = (1080),
// .vsync_start = (1084),
// .vsync_end = (1089),
// .vtotal = (1125),
// .vscan = (0),
// .flags = ((1<<1) | (1<<3)),
// .base.type = 0xdededede

//VSPD
#define LCD_BPP             24
#define LCD_VIR             0         //Virtual Input Enable
#define LCD_RDFMT           0x18
#define LCD_RDCSC           0         //Color Space Conversion Enable
#define LCD_SRCM_ADDR       0x58000000
#define LCD_WRFMT           0x18
#define LCD_WRCSC           0
#define LCD_ODE             0
#define LCD_CFMT            0

#endif //!__DSI_DEFINE__

