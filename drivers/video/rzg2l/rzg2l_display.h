/*
* Copyright (c) 2020 - 2024 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
* File Name    : rzg2l_display.h
* Version      : 1.00
* Description  : RZG2L setting for DU and VCPD unit.
*/

#ifndef RZG2L_DISPLAY_H
#define RZG2L_DISPLAY_H

/** Display frame number */
typedef enum e_display_frame_layer
{
    DISPLAY_FRAME_LAYER_1 = 0,         ///< Frame layer 1
    DISPLAY_FRAME_LAYER_2 = 1          ///< Frame layer 2
} display_frame_layer_t;

/** Display interface operation state */
typedef enum e_display_state
{
    DISPLAY_STATE_CLOSED     = 0,      ///< Display closed
    DISPLAY_STATE_OPENED     = 1,      ///< Display opened
    DISPLAY_STATE_DISPLAYING = 2       ///< Displaying
} display_state_t;

/** Input format setting */
typedef enum e_display_in_format
{
    DISPLAY_IN_FORMAT_32BITS_ARGB8888 = 0, ///< ARGB8888, 32 bits
    DISPLAY_IN_FORMAT_32BITS_RGB888   = 1, ///< RGB888,   32 bits
    DISPLAY_IN_FORMAT_16BITS_RGB565   = 2, ///< RGB565,   16 bits
    DISPLAY_IN_FORMAT_16BITS_ARGB1555 = 3, ///< ARGB1555, 16 bits
    DISPLAY_IN_FORMAT_16BITS_ARGB4444 = 4, ///< ARGB4444, 16 bits
    DISPLAY_IN_FORMAT_CLUT8           = 5, ///< CLUT8
    DISPLAY_IN_FORMAT_CLUT4           = 6, ///< CLUT4
    DISPLAY_IN_FORMAT_CLUT1           = 7, ///< CLUT1
} display_in_format_t;

/** Output format setting */
typedef enum e_display_out_format
{
    DISPLAY_OUT_FORMAT_24BITS_RGB888 = 0, ///< RGB888, 24 bits
    DISPLAY_OUT_FORMAT_18BITS_RGB666 = 1, ///< RGB666, 18 bits
    DISPLAY_OUT_FORMAT_16BITS_RGB565 = 2, ///< RGB565, 16 bits
    DISPLAY_OUT_FORMAT_8BITS_SERIAL  = 3, ///< SERIAL, 8 bits
} display_out_format_t;

/** Polarity of a signal select */
typedef enum e_display_signal_polarity
{
    DISPLAY_SIGNAL_POLARITY_LOACTIVE,  ///< Low active signal
    DISPLAY_SIGNAL_POLARITY_HIACTIVE,  ///< High active signal
} display_signal_polarity_t;

/** Signal synchronization edge select */
typedef enum e_display_sync_edge
{
    DISPLAY_SIGNAL_SYNC_EDGE_RISING,   ///< Signal is synchronized to rising edge
    DISPLAY_SIGNAL_SYNC_EDGE_FALLING,  ///< Signal is synchronized to falling edge
} display_sync_edge_t;

/** Fading control */
typedef enum e_display_fade_control
{
    DISPLAY_FADE_CONTROL_NONE,         ///< Applying no fading control
    DISPLAY_FADE_CONTROL_FADEIN,       ///< Applying fade-in control
    DISPLAY_FADE_CONTROL_FADEOUT,      ///< Applying fade-out control
} display_fade_control_t;

/** Display signal timing setting */
typedef struct st_display_timing
{
    uint16_t total_cyc;                      ///< Total cycles in one line or total lines in one frame
    uint16_t display_cyc;                    ///< Active video cycles or lines
    uint16_t back_porch;                     ///< Back porch cycles or lines
    uint16_t sync_width;                     ///< Sync signal asserting width
    display_signal_polarity_t sync_polarity; ///< Sync signal polarity
} display_timing_t;

/** RGB Color setting */
typedef struct st_display_color
{
    union
    {
        uint32_t argb;                 ///< Entire color
        struct
        {
            uint8_t b;                 ///< blue
            uint8_t g;                 ///< green
            uint8_t r;                 ///< red
            uint8_t a;                 ///< alpha
        } byte;
    };
} display_color_t;

/** Graphics plane input configuration structure */
typedef struct st_display_input_cfg
{
    uint32_t          * p_base;                 ///< Base address to the frame buffer
    uint16_t            hsize;                  ///< Horizontal pixel size in a line
    uint16_t            vsize;                  ///< Vertical pixel size in a frame
    int16_t             coordinate_x;           ///< Coordinate X, this allows to set signed value.
    int16_t             coordinate_y;           ///< Coordinate Y, this allows to set signed value.
    uint32_t            hstride;                ///< Memory stride (bytes) in a line
    display_in_format_t format;                 ///< Input format setting
    uint16_t            data_swap;              ///< Data swap
} display_input_cfg_t;

/** Display output configuration structure */
typedef struct st_display_output_cfg
{
    display_timing_t             htiming;              ///< Horizontal display cycle setting
    display_timing_t             vtiming;              ///< Vertical display cycle setting
    display_out_format_t         format;               ///< Output format setting
    display_signal_polarity_t    data_enable_polarity; ///< Data Enable signal polarity
    display_sync_edge_t          sync_edge;            ///< Signal sync edge selection
    display_color_t              bg_color;             ///< Background color
    bool dithering_on;                                 ///< Dithering on/off
} display_output_cfg_t;

/** Contrast (gain) correction setting */
typedef struct st_display_coordinate
{
    int16_t x;                         ///< Coordinate X, this allows to set signed value.
    int16_t y;                         ///< Coordinate Y, this allows to set signed value.
} display_coordinate_t;

/** Graphics layer blend setup parameter structure */
typedef struct st_display_layer
{
    display_coordinate_t   coordinate;   ///< Blending location (starting point of image)
    display_color_t        bg_color;     ///< Color outside region
    display_fade_control_t fade_control; ///< Layer fade-in/out control on/off
    uint8_t                fade_speed;   ///< Layer fade-in/out frame rate
} display_layer_t;

/** Display main configuration structure */
typedef struct st_display_cfg
{
    /** Generic configuration for display devices */
    display_input_cfg_t  input[2];                         ///< Graphics input frame setting
    display_output_cfg_t output;                           ///< Graphics output frame setting
    display_layer_t      layer[2];                         ///< Graphics layer blend setting
} display_cfg_t;








#endif // RZG2L_DISPLAY_H
