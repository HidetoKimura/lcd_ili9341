#ifndef __ILI9341_SPI__
#define __ILI9341_SPI__

typedef struct {
        u8 r;
        u8 g;
        u8 b;
} color_t;

static const color_t black = {0x00, 0x00, 0x00};
static const color_t white = {0xFF, 0xFF, 0xFF};
static const color_t red   = {0xFF, 0x00, 0x00};
static const color_t green = {0x00, 0xFF, 0x00};
static const color_t blue  = {0x00, 0x00, 0xFF};
static const color_t yellow= {0xFF, 0xFF, 0x00};
static const color_t cyan  = {0x00, 0xFF, 0xFF};
static const color_t magenta={0xFF, 0x00, 0xFF};

typedef struct {
        int     x;
        int     y;
} pos_t;

typedef struct {
        int    width;
        int    height;
        int    color_depth;
        u16*   data;
} image_t;

#define BIT(nr) (UL(1) << (nr))

#endif