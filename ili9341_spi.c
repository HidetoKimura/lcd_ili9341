/****************************************************************************/ 
/*
 *  \file       ili9341_spi.c
 *
 *  \details    ili9341 Display SPI driver
 *
 *  \author     Hideto Kimura
 *
 *  \Tested with Linux raspberrypi 6.1.61-v8+
 *
 ****************************************************************************/
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/miscdevice.h>
#include "ili9341_spi.h"
#include "ezfont.h"
#include "bitmap_data.h"

#define DRV_NAME "lcd-ili9341"
#define DEV_NAME "lcd-ili9341"

#define ILI_COMMAND     1
#define ILI_DATA        0
#define CONSOLE_BUF_SIZE 512

struct ili9341_fix_screeninfo {
        unsigned long smem_start; /* Start of frame buffer mem  */
        u32 smem_len;             /* Length of frame buffer mem */
        u32 line_length;          /* length of a line in bytes  */
};

struct ili9341_var_screeninfo {
        u32 xres;               /* visible resolution		*/
        u32 yres;
        u32 xres_virtual;       /* virtual resolution		*/
        u32 yres_virtual;
        u32 color_depth;     
        u32 height;             /* height of picture in mm    */
        u32 width;              /* width of picture in mm     */
};

struct ili9341_fbinfo {
        struct ili9341_fix_screeninfo fix;
        struct ili9341_var_screeninfo var;
};

struct ili9341 {
        struct device *dev;
        struct spi_device *spi;
        struct ili9341_fbinfo info;
        struct miscdevice misc_device;
        struct {
		struct gpio_desc *reset;
		struct gpio_desc *dc;
		struct gpio_desc *led;
        } gpio;
        pos_t   cursor;
        int     start_col;
        int     end_col;
        int     start_line;
        int     end_line;
        u8      console_buf[CONSOLE_BUF_SIZE];
};

static int rotate = 90;
static int mode_BGR = 1;

static void ili9341_clear_graph(struct ili9341 *item);

int ili9341_write_spi(struct ili9341 *item, void *buf, size_t len)
{
        struct spi_transfer t = {
            .tx_buf = buf,
            .len = len,
            .speed_hz = 20000000,
        };
        struct spi_message m;

        if (!item->spi)
        {
                dev_err(item->dev,
                        "%s: par->spi is unexpectedly NULL\n", __func__);
                return -1;
        }

        spi_message_init(&m);
        spi_message_add_tail(&t, &m);
        return spi_sync(item->spi, &m);
}

int ili9341_write_spi_1_byte(struct ili9341 *item, unsigned char byte)
{
        unsigned char tmp_byte = byte;

        struct spi_transfer t = {
            .tx_buf = &tmp_byte,
            .len = 1,
            .speed_hz = 20000000,
        };
        struct spi_message m;

        pr_debug("%s(len=1): ", __func__);

        if (!item->spi)
        {
                dev_err(item->dev,
                        "%s: par->spi is unexpectedly NULL\n", __func__);
                return -1;
        }

        spi_message_init(&m);
        spi_message_add_tail(&t, &m);
        return spi_sync(item->spi, &m);
}

static void ili9341_init_gpio(struct ili9341 *item)
{

	item->gpio.reset = devm_gpiod_get_index_optional(item->dev, "reset", 0, GPIOD_OUT_HIGH);
        if(IS_ERR(item->gpio.reset))
        {
                dev_err(item->dev, "Failed to get reset gpio\n");
                return;
        }
	item->gpio.dc = devm_gpiod_get_index_optional(item->dev, "dc", 0, GPIOD_OUT_HIGH);
        if(IS_ERR(item->gpio.dc))
        {
                dev_err(item->dev, "Failed to get dc gpio\n");
                return;
        }
	item->gpio.led = devm_gpiod_get_index_optional(item->dev, "led", 0, GPIOD_OUT_LOW);
        if(IS_ERR(item->gpio.led))
        {
                dev_err(item->dev, "Failed to get led gpio\n");
                return;
        }
        gpiod_set_value(item->gpio.reset, 0);

        mdelay(10);

        gpiod_set_value(item->gpio.reset, 1);

        return;
}

static void ili9341_write_data(struct ili9341 *item, unsigned char dc, unsigned char value)
{

        if (dc == ILI_COMMAND)
        {
                gpiod_set_value(item->gpio.dc, 0);
                ili9341_write_spi_1_byte(item, value);
                gpiod_set_value(item->gpio.dc, 1);
        }
        else
        {       // ILI_DATA
                ili9341_write_spi_1_byte(item, value);
        }
}

#define MEM_Y   (7)   /* MY row address order */
#define MEM_X   (6)   /* MX column address order */
#define MEM_V   (5)   /* MV row / column exchange */
#define MEM_L   (4)   /* ML vertical refresh order */
#define MEM_BGR (3)   /* RGB-BGR Order */
#define MEM_H   (2)   /* MH horizontal refresh order */

void ili9341_set_display_res(struct ili9341 *item, int xres, int yres,
                             int xres_virtual, int yres_virtual, int width, int height)
{
        item->info.var.xres = xres;
        item->info.var.yres = yres;
        item->info.var.xres_virtual = xres_virtual;
        item->info.var.yres_virtual = yres_virtual;
        item->info.var.width = width;
        item->info.var.height = height;
        item->info.var.color_depth = 2;
        item->info.fix.line_length = xres * item->info.var.color_depth;
}

static void ili9341_set_display_options(struct ili9341 *item)
{
        // rotate
        ili9341_write_data(item, ILI_COMMAND, 0x36);
        switch (rotate)
        {
        case 0:
                ili9341_write_data(item, ILI_DATA, 1 << MEM_X);
                ili9341_set_display_res(item, 240, 320, 240, 320, 240, 320);
                break;

        case 90:
                ili9341_write_data(item, ILI_DATA, (1 << MEM_Y) | (1 << MEM_X) | (1 << MEM_V));
                ili9341_set_display_res(item, 320, 240, 320, 240, 320, 240);
                break;

        case 180:
                ili9341_write_data(item, ILI_DATA, 1 << MEM_Y);
                ili9341_set_display_res(item, 240, 320, 240, 320, 240, 320);
                break;

        case 270:
                ili9341_write_data(item, ILI_DATA, (1 << MEM_V) | (1 << MEM_L));
                ili9341_set_display_res(item, 320, 240, 320, 240, 320, 240);
                break;
        }
}

/* Init sequence taken from: Arduino Library for the Adafruit 2.2" display */
static int ili9341_init_display(struct ili9341 *item)
{
        /* Software Reset */
        ili9341_write_data(item, ILI_COMMAND, 0x01);

        mdelay(120);

        /* Display OFF */
        ili9341_write_data(item, ILI_COMMAND, 0x28);

        ili9341_write_data(item, ILI_COMMAND, 0xEF);
        ili9341_write_data(item, ILI_DATA, 0x03);
        ili9341_write_data(item, ILI_DATA, 0x80);
        ili9341_write_data(item, ILI_DATA, 0x02);

        ili9341_write_data(item, ILI_COMMAND, 0xCF);
        ili9341_write_data(item, ILI_DATA, 0x00);
        ili9341_write_data(item, ILI_DATA, 0xC1);
        ili9341_write_data(item, ILI_DATA, 0x30);

        ili9341_write_data(item, ILI_COMMAND, 0xED);
        ili9341_write_data(item, ILI_DATA, 0x64);
        ili9341_write_data(item, ILI_DATA, 0x03);
        ili9341_write_data(item, ILI_DATA, 0x12);
        ili9341_write_data(item, ILI_DATA, 0x81);

        ili9341_write_data(item, ILI_COMMAND, 0xE8);
        ili9341_write_data(item, ILI_DATA, 0x85);
        ili9341_write_data(item, ILI_DATA, 0x00);
        ili9341_write_data(item, ILI_DATA, 0x78);

        ili9341_write_data(item, ILI_COMMAND, 0xCB);
        ili9341_write_data(item, ILI_DATA, 0x39);
        ili9341_write_data(item, ILI_DATA, 0x2C);
        ili9341_write_data(item, ILI_DATA, 0x00);
        ili9341_write_data(item, ILI_DATA, 0x34);
        ili9341_write_data(item, ILI_DATA, 0x02);

        ili9341_write_data(item, ILI_COMMAND, 0xF7);
        ili9341_write_data(item, ILI_DATA, 0x20);

        ili9341_write_data(item, ILI_COMMAND, 0xEA);
        ili9341_write_data(item, ILI_DATA, 0x00);
        ili9341_write_data(item, ILI_DATA, 0x00);

        /* Power Control 1 */
        ili9341_write_data(item, ILI_COMMAND, 0xC0);
        ili9341_write_data(item, ILI_DATA, 0x23);

        /* Power Control 2 */
        ili9341_write_data(item, ILI_COMMAND, 0xC1);
        ili9341_write_data(item, ILI_DATA, 0x10);

        /* VCOM Control 1 */
        ili9341_write_data(item, ILI_COMMAND, 0xC5);
        ili9341_write_data(item, ILI_DATA, 0x3e);
        ili9341_write_data(item, ILI_DATA, 0x28);

        /* VCOM Control 2 */
        ili9341_write_data(item, ILI_COMMAND, 0xC7);
        ili9341_write_data(item, ILI_DATA, 0x86);

        /* COLMOD: Pixel Format Set */
        /* 16 bits/pixel */
        ili9341_write_data(item, ILI_COMMAND, 0x3A);
        ili9341_write_data(item, ILI_DATA, 0x55);

        /* Frame Rate Control */
        /* Division ratio = fosc, Frame Rate = 79Hz */
        ili9341_write_data(item, ILI_COMMAND, 0xB1);
        ili9341_write_data(item, ILI_DATA, 0x00);
        ili9341_write_data(item, ILI_DATA, 0x18);

        /* Display Function Control */
        ili9341_write_data(item, ILI_COMMAND, 0xB6);
        ili9341_write_data(item, ILI_DATA, 0x08);
        ili9341_write_data(item, ILI_DATA, 0x82);
        ili9341_write_data(item, ILI_DATA, 0x27);

        /* MADCTL, required to resolve 'mirroring' effect */
        ili9341_write_data(item, ILI_COMMAND, 0x36);
        if (mode_BGR)
        {
                ili9341_write_data(item, ILI_DATA, 0x48);
                ili9341_set_display_options(item);
                printk("COLOR LCD in BGR mode\n");
        }
        else
        {
                ili9341_write_data(item, ILI_DATA, 0x40);
                ili9341_set_display_options(item);
                printk("COLOR LCD in RGB mode\n");
        }

        /* Gamma Function Disable */
        ili9341_write_data(item, ILI_COMMAND, 0xF2);
        ili9341_write_data(item, ILI_DATA, 0x00);

        /* Gamma curve selected  */
        ili9341_write_data(item, ILI_COMMAND, 0x26);
        ili9341_write_data(item, ILI_DATA, 0x01);

        /* Positive Gamma Correction */
        ili9341_write_data(item, ILI_COMMAND, 0xE0);
        ili9341_write_data(item, ILI_DATA, 0x0F);
        ili9341_write_data(item, ILI_DATA, 0x31);
        ili9341_write_data(item, ILI_DATA, 0x2B);
        ili9341_write_data(item, ILI_DATA, 0x0C);
        ili9341_write_data(item, ILI_DATA, 0x0E);
        ili9341_write_data(item, ILI_DATA, 0x08);
        ili9341_write_data(item, ILI_DATA, 0x4E);
        ili9341_write_data(item, ILI_DATA, 0xF1);
        ili9341_write_data(item, ILI_DATA, 0x37);
        ili9341_write_data(item, ILI_DATA, 0x07);
        ili9341_write_data(item, ILI_DATA, 0x10);
        ili9341_write_data(item, ILI_DATA, 0x03);
        ili9341_write_data(item, ILI_DATA, 0x0E);
        ili9341_write_data(item, ILI_DATA, 0x09);
        ili9341_write_data(item, ILI_DATA, 0x00);

        /* Negative Gamma Correction */
        ili9341_write_data(item, ILI_COMMAND, 0xE1);
        ili9341_write_data(item, ILI_DATA, 0x00);
        ili9341_write_data(item, ILI_DATA, 0x0E);
        ili9341_write_data(item, ILI_DATA, 0x14);
        ili9341_write_data(item, ILI_DATA, 0x03);
        ili9341_write_data(item, ILI_DATA, 0x11);
        ili9341_write_data(item, ILI_DATA, 0x07);
        ili9341_write_data(item, ILI_DATA, 0x31);
        ili9341_write_data(item, ILI_DATA, 0xC1);
        ili9341_write_data(item, ILI_DATA, 0x48);
        ili9341_write_data(item, ILI_DATA, 0x08);
        ili9341_write_data(item, ILI_DATA, 0x0F);
        ili9341_write_data(item, ILI_DATA, 0x0C);
        ili9341_write_data(item, ILI_DATA, 0x31);
        ili9341_write_data(item, ILI_DATA, 0x36);
        ili9341_write_data(item, ILI_DATA, 0x0F);

        /* Sleep OUT */
        ili9341_write_data(item, ILI_COMMAND, 0x11);

        mdelay(120);

        /* Display ON */
        ili9341_write_data(item, ILI_COMMAND, 0x29);

        ili9341_clear_graph(item);

        gpiod_set_value(item->gpio.led, 1);

        printk("COLOR LCD driver initialized\n");

        return 0;
}

static void ili9341_set_window(struct ili9341 *item, int xs, int ys, int xe, int ye)
{
        //	int w, h;
        printk("%s(xs=%d, ys=%d, xe=%d, ye=%d)\n", __func__, xs, ys, xe, ye);

        /* Column address */
        ili9341_write_data(item, ILI_COMMAND, 0x2A);
        ili9341_write_data(item, ILI_DATA, xs >> 8);
        ili9341_write_data(item, ILI_DATA, xs & 0xFF);
        ili9341_write_data(item, ILI_DATA, xe >> 8);
        ili9341_write_data(item, ILI_DATA, xe & 0xFF);

        /* Row adress */
        ili9341_write_data(item, ILI_COMMAND, 0x2B);
        ili9341_write_data(item, ILI_DATA, ys >> 8);
        ili9341_write_data(item, ILI_DATA, ys & 0xFF);
        ili9341_write_data(item, ILI_DATA, ye >> 8);
        ili9341_write_data(item, ILI_DATA, ye & 0xFF);

        /* Memory write */
        ili9341_write_data(item, ILI_COMMAND, 0x2C);

        return;
}

static void ili9341_clear_graph(struct ili9341 *item)
{
        switch (rotate)
        {
        case 0:
        case 180:
                ili9341_set_window(item, 0x0000, 0x0000, 0x00ef, 0x013f);
                break;
        case 90:
        case 270:
                ili9341_set_window(item, 0x0000, 0x0000, 0x013f, 0x00ef);
                break;
        }
}

static int ili9341_video_alloc(struct ili9341 *item)
{
        unsigned int frame_size, pages_count;

        dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);

        frame_size = item->info.fix.line_length * item->info.var.yres;
        dev_dbg(item->dev, "%s: item=0x%p frame_size=%u\n",
                __func__, (void *)item, frame_size);

        pages_count = frame_size / PAGE_SIZE;
        if ((pages_count * PAGE_SIZE) < frame_size)
        {
                pages_count++;
        }
        dev_dbg(item->dev, "%s: item=0x%p pages_count=%u\n",
                __func__, (void *)item, pages_count);

        item->info.fix.smem_len = pages_count * PAGE_SIZE;
        item->info.fix.smem_start = (long unsigned int)vmalloc(item->info.fix.smem_len);
        if (!item->info.fix.smem_start)
        {
                dev_err(item->dev, "%s: unable to vmalloc\n", __func__);
                return -ENOMEM;
        }
        memset((void *)item->info.fix.smem_start, 0, item->info.fix.smem_len);

        return 0;
}

static void ili9341_video_free(struct ili9341 *item)
{
        dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);

        vfree((void *)item->info.fix.smem_start);
}

static void ili9341_flush(struct ili9341 *item)
{
        /* Memory write */
        ili9341_write_data(item, ILI_COMMAND, 0x2C);
        ili9341_write_spi(item, (void *)item->info.fix.smem_start, item->info.fix.smem_len);
}

static void ili9341_fill_screen(struct ili9341 *item, color_t* color)
{
        u16* mem;
        u16  tmp;
        u32  len;
        mem = (unsigned short *)item->info.fix.smem_start;

        for (len = item->info.fix.smem_len; len > 0; len -= item->info.var.color_depth)
        {
                // BGR565 Big Endian
                tmp = (color->b >> 3) << 11 | (color->g >> 2) << 5 | color->r >> 3;
                *mem = (tmp & 0xFF << 8) | tmp >> 8;
                mem++;
        }
}

static void ili9341_blt_image(struct ili9341 *item, pos_t* pos, image_t* image)
{
        int i, j;
        u16 *mem;
        u16 *src;

        mem = (u16 *)item->info.fix.smem_start;
        mem += (pos->y * item->info.var.xres) + pos->x;
        src = image->data;

        for (i = 0; i < image->height; i++)
        {
                for (j = 0; j < image->width; j++)
                {
                        *mem++ = *src++;
                }
                mem += item->info.var.xres - image->width;
        }
}

static void set_pixel_to_image(pos_t* pos, color_t* color, image_t* image)
{
        u16 *mem;
        u16  tmp;

        mem = image->data;
        mem += (pos->y * image->width) + pos->x;

        if (image->data - mem > image->width * image->height * image->color_depth)
        {
                return;
        }
        // BGR565 Big Endian
        tmp  = (color->b >> 3) << 11 | (color->g >> 2) << 5 | color->r >> 3;
        *mem = (tmp & 0xFF << 8) | tmp >> 8;

        return;
}

static void ili9341_draw_bitmap(struct ili9341 *item, const bitmap_t *bitmap, int zoom)
{
        u16* data;
        int  x, y, cx, cy, size, offset;
        color_t  color;
        pos_t    pos;
        image_t  image;

        size = (bitmap->width * zoom ) * (bitmap->height * zoom) * item->info.var.color_depth;
        data = (u16*)vmalloc(size);
        if (!data) {
                dev_err(item->dev, "Failed to allocate memory\n");
                return;
        }
        memset(data, 0, size);

        image.width = bitmap->width * zoom;
        image.height = bitmap->height * zoom;
        image.color_depth = item->info.var.color_depth;
        image.data = data;

        for (y = 0; y < image.height ; y++) {
                for (x = 0; x < image.width; x++) {
                        cx = x / zoom;
                        cy = y / zoom;
                        offset = bitmap->data[cy * bitmap->width + cx];
                        color = bitmap->color_table[offset];
                        pos.x = x;
                        pos.y = y;
                        set_pixel_to_image(&pos, &color, &image);
                }
        }

        if (item->cursor.x + image.width > item->end_col) {
                item->cursor.x = item->start_col;
                item->cursor.y += image.height;
        }
        if (item->cursor.y + image.height > item->end_line)
        {
                item->cursor.y = item->start_line;
        }

        pos.x = item->cursor.x;
        pos.y = item->cursor.y;
        ili9341_blt_image(item, &pos, &image);
        item->cursor.x += image.width;

        vfree(data);

        return;
}

static void ili9341_puts(struct ili9341 *item, u8 *str, int zoom)
{
        uint8_t font_data[FONT_HEIGHT];
        int  x, y, cx, cy, size;
        int  char_width, char_height;
        uint8_t *tok;
        color_t  color;
        pos_t    pos;
        image_t  image;
        u16*     data;

        char_width  = FONT_WIDTH * zoom ;
        char_height = FONT_HEIGHT * zoom ;

        size = char_width * char_height * item->info.var.color_depth;
        data = (u16*)vmalloc(size);
        if (!data) {
                dev_err(item->dev, "Failed to allocate memory\n");
                return;
        }

        while (*str != '\0') {
                if (*str == '\n') {
                        tok = ++str;
                        item->cursor.y += char_width;
                        item->cursor.x = item->start_col;
                        if(item->cursor.y >= item->end_col)
                        {
                                item->cursor.y = item->start_line;
                        }
                        continue;
                }
                tok = ezfont_get_fontdata_by_utf8(str, true, font_data, sizeof(font_data));
                if (tok == NULL)
                        break;
                str = tok;

                memset(data, 0, size);
                image.width = char_width;
                image.height = char_height;
                image.color_depth = item->info.var.color_depth;
                image.data = data;

                for (y = 0; y < char_height; y++) {
                        for (x = 0; x < char_width; x++) {
                                cx = x / zoom;
                                cy = y / zoom;
                                if (font_data[cy] & BIT(FONT_WIDTH - 1 - cx)) {
                                        color = white;
                                        pos.x = x;
                                        pos.y = y;
                                        set_pixel_to_image(&pos, &color, &image);
                                }
                        }
                }
                ili9341_blt_image(item, &item->cursor, &image);
                item->cursor.x += char_width;
                if (item->cursor.x >= item->end_col) {
                        item->cursor.x = item->start_col;
                        item->cursor.y += char_height;
                        if(item->cursor.y >= item->end_line)
                        {
                                item->cursor.y = item->start_line;
                        }
                }
        }
        vfree(data);

        return;
}

static void ili9341_set_limit(struct ili9341 *item, int start_col, int end_col, int start_line, int end_line)
{
        if(start_col >= 0 && start_col < item->info.var.xres) {
                item->start_col = start_col;
        } else 
        {
                item->start_col = 0;
        }
        if(end_col >= 0 && end_col < item->info.var.xres) {
                item->end_col = end_col;
        } else 
        {
                item->end_col = item->info.var.xres;
        }
        if(start_line >= 0 && start_line < item->info.var.yres) {
                item->start_line = start_line;
        } else 
        {
                item->start_line = 0;
        }
        if(end_line >= 0 && end_line < item->info.var.yres) {
                item->end_line = end_line;
        } else 
        {
                item->end_line = item->info.var.yres;
        }

}

static void ili9341_set_cursor(struct ili9341 *item, int x, int y)
{
        if( x >= 0 && x < item->info.var.xres &&
            y >= 0 && y < item->info.var.yres) {
                item->cursor.x = x;
                item->cursor.y = y;
        } else {
                item->cursor.x = 0;
                item->cursor.y = 0;
        } 

        ili9341_set_limit(item, item->cursor.x, -1, item->cursor.y, -1);

        return;
}

static void ili9341_draw_opening(struct ili9341 *item)
{
        int i,j;

        ili9341_fill_screen(item, (color_t*)&black);

        // 1st character
        ili9341_set_cursor(item, 16, 4);

        ili9341_draw_bitmap(item, &corner_lt, 1);
        for (i = 0; i < 8; i++) {
                ili9341_draw_bitmap(item, &h_line, 1);
        }
        ili9341_draw_bitmap(item, &corner_rt, 1);
        ili9341_puts(item, "\n", 1);

        for(j = 0; j < 5; j++) {
                ili9341_draw_bitmap(item, &v_line, 1);
                for (i = 0; i < 8; i++) {
                        ili9341_puts(item, " ", 1);
                }
                ili9341_draw_bitmap(item, &v_line, 1);
                ili9341_puts(item, "\n", 1);
        }

        ili9341_draw_bitmap(item, &corner_lb, 1);
        for (i = 0; i < 8; i++) {
                ili9341_draw_bitmap(item, &h_line, 1);
        }
        ili9341_draw_bitmap(item, &corner_rb, 1);
        ili9341_puts(item, "\n", 1);

        ili9341_set_cursor(item, 32,  4);
        ili9341_puts(item, "　りなくす　\n", 1);

        ili9341_set_cursor(item, 32, 24);
        ili9341_puts(item, "ＨＰ：９９９\n", 1);
        ili9341_puts(item, "ＭＰ：９９９\n", 1);
        ili9341_puts(item, "勇者：　９９\n", 1);

        // 2nd character
        ili9341_set_cursor(item, 96, 4);

        ili9341_draw_bitmap(item, &corner_lt, 1);
        for (i = 0; i < 8; i++) {
                ili9341_draw_bitmap(item, &h_line, 1);
        }
        ili9341_draw_bitmap(item, &corner_rt, 1);
        ili9341_puts(item, "\n", 1);

        for(j = 0; j < 5; j++) {
                ili9341_draw_bitmap(item, &v_line, 1);
                for (i = 0; i < 8; i++) {
                        ili9341_puts(item, " ", 1);
                }
                ili9341_draw_bitmap(item, &v_line, 1);
                ili9341_puts(item, "\n", 1);
        }

        ili9341_draw_bitmap(item, &corner_lb, 1);
        for (i = 0; i < 8; i++) {
                ili9341_draw_bitmap(item, &h_line, 1);
        }
        ili9341_draw_bitmap(item, &corner_rb, 1);
        ili9341_puts(item, "\n", 1);

        ili9341_set_cursor(item, 96 + 16,  4);

        ili9341_puts(item, "　マリベル　\n", 1);

        ili9341_set_cursor(item, 96 + 16, 24);

        ili9341_puts(item, "ＨＰ：　１０\n", 1);
        ili9341_puts(item, "ＭＰ：　１０\n", 1);
        ili9341_puts(item, "女王：　９９\n", 1);

        #if 0 // for debug
        ili9341_puts(item, "Hello World!\n", 1);
        ili9341_puts(item, "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789 !\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~\n", 1);
        #endif

        ili9341_set_cursor(item, 100, 100);
        for (i = 0; i < 3; i++)
        {
                ili9341_draw_bitmap(item, &slime, 2);
        }

        ili9341_set_cursor(item, 16, 148);

        ili9341_draw_bitmap(item, &corner_lt, 1);
        for (i = 0; i < 34; i++) {
                ili9341_draw_bitmap(item, &h_line, 1);
        }
        ili9341_draw_bitmap(item, &corner_rt, 1);
        ili9341_puts(item, "\n", 1);

        for(j = 0; j < 8; j++) {
                ili9341_draw_bitmap(item, &v_line, 1);
                for (i = 0; i < 34; i++) {
                        ili9341_puts(item, " ", 1);
                }
                ili9341_draw_bitmap(item, &v_line, 1);
                ili9341_puts(item, "\n", 1);
        }

        ili9341_draw_bitmap(item, &corner_lb, 1);
        for (i = 0; i < 34; i++) {
                ili9341_draw_bitmap(item, &h_line, 1);
        }
        ili9341_draw_bitmap(item, &corner_rb, 1);
        ili9341_puts(item, "\n", 1);

        ili9341_set_cursor(item, 32, 164);
        ili9341_set_limit(item,  32, 32 + 8 * 32, 164, 164 + 8 * 6);

        ili9341_puts(item, "スライムがあらわれた！\n\n", 1);
        ili9341_puts(item, "＊「プルプルッ！ぼく、 悪いスライムじゃないよ」\n", 1);

        ili9341_flush(item);
}

static void ili9341_clear_console(struct ili9341 *item)
{
        int i, j;

        ili9341_set_cursor(item, 32, 164);
        for(j = 0; j < 6; j++) {
                for (i = 0; i < 32; i++) {
                        ili9341_puts(item, " ", 1);
                }
                ili9341_puts(item, "\n", 1);
        }
        ili9341_set_cursor(item, 32, 164);
        ili9341_set_limit(item,  32, 32 + 8 * 32, 164, 164 + 8 * 6);
        return;
}
static void ili9341_draw_ending(struct ili9341 *item)
{
        ili9341_clear_console(item);
        ili9341_puts(item, "りなくすはにげ出した！\n\n", 1);
        ili9341_puts(item, "マリベル「遊んでくれて、ありがと。つまらなかったわ。\nじゃあね。」\n", 1);

        ili9341_flush(item);

        mdelay(3000);

        ili9341_fill_screen(item, (color_t*)&black);
        ili9341_flush(item);

        gpiod_set_value(item->gpio.led, 0);

}

static ssize_t ili9341_write(struct file *file, const char __user *buff, size_t count, loff_t *ppos)
{
	struct ili9341 *item;

        pr_info("ili9341_write() is called.\n");

	item = container_of(file->private_data,
				  struct ili9341, misc_device);
        if(!item) {
                dev_err(item->dev,"Failed to get private data\n");
                return -EFAULT;
        }

        if(count > sizeof(item->console_buf)) {
                dev_err(item->dev,"Too large data\n");
                return -EFAULT;
        }
        
	if(copy_from_user(item->console_buf, buff, count)) {
		pr_info("Bad copied value\n");
		return -EFAULT;
	}

        item->console_buf[count - 1] = '\0';
        ili9341_clear_console(item);
        ili9341_puts(item, item->console_buf, 1);
        ili9341_flush(item);

        return count;
}

static ssize_t ili9341_read(struct file *file, char __user *buff, size_t count, loff_t *ppos)
{
	struct ili9341 *item;

        pr_info("ili9341_read() is called.\n");

	item = container_of(file->private_data,
				  struct ili9341, misc_device);
        if(!item) {
                dev_err(item->dev,"Failed to get private data\n");
                return -EFAULT;
        }

        if(count > sizeof(item->console_buf)) {
                count = sizeof(item->console_buf);
        }
        
	if(*ppos == 0){
		if(copy_to_user(buff, item->console_buf, sizeof(item->console_buf))){
                        dev_err(item->dev, "Bad copied value\n");
			return -EFAULT;
		}
		*ppos+=1;
		return sizeof(item->console_buf);
	}

        return 0;
}


static struct ili9341_fix_screeninfo ili9341_fix  = {
        .line_length = 320 * 2,
};

static struct ili9341_var_screeninfo ili9341_var  = {
        .xres           = 320,
        .yres           = 240,
        .xres_virtual   = 320,
        .yres_virtual   = 240,
        .width          = 320,
        .height         = 240,
        .color_depth    = 2,
};

static const struct file_operations ili9341_fops = {
        .owner = THIS_MODULE,
        .read = ili9341_read,
        .write = ili9341_write,
};



static int ili9341_probe(struct spi_device *spi)
{
        int ret;
        struct ili9341 *item;
        struct device *dev = &spi->dev;

        pr_info("%s: called\n", __func__);

        item = devm_kzalloc(&spi->dev, sizeof(struct ili9341), GFP_KERNEL);
        if (!item)
        {
                dev_err(dev,
                        "%s: unable to kzalloc for ili9341\n", __func__);
                ret = -ENOMEM;
                goto out;
        }
        item->dev = dev;
        item->spi = spi;

        item->info.fix = ili9341_fix;
        item->info.var = ili9341_var;
        
        ret = ili9341_video_alloc(item);
        if (ret)
        {
                dev_err(dev,
                        "%s: unable to ili9341_video_alloc\n", __func__);
                goto out;
        }

        item->misc_device.name = DEV_NAME;
        item->misc_device.minor = MISC_DYNAMIC_MINOR;
        item->misc_device.fops = &ili9341_fops;

        ret = misc_register(&item->misc_device);
        if (ret)
        {
                dev_err(dev,
                        "%s: unable to register misc device\n", __func__);
                goto out_video;
        }

        spi_set_drvdata(spi, item);

        ili9341_init_gpio(item);
        ili9341_init_display(item);
        
        ili9341_draw_opening(item);

        return ret;

out_video:
        ili9341_video_free(item);
out:
        return ret;
}

static void ili9341_remove(struct spi_device *spi)
{
        struct ili9341 *item = spi_get_drvdata(spi);

        if (item)
        {
                ili9341_draw_ending(item);
                ili9341_video_free(item);
                misc_deregister(&item->misc_device);
        }

        return;
}

static const struct spi_device_id ili9341_spi_id[] = {
	{"ili9341-spi", 0},
	{}
};
MODULE_DEVICE_TABLE(spi, ili9341_spi_id);

static const struct of_device_id my_of_ids[] = {
        {.compatible = "myboard,ili9341-spi"},
        {},
};

MODULE_DEVICE_TABLE(of, my_of_ids);

static struct spi_driver ili9341_spi_driver = {
        .driver = {
                .name = DRV_NAME,
                .of_match_table = my_of_ids,
        },
        .probe = ili9341_probe,
        .remove = ili9341_remove,
	.id_table = ili9341_spi_id,
};

static int ili9341_init(void)
{
        int ret_val;

        ret_val = spi_register_driver(&ili9341_spi_driver);
        if (ret_val != 0)
        {
                pr_err("platform value returned %d\n", ret_val);
                return ret_val;
        }

        return 0;
}

static void ili9341_exit(void)
{
        spi_unregister_driver(&ili9341_spi_driver);
}

module_init(ili9341_init);
module_exit(ili9341_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hideto Kimura");
MODULE_DESCRIPTION("ILI9341 SPI Driver");
MODULE_VERSION("1.00");
