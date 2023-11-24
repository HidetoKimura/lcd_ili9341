/***************************************************************************//**
*  \file       ili9341_spi.c
*
*  \details    ili9341 Display SPI driver 
*
*  \author     Hideto Kimura
*
*  \Tested with Linux raspberrypi 6.1.61-v8+
*
* *******************************************************************************/
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/miscdevice.h>

#include "ezfont.h"
#include "bitmap_data.h" 

#define DRV_NAME 	"ili9341_spi"
#define DEV_NAME 	"ili9341_spi"

#define ILI_COMMAND            1
#define ILI_DATA               0

#define ILI_GPIO_DC						25
#define ILI_GPIO_RESET				24

struct ili9341_fix_screeninfo {
	unsigned long smem_start;	/* Start of frame buffer mem  */
	u32 smem_len;			        /* Length of frame buffer mem */
	u32 line_length;		      /* length of a line in bytes  */
};

struct ili9341_var_screeninfo {
	u32 xres;			/* visible resolution		*/
	u32 yres;
	u32 xres_virtual;		/* virtual resolution		*/
	u32 yres_virtual;
	u32 bits_per_pixel;		/* guess what			*/
	u32 grayscale;		/* 0 = color, 1 = grayscale,	*/
	u32 height;			/* height of picture in mm    */
	u32 width;			/* width of picture in mm     */
};

struct ili9341_fbinfo {
  struct ili9341_fix_screeninfo fix;
  struct ili9341_var_screeninfo var;
};

struct ili9341 {
        struct device *dev;
    	  struct spi_device *spi;
        struct ili9341_fbinfo    info;
	      struct miscdevice misc_device;
};

static int rotate = 270;
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

	if (!item->spi) {
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

	if (!item->spi) {
		dev_err(item->dev,
			"%s: par->spi is unexpectedly NULL\n", __func__);
		return -1;
	}

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(item->spi, &m);
}

static void __init ili9341_init_gpio(struct ili9341 *item)
{
	//DC high - data, DC low - command
	gpio_request_one(ILI_GPIO_DC, GPIOF_OUT_INIT_HIGH, "dc");

	//Reset high - normal operation, Reset low - reset
	gpio_request_one(ILI_GPIO_RESET, GPIOF_OUT_INIT_HIGH, "reset");

	gpio_set_value(ILI_GPIO_RESET, 0);

	mdelay(10);

	gpio_set_value(ILI_GPIO_RESET, 1);

	return;
}

static void ili9341_free_gpio(void)
{
	gpio_free(ILI_GPIO_DC);
	gpio_free(ILI_GPIO_RESET);
}

static void ili9341_write_data(struct ili9341 *item, unsigned char dc, unsigned char value) {

	if (dc == ILI_COMMAND) {
		gpio_set_value(ILI_GPIO_DC, 0);
		ili9341_write_spi_1_byte(item, value);
		gpio_set_value(ILI_GPIO_DC, 1);
	} else { //ILI_DATA
		ili9341_write_spi_1_byte(item, value);
	}

}

#define MEM_Y   (7) /* MY row address order */
#define MEM_X   (6) /* MX column address order */
#define MEM_V   (5) /* MV row / column exchange */
#define MEM_L   (4) /* ML vertical refresh order */
#define MEM_BGR (3) /* RGB-BGR Order */
#define MEM_H   (2) /* MH horizontal refresh order */

void ili9341_set_display_res(struct ili9341 *item, int xres, int yres,
		int xres_virtual, int yres_virtual, int width, int height)
{
	item->info.var.xres = xres;
	item->info.var.yres = yres;
	item->info.var.xres_virtual = xres_virtual;
	item->info.var.yres_virtual = yres_virtual;
	item->info.var.width = width;
	item->info.var.height = height;
  item->info.var.bits_per_pixel = 16;
	item->info.fix.line_length = xres * (item->info.var.bits_per_pixel / 8);
}

static void ili9341_set_display_options(struct ili9341 *item)
{
	//rotate
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
		ili9341_write_data(item, ILI_DATA, (1<<MEM_V) | (1 << MEM_L));
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
	if (mode_BGR)	{
		ili9341_write_data(item, ILI_DATA, 0x48);
		ili9341_set_display_options(item);
		printk("COLOR LCD in BGR mode\n");
	} else 	{
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

#if 0
	for (h = 0; h <= ye - ys; h++) {
		for (w = 0; w <= xe - xs; w++) {
			//Blue
//			ili9341_write_data(item, ILI_DATA, 0xF8);
//			ili9341_write_data(item, ILI_DATA, 0x00);
			//Grean
//			ili9341_write_data(item, ILI_DATA, 0x07);
//			ili9341_write_data(item, ILI_DATA, 0xE0);
			//Red
//			ili9341_write_data(item, ILI_DATA, 0x00);
//			ili9341_write_data(item, ILI_DATA, 0x1F);
			//Black
			ili9341_write_data(item, ILI_DATA, 0x00);
			ili9341_write_data(item, ILI_DATA, 0x00);
		}
	}
#endif
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
        if ((pages_count * PAGE_SIZE) < frame_size) {
                pages_count++;
        }
        dev_dbg(item->dev, "%s: item=0x%p pages_count=%u\n",
                __func__, (void *)item, pages_count);

        item->info.fix.smem_len = pages_count * PAGE_SIZE;
        item->info.fix.smem_start = (long unsigned int)vmalloc(item->info.fix.smem_len);
        if (!item->info.fix.smem_start) {
                dev_err(item->dev, "%s: unable to vmalloc\n", __func__);
                return -ENOMEM;
        }
        memset((void *)item->info.fix.smem_start, 0, item->info.fix.smem_len);

        return 0;
}

static void ili9341_video_free(struct ili9341 *item)
{
        dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);

        kfree((void *)item->info.fix.smem_start);
}


static void ili9341_flush(struct ili9341 *item)
{
#if 0
  int i, j;

	/* Memory write */
	ili9341_write_data(item, ILI_COMMAND, 0x2C);

  //Copy all pages.
  for (i = 0; i < item->pages_count; i++) {
      if(i == item->pages_count - 1) {
//        memcpy(item->tmpbuf, item->pages[i].buffer, PAGE_SIZE / 2);
        for (j = 0; j < PAGE_SIZE / 2; j++) {
          item->tmpbuf_be[j] = htons(item->tmpbuf[j]);
        }
        ili9341_write_spi(item, item->tmpbuf_be, PAGE_SIZE / 2);
      }
      else {
//        memcpy(item->tmpbuf, item->pages[i].buffer, PAGE_SIZE);
        for (j=0; j<PAGE_SIZE; j++) {
          item->tmpbuf_be[j] = htons(item->tmpbuf[j]);
        }
        ili9341_write_spi(item, item->tmpbuf_be, PAGE_SIZE);
      }
  }
#else
	/* Memory write */
	ili9341_write_data(item, ILI_COMMAND, 0x2C);
  ili9341_write_spi(item, (void*)item->info.fix.smem_start, item->info.fix.smem_len);
#endif
}

static void ili9341_fill_screen(struct ili9341 *item)
{
  u16*  mem;
  u32   length;
  mem = (unsigned short*)item->info.fix.smem_start;

  for(length = item->info.fix.smem_len; length > 0; length -= 2) {
    *mem = 0x00F8; // Blue
//    *mem = 0xE007; // Green
//    *mem = 0x1F00; // Red
    mem++;
  }

}

static void ili9341_blt_image(struct ili9341 *item, int x, int y, int w, int h, const u16 *data)
{
  int i, j;
  u16 *mem;
  u16 *src;

  mem = (u16*)item->info.fix.smem_start;
  mem += (y * item->info.var.xres) + x;
  src = (u16*)data;

  for (i = 0; i < h; i++) {
    for (j = 0; j < w; j++) {
      *mem++ = *src++;
    }
    mem += item->info.var.xres - w;
  }
}

static void ili9341_create_pixel(int x, int y, int s, u8 r, u8 g, u8 b, uint16_t* data, int len)
{
  u16 *mem;
  u16 temp;

  mem = (u16*)data;
  mem += (y * s) + x;

  if(data - mem > len) {
    return;
  }

  // BGR565
  temp = (b >> 3) << 11 | (g >> 2) << 5 | r >> 3;
  *mem = (temp & 0xFF << 8) | temp >> 8;

  return;

}

static void ili9341_create_bitmap(uint16_t* data, int len)
{
  #define BITMAP_X_RES 16
  #define BITMAP_Y_RES 16
  u8 r, g, b, t;
  int x, y;
  
  for (y = 0; y > BITMAP_Y_RES; y++) {
    for (x = 0; x > BITMAP_X_RES; x++) {
      t = sample_bmp_data[y * BITMAP_X_RES + x];
      r = sample_color_table[t][0];
      g = sample_color_table[t][1];
      b = sample_color_table[t][2];
      ili9341_create_pixel(x, y, BITMAP_X_RES, r, g, b, data, len);
    }
  }
}

#define BIT(nr)			(UL(1) << (nr))
static void ili9341_puts(struct ili9341 *item, u8 *str)
{
  #define FONT_WIDTH  8
  #define FONT_HEIGHT 8

  uint8_t  font_data[FONT_HEIGHT];
  uint16_t bmp_data[FONT_HEIGHT][FONT_WIDTH];
  int      x, y;
  int      cursor = 0;
  int      line = 0;
  uint8_t* tok;

  while ( *str != '\0')
  {
    if (*str == '\n') {
      tok = ++str;
      line += FONT_HEIGHT;
      continue;
    }
    tok = ezfont_get_fontdata_by_utf8(str, true, font_data, sizeof(font_data));
    if (tok == NULL) break;
    str = tok;

    memset(bmp_data, 0, sizeof(bmp_data));
    for (y = 0; y < FONT_HEIGHT; y++) {
      for (x = 0; x < FONT_WIDTH; x++) {
        if (font_data[y] & BIT(FONT_WIDTH - 1 - x)) {
          ili9341_create_pixel( 
            x, y, FONT_WIDTH, 0xFF, 0xFF, 0xFF, &bmp_data[0][0], sizeof(bmp_data));
        }
      }
    }
    ili9341_blt_image(item, cursor, line, FONT_WIDTH, FONT_HEIGHT, &bmp_data[0][0]);
    cursor += FONT_WIDTH;
    if( cursor >= item->info.var.xres ) {
      cursor = 0;
      line += FONT_HEIGHT;
    }

  }
  return;
}


static ssize_t ili9341_write(struct file *file, const char __user *buff, size_t count, loff_t *ppos)
{
	pr_info("ili9341_write() is called.\n");

	return count;
}

static ssize_t ili9341_read(struct file *file, char __user *buff, size_t count, loff_t *ppos)
{
  pr_info("ili9341_read() is called.\n");

	return 0;
}

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
			
	item = devm_kzalloc(&spi->dev, sizeof(struct ili9341), GFP_KERNEL);
  if (!item) {
    dev_err(dev,
            "%s: unable to kzalloc for ili9341\n", __func__);
    ret = -ENOMEM;
    goto out;
  }
  item->dev = dev;
  item->spi = spi;

  ret = ili9341_video_alloc(item);
  if (ret) {
    dev_err(dev,
            "%s: unable to ili9341_video_alloc\n", __func__);
    goto out_info ;
  }

	item->misc_device.name  = DEV_NAME;
	item->misc_device.minor = MISC_DYNAMIC_MINOR;
  item->misc_device.fops  = &ili9341_fops;

	ret = misc_register(&item->misc_device);
	if (ret) {
    dev_err(dev,
            "%s: unable to register misc device\n", __func__);
    goto out_video;
  }

	spi_set_drvdata(spi, item);

  ili9341_init_gpio(item);
  ili9341_init_display(item);

  ili9341_fill_screen(item);

  ili9341_puts(item, "Hello World!\n");

  {
    uint16_t data[16][16];
    ili9341_create_bitmap(&data[0][0], sizeof(data));

    ili9341_blt_image(item, 0, 100, 16, 16, &data[0][0]);
  }

  ili9341_flush(item);

  return ret;

out_video:
  ili9341_video_free(item);
out_info:
#if 0
  kfree(item->tmpbuf_be);
out_tmpbuf_be:
  kfree(item->tmpbuf);
out_tmpbuf:
#endif
  kfree(item);
out:
  return ret;
}

static void ili9341_remove(struct spi_device *spi)
{
	struct ili9341 *item = spi_get_drvdata(spi);
  if (item) {
    ili9341_video_free(item);
    kfree(item);
  }
  ili9341_free_gpio();

	misc_deregister(&item->misc_device);

	return;
}

static const struct of_device_id my_of_ids[] = {
	{ .compatible = "myboard,ili9341-spi" },
	{},
};

MODULE_DEVICE_TABLE(of, my_of_ids);

static struct spi_driver ili9341_spi_driver = { 
	.driver = { 
		.name   = DRV_NAME,
		.of_match_table = my_of_ids,
	},
	.probe  = ili9341_probe, 
	.remove = ili9341_remove,
};    

static int ili9341_init(void)
{
	int ret_val;

	ret_val = spi_register_driver(&ili9341_spi_driver);
	if (ret_val !=0)
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
