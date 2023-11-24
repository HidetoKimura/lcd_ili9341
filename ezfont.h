#ifndef __EZFONT_H__
#define __EZFONT_H__

#include <linux/types.h>

int ezfont_conv_str_utf8_to_utf16(char *p_utf8_str, uint16_t *p_utf16_str, size_t utf16_size);
bool ezfont_get_fontdata_by_utf16(uint16_t utf16, bool h2z, uint8_t *font_data, size_t font_data_size);
char *ezfont_get_fontdata_by_utf8(char *p_utf8, bool h2z, uint8_t *font_data, size_t font_data_size);

#endif /* __EZFONT_H__ */