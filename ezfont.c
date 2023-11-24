#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>

#include "ezfont.h"
#include "misakiUTF16FontData.h"

#define FONT_TABLE_SIZE (sizeof misaki_font_table / sizeof misaki_font_table[0])
#define FONT_LEN 7
#define FONT_WHITE_SQUARE 0x25a1

// Half-width kana to full-width kana conversion table
static const uint8_t _hkremap[] = {
    0x02, 0x0C, 0x0D, 0x01, 0xFB, 0xF2, 0xA1, 0xA3, 0xA5, 0xA7, 0xA9, 0xE3, 0xE5, 0xE7, 0xC3, 0xFD,
    0xA2, 0xA4, 0xA6, 0xA8, 0xAA, 0xAB, 0xAD, 0xAF, 0xB1, 0xB3, 0xB5, 0xB7, 0xB9, 0xBB, 0xBD, 0xBF,
    0xC1, 0xC4, 0xC6, 0xC8, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF, 0xD2, 0xD5, 0xD8, 0xDB, 0xDE, 0xDF,
    0xE0, 0xE1, 0xE2, 0xE4, 0xE6, 0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xEF, 0xF3, 0x9B, 0x9C};

// Read n bytes
// rcvdata: Address to store the read data
// n      : Number of bytes to read
// Returns: Number of bytes read
static uint8_t *read_font_rawdata(int code, uint8_t *buf, size_t size)
{
        if (size > FONT_LEN)
        {
                size = FONT_LEN;
        }
        if (code > FONT_TABLE_SIZE)
        {
                return NULL;
        }
        memcpy(buf, misaki_font_data + size * code, size);
        return buf;
}

// Font code search
// (Refer to the table on ROM with the code and retrieve the font code)
// ucode(in) UTF-16 code
// Returns the font code (0-FONT_TABLE_SIZE) if the corresponding font exists
// Returns -1 if the corresponding font does not exist
static int findcode(uint16_t ucode)
{
        int t_p = 0;                   // Upper limit of search range
        int e_p = FONT_TABLE_SIZE - 1; // Lower limit of search range
        int pos;
        uint16_t d = 0;
        int flg_stop = 0;

        while (true)
        {
                pos = t_p + ((e_p - t_p + 1) / 2);
                d = misaki_font_table[pos];
                if (d == ucode)
                {
                        // Match found
                        flg_stop = 1;
                        break;
                }
                else if (ucode > d)
                {
                        // Greater than
                        t_p = pos + 1;
                        if (t_p > e_p)
                        {
                                break;
                        }
                }
                else
                {
                        // Less than
                        e_p = pos - 1;
                        if (e_p < t_p)
                                break;
                }
        }
        if (!flg_stop)
        {
                return -1;
        }
        return pos;
}

// Check if the UTF16 code is a half-width kana
static int is_hkana(uint16_t ucode)
{
        if (ucode >= 0xFF61 && ucode <= 0xFF9F) {
                return ucode - 0xFF61;
        }
        else {
                return 0;
        }
}

// Convert half-width kana to full-width kana
static uint16_t hkana2kana(uint16_t ucode)
{
        int pos;
        pos = is_hkana(ucode);
        if (pos == 0) {
                return ucode;
        }
        return (uint16_t)(_hkremap[pos] + 0x3000);
}

// Convert UTF16 half-width code to UTF16 full-width code
// (Returns the original code if conversion is not possible)
//   utf16(in): UTF16 character code
//   Returns: Converted code
static uint16_t conv_utf16_han2zen(uint16_t utf16)
{

        utf16 = hkana2kana(utf16);

        if (utf16 > 0xff || utf16 < 0x20) {
                return utf16;
        }

        switch (utf16) {
        case 0x005C:
        case 0x00A2:
        case 0x00A3:
        case 0x00A7:
        case 0x00A8:
        case 0x00AC:
        case 0x00B0:
        case 0x00B1:
        case 0x00B4:
        case 0x00B6:
        case 0x00D7:
        case 0x00F7:
                return utf16;
        case 0x00A5:
                return 0xFFE5;
        case 0x0020:
                return 0x3000;
        case 0x0021:
                return 0xFF01;
        case 0x0022:
                return 0x201D;
        case 0x0023:
                return 0xFF03;
        case 0x0024:
                return 0xFF04;
        case 0x0025:
                return 0xFF05;
        case 0x0026:
                return 0xFF06;
        case 0x0027:
                return 0x2019;
        case 0x0028:
                return 0xFF08;
        case 0x0029:
                return 0xFF09;
        case 0x002A:
                return 0xFF0A;
        case 0x002B:
                return 0xFF0B;
        case 0x002C:
                return 0xFF0C;
        case 0x002D:
                return 0x2212;
        case 0x002E:
                return 0xFF0E;
        }
        return utf16 - 0x2F + 0xFF0F;
}

// Convert UTF8 characters (1-3 bytes) to UTF16
// p_utf8(in):   Address of the UTF8 string
// p_utf16(out): Address to store the UTF16 string
// Returns: Number of bytes processed during conversion
static int char_utf8_to_utf16(char *p_utf8, uint16_t *p_utf16)
{
        uint8_t bytes[3];
        uint16_t unicode16;

        bytes[0] = *p_utf8++;
        if (bytes[0] < 0x80)
        {
                *p_utf16 = bytes[0];
                return 1;
        }
        bytes[1] = *p_utf8++;
        if (bytes[0] >= 0xC0 && bytes[0] < 0xE0)
        {
                unicode16 = 0x1f & bytes[0];
                *p_utf16 = (unicode16 << 6) + (0x3f & bytes[1]);
                return 2;
        }

        bytes[2] = *p_utf8++;
        if (bytes[0] >= 0xE0 && bytes[0] < 0xF0)
        {
                unicode16 = 0x0f & bytes[0];
                unicode16 = (unicode16 << 6) + (0x3f & bytes[1]);
                *p_utf16 = (unicode16 << 6) + (0x3f & bytes[2]);
                return 3;
        }

        return 0;
}
// Convert UTF8 string to UTF16 string
// p_utf8_str(in):   UTF8 string
// p_utf16_str(out): UFT16 string
// utf16_size(in): UFT16 string size
// Returns: UFT16 string length (returns -1 on conversion failure)
int ezfont_conv_str_utf8_to_utf16(char *p_utf8_str, uint16_t *p_utf16_str, size_t utf16_size)
{
        int n;
        int len = 0;

        while (*p_utf8_str)
        {
                n = char_utf8_to_utf16(p_utf8_str, p_utf16_str);
                if (n == 0)
                        return -1;
                p_utf8_str += n;
                if (utf16_size <= 0)
                        break;
                utf16_size--;
                len++;
                p_utf16_str++;
        }
        return len;
}

// Get the Misaki font data of 8 bytes corresponding to UTF16
//   font_data(out): Address to store the font data
//   utf16(in): UTF16 code
//   Returns: true if successful, false otherwise
bool ezfont_get_fontdata_by_utf16(uint16_t utf16, bool h2z, uint8_t *font_data, size_t font_data_size)
{

        int code;

        if (font_data == NULL)
        {
                return false;
        }

        if (font_data_size < FONT_LEN + 1)
        {
                return false;
        }

        if (h2z)
        {
                utf16 = conv_utf16_han2zen(utf16);
        }

        code = findcode(utf16);
        if (code < 0)
        {
                code = findcode(FONT_WHITE_SQUARE);
        }

        if (read_font_rawdata(code, font_data, FONT_LEN) == NULL)
        {
                return false;
        }

        font_data[FONT_LEN] = 0;

        return true;
}

// Get the font data of the specified UTF8 string
//   p_utf8(in) : UTF8 string
//   h2z(in)  : true: convert half-width characters to full-width, false: do not convert
//   font_data(out): Address to store the font data
//   Returns  : Next position of the string, returns NULL if retrieval fails
char *ezfont_get_fontdata_by_utf8(char *p_utf8, bool h2z, uint8_t *font_data, size_t font_data_size)
{
        uint16_t utf16;
        uint8_t n;
        bool result;

        if (p_utf8 == NULL)
                return NULL;

        if (font_data == NULL)
        {
                return NULL;
        }

        if (font_data_size < FONT_LEN + 1)
        {
                return NULL;
        }

        if (*p_utf8 == 0)
                return NULL;

        n = char_utf8_to_utf16(p_utf8, &utf16);
        if (n == 0)
                return NULL;
        result = ezfont_get_fontdata_by_utf16(utf16, h2z, font_data, font_data_size);
        if (result == false)
                return NULL;

        return (p_utf8 + n);
}

MODULE_LICENSE("GPL");