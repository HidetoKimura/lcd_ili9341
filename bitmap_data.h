#ifndef bitmap_data_h
#define bitmap_data_h

typedef struct {
    uint16_t width;
    uint16_t height;
    uint8_t color_table_size;
    const color_t *color_table;
    const uint8_t *data;
} bitmap_t;

static const uint8_t slime_bmp_data[] = {
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 3, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 4, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 4, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 4, 5, 4, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 1, 7, 7, 5, 3, 4, 1, 1, 0, 0, 0, 0,
    0, 0, 1, 7, 7, 5, 5, 5, 4, 3, 4, 4, 1, 0, 0, 0,
    0, 1, 7, 7, 5, 5, 5, 4, 4, 3, 2, 2, 4, 1, 0, 0,
    1, 4, 7, 4, 4, 7, 4, 4, 3, 7, 3, 2, 2, 4, 1, 0,
    1, 4, 3, 4, 7, 1, 7, 3, 7, 1, 7, 2, 2, 4, 1, 0,
    1, 4, 2, 3, 4, 7, 3, 3, 3, 7, 3, 2, 2, 3, 1, 0,
    1, 4, 7, 2, 6, 3, 3, 3, 3, 2, 6, 2, 2, 3, 1, 0,
    0, 1, 4, 3, 2, 6, 6, 6, 6, 6, 2, 3, 3, 1, 0, 0,
    0, 0, 1, 1, 3, 3, 3, 3, 3, 3, 3, 1, 1, 0, 0, 0,
    0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

static const color_t slime_color_table[] = {
    // R    G    B
    {  0,   0,   0},
    { 10,  10,  10},
    { 32,  55, 100},
    { 48,  84, 150},
    {  0, 112, 192},
    {142, 169, 219},
    {198,  89,  17},
    {255, 255, 255},
};

static const bitmap_t slime = {
    .width = 16,
    .height = 16,
    .color_table_size = 8,
    .color_table = slime_color_table,
    .data = slime_bmp_data,
};

static const uint8_t corner_lt_bmp_data[] = {
    0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 1, 1, 1, 
    0, 0, 0, 0, 1, 1, 1, 1, 
    0, 0, 0, 1, 1, 1, 0, 0, 
    0, 0, 0, 1, 1, 0, 0, 0, 
    0, 0, 0, 1, 1, 0, 0, 0, 
 
};

static const uint8_t corner_rt_bmp_data[] = {
    0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 
    1, 1, 1, 0, 0, 0, 0, 0, 
    1, 1, 1, 1, 0, 0, 0, 0, 
    0, 0, 1, 1, 1, 0, 0, 0, 
    0, 0, 0, 1, 1, 0, 0, 0, 
    0, 0, 0, 1, 1, 0, 0, 0, 
 
};

static const uint8_t corner_rb_bmp_data[] = {
    0, 0, 0, 1, 1, 0, 0, 0, 
    0, 0, 0, 1, 1, 0, 0, 0, 
    0, 0, 1, 1, 1, 0, 0, 0, 
    1, 1, 1, 1, 0, 0, 0, 0, 
    1, 1, 1, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 
    
};

static const uint8_t corner_lb_bmp_data[] = {
    0, 0, 0, 1, 1, 0, 0, 0, 
    0, 0, 0, 1, 1, 0, 0, 0, 
    0, 0, 0, 1, 1, 1, 0, 0, 
    0, 0, 0, 0, 1, 1, 1, 1, 
    0, 0, 0, 0, 0, 1, 1, 1, 
    0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 
    
};

static const uint8_t v_line_bmp_data[] = {
    0, 0, 0, 1, 1, 0, 0, 0, 
    0, 0, 0, 1, 1, 0, 0, 0, 
    0, 0, 0, 1, 1, 0, 0, 0, 
    0, 0, 0, 1, 1, 0, 0, 0, 
    0, 0, 0, 1, 1, 0, 0, 0, 
    0, 0, 0, 1, 1, 0, 0, 0, 
    0, 0, 0, 1, 1, 0, 0, 0, 
    0, 0, 0, 1, 1, 0, 0, 0, 
    
};

static const uint8_t h_line_bmp_data[] = {
    0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 
    1, 1, 1, 1, 1, 1, 1, 1, 
    1, 1, 1, 1, 1, 1, 1, 1, 
    0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0,     
};


static const color_t mark_color_table[] = {
    // R    G    B
    {  0,   0,   0},
    {255, 255, 255},
};

static const bitmap_t corner_lt = {
    .width  = 8,
    .height = 8,
    .color_table_size = 2,
    .color_table = mark_color_table,
    .data = corner_lt_bmp_data,
};

static const bitmap_t corner_rt = {
    .width  = 8,
    .height = 8,
    .color_table_size = 2,
    .color_table = mark_color_table,
    .data = corner_rt_bmp_data,
};

static const bitmap_t corner_rb = {
    .width  = 8,
    .height = 8,
    .color_table_size = 2,
    .color_table = mark_color_table,
    .data = corner_rb_bmp_data,
};

static const bitmap_t corner_lb = {
    .width  = 8,
    .height = 8,
    .color_table_size = 2,
    .color_table = mark_color_table,
    .data = corner_lb_bmp_data,
};

static const bitmap_t v_line = {
    .width  = 8,
    .height = 8,
    .color_table_size = 2,
    .color_table = mark_color_table,
    .data = v_line_bmp_data,
};

static const bitmap_t h_line = {
    .width  = 8,
    .height = 8,
    .color_table_size = 2,
    .color_table = mark_color_table,
    .data = h_line_bmp_data,
};


#endif