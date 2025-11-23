#include "Element_recognition.h"

// 帧级去抖参数：连续命中/连续未命中帧数
#define ZEBRA_CONFIRM_FRAMES 2u
#define ZEBRA_RELEASE_FRAMES 2u

// Portable 32-bit popcount
static int zebra_popcount32(uint32_t x)
{
    x = x - ((x >> 1) & 0x55555555u);
    x = (x & 0x33333333u) + ((x >> 2) & 0x33333333u);
    x = (x + (x >> 4)) & 0x0F0F0F0Fu;
    x = x + (x >> 8);
    x = x + (x >> 16);
    return (int)(x & 0x3Fu);
}

// Count horizontal black/white transitions in a single row (bit-packed)
static uint32_t zebra_row_edge_count(const uint32_t *row_bits, int width, int words_per_row)
{
    uint32_t count = 0;

    for (int i = 0; i < words_per_row; ++i)
    {
        uint32_t cur = row_bits[i];
        uint32_t next = (i + 1 < words_per_row) ? row_bits[i + 1] : 0u;
        uint32_t right = (cur >> 1) | (next << 31); // stitch neighbour bits
        uint32_t edges = cur ^ right;
        count += (uint32_t)zebra_popcount32(edges);
    }

    (void)width; // kept for possible future use
    return count;
}

// Row-to-row XOR count: smaller value means two neighbouring rows look similar (long stripes)
static uint32_t zebra_row_xor_count(const uint32_t *row_a_bits, const uint32_t *row_b_bits, int words_per_row)
{
    uint32_t count = 0;

    for (int i = 0; i < words_per_row; ++i)
    {
        uint32_t diff = row_a_bits[i] ^ row_b_bits[i];
        count += (uint32_t)zebra_popcount32(diff);
    }

    return count;
}

// Count number of foreground runs (bit=1) whose width is within [min_w, max_w]
static uint32_t zebra_count_stripes_row(const uint32_t *row_bits, int width, int min_w, int max_w)
{
    uint32_t stripes = 0;
    int run = 0;
    for (int x = 0; x < width; ++x)
    {
        uint32_t word = row_bits[x >> 5];
        uint32_t bit = (word >> (x & 31)) & 1u;
        if (bit)
        {
            run++;
        }
        else
        {
            if (run >= min_w && run <= max_w)
            {
                stripes++;
            }
            run = 0;
        }
    }
    // tail run
    if (run >= min_w && run <= max_w)
    {
        stripes++;
    }
    return stripes;
}

/*
 * 斑马线检测：指定行段内统计水平跳变，并要求相邻行相似（长条特征）。
 * bits         : 位打包后的二值图缓冲区
 * width/height : 图像尺寸（像素）
 * row_start/end: 检测行范围（包含端点，内部自动校正顺序与越界）
 * xor_threshold: 单行跳变阈值（自动限幅 10）
 * 返回值       : 1 检测到（watch.zebra_flag / ZebraInLine 已更新），0 未检测到
 */
uint8_t zebra_detect_bitpacked(const uint32_t *bits,
                               int width, int height,
                               uint8_t row_start, uint8_t row_end,
                               uint16_t xor_threshold)
{
    // 帧级去抖状态
    static uint8_t zebra_state = 0u;   // 0=无斑马线，1=有斑马线
    static uint8_t zebra_on_cnt = 0u;  // 连续命中计数
    static uint8_t zebra_off_cnt = 0u; // 连续未命中计数
    static int last_row = 0;

    if (bits == NULL || width <= 0 || height <= 0)
    {
        return 0;
    }

    // Clamp row range
    if (row_start >= (uint8_t)height)
    {
        row_start = (uint8_t)(height - 1);
    }
    if (row_end >= (uint8_t)height)
    {
        row_end = (uint8_t)(height - 1);
    }
    if (row_start > row_end)
    {
        uint8_t tmp = row_start;
        row_start = row_end;
        row_end = tmp;
    }

    int words_per_row = (width + 31) >> 5;
    uint8_t best_row = row_start;
    uint32_t best_row_edges = 0;
    uint32_t best_row_stripes = 0;
    uint8_t hit_in_rows = 0u;
    const uint32_t *prev_row_bits = NULL;

    for (uint8_t y = row_start; y <= row_end; ++y)
    {
        const uint32_t *row_bits = bits + (int)y * words_per_row;
        uint32_t row_edges = zebra_row_edge_count(row_bits, width, words_per_row);
        uint32_t row_xor = (prev_row_bits != NULL)
                               ? zebra_row_xor_count(row_bits, prev_row_bits, words_per_row)
                               : 0xFFFFFFFFu; // 首行无对比
        uint32_t row_stripes = zebra_count_stripes_row(row_bits, width, 2, 13); // 1<width<14 -> [2,13]

        // 行间相似性（首行无需约束），斑马线为长条，上下相似
        uint32_t row_xor_limit = (width > 0) ? (uint32_t)(width / 2) : 0u;
        if (row_xor_limit < 32u)
        {
            row_xor_limit = 32u;
        }
        uint8_t row_similarity_ok = (prev_row_bits == NULL) ? 1u
                                                            : (row_xor <= row_xor_limit ? 1u : 0u);

        // 单行跳变阈值：用户要求上限 10
        uint16_t thr = xor_threshold;
        if (thr > 10u)
        {
            thr = 10u;
        }

        // 取长补短：任意一行满足“跳变>=阈值 && 行间相似 && 白条数量>6 且宽度在 2~13”即视为命中
        if (row_edges >= (uint32_t)thr && row_similarity_ok && row_stripes > 6u)
        {
            hit_in_rows = 1u;
            if (row_stripes > best_row_stripes || (row_stripes == best_row_stripes && row_edges > best_row_edges))
            {
                best_row_stripes = row_stripes;
                best_row_edges = row_edges;
                best_row = y;
            }
        }

        prev_row_bits = row_bits;
    }

    uint8_t hit = hit_in_rows;

    // 帧级去抖：连续命中确认，连续未命中释放
    if (hit)
    {
        zebra_on_cnt++;
        zebra_off_cnt = 0u;
        if (zebra_on_cnt >= ZEBRA_CONFIRM_FRAMES)
        {
            zebra_state = 1u;
            last_row = (int)best_row;
        }
    }
    else
    {
        zebra_off_cnt++;
        zebra_on_cnt = 0u;
        if (zebra_off_cnt >= ZEBRA_RELEASE_FRAMES)
        {
            zebra_state = 0u;
        }
    }

    // 输出到全局标志
    watch.zebra_flag = zebra_state;
    if (zebra_state)
    {
        watch.ZebraInLine = last_row;
    }

    return zebra_state;
}
