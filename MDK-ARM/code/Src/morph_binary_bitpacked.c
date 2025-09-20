#include "morph_binary_bitpacked.h"
#include <string.h>

#ifndef RESTRICT
#if defined(__GNUC__)
#define RESTRICT __restrict__
#else
#define RESTRICT
#endif
#endif

// 计算每行尾部有效位掩码：用于清除行尾（不足 32 位）的高位，防止脏位参与计算
static inline uint32_t last_word_mask(int width) {
    int rem = width & 31;
    return rem ? ((1u << rem) - 1u) : 0xFFFFFFFFu;
}

// 将 u16 二值图打包到位域：非零即 1（bit=1 表前景）
// 注意：本函数假设一行内从左到右依次对应 word 内的 bit0..bit31（LSB→MSB）
void pack_binary_u16_to_bits(const uint16_t* RESTRICT src, int width, int height, int src_stride_pixels,
                             uint32_t* RESTRICT dst_bits) {
    int wpw = words_per_row(width);
    uint32_t tail = last_word_mask(width);
    for (int y = 0; y < height; y++) {
        const uint16_t* s = src + (size_t)y * src_stride_pixels;
        uint32_t* d = dst_bits + (size_t)y * wpw;
        int x = 0; // 当前像素索引
        for (int i = 0; i < wpw; i++) {
            uint32_t w = 0;
            // 将连续 32 个像素压成一个 32 位 word
            for (int b = 0; b < 32 && x < width; b++, x++) {
                if (s[x]) w |= (1u << b);  // 非零即前景 1
            }
            d[i] = w;
        }
        // 清除行尾无效位，避免后续位运算引入假信号
        d[wpw - 1] &= tail;
    }
}

// 将位域解包为 u16：bit=1 → 0xFFFF，bit=0 → 0
void unpack_bits_to_binary_u16(const uint32_t* RESTRICT src_bits, int width, int height,
                               uint16_t* RESTRICT dst, int dst_stride_pixels) {
    int wpw = words_per_row(width);
    for (int y = 0; y < height; y++) {
        const uint32_t* s = src_bits + (size_t)y * wpw;
        uint16_t* d = dst + (size_t)y * dst_stride_pixels;
        int x = 0;
        for (int i = 0; i < wpw; i++) {
            uint32_t w = s[i];
            for (int b = 0; b < 32 && x < width; b++, x++) {
                d[x] = (w & (1u << b)) ? 0xFFFFu : 0u;
            }
        }
    }
}

// 清除边界一圈像素（防止 3×3 核在边缘处因外部隐含 0/复制策略不同导致伪影）
// - 顶/底行：整行置零
// - 中间行：清最左列与最右列的 1 个像素位
static inline void clear_borders_bitpacked_row(uint32_t* rowWords, int width, int wpw, int isTopOrBottom) {
    if (isTopOrBottom) {
        memset(rowWords, 0, (size_t)wpw * sizeof(uint32_t));
        return;
    }
    // 清最左列 bit0
    rowWords[0] &= ~1u;
    // 清最右列 bit(width-1)
    int lastIdx = wpw - 1;
    int bitPos = (width - 1) & 31;
    rowWords[lastIdx] &= ~(1u << bitPos);
}

// 3×3 腐蚀（位打包版）
// 实现思路：
// 1) 对于当前行及其上下相邻行，分别计算“水平 1×3 最小（对二值相当于逐位 AND）”：
//      (左移一位 | 邻接 word 拼接的高位) & 原位 & (右移一位 | 邻接 word 拼接的低位)
// 2) 将三行的水平结果逐位 AND，得到 3×3 腐蚀结果。
// 3) 每行末尾应用 tail 掩码，最后统一清边界一圈像素。
void erode3x3_bitpacked(const uint32_t* RESTRICT src_bits, uint32_t* RESTRICT dst_bits, int width, int height) {
    int wpw = words_per_row(width);
    uint32_t tail = last_word_mask(width);

    for (int y = 0; y < height; y++) {
        // 取上下行指针（越界则视为全 0 行）
        int yPrev = (y > 0) ? (y - 1) : -1;
        int yNext = (y + 1 < height) ? (y + 1) : -1;

        const uint32_t* rowA = (yPrev >= 0) ? (src_bits + (size_t)yPrev * wpw) : NULL;
        const uint32_t* rowB = src_bits + (size_t)y * wpw;
        const uint32_t* rowC = (yNext >= 0) ? (src_bits + (size_t)yNext * wpw) : NULL;

        uint32_t* out = dst_bits + (size_t)y * wpw;

        for (int i = 0; i < wpw; i++) {
            // 取左右相邻 word（边界处用 0）
            uint32_t aL = (rowA && i > 0)       ? rowA[i - 1] : 0u;
            uint32_t aC = (rowA)                ? rowA[i]     : 0u;
            uint32_t aR = (rowA && i + 1 < wpw) ? rowA[i + 1] : 0u;

            uint32_t bL = (i > 0)               ? rowB[i - 1] : 0u;
            uint32_t bC = rowB[i];
            uint32_t bR = (i + 1 < wpw)         ? rowB[i + 1] : 0u;

            uint32_t cL = (rowC && i > 0)       ? rowC[i - 1] : 0u;
            uint32_t cC = (rowC)                ? rowC[i]     : 0u;
            uint32_t cR = (rowC && i + 1 < wpw) ? rowC[i + 1] : 0u;

            // 水平 1×3（按位 AND），并处理跨 word 的拼接位
            // 左邻像素： (aC << 1) | (aL >> 31)  —— aC 左移1位，aL 最高位拼到 aC 的最低位
            // 右邻像素： (aC >> 1) | (aR << 31) —— aC 右移1位，aR 最低位拼到 aC 的最高位
            uint32_t a_h = ((aC << 1) | (aL >> 31)) & aC & ((aC >> 1) | (aR << 31));
            uint32_t b_h = ((bC << 1) | (bL >> 31)) & bC & ((bC >> 1) | (bR << 31));
            uint32_t c_h = ((cC << 1) | (cL >> 31)) & cC & ((cC >> 1) | (cR << 31));

            // 纵向 3×1（按位 AND）
            uint32_t res = a_h & b_h & c_h;

            // 屏蔽行尾无效位
            if (i == wpw - 1) res &= tail;
            out[i] = res;
        }

        // 清边界一圈像素：行 0/行 h-1 整行清零；中间行清最左/最右 1 列
        clear_borders_bitpacked_row(out, width, wpw, (y == 0 || y == height - 1));
    }
}

// 3×3 膨胀（位打包版）
// 与腐蚀类似，只是把 AND 换成 OR。
void dilate3x3_bitpacked(const uint32_t* RESTRICT src_bits, uint32_t* RESTRICT dst_bits, int width, int height) {
    int wpw = words_per_row(width);
    uint32_t tail = last_word_mask(width);

    for (int y = 0; y < height; y++) {
        int yPrev = (y > 0) ? (y - 1) : -1;
        int yNext = (y + 1 < height) ? (y + 1) : -1;

        const uint32_t* rowA = (yPrev >= 0) ? (src_bits + (size_t)yPrev * wpw) : NULL;
        const uint32_t* rowB = src_bits + (size_t)y * wpw;
        const uint32_t* rowC = (yNext >= 0) ? (src_bits + (size_t)yNext * wpw) : NULL;

        uint32_t* out = dst_bits + (size_t)y * wpw;

        for (int i = 0; i < wpw; i++) {
            uint32_t aL = (rowA && i > 0)       ? rowA[i - 1] : 0u;
            uint32_t aC = (rowA)                ? rowA[i]     : 0u;
            uint32_t aR = (rowA && i + 1 < wpw) ? rowA[i + 1] : 0u;

            uint32_t bL = (i > 0)               ? rowB[i - 1] : 0u;
            uint32_t bC = rowB[i];
            uint32_t bR = (i + 1 < wpw)         ? rowB[i + 1] : 0u;

            uint32_t cL = (rowC && i > 0)       ? rowC[i - 1] : 0u;
            uint32_t cC = (rowC)                ? rowC[i]     : 0u;
            uint32_t cR = (rowC && i + 1 < wpw) ? rowC[i + 1] : 0u;

            // 水平 1×3（按位 OR）
            uint32_t a_h = ((aC << 1) | (aL >> 31)) | aC | ((aC >> 1) | (aR << 31));
            uint32_t b_h = ((bC << 1) | (bL >> 31)) | bC | ((bC >> 1) | (bR << 31));
            uint32_t c_h = ((cC << 1) | (cL >> 31)) | cC | ((cC >> 1) | (cR << 31));

            // 纵向 3×1（按位 OR）
            uint32_t res = a_h | b_h | c_h;

            if (i == wpw - 1) res &= tail;
            out[i] = res;
        }

        clear_borders_bitpacked_row(out, width, wpw, (y == 0 || y == height - 1));
    }
}

// 二值内部梯度：output = clean & ~erode(clean)
// 注：若想要“标准梯度（约两像素宽）”，可改为 output = dilate(clean) & ~erode(clean)。
void internal_gradient_bitpacked(const uint32_t* RESTRICT clean_bits, const uint32_t* RESTRICT eroded_bits,
                                 uint32_t* RESTRICT output_bits, int width, int height) {
    int n = total_words(width, height);
    for (int i = 0; i < n; i++) {
        output_bits[i] = clean_bits[i] & ~eroded_bits[i];
    }
    // 与形态学保持一致，清边界一圈
    int wpw = words_per_row(width);
    for (int y = 0; y < height; y++) {
        clear_borders_bitpacked_row(output_bits + (size_t)y * wpw, width, wpw, (y == 0 || y == height - 1));
    }
}

// 高层流水线：开运算 -> 闭运算 -> 内部梯度（最终得到单像素边缘）
void precise_edge_detection_bitpacked(const uint32_t* RESTRICT src_bits,
                                      uint32_t* RESTRICT tmp1_bits,
                                      uint32_t* RESTRICT tmp2_bits,
                                      uint32_t* RESTRICT out_bits,
                                      int width, int height) {
    // 开运算：先腐蚀（去小噪点）再膨胀（恢复主体形状）
    erode3x3_bitpacked(src_bits, tmp1_bits, width, height);
    dilate3x3_bitpacked(tmp1_bits, tmp2_bits, width, height);

    // 闭运算：先膨胀（填小孔/断裂）再腐蚀（恢复边界）
    dilate3x3_bitpacked(tmp2_bits, tmp1_bits, width, height);
    erode3x3_bitpacked(tmp1_bits, tmp2_bits, width, height); // tmp2 变为“干净图像”

    // 内部梯度：clean - erode(clean)（二值下等价 AND NOT）
    erode3x3_bitpacked(tmp2_bits, tmp1_bits, width, height);
    internal_gradient_bitpacked(tmp2_bits, tmp1_bits, out_bits, width, height);
}

// 适配器：直接用 u16 二值输入（0/非0），输出 u16（0/0xFFFF）
// 内部流程：打包 → 形态学流水线 → 解包
void precise_edge_detection_u16_binary_adapter(const uint16_t* RESTRICT src_u16,
                                               int width, int height, int src_stride_pixels,
                                               uint16_t* RESTRICT dst_u16, int dst_stride_pixels,
                                               uint32_t* RESTRICT tmp1_bits,
                                               uint32_t* RESTRICT tmp2_bits,
                                               uint32_t* RESTRICT tmp3_bits,
                                               uint32_t* RESTRICT out_bits) {
    // 1) 打包：将 0/非0 u16 压到 bit-packed（非零→1）
    pack_binary_u16_to_bits(src_u16, width, height, src_stride_pixels, tmp3_bits);

    // 2) 位打包形态学流水线（开→闭→内部梯度）
    precise_edge_detection_bitpacked(tmp3_bits, tmp1_bits, tmp2_bits, out_bits, width, height);

    // 3) 解包：得到 0/0xFFFF 的 u16 边缘图
    unpack_bits_to_binary_u16(out_bits, width, height, dst_u16, dst_stride_pixels);
}