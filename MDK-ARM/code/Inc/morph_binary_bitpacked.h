#ifndef MORPH_BINARY_BITPACKED_H
#define MORPH_BINARY_BITPACKED_H

/*
  本模块实现“位打包”的二值 3×3 形态学运算（腐蚀/膨胀）和基于开-闭-内部梯度的单像素边缘提取。

  设计说明：
  - 位打包：用 uint32_t 的 32 个 bit 表示 32 个像素；bit=1 表示前景。
  - 存储布局：逐行存放，每行 words_per_row(width) 个 uint32_t；行尾不足 32 位用掩码屏蔽。
  - 二值输入：假设源像素为 uint16_t 且 0/非0，打包时非零即 1；解包输出 0/0xFFFF。
  - 边界策略：统一清除最外一圈像素，避免边界伪影。
*/

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#ifdef __cplusplus
extern "C" {
#endif

/* 兼容 C89/C99 的 inline 定义 */
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 199901L)
  #define MBP_INLINE static inline
#else
  #define MBP_INLINE static
#endif



/* 每行需要的 32bit word 数（例：width=188 → 6 个 word） */
MBP_INLINE int words_per_row(int width) { return (width + 31) >> 5; }

/* 总 word 数（height × words_per_row） */
MBP_INLINE int total_words(int width, int height) { return words_per_row(width) * height; }

/* 将 u16 二值图（0/非0）打包为位域（bit=1 为前景） */
void pack_binary_u16_to_bits(const uint16_t* src, int width, int height, int src_stride_pixels,
                             uint32_t* dst_bits);

/* 将 u8 二值图（0/非0）打包为位域（bit=1 为前景） */
void pack_binary_u8_to_bits(const uint8_t* src, int width, int height, int src_stride_pixels,
                            uint32_t* dst_bits);

/* 将位域解包为 u16（bit=1 → 0xFFFF，bit=0 → 0） */
void unpack_bits_to_binary_u16(const uint32_t* src_bits, int width, int height,
                               uint16_t* dst, int dst_stride_pixels);

/* 将位域解包为 u8（bit=1 → 0xFF，bit=0 → 0） */
void unpack_bits_to_binary_u8(const uint32_t* src_bits, int width, int height,
                              uint8_t* dst, int dst_stride_pixels);

/* 3×3 腐蚀/膨胀（位打包） */
void erode3x3_bitpacked(const uint32_t* src_bits, uint32_t* dst_bits, int width, int height);
void dilate3x3_bitpacked(const uint32_t* src_bits, uint32_t* dst_bits, int width, int height);

/* 二值内部梯度：output = clean & ~erode(clean) */
void internal_gradient_bitpacked(const uint32_t* clean_bits, const uint32_t* eroded_bits,
                                 uint32_t* output_bits, int width, int height);

// 高层流水线0：闭运算
void close_bitpacked(const uint32_t* src_bits, uint32_t* tmp1_bits, uint32_t* out_bits, int width, int height);

/* 高层流水线1：开(腐->膨) → 闭(膨->腐) */
void open_close_bitpacked(const uint32_t* src_bits, uint32_t* tmp1_bits, uint32_t* out_bits, int width, int height);

// 高层流水线2：开运算 -> 闭运算 -> 内部梯度（最终得到单像素边缘）
void precise_edge_detection_bitpacked(const uint32_t* src_bits, uint32_t* tmp1_bits, uint32_t* out_bits, int width, int height);

/* 适配器：对 u16 二值图进行形态学清洗（开运算+闭运算） */
void morph_clean_u16_binary_adapter(const uint16_t* src_u16,
                                    int width, int height,
                                    uint16_t* dst_u16);

/* 适配器：对 u8 二值图进行形态学清洗（开运算+闭运算） */
void morph_clean_u8_binary_adapter(const uint8_t* src_u8,
                                   int width, int height,
                                   uint8_t* dst_u8);

/* ============================================================================
 * 八邻域巡线辅助函数（位打包格式）
 * ============================================================================
 * 这些函数提供在压缩/位打包状态下进行八邻域操作的能力，适用于：
 * - 边缘跟踪（巡线）
 * - 连通性分析
 * - 骨架提取
 * - 其他需要访问邻域像素的图像处理算法
 * 
 * 八邻域定义（相对于中心像素(x,y)的位置）：
 *   7  0  1
 *   6  *  2     * = 中心像素 (x, y)
 *   5  4  3
 * 
 * 索引对应方向：
 *   0:上(0,-1),    1:上右(1,-1),  2:右(1,0),   3:右下(1,1),
 *   4:下(0,1),     5:下左(-1,1),  6:左(-1,0),  7:左上(-1,-1)
 */

/* 获取位打包图像中指定位置的像素值（0 或 1）
 * 参数：
 *   bits   - 位打包图像数据
 *   x, y   - 像素坐标
 *   width  - 图像宽度（像素）
 *   height - 图像高度（像素）
 * 返回：像素值（0 或 1），越界返回 0
 */
MBP_INLINE int get_pixel_bitpacked(const uint32_t* bits, int x, int y, int width, int height) {
    if (x < 0 || x >= width || y < 0 || y >= height) return 0;
    int wpw = words_per_row(width);
    int word_idx = x >> 5;           // x / 32
    int bit_pos = x & 31;            // x % 32
    return (bits[y * wpw + word_idx] >> bit_pos) & 1u;
}

/* 设置位打包图像中指定位置的像素值（0 或 1）
 * 参数：
 *   bits   - 位打包图像数据
 *   x, y   - 像素坐标
 *   width  - 图像宽度（像素）
 *   height - 图像高度（像素）
 *   value  - 要设置的值（0 或非0，非0视为1）
 */
MBP_INLINE void set_pixel_bitpacked(uint32_t* bits, int x, int y, int width, int height, int value) {
    if (x < 0 || x >= width || y < 0 || y >= height) return;
    int wpw = words_per_row(width);
    int word_idx = x >> 5;
    int bit_pos = x & 31;
    if (value) {
        bits[y * wpw + word_idx] |= (1u << bit_pos);   // 设置为 1
    } else {
        bits[y * wpw + word_idx] &= ~(1u << bit_pos);  // 设置为 0
    }
}

/* 获取八邻域像素值（按顺序：上、上右、右、右下、下、下左、左、左上）
 * 参数：
 *   bits      - 位打包图像数据
 *   x, y      - 中心像素坐标
 *   width     - 图像宽度（像素）
 *   height    - 图像高度（像素）
 *   neighbors - 输出数组[8]，存储8个邻域像素值（0或1），由调用者分配
 * 
 * 用途示例：
 *   int neighbors[8];
 *   get_8neighbors_bitpacked(bits, x, y, width, height, neighbors);
 *   // neighbors[0] = 上方像素, neighbors[2] = 右方像素, 等等
 */
void get_8neighbors_bitpacked(const uint32_t* bits, int x, int y, int width, int height, int* neighbors);

/* 统计八邻域中前景像素的数量
 * 参数：
 *   bits   - 位打包图像数据
 *   x, y   - 中心像素坐标
 *   width  - 图像宽度（像素）
 *   height - 图像高度（像素）
 * 返回：邻域中值为1的像素数量（0-8）
 * 
 * 用途示例：
 *   - 判断是否为边界点（count < 8）
 *   - 判断是否为端点（count == 1）
 *   - 判断是否为分支点（count >= 3）
 */
int count_8neighbors_bitpacked(const uint32_t* bits, int x, int y, int width, int height);

#ifdef __cplusplus
}
#endif

#endif /* MORPH_BINARY_BITPACKED_H */