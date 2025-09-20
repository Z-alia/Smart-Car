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

typedef struct Image
{    
    uint16_t output_image[120][188];      // 输出图像（边缘检测结果）
    uint16_t original_image[120][188];    // 原始输入图像
    uint8_t x_points[120];                // X坐标点数组
    uint8_t y_points[120];                // Y坐标点数组
    
    // 新增：位打包工作缓冲区（内部使用，避免外部分配）
    uint32_t tmp1_bits[((188 + 31) >> 5) * 120];  // 工作缓冲区1
    uint32_t tmp2_bits[((188 + 31) >> 5) * 120];  // 工作缓冲区2
    uint32_t tmp3_bits[((188 + 31) >> 5) * 120];  // 工作缓冲区3
    uint32_t out_bits[((188 + 31) >> 5) * 120];   // 输出缓冲区
} Image;
extern struct Image image_buf;
//结构体句柄初始化
void image_process_init(Image *image);

/* 每行需要的 32bit word 数（例：width=188 → 6 个 word） */
MBP_INLINE int words_per_row(int width) { return (width + 31) >> 5; }

/* 总 word 数（height × words_per_row） */
MBP_INLINE int total_words(int width, int height) { return words_per_row(width) * height; }

/* 将 u16 二值图（0/非0）打包为位域（bit=1 为前景） */
void pack_binary_u16_to_bits(const uint16_t* src, int width, int height, int src_stride_pixels,
                             uint32_t* dst_bits);

/* 将位域解包为 u16（bit=1 → 0xFFFF，bit=0 → 0） */
void unpack_bits_to_binary_u16(const uint32_t* src_bits, int width, int height,
                               uint16_t* dst, int dst_stride_pixels);

/* 3×3 腐蚀/膨胀（位打包） */
void erode3x3_bitpacked(const uint32_t* src_bits, uint32_t* dst_bits, int width, int height);
void dilate3x3_bitpacked(const uint32_t* src_bits, uint32_t* dst_bits, int width, int height);

/* 二值内部梯度：output = clean & ~erode(clean) */
void internal_gradient_bitpacked(const uint32_t* clean_bits, const uint32_t* eroded_bits,
                                 uint32_t* output_bits, int width, int height);

/* 高层流水线：开(腐->膨) → 闭(膨->腐) → 内部梯度（输出单像素边缘） */
void precise_edge_detection_bitpacked(const uint32_t* src_bits,
                                      uint32_t* tmp1_bits,
                                      uint32_t* tmp2_bits,
                                      uint32_t* out_bits,
                                      int width, int height);


/* 适配器：直接以 u16 二值输入，输出 u16（0/0xFFFF） */
void precise_edge_detection_u16_binary_adapter(const uint16_t* src_u16,
                                               int width, int height, int src_stride_pixels,
                                               uint16_t* dst_u16, int dst_stride_pixels,
                                               uint32_t* tmp1_bits,
                                               uint32_t* tmp2_bits,
                                               uint32_t* tmp3_bits,
                                               uint32_t* out_bits);

/* 支持 Image 结构体的适配器 */
void precise_edge_detection_image_adapter(Image* image_buf, int width, int height);

#ifdef __cplusplus
}
#endif

#endif /* MORPH_BINARY_BITPACKED_H */