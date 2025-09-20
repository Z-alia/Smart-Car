#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "morph_binary_bitpacked.h"


// 分段属性：若你的链接脚本没有这些段，可先删除这些宏定义中的 section(...) 部分
#ifndef RAM_DTCM
#define RAM_DTCM __attribute__((section(".RAM_DTCM"), aligned(32)))
#endif

#ifndef RAM_D1
#define RAM_D1   __attribute__((section(".RAM_D1"), aligned(32)))
#endif

// 输入/输出帧（u16 二值 0/非0）：建议放 AXI SRAM（或 D2 SRAM），Cache 生效
RAM_D1 static uint16_t original_image[Display_Height][Display_Width];
RAM_D1 static uint16_t edge_image[Display_Height][Display_Width];

// 位打包工作缓冲（最热点）：建议放 DTCM（绕过 D-Cache，延迟最低）
#define WORDS_PER_ROW  ((Display_Width + 31) >> 5)
#define TOTAL_WORDS    (WORDS_PER_ROW * Display_Height)

RAM_DTCM static uint32_t bits_tmp1[TOTAL_WORDS];
RAM_DTCM static uint32_t bits_tmp2[TOTAL_WORDS];
RAM_DTCM static uint32_t bits_tmp3[TOTAL_WORDS];
RAM_DTCM static uint32_t bits_out[TOTAL_WORDS];



// // 二值化
// static void threshold_to_binary(uint16_t threshold){
//     for (int j = 0; j < Display_Height; ++j) {
//         for (int i = 0; i < Display_Width; ++i) {
//             uint16_t temp = mt9v03x_image[j][i];
//             if (temp < (threshold << 8)) {
//                 original_image[j][i] = 0;
//             } else {
//                 original_image[j][i] = 0xFFFF; // 非零即前景
//             }
//         }
//     }
// }

int main(void) {

    // 3) 准备输入：任选一种（阈值化 or 测试图）
    // 3.1 使用你的阈值化流程（与位打包完全兼容）
    threshold_to_binary(100);
    // 3.2 或者使用内置测试图样
    // 4) 流水线处理：打包 → 开/闭 → 内部梯度 → 解包
    uint32_t t0 = DWT_GetCycles();

    precise_edge_detection_u16_binary_adapter(
        &original_image[0][0], Display_Width, Display_Height, Display_Width,
        &edge_image[0][0],     Display_Width,
        bits_tmp1, bits_tmp2, bits_tmp3, bits_out);

    uint32_t t1 = DWT_GetCycles();
    uint32_t cycles = t1 - t0;

    // 5) 结果统计（注意：printf 较慢，尽量不要包含在测时窗口内）
    int edge_count = 0;
    for (int y = 0; y < IMG_H; y++) {
        for (int x = 0; x < IMG_W; x++) {
            if (edge_image[y][x]) edge_count++;
        }
    }

    while (1) {
        // 主循环（此处可接入后续处理/显示/通信）
    }
}