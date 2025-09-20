/*
  示例主程序（最小可跑版本）：
    - 启用 I/D Cache
    - 分配输入/输出帧与位打包缓冲（推荐放置：帧在 AXI SRAM，工作缓冲在 DTCM）
    - 执行：打包 → 开/闭 → 内部梯度 → 解包
    - 集成你的二值化阈值流程（0/0xFFFF），证明完全兼容
    - 使用 DWT 计时统计核心运算耗时（不含 printf）
    - 可选打印各缓冲的内存地址，确认放置区域
*/

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "stm32h7_cache_setup.h"
#include "morph_binary_bitpacked.h"
#include "mem_region_helper.h"

// 图像尺寸（与你的项目一致）
#define IMG_H 120
#define IMG_W 188

// 分段属性：若你的链接脚本没有这些段，可先删除这些宏定义中的 section(...) 部分
#ifndef RAM_DTCM
#define RAM_DTCM __attribute__((section(".RAM_DTCM"), aligned(32)))
#endif

#ifndef RAM_D1
#define RAM_D1   __attribute__((section(".RAM_D1"), aligned(32)))
#endif

// 输入/输出帧（u16 二值 0/非0）：建议放 AXI SRAM（或 D2 SRAM），Cache 生效
RAM_D1 static uint16_t original_image[IMG_H][IMG_W];
RAM_D1 static uint16_t edge_image[IMG_H][IMG_W];

// 位打包工作缓冲（最热点）：建议放 DTCM（绕过 D-Cache，延迟最低）
#define WORDS_PER_ROW  ((IMG_W + 31) >> 5)
#define TOTAL_WORDS    (WORDS_PER_ROW * IMG_H)

RAM_DTCM static uint32_t bits_tmp1[TOTAL_WORDS];
RAM_DTCM static uint32_t bits_tmp2[TOTAL_WORDS];
RAM_DTCM static uint32_t bits_tmp3[TOTAL_WORDS];
RAM_DTCM static uint32_t bits_out[TOTAL_WORDS];

// 用于打印地址所在内存区（可选）
static const char* mem_region_name(mem_region_t r){
    switch (r){
        case MEM_ITCM: return "ITCM";
        case MEM_DTCM: return "DTCM";
        case MEM_AXI_SRAM: return "AXI SRAM";
        case MEM_D2_SRAM1: return "D2 SRAM1";
        case MEM_D2_SRAM2: return "D2 SRAM2";
        case MEM_D2_SRAM3: return "D2 SRAM3";
        case MEM_D3_SRAM4: return "D3 SRAM4";
        case MEM_BKPSRAM: return "BKPSRAM";
        case MEM_FLASH: return "FLASH";
        case MEM_OSPI: return "OSPI";
        case MEM_FMC_NOR: return "FMC NOR/SRAM";
        case MEM_FMC_SDRAM: return "FMC SDRAM";
        default: return "UNKNOWN";
    }
}

// 你的阈值二值化（示例）：temp 与 threshold 的来源按你的工程实际替换
static void threshold_to_binary(uint16_t threshold){
    // 演示：这里随便构造 temp；在你的工程中用真实的灰度/幅值填 original_image
    for (int j = 0; j < IMG_H; ++j) {
        for (int i = 0; i < IMG_W; ++i) {
            uint16_t temp = (uint16_t)((i + j) & 0xFF) << 8; // 假数据：0..255 左移到高 8 位
            if (temp < (threshold << 8)) {
                original_image[j][i] = 0;
            } else {
                original_image[j][i] = 0xFFFF; // 非零即前景
            }
        }
    }
}

// 演示：构造一个小矩形轮廓（也可用作快速自检）
static void fill_test_pattern(void) {
    memset(original_image, 0, sizeof(original_image));
    for (int x = 10; x < 60; x++) {
        original_image[10][x] = 0xFFFF;
        original_image[30][x] = 0xFFFF;
    }
    for (int y = 10; y < 31; y++) {
        original_image[y][10] = 0xFFFF;
        original_image[y][59] = 0xFFFF;
    }
}

int main(void) {
    // 1) 启用 I/D Cache（一次性）
    H7_EnableCaches();

    // 2) 启用 DWT 计数器（用于测时）
    DWT_CycleCounterInit();

    // 3) 准备输入：任选一种（阈值化 or 测试图）
    // 3.1 使用你的阈值化流程（与位打包完全兼容）
    threshold_to_binary(100);
    // 3.2 或者使用内置测试图样
    // fill_test_pattern();

    // 可选：确认数组实际被链接到哪个内存区（调试期很有用）
    printf("original_image @ %p (%s)\r\n", (void*)original_image,
           mem_region_name(where_is_pointer(original_image)));
    printf("edge_image     @ %p (%s)\r\n", (void*)edge_image,
           mem_region_name(where_is_pointer(edge_image)));
    printf("bits_tmp1      @ %p (%s)\r\n", (void*)bits_tmp1,
           mem_region_name(where_is_pointer(bits_tmp1)));
    printf("bits_tmp2      @ %p (%s)\r\n", (void*)bits_tmp2,
           mem_region_name(where_is_pointer(bits_tmp2)));
    printf("bits_tmp3      @ %p (%s)\r\n", (void*)bits_tmp3,
           mem_region_name(where_is_pointer(bits_tmp3)));
    printf("bits_out       @ %p (%s)\r\n", (void*)bits_out,
           mem_region_name(where_is_pointer(bits_out)));

    // 4) 流水线处理：打包 → 开/闭 → 内部梯度 → 解包
    uint32_t t0 = DWT_GetCycles();

    precise_edge_detection_u16_binary_adapter(
        &original_image[0][0], IMG_W, IMG_H, IMG_W,
        &edge_image[0][0],     IMG_W,
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
    // 480 MHz 下，us ≈ cycles / 480
    printf("边缘像素数: %d, 周期: %lu cycles, 约 %.3f us @480MHz\r\n",
           edge_count, (unsigned long)cycles, (double)cycles / 480.0);

    while (1) {
        // 主循环（此处可接入后续处理/显示/通信）
    }
}