# Repository Guidelines

## 项目结构与模块组织
- `Core/Inc`, `Core/Src`：启动、时钟、中断与主循环入口。
- `Drivers/`：官方 HAL/CMSIS，勿随意修改；自定义外设放在 `HardWare/`。
- `Base/`：车模控制、路径/速度逻辑；`新模块/` 用于实验功能，需单独文档。
- `camera_process/`：摄像头 188×120 图像处理（采集、二值化、形态学、元素识别）。
- `MDK-ARM/`：Keil/EIDE 工程文件与 `.clang-format`，保持与 IDE 同步。

## 构建、烧录与常用命令
- 推荐 VS Code + EIDE 打开 `MDK-ARM/base_project.code-workspace`，使用 `Terminal > Run Task` 的 `build/rebuild/clean`，`flash` 或 `build and flash` 进行下载。
- 亦可直接在 Keil 打开 `MDK-ARM/base_project.uvprojx` 编译与烧录。
- 清理/重建请通过任务或 IDE，自行删除 `MDK-ARM/Listings/Objects` 前先确认未在调试中。

## 代码风格与命名约定
- C 语言，4 空格缩进，遵循 `MDK-ARM/.clang-format`；提交前对改动文件运行 clang-format。
- 函数/变量使用 snake_case，宏全大写，文件名与模块一致（如 `morph_binary_bitpacked.c/.h`）。
- CubeMX 生成文件仅在 `/* USER CODE BEGIN */` 区域写入自定义逻辑。

## 测试与验证
- 无独立 PC 端测试目录，主要依赖上板验证（车道保持、摄像头帧率、WiFi/TOF）。
- 图像链路：`Binarization.c` → `morph_binary_bitpacked.c`（开闭运算）→ `Element_recognition.c`。斑马线检测默认在行段 60-70、跳变阈值 10，可按赛道实际调整。
- 如新增算法，请在 `camera_process/` 旁增加注释或示例帧，必要时在 LCD/OLED 打开调试可视化。

## 提交与 PR 指南
- 提交信息简洁聚焦，如 `fix tof uart timeout`、`tune lane pid`；避免“update code”此类笼统描述。
- PR/合并说明应包含：改动动机、主要影响模块（路径）、测试方式（含硬件环境/录像/截图）。
- 避免一次性大改动，保持易回退；不要混入无关格式化或驱动库变更。

## 代理与沟通说明
- 与用户沟通请使用中文，保持专业、可复现的说明。
- 本工程基于 STM32H750 的智能车，请评估改动对实时性、外设时序和供电的影响；尽量保持最小侵入，避免修改 `Drivers/` 与 `MDK-ARM/RTE/`。
