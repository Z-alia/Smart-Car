好的，这是一个根据你的需求生成的 Markdown 文件，包含基于曲率的速度规划和 IIR 滤波的实现步骤，已制成待办清单格式。

```markdown
# 智能车控制升级：曲率速度规划与 IIR 滤波实施计划

## 目标

1.  实现基于路径曲率的动态速度规划，提升连环弯道性能。
2.  对关键控制输入（横向偏差、曲率）应用 IIR 滤波，减少抖动。

---

## 待办事项清单

### Part 1: 基于曲率的速度规划

- [ ] **1.1. 获取路径曲率**
    - [ ]  在图像处理模块中，实现路径曲率的计算逻辑（例如，使用中心线拟合）。
    - [ ]  确保曲率计算函数 `calculate_curvature()` 能够稳定输出当前路径的曲率值 `raw_curvature`。
    - [ ]  将 `raw_curvature` 作为模块输出接口。

- [ ] **1.2. 设计曲率-速度映射**
    - [ ]  定义 `curvature_table[]` 和 `speed_table[]` 数组，或编写映射函数 `get_target_speed_based_on_curvature()`。
    - [ ]  根据你的车型和赛道特性，初步设定查表或函数的参数（例如，最大速度、不同曲率对应的速度）。
    - [ ]  考虑加入安全系数，例如 `final_target_speed = get_target_speed_based_on_curvature(curvature) * safety_factor;` (e.g., `safety_factor = 0.9`)

- [ ] **1.3. 整合到主控制循环**
    - [ ]  在主控制循环中，调用曲率计算函数获取 `raw_curvature`。
    - [ ]  将 `raw_curvature` 输入到速度规划函数，计算出 `final_target_speed`。
    - [ ]  将 `final_target_speed` 作为**速度环 PID** 的设定值（Setpoint）。

### Part 2: IIR 输入滤波

- [ ] **2.1. 定义滤波相关变量**
    - [ ]  在全局变量或结构体中，定义滤波后的输出变量：
        - `float filtered_lateral_error;`
        - `float filtered_curvature;`
    - [ ]  定义滤波系数 `alpha`：
        - `const float alpha_lateral = 0.2;` (可调)
        - `const float alpha_curvature = 0.3;` (可调)
    - [ ]  定义上一次的滤波输出变量（用于 IIR）：
        - `float prev_filtered_lateral_error = 0.0;`
        - `float prev_filtered_curvature = 0.0;`
    - [ ]  定义初始化标志：
        - `bool first_run_flag = true;`

- [ ] **2.2. 编写 IIR 滤波函数**
    - [ ]  编写一个函数 `apply_low_pass_filter_IIR()`，接收原始横向偏差 `raw_lateral_error` 和原始曲率 `raw_curvature`。
    - [ ]  在函数内部实现 IIR 滤波公式：
        - `filtered_lateral_error = alpha_lateral * raw_lateral_error + (1 - alpha_lateral) * prev_filtered_lateral_error;`
        - `filtered_curvature = alpha_curvature * raw_curvature + (1 - alpha_curvature) * prev_filtered_curvature;`
    - [ ]  更新 `prev_filtered_lateral_error` 和 `prev_filtered_curvature`。
    - [ ]  处理首次运行（`first_run_flag`）的情况，直接赋值。

- [ ] **2.3. 整合滤波到主控制循环**
    - [ ]  在主控制循环中，获取原始横向偏差 `raw_lateral_error` 和原始曲率 `raw_curvature`。
    - [ ]  调用 `apply_low_pass_filter_IIR(raw_lateral_error, raw_curvature)` 函数。
    - [ ]  使用滤波后的 `filtered_lateral_error` 作为**方向环 PID** 的误差输入。
    - [ ]  使用滤波后的 `filtered_curvature` 作为**速度规划函数**的输入。

### Part 3: 调试与优化

- [ ] **3.1. 参数调试**
    - [ ]  **速度规划参数**：在赛道上测试，观察不同曲率下的速度响应，调整 `curvature_table[]`, `speed_table[]` 和 `safety_factor`。
    - [ ]  **滤波参数**：观察滤波后的信号平滑度和响应速度，调整 `alpha_lateral` 和 `alpha_curvature`。`alpha` 越小，越平滑但越滞后；`alpha` 越大，响应越快但滤波效果越弱。

- [ ] **3.2. 综合测试**
    - [ ]  在包含直道、弯道、连环弯的完整赛道上进行测试。
    - [ ]  观察车辆的整体行驶稳定性、速度和轨迹平滑度。
    - [ ]  根据测试结果，微调所有相关参数。

- [ ] **3.3. 代码整理与注释**
    - [ ]  清理和注释新增的代码，确保可读性。
    - [ ]  记录下最终调试好的参数值。
```