/*
* Copyright (c) Huawei Technologies Co., Ltd. 2026-2026. All rights reserved.
* Description: 2026 code craft timer
* Author: anzhiwu a00494813
* Create: 2026/3/3
 */

#ifndef INC_2026_CODE_CRAFT_TIMER_H
#define INC_2026_CODE_CRAFT_TIMER_H

#include <chrono>

class Timer {
public:
    // 构造函数：初始化计时器为初始状态
    Timer() : isRunning_(false), totalElapsedNs_(0), startTime_() {}

    /**
     * @brief 启动计时器
     * @note 若计时器已处于运行状态，调用此函数无效果（避免重复启动覆盖起始时间）
     */
    void Start();

    /**
     * @brief 停止（暂停）计时器
     * @note 若计时器已处于停止状态，调用此函数无效果
     */
    void Stop();

    /**
     * @brief 获取计时器当前的时间值
     * @return long long 时间值（纳秒）：
     *         - 运行中：本次启动到当前的时间
     *         - 停止：所有运行时段的累计总时间
     */
    [[nodiscard]] long long GetTime() const;

    /**
     * @brief 重置计时器到初始状态
     * @note 无论当前状态是运行中还是停止，都会清空累计时间并标记为停止
     */
    void Reset();

private:
    bool isRunning_;
    long long totalElapsedNs_;
    std::chrono::high_resolution_clock::time_point startTime_;
};

#endif  // INC_2026_CODE_CRAFT_TIMER_H
