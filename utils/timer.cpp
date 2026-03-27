/*
* Copyright (c) Huawei Technologies Co., Ltd. 2026-2026. All rights reserved.
* Description: 2026 code craft timer cpp
* Author: anzhiwu a00494813
* Create: 2026/3/3
 */

#include "timer.h"

void Timer::Start() {
    if (!isRunning_) {
        startTime_ = std::chrono::high_resolution_clock::now();
        isRunning_ = true;
    }
}

void Timer::Stop() {
    if (isRunning_) {
        // 获取当前时间点，计算本次运行的时间（纳秒）
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(current_time - startTime_);

        // 累计到总时间（转换为long long，避免类型截断）
        totalElapsedNs_ += static_cast<long long>(elapsed.count());
        isRunning_ = false;
    }
}

long long Timer::GetTime() const {
    if (isRunning_) {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(current_time - startTime_);
        return static_cast<long long>(elapsed.count());
    } else {
        return totalElapsedNs_;
    }
}

void Timer::Reset() {
    isRunning_ = false;
    totalElapsedNs_ = 0;
    startTime_ = {};
}