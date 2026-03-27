/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2026-2026. All rights reserved.
 * Description: 2026 code craft enum
 * Author: anzhiwu a00494813
 * Create: 2026/1/28
 */
#ifndef INC_2026_CODE_CRAFT_UTILS_H
#define INC_2026_CODE_CRAFT_UTILS_H

#include <cmath>
#include <vector>
namespace Geom {
struct Point {
    double x;
    double y;

    inline Point& operator+=(const Point& p)
    {
        x += p.x;
        y += p.y;
        return *this;
    }
};

struct Polygon {
    std::vector<Point> pts;

    void MoveByVec(const Point& vec)
    {
        for (auto& pt : pts) {
            pt += vec;
        }
    }

    void SetVertices(const std::vector<Point>& vertex)
    {
        pts.clear();
        for (size_t i = 0; i < vertex.size(); ++i) {
            pts.push_back(vertex[i]);
        }
    }

    double CalcArea() const
    {
        int cnt = static_cast<int>(pts.size());
        if (cnt < 3) {
            return 0.0;
        }

        double s = pts[0].y * (pts[cnt - 1].x - pts[1].x);
        for (int i = 1; i < cnt; i++) {
            s += pts[i].y * (pts[i - 1].x - pts[(i + 1) % cnt].x);
        }

        return fabs(s / 2);
    }
};
}  // namespace Geom
#endif  // INC_2026_CODE_CRAFT_UTILS_H
