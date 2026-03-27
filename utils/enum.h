/*
* Copyright (c) Huawei Technologies Co., Ltd. 2026-2026. All rights reserved.
* Description: 2026 code craft enum
* Author: anzhiwu a00494813
* Create: 2026/1/27
*/

#ifndef INC_2026_CODE_CRAFT_ENUM_H
#define INC_2026_CODE_CRAFT_ENUM_H

#include <algorithm>
#include <iterator>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace andrew::utils {
struct EnumProperty {
    const std::vector<std::string> kStrVector;
    const int32_t kStartFrom;

    explicit EnumProperty(std::vector<std::string> strVector, int32_t startFrom)
        : kStrVector(std::move(strVector)), kStartFrom(startFrom)
    {
    }
};

inline std::vector<std::string> _generateEnumStrVector(const std::string& name, std::string members)
{
    std::replace(members.begin(), members.end(), ',', ' ');
    std::istringstream iss(members);
    std::vector<std::string> res;
    copy(std::istream_iterator<std::string>(iss), {}, std::back_inserter(res));
    // 去掉前缀 name + "::"
    // std::for_each(res.begin(), res.end(), [&name](std::string& elem) { elem = name + "::" + elem; });
    return res;
}
}  // namespace andrew::utils

#define ANDREW_UTILS_ENUM_AUTO_TO_STRING(name, startFrom, members...)                                                 \
    static const andrew::utils::EnumProperty _property_##name(andrew::utils::_generateEnumStrVector(#name, #members), \
                                                              startFrom);                                             \
    inline const char* to_string(name member)                                                                         \
    {                                                                                                                 \
        int32_t memberIdx = (int32_t)member - _property_##name.kStartFrom;                                            \
        return 0 <= memberIdx && memberIdx < (int32_t)_property_##name.kStrVector.size()                              \
                   ? _property_##name.kStrVector.at(memberIdx).c_str()                                                \
                   : "[UNKNOWN]";                                                                                     \
    }

#define ANDREW_CXX_ENUM(name, startFrom, firstMember, otherMembers...) \
    enum class name { firstMember = startFrom, otherMembers };         \
    ANDREW_UTILS_ENUM_AUTO_TO_STRING(name, startFrom, firstMember, ##otherMembers)

/**
 * 1. Example:
 *      CXX_ENUM(Color,
 *               RED,
 *               BLUE)
 * 2. Usage:
 *      Color red = Color::RED;
 *      to_string(red);                 ->  'RED'
 *      Color yellow = (Color)100;
 *      to_string(yellow):              ->  '[UNKNOWN]'
 */
#define CXX_ENUM(name, members...) ANDREW_CXX_ENUM(name, 0, ##members)

#endif  // INC_2026_CODE_CRAFT_ENUM_H
