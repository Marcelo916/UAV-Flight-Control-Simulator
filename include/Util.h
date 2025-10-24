#pragma once
#include <algorithm>

template<typename T>
T clamp(T v, T lo, T hi) {
    return std::max(lo, std::min(v, hi));
}