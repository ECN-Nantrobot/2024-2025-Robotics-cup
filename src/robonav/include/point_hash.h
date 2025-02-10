#ifndef POINT_HASH_H
#define POINT_HASH_H

#include "point.h" // Ensure Point is fully defined
#include <cmath>


namespace std
{
template <> struct hash<ecn::Point> {
    std::size_t operator()(const ecn::Point& point) const
    {
        auto h1 = std::hash<int>{}(static_cast<int>(std::round(point.x)));
        auto h2 = std::hash<int>{}(static_cast<int>(std::round(point.y)));
        return h1 ^ (h2 << 1); // Combine the two hashes
    }
};
} // namespace std
// #include <functional>
#endif // POINT_HASH_H
