#include "../include/Environment.hpp"

Environment::Environment(int w, int h, int d) : width(w), height(h), depth(d) {
    obstacles.assign(width * height * depth, false);
}

int Environment::get_index(const Location& loc) const {
    return (loc.z * width * height) + (loc.y * width) + loc.x;
}

bool Environment::is_valid(const Location& loc) const {
    return loc.x >= 0 && loc.x < width &&
           loc.y >= 0 && loc.y < height &&
           loc.z >= 0 && loc.z < depth;
}

void Environment::set_obstacle(const Location& loc) {
    if (is_valid(loc)) {
        obstacles[get_index(loc)] = true;
    }
}

bool Environment::is_free(const Location& loc) const {
    if (!is_valid(loc)) return false;
    return !obstacles[get_index(loc)];
}