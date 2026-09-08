#pragma once 
#include <vector>
#include "types.hpp"

class Environment{
    private:
        int width, height, depth;
        std::vector<bool> obstacles; //1D array flattened 
    
    public:
        Environment(int w, int h, int d);

        int get_index(const Location& loc) const;
        bool is_valid(const Location& loc) const;
        void set_obstacle(const Location& loc);
        bool is_free(const Location& loc) const;
};
