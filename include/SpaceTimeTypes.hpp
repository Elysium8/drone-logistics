#pragma once 
#include "types.hpp"
#include <functional>


struct STNode {
    Location loc;
    int t;
    int g_score;
    int f_score;

    const STNode* parent;



}