#pragma once
#include "types.hpp"
#include "Environment.hpp"

class MAPFSolver{
    public:
    virtual ~MAPFSolver() = default; 
    virtual void solve(const Environment& env, const MAPFInstance&instance, SearchMetrics& metrics) = 0;
};