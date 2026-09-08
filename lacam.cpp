#include <iostream>
#include <vector>

struct Vertex
{
    int id;
    std::vector<int> neighbors;
    Vertex(int _id) : id(_id) {}
};
