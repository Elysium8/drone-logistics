#pragma once
#include "types.hpp"
#include <string>
#include <fstream>
#include <iostream>
#include <stdexcept>

class PathExporter {
public:
    static void export_paths_json(const std::vector<std::vector<Location>>& paths, const std::string& filepath) {
        std::ofstream file(filepath);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open file for exporting paths: " + filepath);
        }

        file << "[\n";
        for (size_t i = 0; i < paths.size(); ++i) {
            file << "  {\n";
            file << "    \"name\": \"Drone_" << i << "\",\n";
            file << "    \"path\": [\n";
            
            const auto& path = paths[i];
            for (size_t t = 0; t < path.size(); ++t) {
                file << "      [" << path[t].x << ", " << path[t].y << ", " << path[t].z << "]";
                if (t < path.size() - 1) file << ",";
                file << "\n";
            }
            
            file << "    ]\n";
            
            file << "  }";
            if (i < paths.size() - 1) file << ",";
            file << "\n";
        }
        file << "]\n";
        
        file.close();
    }
};