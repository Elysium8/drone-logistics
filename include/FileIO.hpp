#pragma once
#include "types.hpp"
#include "Environment.hpp"
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>

class FileIO {
public:
    static Environment load_map(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) throw std::runtime_error("Could not open map file: " + filepath);

        std::string line;
        int w = 0, h = 0, d = 0;
        bool dims_read = false;
        
        Environment* env = nullptr; 

        while (std::getline(file, line)) {
            // Skips empty lines and comments
            if (line.empty() || line[0] == '#') continue; 

            std::stringstream ss(line);
            if (!dims_read) {
                ss >> w >> h >> d;
                env = new Environment(w, h, d);
                dims_read = true;
            } else {
                char type;
                int x, y, z;
                ss >> type >> x >> y >> z;
                
                if (type == 'O') {
                    env->set_obstacle({x, y, z});
                } else if (type == 'P') {
                    env->add_pad({x, y, z});
                }
            }
        }
        
        if (!env) throw std::runtime_error("Map file was empty.");
        
        Environment result = *env;
        delete env;
        return result;
    }

    static MAPFInstance load_scenario(const std::string& filepath, int num_agents) {
        std::ifstream file(filepath);
        if (!file.is_open()) throw std::runtime_error("Could not open scen file: " + filepath);

        MAPFInstance instance;
        std::string line;
        int agents_loaded = 0;
        int start_time = 0;

        while (std::getline(file, line) && agents_loaded < num_agents) {
            if (line.empty() || line[0] == '#') continue;

            std::stringstream ss(line);
            int sx, sy, sz, gx, gy, gz;
            int delay = 0;
            ss >> sx >> sy >> sz >> gx >> gy >> gz >> delay >> start_time;
            
            instance.starts.push_back({sx, sy, sz});
            instance.goals.push_back({gx, gy, gz});
            instance.delays.push_back(delay);
            instance.start_times.push_back(start_time);
            agents_loaded++;
        }
        return instance;
    }
};