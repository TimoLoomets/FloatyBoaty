#pragma once
#include <fstream>
#include <iostream>

#include <include/visualizer.hpp>

namespace map_editor
{
    class Editor
    {
    private:
        std::unique_ptr<visualizer::Visualizer> visualizer;
        
    public:
        Editor(std::string track_file);
        ~Editor();
    };
    
    void ensure_file_exists(const std::string name);
    void mouse_callback(int event, int x, int y, int flags, void* userdata);
} // namespace map_editor
