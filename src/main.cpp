#include <optional>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;
using std::cout;
using std::cin;
using std::string;
using std::istringstream;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

static bool isOnGrid(float pos) {
    return pos >= 0 && pos <= 100;
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
    // user input for these values using std::cin. Pass the user input to the
    // RoutePlanner object below in place of 10, 10, 90, 90.
    string input;
    float start_x; float start_y;
    float end_x; float end_y;
    // Ask for start x position until a valid input is provided
    cout << "Input start x position: " << "\n";
    while (true) {
        getline(cin, input);
        istringstream inputStream(input);
        if (inputStream >> start_x && isOnGrid(start_x)) {
            break;
        }
        cout << "Invalid input, please try again" << "\n";
    }
    // Ask for start y position until a valid input is provided
    cout << "Input start y position: " << "\n";
    while (true) {
        getline(cin, input);
        istringstream inputStream(input);
        if (inputStream >> start_y && isOnGrid(start_y)) {
            break;
        }
        cout << "Invalid input, please try again" << "\n";
    }
    // Ask for end x position until a valid input is provided
    cout << "Input end x position: " << "\n";
    while (true) {
        getline(cin, input);
        istringstream inputStream(input);
        if (inputStream >> end_x && isOnGrid(end_x)) {
            break;
        }
        cout << "Invalid input, please try again" << "\n";
    }
     // Ask for end y position until a valid input is provided
    cout << "Input end y position: " << "\n";
    while (true) {
        getline(cin, input);
        istringstream inputStream(input);
        if (inputStream >> end_y && isOnGrid(end_y)) {
            break;
        }
        cout << "Invalid input, please try again" << "\n";
    }

    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
