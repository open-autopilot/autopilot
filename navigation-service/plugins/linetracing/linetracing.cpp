#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <chrono>

using namespace std;

struct vec2
{
    int x;
    int y;
};

// Function for moving "move_row", a set amount, "amount".
vector<vec2> move_line(vector<vec2> line, vec2 amount) 
{
    vector<vec2> moved_line;
    for (const auto& point : line) {
        moved_line.push_back({point.x + amount.x, point.y + amount.y});
    }

    return moved_line;
}

// Function for making a line of two points.
vector<vec2> make_line(vec2 point_a, vec2 point_b) {
    vector<vec2> line;

    // Sorting x and y values from smallest to largest.
    vector<int> x_sort = {point_a.x ,point_b.x};
    vector<int> y_sort = {point_a.y, point_b.y};
    std::sort(x_sort.begin(), x_sort.end());
    std::sort(y_sort.begin(), y_sort.end());

    // Calculating the slope of the line.
    double slope = (double(point_b.y-point_a.y)/double(point_b.x-point_a.x));
    double addition = point_a.y-slope*point_a.x;
    cout << "f(X) = " << slope << "X" << "+" << addition << endl;

    // Find all y == whole solutions.
    for (int y = y_sort[0]; y <= y_sort[1]; y++) {
        int xval = (y-addition)/slope;
        line.push_back({xval,y});
        line.push_back({xval+1,y});
    }

    // Find all x == whole solutions.
    for (int x = x_sort[0]; x <= x_sort[1]; ++x) {
        int yval = slope*x+addition;
        line.push_back({x,yval});
        line.push_back({x,yval+1});
    }

    // Todo: Replace codeblock with std::unique.
    vector<vec2> unique_line;
    for (auto point : line) {
        bool unique = true;
        for (auto point1 : unique_line) {
            if (point1.x == point.x && point1.y == point.y) {
                unique = false;
            }
        }

        if (unique)
        {
            unique_line.push_back(point);
        }
    }
    line = unique_line;

    std::sort(line.begin(), line.end(), [](auto a, auto b) {
       return a.x < b.x || (a.x == b.x && a.y < b.y);
    });

    return line;
}

vector<vector<vec2>> make_headlands(vec2 last_vec, int lines, vec2 point_a, vec2 point_b) {
    // Generating base headland lines. 
    vector<vector<vec2>> headlands;
    vec2 point_a_accent = {point_a.x+last_vec.x, point_a.y+last_vec.y};
    vec2 point_b_accent = {point_b.x+last_vec.x, point_b.y+last_vec.y};
    vector<vec2> headland_a = make_line(point_a, point_a_accent);
    vector<vec2> headland_b = make_line(point_b, point_b_accent);
    headlands.push_back(headland_a);
    headlands.push_back(headland_b);

    // Calculating ab and ba normilized.
    vec2 ab = {point_b.x-point_a.x, point_b.y-point_a.y};
    double ab_unit_x = {ab.x/sqrt(ab.x*ab.x+ab.y*ab.y)};
    double ab_unit_y = {ab.y/sqrt(ab.x*ab.x+ab.y*ab.y)};
    double ba_unit_x = -ab_unit_x;
    double ba_unit_y = -ab_unit_y;

    // Creating headland a.
    vec2 move_vec; 
    for (double i = 1; i < lines; i++) {
        int move_x = (int)round(ab_unit_x*i);
        int move_y = (int)round(ab_unit_y*i);
        move_vec = {-move_y, -move_x};
        std::cout << "Move vec: " << move_vec.x << " " << move_vec.y << std::endl;

        vector<vec2> moved_line = move_line(headland_a, move_vec);
        headlands.push_back(moved_line);
    }

    // Creating headland b.
    for (double i = 1; i < lines; i++) {
        int move_x = (int)round(ba_unit_x*i);
        int move_y = (int)round(ba_unit_y*i);
        move_vec = {-move_y, -move_x};
        std::cout << "Move vec: " << move_vec.x << " " << move_vec.y << std::endl;

        vector<vec2> moved_line = move_line(headland_b, move_vec);
        headlands.push_back(moved_line);
    }

    return headlands;
}

vector<vector<vec2>> make_map(double line_distance, int lines, vec2 point_a, vec2 point_b, int headland_lines) {
    vector<vector<vec2>> map;
    int delta_x = point_b.x-point_a.x;
    int delta_y = point_b.y-point_a.y;

    // Calculating the perpendicular.
    vec2 perp = {delta_x,-delta_y};
    double perp_lenght = sqrt(perp.x*perp.x+perp.y*perp.y);
    double perp_unit_x = perp.x/perp_lenght;
    double perp_unit_y = perp.y/perp_lenght;

    // Creating the first line.
    vector<vec2> ab_line = make_line(point_a, point_b);
    map.push_back(ab_line);

    // Creating other lines.
    vec2 move_vec; 
    for (double i = 1; i < lines; i++) {
        int move_x = (int)round(perp_unit_x*line_distance*i);
        int move_y = (int)round(perp_unit_y*line_distance*i);
        move_vec = {-move_y, -move_x};
        std::cout << "Move vec: " << move_vec.x << " " << move_vec.y << std::endl;

        vector<vec2> moved_line = move_line(ab_line, move_vec);
        map.push_back(moved_line);
    }

    // Creating headlands
    vector<vector<vec2>> headlands = make_headlands(move_vec, headland_lines, point_a, point_b);
    for (const auto& headland : headlands) {
        map.push_back(headland);
    }

    return map;
}

std::string map_tostring(vector<vector<vec2>> map) {
    std::string map_string;
    for (const auto& line : map) {
        for (const auto& point : line) {
            map_string += "(" + std::to_string(point.x) + "," + std::to_string(point.y) + "),";
        }
        map_string += "\n";
    }

    return map_string;
}

int main( ) 
{  
    // Parameters for the lines.
    double line_distance = 30; //row_dis = dis(m)/0.05m
    int line_amount = 20;
    vec2 point_a = {2,3}; 
    vec2 point_b = {500,500};
    int headland_lines = 50;

    // Creating the map.
    auto map = make_map(line_distance, line_amount, point_a, point_b, headland_lines);
    std::string map_string = map_tostring(map);
    cout << map_string << endl;
}

