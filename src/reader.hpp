// reader.hpp
#ifndef READER_HPP
#define READER_HPP

#include "coefficient.hpp"
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>

using namespace std;

class Reader
{
private:
    string map_input;
    string agv_input;
    string coef_res_input;
    string tsg_input;
    void set_max_node();

public:
    Coef coef;
    string filepath;

    Reader(string path);

    void controller();
    void get_map_input();
    void get_TSG_input();
    void get_agv_info();
    void get_coef_res_input();
    Coef set_coef();
};



#endif // READER_HPP
