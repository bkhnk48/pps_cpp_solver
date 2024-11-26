// reader.cpp
#include <climits>
#include "reader.hpp"

Reader::Reader(string path) : filepath(path) {}

void Reader::set_max_node(){
    int tmp = INT_MAX;
    for(auto agv : this->coef.AGVs){
        tmp = min(agv.destination_node,tmp);
    }
    this->coef.max = tmp;
}

void Reader::controller()
{
    ifstream file(this->filepath);
    string line;
    if (!file.is_open())
    {
        std::cerr << "Không thể mở file!" << std::endl;
        return;
    }

    getline(file, this->agv_input);
    this->agv_input.pop_back();
    getline(file, this->coef_res_input);
    this->coef_res_input.pop_back();
    getline(file, this->map_input);
    this->map_input.pop_back();
    getline(file, this->tsg_input);
    file.close();
}

void Reader::get_map_input()
{
    ifstream mapfile(this->map_input);
    string line;
    int N = -1;

    if (!mapfile.is_open())
    {
        std::cerr << "Không thể mở file!" << std::endl;
        return;
    }
    while (getline(mapfile, line))
    {
        istringstream iss(line);
        char label;
        int id1, id2, weight;
        if (iss >> label >> id1 >> id2 >> weight)
        {
            N = max(N, id1);
            N = max(N, id2);
        }
        else
        {
            std::cerr << "Dòng không hợp lệ: " << line << std::endl;
        }
    }

    this->coef.N = N;
}

void Reader::get_TSG_input()
{
    ifstream tsg_file(this->tsg_input);
    string line;
    Graph g = Graph();

    if (!tsg_file.is_open())
    {
        std::cerr << "Không thể mở file!" << std::endl;
        return;
    }
    while (getline(tsg_file, line))
    {
        if (line[0] == 'a')
        {
            istringstream iss(line);
            Edge e;
            string temp;
            iss >> temp;
            iss >> e.start_node;
            iss >> e.end_node;
            iss >> e.lower;
            iss >> e.upper;
            iss >> e.weight;
            if (e.end_node < this->coef.max)
            {
                g.edges.push_back(e);
            }
        }
    }

    this->coef.outvertex = g.vertex_with_the_same_start_node(g.edges);
    this->coef.invertex = g.vertex_with_the_same_target_node(g.edges);
}

void Reader::get_agv_info()
{
    ifstream file_agv(this->agv_input);
    string line, str;
    int flow, agv_id = 0;

    if (!file_agv.is_open())
    {
        std::cerr << "Không thể mở file!" << std::endl;
        return;
    }

    while (true)
    {
        AGV agv;
        agv.id = agv_id;
        if (!getline(file_agv, line))
            break;
        istringstream start_iss(line);
        start_iss >> str >> agv.start_node >> flow;

        if (!getline(file_agv, line))
            break;
        istringstream end_iss(line);
        end_iss >> str >> agv.end_node >> flow;

        if (!getline(file_agv, line))
            break;
        istringstream window_time_iss(line);
        window_time_iss >> str >> agv.earliness >> agv.tardliness >> agv.destination_node;

        agv_id++;
        this->coef.AGVs.push_back(agv);
    }
}

void Reader::get_coef_res_input()
{
    ifstream coef_res_file(this->coef_res_input);
    string line, str;
    vector<Edge> edges;

    if (!coef_res_file.is_open())
    {
        std::cerr << "Không thể mở file!" << std::endl;
        return;
    }
    getline(coef_res_file, line);
    stringstream iss(line);
    iss >> str >> this->coef.alpha >> this->coef.beta >> this->coef.gamma;

    while (getline(coef_res_file, line))
    {
        if (line[0] == 'r')
        {
            Edge e;
            stringstream iss(line);
            iss >> str >> e.start_node >> e.end_node;
            edges.push_back(e);
        }
        else if (line[0] == 'u')
        {
            int u;
            stringstream iss(line);
            iss >> str >> u;
            this->coef.restriction.push_back(make_pair(edges, u));
            edges.clear();
        }
    }
}

Coef Reader::set_coef()
{
    
    controller();
    get_coef_res_input();
    get_agv_info();
    set_max_node();
    get_map_input();
    get_TSG_input();

    return this->coef;
}
