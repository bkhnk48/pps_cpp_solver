#include <iostream>
#include <vector>
#include <fstream>
#include <map>
#include <sstream>
#include <string>

using namespace std;

#ifndef EDGE_HPP
#define EDGE_HPP
class Edge
{
public:
    int start_node;
    int end_node;
    int lower;
    int upper;
    int weight;

    Edge() : start_node(0), end_node(0), lower(0), upper(0), weight(0.0) {}

    Edge(int start, int end, int low, int up, int w)
        : start_node(start), end_node(end), lower(low), upper(up), weight(w) {}
};

class Graph
{
public:
    


    vector<Edge> edges;

    map<int, vector<Edge>> vertex_with_the_same_start_node(vector<Edge> edges)
    {
        map<int, vector<Edge>> tmp;
        for (const auto& e : edges)
        {
            tmp[e.start_node].push_back(e);
        }
        return tmp;
    }

    map<int, vector<Edge>> vertex_with_the_same_target_node(vector<Edge> edges)
    {
        map<int, vector<Edge>> tmp;
        for (const auto& e : edges)
        {
            tmp[e.end_node].push_back(e);
        }
        return tmp;
    }
};

#endif // EDGE_HPP