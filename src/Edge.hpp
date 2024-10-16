#include<iostream>
#include<vector>
#include<map>
using namespace std;

typedef struct {
    int start_node;
    int end_node;
    int lower;
    int upper;
    int weight;
} edge;
vector<edge> edges;
map<int, vector<edge>> vertex_with_the_same_start_node(){
    map<int, vector<edge>> tmp;
    for(auto e : edges){
        tmp[e.start_node].push_back(e);
    }
    return tmp;
}
map<int, vector<edge>> vertex_with_the_same_target_node(){
    map<int, vector<edge>> tmp;
    for(auto e : edges){
        tmp[e.end_node].push_back(e);
    }
    return tmp;
}
map<int, vector<edge>> invertex;
map<int, vector<edge>> outvertex;