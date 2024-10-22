#include<iostream>
#include<vector>
#include <fstream> 
#include<map>
#include <sstream>
#include <string>
#include"AGV.hpp"
#include"Edge.hpp"
using namespace std;

ifstream file("/mnt/d/DATN/solver/prj/src/example.txt");
void readinput(){
    string str;
    vector<string> agvsinfo;
    while (getline(file,str)){
        if(str[0] == 'n'){
            agvsinfo.push_back(str);
        }    
        if(str[0] == 'a'){
            istringstream iss(str);
            edge e;
            string temp;
            iss >> temp;
            iss >> e.start_node;
            iss >> e.end_node;
            iss >> e.lower;
            iss >> e.upper;
            iss >> e.weight;
            edges.push_back(e);
        }
    }
    int i,count = 0;
    for(i=0;i<(agvsinfo.size())/2; i++){
        AGV agv;
        string tmp;
        istringstream iss1(agvsinfo[i]);
        istringstream iss2(agvsinfo[i+(agvsinfo.size())/2]);
        iss1>>tmp;
        iss1>>agv.start_node;
        iss2>>tmp;
        iss2>>agv.end_node;
        agv.id = count;
        count++;
        AGVs.push_back(agv);
    }
}