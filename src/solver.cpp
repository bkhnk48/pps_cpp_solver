#include <iostream>
#include <map>
#include <vector>
#include <string>
#include "scip/scip.h"
#include "scip/scipdefplugins.h"
#include "scip_exception.hpp"
#include "reader.hpp"
using namespace std;

int startNode = 1;
int endNode = 3;
SCIP* scip = nullptr; 
map<string, SCIP_VAR*> varmap;  // Để lưu biến và tên của các cạnh
vector<SCIP_CONS*> cons1;
vector<SCIP_CONS*> cons2;
vector<SCIP_CONS*> cons3;
vector<SCIP_CONS*> cons4;
vector<SCIP_CONS*> cons56;


// Bài toán chính 
SCIP_RETCODE mainproblem(){
    for(auto agv : AGVs){
        for (auto pair : outvertex) {
            int key = pair.first;
            vector<edge> v = pair.second;
            for (auto e : v) {
                SCIP_VAR* var = nullptr;
                string varname = "x_" + to_string(agv.id) + "_" + to_string(e.start_node) + "_" + to_string(e.end_node);
                SCIP_CALL(SCIPcreateVarBasic(scip, &var, varname.c_str(), 0.0, 1.0, e.weight, SCIP_VARTYPE_BINARY));
                SCIP_CALL(SCIPaddVar(scip, var));
                varmap[varname] = var;  // Lưu lại biến vào map
            }
        }
    }
    
    return SCIP_OKAY;  // Trả về mã lỗi thành công
}

// Ràng buộc tại mỗi cạnh chỉ có 1 AGv đi qua
SCIP_RETCODE Constraints1() {
    for (auto pair : outvertex) {
        int node = pair.first;
        // Lặp qua các cạnh xuất phát từ đỉnh này
        for (auto e : pair.second) {
            SCIP_CONS* cons = nullptr;
        
            // Tạo ràng buộc tuyến tính cơ bản cho đỉnh node
            SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, ("Max_flow_at_" + to_string(node)).c_str(), 0, nullptr, nullptr, 0.0, 1.0));
            // Thêm biến dòng chảy cho mỗi AGV
            for (auto agv : AGVs) {
                std::string varname = "x_" + std::to_string(agv.id) + "_" + to_string(e.start_node) + "_" + to_string(e.end_node);
                
                // Thêm hệ số biến vào ràng buộc tuyến tính
                SCIP_CALL(SCIPaddCoefLinear(scip, cons, varmap[varname], 1.0));
            }
            // Thêm ràng buộc vào bài toán SCIP
            SCIP_CALL(SCIPaddCons(scip, cons));

            // Lưu trữ ràng buộc để sử dụng sau này
            cons1.push_back(cons);
        }
        
        
        
        
        
    }
    
    return SCIP_OKAY;  // Trả về mã lỗi thành công
}

// Ràng buộc tại 1 đỉnh số AGV  vào phải bằng số AGV ra 
SCIP_RETCODE Contraints2(){
    for(auto pair : outvertex){
        int node = pair.first;
        if (invertex.find(node) != invertex.end() && outvertex.find(node) != outvertex.end()){
            SCIP_CONS* cons = nullptr;
            SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, ("flow_at_" + to_string(node)).c_str(), 0, nullptr, nullptr, 0.0, 0.0));
            // Tổng các cạnh đi vào đỉnh
            for (auto e : invertex[node]){
                for(auto agv : AGVs){
                    string varname = "x_" + to_string(agv.id) + "_" + to_string(e.start_node) + "_" + to_string(e.end_node);
                    SCIP_CALL(SCIPaddCoefLinear(scip, cons, varmap[varname], 1.0));    
                }
            }
            // Trừ tổng các cạnh đi ra khỏi đỉnh
            for (auto e : outvertex[node]){
                for(auto agv : AGVs){
                    string varname = "x_" + to_string(agv.id) + "_" + to_string(e.start_node) + "_" + to_string(e.end_node);
                    SCIP_CALL(SCIPaddCoefLinear(scip, cons, varmap[varname], -1.0));
                }
            }
            SCIP_CALL(SCIPaddCons(scip, cons));
            cons2.push_back(cons);
        }
    }
    return SCIP_OKAY;  // Trả về mã lỗi thành công
}

// Ràng buộc xe chỉ dừng ở 1 cạnh trên thuộc Ol
SCIP_RETCODE Contraints3(){
    for(auto agv : AGVs){
       for(auto e : invertex[agv.end_node]){
            SCIP_CONS* cons = nullptr;
            SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, ("Stop_at_" + to_string(e.start_node)).c_str(), 0, nullptr, nullptr, 0.0, 0.0));
            for(auto e1 : invertex[e.start_node]){
                string varname = "x_" + to_string(agv.id) + "_" + to_string(e1.start_node) + "_" + to_string(e1.end_node);
                SCIP_CALL(SCIPaddCoefLinear(scip, cons, varmap[varname], 1.0));
            }
            string varname = "x_" + to_string(agv.id) + "_" + to_string(e.start_node) + "_" + to_string(e.end_node);   
            SCIP_CALL(SCIPaddCoefLinear(scip, cons, varmap[varname], -1.0));
            SCIP_CALL(SCIPaddCons(scip, cons));
            cons3.push_back(cons);
       }
    }
    return SCIP_OKAY;  // Trả về mã lỗi thành công
}

//Ràng buộc xe bắt buộc phải đi từ sm
SCIP_RETCODE Contraints456(){
    // Ràng buộc cho M
    for(auto agv : AGVs){
        SCIP_CONS* cons = nullptr;
        SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, ("Start_at_" + to_string(agv.start_node)).c_str(), 0, nullptr, nullptr, 1, 1));
        for(auto e : outvertex[agv.start_node]){
            string varname = "x_" + to_string(agv.id) + "_" + to_string(e.start_node) + "_" + to_string(e.end_node);
            SCIP_CALL(SCIPaddCoefLinear(scip, cons, varmap[varname], 1.0));
        }
        SCIP_CALL(SCIPaddCons(scip, cons));
        cons4.push_back(cons);
    }
    // Ràng buộc cho L 
    for(auto agv : AGVs){
        SCIP_CONS* cons = nullptr;
        SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, ("End_at_" + to_string(agv.end_node)).c_str(), 0, nullptr, nullptr, 1, 1));
        for (auto e : invertex[agv.end_node]){
            string varname = "x_" + to_string(agv.id) + "_" + to_string(e.start_node) + "_" + to_string(e.end_node);
            SCIP_CALL(SCIPaddCoefLinear(scip, cons, varmap[varname], 1.0));
        }
        SCIP_CALL(SCIPaddCons(scip, cons));
        cons56.push_back(cons);
    }

    return SCIP_OKAY;  // Trả về mã lỗi thành công
}

void init() {
    readinput();
    invertex = vertex_with_the_same_target_node();
    outvertex = vertex_with_the_same_start_node();
}

int main(int argc, char** argv) {
    init();

    // 1. Khởi tạo môi trường SCIP
    SCIP_CALL(SCIPcreate(&scip));
    SCIP_CALL(SCIPincludeDefaultPlugins(scip));

    // 2. Đặt tên bài toán
    SCIP_CALL(SCIPcreateProbBasic(scip, "Shortest_Path_Problem"));

    // 3. Tạo các biến nhị phân đại diện cho các cạnh
    SCIP_CALL(mainproblem());   

    // 4. Đặt hàm mục tiêu là tối thiểu hóa tổng trọng số các cạnh được chọn
    SCIP_CALL(SCIPsetObjsense(scip, SCIP_OBJSENSE_MINIMIZE));

    // 5. Thêm ràng buộc
    SCIP_CALL(Constraints1()); 

    SCIP_CALL(Contraints2());  

    SCIP_CALL(Contraints3()); 

    SCIP_CALL(Contraints456());  

    // 6. Tối ưu hóa bài toán
    SCIP_CALL(SCIPsolve(scip));

    // 7. In kết quả
    SCIP_SOL* sol = SCIPgetBestSol(scip);
    if (sol != nullptr) {
        cout << "Optimal solution found:" << endl;
        for (auto& [varname, var] : varmap) {
            SCIP_Real val = SCIPgetSolVal(scip, sol, var);
            int int_val = static_cast<int>(val);
            if(int_val == 1){
                cout << varname << " = " << int_val << endl;
            }
        }
    } else {
        cout << "No solution found." << endl;
    }

    // Giải phóng tất cả các biến trong varmap
    for (auto& [varname, var] : varmap) {
        SCIP_CALL(SCIPreleaseVar(scip, &var));  // Đảm bảo tất cả các biến đều được giải phóng
    }

    // Giải phóng các constraints
    for(SCIP_CONS* con : cons1){
        SCIP_CALL(SCIPreleaseCons(scip, &con));
    }
    for(SCIP_CONS* con : cons2){
        SCIP_CALL(SCIPreleaseCons(scip, &con));
    }
    for(SCIP_CONS* con : cons3){
        SCIP_CALL(SCIPreleaseCons(scip, &con));
    }
    for(SCIP_CONS* con : cons4){
        SCIP_CALL(SCIPreleaseCons(scip, &con));
    }
    for(SCIP_CONS* con : cons56){
        SCIP_CALL(SCIPreleaseCons(scip, &con));
    }

    // Giải phóng SCIP
    SCIP_CALL(SCIPfree(&scip));
    
    return 0;
}
