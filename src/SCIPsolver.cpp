#include "SCIPsovler.hpp"

Solver::Solver(Coef problem) : Problem(problem) {};

SCIP_RETCODE Solver::mainproblem()
{

    for (auto agv : this->Problem.AGVs)
    {
        for (auto pair : this->Problem.outvertex)
        {
            int key = pair.first;
            vector<Edge> v = pair.second;
            for (auto e : v)
            {
                SCIP_VAR *var = nullptr;
                string varname = "x_" + to_string(agv.id) + "_" + to_string(e.start_node) + "_" + to_string(e.end_node);
                if (varmap.find(varname) == varmap.end())
                {
                    SCIP_CALL(SCIPcreateVarBasic(this->scip,
                                                 &var,
                                                 varname.c_str(),
                                                 0.0,
                                                 1.0,
                                                 this->Problem.alpha * e.weight,
                                                 SCIP_VARTYPE_BINARY));
                    SCIP_CALL(SCIPaddVar(this->scip, var));
                    this->varmap[varname] = var; // Lưu lại biến vào map
                }
            }
        }
    }

    // Time_Window

    set<int> destination;
    for (auto agv : this->Problem.AGVs)
    {
        for (auto pair : this->Problem.invertex)
        {
            if (agv.end_node % this->Problem.N == pair.first % this->Problem.N)
            {
                destination.insert(pair.first);
            }
        }
    }

    for (auto agv : this->Problem.AGVs)
    {
        for (auto Possible_end_node : destination)
        {
            int totaltime;
            if (Possible_end_node % this->Problem.N == 0)
            {
                totaltime = (Possible_end_node / this->Problem.N) - 1;
            }
            else
            {
                totaltime = (Possible_end_node / this->Problem.N);
            }
            int weight = max(agv.earliness - totaltime, 0);
            weight = max(weight, totaltime - agv.tardliness);
            for (auto e : this->Problem.invertex[Possible_end_node])
            {
                string varname = "x_" + to_string(agv.id) + "_" + to_string(e.start_node) + "_" + to_string(e.end_node);
                SCIP_CALL(SCIPchgVarObj(this->scip, this->varmap[varname], this->Problem.beta * (e.weight + weight)));
            }
        }
    }

    // Restriction
    for (int i = 0; i < this->Problem.restriction.size(); ++i)
    {
        auto pair = this->Problem.restriction[i];
        SCIP_VAR *var = nullptr;
        string varname = "z_" + to_string(i);
        if (this->varmap.find(varname) == this->varmap.end())
        {
            SCIP_CALL(SCIPcreateVarBasic(this->scip,
                                         &var,
                                         varname.c_str(),
                                         0.0,
                                         SCIPinfinity(scip),
                                         this->Problem.gamma,
                                         SCIP_VARTYPE_CONTINUOUS));
            SCIP_CALL(SCIPaddVar(this->scip, var));
            this->varmap[varname] = var;
        }
    }

    // z_i_j >= number of agv - Ur
    for (int i = 0; i < this->Problem.restriction.size(); ++i)
    {

        SCIP_CONS *cons = nullptr;
        SCIP_CALL(SCIPcreateConsBasicLinear(this->scip,
                                            &cons,
                                            "cons_z_ge_x",
                                            0,
                                            nullptr,
                                            nullptr,
                                            0.0,
                                            SCIPinfinity(scip)));
        string z_var = "z_" + to_string(i);
        SCIP_CALL(SCIPaddCoefLinear(this->scip, cons, this->varmap[z_var], 1.0));
        auto pair = this->Problem.restriction[i];
        for (auto agv : this->Problem.AGVs)
        {
            for (auto e : pair.first)
            {
                string varname = "x_" + to_string(agv.id) + "_" + to_string(e.start_node) + "_" + to_string(e.end_node);
                SCIP_CALL(SCIPaddCoefLinear(this->scip, cons, this->varmap[varname], -1.0));
            }
        }
        SCIP_CALL(SCIPchgRhsLinear(this->scip, cons, -pair.second));
        SCIP_CALL(SCIPaddCons(this->scip, cons));
        this->subcons1.push_back(cons);
    }

    // z_i_j >= 0
    for (int i = 0; i < this->Problem.restriction.size(); ++i){
        SCIP_CONS *cons = nullptr;
        SCIP_CALL(SCIPcreateConsBasicLinear(this->scip, 
                                            &cons,
                                            "cons_z_ge_0", 
                                            0, 
                                            nullptr, 
                                            nullptr, 
                                            0.0,
                                            SCIPinfinity(scip)));
        string z_var = "z_" + to_string(i);
        SCIP_CALL(SCIPaddCoefLinear(this->scip, cons, this->varmap[z_var], 1.0));
        SCIP_CALL(SCIPaddCons(this->scip, cons));
    }




    return SCIP_OKAY;
}

SCIP_RETCODE Solver::Constraint1()
{

    for (auto pair : this->Problem.outvertex)
    {
        int node = pair.first;
        // Lặp qua các cạnh xuất phát từ đỉnh này
        for (auto e : pair.second)
        {
            SCIP_CONS *cons = nullptr;

            // Tạo ràng buộc tuyến tính cơ bản cho đỉnh node
            SCIP_CALL(SCIPcreateConsBasicLinear(this->scip,
                                                &cons,
                                                ("Max_flow_at_" + to_string(node)).c_str(),
                                                0,
                                                nullptr,
                                                nullptr,
                                                0.0,
                                                1.0));
            // Thêm biến dòng chảy cho mỗi AGV
            for (auto agv : this->Problem.AGVs)
            {
                std::string varname = "x_" + std::to_string(agv.id) + "_" + to_string(e.start_node) + "_" + to_string(e.end_node);

                // Thêm hệ số biến vào ràng buộc tuyến tính
                SCIP_CALL(SCIPaddCoefLinear(this->scip, cons, this->varmap[varname], 1.0));
            }
            // Thêm ràng buộc vào bài toán SCIP
            SCIP_CALL(SCIPaddCons(this->scip, cons));

            // Lưu trữ ràng buộc để sử dụng sau này
            this->cons1.push_back(cons);
        }
    }

    return SCIP_OKAY; // Trả về mã lỗi thành công
}

SCIP_RETCODE Solver::Constraint2()
{
    set<int> eliminate;
    for (auto agv : this->Problem.AGVs)
    {
        eliminate.insert(agv.start_node);
        for (auto pair : this->Problem.invertex)
        {
            if (agv.end_node % this->Problem.N == pair.first % this->Problem.N)
            {
                eliminate.insert(pair.first);
            }
        }
    }

    for (auto agv : this->Problem.AGVs)
    {
        for (auto pair : this->Problem.outvertex)
        {
            int node = pair.first;
            if (eliminate.find(node) == eliminate.end())
            {
                SCIP_CONS *cons = nullptr;
                SCIP_CALL(SCIPcreateConsBasicLinear(
                    this->scip,
                    &cons,
                    ("flow_at_" + to_string(node)).c_str(),
                    0,
                    nullptr,
                    nullptr,
                    0.0,
                    0.0));
                for (auto e : this->Problem.invertex[node])
                {
                    string varname = "x_" + to_string(agv.id) + "_" + to_string(e.start_node) + "_" + to_string(e.end_node);
                    SCIP_CALL(SCIPaddCoefLinear(this->scip, cons, this->varmap[varname], 1.0));
                }
                for (auto e : this->Problem.outvertex[node])
                {
                    string varname = "x_" + to_string(agv.id) + "_" + to_string(e.start_node) + "_" + to_string(e.end_node);
                    SCIP_CALL(SCIPaddCoefLinear(this->scip, cons, this->varmap[varname], -1.0));
                }
                SCIP_CALL(SCIPaddCons(this->scip, cons));
                this->cons2.push_back(cons);
            }
        }
    }

    return SCIP_OKAY;
}

SCIP_RETCODE Solver::Constraint3()
{
    set<int> destination;
    for (auto agv : this->Problem.AGVs)
    {
        for (auto pair : this->Problem.invertex)
        {
            if (agv.end_node % this->Problem.N == pair.first % this->Problem.N)
            {
                destination.insert(pair.first);
            }
        }
    }

    for (auto agv : this->Problem.AGVs)
    {
        for (auto Possible_end_node : destination)
        {
            SCIP_CONS *cons = nullptr;
            SCIP_CALL(SCIPcreateConsBasicLinear(this->scip,
                                                &cons,
                                                ("Stop_at_" + to_string(Possible_end_node)).c_str(),
                                                0,
                                                nullptr,
                                                nullptr,
                                                0.0,
                                                1.0));
            for (auto e : this->Problem.invertex[Possible_end_node])
            {
                string varname = "x_" + to_string(agv.id) + "_" + to_string(e.start_node) + "_" + to_string(e.end_node);
                SCIP_CALL(SCIPaddCoefLinear(this->scip, cons, this->varmap[varname], 1.0));
            }
            if (this->Problem.outvertex.find(Possible_end_node) != this->Problem.outvertex.end())
            {
                for (auto e : this->Problem.outvertex[Possible_end_node])
                {
                    string varname = "x_" + to_string(agv.id) + "_" + to_string(e.start_node) + "_" + to_string(e.end_node);
                    SCIP_CALL(SCIPaddCoefLinear(this->scip, cons, this->varmap[varname], -1.0));
                }
            }
            SCIP_CALL(SCIPaddCons(this->scip, cons));
            this->cons3.push_back(cons);
        }
    }

    return SCIP_OKAY;
}

SCIP_RETCODE Solver::Constraint4()
{
    for (auto agv : this->Problem.AGVs)
    {
        SCIP_CONS *cons = nullptr;
        SCIP_CALL(SCIPcreateConsBasicLinear(this->scip,
                                            &cons,
                                            ("Start_at_" + to_string(agv.start_node)).c_str(),
                                            0,
                                            nullptr,
                                            nullptr,
                                            1,
                                            1));
        for (auto e : this->Problem.outvertex[agv.start_node])
        {
            string varname = "x_" + to_string(agv.id) + "_" + to_string(e.start_node) + "_" + to_string(e.end_node);
            SCIP_CALL(SCIPaddCoefLinear(this->scip, cons, this->varmap[varname], 1.0));
        }
        if (this->Problem.invertex.find(agv.start_node) != this->Problem.invertex.end())
        {
            for (auto e : this->Problem.invertex[agv.start_node])
            {
                string varname = "x_" + to_string(agv.id) + "_" + to_string(e.start_node) + "_" + to_string(e.end_node);
                SCIP_CALL(SCIPaddCoefLinear(this->scip, cons, this->varmap[varname], -1.0));
            }
        }

        SCIP_CALL(SCIPaddCons(this->scip, cons));

        this->cons4.push_back(cons);
    }

    return SCIP_OKAY;
}

SCIP_RETCODE Solver::Constraint5()
{
    set<int> destination;
    for (auto agv : this->Problem.AGVs)
    {
        for (auto pair : this->Problem.invertex)
        {
            if (agv.end_node % this->Problem.N == pair.first % this->Problem.N)
            {
                destination.insert(pair.first);
            }
        }
    }

    for (auto agv : this->Problem.AGVs)
    {
        SCIP_CONS *cons = nullptr;

        SCIP_CALL(SCIPcreateConsBasicLinear(this->scip,
                                            &cons,
                                            "End_Task_Equal_1",
                                            0,
                                            nullptr,
                                            nullptr,
                                            1.0,
                                            1.0));
        for (auto Possible_end_node : destination)
        {

            for (auto e : this->Problem.invertex[Possible_end_node])
            {
                string varname = "x_" + to_string(agv.id) + "_" + to_string(e.start_node) + "_" + to_string(e.end_node);
                SCIP_CALL(SCIPaddCoefLinear(this->scip, cons, this->varmap[varname], 1.0));
            }
        }

        SCIP_CALL(SCIPaddCons(this->scip, cons));
        this->cons5.push_back(cons);
    }

    return SCIP_OKAY;
}

SCIP_RETCODE Solver::Constraint6()
{

    return SCIP_OKAY;
}

SCIP_RETCODE Solver::Solve()
{

    SCIP_CALL(SCIPcreate(&this->scip));
    SCIP_CALL(SCIPincludeDefaultPlugins(this->scip));

    SCIP_CALL(SCIPcreateProbBasic(this->scip, "Shortest_Path_Problem"));
    SCIP_CALL(mainproblem());
    SCIP_CALL(SCIPsetObjsense(this->scip, SCIP_OBJSENSE_MINIMIZE));

    SCIP_CALL(Constraint1());

    SCIP_CALL(Constraint2());

    SCIP_CALL(Constraint3());

    SCIP_CALL(Constraint4());

    SCIP_CALL(Constraint5());

    // SCIP_CALL(Constraint6());

    SCIP_CALL(SCIPsolve(this->scip));

    return SCIP_OKAY;
}

SCIP_RETCODE Solver::Result()
{

    SCIP_SOL *sol = SCIPgetBestSol(this->scip);

    if (sol != nullptr)
    {
        cout << "Optimal solution found:" << endl;
        for (auto &[varname, var] : this->varmap)
        {
            SCIP_Real val = SCIPgetSolVal(scip, sol, var);
            int int_val = static_cast<int>(val);

            if (int_val == 1)
            {
                cout << varname << " = " << int_val << endl;
            }
        }
    }
    else
    {
        cout << "No solution found." << endl;
    }

    return SCIP_OKAY;
}

SCIP_RETCODE Solver::Free_resources()
{

    for (auto &[varname, var] : this->varmap)
    {
        SCIP_CALL(SCIPreleaseVar(this->scip, &var)); // Đảm bảo tất cả các biến đều được giải phóng
    }

    // Giải phóng các constraints
    for (SCIP_CONS *con : this->subcons1)
    {
        SCIP_CALL(SCIPreleaseCons(this->scip, &con));
    }
    for (SCIP_CONS *con : this->subcons2)
    {
        SCIP_CALL(SCIPreleaseCons(this->scip, &con));
    }
    for (SCIP_CONS *con : this->cons1)
    {
        SCIP_CALL(SCIPreleaseCons(this->scip, &con));
    }
    for (SCIP_CONS *con : this->cons2)
    {
        SCIP_CALL(SCIPreleaseCons(this->scip, &con));
    }
    for (SCIP_CONS *con : this->cons3)
    {
        SCIP_CALL(SCIPreleaseCons(this->scip, &con));
    }
    for (SCIP_CONS *con : this->cons4)
    {
        SCIP_CALL(SCIPreleaseCons(this->scip, &con));
    }
    for (SCIP_CONS *con : this->cons6)
    {
        SCIP_CALL(SCIPreleaseCons(this->scip, &con));
    }

    return SCIP_OKAY;
}