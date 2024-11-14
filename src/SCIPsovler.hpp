#ifndef SCIP_SOLVER_HPP  
#define SCIP_SOLVER_HPP

#include "coefficient.hpp"
#include <algorithm>
#include <set>
#include "scip/scip.h"
#include "scip/scipdefplugins.h"

class Solver{
    private:
        SCIP *scip = nullptr;
        vector<SCIP_CONS *> subcons1;
        vector<SCIP_CONS *> subcons2;
        vector<SCIP_CONS *> cons1;
        vector<SCIP_CONS *> cons2;
        vector<SCIP_CONS *> cons3;
        vector<SCIP_CONS *> cons4;
        vector<SCIP_CONS *> cons5;
        vector<SCIP_CONS *> cons6;
        map<string, SCIP_VAR *> varmap;
    public:
        Coef Problem;

        Solver(Coef Problem);
        SCIP_RETCODE mainproblem();
        SCIP_RETCODE Constraint1();
        SCIP_RETCODE Constraint2();
        SCIP_RETCODE Constraint3();
        SCIP_RETCODE Constraint4();
        SCIP_RETCODE Constraint5();
        SCIP_RETCODE Constraint6();
        SCIP_RETCODE Solve();
        SCIP_RETCODE Result();
        SCIP_RETCODE Free_resources();
};

#endif