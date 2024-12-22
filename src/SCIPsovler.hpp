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
        vector<SCIP_CONS *> cons;
        map<string, SCIP_VAR *> varmap;
        vector<AGV> agv_end_queue;
        SCIP_RETCODE mainproblem();
        SCIP_RETCODE Constraint1();
        SCIP_RETCODE Constraint2();
        SCIP_RETCODE Constraint3();
        SCIP_RETCODE Constraint4();
        SCIP_RETCODE Constraint5();
        SCIP_RETCODE Constraint6();
        SCIP_RETCODE SubTWcons();
        SCIP_RETCODE SubRestrictioncons();
    public:
        Coef Problem;

        Solver(Coef Problem);
       
        void set_end_queue();
        SCIP_RETCODE Solve();
        SCIP_RETCODE Result();
        SCIP_RETCODE Free_resources();
};

#endif