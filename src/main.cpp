#include "reader.hpp"
#include "SCIPsovler.hpp"

int main(){
    string filepath = "/mnt/d/DATN/solver/prj/data/general.txt";
    Reader reader(filepath);
    // Sử dụng hàm set_coef để đọc toàn bộ dữ liệu
    Coef coef = reader.set_coef();
    Solver sol(coef);
    // sol.set_end_queue();
    SCIP_CALL(sol.Solve());
    SCIP_CALL(sol.Result());
    SCIP_CALL(sol.Free_resources());

    return 0;
}