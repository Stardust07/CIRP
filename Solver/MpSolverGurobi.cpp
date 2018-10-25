#include "MpSolverGurobi.h"


using namespace std;


GRBEnv MpSolverGurobi::globalEnv;

MpSolverGurobi::MpSolverGurobi() : model(globalEnv), status(ResultStatus::Ready) {}

void MpSolverGurobi::updateStatus() {
    switch (model.get(GRB_IntAttr_Status)) {
    case GRB_OPTIMAL:
    case GRB_INTERRUPTED:
        status = ResultStatus::Optimal; break;
    case GRB_SUBOPTIMAL:
        status = ResultStatus::Feasible; break;
    case GRB_LOADED:
    case GRB_INPROGRESS:
        status = ResultStatus::Proceeding; break;
    case GRB_ITERATION_LIMIT:
    case GRB_NODE_LIMIT:
    case GRB_TIME_LIMIT:
    case GRB_SOLUTION_LIMIT:
        status = ResultStatus::ExceedLimit; break;
    case GRB_CUTOFF:
        status = ResultStatus::InsolubleCutoff; break;
    case GRB_INFEASIBLE:
    case GRB_INF_OR_UNBD:
    case GRB_UNBOUNDED:
        status = ResultStatus::InsolubleModel; break;
    default:
        status = ResultStatus::Error; break;
    }
    if (model.get(GRB_IntAttr_SolCount) > 0) { status = ResultStatus::Feasible; }
}

bool MpSolverGurobi::reportStatus(ResultStatus status) {
    switch (status) {
    case Optimal:
    case Feasible:
        return true;
    case Proceeding:
    case Ready:
        cout << "Unsolved." << endl; break;
    case InsolubleCutoff:
    case InsolubleModel:
        cout << "Infeasible." << endl; break;
    case ExceedLimit:
        cout << "Exceed Limit." << endl; break;
    case OutOfMemory:
        cout << "Out Of Memory." << endl; break;
    case Error:
    default:
        cout << "Unknown Error." << endl; break;
    }
    return false;
}

bool MpSolverGurobi::optimize() {
    return reportStatus(solve());
}

bool MpSolverGurobi::optimizeManually() {
    bool ret = reportStatus(solve());
    while (postprocess()) {
        ret = reportStatus(solve());
    }
    return ret;
}