////////////////////////////////
/// usage : 1.	model of inventory routing problem.
/// 
/// note  : 1.	
////////////////////////////////

#ifndef SMART_QYM_INVENTORY_ROUTING_TABU_SOLVER_H
#define SMART_QYM_INVENTORY_ROUTING_TABU_SOLVER_H


#include <algorithm>

#include "IrpModel.h"


namespace szx {

class TabuSolver {
    #pragma region Type
public:
    //using Input = IrpModelSolver::Input;

    struct Configuration {
        bool findFeasibleFirst;

        Configuration(bool feasible = false):findFeasibleFirst(feasible){}
    } cfg;

    struct Solution {
        IrpModelSolver::PresetX presetX;
        double bestObj;
        double elapsedSeconds;
    };

    struct RemoveMove {
        ID v;
        ID t;
        ID n;
    };

    #pragma endregion Type

    #pragma region Constant
public:
    #pragma endregion Constant

    #pragma region Constructor
public:
    TabuSolver() {}
    TabuSolver(const IrpModelSolver::Input &inp, const List<List<double>> &rc) :input(inp) {
        aux.routingCost = rc;
        aux.tabuTable.resize(input.periodNum);
        for (int t = 0; t < input.periodNum; ++t) {
            aux.tabuTable[t].resize(input.nodeNum);
            for (int i = 0; i < input.nodeNum; ++i) {
                aux.tabuTable[t][i].resize(2, -1);
            }
        }
    }
    #pragma endregion Constructor

    #pragma region Method
public:
    bool solve();

protected:
    bool greedyInit();
    bool localSearch();

    void recordSolution(const IrpModelSolver::Input &input, const IrpModelSolver::PresetX &presetX);


    // solve tsp model.
    bool generateRouting(List<List<bool>> &edges, double &obj, double &seconds,
        const List<double> &quantity, const IrpModelSolver::Input inp);
    #pragma endregion Method

    #pragma region Field
public:
    IrpModelSolver::Input input;
    Solution sln;
protected:
    struct {
        List<List<double>> routingCost;
        // tabuTable[t][n][0] 从未访问变为访问
        // tabuTable[t][n][1] 从访问变为未访问
        List<List<List<int>>> tabuTable;
        double routingObj;
        double holdingObj;
    } aux;
    #pragma endregion Field
}; // TabuSolver

}


#endif SMART_QYM_INVENTORY_ROUTING_TABU_SOLVER_H
