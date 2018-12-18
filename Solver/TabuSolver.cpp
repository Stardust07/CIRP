#include "TabuSolver.h"

#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>


using namespace std;


namespace szx {

bool TabuSolver::solve() {
    if (!greedyInit()) { return false; }
    if (!localSearch()) {
        return true;
    }
    return true;
}

bool TabuSolver::greedyInit() {
    // decide the delivery quantity at each customer in each period.
    IrpModelSolver irpSolver(input);
    irpSolver.routingCost = aux.routingCost;
    if (cfg.findFeasibleFirst) { irpSolver.setFindFeasiblePreference(); }
    if (!irpSolver.solveInventoryModel()) { return false; }

    double elapsedSeconds = irpSolver.getDurationInSecond();
    aux.holdingObj = irpSolver.getHoldingCostInPeriod(0, input.periodNum);
    aux.routingObj = 0;
    
    IrpModelSolver::PresetX &presetX(sln.presetX);
    //IrpModelSolver::PresetX presetX(move(irpSolver.presetX));
    presetX = move(irpSolver.presetX);
    presetX.xEdge.resize(input.vehicleNum);
    // initilize routing in each period as tsp.
    for (int v = 0; v < input.vehicleNum; ++v) {
        presetX.xEdge[v].resize(input.periodNum);
        for (int t = 0; t < input.periodNum; ++t) {
            cout << "\nSolving period " << t;
            if (!generateRouting(presetX.xEdge[v][t], aux.routingObj, elapsedSeconds, presetX.xQuantity[v][t], input)) {
                cout << "Period " << t << " has not solved." << endl;
                return false;
            }
            //presetX.xEdge[v][t].resize(input.nodeNum);
            //for (int i = 0; i < input.nodeNum; ++i) {
            //    presetX.xEdge[v][t][i].resize(input.nodeNum, false);
            //    for (int j = 0; j < input.nodeNum; ++j) {
            //        presetX.xEdge[v][t][i][j] = false;
            //    }
            //}
        }
    }

    sln.bestObj = aux.holdingObj + aux.routingObj;
    sln.elapsedSeconds = elapsedSeconds;
    recordSolution(input, presetX);
    return true;
}

bool TabuSolver::localSearch() {
    //local search
    // initialize tabu table.
    List<List<List<int>>> tabuTable(input.periodNum);
    for (int t = 0; t < input.periodNum; ++t) {
        tabuTable[t].resize(input.nodeNum);
        for (int i = 0; i < input.nodeNum; ++i) {
            tabuTable[t][i].resize(2, -1);
        }
    }

    IrpModelSolver::PresetX &presetX(sln.presetX);  // optimal solution
    double bestObj = sln.bestObj;
    int iter = 0;
    while (/*iter < 200*/bestObj - input.bestObjective > IrpModelSolver::DefaultDoubleGap) {
        IrpModelSolver solver(input);
        IrpModelSolver::PresetX curPresetX(presetX);

        // 邻域动作
        // update routing cost
        auto deleteNode = [&](const ID &v, const ID &t, const ID &n) {
            // delete a node from the path in period 
            bool breakFlag = false;
            for (ID i = 0; ((i < input.nodeNum) && !breakFlag); ++i) {
                if (!curPresetX.xEdge[v][t][i][n]) { continue; }
                for (ID j = 0; ((j < input.nodeNum) && !breakFlag); ++j) {
                    if (!curPresetX.xEdge[v][t][n][j]) { continue; }
                    curPresetX.xEdge[v][t][i][j] = true;
                    curPresetX.xEdge[v][t][i][n] = false;
                    curPresetX.xEdge[v][t][n][j] = false;
                    aux.routingObj = aux.routingObj + aux.routingCost[i][j] - aux.routingCost[i][n] - aux.routingCost[n][j];
                    breakFlag = true;
                }
            }
        };
        auto insertNode = [&](const ID &v, const ID &t, const ID &n) {
            // insert a node into the path in period it
            // 找到一对 ij ，在中间插入 n 使得开销增加最小
            ID u = 0, w = 0;
            int delta = INT32_MAX;
            ID i = 0;
            do {
                for (ID j = 0; j < input.nodeNum; ++j) {
                    if (!curPresetX.xEdge[v][t][i][j]) { continue; }
                    if (aux.routingCost[i][n] + aux.routingCost[n][j] - aux.routingCost[i][j] < delta) {
                        delta = aux.routingCost[i][n] + aux.routingCost[n][j] - aux.routingCost[i][j];
                        u = i;
                        w = j;
                    }
                    i = j;
                    break;
                }
            } while (i != 0);
            curPresetX.xEdge[v][t][u][n] = true;
            curPresetX.xEdge[v][t][n][w] = true;
            curPresetX.xEdge[v][t][u][w] = false;
            aux.routingObj += delta;
        };
        auto findSwapMove = [&](ID &v, ID &t, ID &n) {
            while (true) {
                v = rand() % input.vehicleNum;
                t = rand() % (input.periodNum - 1);
                n = rand() % input.nodeNum;
                if (curPresetX.xVisited[v][t][n] == curPresetX.xVisited[v][t + 1][n]) { continue; }
                if ((curPresetX.xVisited[v][t][n] && (iter <= tabuTable[t][n][1]))
                    || (!curPresetX.xVisited[v][t][n] && (iter <= tabuTable[t][n][0]))) {
                    continue;
                }
                if ((curPresetX.xVisited[v][t + 1][n] && (iter <= tabuTable[t + 1][n][1]))
                    || (!curPresetX.xVisited[v][t + 1][n] && (iter <= tabuTable[t + 1][n][0]))) {
                    continue;
                }
                break;
            }
        };
        auto makeSwapMove = [&](const ID &v, const ID &t, const ID &n) {
            int tabuLen = rand() % 5;
            // make move
            bool temp = curPresetX.xVisited[v][t][n];
            curPresetX.xVisited[v][t][n] = curPresetX.xVisited[v][t + 1][n];
            curPresetX.xVisited[v][t + 1][n] = temp;

            // set tabu table
            tabuTable[t][n][(curPresetX.xVisited[v][t][n] ? 1 : 0)] = iter + tabuLen + rand() % 2;
            tabuTable[t + 1][n][(curPresetX.xVisited[v][t + 1][n] ? 1 : 0)] = iter + tabuLen + rand() % 2;

            if (curPresetX.xVisited[v][t][n]) {
                deleteNode(v, t + 1, n);
                insertNode(v, t, n);
            } else {
                deleteNode(v, t, n);
                insertNode(v, t + 1, n);
            }
        };
        for (int c = 0; c < 1; ++c) {
            ID v, t, n;
            findSwapMove(v, t, n);
            makeSwapMove(v, t, n);
        }

        solver.presetX = curPresetX;
        solver.enablePresetSolution();
        solver.routingCost = aux.routingCost;
        if (cfg.findFeasibleFirst) { solver.setFindFeasiblePreference(); }
        if (!solver.solveInventoryModel()) { continue; }

        curPresetX.xQuantity = solver.presetX.xQuantity;
        double obj = solver.getMpSolverObjValue();
        double duration = solver.getDurationInSecond();

        if (obj + aux.routingObj < bestObj) {
            bestObj = obj + aux.routingObj;
            presetX = curPresetX;
        }
        //recordSolution(originInput, presetX);
        ofstream logFile("iters.txt", ios::app);
        logFile << iter << ":\t" << bestObj << endl;
        logFile.close();
        ++iter;
    }
    return true;
}

void TabuSolver::recordSolution(const IrpModelSolver::Input &input, const IrpModelSolver::PresetX &presetX) {
    for (int v = 0; v < input.vehicleNum; ++v) {
        for (int t = 0; t < input.periodNum; ++t) {
            for (int i = 0; i < input.nodeNum; ++i) {
                if ((presetX.xQuantity[v][t][i] == 0 && presetX.xVisited[v][t][i])
                    || (presetX.xQuantity[v][t][i] > 0 && !presetX.xVisited[v][t][i])) {
                    cout << i << "\t" << presetX.xQuantity[v][t][i] << "\t" << presetX.xVisited[v][t][i] << endl;
                }
            }
        }
    }
    ostringstream oss;
    double x = 0;
    for (int i = 0; i < input.nodeNum; ++i) {
        double cost = 0;
        int q = input.nodes[i].initialQuantity;
        cost += input.nodes[i].holdingCost * q;
        //oss << "node " << i << " (" << input.nodes[i].capacity << "): " << q << "\t";
        oss << "node " << i << " (" << input.nodes[i].capacity << "):\t " << q << "\t";
        for (int t = 0; t < input.periodNum; ++t) {
            for (int v = 0; v < input.vehicleNum; ++v) {
                //oss << q << "+" << lround(presetX.xQuantity[v][t][i]);
                q += (i > 0) ? lround(presetX.xQuantity[v][t][i]) : -lround(presetX.xQuantity[v][t][i]);
            };
            q -= (i > 0) ? input.nodes[i].unitDemand : -input.nodes[i].unitDemand;
            cost += input.nodes[i].holdingCost * q;
            //oss << "-" << input.nodes[i].unitDemand << "=" << q << "\t";
            oss << lround(presetX.xQuantity[0][t][i]) << "\t";
        }
        oss << "/" << cost;
        x += cost;
        oss << endl;
    }
    oss << x << endl;
    ofstream logFile("solution.txt", ios::app);
    logFile << oss.str();
    logFile.close();
}

bool TabuSolver::generateRouting(List<List<bool>>& edges, double & obj, double & seconds, const List<double>& quantity, const IrpModelSolver::Input inp) {
    const int DefaultTimeLimit = 120;

    // if no delivery quantity in the period, return true.
    if (quantity.empty() || (quantity[0] < IrpModelSolver::DefaultDoubleGap)) {
        std::cout << " with 0 nodes.\n\n";
        return true;
    }

    List<int> nodeIndices;
    IrpModelSolver::Input routeInput;
    routeInput.periodNum = 1;
    routeInput.vehicleNum = inp.vehicleNum;
    routeInput.vehicleCapacity = inp.vehicleCapacity;
    routeInput.nodes.push_back(inp.nodes[0]);
    // add supplier.
    nodeIndices.push_back(0);
    edges.resize(inp.nodeNum);
    edges[0].resize(inp.nodeNum, false);
    for (ID i = 1; i < inp.nodeNum; ++i) {
        edges[i].resize(inp.nodeNum, false);
        if (quantity[i] > IrpModelSolver::DefaultDoubleGap) {
            routeInput.nodes.push_back(inp.nodes[i]);
            nodeIndices.push_back(i);
        }
    }
    routeInput.nodeNum = routeInput.nodes.size();

    std::cout << " with " << routeInput.nodeNum << " nodes.\n\n";
    IrpModelSolver routeSolver(routeInput, false);
    routeSolver.setTimeLimitInSecond(DefaultTimeLimit);
    // set routing cost for route solver.
    routeSolver.routingCost.resize(routeInput.nodeNum);
    for (ID i = 0; i < routeInput.nodeNum; ++i) {
        routeSolver.routingCost[i].resize(routeInput.nodeNum, 0);
        for (ID j = 0; j < routeInput.nodeNum; ++j) {
            if (i == j) { continue; }
            routeSolver.routingCost[i][j] = aux.routingCost[nodeIndices[i]][nodeIndices[j]];
        }
    }

    if (!routeSolver.solveTspModel()) { return false; }
    obj += routeSolver.getMpSolverObjValue();
    seconds += routeSolver.getDurationInSecond();

    for (int i = 0; i < routeInput.nodeNum; ++i) {
        for (int j = 0; j < routeInput.nodeNum; ++j) {
            if (i == j) { continue; }
            edges[nodeIndices[i]][nodeIndices[j]] = routeSolver.presetX.xEdge[0][0][i][j];
        }
    }
    return true;
}

}