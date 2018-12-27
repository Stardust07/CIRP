#include "IrpModel.h"


using namespace std;


static constexpr char Delimiter = '_';
std::string str(const char *obj) { return std::string(obj); }
template<typename T>
std::string str(const T &obj) { return std::to_string(obj); }
template<typename T, typename ... Ts>
std::string str(const T &obj, Ts ... objs) { return str(obj) + Delimiter + str(objs ...); }


namespace szx {

bool IrpModelSolver::solve() {
    initSolver();

    addDecisionVars();
    addPathConnectivityConstraint();
    addDeliveryQuantityConstraint();
    addLoadDeliveryBalanceConstraint();
    addCustomerLevelConstraint();
    addSupplierLevelConstraint();
    if (!cfg.useLazyConstraints) {
        addSubtourEliminationConstraint();
    }
    if (cfg.useBenchmark || (cfg.useLazyConstraints && !cfg.allowSubtour)) {
        setSolutionFoundCallback();
    }
    
    if (cfg.usePresetSolution) {
        setInitSolution();
    }
    if (cfg.allowShortage) {
        setShortageQuantityObjective();
        if (!mpSolver.optimize() || mpSolver.getObjectiveValue() > 0) {
            elapsedSeconds = DefaultTimeLimitSecond;
            return false;
        }
        mpSolver.makeConstraint(totalShortageQuantity() <= mpSolver.getObjectiveValue());
    } else {
        setTotalCostObjective();
        if (!mpSolver.optimize()) {
            elapsedSeconds = mpSolver.getDurationInSecond();
            return false;
        }
        elapsedSeconds = (cfg.usePresetSolution || !cfg.useBenchmark) ? mpSolver.getDurationInSecond() : elapsedSeconds;
    }
    retrieveSolution();

    return true;
    //return check();
}

bool IrpModelSolver::solveIteratively() {
    bool lazy = true;

    initSolver();
    relaxTspSubtourConstraint();
    currentObjective = { 0, 0, INFINITY };

    addDecisionVars();
    addPathConnectivityConstraint();
    addDeliveryQuantityConstraint();
    addLoadDeliveryBalanceConstraint();
    addCustomerLevelConstraint();
    addSupplierLevelConstraint();
    setTotalCostObjective();

    // use lazy constraint
    if(lazy) {
        InfeasibleFound cb(*this);
        mpSolver.setCallback(&cb);
        //mpSolver.makeConstraint(x.xQuantity[0][0][0] <= 0);
        //mpSolver.makeConstraint(x.xQuantity[0][5][0] <= 0);
        if (!mpSolver.optimize()) {
            elapsedSeconds = mpSolver.getDurationInSecond();
            return (currentObjective.totalCost < INFINITY);
        }
    } else { // model
        while (elapsedSeconds < DefaultTimeLimitSecond) {
            if (!mpSolver.optimize()) {
                elapsedSeconds = mpSolver.getDurationInSecond();
                return (currentObjective.totalCost < INFINITY);
            }
            elapsedSeconds += mpSolver.getDurationInSecond();
            // solve tsp with lkh.
            List2D<lkh::Tour> tours;
            double holdingCost = getHoldingCostInPeriod(0, input.periodNum);
            double routingCost =
                solveVrpWithLkh(tours, [&](ID v, ID t, ID i) { return (mpSolver.getValue(x.xQuantity[v][t][i]) > DefaultDoubleGap); });
            if (routingCost < 0) {
                // failed to solve tsp with lkh
                ;
            }

            // update optimal.
            if (holdingCost + routingCost < currentObjective.totalCost) {
                cout << "* ";
                currentObjective.holdingCost = holdingCost;
                currentObjective.routingCost = routingCost;
                currentObjective.totalCost = holdingCost + routingCost;
                retrieveDeliveryQuantity(presetX, [&](ID v, ID t, ID i) { return mpSolver.getValue(x.xQuantity[v][t][i]); });
                convertTourToEdges(presetX, tours);
            }
            cout << (holdingCost + routingCost) << "(T) = " << holdingCost << "(H) + " << routingCost << "(R)" << endl;

            // forbid current solution with lazy constraints
            // gurantee that visited node set different from current solution.
            MpSolver::LinearExpr nodeDiff = 0;
            for (ID v = 0; v < input.vehicleNum; ++v) {
                for (ID t = 0; t < input.periodNum; ++t) {
                    for (ID i = 1; i < input.nodeNum; ++i) {
                        MpSolver::LinearExpr degree = 0;
                        for (ID j = 0; j < input.nodeNum; ++j) {
                            if (j == i) { continue; }
                            degree += x.xEdge[v][t][i][j];
                        }
                        if (mpSolver.getValue(x.xQuantity[v][t][i]) <= DefaultDoubleGap) {
                            // unvisited node.
                            nodeDiff += degree;
                        } else {
                            // visited node.
                            nodeDiff += 1 - degree;
                        }
                    }
                }
            }
            mpSolver.makeConstraint(nodeDiff >= 1);
        }
    }
    return true;
}

bool IrpModelSolver::solveInventoryModel() {
    initSolver();
    cfg.useBenchmark = false;

    addInventoryVariables();
    addNodeCapacityConstraint();
    addQuantityConsistencyConstraint();
    setHoldingCostObjective();
    if (!mpSolver.optimize()) {
        elapsedSeconds = mpSolver.getDurationInSecond();
        return false;
    }

    // retrieve solution
    presetX.xQuantity.resize(input.vehicleNum);
    for (ID v = 0; v < input.vehicleNum; ++v) {
        presetX.xQuantity[v].resize(input.periodNum);
        for (ID t = 0; t < input.periodNum; ++t) {
            presetX.xQuantity[v][t].resize(input.nodeNum);
            fill(presetX.xQuantity[v][t].begin(), presetX.xQuantity[v][t].end(), 0);
            for (ID i = 0; i < input.nodeNum; ++i) {
                presetX.xQuantity[v][t][i] = mpSolver.getValue(x.xQuantity[v][t][i]);
            }
        }
    }

    presetX.xVisited.resize(input.vehicleNum);
    for (ID v = 0; v < input.vehicleNum; ++v) {
        presetX.xVisited[v].resize(input.periodNum);
        for (ID t = 0; t < input.periodNum; ++t) {
            if (cfg.usePresetSolution && presetX.isPeriodFixed[t] && !cfg.optimizeTotalCost) { continue; }
            presetX.xVisited[v][t].resize(input.nodeNum);
            fill(presetX.xVisited[v][t].begin(), presetX.xVisited[v][t].end(), false);
            for (ID i = 0; i < input.nodeNum; ++i) {
                presetX.xVisited[v][t][i] = mpSolver.isTrue(x.xVisited[v][t][i]);
            }
        }
    }
    return true;
}

bool IrpModelSolver::solveTspModel() {
    initSolver();
    cfg.useBenchmark = false;

    addRoutingVariables();
    addNodeDegreeConstraints();
    setRoutingCostObjective();
    mpSolver.setCallback(&tspCallback);
    if (!mpSolver.optimize()) {
        elapsedSeconds = mpSolver.getDurationInSecond();
        return false;
    }

    // retrieve solution
    presetX.xEdge.resize(input.vehicleNum);
    for (ID v = 0; v < input.vehicleNum; ++v) {
        presetX.xEdge[v].resize(input.periodNum);
        for (ID t = 0; t < input.periodNum; ++t) {
            presetX.xEdge[v][t].resize(input.nodeNum);
            for (ID i = 0; i < input.nodeNum; ++i) {
                presetX.xEdge[v][t][i].resize(input.nodeNum);
                for (ID j = 0; j < input.nodeNum; ++j) {
                    if (i == j) { continue; }
                    presetX.xEdge[v][t][i][j] = mpSolver.isTrue(x.xEdge[v][t][i][j]);

                }
            }
        }
    }
    return true;
}

bool IrpModelSolver::optimizeInventory() {
    initSolver();
    cfg.useBenchmark = false;

    // optimize inventory only while the visited node set is determined. 
    addInventoryVariables();
    addNodeCapacityConstraint();
    addQuantityConsistencyConstraint();
    setHoldingCostObjective();

    if (!mpSolver.optimize()) {
        elapsedSeconds = mpSolver.getDurationInSecond();
        return false;
    }

    // retrieve solution
    presetX.xQuantity.resize(input.vehicleNum);
    for (ID v = 0; v < input.vehicleNum; ++v) {
        presetX.xQuantity[v].resize(input.periodNum);
        for (ID t = 0; t < input.periodNum; ++t) {
            presetX.xQuantity[v][t].resize(input.nodeNum);
            fill(presetX.xQuantity[v][t].begin(), presetX.xQuantity[v][t].end(), 0);
            for (ID i = 0; i < input.nodeNum; ++i) {
                presetX.xQuantity[v][t][i] = mpSolver.getValue(x.xQuantity[v][t][i]);
            }
        }
    }
    return true;
}

void IrpModelSolver::retrieveSolution() {
    presetX.xEdge.resize(input.vehicleNum);
    for (ID v = 0; v < input.vehicleNum; ++v) {
        presetX.xEdge[v].resize(input.periodNum);
        for (ID t = 0; t < input.periodNum; ++t) {
            if (cfg.usePresetSolution && presetX.isPeriodFixed[t]) { continue; }
            presetX.xEdge[v][t].resize(input.nodeNum);
            for (ID i = 0; i < input.nodeNum; ++i) {
                presetX.xEdge[v][t][i].resize(input.nodeNum);
                fill(presetX.xEdge[v][t][i].begin(), presetX.xEdge[v][t][i].end(), false);
                if (aux.skipNode[t][i]) { continue; }
                for (ID j = 0; j < input.nodeNum; ++j) {
                    if ((i == j) || aux.skipNode[t][j]) { continue; }
                    presetX.xEdge[v][t][i][j] = mpSolver.isTrue(x.xEdge[v][t][i][j]);
                }
            }
        }
    }

    presetX.xQuantity.resize(input.vehicleNum);
    for (ID v = 0; v < input.vehicleNum; ++v) {
        presetX.xQuantity[v].resize(input.periodNum);
        for (ID t = 0; t < input.periodNum; ++t) {
            if (cfg.usePresetSolution && presetX.isPeriodFixed[t] && !cfg.optimizeTotalCost) { continue; }
            presetX.xQuantity[v][t].resize(input.nodeNum);
            fill(presetX.xQuantity[v][t].begin(), presetX.xQuantity[v][t].end(), 0);
            for (ID i = 0; i < input.nodeNum; ++i) {
                if (aux.skipNode[t][i]) { continue; }
                presetX.xQuantity[v][t][i] = mpSolver.getValue(x.xQuantity[v][t][i]);
            }
        }
    }

    // skip nodes without delivery quantity.
    for (ID t = 0; t < input.periodNum; ++t) {
        for (ID v = 0; v < input.vehicleNum; ++v) {
            ID i = 0;
            ID prev = -1;
            for (ID j = 0; j < input.nodeNum; ++j) {
                if (i == j) { continue; }
                if (!presetX.xEdge[v][t][i][j]) { continue; }
                
                if (lround(presetX.xQuantity[v][t][j]) <= 0) {
                    presetX.xEdge[v][t][i][j] = false;
                    if (prev < 0) { prev = i; }
                } else if (prev >= 0) {
                    presetX.xEdge[v][t][i][j] = false;
                    presetX.xEdge[v][t][prev][j] = true;
                    prev = -1;
                }

                if (j == 0) { break; }
                i = j;
                j = -1;
            }
            if (prev >= 0) { presetX.xEdge[v][t][prev][0] = true; }
        }
    }

    if (cfg.useLazyConstraints) { return; }
    presetX.xSequence.resize(input.vehicleNum);
    for (ID v = 0; v < input.vehicleNum; ++v) {
        presetX.xSequence[v].resize(input.periodNum);
        for (ID t = 0; t < input.periodNum; ++t) {
            if (cfg.usePresetSolution && presetX.isPeriodFixed[t]) { continue; }
            presetX.xSequence[v][t].resize(input.nodeNum);
            fill(presetX.xSequence[v][t].begin(), presetX.xSequence[v][t].end(), 0);
            for (ID i = 0; i < input.nodeNum; ++i) {
                if (aux.skipNode[t][i]) { continue; }
                presetX.xSequence[v][t][i] = mpSolver.getValue(x.xSequence[v][t][i]);
            }
        }
    }
}

void IrpModelSolver::retrieveDeliveryQuantity(PresetX &presetX, std::function<double(ID v, ID t, ID i)> getQuantity) {
    presetX.xEdge.resize(input.vehicleNum);
    for (ID v = 0; v < input.vehicleNum; ++v) {
        presetX.xEdge[v].resize(input.periodNum);
        for (ID t = 0; t < input.periodNum; ++t) {
            presetX.xEdge[v][t].resize(input.nodeNum);
            for (ID i = 0; i < input.nodeNum; ++i) {
                presetX.xEdge[v][t][i].resize(input.nodeNum);
                fill(presetX.xEdge[v][t][i].begin(), presetX.xEdge[v][t][i].end(), false);
            }
        }
    }

    presetX.xQuantity.resize(input.vehicleNum);
    for (ID v = 0; v < input.vehicleNum; ++v) {
        presetX.xQuantity[v].resize(input.periodNum);
        for (ID t = 0; t < input.periodNum; ++t) {
            presetX.xQuantity[v][t].resize(input.nodeNum);
            fill(presetX.xQuantity[v][t].begin(), presetX.xQuantity[v][t].end(), 0);
            for (ID i = 0; i < input.nodeNum; ++i) {
                presetX.xQuantity[v][t][i] = getQuantity(v, t, i);
            }
        }
    }
}

void IrpModelSolver::saveSolution(const Input &input, const PresetX &presetX, const string &path) {
    ofstream ofs(path);

    for (ID t = 0; t < input.periodNum; ++t) {
        ofs << "period," << t << ",================================" << endl;
        for (ID v = 0; v < input.vehicleNum; ++v) {
            ostringstream route;
            route << v << ",";
            ostringstream quant;
            quant.precision(2);
            quant << ",";

            if (!presetX.xEdge[v][t].empty()) {
                ID i = 0;
                for (ID j = 0; j < input.nodeNum; ++j) {
                    if (i == j) { continue; }
                    if (!presetX.xEdge[v][t][i][j]) { continue; }

                    route << i << ",";
                    quant << fixed << presetX.xQuantity[v][t][i] << ",";

                    if (j == 0) { break; }
                    i = j;
                    j = -1;
                }
            }

            ofs << route.str() << endl << quant.str() << endl;
        }
    }
}

void IrpModelSolver::addDecisionVars() {
    x.xEdge.resize(input.vehicleNum);
    for (ID v = 0; v < input.vehicleNum; ++v) {
        x.xEdge[v].resize(input.periodNum);
        for (ID t = 0; t < input.periodNum; ++t) {
            if (cfg.usePresetSolution && presetX.isPeriodFixed[t]) { continue; }
            x.xEdge[v][t].resize(input.nodeNum);
            for (ID i = 0; i < input.nodeNum; ++i) {
                if (aux.skipNode[t][i]) { continue; }
                x.xEdge[v][t][i].resize(input.nodeNum);
                for (ID j = 0; j < input.nodeNum; ++j) {
                    if (aux.skipNode[t][j]) { continue; }
                    if (i == j) { continue; }
                    ostringstream oss;
                    oss << "x_" << v << "_" << t << "_" << i << "_" << j;
                    x.xEdge[v][t][i][j] = mpSolver.makeVar(MpSolver::VariableType::Bool, 0, 1, oss.str());
                }
            }
        }
    }

    x.xQuantity.resize(input.vehicleNum);
    for (ID v = 0; v < input.vehicleNum; ++v) {
        x.xQuantity[v].resize(input.periodNum);
        for (ID t = 0; t < input.periodNum; ++t) {
            if (cfg.usePresetSolution && presetX.isPeriodFixed[t] && !cfg.optimizeTotalCost) { continue; }
            x.xQuantity[v][t].resize(input.nodeNum);
            for (ID i = 0; i < input.nodeNum; ++i) {
                if (aux.skipNode[t][i]) { continue; }
                ostringstream oss;
                oss << "q_" << v << "_" << t << "_" << i;
                x.xQuantity[v][t][i] = mpSolver.makeVar(MpSolver::VariableType::Real, 0, min(input.vehicleCapacity, input.nodes[i].capacity), oss.str());
            }
        }
    }
    if (cfg.allowShortage) {
        x.xMinLevel.resize(input.nodeNum);
        x.xShortage.resize(input.nodeNum);
        x.xShortage[0] = mpSolver.makeVar(MpSolver::VariableType::Bool);
        for (ID i = 1; i < input.nodeNum; ++i) {
            x.xMinLevel[i] = mpSolver.makeVar(MpSolver::VariableType::Real, -Infinity, input.nodes[i].capacity);
            x.xShortage[i] = mpSolver.makeVar(MpSolver::VariableType::Real, 0, Infinity);
        }
    }

    if (cfg.useLazyConstraints) { return; }
    x.xSequence.resize(input.vehicleNum);
    for (ID v = 0; v < input.vehicleNum; ++v) {
        x.xSequence[v].resize(input.periodNum);
        for (ID t = 0; t < input.periodNum; ++t) {
            if (cfg.usePresetSolution && presetX.isPeriodFixed[t]) { continue; }
            x.xSequence[v][t].resize(input.nodeNum);
            for (ID i = 0; i < input.nodeNum; ++i) {
                if (aux.skipNode[t][i]) { continue; }
                ostringstream oss;
                oss << "u_" << v << "_" << t << "_" << i;
                x.xSequence[v][t][i] = mpSolver.makeVar(MpSolver::VariableType::Integer, 1, input.nodeNum, oss.str());
            }
        }
    }
}

void IrpModelSolver::addPathConnectivityConstraint() {
    for (ID t = 0; t < input.periodNum; ++t) {
        if (cfg.usePresetSolution && presetX.isPeriodFixed[t]) { continue; }
        for (ID i = 0; i < input.nodeNum; ++i) {
            if (aux.skipNode[t][i]) { continue; }
            MpSolver::LinearExpr visitedTime = 0;
            for (ID v = 0; v < input.vehicleNum; ++v) {
                MpSolver::LinearExpr inEdge = 0;
                MpSolver::LinearExpr outEdge = 0;
                for (ID j = 0; j < input.nodeNum; ++j) {
                    if (i == j) { continue; }
                    if (aux.skipNode[t][j]) { continue; }
                    inEdge += x.xEdge[v][t][j][i];
                    outEdge += x.xEdge[v][t][i][j];
                }
                mpSolver.makeConstraint(inEdge == outEdge, "io");
                if (i == 0) {
                    mpSolver.makeConstraint(inEdge <= 1);
                }
                visitedTime += inEdge;
            }
            if (i > 0) {
                mpSolver.makeConstraint(visitedTime <= 1, "vt");
            }
        }
    }
}

void IrpModelSolver::addDeliveryQuantityConstraint() {
    for (ID t = 0; t < input.periodNum; ++t) {
        if (cfg.usePresetSolution && presetX.isPeriodFixed[t] && !cfg.optimizeTotalCost) { continue; }
        // for supplier
        for (ID v = 0; v < input.vehicleNum; ++v) {
            MpSolver::LinearExpr visitedTime = 0;
            for (ID j = 1; j < input.nodeNum; ++j) {
                if (aux.skipNode[t][j]) { continue; }
                if (cfg.usePresetSolution && presetX.isPeriodFixed[t] && cfg.optimizeTotalCost) {
                    if (presetX.xEdge[v][t][0][j]) { visitedTime += 1; break; }
                } else {
                    visitedTime += x.xEdge[v][t][0][j];
                }
            }
            mpSolver.makeConstraint(x.xQuantity[v][t][0] <= input.vehicleCapacity * visitedTime, "qv0");
        }

        // for customers
        for (ID i = 1; i < input.nodeNum; ++i) {
            if (aux.skipNode[t][i]) { continue; }
            MpSolver::LinearExpr totalQuantity = 0;
            MpSolver::LinearExpr visitedTime = 0;
            for (ID v = 0; v < input.vehicleNum; ++v) {
                totalQuantity += x.xQuantity[v][t][i];
                for (ID j = 0; j < input.nodeNum; ++j) {
                    if (i == j) { continue; }
                    if (aux.skipNode[t][j]) { continue; }
                    if (cfg.usePresetSolution && presetX.isPeriodFixed[t] && cfg.optimizeTotalCost) {
                        if (presetX.xEdge[v][t][i][j]) { visitedTime += 1; break; }
                    } else {
                        visitedTime += x.xEdge[v][t][i][j];
                    }
                }
            }
            mpSolver.makeConstraint(totalQuantity <= input.nodes[i].capacity * visitedTime, "qv");
        }
    }

    //for (ID v = 0; v < input.vehicleNum; ++v) {
    //    for (ID t = 0; t < input.periodNum; ++t) {
    //        for (ID i = 1; i < input.nodeNum; ++i) {
    //            if (aux.skipNode[t][i]) { continue; }
    //            MpSolver::LinearExpr visitedTime = 0;
    //            for (ID j = 0; j < input.nodeNum; ++j) {
    //                if (i == j) { continue; }
    //                if (aux.skipNode[t][j]) { continue; }
    //                visitedTime += x.xEdge[v][t][i][j];
    //            }
    //            mpSolver.makeConstraint(x.xQuantity[v][t][i] <= min(input.vehicleCapacity, input.nodes[i].capacity) * visitedTime);
    //        }
    //    }
    //}
}

void IrpModelSolver::addLoadDeliveryBalanceConstraint() {
    for (ID v = 0; v < input.vehicleNum; ++v) {
        for (ID t = 0; t < input.periodNum; ++t) {
            if (cfg.usePresetSolution && presetX.isPeriodFixed[t] && !cfg.optimizeTotalCost) { continue; }
            MpSolver::LinearExpr totalQuantity = 0;
            for (ID i = 1; i < input.nodeNum; ++i) {
                if (aux.skipNode[t][i]) { continue; }
                totalQuantity += x.xQuantity[v][t][i];
            }
            mpSolver.makeConstraint(x.xQuantity[v][t][0] == totalQuantity, "db");
        }
    }
}

void IrpModelSolver::addCustomerLevelConstraint() {
    for (ID i = 1; i < input.nodeNum; ++i) {
        MpSolver::LinearExpr restQuantity = 0;
        restQuantity += input.nodes[i].initialQuantity;
        for (ID t = 0; t < input.periodNum; ++t) {
            if (!aux.skipNode[t][i]) {
                for (ID v = 0; v < input.vehicleNum; ++v) {
                    if (cfg.usePresetSolution && presetX.isPeriodFixed[t] && !cfg.optimizeTotalCost) {
                        restQuantity += presetX.xQuantity[v][t][i];
                    } else {
                        restQuantity += x.xQuantity[v][t][i];
                    }
                }
            }
            mpSolver.makeConstraint(restQuantity <= input.nodes[i].capacity, str("mc", i, t));
            restQuantity -= input.nodes[i].unitDemand;
            mpSolver.makeConstraint(restQuantity >= (cfg.allowShortage ? x.xMinLevel[i] : MpSolver::LinearExpr(input.nodes[i].minLevel)), "ml");
        }
        if (cfg.allowShortage) {
            mpSolver.makeConstraint(x.xShortage[i] >= -x.xMinLevel[i], "sm");
        }
    }
}

void IrpModelSolver::addSupplierLevelConstraint() {
    MpSolver::LinearExpr restQuantity = 0;
    restQuantity += input.nodes[0].initialQuantity;
    for (ID t = 0; t < input.periodNum; ++t) {
        for (ID v = 0; v < input.vehicleNum; ++v) {
            if (cfg.usePresetSolution && presetX.isPeriodFixed[t] && !cfg.optimizeTotalCost) {
                restQuantity -= presetX.xQuantity[v][t][0];
            } else {
                restQuantity -= x.xQuantity[v][t][0];
            }
        }
        // TODO[qym][5]: adjust the order 
        //mpSolver.makeConstraint(restQuantity >= input.nodes[0].minLevel, "sr");
        //restQuantity += input.nodes[0].unitDemand;
        restQuantity += input.nodes[0].unitDemand;
        mpSolver.makeConstraint(restQuantity >= input.nodes[0].minLevel, "sr");
    }
}

void IrpModelSolver::addSubtourEliminationConstraint() {
    for (ID v = 0; v < input.vehicleNum; ++v) {
        for (ID t = 0; t < input.periodNum; ++t) {
            for (ID i = 1; i < input.nodeNum; ++i) {
                if (aux.skipNode[t][i]) { continue; }
                for (ID j = 1; j < input.nodeNum; ++j) {
                    if (i == j) { continue; }
                    if (aux.skipNode[t][j]) { continue; }
                    mpSolver.makeConstraint(
                        x.xSequence[v][t][i] - x.xSequence[v][t][j] <= input.nodeNum - 1 - input.nodeNum * x.xEdge[v][t][i][j], "se");
                }
            }
        }
    }
}

void IrpModelSolver::setInitSolution() {
    if (!cfg.usePresetSolution || presetX.xEdge.empty()) { return; }

    for (ID v = 0; v < input.vehicleNum; ++v) {
        for (ID t = 0; t < input.periodNum; ++t) {
            if (presetX.isPeriodFixed[t]) { continue; }
            for (ID i = 0; i < input.nodeNum; ++i) {
                if (aux.skipNode[t][i]) { continue; }
                for (ID j = 0; j < input.nodeNum; ++j) {
                    if (i == j) { continue; }
                    if (aux.skipNode[t][j]) { continue; }
                    mpSolver.setInitValue(x.xEdge[v][t][i][j], presetX.xEdge[v][t][i][j]);
                }
            }
        }
    }
    for (ID v = 0; v < input.vehicleNum; ++v) {
        for (ID t = 0; t < input.periodNum; ++t) {
            if (presetX.isPeriodFixed[t] && !cfg.optimizeTotalCost) { continue; }
            for (ID i = 0; i < input.nodeNum; ++i) {
                if (aux.skipNode[t][i]) { continue; }
                mpSolver.setInitValue(x.xQuantity[v][t][i], presetX.xQuantity[v][t][i]);
            }
        }
    }

    if (cfg.useLazyConstraints) { return; }
    for (ID v = 0; v < input.vehicleNum; ++v) {
        for (ID t = 0; t < input.periodNum; ++t) {
            if (presetX.isPeriodFixed[t]) { continue; }
            for (ID i = 0; i < input.nodeNum; ++i) {
                if (aux.skipNode[t][i]) { continue; }
                mpSolver.setInitValue(x.xSequence[v][t][i], presetX.xSequence[v][t][i]);
            }
        }
    }
}

void IrpModelSolver::addInventoryVariables() {
    x.xQuantity.resize(input.vehicleNum);
    x.xVisited.resize(input.vehicleNum);
    for (ID v = 0; v < input.vehicleNum; ++v) {
        x.xQuantity[v].resize(input.periodNum);
        x.xVisited[v].resize(input.periodNum);
        for (ID t = 0; t < input.periodNum; ++t) {
            x.xQuantity[v][t].resize(input.nodeNum);
            x.xVisited[v][t].resize(input.nodeNum);

            // load quantity at supplier is negative in irp model, positive otherwise.  
            for (ID i = 0; i < input.nodeNum; ++i) {
                x.xQuantity[v][t][i] = mpSolver.makeVar(
                    MpSolver::VariableType::Real, 0, min(input.vehicleCapacity, input.nodes[i].capacity));
                if (cfg.usePresetSolution) {
                    double value = (presetX.xVisited[v][t][i] ? 1 : 0);
                    x.xVisited[v][t][i] = mpSolver.makeVar(MpSolver::VariableType::Bool, value, value);
                } else {
                    x.xVisited[v][t][i] = mpSolver.makeVar(MpSolver::VariableType::Bool);
                }
            }
        }
    }
    // relationship between xQuantity and xVisited.
    for (ID v = 0; v < input.vehicleNum; ++v) {
        for (ID t = 0; t < input.periodNum; ++t) {
            for (ID i = 1; i < input.nodeNum; ++i) {
                mpSolver.makeConstraint(x.xQuantity[v][t][i] <= min(input.vehicleCapacity, input.nodes[i].capacity) * x.xVisited[v][t][i]);
                mpSolver.makeConstraint(x.xVisited[v][t][i] <= x.xQuantity[v][t][i]);
            }
        }
    }
}

void IrpModelSolver::addNodeCapacityConstraint() {
    // for customers.
    for (ID i = 1; i < input.nodeNum; ++i) {
        MpSolver::LinearExpr restQuantity = input.nodes[i].initialQuantity;
        for (ID t = 0; t < input.periodNum; ++t) {
            for (ID v = 0; v < input.vehicleNum; ++v) {
                restQuantity += x.xQuantity[v][t][i];
            }
            mpSolver.makeConstraint(restQuantity <= input.nodes[i].capacity);
            restQuantity -= input.nodes[i].unitDemand;
            mpSolver.makeConstraint(restQuantity >= input.nodes[i].minLevel);
        }
    }
    // for supplier.
    MpSolver::LinearExpr restQuantity = input.nodes[0].initialQuantity;
    for (ID t = 0; t < input.periodNum; ++t) {
        for (ID v = 0; v < input.vehicleNum; ++v) {
            restQuantity -= x.xQuantity[v][t][0];
        }
        restQuantity += input.nodes[0].unitDemand;
        mpSolver.makeConstraint(restQuantity >= input.nodes[0].minLevel);
    }
}

void IrpModelSolver::addQuantityConsistencyConstraint() {
    for (ID v = 0; v < input.vehicleNum; ++v) {
        for (ID t = 0; t < input.periodNum; ++t) {
            MpSolver::LinearExpr expr = -x.xQuantity[v][t][0];
            for (ID i = 1; i < input.nodeNum; ++i) {
                expr += x.xQuantity[v][t][i];
            }
            mpSolver.makeConstraint(expr == 0);
        }
    }
}

void IrpModelSolver::setHoldingCostObjective() {
    const int ObjWeight = 0;

    MpSolver::LinearExpr totalCost = 0;
    MpSolver::LinearExpr restQuantity = input.nodes[0].initialQuantity;
    totalCost += input.nodes[0].holdingCost * restQuantity;
    for (ID t = 0; t < input.periodNum; ++t) {
        for (ID v = 0; v < input.vehicleNum; ++v) {
            restQuantity -= x.xQuantity[v][t][0];
        }
        restQuantity += input.nodes[0].unitDemand;
        totalCost += input.nodes[0].holdingCost * restQuantity;
    }

    for (ID i = 1; i < input.nodeNum; ++i) {
        restQuantity = input.nodes[i].initialQuantity;
        totalCost += input.nodes[i].holdingCost * restQuantity;
        for (ID t = 0; t < input.periodNum; ++t) {
            for (ID v = 0; v < input.vehicleNum; ++v) {
                restQuantity += x.xQuantity[v][t][i];
            }
            restQuantity -= input.nodes[i].unitDemand;
            totalCost += input.nodes[i].holdingCost * restQuantity;
        }
    }

    for (ID i = 1; i < input.nodeNum; ++i) {
        int leastDeliveryNum = ceil(double(input.nodes[i].unitDemand * 6 - input.nodes[i].initialQuantity) / input.nodes[i].capacity);
        MpSolver::LinearExpr times = 0;
        for (ID v = 0; v < input.vehicleNum; ++v) {
            for (ID t = 0; t < input.periodNum; ++t) {
                times += x.xVisited[v][t][i];
            }
        }
        //totalCost += (times - leastDeliveryNum) * 0;
    }

    MpSolver::LinearExpr estimatedRoutingCost = 0;
    for (ID i = 1; i < input.nodeNum; ++i) {
        for (ID t = 0; t < input.periodNum; ++t) {
            for (ID v = 0; v < input.vehicleNum; ++v) {
                estimatedRoutingCost += (routingCost[0][i] + routingCost[i][0]) * x.xVisited[v][t][i];
            }
        }
    }

    //totalCost = estimatedRoutingCost;

    mpSolver.setObjective(totalCost, MpSolver::OptimaOrientation::Minimize);
}

void IrpModelSolver::addRoutingVariables() {
    x.xEdge.resize(input.vehicleNum);
    for (ID v = 0; v < input.vehicleNum; ++v) {
        x.xEdge[v].resize(input.periodNum);
        for (ID t = 0; t < input.periodNum; ++t) {
            x.xEdge[v][t].resize(input.nodeNum);
            for (ID i = 0; i < input.nodeNum; ++i) {
                x.xEdge[v][t][i].resize(input.nodeNum);
                for (ID j = 0; j < input.nodeNum; ++j) {
                    if (i == j) { continue; }
                    x.xEdge[v][t][i][j] = mpSolver.makeVar(MpSolver::VariableType::Bool, 0, 1);
                }
            }
        }
    }
}

void IrpModelSolver::addNodeDegreeConstraints() {
    for (ID t = 0; t < input.periodNum; ++t) {
        for (ID v = 0; v < input.vehicleNum; ++v) {
            for (ID i = 0; i < input.nodeNum; ++i) {
                MpSolver::LinearExpr inEdge = 0;
                MpSolver::LinearExpr outEdge = 0;
                for (ID j = 0; j < input.nodeNum; ++j) {
                    if (i == j) { continue; }
                    inEdge += x.xEdge[v][t][j][i];
                    outEdge += x.xEdge[v][t][i][j];
                }
                mpSolver.makeConstraint(inEdge == outEdge);
                mpSolver.makeConstraint(inEdge == 1);
            }
        }
    }
}

void IrpModelSolver::setRoutingCostObjective() {
    mpSolver.setObjective(routingCostInPeriod(0, input.periodNum), MpSolver::OptimaOrientation::Minimize);
}

IrpModelSolver::MpSolver::LinearExpr IrpModelSolver::totalCostInPeriod(int start, int size, bool addInitial) {
    return (routingCostInPeriod(start, size) + holdingCostInPeriod(start, size, addInitial));
}

IrpModelSolver::MpSolver::LinearExpr IrpModelSolver::routingCostInPeriod(int start, int size) {
    MpSolver::LinearExpr totalCost = 0;

    for (ID t = start; (t < start + size) && (t < input.periodNum); ++t) {
        for (ID v = 0; v < input.vehicleNum; ++v) {
            for (ID i = 0; i < input.nodeNum; ++i) {
                if (aux.skipNode[t][i]) { continue; }
                for (ID j = 0; j < input.nodeNum; ++j) {
                    if ((i == j) || aux.skipNode[t][j]) { continue; }
                    if (cfg.usePresetSolution && presetX.isPeriodFixed[t]) {
                        totalCost += routingCost[i][j] * (presetX.xEdge[v][t][i][j] ? 1 : 0);
                    } else {
                        totalCost += routingCost[i][j] * x.xEdge[v][t][i][j];
                    }
                }
            }
        }
    }
    return totalCost;
}

IrpModelSolver::MpSolver::LinearExpr IrpModelSolver::holdingCostInPeriod(int start, int size, bool addInitial) {
    MpSolver::LinearExpr totalCost = 0;

    // cost for supplier.
    MpSolver::LinearExpr restQuantity = input.nodes[0].initialQuantity;
    if (addInitial && (start == 0)) {
        // add holding cost of initial quantity.
        totalCost += input.nodes[0].holdingCost * restQuantity;
    }
    for (ID t = 0; (t < start + size) && (t < input.periodNum); ++t) {
        for (ID v = 0; v < input.vehicleNum; ++v) {
            if (cfg.usePresetSolution && presetX.isPeriodFixed[t] && !cfg.optimizeTotalCost) {
                restQuantity -= presetX.xQuantity[v][t][0];
            } else {
                restQuantity -= x.xQuantity[v][t][0];
            }
        }
        restQuantity += input.nodes[0].unitDemand;
        if (t < start) { continue; }
        totalCost += input.nodes[0].holdingCost * restQuantity;
    }
    
    // cost for customers.
    for (ID i = 1; i < input.nodeNum; ++i) {
        restQuantity = input.nodes[i].initialQuantity;
        if (addInitial && (start == 0)) {
            // add holding cost of initial quantity.
            totalCost += input.nodes[i].holdingCost * restQuantity;
        }
        for (ID t = 0; (t < start + size) && (t < input.periodNum); ++t) {
             //check skipped nodes
            if (!aux.skipNode[t][i]) {
                for (ID v = 0; v < input.vehicleNum; ++v) {
                    if (cfg.usePresetSolution && presetX.isPeriodFixed[t] && !cfg.optimizeTotalCost) {
                        restQuantity += presetX.xQuantity[v][t][i];
                    } else {
                        restQuantity += x.xQuantity[v][t][i];
                    }
                }
            }
            restQuantity -= input.nodes[i].unitDemand;
            if (t < start) { continue; }
            totalCost += input.nodes[i].holdingCost * restQuantity;
        }
    }
    return totalCost;
}

// TODO[qym][5]: to delete 
//IrpModelSolver::MpSolver::LinearExpr IrpModelSolver::increasedHoldingCost(ID t) {
//    MpSolver::LinearExpr totalCost = 0;
//
//    if (!cfg.usePresetSolution || !presetX.isPeriodFixed[t]) { return 0; }
//    for (ID v = 0; v < input.vehicleNum; ++v) {
//        totalCost -= input.nodes[0].holdingCost * (x.xQuantity[v][t][0] - presetX.xQuantity[v][t][0]);
//    }
//    for (ID i = 1; i < input.nodeNum; ++i) {
//        if (aux.skipNode[t][i]) { continue; }
//        for (ID v = 0; v < input.vehicleNum; ++v) {
//            totalCost += input.nodes[i].holdingCost * (x.xQuantity[v][t][i] - presetX.xQuantity[v][t][i]);
//        }
//    }
//
//    return totalCost;
//}

IrpModelSolver::MpSolver::LinearExpr IrpModelSolver::restQuantityUntilPeriod(ID node, int size) {
    MpSolver::LinearExpr totalQuantity = 0;
    for (ID t = 0; t < size; ++t) {
        for (ID v = 0; v < input.vehicleNum; ++v) {
            totalQuantity += x.xQuantity[v][t][node];
        }
        totalQuantity -= input.nodes[node].unitDemand;
    }
    totalQuantity = ((node > 0) ? totalQuantity : -totalQuantity);
    return (input.nodes[node].initialQuantity + totalQuantity);
}

IrpModelSolver::MpSolver::LinearExpr IrpModelSolver::totalShortageQuantity() {
    MpSolver::LinearExpr shortage = 0;
    for (ID i = 1; i < input.nodeNum; ++i) {
        shortage += x.xShortage[i];
    }
    return shortage;
}

double IrpModelSolver::getHoldingCost(const List2D<List<MpSolver::DecisionVar>> &quantity, std::function<double(MpSolver::DecisionVar)> getValue) {
    double totalCost = 0;
    List2D<int> rq(input.nodeNum, List<int>(input.periodNum + 1, 0));
    // cost for supplier.
    int restQuantity = input.nodes[0].initialQuantity;
    totalCost += input.nodes[0].holdingCost * restQuantity;
    rq[0][0] = input.nodes[0].initialQuantity;
    for (ID t = 0; t < input.periodNum; ++t) {
        for (ID v = 0; v < input.vehicleNum; ++v) {
            restQuantity -= lround(getValue(quantity[v][t][0]));
        }
        restQuantity += input.nodes[0].unitDemand;
        totalCost += input.nodes[0].holdingCost * restQuantity;
        rq[0][t + 1] = restQuantity;
    }

    // cost for customers.
    for (ID i = 1; i < input.nodeNum; ++i) {
        restQuantity = input.nodes[i].initialQuantity;
        totalCost += input.nodes[i].holdingCost * restQuantity;
        rq[i][0] = input.nodes[i].initialQuantity;
        for (ID t = 0; t < input.periodNum; ++t) {
            for (ID v = 0; v < input.vehicleNum; ++v) {
                restQuantity += lround(getValue(quantity[v][t][i]));
            }
            restQuantity -= input.nodes[i].unitDemand;
            totalCost += input.nodes[i].holdingCost * restQuantity;
            rq[i][t + 1] = restQuantity;
        }
    }

    return totalCost;
}

double IrpModelSolver::solveVrpWithLkh(List2D<lkh::Tour>& tours, std::function<bool(ID, ID, ID)> isVisited) {
    // add tsp cache
    //const String TspCacheDir("TspCache/");
    //System::makeSureDirExist(TspCacheDir);
    //CachedTspSolver tspSolver(nodeNum, TspCacheDir + env.friendlyInstName() + ".csv");


    double routingObj = 0;
    tours.resize(input.vehicleNum);
    for (ID v = 0; v < input.vehicleNum; ++v) {
        tours[v].resize(input.periodNum);
        for (ID t = 0; t < input.periodNum; ++t) {
            List<ID> visitedNodes;
            for (ID i = 0; i < input.nodeNum; ++i) {
                if (isVisited(v, t, i)) {
                    visitedNodes.push_back(i);
                }
            }
            if (visitedNodes.size() < 2) { continue; }

            lkh::Tour tour;
            if (visitedNodes.size() >= 3) {
                lkh::CoordList2D coordList(visitedNodes.size()); 
                for (int i = 0; i < visitedNodes.size(); ++i) {
                    coordList[i].x = input.nodes[visitedNodes[i]].xCoord;
                    coordList[i].y = input.nodes[visitedNodes[i]].yCoord;
                }
                if (lkh::solveTsp(tour, coordList)) {
                    //cout << "Failed to optimize period " << t << endl;
                    //return -1;
                }
                routingObj += tour.distance;

                // adjust node id on tour.
                tours[v][t].distance = tour.distance;
                for (int i = 0; i < tour.nodes.size(); ++i) {
                    tours[v][t].nodes.push_back(visitedNodes[tour.nodes[i]]);
                }
            } else {
                // 2 nodes.
                tours[v][t].nodes = visitedNodes;
                tours[v][t].distance = routingCost[visitedNodes[0]][visitedNodes[1]]+ routingCost[visitedNodes[1]][visitedNodes[0]];
            }
            //cout << "Period " << t << "\t" << tours[v][t].nodes.size() << "\t" << tours[v][t].distance << endl;
        }
    }
    return routingObj;
}

bool IrpModelSolver::check() {
    // check feasibility
    // check subtour
    for (ID v = 0; v < input.vehicleNum; ++v) {
        for (ID t = 0; t < input.periodNum; ++t) {
            if (cfg.usePresetSolution && presetX.isPeriodFixed[t]) { continue; }
            List<bool> visited(input.nodeNum, false);
            visited[0] = true;
            ID n = 0;
            do {
                for (ID i = 0; i < input.nodeNum; ++i) {
                    if (n == i) { continue; }
                    if (presetX.xEdge[v][t][n][i]) {
                        visited[i] = true;
                        n = i;
                        break;
                    }
                }
            } while (n != 0);
            for (ID s = 1; s < input.nodeNum; ++s) {
                if (visited[s]) { continue; }
                int len = 0;
                n = s;
                do {
                    for (ID i = 0; i < input.nodeNum; ++i) {
                        if (n == i) { continue; }
                        if (presetX.xEdge[v][t][n][i]) {
                            ++len;
                            visited[i] = true;
                            n = i;
                            break;
                        }
                    }
                } while (n != s);
                if (len > 0) {
                    cout << "Infeasible: subtour." << endl;
                    return false;
                }
            }
        }
    }

    // check quantity reasonability
    for (ID t = 0; t < input.periodNum; ++t) {
        if (cfg.usePresetSolution && presetX.isPeriodFixed[t] && !cfg.optimizeTotalCost) { continue; }
        for (ID v = 0; v < input.vehicleNum; ++v) {
            for (ID i = 0; i < input.nodeNum; ++i) {
                int visitedTimes = 0;
                for (ID j = 0; j < input.nodeNum; ++j) {
                    if (i == j) { continue; }
                    visitedTimes += presetX.xEdge[v][t][i][j];
                }
                if (visitedTimes > 1) { cout << "Infeasible: multiple visit." << endl; return false; }
                if (lround(presetX.xQuantity[v][t][i]) > input.nodes[i].capacity * visitedTimes) {
                    cout << "Infeasible: unreasonable quantity getQuantity(" << v << "," << t << "," << i << ")=" << presetX.xQuantity[v][t][i] << " when visit " << visitedTimes << " times." << endl;
                    return false;
                }
            }
        }
    }

    // check load-deliver equality.
    for (ID t = 0; t < input.periodNum; ++t) {
        for (ID v = 0; v < input.vehicleNum; ++v) {
            double q = presetX.xQuantity[v][t][0];
            for (ID i = 1; i < input.nodeNum; ++i) {
                q -= presetX.xQuantity[v][t][i];
            }
            if (abs(q) > DefaultDoubleGap) {
                cout << "Infeasible: loaded quantity is not equal to delivered quantity (diff=" << q << ")." << endl;
                return false;
            }
        }
    }

    // check rest quantity
    double totalHoldingCost = 0;
    for (ID i = 0; i < input.nodeNum; ++i) {
        double restQuantity = input.nodes[i].initialQuantity;
        totalHoldingCost += input.nodes[i].holdingCost * restQuantity;
        for (ID t = 0; t < input.periodNum; ++t) {
            for (ID v = 0; v < input.vehicleNum; ++v) {
                restQuantity += ((i > 0) ? presetX.xQuantity[v][t][i] : -presetX.xQuantity[v][t][i]);
            }
            if (lround(restQuantity) > input.nodes[i].capacity) {
                cout << "Infeasible: excessive rest quantity at node " << i << "(" << restQuantity << "/" << input.nodes[i].capacity << ") in period " << t << endl;
                return false;
            }
            restQuantity -= ((i > 0) ? input.nodes[i].unitDemand : -input.nodes[i].unitDemand);
            if (lround(restQuantity) < input.nodes[i].minLevel) {
                cout << "Infeasible: inadequate rest quantity at node " << i << "(" << restQuantity << "/" << input.nodes[i].minLevel << ") in period " << t << endl;
                return false;
            }
            totalHoldingCost += input.nodes[i].holdingCost * restQuantity;
        }
    }

    double totalRoutingCost = 0;
    for (ID t = 0; t < input.periodNum; ++t) {
        for (ID v = 0; v < input.vehicleNum; ++v) {
            for (ID i = 0; i < input.nodeNum; ++i) {
                for (ID j = 0; j < input.nodeNum; ++j) {
                    if (i == j) { continue; }
                    totalRoutingCost += (presetX.xEdge[v][t][i][j] ? routingCost[i][j] : 0);
                }
            }
        }
    }
    double totalCost = totalRoutingCost + totalHoldingCost;
    cout << "\nOBJ (R+H) = " << totalCost << " = " << totalRoutingCost << " (" << totalRoutingCost / totalCost << ") + " << totalHoldingCost << " (" << totalHoldingCost / totalCost << ")\n\n" << endl;

    // check obj match.
    List<bool> backup(presetX.isPeriodFixed);
    fill(presetX.isPeriodFixed.begin(), presetX.isPeriodFixed.end(), true);
    if (!cfg.allowShortage && (totalCost - getTotalCostInPeriod(0, input.periodNum) > DefaultDoubleGap)) {
        cout << "Worse actual objective = " << totalCost << ">" << getTotalCostInPeriod(0, input.periodNum) << endl;
        presetX.isPeriodFixed = backup;
        return false;
    } else if (!cfg.allowShortage && (getTotalCostInPeriod(0, input.periodNum) - totalCost > DefaultDoubleGap)) {
        cout << "Better actual objective = " << totalCost << "<" << getTotalCostInPeriod(0, input.periodNum) << endl;
    }
    presetX.isPeriodFixed = backup;

    return true;
}

bool IrpModelSolver::eliminateSubtour(
    function<bool(const MpSolver::DecisionVar&)> isTrue,
    function<void(const MpSolver::LinearRange&)> addLazy) {
    bool subtourFound = false;
    for (ID v = 0; v < input.vehicleNum; ++v) {
        for (ID t = 0; t < input.periodNum; ++t) {
            if (cfg.usePresetSolution && presetX.isPeriodFixed[t]) { continue; }
            List<bool> visited(input.nodeNum, false);
            visited[0] = true;
            ID n = 0;
            do {
                for (ID i = 0; i < input.nodeNum; ++i) {
                    if (n == i) { continue; }
                    if (aux.skipNode[t][i]) { continue; }
                    if (isTrue(x.xEdge[v][t][n][i])) {
                        visited[i] = true;
                        n = i;
                        break;
                    }
                }
            } while (n != 0);
            for (ID s = 1; s < input.nodeNum; ++s) {
                if (visited[s]) { continue; }
                if (aux.skipNode[t][s]) { continue; }
                MpSolver::LinearExpr expr = 0;
                List<ID> path;
                int len = 0;
                n = s;
                path.push_back(n);
                do {
                    for (ID i = 0; i < input.nodeNum; ++i) {
                        if (n == i) { continue; }
                        if (aux.skipNode[t][i]) { continue; }
                        if (isTrue(x.xEdge[v][t][n][i])) {
                            expr += x.xEdge[v][t][n][i];
                            ++len;
                            visited[i] = true;
                            n = i;
                            path.push_back(n);
                            break;
                        }
                    }
                } while (n != s);
                if (len > 0) {
                    addLazy(expr <= len - 1);
                    subtourFound = true;
                    if (!cfg.forbidAllSubtours) { break; }
                }
            }
        }
    }
    return subtourFound;
}

void IrpModelSolver::SolutionFound::callback() {
    try {
        if (where == GRB_CB_MIPSOL) {
            bool subtourFound = false;
            if (solver.cfg.useLazyConstraints && !solver.cfg.allowSubtour) {
                subtourFound = solver.eliminateSubtour(
                    [&](const MpSolver::DecisionVar &var) { return (getSolution(var) > (1 - IrpModelSolver::DefaultDoubleGap)); },
                    [&](const GRBTempConstr& tc) { addLazy(tc); });

            }
            if (subtourFound) { return; }

            // terminate optimization if best objective reached.
            if (solver.cfg.useBenchmark && abs(getDoubleInfo(GRB_CB_MIPSOL_OBJ) - solver.input.bestObjective) <= DefaultDoubleGap) {
                solver.elapsedSeconds = getDoubleInfo(GRB_CB_RUNTIME);
                abort();
            }
            // update optimal and duration.
            if (solver.cfg.useBenchmark && (getDoubleInfo(GRB_CB_MIPSOL_OBJ) < solver.currentObjective.totalCost)) {
                solver.currentObjective.totalCost = getDoubleInfo(GRB_CB_MIPSOL_OBJ);
                solver.elapsedSeconds = getDoubleInfo(GRB_CB_RUNTIME);
            }
            //if (solver.cfg.usePresetSolution
            //    && (getDoubleInfo(GRB_CB_MIP_OBJBST) - getDoubleInfo(GRB_CB_MIP_OBJBND) / getDoubleInfo(GRB_CB_MIP_OBJBND) < 0.05)) {
            //    abort();
            //}
        }
    } catch (GRBException e) {
        cout << "Error number: " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch (...) {
        cout << "Error during callback" << endl;
    }
}

void IrpModelSolver::SolutionFound::printSolutionInfo() {
    const IrpModelSolver::Input &input(solver.input);
    for (ID v = 0; v < solver.input.vehicleNum; ++v) {
        for (ID t = 0; t < input.periodNum; ++t) {
            for (ID i = 0; i < input.nodeNum; ++i) {
                if (solver.aux.skipNode[t][i]) { continue; }
                for (ID j = 0; j < input.nodeNum; ++j) {
                    if (i == j) { cout << "** "; continue; }
                    if (solver.aux.skipNode[t][i]) { continue; }
                    cout << getSolution(solver.x.xEdge[v][t][i][j]) << " ";
                }
                cout << endl;
            }
            cout << endl;
        }
    }
}

void IrpModelSolver::TspSolutionFound::callback() {
    try {
        if (where == GRB_CB_MIPSOL) {
            solver.eliminateSubtour(
                [&](const MpSolver::DecisionVar &var) { return (getSolution(var) > (1 - IrpModelSolver::DefaultDoubleGap)); },
                [&](const GRBTempConstr& tc) { addLazy(tc); });
        }
    } catch (GRBException e) {
        cout << "Error number: " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch (...) {
        cout << "Error during tsp callback" << endl;
    }
}

void IrpModelSolver::InfeasibleFound::callback() {
    try {
        if (where == GRB_CB_MIPSOL) {
            if (getDoubleInfo(GRB_CB_MIPSOL_OBJ) > min(solver.currentObjective.totalCost , input.referenceObjective)) { return; }
            // solve tsp with lkh.
            List2D<lkh::Tour> tours;
            double holdingCost = getHoldingCost();
            double routingCost = 
                solver.solveVrpWithLkh(tours, [&](ID v, ID t, ID i) { return (getSolution(solver.x.xQuantity[v][t][i]) > DefaultDoubleGap); });
            if (routingCost < 0) {
                // failed to solve tsp with lkh
            }

            // forbid current solution with lazy constraints
            int strategy = 2;
            if (strategy == 1) {
                // make the visited nodes set different at a randomly selected period.
                ID v = rand() % input.vehicleNum;
                ID t = rand() % input.periodNum;
                MpSolver::LinearExpr diffCount = 0;
                for (ID i = 1; i < input.nodeNum; ++i) {
                    bool visited = false;
                    MpSolver::LinearExpr degree = 0;
                    for (ID j = 0; j < input.nodeNum; ++j) {
                        if (j == i) { continue; }
                        if (isTrue(solver.x.xEdge[v][t][i][j])) { visited = true; }
                        degree += solver.x.xEdge[v][t][i][j];
                    }
                    if (visited) {
                        // visited node.
                        diffCount += 1 - degree;
                    } else {
                        // unvisited node.
                        diffCount += degree;
                    }
                }
                addLazy(diffCount >= 1);
            }

            if (strategy == 2) {
                ostringstream oss;
                // gurantee that visited node set different from current solution.
                MpSolver::LinearExpr diffCount = 0;
                for (ID v = 0; v < input.vehicleNum; ++v) {
                    for (ID t = 0; t < input.periodNum; ++t) {
                        oss << "Period " << t << endl;
                        for (ID i = 0; i < input.nodeNum; ++i) {
                            bool visited = false;
                            MpSolver::LinearExpr degree = 0;
                            for (ID j = 0; j < input.nodeNum; ++j) {
                                if (j == i) { continue; }
                                if (isTrue(solver.x.xEdge[v][t][i][j])) { visited = true; }
                                degree += solver.x.xEdge[v][t][i][j];
                            }
                            if (visited) {
                                // visited node.
                                diffCount += 1 - degree;
                            } else {
                                // unvisited node.
                                diffCount += degree;
                            }
                            oss << visited << "," << getSolution(solver.x.xQuantity[v][t][i]) << endl;
                        }
                        oss << endl;
                    }
                }
                addLazy(diffCount >= 1);

                //ofstream ofs("ttttext.csv", ios::app);
                //ofs << oss.str() << (holdingCost + routingCost) << endl;
                //ofs.close();
            }

            // ½ûµôÀë²Ö¿âÌ«Ô¶µÄ×Ó»ØÂ·
            if (strategy == 3) {
                ID selectedPeriod = -1;
                double maxIncrease = 0;
                for (ID v = 0; v < input.vehicleNum; ++v) {
                    for (ID t = 0; t < input.periodNum; ++t) {
                        double delta = tours[v][t].distance;
                        for (ID i = 0; i < input.nodeNum; ++i) {
                            for (ID j = 0; j < input.nodeNum; ++j) {
                                if (i == j) { continue; }
                                if (!isTrue(solver.x.xEdge[v][t][i][j])) { continue; }
                                delta -= solver.routingCost[i][j];
                            }
                        }
                        cout << t << "\t" << delta << "\n";
                        if (delta > maxIncrease) {
                            maxIncrease = delta;
                            selectedPeriod = t;
                        }
                    }
                }
                if (selectedPeriod < 0) {
                    return;
                }
                List<ID> nodes;
                for (ID i = 0; i < input.nodeNum; ++i) {
                    if (getSolution(solver.x.xQuantity[0][selectedPeriod][i]) > DefaultDoubleGap) {
                        nodes.push_back(i);
                    }
                }
                for (ID v = 0; v < input.vehicleNum; ++v) {
                    for (ID t = 0; t < input.periodNum; ++t) {
                        MpSolver::LinearExpr expr = 0;
                        for (auto i = nodes.begin(); i != nodes.end(); ++i) {
                            for (ID j = 0; j < input.nodeNum; ++j) {
                                if (*i == j) { continue; }
                                expr -= solver.x.xEdge[v][t][*i][j];
                            }
                        }
                        addLazy(expr <= nodes.size() - 1);
                    }
                }
                // find subtours.
                //for (ID v = 0; v < input.vehicleNum; ++v) {
                //    for (ID t = 0; t < input.periodNum; ++t) {
                //        List<bool> visited(input.nodeNum, false);
                //        List<List<ID>> subTours;
                //        for (ID i = 0; i < input.nodeNum; ++i) {
                //            if (visited[i]) { continue; }
                //            ID p = i;
                //            List<ID> subTour;
                //            do {
                //                for (ID j = 0; j < input.nodeNum; ++j) {
                //                    if (p == j) { continue; }
                //                    if (isTrue(solver.x.xEdge[v][t][p][j])) {
                //                        subTour.push_back(p);
                //                        visited[p] = true;
                //                        p = j;
                //                        break;
                //                    }
                //                }
                //            } while (p != i);
                //            if (subTour.size() > 0) { subTour.push_back(p); subTours.push_back(subTour); }
                //        }
                //        for (auto s = subTours.begin(); s != subTours.end(); ++s) {
                //            for (auto n = s->begin(); n != s->end(); ++n) {
                //                cout << *n << "\t";
                //            }
                //            cout << endl;
                //        }
                //        cout << endl;
                //    }
                //}
            }
            
            // update optimal.
            if (holdingCost + routingCost < solver.currentObjective.totalCost) {
                cout << "* ";
                solver.currentObjective.holdingCost = holdingCost;
                solver.currentObjective.routingCost = routingCost;
                solver.currentObjective.totalCost = holdingCost + routingCost;
                retrieveDeliveryQuantity();
                solver.convertTourToEdges(solver.presetX, tours);
            }
            cout << (holdingCost + routingCost) << "(T) = " << holdingCost << "(H) + " << routingCost << "(R)" << endl;
        }
    } catch (GRBException e) {
        cout << "Error number: " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch (...) {
        cout << "Error during callback" << endl;
    }
}

double IrpModelSolver::InfeasibleFound::getHoldingCost() {
    return solver.getHoldingCost(solver.x.xQuantity, [&](const MpSolver::DecisionVar &var) { return getSolution(var); });
}

void IrpModelSolver::InfeasibleFound::retrieveDeliveryQuantity() {
    solver.retrieveDeliveryQuantity(solver.presetX, [&](ID v, ID t, ID i) { return (getSolution(solver.x.xQuantity[v][t][i])); });
}

}
