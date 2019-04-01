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
    if (!cfg.isEliminationLazy() && !cfg.isSubtoursAllowed()) {
        addSubtourEliminationConstraint();
    }
    SolutionFound cb(*this);
    if (cfg.useBenchmark || cfg.isEliminationLazy()) { mpSolver.setCallback(&cb); }

    if (cfg.isPresetEnabled()) { setInitSolution(); }
    if (cfg.isShortageRelaxed()) {
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
        elapsedSeconds = (cfg.isPresetEnabled() || !cfg.useBenchmark) ? mpSolver.getDurationInSecond() : elapsedSeconds;
    }
    retrieveSolution();

    return true;
    //return check();
}

bool IrpModelSolver::solveIteratively() {
    bool lazy = true;

    initSolver();

    relaxTspSubtourConstraint();
    cfg.subtourPolicy = Configuration::SubtourPolicy::EliminateAllSubtours;
    currentObjective = { INFINITY, INFINITY, INFINITY };
    cfg.useBenchmark = true;

    addDecisionVars();
    addPathConnectivityConstraint();
    addDeliveryQuantityConstraint();
    addLoadDeliveryBalanceConstraint();
    addCustomerLevelConstraint();
    addSupplierLevelConstraint();
    //setTotalCostObjective();
    mpSolver.setObjective(estimatedRoutingCost() + holdingCostInPeriod(0, input.periodNum), OptimaOrientation::Minimize);

    // use lazy constraint
    if (lazy) {
        RelaxedSolutionFound cb(*this);
        mpSolver.setCallback(&cb);
        //mpSolver.makeConstraint(x.xQuantity[0][0][0] <= 0);
        //mpSolver.makeConstraint(x.xQuantity[0][5][0] <= 0);
        if (!mpSolver.optimize()) {
            elapsedSeconds = mpSolver.getDurationInSecond();
            return (currentObjective.totalCost < INFINITY);
        }
    } else { // model
        elapsedSeconds = 0;
        while (elapsedSeconds < DefaultTimeLimitSecond) {
            if (!mpSolver.optimize()) {
                elapsedSeconds = mpSolver.getDurationInSecond();
                return (currentObjective.totalCost < INFINITY);
            }
            elapsedSeconds += mpSolver.getDurationInSecond();
            // solve tsp with lkh.
            List2D<CachedTspSolver::Tour> tours;
            double holdingCost = getHoldingCostInPeriod(0, input.periodNum);
            double routingCost =
                solveVrpWithLkh(tours, [&](ID v, ID t, ID i) { return (mpSolver.getValue(x.xQuantity[v][t][i]) > DefaultDoubleGap); });
            if (routingCost < 0) { continue; }

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
            LinearExpr nodeDiff = 0;
            for (ID v = 0; v < input.vehicleNum; ++v) {
                for (ID t = 0; t < input.periodNum; ++t) {
                    for (ID i = 1; i < input.nodeNum; ++i) {
                        LinearExpr degree = 0;
                        for (ID j = 0; j < input.nodeNum; ++j) {
                            if (j == i) { continue; }
                            degree += x.xEdge[v][t][i][j];
                        }
                        if (mpSolver.getValue(x.xQuantity[v][t][i]) < DefaultDoubleGap) {
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
            if (cfg.isPresetEnabled() && aux.isPeriodFixed[t] && !cfg.withGlobalOptimization()) { continue; }
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

    TspSolutionFound cb(*this);
    mpSolver.setCallback(&cb);
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
            if (cfg.isPresetEnabled() && aux.isPeriodFixed[t]) { continue; }
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
            if (cfg.isPresetEnabled() && aux.isPeriodFixed[t] && !cfg.withGlobalOptimization()) { continue; }
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

    if (cfg.isSubtoursAllowed() || cfg.isEliminationLazy()) { return; }
    presetX.xSequence.resize(input.vehicleNum);
    for (ID v = 0; v < input.vehicleNum; ++v) {
        presetX.xSequence[v].resize(input.periodNum);
        for (ID t = 0; t < input.periodNum; ++t) {
            if (cfg.isPresetEnabled() && aux.isPeriodFixed[t]) { continue; }
            presetX.xSequence[v][t].resize(input.nodeNum);
            fill(presetX.xSequence[v][t].begin(), presetX.xSequence[v][t].end(), 0);
            for (ID i = 0; i < input.nodeNum; ++i) {
                if (aux.skipNode[t][i]) { continue; }
                presetX.xSequence[v][t][i] = mpSolver.getValue(x.xSequence[v][t][i]);
            }
        }
    }
}

void IrpModelSolver::retrieveDeliveryQuantity(PresetX &presetX, GetValueById<double> getQuantity) {
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

void IrpModelSolver::saveSolution(const Input &input, const PresetX &presetX, const String &path) {
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
    //x.xSubtour = mpSolver.makeVar(VariableType::Real, 0, Infinity);

    x.xEdge.resize(input.vehicleNum);
    for (ID v = 0; v < input.vehicleNum; ++v) {
        x.xEdge[v].resize(input.periodNum);
        for (ID t = 0; t < input.periodNum; ++t) {
            if (cfg.isPresetEnabled() && aux.isPeriodFixed[t]) { continue; }
            x.xEdge[v][t].resize(input.nodeNum);
            for (ID i = 0; i < input.nodeNum; ++i) {
                if (aux.skipNode[t][i]) { continue; }
                x.xEdge[v][t][i].resize(input.nodeNum);
                for (ID j = 0; j < input.nodeNum; ++j) {
                    if (aux.skipNode[t][j]) { continue; }
                    if (i == j) { continue; }
                    ostringstream oss;
                    oss << "x_" << v << "_" << t << "_" << i << "_" << j;
                    x.xEdge[v][t][i][j] = mpSolver.makeVar(VariableType::Bool, 0, 1, oss.str());
                }
            }
        }
    }

    x.xQuantity.resize(input.vehicleNum);
    for (ID v = 0; v < input.vehicleNum; ++v) {
        x.xQuantity[v].resize(input.periodNum);
        for (ID t = 0; t < input.periodNum; ++t) {
            if (cfg.isPresetEnabled() && aux.isPeriodFixed[t] && !cfg.withGlobalOptimization()) { continue; }
            x.xQuantity[v][t].resize(input.nodeNum);
            for (ID i = 0; i < input.nodeNum; ++i) {
                if (aux.skipNode[t][i]) { continue; }
                ostringstream oss;
                oss << "q_" << v << "_" << t << "_" << i;
                x.xQuantity[v][t][i] = mpSolver.makeVar(VariableType::Real, 0, min(input.vehicleCapacity, input.nodes[i].capacity), oss.str());
            }
        }
    }
    if (cfg.isShortageRelaxed()) {
        x.xMinLevel.resize(input.nodeNum);
        x.xShortage.resize(input.nodeNum);
        x.xShortage[0] = mpSolver.makeVar(VariableType::Bool);
        for (ID i = 1; i < input.nodeNum; ++i) {
            x.xMinLevel[i] = mpSolver.makeVar(VariableType::Real, -Infinity, input.nodes[i].capacity);
            x.xShortage[i] = mpSolver.makeVar(VariableType::Real, 0, Infinity);
        }
    }

    if (cfg.isSubtoursAllowed() || cfg.isEliminationLazy()) { return; }
    x.xSequence.resize(input.vehicleNum);
    for (ID v = 0; v < input.vehicleNum; ++v) {
        x.xSequence[v].resize(input.periodNum);
        for (ID t = 0; t < input.periodNum; ++t) {
            if (cfg.isPresetEnabled() && aux.isPeriodFixed[t]) { continue; }
            x.xSequence[v][t].resize(input.nodeNum);
            for (ID i = 0; i < input.nodeNum; ++i) {
                if (aux.skipNode[t][i]) { continue; }
                ostringstream oss;
                oss << "u_" << v << "_" << t << "_" << i;
                x.xSequence[v][t][i] = mpSolver.makeVar(VariableType::Integer, 1, input.nodeNum, oss.str());
            }
        }
    }
}

void IrpModelSolver::addPathConnectivityConstraint() {
    for (ID t = 0; t < input.periodNum; ++t) {
        if (cfg.isPresetEnabled() && aux.isPeriodFixed[t]) { continue; }
        for (ID i = 0; i < input.nodeNum; ++i) {
            if (aux.skipNode[t][i]) { continue; }
            LinearExpr visitedTime = 0;
            for (ID v = 0; v < input.vehicleNum; ++v) {
                LinearExpr inEdge = 0;
                LinearExpr outEdge = 0;
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
        if (cfg.isPresetEnabled() && aux.isPeriodFixed[t] && !cfg.withGlobalOptimization()) { continue; }
        // for supplier
        for (ID v = 0; v < input.vehicleNum; ++v) {
            LinearExpr visitedTime = 0;
            for (ID j = 1; j < input.nodeNum; ++j) {
                if (aux.skipNode[t][j]) { continue; }
                if (cfg.isPresetEnabled() && aux.isPeriodFixed[t] && cfg.withGlobalOptimization()) {
                    if (presetX.xEdge[v][t][0][j]) { visitedTime += 1; break; }
                } else {
                    visitedTime += x.xEdge[v][t][0][j];
                }
            }
            mpSolver.makeConstraint(x.xQuantity[v][t][0] <= input.vehicleCapacity * visitedTime, "qv0");
            mpSolver.makeConstraint(x.xQuantity[v][t][0] >= visitedTime, "qv0-1");
        }

        // for customers
        for (ID i = 1; i < input.nodeNum; ++i) {
            if (aux.skipNode[t][i]) { continue; }
            LinearExpr totalQuantity = 0;
            LinearExpr visitedTime = 0;
            for (ID v = 0; v < input.vehicleNum; ++v) {
                totalQuantity += x.xQuantity[v][t][i];
                for (ID j = 0; j < input.nodeNum; ++j) {
                    if (i == j) { continue; }
                    if (aux.skipNode[t][j]) { continue; }
                    if (cfg.isPresetEnabled() && aux.isPeriodFixed[t] && cfg.withGlobalOptimization()) {
                        if (presetX.xEdge[v][t][i][j]) { visitedTime += 1; break; }
                    } else {
                        visitedTime += x.xEdge[v][t][i][j];
                    }
                }
                mpSolver.makeConstraint(x.xQuantity[v][t][i] >= visitedTime);
            }
            mpSolver.makeConstraint(totalQuantity <= input.nodes[i].capacity * visitedTime, "qv");
        }
    }

    //for (ID v = 0; v < input.vehicleNum; ++v) {
    //    for (ID t = 0; t < input.periodNum; ++t) {
    //        for (ID i = 1; i < input.nodeNum; ++i) {
    //            if (aux.skipNode[t][i]) { continue; }
    //            LinearExpr visitedTime = 0;
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
            if (cfg.isPresetEnabled() && aux.isPeriodFixed[t] && !cfg.withGlobalOptimization()) { continue; }
            LinearExpr totalQuantity = 0;
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
        LinearExpr restQuantity = 0;
        restQuantity += input.nodes[i].initialQuantity;
        for (ID t = 0; t < input.periodNum; ++t) {
            if (!aux.skipNode[t][i]) {
                for (ID v = 0; v < input.vehicleNum; ++v) {
                    if (cfg.isPresetEnabled() && aux.isPeriodFixed[t] && !cfg.withGlobalOptimization()) {
                        restQuantity += presetX.xQuantity[v][t][i];
                    } else {
                        restQuantity += x.xQuantity[v][t][i];
                    }
                }
            }
            mpSolver.makeConstraint(restQuantity <= input.nodes[i].capacity, str("mc", i, t));
            restQuantity -= input.nodes[i].unitDemand;
            mpSolver.makeConstraint(restQuantity >= (cfg.isShortageRelaxed() ? x.xMinLevel[i] : LinearExpr(input.nodes[i].minLevel)), "ml");
        }
        if (cfg.isShortageRelaxed()) {
            mpSolver.makeConstraint(x.xShortage[i] >= -x.xMinLevel[i], "sm");
        }
    }
}

void IrpModelSolver::addSupplierLevelConstraint() {
    LinearExpr restQuantity = 0;
    restQuantity += input.nodes[0].initialQuantity;
    for (ID t = 0; t < input.periodNum; ++t) {
        for (ID v = 0; v < input.vehicleNum; ++v) {
            if (cfg.isPresetEnabled() && aux.isPeriodFixed[t] && !cfg.withGlobalOptimization()) {
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
    if (!cfg.isPresetEnabled() || presetX.xEdge.empty()) { return; }

    for (ID v = 0; v < input.vehicleNum; ++v) {
        for (ID t = 0; t < input.periodNum; ++t) {
            if (aux.isPeriodFixed[t]) { continue; }
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
            if (aux.isPeriodFixed[t] && !cfg.withGlobalOptimization()) { continue; }
            for (ID i = 0; i < input.nodeNum; ++i) {
                if (aux.skipNode[t][i]) { continue; }
                mpSolver.setInitValue(x.xQuantity[v][t][i], presetX.xQuantity[v][t][i]);
            }
        }
    }

    if (cfg.isSubtoursAllowed() || cfg.isEliminationLazy()) { return; }
    for (ID v = 0; v < input.vehicleNum; ++v) {
        for (ID t = 0; t < input.periodNum; ++t) {
            if (aux.isPeriodFixed[t]) { continue; }
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
                    VariableType::Real, 0, min(input.vehicleCapacity, input.nodes[i].capacity));
                if (cfg.isPresetEnabled()) {
                    double value = (presetX.xVisited[v][t][i] ? 1 : 0);
                    x.xVisited[v][t][i] = mpSolver.makeVar(VariableType::Bool, value, value);
                } else {
                    x.xVisited[v][t][i] = mpSolver.makeVar(VariableType::Bool);
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
        LinearExpr restQuantity = input.nodes[i].initialQuantity;
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
    LinearExpr restQuantity = input.nodes[0].initialQuantity;
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
            LinearExpr expr = -x.xQuantity[v][t][0];
            for (ID i = 1; i < input.nodeNum; ++i) {
                expr += x.xQuantity[v][t][i];
            }
            mpSolver.makeConstraint(expr == 0);
        }
    }
}

void IrpModelSolver::setHoldingCostObjective() {
    const int ObjWeight = 0;

    LinearExpr totalCost = 0;
    LinearExpr restQuantity = input.nodes[0].initialQuantity;
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
        LinearExpr times = 0;
        for (ID v = 0; v < input.vehicleNum; ++v) {
            for (ID t = 0; t < input.periodNum; ++t) {
                times += x.xVisited[v][t][i];
            }
        }
        //totalCost += (times - leastDeliveryNum) * 0;
    }

    LinearExpr estimatedRoutingCost = 0;
    for (ID i = 1; i < input.nodeNum; ++i) {
        for (ID t = 0; t < input.periodNum; ++t) {
            for (ID v = 0; v < input.vehicleNum; ++v) {
                estimatedRoutingCost += (routingCost[0][i] + routingCost[i][0]) * x.xVisited[v][t][i];
            }
        }
    }

    //totalCost = estimatedRoutingCost;

    mpSolver.setObjective(totalCost, OptimaOrientation::Minimize);
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
                    x.xEdge[v][t][i][j] = mpSolver.makeVar(VariableType::Bool, 0, 1);
                }
            }
        }
    }
}

void IrpModelSolver::addNodeDegreeConstraints() {
    for (ID t = 0; t < input.periodNum; ++t) {
        for (ID v = 0; v < input.vehicleNum; ++v) {
            for (ID i = 0; i < input.nodeNum; ++i) {
                LinearExpr inEdge = 0;
                LinearExpr outEdge = 0;
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
    mpSolver.setObjective(routingCostInPeriod(0, input.periodNum), OptimaOrientation::Minimize);
}

IrpModelSolver::LinearExpr IrpModelSolver::totalCostInPeriod(int start, int size, bool addInitial) {
    return (routingCostInPeriod(start, size) + holdingCostInPeriod(start, size, addInitial));
}

IrpModelSolver::LinearExpr IrpModelSolver::routingCostInPeriod(int start, int size) {
    LinearExpr totalCost = 0;

    for (ID t = start; (t < start + size) && (t < input.periodNum); ++t) {
        for (ID v = 0; v < input.vehicleNum; ++v) {
            for (ID i = 0; i < input.nodeNum; ++i) {
                if (aux.skipNode[t][i]) { continue; }
                for (ID j = 0; j < input.nodeNum; ++j) {
                    if ((i == j) || aux.skipNode[t][j]) { continue; }
                    if (cfg.isPresetEnabled() && aux.isPeriodFixed[t]) {
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

IrpModelSolver::LinearExpr IrpModelSolver::holdingCostInPeriod(int start, int size, bool addInitial) {
    LinearExpr totalCost = 0;

    // cost for supplier.
    LinearExpr restQuantity = input.nodes[0].initialQuantity;
    if (addInitial && (start == 0)) {
        // add holding cost of initial quantity.
        totalCost += input.nodes[0].holdingCost * restQuantity;
    }
    for (ID t = 0; (t < start + size) && (t < input.periodNum); ++t) {
        for (ID v = 0; v < input.vehicleNum; ++v) {
            if (cfg.isPresetEnabled() && aux.isPeriodFixed[t] && !cfg.withGlobalOptimization()) {
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
                    if (cfg.isPresetEnabled() && aux.isPeriodFixed[t] && !cfg.withGlobalOptimization()) {
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

IrpModelSolver::LinearExpr IrpModelSolver::estimatedRoutingCost() {
    LinearExpr penalty = 0;
    /*
    {
        List<lkh::Tour> subtours;
        MpSolver tspSolver;
        // variables.
        List2D<DecisionVar> edge;
        edge.resize(input.nodeNum);
        for (ID i = 0; i < input.nodeNum; ++i) {
            edge[i].resize(input.nodeNum);
            for (ID j = 0; j < input.nodeNum; ++j) {
                if (i == j) { continue; }
                edge[i][j] = tspSolver.makeVar(VariableType::Bool, 0, 1);
            }
        }
        // constraints.
        for (ID i = 0; i < input.nodeNum; ++i) {
            LinearExpr inEdge = 0;
            LinearExpr outEdge = 0;
            for (ID j = 0; j < input.nodeNum; ++j) {
                if (i == j) { continue; }
                inEdge += edge[j][i];
                outEdge += edge[i][j];
            }
            tspSolver.makeConstraint(inEdge == outEdge);
            tspSolver.makeConstraint(inEdge == 1);
        }
        // objective.
        LinearExpr tc = 0;
        for (ID i = 0; i < input.nodeNum; ++i) {
            for (ID j = 0; j < input.nodeNum; ++j) {
                if (i == j) { continue; }
                tc += routingCost[i][j] * edge[i][j];
            }
        }
        tspSolver.setObjective(tc, OptimaOrientation::Minimize);

        for (int iter = 0; (iter < 3) && tspSolver.optimize(); ++iter) {
            // solution.
            List<bool> visited(input.nodeNum, false);
            for (ID i = 0; i < input.nodeNum; ++i) {
                if (visited[i]) { continue; }
                ID p = i;
                lkh::Tour subtour;
                subtour.distance = 0;
                LinearExpr expr = 0;
                do {
                    for (ID j = 0; j < input.nodeNum; ++j) {
                        if (p == j) { continue; }
                        if (tspSolver.isTrue(edge[p][j])) {
                            subtour.distance += routingCost[p][j];
                            subtour.nodes.push_back(p);
                            expr += edge[p][j];
                            visited[p] = true;
                            p = j;
                            break;
                        }
                    }
                } while (p != i);
                if (subtour.distance > 0) {
                    subtour.nodes.push_back(p);
                    subtours.push_back(subtour);
                    if (i != 0) { tspSolver.makeConstraint(expr <= expr.size() - 1); }
                }
            }

        }

        for (auto t = subtours.begin(); t != subtours.end(); ++t) {
            for (auto n = t->nodes.begin(); n != t->nodes.end(); ++n) {
                cout << *n << " ";
            }
            cout << "\t" << t->distance << endl;
        }

        for (ID v = 0; v < input.vehicleNum; ++v) {
            for (ID t = 0; t < input.periodNum; ++t) {
                for (auto s = subtours.begin(); s != subtours.end(); ++s) {
                    DecisionVar var = mpSolver.makeVar(VariableType::Real, 0, Infinity);
                    x.xSt.push_back(var);
                    LinearExpr path = 0;
                    auto pre = s->nodes.begin();
                    for (auto n = (pre + 1); n != s->nodes.end(); ++n) {
                        path += x.xEdge[v][t][*pre][*n];
                        pre = n;
                    }
                    //mpSolver.makeConstraint(var >= (800.0 / (s->distance * s->nodes.size())) * (path - path.size() + 1));
                    mpSolver.makeConstraint(var >= (20.0 / (s->nodes.size() - 1)) * (path - path.size() + 1));
                    penalty += var;
                }
            }
        }
    }
    */

    for (ID v = 0; v < input.vehicleNum; ++v) {
        LinearExpr visitedTime = 0;
        for (ID t = 0; t < input.periodNum; ++t) {
            for (ID i = 1; i < input.nodeNum; ++i) {
                visitedTime += x.xEdge[v][t][0][i];
            }
        }
        //mpSolver.makeConstraint(visitedTime <= 4);
        penalty += aux.visitParameter * visitedTime;
    }   

    return (aux.routingParameter * routingCostInPeriod(0, input.periodNum) + penalty);
}

// TODO[qym][5]: to delete 
//IrpModelSolver::LinearExpr IrpModelSolver::increasedHoldingCost(ID t) {
//    LinearExpr totalCost = 0;
//
//    if (!cfg.isPresetEnabled() || !aux.isPeriodFixed[t]) { return 0; }
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

IrpModelSolver::LinearExpr IrpModelSolver::restQuantityUntilPeriod(ID node, int size) {
    LinearExpr totalQuantity = 0;
    for (ID t = 0; t < size; ++t) {
        for (ID v = 0; v < input.vehicleNum; ++v) {
            totalQuantity += x.xQuantity[v][t][node];
        }
        totalQuantity -= input.nodes[node].unitDemand;
    }
    totalQuantity = ((node > 0) ? totalQuantity : -totalQuantity);
    return (input.nodes[node].initialQuantity + totalQuantity);
}

IrpModelSolver::LinearExpr IrpModelSolver::totalShortageQuantity() {
    LinearExpr shortage = 0;
    for (ID i = 1; i < input.nodeNum; ++i) {
        shortage += x.xShortage[i];
    }
    return shortage;
}

double IrpModelSolver::getHoldingCost(const List3D<DecisionVar> &quantity, GetVarValue<double> getValue) {
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

double IrpModelSolver::solveVrpWithLkh(List2D<CachedTspSolver::Tour>& tours, GetValueById<bool> isVisited) {
    // add tsp cache
    CachedTspSolver tspSolver(input.nodeNum, aux.tspCacheFilePath);

    double routingObj = 0;
    tours.resize(input.vehicleNum);
    for (ID v = 0; v < input.vehicleNum; ++v) {
        tours[v].resize(input.periodNum);
        for (ID t = 0; t < input.periodNum; ++t) {
            List<ID> visitedNodes;
            List<bool> containNode(input.nodeNum, false);
            CachedTspSolver::CoordList2D coords;
            for (ID i = 0; i < input.nodeNum; ++i) {
                if (isVisited(v, t, i)) {
                    visitedNodes.push_back(i);
                    containNode[i] = true;
                    coords.push_back({ input.nodes[i].xCoord, input.nodes[i].yCoord });
                }
            }
            if (visitedNodes.size() < 2) { continue; }
            if (visitedNodes.size() >= 3) {
                CachedTspSolver::Tour tour;
                if (!tspSolver.solve(tour, containNode, coords, [&](ID n) { return visitedNodes[n]; })) { return -1; }
                //if (lkh::solveTsp(tour, coordList)) {
                    //cout << "Failed to optimize period " << t << endl;
                    //return -1;
                //}
                tours[v][t].nodes = tour.nodes;
                tours[v][t].distance = tour.distance;
                // adjust node id on tour.
                //for (int i = 0; i < tour.nodes.size(); ++i) {
                    //tours[v][t].nodes.push_back(visitedNodes[tour.nodes[i]]);
                //}
            } else {
                // 2 nodes.
                tours[v][t].nodes = visitedNodes;
                tours[v][t].distance = routingCost[visitedNodes[0]][visitedNodes[1]]+ routingCost[visitedNodes[1]][visitedNodes[0]];
            }
            //cout << "Period " << t << "\t" << tours[v][t].nodes.size() << "\t" << tours[v][t].distance << endl;
            routingObj += tours[v][t].distance;
        }
    }
    return routingObj;
}

bool IrpModelSolver::check() {
    // check feasibility
    // check subtour
    for (ID v = 0; v < input.vehicleNum; ++v) {
        for (ID t = 0; t < input.periodNum; ++t) {
            if (cfg.isPresetEnabled() && aux.isPeriodFixed[t]) { continue; }
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
        if (cfg.isPresetEnabled() && aux.isPeriodFixed[t] && !cfg.withGlobalOptimization()) { continue; }
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
    List<bool> backup(aux.isPeriodFixed);
    fill(aux.isPeriodFixed.begin(), aux.isPeriodFixed.end(), true);
    if (!cfg.isShortageRelaxed() && (totalCost - getTotalCostInPeriod(0, input.periodNum) > DefaultDoubleGap)) {
        cout << "Worse actual objective = " << totalCost << ">" << getTotalCostInPeriod(0, input.periodNum) << endl;
        aux.isPeriodFixed = backup;
        return false;
    } else if (!cfg.isShortageRelaxed() && (getTotalCostInPeriod(0, input.periodNum) - totalCost > DefaultDoubleGap)) {
        cout << "Better actual objective = " << totalCost << "<" << getTotalCostInPeriod(0, input.periodNum) << endl;
    }
    aux.isPeriodFixed = backup;

    return true;
}

bool IrpModelSolver::eliminateSubtour(GetVarValue<bool> isTrue, MakeConstraint addLazy) {
    auto findLoop = [&](ID v, ID t, ID s, List<bool> &visited, function<void(const DecisionVar&)> process) {
        int len = 0;
        ID i = s;
        do {
            for (ID j = 0; j < input.nodeNum; ++j) {
                if ((j == i) || visited[j] || aux.skipNode[t][j]) { continue; }
                if (isTrue(x.xEdge[v][t][i][j])) {
                    visited[j] = true;
                    process(x.xEdge[v][t][i][j]);
                    ++len;
                    i = j;
                    break;
                }
            }
        } while (i != s);
        visited[s] = true;
        return len;
    };

    bool subtourFound = false;
    bool terminateLoop = false;
    Length maxLen = 0; // node count
    LinearExpr bestSubtour = 0;
    int bestSubtourCount = 0;
    for (ID v = 0; (v < input.vehicleNum) && !terminateLoop; ++v) {
        for (ID t = 0; (t < input.periodNum) && !terminateLoop; ++t) {
            if (cfg.isPresetEnabled() && aux.isPeriodFixed[t]) { continue; }
            List<bool> visited(input.nodeNum, false);
            findLoop(v, t, 0, visited, [](const DecisionVar &var) { return; });

            // find sub tours.
            for (ID s = 1; (s < input.nodeNum) && !terminateLoop; ++s) {
                if (visited[s] || aux.skipNode[t][s]) { continue; }
                LinearExpr subtour = 0;
                int len = findLoop(v, t, s, visited, [&](const DecisionVar &var) { subtour += var; });
                if (len <= 0) { continue; }
                subtourFound = true;
                if (cfg.subtourPolicy == Configuration::SubtourPolicy::EliminateAllSubtours) {
                    addLazy(subtour <= len - 1);
                } else if (cfg.subtourPolicy == Configuration::SubtourPolicy::EliminateFirstSubtour) {
                    maxLen = len;
                    bestSubtour = subtour;
                    terminateLoop = true;
                } else if (cfg.subtourPolicy == Configuration::SubtourPolicy::EliminateLongestSubtour) {
                    if (len > maxLen) {
                        maxLen = len;
                        bestSubtour = subtour;
                        bestSubtourCount = 1;
                    } else if (len == maxLen) {
                        ++bestSubtourCount;
                        bool update = (rand() % bestSubtourCount < 1);
                        maxLen = (update ? len : maxLen);
                        bestSubtour = (update ? subtour : bestSubtour);
                    }
                }
            }
        }
    }
    if (subtourFound && (cfg.subtourPolicy != Configuration::SubtourPolicy::EliminateAllSubtours)) {
        addLazy(bestSubtour <= maxLen - 1);
    }
    return subtourFound;
}

void IrpModelSolver::SolutionFound::callback() {
    try {
        if (where == GRB_CB_MIPSOL) {
            bool subtourFound = false;
            if (solver.cfg.isEliminationLazy()) {
                subtourFound = solver.eliminateSubtour(
                    [&](const DecisionVar &var) { return (getSolution(var) > (1 - IrpModelSolver::DefaultDoubleGap)); },
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
            //if (solver.cfg.isPresetEnabled()
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
                [&](const DecisionVar &var) { return (getSolution(var) > (1 - IrpModelSolver::DefaultDoubleGap)); },
                [&](const GRBTempConstr& tc) { addLazy(tc); });
        }
    } catch (GRBException e) {
        cout << "Error number: " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch (...) {
        cout << "Error during tsp callback" << endl;
    }
}

void IrpModelSolver::RelaxedSolutionFound::callback() {
    try {
        if (where == GRB_CB_MIPSOL) {
            //if (getDoubleInfo(GRB_CB_MIPSOL_OBJ) > min(solver.currentObjective.totalCost, input.referenceObjective)) { return; }
            //{
            //    double bestObj =
            //        solver.currentObjective.holdingCost
            //        + solver.currentObjective.routingCost * solver.aux.routingParameter
            //        + 4 * solver.aux.visitParameter;
            //    if (getDoubleInfo(GRB_CB_MIPSOL_OBJ) > bestObj) { cout << endl;return; }
            //}

            // eliminate sub tours.
            if (iterNoImprove > iterNoImprove) {
                bool subtourFound = solver.eliminateSubtour(
                    [&](const DecisionVar &var) { return (getSolution(var) > (1 - IrpModelSolver::DefaultDoubleGap)); },
                    [&](const GRBTempConstr& tc) { addLazy(tc); });
            }

            // forbid current solution with lazy constraints
            int strategy = 2;
            List<int> visitedPeriodNum(input.vehicleNum, 0);
            if (strategy == 1) {
                // make the visited nodes set different at a randomly selected period.
                ID v = rand() % input.vehicleNum;
                ID t = rand() % input.periodNum;
                LinearExpr diffCount = 0;
                for (ID i = 1; i < input.nodeNum; ++i) {
                    bool visited = false;
                    LinearExpr degree = 0;
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
                // gurantee that visited node set different from current solution.
                LinearExpr diffCount = 0;
                for (ID v = 0; v < input.vehicleNum; ++v) {
                    for (ID t = 0; t < input.periodNum; ++t) {
                        for (ID i = 0; i < input.nodeNum; ++i) {
                            bool visited = false;
                            LinearExpr degree = 0;
                            for (ID j = 0; j < input.nodeNum; ++j) {
                                if (j == i) { continue; }
                                if (isTrue(solver.x.xEdge[v][t][i][j])) { visited = true; }
                                degree += solver.x.xEdge[v][t][i][j];
                            }
                            if (visited) {
                                // visited node.
                                diffCount += 1 - degree;
                                if (i == 0) { ++visitedPeriodNum[v]; }
                            } else {
                                // unvisited node.
                                diffCount += degree;
                            }
                        }
                    }
                }
                addLazy(diffCount >= 1);
            }
            if (visitedPeriodNum[0] >= input.periodNum - 1) { return; }
            cout << getDoubleInfo(GRB_CB_MIPSOL_OBJ) << "\t";
            cout << visitedPeriodNum[0] << "\t";
            // 子回路*距离增加量 作为惩罚项加入目标
            /*
            {
                for (ID v = 0; v < input.vehicleNum; ++v) {
                    for (ID t = 0; t < input.periodNum; ++t) {
                        
                        LinearExpr edges = 0; // record current path(including subtours).
                        double increasedCost = tours[v][t].distance;
                        if (increasedCost == 0) { continue; }
                        double dist = 0;
                        for (ID i = 0; i < input.nodeNum; ++i) {
                            for (ID j = 0; j < input.nodeNum; ++j) {
                                if (i == j) { continue; }
                                if (!isTrue(solver.x.xEdge[v][t][i][j])) { continue; }
                                increasedCost -= solver.routingCost[i][j];
                                edges += solver.x.xEdge[v][t][i][j];
                            }
                        }
                        //increasedCost = increasedCost / 3;
                        //addLazy(solver.x.xSubtour >= increasedCost * (edges - edges.size() + 1));
                        if (increasedCost <= 0) { continue; }
                       
                        // find subtours.
                        List<bool> visited(input.nodeNum, false);
                        List<LinearExpr> subtours;
                        for (ID i = 0; i < input.nodeNum; ++i) {
                            if (visited[i]) { continue; }
                            ID p = i;
                            LinearExpr subtour;
                            do {
                                for (ID j = 0; j < input.nodeNum; ++j) {
                                    if (p == j) { continue; }
                                    if (isTrue(solver.x.xEdge[v][t][p][j])) {
                                        subtour += solver.x.xEdge[v][t][p][j];
                                        visited[p] = true;
                                        p = j;
                                        break;
                                    }
                                }
                            } while (p != i);
                            if (i == 0) { continue; }
                            if (subtour.size() > 0) { subtours.push_back(subtour); }
                        }
                        for (auto s = subtours.begin(); s != subtours.end(); ++s) {
                            addLazy(solver.x.xSubtour >= increasedCost * 4 * (*s - s->size() + 1));
                        }
                    }
                }
            }
            */
            
            // solve tsp with lkh.
            List2D<CachedTspSolver::Tour> tours;
            double holdingCost = getHoldingCost();
            double routingCost =
                solver.solveVrpWithLkh(tours, [&](ID v, ID t, ID i) { return (getSolution(solver.x.xQuantity[v][t][i]) > DefaultDoubleGap); });
            if (routingCost < 0) {
                // failed to solve tsp with lkh
                return;
            }
            ++iterNoImprove;
            // update optimal.
            if (holdingCost + routingCost < solver.currentObjective.totalCost) {
                iterNoImprove = 0;
                cout << "* ";
                solver.currentObjective.holdingCost = holdingCost;
                solver.currentObjective.routingCost = routingCost;
                solver.currentObjective.totalCost = holdingCost + routingCost;
                retrieveDeliveryQuantity();
                solver.convertTourToEdges(solver.presetX, tours);
                if (solver.cfg.useBenchmark && (solver.currentObjective.totalCost <= solver.input.bestObjective)) {
                    abort();
                }
                // record path.
                /*
                {
                    double tc = 0;
                    ostringstream oss;
                    for (ID v = 0; v < input.vehicleNum; ++v) {
                        for (ID t = 0; t < input.periodNum; ++t) {
                            oss << tours[v][t].distance << ",,";
                            for (auto i = tours[v][t].nodes.begin(); i != tours[v][t].nodes.end(); ++i) {
                                oss << *i << ",";
                            }
                            oss << endl;
                            ostringstream subtour;
                            double cost = 0;
                            List<bool> visited(input.nodeNum, false);
                            for (ID i = 0; i < input.nodeNum; ++i) {
                                if (visited[i]) { continue; }
                                ID p = i;
                                do {
                                    for (ID j = 0; j < input.nodeNum; ++j) {
                                        if (p == j) { continue; }
                                        if (isTrue(solver.x.xEdge[v][t][p][j])) {
                                            if (p == i) { subtour << p << ","; }
                                            subtour << j << ",";
                                            if (j == i) { subtour << ","; }
                                            cost += solver.routingCost[p][j];
                                            visited[p] = true;
                                            p = j;
                                            break;
                                        }
                                    }
                                } while (p != i);
                            }
                            oss << cost << ",," << subtour.str() << endl;
                            tc += cost;
                        }
                        oss << endl << endl;
                    }
                    ostringstream fileName;
                    fileName << "tmp/" << (tc + routingCost) << "_" << (holdingCost + routingCost) << ".csv";
                    ofstream ofs(fileName.str());
                    ofs << oss.str();
                    ofs.close();
                }
                */
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

double IrpModelSolver::RelaxedSolutionFound::getHoldingCost() {
    return solver.getHoldingCost(solver.x.xQuantity, [&](const DecisionVar &var) { return getSolution(var); });
}

void IrpModelSolver::RelaxedSolutionFound::retrieveDeliveryQuantity() {
    solver.retrieveDeliveryQuantity(solver.presetX, [&](ID v, ID t, ID i) { return (getSolution(solver.x.xQuantity[v][t][i])); });
}

}
