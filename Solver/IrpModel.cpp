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
    addDecisionVars();
    addPathConnectivityConstraint();
    addDeliveryQuantityConstraint();
    addLoadDeliveryBalanceConstraint();
    addCustomerLevelConstraint();
    addSupplierLevelConstraint();
    if (!cfg.useLazyConstraints) {
        addSubtourEliminationConstraint();
    }
    if (cfg.useBenchmark || cfg.useLazyConstraints) {
        setSubTourCallback();
    }
    if (cfg.usePresetSolution) {
        setInitSolution();
    }
    if (cfg.relaxMinlevel) {
        setShortageQuantityObjective();
        if (!mpSolver.optimize() || mpSolver.getObjectiveValue() > 0) {
            elapsedSeconds = DefaultTimeLimitSecond;
            return false;
        }
        mpSolver.makeConstraint(totalShortageQuantity() <= mpSolver.getObjectiveValue());
    } else {
        setCostObjective();
        if (!mpSolver.optimize()) {
            elapsedSeconds = mpSolver.getDurationInSecond();
            return false;
        }
        elapsedSeconds = (cfg.usePresetSolution || !cfg.useBenchmark) ? mpSolver.getDurationInSecond() : elapsedSeconds;
    }
    retrieveSolution();

    return check();
}

bool IrpModelSolver::solveIRPModel() {
    initRoutingCost();

    addIRPVariables();
    addNodeCapacityConstraint();
    addQuantityConsistencyConstraint();
    setHoldingCostObjective();
    if (!mpSolver.optimize()) {
        elapsedSeconds = mpSolver.getDurationInSecond();
        return false;
    }

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

bool IrpModelSolver::solveRoutingModel() {
    initRoutingCost();
    addRoutingVariables();
    addRoutingConstraints();
    setRoutingCostObjective();
    mpSolver.setCallback(&tspCallback);
    if (!mpSolver.optimize()) {
        elapsedSeconds = mpSolver.getDurationInSecond();
        return false;
    }
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

void IrpModelSolver::retrieveSolution() {
    List2D<List<double>> &deliveryQuantity(sln.deliveryQuantity);
    deliveryQuantity = List2D<List<double>>(input.vehicleNum, List2D<double>(input.periodNum, List<double>(input.nodeNum, 0)));
    List<double> &costForPeriod(sln.costForPeriod);
    costForPeriod.resize(input.periodNum, 0);
    for (int t = 0; t < input.periodNum; ++t) {
        costForPeriod[t] = getCostInPeriod(t, 1);

        for (int v = 0; v < input.vehicleNum; ++v) {
            for (int i = 0; i < input.nodeNum; ++i) {
                if (aux.skipNode[t][i]) { continue; }
                deliveryQuantity[v][t][i] = getQuantity(v, t, i);
            }
        }
        if (!cfg.usePresetSolution) { continue; }
    }

    cout << "Cost: ";
    for (int t = 0; t < input.periodNum; ++t) {
        cout << costForPeriod[t] << ", ";
    }
    cout << getCostInPeriod(0, input.periodNum) << endl;
    //cout << endl << "Quantity: " << endl;
    //for (int v = 0; v < input.vehicleNum; ++v) {
    //    for (int i = 0; i < input.nodeNum; ++i) {
    //        cout << "Node " << i << ": ";
    //        for (int t = 0; t < input.periodNum; ++t) {
    //            cout << deliveryQuantity[v][t][i] << ", ";
    //        }
    //        cout << endl;
    //    }
    //}
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

//void IrpModelSolver::saveSolution(const Input &input, const PresetX &presetX, const string &path) {
//    ofstream ofs(path);
//
//    for (ID t = 0; t < input.periodNum; ++t) {
//        ofs << "period," << t << ",================================" << endl;
//        for (ID v = 0; v < input.vehicleNum; ++v) {
//            ostringstream route;
//            route << v << ",";
//            ostringstream quant;
//            quant.precision(2);
//            quant << ",";
//
//            if (!presetX.xEdge[v][t].empty()) {
//                ID i = 0;
//                for (ID j = 0; j < input.nodeNum; ++j) {
//                    if (i == j) { continue; }
//                    if (!presetX.xEdge[v][t][i][j]) { continue; }
//
//                    route << i << ",";
//                    quant << fixed << presetX.xQuantity[v][t][i] << ",";
//
//                    if (j == 0) { break; }
//                    i = j;
//                    j = -1;
//                }
//            }
//
//            ofs << route.str() << endl << quant.str() << endl;
//        }
//    }
//}

void IrpModelSolver::addDecisionVars() {
    initSkipNodes();
    
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
    if (cfg.relaxMinlevel) {
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
    if (cfg.usePresetSolution) {
        for (ID t = 0; t < input.periodNum; ++t) {
            if (presetX.isPeriodFixed[t] && !cfg.optimizeTotalCost) { continue; }
            for (ID i = 0; i < input.nodeNum; ++i) {
                if (aux.skipNode[t][i]) { continue; }
                for (ID v = 0; v < input.vehicleNum; ++v) {
                    if (lround(presetX.xQuantity[v][t][i]) <= 0) {
                        mpSolver.makeConstraint(x.xQuantity[v][t][i] <= 0, "fq");
                    }
                }
            }
        }
    }

    for (ID t = 0; t < input.periodNum; ++t) {
        if (cfg.usePresetSolution && presetX.isPeriodFixed[t]) { continue; }
        for (ID i = 0; i < input.nodeNum; ++i) {
            if (aux.skipNode[t][i]) { continue; }
            MpSolver::LinearExpr totalQuantity = 0;
            MpSolver::LinearExpr visitedTime = 0;
            for (ID v = 0; v < input.vehicleNum; ++v) {
                totalQuantity += x.xQuantity[v][t][i];
                for (ID j = 0; j < input.nodeNum; ++j) {
                    if (i == j) { continue; }
                    if (aux.skipNode[t][j]) { continue; }
                    visitedTime += x.xEdge[v][t][i][j];
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
            mpSolver.makeConstraint(restQuantity >= (cfg.relaxMinlevel ? x.xMinLevel[i] : MpSolver::LinearExpr(input.nodes[i].minLevel)), "ml");
        }
        if (cfg.relaxMinlevel) {
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

void IrpModelSolver::addIRPVariables() {
    x.xQuantity.resize(input.vehicleNum);
    for (ID v = 0; v < input.vehicleNum; ++v) {
        x.xQuantity[v].resize(input.periodNum);
        for (ID t = 0; t < input.periodNum; ++t) {
            x.xQuantity[v][t].resize(input.nodeNum);
            x.xQuantity[v][t][0] = mpSolver.makeVar(
                MpSolver::VariableType::Real, -min(input.vehicleCapacity, input.nodes[0].capacity), 0);
            for (ID i = 1; i < input.nodeNum; ++i) {
                x.xQuantity[v][t][i] = mpSolver.makeVar(
                    MpSolver::VariableType::Real, 0, min(input.vehicleCapacity, input.nodes[i].capacity));
            }
        }
    }
}

void IrpModelSolver::addNodeCapacityConstraint() {
    for (ID i = 1; i < input.nodeNum; ++i) {
        MpSolver::LinearExpr restQuantity = 0;
        restQuantity += input.nodes[i].initialQuantity;
        for (ID t = 0; t < input.periodNum; ++t) {
            for (ID v = 0; v < input.vehicleNum; ++v) {
                restQuantity += x.xQuantity[v][t][i];
            }
            mpSolver.makeConstraint(restQuantity <= input.nodes[i].capacity);
            restQuantity -= input.nodes[i].unitDemand;
            mpSolver.makeConstraint(restQuantity >= input.nodes[i].minLevel);
        }
    }
    MpSolver::LinearExpr restQuantity = 0;
    restQuantity += input.nodes[0].initialQuantity;
    for (ID t = 0; t < input.periodNum; ++t) {
        for (ID v = 0; v < input.vehicleNum; ++v) {
            restQuantity += x.xQuantity[v][t][0];
        }
        restQuantity += input.nodes[0].unitDemand;
        mpSolver.makeConstraint(restQuantity >= input.nodes[0].minLevel);
    }
}

void IrpModelSolver::addQuantityConsistencyConstraint() {
    for (ID v = 0; v < input.vehicleNum; ++v) {
        for (ID t = 0; t < input.periodNum; ++t) {
            MpSolver::LinearExpr expr = 0;
            for (ID i = 0; i < input.nodeNum; ++i) {
                expr += x.xQuantity[v][t][i];
            }
            mpSolver.makeConstraint(expr == 0);
        }
    }
}

void IrpModelSolver::setHoldingCostObjective() {
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

void IrpModelSolver::addRoutingConstraints() {
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
    MpSolver::LinearExpr totalCost = 0;
    for (ID v = 0; v < input.vehicleNum; ++v) {
        for (ID t = 0; t < input.periodNum; ++t) {
            for (ID i = 0; i < input.nodeNum; ++i) {
                for (ID j = 0; j < input.nodeNum; ++j) {
                    if (i == j) { continue; }
                    totalCost += routingCost[i][j] * x.xEdge[v][t][i][j];
                }
            }
        }
    }
    mpSolver.setObjective(totalCost, MpSolver::OptimaOrientation::Minimize);
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
                        totalCost += routingCost[i][j] * presetX.xEdge[v][t][i][j];
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

IrpModelSolver::MpSolver::LinearExpr IrpModelSolver::increasedHoldingCost(ID t) {
    MpSolver::LinearExpr totalCost = 0;

    if (!cfg.usePresetSolution || !presetX.isPeriodFixed[t]) { return 0; }
    for (ID v = 0; v < input.vehicleNum; ++v) {
        totalCost -= input.nodes[0].holdingCost * (x.xQuantity[v][t][0] - presetX.xQuantity[v][t][0]);
    }
    for (ID i = 1; i < input.nodeNum; ++i) {
        if (aux.skipNode[t][i]) { continue; }
        for (ID v = 0; v < input.vehicleNum; ++v) {
            totalCost += input.nodes[i].holdingCost * (x.xQuantity[v][t][i] - presetX.xQuantity[v][t][i]);
        }
    }

    return totalCost;
}

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

//void IrpModelSolver::record() {
//    ostringstream log;
//    log << input.instanceName << ","
//        << input.bestObjective << ","
//        << mpSolver.getObjectiveValue() << ","
//        << elapsedSeconds << ","
//        << input.referenceObjective << ","
//        << getBenchmarkDuration() << endl;
//
//    // append all text atomically.
//    static mutex logFileMutex;
//    lock_guard<mutex> logFileGuard(logFileMutex);
//
//    ofstream logFileName(input.logFileName, ios::app);
//    logFileName.seekp(0, ios::end);
//    if (logFileName.tellp() <= 0) {
//        logFileName << "Instance,Best Obj,ObjValue,Duration,Benchmark,Cpu" << endl;
//    }
//    logFileName << log.str();
//    logFileName.close();
//}

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
    if (!cfg.relaxMinlevel && (totalCost - getCostInPeriod(0, input.periodNum) > DefaultDoubleGap)) {
        cout << "Worse actual objective = " << totalCost << ">" << getCostInPeriod(0, input.periodNum) << endl;
        presetX.isPeriodFixed = backup;
        return false;
    } else if (!cfg.relaxMinlevel && (getCostInPeriod(0, input.periodNum) - totalCost > DefaultDoubleGap)) {
        cout << "Better actual objective = " << totalCost << "<" << getCostInPeriod(0, input.periodNum) << endl;
    }
    presetX.isPeriodFixed = backup;

    return true;
}

void IrpModelSolver::SolutionFound::callback() {
    try {
        if (where == GRB_CB_MIPSOL) {
            bool subtourFound = false;
            if (solver.cfg.useLazyConstraints) {
                subtourFound = eliminateSubtour();
            }
            if (subtourFound) { return; }

            if (solver.cfg.useBenchmark && abs(getDoubleInfo(GRB_CB_MIPSOL_OBJ) - solver.input.bestObjective) <= DefaultDoubleGap) {
                solver.elapsedSeconds = getDoubleInfo(GRB_CB_RUNTIME);
                abort();
            }
            if (solver.cfg.useBenchmark && (getDoubleInfo(GRB_CB_MIPSOL_OBJ) < solver.currentObjective)) {
                solver.currentObjective = getDoubleInfo(GRB_CB_MIPSOL_OBJ);
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

bool IrpModelSolver::SolutionFound::eliminateSubtour() {
    bool subtourFound = false;
    for (ID v = 0; v < input.vehicleNum; ++v) {
        for (ID t = 0; t < input.periodNum; ++t) {
            if (solver.cfg.usePresetSolution && solver.presetX.isPeriodFixed[t]) { continue; }
            List<bool> visited(input.nodeNum, false);
            visited[0] = true;
            ID n = 0;
            do {
                for (ID i = 0; i < input.nodeNum; ++i) {
                    if (n == i) { continue; }
                    if (solver.aux.skipNode[t][i]) { continue; }
                    if (isTrue(solver.x.xEdge[v][t][n][i])) {
                        visited[i] = true;
                        n = i;
                        break;
                    }
                }
            } while (n != 0);
            for (ID s = 1; s < input.nodeNum; ++s) {
                if (visited[s]) { continue; }
                if (solver.aux.skipNode[t][s]) { continue; }
                MpSolver::LinearExpr expr = 0;
                List<ID> path;
                int len = 0;
                n = s;
                path.push_back(n);
                do {
                    for (ID i = 0; i < input.nodeNum; ++i) {
                        if (n == i) { continue; }
                        if (solver.aux.skipNode[t][i]) { continue; }
                        if (isTrue(solver.x.xEdge[v][t][n][i])) {
                            expr += solver.x.xEdge[v][t][n][i];
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
                    if(!solver.cfg.forbidAllSubtours) { break; }
                }
            }
        }
    }
    return subtourFound;
}

void IrpModelSolver::SolutionFound::printSolutionInfo() {
    for (ID v = 0; v < input.vehicleNum; ++v) {
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

void IrpModelSolver::TSPSolutionFound::callback() {
    try {
        if (where == GRB_CB_MIPSOL) {
            eliminateSubtour();
        }
    } catch (GRBException e) {
        cout << "Error number: " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch (...) {
        cout << "Error during tsp callback" << endl;
    }
}

bool IrpModelSolver::TSPSolutionFound::eliminateSubtour() {
    bool subtourFound = false;
    for (ID v = 0; v < input.vehicleNum; ++v) {
        for (ID t = 0; t < input.periodNum; ++t) {
            List<bool> visited(input.nodeNum, false);
            visited[0] = true;
            ID n = 0;
            do {
                for (ID i = 0; i < input.nodeNum; ++i) {
                    if (n == i) { continue; }
                    if (isTrue(solver.x.xEdge[v][t][n][i])) {
                        visited[i] = true;
                        n = i;
                        break;
                    }
                }
            } while (n != 0);
            for (ID s = 1; s < input.nodeNum; ++s) {
                if (visited[s]) { continue; }
                MpSolver::LinearExpr expr = 0;
                List<ID> path;
                int len = 0;
                n = s;
                path.push_back(n);
                do {
                    for (ID i = 0; i < input.nodeNum; ++i) {
                        if (n == i) { continue; }
                        if (isTrue(solver.x.xEdge[v][t][n][i])) {
                            expr += solver.x.xEdge[v][t][n][i];
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
                    //break;
                }
            }
        }
    }
    return subtourFound;
}

}
