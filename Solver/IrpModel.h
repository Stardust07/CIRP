////////////////////////////////
/// usage : 1.	model of inventory routing problem.
/// 
/// note  : 1.	
////////////////////////////////

#ifndef SMART_QYM_INVENTORY_ROUTING_MODEL_H
#define SMART_QYM_INVENTORY_ROUTING_MODEL_H


#include <algorithm>
#include <fstream>
#include <sstream>

#include "Common.h"
#include "MpSolverGurobi.h"

namespace szx {

class IrpModelSolver {
    #pragma region Type
public:
    using MpSolver = MpSolverGurobi;
    template<typename T>
    using List2D = List<List<T>>;

    struct Node {
        double xPoint;
        double yPoint;
        int initialQuantity;
        int capacity;
        int minLevel;
        int unitDemand;
        double holdingCost;
    };

    struct Input {
        int periodNum;
        int vehicleNum;
        int vehicleCapacity;
        int nodeNum;
        List<Node> nodes;

        double bestObjective;
        double referenceObjective;
    };

    struct Solution {
        List2D<List<double>> deliveryQuantity;
        List<double> costForPeriod;
    };

    struct PresetX {
        List<bool> isPeriodFixed;
        List2D<List2D<bool>> xEdge;
        List2D<List<double>> xQuantity;
        List2D<List<double>> xSequence;
    };
    #pragma endregion Type

    #pragma region Constant
public:
    static constexpr double Infinity = MpSolver::Infinity;
    static constexpr auto DefaultObjectiveOptimaOrientation = MpSolver::OptimaOrientation::Minimize;
    static constexpr int MillisecondsPerSecond = 1000;
    static constexpr int DefaultTimeLimitSecond = 7200;
    static constexpr int DefaultMaxThreadNum = 8;
    static constexpr int DefaultSlideWindowSize = 3;
    static constexpr double DefaultDoubleGap = 0.001;
    #pragma endregion Constant

    #pragma region Constructor
public:
    IrpModelSolver() : callback(*this), tspCallback(*this) {
        initSolver();
    }
    IrpModelSolver(const Input &inp, bool useBenchmark = true) : input(inp), callback(*this), tspCallback(*this) {
        //std::cout << "Solving " << "Instances_" + input.dirName + "\\" + input.instanceName << std::endl;
        cfg.useBenchmark = useBenchmark;
        initSolver();
    }
    #pragma endregion Constructor

    #pragma region Method
public:
    bool solve();
    bool solveIRPModel();
    bool solveRoutingModel();
    void retrieveSolution();

    static void saveSolution(const Input &input, const PresetX &presetX, const std::string &path);

    MpSolver::LinearExpr totalCostInPeriod(int start, int size, bool addInitial = true);
    MpSolver::LinearExpr routingCostInPeriod(int start, int size);
    MpSolver::LinearExpr holdingCostInPeriod(int start, int size, bool addInitial = true);
    //MpSolver::LinearExpr increasedHoldingCost(ID t);
    MpSolver::LinearExpr restQuantityUntilPeriod(ID node, int size);
    MpSolver::LinearExpr totalShortageQuantity();

    double getCostInPeriod(int start, int size, bool addInitial = true) { return mpSolver.getValue(totalCostInPeriod(start, size, addInitial)); }
    double getRestQuantityUntilPeriod(ID node, int size) { return mpSolver.getValue(restQuantityUntilPeriod(node, size)); }
    double getQuantity(ID v, ID t, ID n) { return mpSolver.getValue(x.xQuantity[v][t][n]); }
    double getBestObj() { return input.bestObjective; }
    double getReferenceObj() { return input.referenceObjective; }
    double getObjValue() { return mpSolver.getObjectiveValue(); }
    double getDurationInSecond() { return elapsedSeconds; }

    void enablePresetSolution() {
        if (cfg.usePresetSolution) { return; }
        cfg.useBenchmark = false;
        cfg.usePresetSolution = true;
        cfg.optimizeTotalCost = true;
        presetX.isPeriodFixed.resize(input.periodNum, false);
    }
    void fixPeriod(ID t) {
        if (!cfg.usePresetSolution) { enablePresetSolution(); };
        presetX.isPeriodFixed[t] = true;
    }
    void setPresetQuantity(ID v, ID t, List<double> xQuantity) {
        presetX.xQuantity[v][t] = xQuantity;
    }
    void setFindFeasiblePreference() { mpSolver.setFocus(1); }
    void enableRelaxMinLevel() { cfg.relaxMinlevel = true; }
    void setTimeLimitInSecond(int second) { mpSolver.setTimeLimitInSecond(second); }

protected:
    bool check();

    void addDecisionVars();
    void addPathConnectivityConstraint();
    void addDeliveryQuantityConstraint();
    void addLoadDeliveryBalanceConstraint();
    void addCustomerLevelConstraint();
    void addSupplierLevelConstraint();
    void addSubtourEliminationConstraint();
    void setCostObjective() {
        mpSolver.setObjective(totalCostInPeriod(0, input.periodNum), MpSolver::OptimaOrientation::Minimize);
    }
    void setShortageQuantityObjective() {
        mpSolver.setObjective(totalShortageQuantity(), MpSolver::OptimaOrientation::Minimize);
    }
    void setMixedObjective() {
        MpSolver::LinearExpr expr = 0;
        expr += totalCostInPeriod(0, input.periodNum);
        MpSolver::LinearExpr totalShortage = 0;
        int maxShortage = 0;
        for (ID i = 1; i < input.nodeNum; ++i) {
            totalShortage += x.xShortage[i];
            int restQuantity = input.nodes[i].initialQuantity - input.periodNum * input.nodes[i].unitDemand;
            maxShortage += ((restQuantity >= 0) ? 0 : -restQuantity);
        }
        mpSolver.makeConstraint(maxShortage * x.xShortage[0] >= totalShortage);

        expr += x.xShortage[0] * getBestObj() * 15;
        mpSolver.setObjective(expr, MpSolver::OptimaOrientation::Minimize);
    }
    void setSubTourCallback() { mpSolver.setCallback(&callback); }
    void setInitSolution();
    void initSolver() {
        elapsedSeconds = 0;
        setTimeLimitInSecond(DefaultTimeLimitSecond);
        mpSolver.setMaxThreadNum(DefaultMaxThreadNum);
        if (cfg.useBenchmark) {
            currentObjective = 2 * getReferenceObj();
        } else {
            currentObjective = 0;
        }
        //mpSolver.setOutput(false);
    }
    void initRoutingCost() {
        routingCost.resize(input.nodeNum);
        for (ID i = 0; i < input.nodeNum; ++i) {
            routingCost[i].resize(input.nodeNum);
            for (ID j = 0; j < input.nodeNum; ++j) {
                routingCost[i][j] = costOfEdge(input.nodes[i], input.nodes[j]);
            }
        }
    }

    double costOfEdge(Node i, Node j) {
        double value = 0;
        value = sqrt(pow(i.xPoint - j.xPoint, 2) + pow(i.yPoint - j.yPoint, 2));
        value = round(value);
        return value;
    }

    /*List<List<ID>> computeSubsets(List<ID> nodes) {
        if (nodes.size() == 1) { return List<List<ID>>(1, List<ID>(1, nodes[0])); }

        List<List<ID>> subSets;
        List<ID> newSet(nodes);
        for (ID i = 0; i < 1; ++i) {
            newSet.erase(newSet.begin() + i);
            List<List<ID>> res = computeSubsets(newSet);

            for (auto j = res.begin(); j != res.end(); ++j) {
                subSets.push_back(*j);
                List<ID> list(*j);
                list.push_back(nodes[i]);
                subSets.push_back(list);
            }

            subSets.push_back({ nodes[i] });
            newSet = nodes;
        }
        return subSets;
    }*/

    void initSkipNodes(double prob = 0.75) {
        aux.skipNode = List2D<bool>(input.periodNum, List<bool>(input.nodeNum, false));

        if (presetX.xQuantity.empty()) { return; }

        auto shouldSkipUnvisitedNode = [&](ID t, ID n, double prob) {
            for (ID v = 0; v < input.vehicleNum; ++v) {
				// don't skip customers with preset delivery quantity.
                if (lround(presetX.xQuantity[v][t][n]) > 0) { return false; }
            }
            return (rand() % 100 < 100 * prob);
        };
        
        for (ID t = 0; t < input.periodNum; ++t) {
            if (cfg.usePresetSolution && presetX.isPeriodFixed[t]) { continue; }
            ID skipCount = 0;
            std::cout << t << ": ";
            for (ID n = 1; n < input.nodeNum; ++n) {
                aux.skipNode[t][n] = shouldSkipUnvisitedNode(t, n, prob);
                skipCount += aux.skipNode[t][n];
                if (aux.skipNode[t][n]) { std::cout << n << " "; }
            }
            std::cout << std::endl << "skip " << skipCount << " nodes in period " << t << std::endl;
        }

        for (ID t = 0; t < input.periodNum; ++t) {
            if (cfg.usePresetSolution && presetX.isPeriodFixed[t]) { continue; }
            for (ID i = 1; i < input.nodeNum; ++i) {
                aux.skipNode[t][i] = shouldSkipUnvisitedNode(t, i, prob);
            }
        }
    }

    void addIRPVariables();
    void addNodeCapacityConstraint();
    void addQuantityConsistencyConstraint();
    void setHoldingCostObjective();

    void addRoutingVariables();
    void addRoutingConstraints();
    void setRoutingCostObjective();
    #pragma endregion Method

    #pragma region Field
public:
    Input input;
    Solution sln;
    List2D<double> routingCost;

    double elapsedSeconds;
    double currentObjective;

    // initial sln
    PresetX presetX;

protected:
    MpSolver mpSolver;

    struct Configuration {
        bool useLazyConstraints;
        bool useBenchmark;
        bool relaxMinlevel;
        bool optimizeTotalCost;
        bool usePresetSolution;
        bool forbidAllSubtours;

        Configuration() :useLazyConstraints(true), useBenchmark(true), relaxMinlevel(false),
            optimizeTotalCost(false), usePresetSolution(false), forbidAllSubtours(true) {}
    } cfg;

    struct {
        List2D<List2D<MpSolver::DecisionVar>> xEdge;
        List2D<List<MpSolver::DecisionVar>> xQuantity;
        List2D<List<MpSolver::DecisionVar>> xSequence;
        List<MpSolver::DecisionVar> xMinLevel;
        List<MpSolver::DecisionVar> xShortage;
    } x;

    struct {
        List2D<bool> skipNode;
    } aux;

    class SolutionFound : public GRBCallback {
        IrpModelSolver &solver;
        const IrpModelSolver::Input &input;

    public:
        SolutionFound(IrpModelSolver &sol) :solver(sol), input(sol.input) {}

    protected:
        void callback();

        bool isTrue(MpSolver::DecisionVar var) {
            double x = getSolution(var);
            return (getSolution(var) > (1 - IrpModelSolver::DefaultDoubleGap));
        }
        bool eliminateSubtour();
        void printSolutionInfo();
    } callback;

    class TSPSolutionFound : public GRBCallback {
        IrpModelSolver &solver;
        const IrpModelSolver::Input &input;

    public:
        TSPSolutionFound(IrpModelSolver &sol) :solver(sol), input(sol.input) {}

    protected:
        void callback();

        bool isTrue(MpSolver::DecisionVar var) {
            double x = getSolution(var);
            return (getSolution(var) > (1 - IrpModelSolver::DefaultDoubleGap));
        }
        bool eliminateSubtour();
        void printSolutionInfo();
    } tspCallback;
    #pragma endregion Field
}; // IrpModelSolver

}


#endif SMART_QYM_INVENTORY_ROUTING_MODEL_H