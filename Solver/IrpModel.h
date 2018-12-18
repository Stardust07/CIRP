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
#include "LKH3Lib/lkhSolver.h"


namespace szx {

class IrpModelSolver {
    #pragma region Type
public:
    using MpSolver = MpSolverGurobi;
    template<typename T>
    using List2D = List<List<T>>;

    struct Node {
        double xCoord;
        double yCoord;
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

    struct PresetX {
        List<bool> isPeriodFixed;        // isPeriodFixed[t] = true shows the routing of period t is fixed.
        List2D<List2D<bool>> xEdge;      // xEdge[v][t][i][j] = true shows edge (i,j) is on the path in period t.
        List2D<List<double>> xQuantity;  // xQuantity[v][t][i] denotes the delivery quantity of node i in period t.
        List2D<List<double>> xSequence;  // xSequence[v][t][i] denotes the visit order of node i in period t.
        List2D<List<bool>> xVisited;     // xVisited[v][t][i] = true shows node i is visited in period t.
    };

    struct Objective {
        double routingCost;
        double holdingCost;
        double totalCost;
    };

    #pragma endregion Type

    #pragma region Constant
public:
    static constexpr double Infinity = MpSolver::Infinity;
    static constexpr auto DefaultObjectiveOptimaOrientation = MpSolver::OptimaOrientation::Minimize;
    static constexpr int MillisecondsPerSecond = 1000;
    static constexpr int DefaultTimeLimitSecond = 7200;
    static constexpr int DefaultMaxThreadNum = 4;
    static constexpr int DefaultSlideWindowSize = 3;
    static constexpr double DefaultDoubleGap = 0.001;
    #pragma endregion Constant

    #pragma region Constructor
public:
    IrpModelSolver() : callback(*this), tspCallback(*this) {
        initSolver();
    }
    IrpModelSolver(const Input &inp, bool useBenchmark = true) : input(inp), callback(*this), tspCallback(*this) {
        cfg.useBenchmark = useBenchmark;
        initSolver();
    }
    #pragma endregion Constructor

    #pragma region Method
public:
    bool solve();
    bool solveIteratively();
    bool solveInventoryModel();
    bool solveTspModel();
    bool optimizeInventory();
    void retrieveSolution();

    static void saveSolution(const Input &input, const PresetX &presetX, const std::string &path);

    // cost expressions
    MpSolver::LinearExpr totalCostInPeriod(int start, int size, bool addInitial = true);
    MpSolver::LinearExpr routingCostInPeriod(int start, int size);
    MpSolver::LinearExpr holdingCostInPeriod(int start, int size, bool addInitial = true);
    //MpSolver::LinearExpr increasedHoldingCost(ID t);
    MpSolver::LinearExpr restQuantityUntilPeriod(ID node, int size);
    MpSolver::LinearExpr totalShortageQuantity();

    double getMpSolverObjValue() { return mpSolver.getObjectiveValue(); }
    double getDurationInSecond() { return elapsedSeconds; }
    double getHoldingCost(const List2D<List<MpSolver::DecisionVar>> &quantity, std::function<double(MpSolver::DecisionVar)> getValue);
    double getHoldingCostInPeriod(int start, int size, bool addInitial = true) { return mpSolver.getValue(holdingCostInPeriod(start, size, addInitial)); }
    double getTotalCostInPeriod(int start, int size, bool addInitial = true) { return mpSolver.getValue(totalCostInPeriod(start, size, addInitial)); }
    double getRestQuantityUntilPeriod(ID node, int size) { return mpSolver.getValue(restQuantityUntilPeriod(node, size)); }
    int getIntQuantity(ID v, ID t, ID n) { return lround(mpSolver.getValue(x.xQuantity[v][t][n])); }
    double solveVrpWithLkh(List2D<lkh::Tour> &tours, std::function<bool(ID, ID, ID)> isVisited);

    void enablePresetSolution() {
        if (cfg.usePresetSolution) { return; }
        //cfg.useBenchmark = false;
        cfg.usePresetSolution = true;
        cfg.optimizeTotalCost = true;
        presetX.isPeriodFixed.resize(input.periodNum, false);
    }
    void fixPeriod(ID t) {
        if (!cfg.usePresetSolution) { enablePresetSolution(); };
        if ((t < 0) || (presetX.isPeriodFixed.size() <= t)) { return; }
        presetX.isPeriodFixed[t] = true;
    }
    void setPresetQuantity(ID v, ID t, List<double> xQuantity) {
        if ((v < 0) || (t < 0)) { return; }
        if ((presetX.xQuantity.size() <= v) || (presetX.xQuantity[v].size() <= t)) { return; }
        presetX.xQuantity[v][t] = xQuantity;
    }
    void enableRelaxShortage() { cfg.allowShortage = true; }
    void relaxTspSubtourConstraint() {
        cfg.allowSubtour = true;
        cfg.useBenchmark = false;
    }
    // set mpsolver parameters.
    void setFindFeasiblePreference() { mpSolver.setFocus(1); }
    void setTimeLimitInSecond(int second) { mpSolver.setTimeLimitInSecond(second); }
    void setMaxThreadNum(int thread) { mpSolver.setMaxThreadNum(thread); }
    void setOutput(int enableOutput) { mpSolver.setOutput(enableOutput); }

protected:
    bool check();

    // for complete model and relaxation.
    void addDecisionVars();
    void addPathConnectivityConstraint();
    void addDeliveryQuantityConstraint();
    void addLoadDeliveryBalanceConstraint();
    void addCustomerLevelConstraint();
    void addSupplierLevelConstraint();
    void addSubtourEliminationConstraint();
    void setTotalCostObjective() {
        mpSolver.setObjective(totalCostInPeriod(0, input.periodNum), MpSolver::OptimaOrientation::Minimize);
    }
    void setShortageQuantityObjective() {
        mpSolver.setObjective(totalShortageQuantity(), MpSolver::OptimaOrientation::Minimize);
    }

    void setSolutionFoundCallback() { mpSolver.setCallback(&callback); }
    void setInitSolution();
    void initSolver() {
        elapsedSeconds = 0;
        setTimeLimitInSecond(DefaultTimeLimitSecond);
        setMaxThreadNum(DefaultMaxThreadNum);
        setOutput(cfg.enableMpOutput);

        if (cfg.useBenchmark) {
            currentObjective.totalCost = 2 * input.referenceObjective;
        } else {
            currentObjective.totalCost = 0;
        }
    }
    void initRoutingCost() {
        if (routingCost.empty()) { 
            std::cout << "Error: missing routing cost!" << std::endl;
            int x; 
            std::cin >> x;
        } else {
            return;
        }
    }

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
    }

    // for inventory model.
    void addInventoryVariables();
    void addNodeCapacityConstraint();
    void addQuantityConsistencyConstraint();
    void setHoldingCostObjective();

    // for routing model.
    void addRoutingVariables();
    void addNodeDegreeConstraints();
    void setRoutingCostObjective();

    bool eliminateSubtour(std::function<bool(const MpSolver::DecisionVar&)> isTrue, std::function<void(const MpSolver::LinearRange&)> addLazy);
    void convertTourToEdges(PresetX &presetX, const List2D<lkh::Tour> &tours) {
        for (ID v = 0; v < input.vehicleNum; ++v) {
            for (ID t = 0; t < input.periodNum; ++t) {
                // convert result of lkh to presetx
                if (tours[v][t].nodes.empty()) { continue; }
                int i = 0;
                for (; i < tours[v][t].nodes.size() - 1; ++i) {
                    presetX.xEdge[v][t][tours[v][t].nodes[i]][tours[v][t].nodes[i + 1]] = true;
                }
                presetX.xEdge[v][t][tours[v][t].nodes[i]][0] = true;
            }
        }
    }
    // retrieve quantity only and reset edge to false.
    void retrieveDeliveryQuantity(PresetX &presetX, std::function<double(ID v, ID t, ID i)> getQuantity);
    #pragma endregion Method

    #pragma region Field
public:
    Input input;
    List2D<double> routingCost;

    double elapsedSeconds;    // elapsed seconds when the best solution found.
    Objective currentObjective;

    // initial solution
    PresetX presetX;

protected:
    MpSolver mpSolver;

    struct Configuration {
        bool useLazyConstraints;
        bool useBenchmark;
        bool usePresetSolution;
        bool allowShortage;
        bool allowSubtour;
        bool optimizeTotalCost;
        bool forbidAllSubtours;
        bool enableMpOutput;

        Configuration() :useLazyConstraints(true), useBenchmark(true), usePresetSolution(false), allowShortage(false),
            allowSubtour(false), optimizeTotalCost(false), forbidAllSubtours(true), enableMpOutput(true) {}
    } cfg;

    struct {
        List2D<List2D<MpSolver::DecisionVar>> xEdge;
        List2D<List<MpSolver::DecisionVar>> xQuantity; // delivered quantity
        List2D<List<MpSolver::DecisionVar>> xSequence; // visited order
        List<MpSolver::DecisionVar> xMinLevel;         // minimal surplus for nodes
        List<MpSolver::DecisionVar> xShortage;         // shortage quantity for nodes,always non-negative
        List2D<List<MpSolver::DecisionVar>> xVisited;
    } x;

    struct {
        List2D<bool> skipNode;
    } aux;

    class SolutionFound : public GRBCallback {
        IrpModelSolver &solver;

    public:
        SolutionFound(IrpModelSolver &sol) :solver(sol) {}

    protected:
        void callback();
        void printSolutionInfo(); // print routing matrix.
    } callback;

    class TspSolutionFound : public GRBCallback {
        IrpModelSolver &solver;

    public:
        TspSolutionFound(IrpModelSolver &sol) :solver(sol) {}

    protected:
        void callback();
    } tspCallback;

    // for tsp-relaxed model
    class InfeasibleFound : public GRBCallback {
        IrpModelSolver &solver;
        const IrpModelSolver::Input &input;

    public:
        InfeasibleFound(IrpModelSolver &sol) :solver(sol), input(sol.input) {}

    protected:
        void callback();
        bool isTrue(MpSolver::DecisionVar var) {
            double x = getSolution(var);
            return (getSolution(var) > (1 - IrpModelSolver::DefaultDoubleGap));
        }
        double getHoldingCost();
        void retrieveDeliveryQuantity();
    };
    #pragma endregion Field
}; // IrpModelSolver

}


#endif SMART_QYM_INVENTORY_ROUTING_MODEL_H
