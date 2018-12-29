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
#include "LKH3Lib/CachedTspSolver.h"


namespace szx {

class IrpModelSolver {
    #pragma region Type
public:
    using MpSolver = MpSolverGurobi;
    using VariableType = MpSolver::VariableType;
    using DecisionVar = MpSolver::DecisionVar;
    using LinearExpr = MpSolver::LinearExpr;
    using OptimaOrientation = MpSolver::OptimaOrientation;
    template<typename T>
    using List2D = List<List<T>>;
    template<typename T>
    using List3D = List2D<List<T>>;
    template<typename T>
    using GetVarValue = std::function<T(const DecisionVar&)>;
    template <typename T>
    using GetValueById = std::function<T(ID, ID, ID)>;
    using MakeConstraint = std::function<void(const MpSolver::LinearRange&)>;

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
        List2D<List2D<bool>> xEdge;// xEdge[v][t][i][j] = true shows edge (i,j) is on the path in period t.
        List3D<double> xQuantity;  // xQuantity[v][t][i] denotes the delivery quantity of node i in period t.
        List3D<double> xSequence;  // xSequence[v][t][i] denotes the visit order of node i in period t.
        List3D<bool> xVisited;     // xVisited[v][t][i] = true shows node i is visited in period t.
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
    static constexpr auto DefaultObjectiveOptimaOrientation = OptimaOrientation::Minimize;
    static constexpr int MillisecondsPerSecond = 1000;
    static constexpr int DefaultTimeLimitSecond = 600;
    static constexpr int DefaultMaxThreadNum = 4;
    static constexpr int DefaultSlideWindowSize = 3;
    static constexpr double DefaultDoubleGap = 0.001;
    #pragma endregion Constant

    #pragma region Constructor
public:
    IrpModelSolver() {}
    IrpModelSolver(const Input &inp, bool useBenchmark = true) : input(inp) {
        cfg.useBenchmark = useBenchmark;
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

    static void saveSolution(const Input &input, const PresetX &presetX, const String &path);

    // cost expressions
    LinearExpr totalCostInPeriod(int start, int size, bool addInitial = true);
    LinearExpr routingCostInPeriod(int start, int size);
    LinearExpr holdingCostInPeriod(int start, int size, bool addInitial = true);
    //LinearExpr increasedHoldingCost(ID t);
    LinearExpr restQuantityUntilPeriod(ID node, int size);
    LinearExpr totalShortageQuantity();

    double getMpSolverObjValue() { return mpSolver.getObjectiveValue(); }
    double getDurationInSecond() { return elapsedSeconds; }
    double getHoldingCost(const List3D<DecisionVar> &quantity, GetVarValue<double> getValue);
    double getHoldingCostInPeriod(int start, int size, bool addInitial = true) { return mpSolver.getValue(holdingCostInPeriod(start, size, addInitial)); }
    double getTotalCostInPeriod(int start, int size, bool addInitial = true) { return mpSolver.getValue(totalCostInPeriod(start, size, addInitial)); }
    double getRestQuantityUntilPeriod(ID node, int size) { return mpSolver.getValue(restQuantityUntilPeriod(node, size)); }
    int getIntQuantity(ID v, ID t, ID n) { return lround(mpSolver.getValue(x.xQuantity[v][t][n])); }

    void enablePresetSolution() {
        if (cfg.isPresetEnabled()) { return; }
        cfg.presetPolicy = Configuration::PresetPolicy::OptimizeHoldingCostOfAll;
        aux.isPeriodFixed.resize(input.periodNum, false);
    }
    void setPeriodFixed(ID t) {
        if (!cfg.isPresetEnabled()) { enablePresetSolution(); };
        if ((t < 0) || (aux.isPeriodFixed.size() <= t)) { return; }
        aux.isPeriodFixed[t] = true;
    }
    void setPresetQuantity(ID v, ID t, List<double> quantity) {
        if ((v < 0) || (t < 0)) { return; }
        if ((presetX.xQuantity.size() <= v) || (presetX.xQuantity[v].size() <= t)) { return; }
        presetX.xQuantity[v][t] = quantity;
    }
    List<bool>& periodFixedData() { return aux.isPeriodFixed; }

    void relaxShortageConstraint() { 
        cfg.relaxationPolicy = Configuration::RelaxationPolicy::RelaxShortage; 
        cfg.useBenchmark = false;
    }
    void relaxTspSubtourConstraint() {
        cfg.subtourPolicy = Configuration::SubtourPolicy::AllowSubtours;
        cfg.relaxationPolicy = Configuration::RelaxationPolicy::RelaxSubtour;
        cfg.useBenchmark = false;
    }
    void setTspCachePath(const String &path) { aux.tspCacheFilePath = path; }

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
        //double k = 4;
        //mpSolver.setObjective(
        //    (routingCostInPeriod(0, input.periodNum) + k * holdingCostInPeriod(0, input.nodeNum)),
        //    OptimaOrientation::Minimize);
        mpSolver.setObjective(totalCostInPeriod(0, input.periodNum), OptimaOrientation::Minimize);
    }
    void setShortageQuantityObjective() {
        mpSolver.setObjective(totalShortageQuantity(), OptimaOrientation::Minimize);
    }

    // initialization
    void setInitSolution();
    void initSolver() {
        elapsedSeconds = 0;
        if (cfg.useBenchmark) {
            currentObjective.totalCost = 2 * input.referenceObjective;
        } else {
            currentObjective.totalCost = 0;
        }

        // set mpsolver parameters
        setTimeLimitInSecond(DefaultTimeLimitSecond);
        setMaxThreadNum(DefaultMaxThreadNum);
        setOutput(cfg.enableMpOutput);

        // set auxilury data
        initSkipNodes();
        initRoutingCost();
    }
    void initRoutingCost() {
        if (routingCost.empty()) { 
            std::cout << "Error: missing routing cost!" << std::endl;
            system("pause");
        } else {
            return;
        }
    }
    void initSkipNodes(double prob = 0.75) {
        aux.skipNode = List2D<bool>(input.periodNum, List<bool>(input.nodeNum, false));
        if (!cfg.skipNodes) { return; }

        if (presetX.xQuantity.empty()) { return; }

        auto shouldSkipUnvisitedNode = [&](ID t, ID n, double prob) {
            for (ID v = 0; v < input.vehicleNum; ++v) {
				// don't skip customers with preset delivery quantity.
                if (lround(presetX.xQuantity[v][t][n]) > 0) { return false; }
            }
            return (rand() % 100 < 100 * prob);
        };
        
        for (ID t = 0; t < input.periodNum; ++t) {
            if (cfg.isPresetEnabled() && aux.isPeriodFixed[t]) { continue; }
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

    // auxilury methods.
    bool eliminateSubtour(GetVarValue<bool> isTrue, MakeConstraint addLazy);
    void convertTourToEdges(PresetX &presetX, const List2D<CachedTspSolver::Tour> &tours) {
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
    double solveVrpWithLkh(List2D<CachedTspSolver::Tour> &tours, GetValueById<bool> isVisited);

    // retrieve quantity only and reset edge to false.
    void retrieveDeliveryQuantity(PresetX &presetX, GetValueById<double> getQuantity);
    // TODO[qym][0]: rename 
    void banCurSln(GetValueById<bool> isVisited, MakeConstraint addLazy);
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
        enum SubtourPolicy {
            AllowSubtours = 0,
            BanSubtours = 1,
            SequenceModel = 1,
            EliminateSubtours = 2,
            EliminateAllSubtours = 2,
            EliminateFirstSubtour = 3,
            EliminateLongestSubtour = 4
        };
        enum RelaxationPolicy {
            NoRelaxation, RelaxSubtour, RelaxShortage
        };
        enum PresetPolicy {
            NoPresetSolution= 0, 
            WithPresetSolution = 1, 
            OptimizeHoldingCostOfAll = 1, 
            OptimizeHoldingCostOfPart = 2
        };
        
        bool useBenchmark; // abort optimization in advance
        bool skipNodes;
        SubtourPolicy subtourPolicy;
        RelaxationPolicy relaxationPolicy;
        PresetPolicy presetPolicy;
        bool enableMpOutput;

        Configuration() :useBenchmark(true), enableMpOutput(true),
            skipNodes(false), presetPolicy(PresetPolicy::NoPresetSolution),
            relaxationPolicy(RelaxationPolicy::NoRelaxation), subtourPolicy(SubtourPolicy::EliminateAllSubtours) {}

        bool isEliminationLazy() { return (subtourPolicy >= SubtourPolicy::EliminateSubtours); }
        bool isSubtoursAllowed() { return (subtourPolicy == SubtourPolicy::AllowSubtours); }
        bool isShortageRelaxed() { return (relaxationPolicy == RelaxationPolicy::RelaxShortage); }
        bool isPresetEnabled() { return (presetPolicy >= PresetPolicy::WithPresetSolution); }
        bool withGlobalOptimization() { return (presetPolicy == PresetPolicy::OptimizeHoldingCostOfAll); }
    } cfg;

    struct {
        List2D<List2D<DecisionVar>> xEdge;
        List3D<DecisionVar> xQuantity;       // delivered quantity
        List3D<DecisionVar> xSequence;       // visited order
        List<DecisionVar> xMinLevel;         // minimal surplus for nodes
        List<DecisionVar> xShortage;         // shortage quantity for nodes,always non-negative
        List3D<DecisionVar> xVisited;
    } x;

    struct {
        List<bool> isPeriodFixed;  // isPeriodFixed[t] = true shows the routing of period t is fixed.
        List2D<bool> skipNode;
        String tspCacheFilePath;
    } aux;

    class SolutionFound : public GRBCallback {
        IrpModelSolver &solver;

    public:
        SolutionFound(IrpModelSolver &sol) :solver(sol) {}

    protected:
        void callback();
        void printSolutionInfo(); // print routing matrix.
    };

    class TspSolutionFound : public GRBCallback {
        IrpModelSolver &solver;

    public:
        TspSolutionFound(IrpModelSolver &sol) :solver(sol) {}

    protected:
        void callback();
    };

    // for tsp-relaxed model
    class RelaxedSolutionFound : public GRBCallback {
        IrpModelSolver &solver;
        const IrpModelSolver::Input &input;

    public:
        RelaxedSolutionFound(IrpModelSolver &sol) :solver(sol), input(sol.input) {}

    protected:
        void callback();
        bool isTrue(const DecisionVar &var) {
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
