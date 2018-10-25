////////////////////////////////
/// usage : 1.	wrap Gurobi optimizer to be more self documenting and easier to use.
///
/// note  : 1.  this project does not provide Gurobi lisence.
////////////////////////////////

#ifndef SMART_QYM_MPSOLVER_GUROBI_H
#define SMART_QYM_MPSOLVER_GUROBI_H


#include <string>
#include <vector>
#include <functional>

#include "gurobi_c.h"
#include "gurobi_c++.h"


class MpSolverGurobi {
    #pragma region Constant
public:
    // Gurobi variable can be GRB_CONTINUOUS, GRB_BINARY, GRB_INTEGER, GRB_SEMICONT, or GRB_SEMIINT.
    // Semi-continuous variables can take any value between the specified lower and upper bounds, or a value of zero. 
    // Semi-integer variables can take any integer value between the specified lower and upper bounds, or a value of zero
    enum VariableType { Bool = GRB_BINARY, Integer = GRB_INTEGER, Real = GRB_CONTINUOUS, SemiInt = GRB_SEMIINT, SemiReal = GRB_SEMICONT };

    enum OptimaOrientation { Minimize = GRB_MINIMIZE, Maximize = GRB_MAXIMIZE };

    // status for the most recent optimization.
    enum ResultStatus {
        Optimal,         // GRB_OPTIMAL
        Feasible,        // GRB_SUBOPTIMAL || (SolutionCount > 0)
        Proceeding,      // GRB_LOADED || GRB_INPROGRESS
        Ready,           // ready for solve()
        ExceedLimit,     // GRB_ITERATION_LIMIT || GRB_NODE_LIMIT || GRB_TIME_LIMIT || GRB_SOLUTION_LIMIT
        InsolubleCutoff, // GRB_CUTOFF
        InsolubleModel,  // GRB_INFEASIBLE || GRB_INF_OR_UNBD || GRB_UNBOUNDED
        OutOfMemory,     // OUT_OF_MEMORY
        Error            // any other status code or error code (you may not get it as exception is thrown)
    };

    // behavior of searching or recording alternative solutions.
    enum PoolingMode {
        Incidental = 0, // only keeps solutions found along the way to the optimum incidentally.
        Good = 1,       // keep searching for high quality solutions after the optimum is found.
        Top = 2         // search top K solutions in a systematic way.
    };

    static constexpr double Infinity = GRB_INFINITY;
    /// default thread number to let Gurobi make the decision.
    static constexpr int AutoThreading = 0;

    static constexpr int DefaultObjectivePriority = 0;
    static constexpr OptimaOrientation DefaultObjectiveOptimaOrientation = Maximize;

    static constexpr int MillisecondsPerSecond = 1000;

    static constexpr int MaxObjectivePriority = 32;

    static constexpr auto DefaultParameterPath = "tune.prm";
    static constexpr auto DefaultIrreducibleInconsistentSubsystemPath = "iis.ilp";
    #pragma endregion Constant

    #pragma region Type
public:
    using DecisionVar = GRBVar;
    using Constraint = GRBConstr;
    using LinearExpr = GRBLinExpr;
    using LinearRange = GRBTempConstr;

    using Millisecond = long long;
    using String = std::string;
    using OnSolutionFound = std::function<bool()>;
    #pragma endregion Type

    #pragma region Constructor
public:
    MpSolverGurobi();

    void loadModel(const String &inputPath) { model.read(inputPath); }
    void saveModel(const String &outputPath) {
        updateModel();
        model.write(outputPath);
    }
    #pragma endregion Constructor

    #pragma region Method
public:
    bool optimize();
    bool optimizeManually();

    void computeIIS(const String &outputPath = DefaultIrreducibleInconsistentSubsystemPath) {
        updateModel();
        try {
            model.computeIIS();
            model.write(outputPath);
        } catch (GRBException&) {
            status = ResultStatus::Error;
        }
    }

    // status.
    static bool reportStatus(ResultStatus status);
    Millisecond getDuration() const { return static_cast<Millisecond>(getDurationInSecond() * MillisecondsPerSecond); }
    double getDurationInSecond() const { return model.get(GRB_DoubleAttr_Runtime); }

    // decisions.
    DecisionVar makeVar(VariableType type, double lb, double ub, const String &name) { return model.addVar(lb, ub, 0, static_cast<char>(type), name); }
    DecisionVar makeVar(VariableType type, double lb = 0, double ub = 1.0) { return makeVar(type, lb, ub, ""); }

    double getIntValue(const LinearExpr &expr) const { return round(getValue(expr)); }
    double getValue(const LinearExpr &expr) const { return (expr.getValue()); }
    double getValue(const DecisionVar &var) const { return var.get(GRB_DoubleAttr_X); }
    double getAltValue(const DecisionVar &var, int solutionIndex) {
        setAltSolutionIndex(solutionIndex);
        return var.get(GRB_DoubleAttr_Xn);
    }

    // use the given value as the initial sln in MIP.
    void setInitValue(DecisionVar &var, double value) { var.set(GRB_DoubleAttr_Start, value); }

    void setHintValue(DecisionVar &var, double value) { var.set(GRB_DoubleAttr_VarHintVal, value); }
    void setHintPrioriy(DecisionVar &var, int priority) { var.set(GRB_IntAttr_VarHintPri, priority); }

    bool isTrue(DecisionVar var) const { return getValue(var) > 0.5; }
    int getVariableCount() const { return model.get(GRB_IntAttr_NumVars); }

    //Arr<DecisionVar> getAllVars() const {
    //    return Arr<DecisionVar>(getVariableCount(), model.getVars());
    //}
    //void getAllValues(const Arr<DecisionVar> &vars, Arr<double> &values) const {
    //    values.init(vars.size());
    //    auto val = values.begin();
    //    for (auto var = vars.begin(); var != vars.end(); ++var, ++val) { *val = getValue(*var); }
    //}
    //void setAllInitValues(Arr<DecisionVar> &vars, const Arr<double> &values) {
    //    auto val = values.begin();
    //    for (auto var = vars.begin(); var != vars.end(); ++var, ++val) { setInitValue(*var, *val); }
    //}

    int getSolutionCount() const { return model.get(GRB_IntAttr_SolCount); }

    double getPoolObjBound() const { return model.get(GRB_DoubleAttr_PoolObjBound); }

    // constraints.
    Constraint makeConstraint(const LinearRange &r, const String &name = "") { return model.addConstr(r, name); }
    void removeConstraint(Constraint constraint) { model.remove(constraint); }
    int getConstraintCount() const { return model.get(GRB_IntAttr_NumConstrs); }

    // objectives.
    void setObjective(const LinearExpr &expr, OptimaOrientation orientation) {
        model.setObjective(expr, orientation);
    }

    double getObjectiveValue() const { return model.get(GRB_DoubleAttr_ObjVal); }
    double getAltObjectiveValue(int solutionIndex) {
        setAltSolutionIndex(solutionIndex);
        return model.get(GRB_DoubleAttr_PoolObjVal);
    }

    // configurations.
    void setTimeLimit(Millisecond millisecond) { setTimeLimitInSecond(static_cast<double>(millisecond / MillisecondsPerSecond)); }
    void setTimeLimitInSecond(double second) { model.set(GRB_DoubleParam_TimeLimit, second); }
    void setTimeLimitInSecond(int objIndex, double second) { model.getMultiobjEnv(objIndex).set(GRB_DoubleParam_TimeLimit, second); }
    void setMaxThreadNum(int threadNum) { model.set(GRB_IntParam_Threads, threadNum); }
    void setOutput(bool enable = true) { model.set(GRB_IntParam_OutputFlag, enable); }
    void setFocus(int focus = 0) { model.set(GRB_IntParam_MIPFocus, focus); }
    void setMIPGap() { model.set(GRB_DoubleAttr_MIPGap, 0.05); }
    void setImproveStartGap() { model.set(GRB_DoubleParam_ImproveStartGap, 0.05); }
    void setCallback(GRBCallback *callback) {
        model.set(GRB_IntParam_LazyConstraints, 1);
        model.setCallback(callback);
    }

    void setPostprocess(OnSolutionFound p) {
        postprocess = p;
    }

protected:
    void setOptimaOrientation(OptimaOrientation optimaOrientation = DefaultObjectiveOptimaOrientation) {
        model.set(GRB_IntAttr_ModelSense, optimaOrientation);
    }

    // determines which alternative sln will be retrieved when calling var.get(GRB_DoubleAttr_Xn).
    void setAltSolutionIndex(int solutionIndex) { model.set(GRB_IntParam_SolutionNumber, solutionIndex); }

    void setPoolingMode(PoolingMode poolingMode) { model.set(GRB_IntParam_PoolSearchMode, poolingMode); }
    void setMaxSolutionPoolSize(int maxSolutionNum) { model.set(GRB_IntParam_PoolSolutions, maxSolutionNum); }
    void setMaxSolutionRelPoolGap(double maxRelPoolGap) { model.set(GRB_DoubleParam_PoolGap, maxRelPoolGap); }

    bool isConstant(const LinearExpr &expr) { return (expr.size() == 0); }
    void updateModel() { model.update(); }

    ResultStatus solve() {
        try {
            model.optimize();
            updateStatus();
        } catch (GRBException &e) {
            if (e.getErrorCode() == GRB_ERROR_OUT_OF_MEMORY) {
                status = ResultStatus::OutOfMemory;
            } else {
                status = ResultStatus::Error;
            }
        }
        return status;
    }

    void updateStatus();
    #pragma endregion Method

    #pragma region Field
public:
protected:
    static GRBEnv globalEnv;

    // definition of the problem to solve.
    GRBModel model;

    // status for the most recent optimization.
    ResultStatus status;
    OnSolutionFound postprocess;
    #pragma endregion Field
}; // MpSolverGurobi


#endif SMART_QYM_MPSOLVER_GUROBI_H
