#include "Solver.h"

#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <mutex>

#include <cmath>


using namespace std;


namespace szx {

#pragma region Solver::Cli
int Solver::Cli::run(int argc, char * argv[]) {
    Log(LogSwitch::Szx::Cli) << "parse command line arguments." << endl;
    Set<String> switchSet;
    Map<String, char*> optionMap({ // use string as key to compare string contents instead of pointers.
        { InstancePathOption(), nullptr },
        { SolutionPathOption(), nullptr },
        { RandSeedOption(), nullptr },
        { TimeoutOption(), nullptr },
        { MaxIterOption(), nullptr },
        { JobNumOption(), nullptr },
        { RunIdOption(), nullptr },
        { EnvironmentPathOption(), nullptr },
        { ConfigPathOption(), nullptr },
        { LogPathOption(), nullptr }
    });

    for (int i = 1; i < argc; ++i) { // skip executable name.
        auto mapIter = optionMap.find(argv[i]);
        if (mapIter != optionMap.end()) { // option argument.
            mapIter->second = argv[++i];
        } else { // switch argument.
            switchSet.insert(argv[i]);
        }
    }

    Log(LogSwitch::Szx::Cli) << "execute commands." << endl;
    if (switchSet.find(HelpSwitch()) != switchSet.end()) {
        cout << HelpInfo() << endl;
    }

    if (switchSet.find(AuthorNameSwitch()) != switchSet.end()) {
        cout << AuthorName() << endl;
    }

    Solver::Environment env;
    env.load(optionMap);
    if (env.instPath.empty() || env.slnPath.empty()) { return -1; }

    Solver::Configuration cfg;
    cfg.load(env.cfgPath);

    Log(LogSwitch::Szx::Input) << "load instance " << env.instPath << " (seed=" << env.randSeed << ")." << endl;
    Problem::Input input;
    if (!input.load(env.instPath)) { return -1; }

    Solver solver(input, env, cfg);
    solver.solve();

    pb::Submission submission;
    submission.set_thread(to_string(env.jobNum));
    submission.set_instance(env.friendlyInstName());
    submission.set_duration(to_string(solver.timer.elapsedSeconds()) + "s");
    submission.set_obj(solver.output.totalCost);

    solver.output.save(env.slnPath, submission);
    #if QYM_DEBUG
    solver.output.save(env.solutionPathWithTime(), submission);
    solver.record();
    #endif // QYM_DEBUG

    return 0;
}
#pragma endregion Solver::Cli

#pragma region Solver::Environment
void Solver::Environment::load(const Map<String, char*> &optionMap) {
    char *str;

    str = optionMap.at(Cli::EnvironmentPathOption());
    if (str != nullptr) { loadWithoutCalibrate(str); }

    str = optionMap.at(Cli::InstancePathOption());
    if (str != nullptr) { instPath = str; }

    str = optionMap.at(Cli::SolutionPathOption());
    if (str != nullptr) { slnPath = str; }

    str = optionMap.at(Cli::RandSeedOption());
    if (str != nullptr) { randSeed = atoi(str); }

    str = optionMap.at(Cli::TimeoutOption());
    if (str != nullptr) { msTimeout = static_cast<Duration>(atof(str) * Timer::MillisecondsPerSecond); }

    str = optionMap.at(Cli::MaxIterOption());
    if (str != nullptr) { maxIter = atoi(str); }

    str = optionMap.at(Cli::JobNumOption());
    if (str != nullptr) { jobNum = atoi(str); }

    str = optionMap.at(Cli::RunIdOption());
    if (str != nullptr) { rid = str; }

    str = optionMap.at(Cli::ConfigPathOption());
    if (str != nullptr) { cfgPath = str; }

    str = optionMap.at(Cli::LogPathOption());
    if (str != nullptr) { logPath = str; }

    calibrate();
}

void Solver::Environment::load(const String &filePath) {
    loadWithoutCalibrate(filePath);
    calibrate();
}

void Solver::Environment::loadWithoutCalibrate(const String &filePath) {
    // EXTEND[qym][8]: load environment from file.
    // EXTEND[qym][8]: check file existence first.
}

void Solver::Environment::save(const String &filePath) const {
    // EXTEND[qym][8]: save environment to file.
}
void Solver::Environment::calibrate() {
    // adjust thread number.
    int threadNum = thread::hardware_concurrency();
    if ((jobNum <= 0) || (jobNum > threadNum)) { jobNum = threadNum; }

    // adjust timeout.
    msTimeout -= Environment::SaveSolutionTimeInMillisecond;
}
#pragma endregion Solver::Environment

#pragma region Solver::Configuration
void Solver::Configuration::load(const String &filePath) {
    // EXTEND[szx][5]: load configuration from file.
    // EXTEND[szx][8]: check file existence first.
}

void Solver::Configuration::save(const String &filePath) const {
    // EXTEND[szx][5]: save configuration to file.
}
#pragma endregion Solver::Configuration

#pragma region Solver
bool Solver::solve() {
    init();

    int workerNum = 1;//(max)(1, env.jobNum / cfg.threadNumPerWorker);
    cfg.threadNumPerWorker = 4;//env.jobNum / workerNum;
    List<Solution> solutions(workerNum, Solution(this));
    List<bool> success(workerNum);

    Log(LogSwitch::Szx::Framework) << "launch " << workerNum << " workers." << endl;
    List<thread> threadList;
    threadList.reserve(workerNum);
    for (int i = 0; i < workerNum; ++i) {
        // TODO[szx][2]: as *this is captured by ref, the solver should support concurrency itself, i.e., data members should be read-only or independent for each worker.
        // OPTIMIZE[szx][3]: add a list to specify a series of algorithm to be used by each threads in sequence.
        threadList.emplace_back([&, i]() { success[i] = optimize(solutions[i], i); });
    }
    for (int i = 0; i < workerNum; ++i) { threadList.at(i).join(); }

    Log(LogSwitch::Szx::Framework) << "collect best result among all workers." << endl;
    int bestIndex = -1;
    double bestValue = 0;
    for (int i = 0; i < workerNum; ++i) {
        if (!success[i]) { continue; }
        Log(LogSwitch::Szx::Framework) << "worker " << i << " got " << solutions[i].totalCost << endl;
        if (solutions[i].totalCost <= bestValue) { continue; }
        bestIndex = i;
        bestValue = solutions[i].totalCost;
    }

    env.rid = to_string(bestIndex);
    if (bestIndex < 0) { return false; }
    output = solutions[bestIndex];
    return true;
}

void Solver::record() const {
    #if QYM_DEBUG
    int generation = 0;

    ostringstream log;

    System::MemoryUsage mu = System::peakMemoryUsage();

    double obj = output.totalCost;
    double checkerObj = -1;
    bool feasible = check(checkerObj);

    // record basic information.
    log << env.friendlyLocalTime() << ","
        << env.rid << ","
        << env.instPath << ","
        << feasible << "," << (obj - checkerObj) << ","
        << output.totalCost << ","
        << input.bestobj() << ","
        << input.referenceobj() << ","
        << timer.elapsedSeconds() << ","
        << input.referencetime() << ","
        << mu.physicalMemory << "," << mu.virtualMemory << ","
        << env.randSeed << ","
        << cfg.toBriefStr() << ","
        << generation << "," << iteration;

    // record solution vector.
    // EXTEND[qym][2]: save solution in log.
    log << endl;

    // append all text atomically.
    static mutex logFileMutex;
    lock_guard<mutex> logFileGuard(logFileMutex);

    ofstream logFile(env.logPath, ios::app);
    logFile.seekp(0, ios::end);
    if (logFile.tellp() <= 0) {
        logFile << "Time,ID,Instance,Feasible,ObjMatch,Cost,MinCost,RefCost,Duration,RefDuration,PhysMem,VirtMem,RandSeed,Config,Generation,Iteration,Solution" << endl;
    }
    logFile << log.str();
    logFile.close();
    #endif // QYM_DEBUG
}

bool Solver::check(double &checkerObj) const {
    #if QYM_DEBUG
    enum CheckerFlag {
        IoError = 0x0,
        FormatError = 0x1,
        SubtourExistenceError = 0x2,
        MultipleVisitError = 0x4,
        LoadDeliveryError = 0x8,
        QuantityReasonabilityError = 0x16
    };

    checkerObj = System::exec("Checker.exe " + env.instPath + " " + env.solutionPathWithTime());
    if (checkerObj > 0) {
        checkerObj = (double)checkerObj / 1000;
        return true;
    }
    int errorCode = (int)checkerObj / 1000;
    errorCode = ~errorCode;
    if (errorCode == CheckerFlag::IoError) { Log(LogSwitch::Checker) << "IoError." << endl; }
    if (errorCode & CheckerFlag::FormatError) { Log(LogSwitch::Checker) << "FormatError." << endl; }
    if (errorCode & CheckerFlag::SubtourExistenceError) { Log(LogSwitch::Checker) << "SubtourExistenceError." << endl; }
    if (errorCode & CheckerFlag::MultipleVisitError) { Log(LogSwitch::Checker) << "MultipleVisitError." << endl; }
    if (errorCode & CheckerFlag::LoadDeliveryError) { Log(LogSwitch::Checker) << "LoadDeliveryError." << endl; }
    if (errorCode & CheckerFlag::QuantityReasonabilityError) { Log(LogSwitch::Checker) << "QuantityReasonabilityError." << endl; }
    checkerObj = errorCode;
    return false;
    #else
    checkerObj = 0;
    return true;
    #endif // QYM_DEBUG
}

void Solver::init() {
    aux.routingCost.resize(input.nodes_size(), List<double>(input.nodes_size(), 0));
    for (auto i = input.nodes().begin(); i != input.nodes().end(); ++i) {
        for (auto j = input.nodes().begin(); j != input.nodes().end(); ++j) {
            if (i->id() == j->id()) { continue; }
            double value = 0;
            value = sqrt(pow(i->x() - j->x(), 2) + pow(i->y() - j->y(), 2));
            value = round(value);
            aux.routingCost[i->id()][j->id()] = value;
        }
    }
}

bool Solver::optimize(Solution &sln, ID workerId) {
    Log(LogSwitch::Szx::Framework) << "worker " << workerId << " starts." << endl;

    ID nodeNum = input.nodes_size();
    ID vehicleNum = input.vehicles_size();
    const auto &vehicles(*input.mutable_vehicles());
    const auto &nodes(*input.mutable_nodes());

    // reset solution state.
    bool status = true;

    sln.totalCost = 0;

    // TODO[0]: replace the following random assignment with your own algorithm.
    solveWithCompleteModel(sln, true);
    //solveWithRelaxedInit(sln);
    //solveWithDecomposition(sln);

    Log(LogSwitch::Szx::Framework) << "worker " << workerId << " ends." << endl;
    return status;
}
bool Solver::solveWithCompleteModel(Solution & sln, bool findFeasibleFirst) {
    IrpModelSolver modelSolver;
    if (findFeasibleFirst) { modelSolver.setFindFeasiblePreference(); }

    //IrpModelSolver::Input &modelInput(modelSolver.input);
    //modelInput.nodeNum = input.nodes_size();
    //modelInput.periodNum = input.periodnum();
    //modelInput.vehicleNum = input.vehicles_size();
    //modelInput.bestObjective = input.bestobj();
    //modelInput.referenceObjective = input.referenceobj();

    //modelInput.nodes.resize(modelInput.nodeNum);
    //for (auto i = input.nodes().begin(); i != input.nodes().end(); ++i) {
    //    //modelInput.nodes[i->id()].xPoint = i->x();
    //    //modelInput.nodes[i->id()].yPoint = i->y();
    //    modelInput.nodes[i->id()].initialQuantity = i->initquantity();
    //    modelInput.nodes[i->id()].capacity = i->capacity();
    //    modelInput.nodes[i->id()].minLevel = i->minlevel();
    //    modelInput.nodes[i->id()].unitDemand = i->unitdemand();
    //    modelInput.nodes[i->id()].holdingCost = i->holidingcost();
    //}
    //modelInput.nodes[0].unitDemand = -modelInput.nodes[0].unitDemand;
    //modelInput.vehicleCapacity = input.vehicles()[0].capacity();
    convertToModelInput(modelSolver.input, input);
    modelSolver.routingCost = aux.routingCost;

    if (!modelSolver.solve()) { return false; }

    // record solution.
    sln.totalCost = modelSolver.getObjValue();
    auto &allRoutes(*sln.mutable_allroutes());
    const IrpModelSolver::PresetX &presetX(modelSolver.presetX);
    for (ID p = 0; p < input.periodnum(); ++p) {
        auto &routeInPeriod(*allRoutes.Add());
        for (ID v = 0; v != input.vehicles_size(); ++v) {
            auto &route(*routeInPeriod.add_routes());
            // TODO[qym][5]:
            if (!presetX.xEdge[v][p].empty()) {
                ID i = 0;
                for (ID j = 0; j < input.nodes_size(); ++j) {
                    if (i == j) { continue; }
                    if (!presetX.xEdge[v][p][i][j]) { continue; }
                    auto &delivery(*route.add_deliveries());
                    delivery.set_node(j);
                    delivery.set_quantity(lround(presetX.xQuantity[v][p][j]));

                    if (j == 0) {
                        delivery.set_quantity(-lround(presetX.xQuantity[v][p][j]));
                        break;
                    }
                    i = j;
                    j = -1;
                }
            }
        }
    }

    return true;
}

bool Solver::solveWithRelaxedInit(Solution & sln, bool findFeasibleFirst) {
    const int ReduceModelScaleIteration = 3;
    const int RandModelScaleIteration = 6;
    const int DefaultMoveCount = 2 - (rand() % 2);
    const int DefaultTimeLimitPerIteration = 600;
    const int DefaultTimeLimitSecond = IrpModelSolver::DefaultTimeLimitSecond;

    vector<double> costForPeriod(input.periodnum(), 0);
    
    // generate initial solution using slack constraints for customer's min level.
    IrpModelSolver initSolver;
    convertToModelInput(initSolver.input, input);
    initSolver.routingCost = aux.routingCost;
    initSolver.enableRelaxMinLevel();
    
    if (!initSolver.solve()) { return false; }
    double bestObj = initSolver.getObjValue();
    costForPeriod = initSolver.sln.costForPeriod;

    // initialize tabu table.
    vector<int> tabuTable(input.periodnum(), -1);
    for (int i = (input.periodnum() + 1) / 2; i < input.periodnum(); ++i) {
        tabuTable[i] = tabuTable[i - 1] + 1;
    }

    int iter = 0;
    int iterNoImprove = 0;
    int timeLimitPerIteration = DefaultTimeLimitPerIteration;
    int moveCount = DefaultMoveCount;  // select moveCount periods to reoptimize per iteration.
    double secondsUntilOpt = initSolver.getDurationInSecond(); // elapsed seconds when the best solution was found.
    double totalSeconds = secondsUntilOpt; // total elapsed seconds.

    IrpModelSolver::PresetX presetX(move(initSolver.presetX));
    // reoptimize with initial solution
    while (totalSeconds < DefaultTimeLimitSecond && abs(bestObj - input.referenceobj()) > IrpModelSolver::DefaultDoubleGap) {
        // update 
        if (iterNoImprove > RandModelScaleIteration) {
            timeLimitPerIteration = min(timeLimitPerIteration * 1.2, DefaultTimeLimitSecond - totalSeconds);
            moveCount = (rand() % input.periodnum()) + 1;
        } else if (iterNoImprove > ReduceModelScaleIteration) { // shrink neighborhood in case it is too complex.
            timeLimitPerIteration = min(timeLimitPerIteration * 1.2, DefaultTimeLimitSecond - totalSeconds);
            moveCount = makeSureInRange(static_cast<int>(rand() % max(input.periodnum() / 3, 1) + 1), 1, moveCount);
        } else if (iterNoImprove > 0) { // expand neighborhood in case stagnating in local optima.
            timeLimitPerIteration = min(timeLimitPerIteration * 1.5, DefaultTimeLimitSecond - totalSeconds);
            moveCount = makeSureInRange(++moveCount, 1, input.periodnum() * 2 / 3);
        } else {
            timeLimitPerIteration = min(DefaultTimeLimitPerIteration, DefaultTimeLimitSecond - static_cast<int>(totalSeconds));
        }

        //IrpModelSolver::saveSolution(initSolver.input, presetX, "../Solution/" + dir + "." + instance.substr(0, instance.find_first_of('.')) + "." + getTightLocalTime() + ".csv");
        IrpModelSolver solver(initSolver.input);
        solver.enablePresetSolution();
        solver.presetX = presetX;
        solver.setTimeLimitInSecond(timeLimitPerIteration);
        if (!getFixedPeriods(input.periodnum(), solver.presetX.isPeriodFixed, iter, tabuTable, moveCount)) { break; }

        //cout << "\nFix: ";
        //double fixedCost = 0;
        //for (int i = 0; i < input.periodnum(); ++i) {
        //    if (!solver.presetX.isPeriodFixed[i]) { continue; }
        //    cout << i << " ";
        //    fixedCost += costForPeriod[i];
        //    solver.fixPeriod(i);
        //}
        //cout << "\n" << endl;

        if (findFeasibleFirst) { solver.setFindFeasiblePreference(); }
        bool feasibleFound = solver.solve();
        totalSeconds += solver.getDurationInSecond();
        ++iter;
        ++iterNoImprove;
        if (!feasibleFound) { continue; }
        if (bestObj - solver.getCostInPeriod(0, input.periodnum()) < IrpModelSolver::DefaultDoubleGap) { continue; }

        iterNoImprove = 0;
        secondsUntilOpt = totalSeconds;
        bestObj = solver.getCostInPeriod(0, input.periodnum());
        cout << "Best: " << bestObj << endl;

        costForPeriod = solver.sln.costForPeriod;
        presetX = move(solver.presetX);
    }

    return true;
}

bool Solver::solveWithDecomposition(Solution & sln, bool findFeasibleFirst) {
    return true;
}

void Solver::convertToModelInput(IrpModelSolver::Input & model, const Problem::Input & problem) {
    model.nodeNum = problem.nodes_size();
    model.periodNum = problem.periodnum();
    model.vehicleNum = problem.vehicles_size();
    model.bestObjective = problem.bestobj();
    model.referenceObjective = problem.referenceobj();

    model.nodes.resize(model.nodeNum);
    for (auto i = problem.nodes().begin(); i != problem.nodes().end(); ++i) {
        //model.nodes[i->id()].xPoint = i->x();
        //model.nodes[i->id()].yPoint = i->y();
        model.nodes[i->id()].initialQuantity = i->initquantity();
        model.nodes[i->id()].capacity = i->capacity();
        model.nodes[i->id()].minLevel = i->minlevel();
        model.nodes[i->id()].unitDemand = i->unitdemand();
        model.nodes[i->id()].holdingCost = i->holidingcost();
    }
    model.nodes[0].unitDemand = -model.nodes[0].unitDemand;
    model.vehicleCapacity = problem.vehicles()[0].capacity();
}
#pragma endregion Solver

}
