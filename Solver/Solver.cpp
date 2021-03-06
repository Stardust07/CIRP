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
    log.precision(2);
    log.setf(std::ios::fixed);
    System::MemoryUsage mu = System::peakMemoryUsage();

    double obj = output.totalCost;
    double checkerObj = -1;
    bool feasible = check(checkerObj);

    // record basic information.
    log << env.friendlyLocalTime() << ","
        << env.rid << ","
        << env.friendlyInstName() << ","
        << feasible << "," << lround(obj - checkerObj) << ","
        << obj << ","
        << input.bestobj() << ","
        << input.referenceobj() << ","
        << timer.elapsedSeconds() << ","
        << input.referencetime() << ","
        << mu.physicalMemory << "," << mu.virtualMemory << ","
        << env.randSeed << ","
        << cfg.toBriefStr() << ","
        << generation << "," << iteration
        << "," << checkerObj << "," << output.routingcost() << "," << output.holdingcost() << "";

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
    //if (errorCode & CheckerFlag::FormatError) { Log(LogSwitch::Checker) << "FormatError." << endl; }
    //if (errorCode & CheckerFlag::SubtourExistenceError) { Log(LogSwitch::Checker) << "SubtourExistenceError." << endl; }
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
            value = hypot(i->x() - j->x(), i->y() - j->y());
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
    switch (cfg.alg) {
    case Configuration::Algorithm::RelaxedInventory:solveWithInventoryRelaxed(sln); break;
    case Configuration::Algorithm::Decomposition:solveWithDecomposition(sln); break;
    case Configuration::Algorithm::Analysis:analyzeSolution(); break;
    case Configuration::Algorithm::RelaxedTsp:solveWithTspRelaxed(sln); break;
    case Configuration::Algorithm::IterativeModel:solveWithIterativeModel(sln); break;
    case Configuration::Algorithm::Test:solveWithTest(sln); break;
    case Configuration::Algorithm::CompleteModel:
    default:
        solveWithCompleteModel(sln);
        //solveWithCompleteModel(sln, true);
        break;
    }

    Log(LogSwitch::Szx::Framework) << "worker " << workerId << " ends." << endl;
    return status;
}
bool Solver::solveWithCompleteModel(Solution & sln, bool findFeasibleFirst) {
    IrpModelSolver modelSolver;
    if (findFeasibleFirst) { modelSolver.setFindFeasiblePreference(); }

    convertToModelInput(modelSolver.input, input);
    modelSolver.routingCost = aux.routingCost;

    if (!modelSolver.solve()) { return false; }

    // record solution.
    sln.totalCost = modelSolver.getTotalCostInPeriod(0, input.periodnum());
    //ofstream ofs("lowerbound.csv", ios::app);
    //ofs.seekp(0, ios::end);
    //ofs << sln.totalCost << endl;
    //ofs.close();

    retrieveOutputFromModel(sln, modelSolver.presetX);

    return true;
}

bool Solver::solveWithTspRelaxed(Solution & sln, bool findFeasibleFirst) {
    IrpModelSolver modelSolver;
    if (findFeasibleFirst) { modelSolver.setFindFeasiblePreference(); }

    convertToModelInput(modelSolver.input, input);
    modelSolver.routingCost = aux.routingCost;
    modelSolver.relaxTspSubtourConstraint();
    if (!modelSolver.solve()) { return false; }
    
    IrpModelSolver::PresetX &presetX(modelSolver.presetX);
    double holdingObj = modelSolver.getHoldingCostInPeriod(0, input.periodnum());
    double elapsedSeconds = modelSolver.getDurationInSecond();
    double routingObj = 0;

    // initilize routing in each period as tsp.
    if (!solveVrp(presetX, routingObj, elapsedSeconds, modelSolver.input.nodes)) { return false; }
    cout << (holdingObj + routingObj) << "=" << routingObj << "(R) + " << holdingObj << "(H)\n";

    // recordSolution(originInput, presetX);
    // writePathToCsv(presetX);
    
    sln.totalCost = holdingObj + routingObj;
    retrieveOutputFromModel(sln, presetX);
    return true;

    // TODO[qym][9]: to delete 
    // count visited times.
    List<int> nodeVisitedTimes(input.nodes_size(), 0);
    List<int> periodVisitedTimes(input.periodnum(), 0);
    List<int> deliveryQuantity(input.nodes_size(), 0);
    for (ID t = 0; t < input.periodnum(); ++t) {
        for (ID i = 0; i < input.nodes_size(); ++i) {
            if (presetX.xQuantity[0][t][i] > IrpModelSolver::DefaultDoubleGap) {
                ++nodeVisitedTimes[i];
                ++periodVisitedTimes[t];
            }
            deliveryQuantity[i] += lround(presetX.xQuantity[0][t][i]);
        }
    }
    // sort periods by visited times descending order
    List<int> periodOrder(input.periodnum());
    for (ID t = 0; t < input.periodnum(); ++t) {
        periodOrder[t] = t;
    }
    sort(periodOrder.begin(), periodOrder.end(), [&](int l, int r) {
        return (periodVisitedTimes[l] > periodVisitedTimes[r]);
    });

    // local search
    struct NodeStatus {
        ID period;
        ID preNode;
        ID postNode;
        ID quantity;

        NodeStatus() :period(-1), preNode(-1), postNode(-1), quantity(0) {}
        NodeStatus(ID t, ID pre, ID post, ID q) :period(t), preNode(pre), postNode(post), quantity(q) {}
    };
    // 交换相邻两个周期的配送
    // 原则：均衡每个周期访问次数
    double bestObj = sln.totalCost;
    double bestHolding = holdingObj;
    IrpModelSolver::PresetX bestSln(presetX);
    int iter = 0;
    while (iter < 100000) {
        double minIncrement = INT_MAX;
        double holdingIncrement, routingIncrement;
        ID opVehicle = -1;
        ID opNode = -1;
        List<NodeStatus> moves;
        bool feasibleMoveFound = false;

        sort(periodOrder.begin(), periodOrder.end(), [&](int l, int r) {
            return (periodVisitedTimes[l] > periodVisitedTimes[r]);
        });
        for (ID v = 0; v < input.vehicles_size(); ++v) {
            for (auto t = periodOrder.begin(); t != periodOrder.end() && !feasibleMoveFound; ++t) {
                if (periodVisitedTimes[*t] <= 0) { continue; }
                ID candidate = -1;
                if ((*t - 1 >= 0) && (periodVisitedTimes[*t - 1] > 0) && (periodVisitedTimes[*t - 1] < periodVisitedTimes[*t])) {
                    candidate = *t - 1;
                } else if ((*t + 1 < input.periodnum()) && (periodVisitedTimes[*t + 1] > 0) && (periodVisitedTimes[*t + 1] < periodVisitedTimes[*t])) {
                    candidate = *t + 1;
                }
                if (candidate < 0) { continue; }
                // quantity[t]>0 && quantity[t-1]==0时交换
                for (ID i = 1; i < input.nodes_size(); ++i) {
                    if ((presetX.xQuantity[v][*t][i] <= IrpModelSolver::DefaultDoubleGap)
                        || (presetX.xQuantity[v][candidate][i] > IrpModelSolver::DefaultDoubleGap)) {
                        continue;
                    }
                    // swap two adjacent periods.
                    bool valid = true;
                    // check customer.
                    int restQuantity = input.nodes()[i].initquantity();
                    for (ID p = 0; p < input.periodnum(); ++p) {
                        restQuantity += ((p != candidate) && (p != *t)) ? lround(presetX.xQuantity[v][p][i])
                            : ((p == *t) ? lround(presetX.xQuantity[v][candidate][i]) : lround(presetX.xQuantity[v][*t][i]));
                        if ((restQuantity > input.nodes()[i].capacity()) || (restQuantity - input.nodes()[i].unitdemand() < 0)) {
                            valid = false;
                            break;
                        }
                        restQuantity -= input.nodes()[i].unitdemand();
                    }
                    // check supplier.
                    restQuantity = input.nodes()[0].initquantity();
                    for (ID p = 0; p < input.periodnum(); ++p) {
                        restQuantity -= lround(presetX.xQuantity[v][p][0]);
                        if (p == *t) {
                            restQuantity += lround(presetX.xQuantity[v][*t][i]);
                        } else if (p == candidate) {
                            restQuantity -= lround(presetX.xQuantity[v][*t][i]);
                        }
                        if ((restQuantity < 0) 
                            || ((p == candidate) && (lround(presetX.xQuantity[v][p][0]) + lround(presetX.xQuantity[v][*t][i]) > input.vehicles()[v].capacity()))) {
                            valid = false;
                            break;
                        }
                        restQuantity -= input.nodes()[0].unitdemand();
                    }
                    if (!valid) { continue; }

                    feasibleMoveFound = true;
                    // calculate increment of cost.
                    double hc = (input.nodes()[i].holdingcost() - input.nodes()[0].holdingcost()) * lround(presetX.xQuantity[v][*t][i]) * (*t - candidate);
                    double rc = 0;
                    NodeStatus rm, ins;
                    if ((presetX.xQuantity[v][*t][i] > IrpModelSolver::DefaultDoubleGap) 
                        && (presetX.xQuantity[v][candidate][i] <= IrpModelSolver::DefaultDoubleGap)) {
                        // remove node.
                        for (ID j = 0; j < input.nodes_size(); ++j) {
                            if (i == j) { continue; }
                            if (presetX.xEdge[v][*t][i][j]) {
                                rm.postNode = j;
                            }
                            if (presetX.xEdge[v][*t][j][i]) {
                                rm.preNode = j;
                            }
                            if ((rm.preNode >= 0) && (rm.postNode >= 0)) { break; }
                        }
                        rm.period = *t;
                        rm.quantity = 0;
                        rc += aux.routingCost[rm.preNode][rm.postNode] - aux.routingCost[rm.preNode][i] - aux.routingCost[i][rm.postNode];
                        
                        // insert node.
                        int minIncreasedCost = INT_MAX;
                        int slnCount = 0;
                        for (ID j = 0; j < input.nodes_size(); ++j) {
                            for (ID k = 0; k < input.nodes_size(); ++k) {
                                if (k == j) { continue; }
                                if (!presetX.xEdge[v][candidate][j][k]) { continue; }
                                int cost = aux.routingCost[j][i] + aux.routingCost[i][k] - aux.routingCost[j][k];
                                if (cost < minIncreasedCost) {
                                    minIncreasedCost = cost;
                                    ins.preNode = j;
                                    ins.postNode = k;
                                    slnCount = 1;
                                } else if (cost == minIncreasedCost) {
                                    ++slnCount;
                                    if (rand() % slnCount < 1) {
                                        // select randomly when solution with the same delta.
                                        ins.preNode = j;
                                        ins.postNode = k;
                                    }
                                }
                            }
                        }
                        if ((ins.preNode < 0) && (ins.postNode < 0)) {
                            ins.preNode = 0;
                            ins.postNode = 0;
                            minIncreasedCost = aux.routingCost[0][i] + aux.routingCost[i][0];
                            cout << "***Warning: " << candidate << "," << *t << endl;
                        }
                        ins.period = candidate;
                        ins.quantity = lround(presetX.xQuantity[v][*t][i]);
                        rc += minIncreasedCost;
                    }
                    //cout << "delta: " << hc << "(H)\t" << rc << "(R)\n";
                    if (hc + rc < minIncrement) {
                        minIncrement = hc + rc;
                        holdingIncrement = hc;
                        routingIncrement = rc;
                        opVehicle = v;
                        opNode = i;
                        moves.clear();
                        moves.push_back(rm);
                        moves.push_back(ins);
                    }
                }
            }
        }
        if (!feasibleMoveFound) { break; }

        if (opNode < 0) { break; }

        // make move.
        cout << "Node " << opNode << ":\t";
        for (auto mv = moves.begin(); mv != moves.end(); ++mv) {
            if (mv->period < 0) { continue; }
            presetX.xQuantity[opVehicle][mv->period][0] -= (lround(presetX.xQuantity[opVehicle][mv->period][opNode]) - mv->quantity);
            presetX.xQuantity[opVehicle][mv->period][opNode] = mv->quantity;

            if ((mv->preNode < 0) || (mv->postNode < 0)) { continue; }
            periodVisitedTimes[mv->period] += (mv->quantity > 0) ? 1 : -1;

            presetX.xEdge[opVehicle][mv->period][mv->preNode][opNode] = (mv->quantity > 0);
            presetX.xEdge[opVehicle][mv->period][opNode][mv->postNode] = (mv->quantity > 0);
            presetX.xEdge[opVehicle][mv->period][mv->preNode][mv->postNode] = !(mv->quantity > 0);
            if (mv->quantity > 0) {
                cout << "Insert " << mv->period << "\t";
            } else {
                cout << "Remove " << mv->period << "\t";
            }
        }
        // TODO[qym][5]: optimize routing with tsp.

        holdingObj += holdingIncrement;
        routingObj += routingIncrement;
        sln.totalCost += minIncrement;
        if (sln.totalCost < bestObj) {
            bestSln = presetX;
            bestObj = sln.totalCost;
            bestHolding = holdingObj;
        }
        cout << sln.totalCost << "\t" << bestObj << endl;
        ++iter;
    }
    presetX = bestSln;
    sln.totalCost = bestObj;
    // 重新规划一个客户的所有周期配送方案
    while (false) {
         int periodNum = input.periodnum();
        double decreasedCost = -INT_MAX;
        ID opVehicle = -1;
        ID opNode = -1;
        List<NodeStatus> opDelivery;
        for (ID i = 1; i < input.nodes_size(); ++i) {
            int leastDeliveryNum = ceil(
                double(input.nodes()[i].unitdemand() * periodNum - input.nodes()[i].initquantity()) / input.nodes()[i].capacity());
            if (nodeVisitedTimes[i] <= leastDeliveryNum) { continue; }
            for (ID v = 0; v < input.vehicles_size(); ++v) {
                //if (input.nodes()[i].holdingcost() > input.nodes()[0].holdingcost()) {
                //    // 推迟送货
                //}
                // reassign delivery at node i
                bool valid = true;
                List<NodeStatus> newSln(periodNum);
                int restQuantity = input.nodes()[i].initquantity();
                for (ID t = 0; t < periodNum; ++t) {
                    restQuantity -= input.nodes()[i].unitdemand();
                    if (restQuantity >= 0) {
                        newSln[t].quantity = 0;
                    } else {
                        //newSln[t].quantity = min(input.vehicles()[v].capacity() - lround(presetX.xQuantity[v][t][0]), (long)input.nodes()[i].capacity());
                        newSln[t].quantity = input.nodes()[i].capacity();
                        restQuantity += newSln[t].quantity;
                        if (restQuantity < 0) {
                            valid = false;
                            break;
                        }
                    }
                }
                // check vehicle and supplier.
                if (!valid) { continue; }

                double delta = 0;
                for (ID t = 0; t < periodNum; ++t) {
                    int decreasedQuantity = lround(presetX.xQuantity[v][t][i]) - newSln[t].quantity;
                    delta += (input.nodes()[i].holdingcost() - input.nodes()[0].holdingcost()) * decreasedQuantity * (input.periodnum() - t);
                    if (((presetX.xQuantity[v][t][i] < IrpModelSolver::DefaultDoubleGap) && (newSln[t].quantity == 0))
                        || ((presetX.xQuantity[v][t][i] > IrpModelSolver::DefaultDoubleGap) && (newSln[t].quantity > 0))) {
                        continue;
                    }
                    ID pre = -1, post = -1;
                    if (((presetX.xQuantity[v][t][i] > IrpModelSolver::DefaultDoubleGap) && (newSln[t].quantity == 0))) {
                        // remove node.
                        for (ID j = 0; j < input.nodes_size(); ++j) {
                            if (i == j) { continue; }
                            if (presetX.xEdge[v][t][i][j]) {
                                newSln[t].postNode = j;
                            }
                            if (presetX.xEdge[v][t][j][i]) {
                                newSln[t].preNode = j;
                            }
                            if ((pre >= 0) && (post >= 0)) { break; }
                        }
                        delta += aux.routingCost[newSln[t].preNode][i] + aux.routingCost[i][newSln[t].postNode] - aux.routingCost[newSln[t].preNode][newSln[t].postNode];
                    } else {
                        // insert node.
                        int minCost = INT_MAX;
                        int slnCount = 0;
                        for (ID j = 0; j < input.nodes_size(); ++j) {
                            for (ID k = 0; k < input.nodes_size(); ++k) {
                                if (k == j) { continue; }
                                if (!presetX.xEdge[v][t][j][k]) { continue; }
                                int cost = aux.routingCost[j][i] + aux.routingCost[i][k] - aux.routingCost[j][k];
                                if (cost < minCost) {
                                    minCost = cost;
                                    newSln[t].preNode = j;
                                    newSln[t].postNode = k;
                                    slnCount = 1;
                                } else if (cost == minCost) {
                                    ++slnCount;
                                    if (rand() % slnCount < 1) {
                                        // select randomly when solution with the same delta.
                                        newSln[t].preNode = j;
                                        newSln[t].postNode = k;
                                    }
                                }
                            }
                        }
                        if ((newSln[t].preNode < 0) && (newSln[t].postNode < 0)) {
                            newSln[t].preNode = 0;
                            newSln[t].postNode = 0;
                            minCost = aux.routingCost[0][i] + aux.routingCost[i][0];
                        }
                        delta -= minCost;
                    }
                }
                cout << "delta: " << delta << endl;
                if (delta > decreasedCost) {
                    decreasedCost = delta;
                    opVehicle = v;
                    opNode = i;
                    opDelivery = newSln;
                }
            }
        }

        cout << opNode << "," << decreasedCost << endl;
        if (opNode < 0) {
            break;
        }

        nodeVisitedTimes[opNode] = 0;
        for (ID t = 0; t < periodNum; ++t) {
            int deQuantity = lround(presetX.xQuantity[opVehicle][t][opNode]) - opDelivery[t].quantity;
            presetX.xQuantity[opVehicle][t][0] -= deQuantity;
            presetX.xQuantity[opVehicle][t][opNode] = opDelivery[t].quantity;
            
            if ((opDelivery[t].preNode < 0) || (opDelivery[t].postNode < 0)) { continue; }
            if (deQuantity == 0) { continue; }
            if (deQuantity > 0) {
                // remove node.
            }
                                    
            nodeVisitedTimes[opNode] += (opDelivery[t].quantity > 0);
            presetX.xEdge[opVehicle][t][opDelivery[t].preNode][opNode] = (deQuantity < 0);
            presetX.xEdge[opVehicle][t][opNode][opDelivery[t].postNode] = (deQuantity < 0);
            presetX.xEdge[opVehicle][t][opDelivery[t].preNode][opDelivery[t].postNode] = (deQuantity > 0);
        }

        sln.totalCost -= decreasedCost;
    }
    cout << "total cost = " << sln.totalCost << endl;

    while (false) {
        double decreasedCost = 0;
        ID deletePeriod = -1, deleteVehicle = -1;
        ID deleteNode = -1, preNode = -1, postNode = -1;
        for (ID t = 0; t < input.periodnum(); ++t) {
            for (ID v = 0; v < input.vehicles_size(); ++v) {
                if (presetX.xQuantity[v][t][0] < IrpModelSolver::DefaultDoubleGap) { continue; }
                for (ID i = 1; i < input.nodes_size(); ++i) {
                    if (presetX.xQuantity[v][t][i] < IrpModelSolver::DefaultDoubleGap) { continue; }
                    bool valid = true;
                    //if (presetX.xQuantity[v][t][i] + presetX.xQuantity[v][t][0] > input.vehicles()[v].capacity()) { continue; }
                    int restQuantity = input.nodes()[i].initquantity();
                    for (ID j = 0; j < input.periodnum(); ++j) {
                        restQuantity -= input.nodes()[i].unitdemand();
                        restQuantity += (j == t) ? 0 : lround(presetX.xQuantity[v][j][i]);
                        if (restQuantity < 0) { valid = false; }
                    }
                    if (valid) {
                        double delta = (input.nodes()[i].holdingcost() - input.nodes()[0].holdingcost()) * lround(presetX.xQuantity[v][t][i]) * (input.periodnum() - t);
                        ID pre = -1, post = -1;
                        for (ID j = 0; j < input.nodes_size(); ++j) {
                            if (i == j) { continue; }
                            if (presetX.xEdge[v][t][i][j]) {
                                post = j;
                            }
                            if (presetX.xEdge[v][t][j][i]) {
                                pre = j;
                            }
                            if ((pre >= 0) && (post >= 0)) { break; }
                        }
                        delta += aux.routingCost[pre][i] + aux.routingCost[i][post] - aux.routingCost[pre][post];

                        if (delta > decreasedCost) {
                            decreasedCost = delta;
                            deleteVehicle = v;
                            deletePeriod = t;
                            deleteNode = i;
                            preNode = pre;
                            postNode = post;
                        }
                    }
                }
            }
        }
        cout << deleteNode << "," << deletePeriod << "," << decreasedCost << endl;
        if (decreasedCost <= 0) {
            break;
        }

        presetX.xQuantity[deleteVehicle][deletePeriod][0] -= lround(presetX.xQuantity[deleteVehicle][deletePeriod][deleteNode]);
        presetX.xQuantity[deleteVehicle][deletePeriod][deleteNode] = 0;
        presetX.xEdge[deleteVehicle][deletePeriod][preNode][deleteNode] = false;
        presetX.xEdge[deleteVehicle][deletePeriod][deleteNode][postNode] = false;
        presetX.xEdge[deleteVehicle][deletePeriod][preNode][postNode] = true;

        sln.totalCost -= decreasedCost;
        cout << sln.totalCost << endl;
    }

    routingObj = 0;
    if (!solveVrp(presetX, routingObj, elapsedSeconds, modelSolver.input.nodes)) { return false; }
    // record solution.
    sln.totalCost = bestHolding + routingObj;
    retrieveOutputFromModel(sln, modelSolver.presetX);

    return true;
}

bool Solver::solveWithTest(Solution & sln, bool findFeasibleFirst) {
    const String TspCacheDir("lkh/TspCache/");
    System::makeSureDirExist(TspCacheDir);

    IrpModelSolver irpSolver;
    if (findFeasibleFirst) { irpSolver.setFindFeasiblePreference(); }

    convertToModelInput(irpSolver.input, input);
    irpSolver.routingCost = aux.routingCost;
    irpSolver.setTspCachePath(TspCacheDir + env.friendlyInstName() + ".csv");
    if (!irpSolver.solveIteratively()) { return false; }

    IrpModelSolver::PresetX &presetX(irpSolver.presetX);
    double elapsedSeconds = irpSolver.getDurationInSecond();
    double holdingObj = irpSolver.currentObjective.holdingCost;
    double routingObj = irpSolver.currentObjective.routingCost;
    sln.totalCost = irpSolver.currentObjective.totalCost;
    cout << (holdingObj + routingObj) << "=" << routingObj << "(R) + " << holdingObj << "(H)\n";

    IrpModelSolver::PresetX bestSln(irpSolver.presetX);
    double bestObj = irpSolver.currentObjective.totalCost;

    // search
    // define parameters.
    const int StopIter = 200;
    const int MoveThreshold = 20;
    const int OptimizationCycle = 10;

    // define moves.
    struct BasicMove {
        ID vehicle;
        ID period;
        ID node;
        ID preNode;
        ID postNode;
        double quantity;

        BasicMove() :vehicle(-1), period(-1), node(-1), preNode(-1), postNode(-1), quantity(0){}
        BasicMove(ID v, ID t, ID i, ID pre, ID post, double q) :vehicle(v), period(t), node(i), preNode(pre), postNode(post), quantity(q){}
    };

    struct SwapMove {
        // actions[0] presents add move, respectively, actions[1] presents remove move.
        List<BasicMove> actions;

        SwapMove() { actions.resize(2, BasicMove()); }
        SwapMove(ID v, ID i, ID t1, ID t2, ID pre1 = -1, ID post1 = -1, ID pre2 = -1, ID post2 = -1) {
            actions.push_back({ v, t1, i, pre1, post1, 0 });
            actions.push_back({ v, t2, i, pre2, post2, 0 });
        }
    };
    // initialize auxiliary data structure
    presetX.xVisited.resize(input.vehicles_size());
    for (ID v = 0; v < input.vehicles_size(); ++v) {
        presetX.xVisited[v].resize(input.periodnum());
        for (ID t = 0; t < input.periodnum(); ++t) {
            presetX.xVisited[v][t].resize(input.nodes_size(), false);
            for (ID i = 0; i < input.nodes_size(); ++i) {
                if (presetX.xQuantity[v][t][i] > IrpModelSolver::DefaultDoubleGap) {
                    presetX.xVisited[v][t][i] = true;
                }
            }
        }
    }

    List3D<bool> removeFeasible;
    removeFeasible.resize(input.vehicles_size());
    for (ID v = 0; v < input.vehicles_size(); ++v) {
        removeFeasible[v].resize(input.periodnum());
        for (ID t = 0; t < input.periodnum(); ++t) {
            removeFeasible[v][t].resize(input.nodes_size(), false);
            if (!presetX.xVisited[v][t][0]) { continue; }
            for (ID i = 1; i < input.nodes_size(); ++i) {
                if (!presetX.xVisited[v][t][i]) { continue; }
                removeFeasible[v][t][i] = true;
                double rest = input.nodes()[i].initquantity();
                for (ID p = 0; p < input.periodnum(); ++p) {
                    rest -= input.nodes()[i].unitdemand();
                    rest += ((p != t) ? presetX.xQuantity[v][p][i] : 0);
                    if ((p >= t) && (rest < 0)) {
                        removeFeasible[v][t][i] = false;
                        break;
                    }
                }
            }
        }
    }
    List<double> maxProvidedBySupplier;
    List2D<double> maxDeliveryToCustomer;
    List2D<double> vehicleCapacity;
    List3D<double> addFeasible;
    maxProvidedBySupplier.resize(input.periodnum(), 0);
    maxDeliveryToCustomer.resize(input.periodnum(), List<double>(input.nodes_size(), 0));
    vehicleCapacity.resize(input.vehicles_size(), List<double>(input.periodnum(), 0));
    addFeasible.resize(input.vehicles_size(), List2D<double>(input.periodnum(), List<double>(input.nodes_size(), 0)));
    //for (ID i = 1; i < input.nodes_size(); ++i) {
    //    if (input.nodes()[i].holdingcost() >= input.nodes()[0].holdingcost()) { continue; }
    //    double unitDemand = input.nodes()[i].unitdemand();
    //    double supplierUnitDemand = input.nodes()[0].unitdemand();
    //    for (ID v = 0; v < input.vehicles_size(); ++v) {
    //        double rest = input.nodes()[i].initquantity();
    //        double restOfSupplier = input.nodes()[0].initquantity();
    //        for (ID t = 0; t < input.periodnum(); ++t) {
    //            vehicleCapacity[v][t] = input.vehicles()[v].capacity() - presetX.xQuantity[v][t][0];
    //            rest -= unitDemand;
    //            rest += presetX.xQuantity[v][t][i];
    //            restOfSupplier += supplierUnitDemand;
    //            restOfSupplier -= presetX.xQuantity[v][t][0];
    //            if (!presetX.xVisited[v][t][0] || presetX.xVisited[v][t][i]) { continue; }
    //            double deliveryQuantity = vehicleCapacity[v][t];
    //            // calculate max quantity provided by supplier.
    //            double restOfSupplierIfDelivered = restOfSupplier;
    //            maxProvidedBySupplier[t] = restOfSupplier;
    //            for (ID p = t + 1; p < input.periodnum(); ++p) {
    //                restOfSupplierIfDelivered += supplierUnitDemand;
    //                restOfSupplierIfDelivered -= presetX.xQuantity[v][p][0];
    //                maxProvidedBySupplier[t] = min(maxProvidedBySupplier[t], restOfSupplierIfDelivered);
    //            }
    //            deliveryQuantity = min(deliveryQuantity, maxProvidedBySupplier[t][i]);
    //            // calculate max delivery to customer.
    //            double restOfCustomerIfDelivered = rest + unitDemand;
    //            maxDeliveryToCustomer[t][i] = input.nodes()[i].capacity() - restOfCustomerIfDelivered;
    //            for (ID p = t + 1; p < input.periodnum(); ++p) {
    //                restOfCustomerIfDelivered -= unitDemand;
    //                restOfCustomerIfDelivered += presetX.xQuantity[v][p][i];
    //                maxDeliveryToCustomer[t][i] = min(maxDeliveryToCustomer[t][i], input.nodes()[i].capacity() - restOfCustomerIfDelivered);
    //            }
    //            deliveryQuantity = min(deliveryQuantity, maxDeliveryToCustomer[t][i]);
    //            addFeasible[v][t][i] = deliveryQuantity;
    //        }
    //    }
    //}
    {
        // init data structure
        double supplierUnitDemand = input.nodes()[0].unitdemand();
        double restOfSupplier = input.nodes()[0].initquantity();
        List<double> restOfCustomers(input.nodes_size(), 0);
        for (ID t = 0; t < input.periodnum(); ++t) {
            restOfSupplier += supplierUnitDemand;
            for (ID v = 0; v < input.vehicles_size(); ++v) {
                vehicleCapacity[v][t] = input.vehicles()[v].capacity() - presetX.xQuantity[v][t][0];
                restOfSupplier -= presetX.xQuantity[v][t][0];
            }
            // calculate max quantity provided by supplier.
            double restOfSupplierIfDelivered = restOfSupplier;
            maxProvidedBySupplier[t] = restOfSupplier;
            for (ID p = t + 1; p < input.periodnum(); ++p) {
                restOfSupplierIfDelivered += supplierUnitDemand;
                for (ID v = 0; v < input.vehicles_size(); ++v) {
                    restOfSupplierIfDelivered -= presetX.xQuantity[v][p][0];
                }
                maxProvidedBySupplier[t] = min(maxProvidedBySupplier[t], restOfSupplierIfDelivered);
            }
            // calculate max delivery to customer.
            for (ID i = 1; i < input.nodes_size(); ++i) {
                if (input.nodes()[i].holdingcost() >= input.nodes()[0].holdingcost()) { continue; }
                double unitDemand = input.nodes()[i].unitdemand();
                if (t == 0) { restOfCustomers[i] = input.nodes()[i].initquantity(); }
                restOfCustomers[i] -= unitDemand;
                for (ID v = 0; v < input.vehicles_size(); ++v) {
                    restOfCustomers[i] += presetX.xQuantity[v][t][i];
                }
                double restOfCustomerIfDelivered = restOfCustomers[i] + unitDemand;
                maxDeliveryToCustomer[t][i] = input.nodes()[i].capacity() - restOfCustomerIfDelivered;
                for (ID p = t + 1; p < input.periodnum(); ++p) {
                    restOfCustomerIfDelivered -= unitDemand;
                    for (ID v = 0; v < input.vehicles_size(); ++v) {
                        restOfCustomerIfDelivered += presetX.xQuantity[v][p][i];
                    }
                    maxDeliveryToCustomer[t][i] = min(maxDeliveryToCustomer[t][i], input.nodes()[i].capacity() - restOfCustomerIfDelivered);
                }
                for (ID v = 0; v < input.vehicles_size(); ++v) {
                    // TODO[qym][5]: allow add node to unvisited period. 
                    if (!presetX.xVisited[v][t][0] || presetX.xVisited[v][t][i]) { continue; }
                    addFeasible[v][t][i] = min(vehicleCapacity[v][t], maxProvidedBySupplier[t], maxDeliveryToCustomer[t][i]);
                }
            }
        }
    }
   
    // methods.
    auto findRemove = [&](BasicMove &mv, double &objDelta)->bool {
        double supplierHCost = input.nodes()[0].holdingcost();
        for (ID v = 0; v < input.vehicles_size(); ++v) {
            for (ID t = 0; t < input.periodnum(); ++t) {
                if (!presetX.xVisited[v][t][0]) { continue; }
                for (ID i = 1; i < input.nodes_size(); ++i) {
                    if (!removeFeasible[v][t][i]) { continue; }
                    double hc = (input.periodnum() - t) * (supplierHCost - input.nodes()[i].holdingcost()) * presetX.xQuantity[v][t][i];
                    // find node location
                    ID pre = -1, post = -1;
                    for (ID j = 0; j < input.nodes_size(); ++j) {
                        if (i == j) { continue; }
                        if (presetX.xEdge[v][t][j][i]) { pre = j; }
                        if (presetX.xEdge[v][t][i][j]) { post = j; }
                        if ((pre >= 0) && (post >= 0)) { break; }
                    }
                    if ((pre < 0) || (post < 0)) { cout << "Warning"; system("pause"); return false; }
                    double rc = aux.routingCost[pre][post] - aux.routingCost[pre][i] - aux.routingCost[i][post];
                    if (hc + rc < objDelta) {
                        objDelta = hc + rc;
                        mv.node = i;
                        mv.period = t;
                        mv.vehicle = v;
                        mv.preNode = pre;
                        mv.postNode = post;
                        mv.quantity = 0;
                    }
                }
            }
        }
        return (mv.node >= 0);
    };

    auto findAdd = [&](BasicMove &mv, double &objDelta)->bool {
        double supplierHCost = input.nodes()[0].holdingcost();

        for (ID v = 0; v < input.vehicles_size(); ++v) {
            for (ID t = 0; t < input.periodnum(); ++t) {
                if (!presetX.xVisited[v][t][0]) { continue; }
                for (ID i = 1; i < input.nodes_size(); ++i) {
                    if (addFeasible[v][t][i] <= 0) { continue; }
                    double hc = (input.periodnum() - t) * (input.nodes()[i].holdingcost() - supplierHCost) * addFeasible[v][t][i];
                    // find node location
                    ID pre = -1, post = -1;
                    double rc = HUGE_VAL;
                    for (ID j = 0; j < input.nodes_size(); ++j) {
                        if ((i == j) || !presetX.xVisited[v][t][j]){ continue; }
                        for (ID k = 0; k < input.nodes_size(); ++k) {
                            if ((i == k) || (j == k)) { continue; }
                            if (!presetX.xEdge[v][t][j][k]) { continue; }
                            if (aux.routingCost[j][i] + aux.routingCost[i][k] - aux.routingCost[j][k] < rc) {
                                rc = aux.routingCost[j][i] + aux.routingCost[i][k] - aux.routingCost[j][k];
                                pre = j;
                                post = k;
                            }
                        }
                    }
                    if ((pre < 0) || (post < 0)) { cout << "Warning"; system("pause"); return false; }
                    if (hc + rc < objDelta) {
                        objDelta = hc + rc;
                        mv.node = i;
                        mv.period = t;
                        mv.vehicle = v;
                        mv.preNode = pre;
                        mv.postNode = post;
                        mv.quantity = addFeasible[v][t][i];
                    }
                }
            }
        }
        return (mv.node >= 0);
    };

    auto makeRemove = [&](const BasicMove &mv, double objDelta) {
        double oldQuantity = presetX.xQuantity[mv.vehicle][mv.period][mv.node];
        presetX.xVisited[mv.vehicle][mv.period][mv.node] = false;
        presetX.xQuantity[mv.vehicle][mv.period][0] -= oldQuantity;
        presetX.xQuantity[mv.vehicle][mv.period][mv.node] = mv.quantity;
        presetX.xEdge[mv.vehicle][mv.period][mv.preNode][mv.node] = false;
        presetX.xEdge[mv.vehicle][mv.period][mv.node][mv.postNode] = false;
        presetX.xEdge[mv.vehicle][mv.period][mv.preNode][mv.postNode] = true;
        sln.totalCost += objDelta;

        removeFeasible[mv.vehicle][mv.period][mv.node] = false;
        for (ID t = 0; t < input.periodnum(); ++t) {
            if (!presetX.xVisited[mv.vehicle][t][mv.node]) { continue; }
            removeFeasible[mv.vehicle][t][mv.node] = true;
            double rest = input.nodes()[mv.node].initquantity();
            for (ID p = 0; p < input.periodnum(); ++p) {
                rest -= input.nodes()[mv.node].unitdemand();
                rest += ((p != t) ? presetX.xQuantity[mv.vehicle][p][mv.node] : 0);
                if ((p >= t) && (rest < 0)) {
                    removeFeasible[mv.vehicle][t][mv.node] = false;
                    break;
                }
            }
        }
        
        // for vehicle. 
        vehicleCapacity[mv.vehicle][mv.period] += oldQuantity;
        // for supplier
        // TODO[qym][0]: 
        double supplierUnitDemand = input.nodes()[0].unitdemand();
        for (ID t = 0; t < input.periodnum(); ++t) {
            maxProvidedBySupplier[t];
            double restOfSupplierIfDelivered = restOfSupplier;
            maxProvidedBySupplier[v][t][i] = restOfSupplier;
            for (ID p = t + 1; p < input.periodnum(); ++p) {
                restOfSupplierIfDelivered += supplierUnitDemand;
                restOfSupplierIfDelivered -= presetX.xQuantity[v][p][0];
                maxProvidedBySupplier[v][t][i] = min(maxProvidedBySupplier[v][t][i], restOfSupplierIfDelivered);
            }
            deliveryQuantity = min(deliveryQuantity, maxProvidedBySupplier[v][t][i]);
        }
        // for customer.
        double customerUnitDemand = input.nodes()[mv.node].unitdemand();
        //double restOfCustmer = input.nodes()[mv.node].initquantity();
        //for (ID t = 0; t < input.periodnum(); ++t) {
        //    restOfCustmer -= customerUnitDemand;
        //    for (ID v = 0; v < input.vehicles_size(); ++v) {
        //        restOfCustmer += presetX.xQuantity[v][t][mv.node];
        //    }
        //    double restIfDelivered = restOfCustmer + customerUnitDemand;
        //    maxDeliveryToCustomer[t][mv.node] = input.nodes()[mv.node].capacity() - restIfDelivered;
        //    for (ID p = t + 1; p < input.periodnum(); ++p) {
        //        restIfDelivered -= customerUnitDemand;
        //        for (ID v = 0; v < input.vehicles_size(); ++v) {
        //            restIfDelivered += presetX.xQuantity[v][p][mv.node];
        //        }
        //        maxDeliveryToCustomer[t][mv.node] = min(maxDeliveryToCustomer[t][mv.node], input.nodes()[mv.node].capacity() - restIfDelivered);
        //    }
        //}
        for (ID v = 0; v < input.vehicles_size(); ++v) {
            oldQuantity += presetX.xQuantity[v][mv.period][mv.node];
        }
        for (ID t = 0; t < input.periodnum(); ++t) {
            if (t == mv.period) { continue; }
            if (t > mv.period) {
                maxDeliveryToCustomer[t][mv.node] += oldQuantity;
            } else {
                if (customerUnitDemand >= oldQuantity) {
                    continue;
                } else {
                    //maxDeliveryToCustomer[t][mv.node] += oldQuantity - customerUnitDemand;
                    double maxQuantity = input.nodes()[mv.node].capacity() - input.nodes()[mv.node].initquantity();
                    maxDeliveryToCustomer[t][mv.node] = HUGE_VAL;
                    for (ID p = 0; p < input.periodnum(); ++p) {
                        if (p != t) {
                            double delivery = 0;
                            for (ID v = 0; v < input.vehicles_size(); ++v) {
                                delivery += presetX.xQuantity[v][p][mv.node];
                            }
                            maxQuantity -= delivery;
                            maxQuantity += customerUnitDemand;
                        }
                        if (p >= t) {
                            maxDeliveryToCustomer[t][mv.node] = min(maxDeliveryToCustomer[t][mv.node], maxQuantity);
                        }
                    }
                }
            }
        }
        for (ID v = 0; v < input.vehicles_size(); ++v) {
            for (ID t = 0; t < input.periodnum(); ++t) {
                for (ID i = 1; i < input.nodes_size(); ++i) {
                    addFeasible[v][t][i] = min(vehicleCapacity[v][t], maxProvidedBySupplier[t], maxDeliveryToCustomer[t][i]);
                }
            }
        }
    };

    auto makeAdd = [&](const BasicMove &mv, double objDelta) {
        presetX.xVisited[mv.vehicle][mv.period][mv.node] = true;
        presetX.xQuantity[mv.vehicle][mv.period][mv.node] = mv.quantity;
        presetX.xQuantity[mv.vehicle][mv.period][0] += mv.quantity;
        presetX.xEdge[mv.vehicle][mv.period][mv.preNode][mv.node] = true;
        presetX.xEdge[mv.vehicle][mv.period][mv.node][mv.postNode] = true;
        presetX.xEdge[mv.vehicle][mv.period][mv.preNode][mv.postNode] = false;
        sln.totalCost += objDelta;

        for (ID t = 0; t < input.periodnum(); ++t) {
            if (!presetX.xVisited[mv.vehicle][t][mv.node]) { continue; }
            removeFeasible[mv.vehicle][t][mv.node] = true;
            double rest = input.nodes()[mv.node].initquantity();
            for (ID p = 0; p < input.periodnum(); ++p) {
                rest -= input.nodes()[mv.node].unitdemand();
                rest += ((p != t) ? presetX.xQuantity[mv.vehicle][p][mv.node] : 0);
                if ((p >= t) && (rest < 0)) {
                    removeFeasible[mv.vehicle][t][mv.node] = false;
                    break;
                }
            }
        }

        vehicleCapacity[mv.vehicle][mv.period] -= mv.quantity;


    };
    // start iterations.
    int iter = 0;
    int iterNoImprove = 0;
    //找目标值增量最小的动作
    while (iter < StopIter) {
        if (iterNoImprove <= MoveThreshold) {
            // find best from add and remove seperately.
            BasicMove rmMv, addMv;
            double rmObj = HUGE_VAL, addObj = HUGE_VAL;;
            // remove
            if (!findRemove(rmMv, rmObj)) {
                cout << "Not find remove move.";
            }
           
            //add
            if (!findAdd(addMv, addObj)) {
                cout << "Not find add move.";
            }
             // make move.
            if (addObj < rmObj) {
                makeAdd(addMv, addObj);
            } else {
                makeRemove(rmMv, rmObj);
            }
        } else {
            // find best from add or remove or swap.
            int moveType = rand() % 3;
            double objDelta = HUGE_VAL;
            if (moveType == 0) {
                // add
                BasicMove mv;
                if (!findAdd(mv, objDelta)) {
                    cout << "Not find add move.";
                    system("pause");
                }
                // make move.
                makeAdd(mv, objDelta);
            } else if (moveType == 1) {
                // remove
                BasicMove mv;
                if (!findRemove(mv, objDelta)) {
                    cout << "Not find remove move."; 
                    system("pause");
                }
                // make move.
                makeRemove(mv, objDelta);
            } else {
                // swap
                ;
            }
        }

        if (iter % OptimizationCycle == 0) {
            // optimize accurately.
            IrpModelSolver inventorySolver(irpSolver.input);
            if (findFeasibleFirst) { inventorySolver.setFindFeasiblePreference(); }
            inventorySolver.routingCost = aux.routingCost;
            inventorySolver.enablePresetSolution();
            inventorySolver.presetX.xVisited = presetX.xVisited;
            if (!inventorySolver.solveInventoryModel()) { return false; }
            presetX.xQuantity = inventorySolver.presetX.xQuantity;
            holdingObj = inventorySolver.getHoldingCostInPeriod(0, input.periodnum());

            routingObj = 0;
            if (!solveVrp(presetX, routingObj, elapsedSeconds, irpSolver.input.nodes)) { return false; }
            cout << (holdingObj + routingObj) << "=" << routingObj << "(R) + " << holdingObj << "(H)\n";
            sln.totalCost = (holdingObj + routingObj);
        }
        ++iterNoImprove;
        ++iter;
        if (sln.totalCost < bestObj) {
            bestSln = presetX;
            bestObj = sln.totalCost;
            iterNoImprove = 0;
        }
    }


    /*
    writePathToCsv(presetX);
    List<List<Set<ID>>> unvisitedNodes(input.vehicles_size(), List<Set<ID>>(input.periodnum()));
    presetX.xVisited.resize(input.vehicles_size());
    for (ID v = 0; v < input.vehicles_size(); ++v) {
        presetX.xVisited[v].resize(input.periodnum());
        for (ID t = 0; t < input.periodnum(); ++t) {
            presetX.xVisited[v][t].resize(input.nodes_size(), false);
            for (ID i = 0; i < input.nodes_size(); ++i) {
                if (presetX.xQuantity[v][t][i] > IrpModelSolver::DefaultDoubleGap) {
                    presetX.xVisited[v][t][i] = true;
                } else {
                    unvisitedNodes[v][t].insert(i);
                }
            }
        }
    }
    while (iter < 150) {
        if (iter % 5 > 0) {
            double bestDelta = IrpModelSolver::Infinity;
            AddMove bestMove;
            for (ID v = 0; v < input.vehicles_size(); ++v) {
                for (ID t = 0; t < input.periodnum(); ++t) {
                    // skip non-delivered period.
                    if (unvisitedNodes.size() >= input.nodes_size()) { continue; }
                    for (auto ins = unvisitedNodes[v][t].begin(); ins != unvisitedNodes[v][t].end(); ++ins) {
                        double objDelta = 0;
                        // estimate holding cost
                        objDelta += (input.nodes()[*ins].holdingcost() - input.nodes()[0].holdingcost()) * input.nodes()[*ins].capacity();
                        // find insert position, estimate routing cost
                        double rc = IrpModelSolver::Infinity;
                        ID preNode = -1, postNode = -1;
                        int posCount = 0;
                        for (ID i = 0; i < input.nodes_size(); ++i) {
                            for (ID j = 0; j < input.nodes_size(); ++j) {
                                if (i == j) { continue; }
                                if (!presetX.xEdge[v][t][i][j]) { continue; }
                                if (aux.routingCost[i][*ins] + aux.routingCost[*ins][j] - aux.routingCost[i][j] < rc) {
                                    rc = aux.routingCost[i][*ins] + aux.routingCost[*ins][j] - aux.routingCost[i][j];
                                    preNode = i;
                                    postNode = j;
                                    posCount = 1;
                                } else if (aux.routingCost[i][*ins] + aux.routingCost[*ins][j] - aux.routingCost[i][j] == rc) {
                                    ++posCount;
                                    if (rand() % posCount < 1) {
                                        preNode = i;
                                        postNode = j;
                                    }
                                }
                            }
                        }
                        if (posCount <= 0) { continue; }
                        objDelta += rc;
                        if (objDelta < bestDelta) {
                            bestDelta = objDelta;
                            bestMove = { v, t, *ins, preNode, postNode };
                        }
                    }

                }
            }
            if (bestMove.node < 0) { break; }
            //cout << "Visit " << bestMove.node << " at Period " << bestMove.period << endl;
            presetX.xVisited[bestMove.vehicle][bestMove.period][bestMove.node] = true;
            unvisitedNodes[bestMove.vehicle][bestMove.period].erase(bestMove.node);

            IrpModelSolver inventorySolver(irpSolver.input);
            if (findFeasibleFirst) { inventorySolver.setFindFeasiblePreference(); }

            inventorySolver.routingCost = aux.routingCost;
            inventorySolver.enablePresetSolution();
            inventorySolver.presetX.xVisited = presetX.xVisited;

            if (!inventorySolver.solveInventoryModel()) { return false; }

            presetX.xQuantity = inventorySolver.presetX.xQuantity;
            holdingObj = inventorySolver.getHoldingCostInPeriod(0, input.periodnum());
            routingObj = 0;
            if (!solveVrp(presetX, routingObj, elapsedSeconds, irpSolver.input.nodes)) { return false; }
            cout << (holdingObj + routingObj) << "=" << routingObj << "(R) + " << holdingObj << "(H)\n";
            sln.totalCost = (holdingObj + routingObj);
        } else {
            while (true) {
                double decreasedCost = 0;
                ID deletePeriod = -1, deleteVehicle = -1;
                ID deleteNode = -1, preNode = -1, postNode = -1;
                for (ID t = 0; t < input.periodnum(); ++t) {
                    for (ID v = 0; v < input.vehicles_size(); ++v) {
                        if (presetX.xQuantity[v][t][0] < IrpModelSolver::DefaultDoubleGap) { continue; }
                        for (ID i = 1; i < input.nodes_size(); ++i) {
                            if (presetX.xQuantity[v][t][i] < IrpModelSolver::DefaultDoubleGap) { continue; }
                            bool valid = true;
                            //if (presetX.xQuantity[v][t][i] + presetX.xQuantity[v][t][0] > input.vehicles()[v].capacity()) { continue; }
                            int restQuantity = input.nodes()[i].initquantity();
                            for (ID j = 0; j < input.periodnum(); ++j) {
                                restQuantity -= input.nodes()[i].unitdemand();
                                restQuantity += (j == t) ? 0 : lround(presetX.xQuantity[v][j][i]);
                                if (restQuantity < 0) { valid = false; }
                            }
                            if (valid) {
                                double delta = (input.nodes()[i].holdingcost() - input.nodes()[0].holdingcost()) * lround(presetX.xQuantity[v][t][i]) * (input.periodnum() - t);
                                ID pre = -1, post = -1;
                                for (ID j = 0; j < input.nodes_size(); ++j) {
                                    if (i == j) { continue; }
                                    if (presetX.xEdge[v][t][i][j]) {
                                        post = j;
                                    }
                                    if (presetX.xEdge[v][t][j][i]) {
                                        pre = j;
                                    }
                                    if ((pre >= 0) && (post >= 0)) { break; }
                                }
                                delta += aux.routingCost[pre][i] + aux.routingCost[i][post] - aux.routingCost[pre][post];

                                if (delta > decreasedCost) {
                                    decreasedCost = delta;
                                    deleteVehicle = v;
                                    deletePeriod = t;
                                    deleteNode = i;
                                    preNode = pre;
                                    postNode = post;
                                }
                            }
                        }
                    }
                }
                //cout << deleteNode << "," << deletePeriod << "," << decreasedCost << endl;
                if (decreasedCost <= 0) {
                    break;
                }

                presetX.xQuantity[deleteVehicle][deletePeriod][0] -= lround(presetX.xQuantity[deleteVehicle][deletePeriod][deleteNode]);
                presetX.xQuantity[deleteVehicle][deletePeriod][deleteNode] = 0;
                presetX.xEdge[deleteVehicle][deletePeriod][preNode][deleteNode] = false;
                presetX.xEdge[deleteVehicle][deletePeriod][deleteNode][postNode] = false;
                presetX.xEdge[deleteVehicle][deletePeriod][preNode][postNode] = true;
                presetX.xVisited[deleteVehicle][deletePeriod][deleteNode] = false;
                unvisitedNodes[deleteVehicle][deletePeriod].insert(deleteNode);
                sln.totalCost -= decreasedCost;
                cout << sln.totalCost << "\t";
            }
            cout << endl;
        }
        //recordSolution(originInput, presetX);
        if (sln.totalCost < minCost) {
            bestSln = presetX;
            minCost = sln.totalCost;
        }
        ++iter;
    }
    */
    
    retrieveOutputFromModel(sln, bestSln);
    return true;
}

bool Solver::solveWithIterativeModel(Solution & sln, bool findFeasibleFirst) {
    const String TspCacheDir("lkh/TspCache/");
    System::makeSureDirExist(TspCacheDir);

    IrpModelSolver irpSolver;
    if (findFeasibleFirst) { irpSolver.setFindFeasiblePreference(); }

    convertToModelInput(irpSolver.input, input);
    irpSolver.routingCost = aux.routingCost;
    irpSolver.setTspCachePath(TspCacheDir + env.friendlyInstName() + ".csv");
    if (!irpSolver.solveIteratively()) { return false; }

    IrpModelSolver::PresetX &presetX(irpSolver.presetX);
    double elapsedSeconds = irpSolver.getDurationInSecond();
    double holdingObj = irpSolver.currentObjective.holdingCost;
    double routingObj = irpSolver.currentObjective.routingCost;
    sln.totalCost = irpSolver.currentObjective.totalCost;
    cout << (holdingObj + routingObj) << "=" << routingObj << "(R) + " << holdingObj << "(H)\n";

    // record path.
    // writePathToCsv(presetX);

    // local search?
    // TODO[qym][5]:  add nodes
    // TODO[qym][5]: local search 
    while (false) {
        double delta = INT_MAX;
        for (ID v = 0; v < input.vehicles_size(); ++v) {
            for (ID t = 0; t < input.periodnum(); ++t) {
                for (ID i = 0; i < input.nodes_size(); ++i) {
                    ;
                }
            }   
        }
    }
    // remove nodes.
    while (true) {
        double decreasedCost = 0;
        ID deletePeriod = -1, deleteVehicle = -1;
        ID deleteNode = -1, preNode = -1, postNode = -1;
        for (ID t = 0; t < input.periodnum(); ++t) {
            for (ID v = 0; v < input.vehicles_size(); ++v) {
                if (presetX.xQuantity[v][t][0] < IrpModelSolver::DefaultDoubleGap) { continue; }
                for (ID i = 1; i < input.nodes_size(); ++i) {
                    if (presetX.xQuantity[v][t][i] < IrpModelSolver::DefaultDoubleGap) { continue; }
                    bool valid = true;
                    //if (presetX.xQuantity[v][t][i] + presetX.xQuantity[v][t][0] > input.vehicles()[v].capacity()) { continue; }
                    int restQuantity = input.nodes()[i].initquantity();
                    for (ID j = 0; j < input.periodnum(); ++j) {
                        restQuantity -= input.nodes()[i].unitdemand();
                        restQuantity += (j == t) ? 0 : lround(presetX.xQuantity[v][j][i]);
                        if (restQuantity < 0) { valid = false; }
                    }
                    if (valid) {
                        double delta = (input.nodes()[i].holdingcost() - input.nodes()[0].holdingcost()) * lround(presetX.xQuantity[v][t][i]) * (input.periodnum() - t);
                        ID pre = -1, post = -1;
                        for (ID j = 0; j < input.nodes_size(); ++j) {
                            if (i == j) { continue; }
                            if (presetX.xEdge[v][t][i][j]) {
                                post = j;
                            }
                            if (presetX.xEdge[v][t][j][i]) {
                                pre = j;
                            }
                            if ((pre >= 0) && (post >= 0)) { break; }
                        }
                        delta += aux.routingCost[pre][i] + aux.routingCost[i][post] - aux.routingCost[pre][post];

                        if (delta > decreasedCost) {
                            decreasedCost = delta;
                            deleteVehicle = v;
                            deletePeriod = t;
                            deleteNode = i;
                            preNode = pre;
                            postNode = post;
                        }
                    }
                }
            }
        }
        cout << deleteNode << "," << deletePeriod << "," << decreasedCost << endl;
        if (decreasedCost <= 0) {
            break;
        }

        presetX.xQuantity[deleteVehicle][deletePeriod][0] -= lround(presetX.xQuantity[deleteVehicle][deletePeriod][deleteNode]);
        presetX.xQuantity[deleteVehicle][deletePeriod][deleteNode] = 0;
        presetX.xEdge[deleteVehicle][deletePeriod][preNode][deleteNode] = false;
        presetX.xEdge[deleteVehicle][deletePeriod][deleteNode][postNode] = false;
        presetX.xEdge[deleteVehicle][deletePeriod][preNode][postNode] = true;

        sln.totalCost -= decreasedCost;
        cout << sln.totalCost << endl;
    }

    retrieveOutputFromModel(sln, irpSolver.presetX);
    return true;
}

bool Solver::solveWithInventoryRelaxed(Solution & sln, bool findFeasibleFirst) {
    const int ReduceModelScaleIteration = 3;
    const int RandModelScaleIteration = 6;
    const int DefaultMoveCount = 2 - (rand() % 2);
    const int DefaultTimeLimitPerIteration = 120;
    const int DefaultTimeLimitSecond = IrpModelSolver::DefaultTimeLimitSecond;

    // generate initial solution using slack constraints for customer's min level.
    IrpModelSolver initSolver;
    convertToModelInput(initSolver.input, input);
    initSolver.routingCost = aux.routingCost;
    initSolver.relaxShortageConstraint();   
    if (!initSolver.solve()) { return false; }

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
    double bestObj = initSolver.getTotalCostInPeriod(0, input.periodnum(), true);

    IrpModelSolver::PresetX presetX(move(initSolver.presetX));
    // reoptimize with initial solution
    while ((totalSeconds < DefaultTimeLimitSecond) && (abs(bestObj - input.referenceobj()) > IrpModelSolver::DefaultDoubleGap)) {
        // update time limit and move count according to the number of iterations where solution is not improved. 
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

        IrpModelSolver::saveSolution(initSolver.input, presetX, env.friendlyLocalTime() + "_" + to_string(iter) + "_" + env.logPath);
        IrpModelSolver solver(initSolver.input);
        auto &isPeriodFixed(solver.periodFixedData());
        solver.routingCost = aux.routingCost;
        solver.presetX = presetX; // setting presetX and enabling preset solution should not reverse the order.
        solver.enablePresetSolution();
        solver.setTimeLimitInSecond(timeLimitPerIteration);
        if (!getFixedPeriods(input.periodnum(), isPeriodFixed, iter, tabuTable, moveCount)) { break; }
        if (findFeasibleFirst) { solver.setFindFeasiblePreference(); }
        
        bool feasibleFound = solver.solve();
        totalSeconds += solver.getDurationInSecond();
        ++iter;
        ++iterNoImprove;
        if (!feasibleFound) { continue; }
        if (bestObj - solver.getTotalCostInPeriod(0, input.periodnum()) < IrpModelSolver::DefaultDoubleGap) { continue; }
        // update optimal.
        iterNoImprove = 0;
        bestObj = solver.getTotalCostInPeriod(0, input.periodnum());
        secondsUntilOpt = totalSeconds;
        
        cout << "\nFix: ";
        for (int i = 0; i < input.periodnum(); ++i) {
            if (!isPeriodFixed[i]) { continue; }
            cout << i << " ";
        }
        cout << "\nBest: " << bestObj << endl;

        presetX = move(solver.presetX);
    }

    sln.totalCost = bestObj;
    retrieveOutputFromModel(sln, presetX);
    return true;
}

bool Solver::solveWithDecomposition(Solution & sln, bool findFeasibleFirst) {
    IrpModelSolver::Input originInput;
    convertToModelInput(originInput, input);
    TabuSolver tabuSolver(originInput, aux.routingCost);
    if (findFeasibleFirst) { tabuSolver.cfg.findFeasibleFirst = true; }

    if (!tabuSolver.solve()) { return false; }
    sln.totalCost = tabuSolver.sln.bestObj;
    retrieveOutputFromModel(sln, tabuSolver.sln.presetX);
    return true;
}

void Solver::analyzeSolution() {
    Problem::Output sln;
    if (!sln.load(env.DefaultAnalysisDir() + "Solution/" + env.friendlyInstName())) { return; }


    List<List<int>> visitedTimes(input.nodes_size());
    List<List<int>> deliveredQuantity(input.nodes_size(), List<int>(input.periodnum(), 0));
    List<List<int>> preNode(input.nodes_size(), List<int>(input.periodnum(), -1));
    List<List<int>> sufNode(input.nodes_size(), List<int>(input.periodnum(), -1));
    {
        // content for log file.
        ostringstream oss;
        oss << env.instPath << ",";
        auto routes = sln.mutable_allroutes();
        ID pid = 0;
        for (auto r = routes->begin(); r != routes->end(); ++r, ++pid) {
            auto deliveries = r->routes().begin()->deliveries(); // only one vehicle
            oss << (deliveries.size() - 1) << ",";
            ID pre = 0;
            for (auto d = deliveries.begin(); d != deliveries.end(); ++d) {
                visitedTimes[d->node()].push_back(pid);
                deliveredQuantity[d->node()][pid] = d->quantity();
                preNode[d->node()][pid] = pre;
                sufNode[pre][pid] = d->node();
                pre = d->node();
            }
        }
        for (ID i = 1; i < input.nodes_size(); ++i) {
            oss << "," << visitedTimes[i].size();
        }
        oss << endl;

        ofstream logFile("analysis.csv", ios::app);
        logFile.seekp(0, ios::end);
        if (logFile.tellp() <= 0) {
            logFile << "Instance";
            for (ID i = 0; i < input.periodnum(); ++i) {
                logFile << ",Period " << i;
            }
            logFile << ",Node";
            for (ID i = 1; i < input.nodes_size(); ++i) {
                logFile << ",Node " << i;
            }
            logFile << endl;
        }
        logFile << oss.str();
        logFile.close();
    }

    {
        // content for single analysis file.
        ostringstream oss;
        //oss << "Node,Init,Unit,Capacity,LatePeriod,MinPeriod,Solution\n";
        oss << "Node,初始,消耗,容量,最晚周期,最少周期,实际周期,提前配送,超额配送";
        for (ID t = 0; t < input.periodnum(); ++t) {
            oss << ",Period " << t << ",,";
        }
        for (ID t = 0; t < input.periodnum(); ++t) {
            oss << ",Period " << t;
        }
        oss << endl;
        for (ID i = 0; i < input.nodes_size(); ++i) {
            ID latestPeriod = input.nodes()[i].initquantity() / input.nodes()[i].unitdemand(); //最迟第几个周期需要送货
            int leastDeliveryNum = ceil(double(input.nodes()[i].unitdemand() * 6 - input.nodes()[i].initquantity()) / input.nodes()[i].capacity());
            oss << i << ",";
            oss << input.nodes()[i].initquantity() << ","
                << input.nodes()[i].unitdemand() << ","
                << input.nodes()[i].capacity() << ","
                << latestPeriod << ","
                << leastDeliveryNum << ","
                << visitedTimes[i].size() << ",";
            oss << (visitedTimes[i][0] - latestPeriod) << ","
                << (visitedTimes[i].size() - leastDeliveryNum) << ",";
            //for (auto j = visitedTimes[i].begin(); j != visitedTimes[i].end(); ++j) {
            //    oss << *j << ",";
            //}
            for (ID t = 0; t < input.periodnum(); ++t) {
                oss << preNode[i][t] << "," << sufNode[i][t] << ",";
                if (preNode[i][t] >= 0 && sufNode[i][t] >= 0) {
                    oss << aux.routingCost[preNode[i][t]][i] + aux.routingCost[i][sufNode[i][t]] << ",";
                } else {
                    oss << ",";
                }
            }
            for (ID t = 0; t < input.periodnum(); ++t) {
                oss << deliveredQuantity[i][t] << ",";
            }
            oss << endl;
        }

        ofstream ofs(env.analysisPath());
        ofs << oss.str();
        ofs.close();
    }

    //// analyze routing cost between nodes.
    //ostringstream oss;
    //for (ID i = 0; i < input.nodes_size(); ++i) {
    //    oss << "," << i;
    //}
    //oss << endl;
    //for (ID i = 1; i < input.nodes_size(); ++i) {
    //    int minCost = aux.routingCost[0][i];
    //    oss << i << "," << aux.routingCost[0][i];
    //    for (ID j = 1; j < input.nodes_size(); ++j) {
    //        if (i == j) { oss << ","; continue; }
    //        oss << "," << (aux.routingCost[0][j] + aux.routingCost[j][i]);
    //        minCost = (aux.routingCost[0][j] + aux.routingCost[j][i]) < minCost ? (aux.routingCost[0][j] + aux.routingCost[j][i]) : minCost;
    //    }
    //    oss << "," << (minCost) << endl;
    //}
    //cout << endl;
    //ofstream ofs("routing.csv");
    //ofs << oss.str();
    //ofs.close();
}

void Solver::convertToModelInput(IrpModelSolver::Input & model, const Problem::Input & problem) {
    model.nodeNum = problem.nodes_size();
    model.periodNum = problem.periodnum();
    model.vehicleNum = problem.vehicles_size();
    model.bestObjective = problem.bestobj();
    model.referenceObjective = problem.referenceobj();

    model.nodes.resize(model.nodeNum);
    for (auto i = problem.nodes().begin(); i != problem.nodes().end(); ++i) {
        model.nodes[i->id()].initialQuantity = i->initquantity();
        model.nodes[i->id()].capacity = i->capacity();
        model.nodes[i->id()].minLevel = i->minlevel();
        model.nodes[i->id()].unitDemand = i->unitdemand();
        model.nodes[i->id()].holdingCost = i->holdingcost();
        model.nodes[i->id()].xCoord = i->x();
        model.nodes[i->id()].yCoord = i->y();
    }
    model.nodes[0].unitDemand = -model.nodes[0].unitDemand;
    model.vehicleCapacity = problem.vehicles()[0].capacity();
}

void Solver::retrieveOutputFromModel(Problem::Output & sln, const IrpModelSolver::PresetX & presetX) {
    // calculate routing cost and holding cost separately
    double routingCost = 0;
    double holdingCost = 0;
    List<List<int>> rq(input.nodes_size(), List<int>(input.periodnum() + 1, 0));

    List<int> restQuantity(input.nodes_size(), 0);
    for (auto i = input.nodes().begin(); i != input.nodes().end(); ++i) {
        restQuantity[i->id()] = i->initquantity();
        holdingCost += i->holdingcost() *  i->initquantity();
        rq[i->id()][0] = i->initquantity();
    }
    auto &allRoutes(*sln.mutable_allroutes());
    for (ID t = 0; t < input.periodnum(); ++t) {
        auto &routeInPeriod(*allRoutes.Add());
        for (ID v = 0; v != input.vehicles_size(); ++v) {
            auto &route(*routeInPeriod.add_routes());
            if (!presetX.xEdge[v][t].empty()) {
                ID i = 0;
                do {
                    for (ID j = 0; j < input.nodes_size(); ++j) {
                        if (i == j) { continue; }
                        if (!presetX.xEdge[v][t][i][j]) { continue; }
                        auto &delivery(*route.add_deliveries());
                        delivery.set_node(j);
                        delivery.set_quantity(lround(presetX.xQuantity[v][t][j]));

                        routingCost += aux.routingCost[i][j];
                        if (j == 0) {
                            delivery.set_quantity(-lround(presetX.xQuantity[v][t][j]));
                            restQuantity[j] -= lround(presetX.xQuantity[v][t][j]);
                        } else {
                            restQuantity[j] += lround(presetX.xQuantity[v][t][j]);
                        }
                        i = j;
                        break;
                    } 
                } while (i != 0);
            }
        }
        for (auto i = input.nodes().begin(); i != input.nodes().end(); ++i) {
            restQuantity[i->id()] -= i->unitdemand();
            holdingCost += i->holdingcost() * restQuantity[i->id()];
            rq[i->id()][t + 1] = restQuantity[i->id()];
        }
    }

    sln.set_routingcost(routingCost);
    sln.set_holdingcost(holdingCost);
    sln.totalCost = routingCost + holdingCost;
}

void Solver::writePathToCsv(const IrpModelSolver::PresetX & presetX) {
    ostringstream oss;
    //for (ID t = 0; t < input.periodnum(); ++t) {
    //    for (ID v = 0; v < input.vehicles_size(); ++v) {
    //        oss << "period " << t << endl;

    //        List<bool> visited(input.nodes_size(), false);
    //        for (ID i = 0; i < input.nodes_size(); ++i) {
    //            if (visited[i]) { continue; }
    //            ID p = i;
    //            do {
    //                for (ID j = 0; j < input.nodes_size(); ++j) {
    //                    if (p == j) { continue; }
    //                    if (presetX.xEdge[v][t][p][j]) {
    //                        oss << p << ",";
    //                        visited[p] = true;
    //                        p = j;
    //                        break;
    //                    }
    //                }
    //            } while(p != i);
    //            if (visited[p]) {
    //                oss << p << "\n";
    //            }
    //        }
    //        //if (presetX.xQuantity[v][t][0] > IrpModelSolver::DefaultDoubleGap) {
    //        //    while (true) {
    //        //        for (ID j = 0; j < input.nodes_size(); ++j) {
    //        //            if (i == j) { continue; }
    //        //            if (visited[j] || !presetX.xEdge[v][t][i][j]) { continue; }
    //        //            if (i == 0) { oss << i << ","; }
    //        //            i = j;
    //        //            oss << i << ",";
    //        //            break;
    //        //        }
    //        //        visited[i] = true;
    //        //        if (i == 0) { oss << endl; break; }
    //        //    }
    //        //}

    //        //for (ID u = 0; u < input.nodes_size(); ++u) {
    //        //    if (visited[u]) { continue; }
    //        //    visited[u] = true;
    //        //    i = u;
    //        //    if (presetX.xQuantity[v][t][u] < IrpModelSolver::DefaultDoubleGap) { continue; }
    //        //    while (true) {
    //        //        for (ID j = 0; j < input.nodes_size(); ++j) {
    //        //            if (i == j) { continue; }
    //        //            if (!presetX.xEdge[v][t][i][j]) { continue; }
    //        //            if (i == u) { oss << u << ","; }
    //        //            i = j;
    //        //            visited[i] = true;
    //        //            oss << i << ",";
    //        //            break;
    //        //        }
    //        //        if (i == u) { oss << endl; break; }
    //        //    }
    //        //}
    //        //oss << endl;
    //    }
    //}


    for (ID v = 0; v < input.vehicles_size(); ++v) {
        for (ID t = 0; t < input.periodnum(); ++t) {
            for (ID i = 0; i < input.nodes_size(); ++i) {
                if (presetX.xQuantity[v][t][i] < IrpModelSolver::DefaultDoubleGap) {
                    oss << 0 << ",";
                } else {
                    oss << 1 << ",";
                }
            }
            oss << endl;
        }
    }
    ofstream ofs(env.solutionPathWithTime() + ".path.csv");
    ofs << oss.str();
    ofs.close();
}

bool Solver::getFixedPeriods(int periodNum, List<bool>& isPeriodFixed, int iter, List<int>& tabuTable, int totalMoveCount) {
    // reset tabu table when no feasible move found.
    const bool resetTabuTable = true;

    int nonTabuCount = 0;
    for (int t = 0; t < periodNum; ++t) {
        if (iter > tabuTable[t]) { ++nonTabuCount; }
    }

    // return or reset tabu table if the number of non-tabu periods is less than half of total move count. 
    if (nonTabuCount < ((totalMoveCount + 1) / 2)) {
        if (!resetTabuTable) { return false; }
        while (nonTabuCount < ((totalMoveCount + 1) / 2)) {
            // reset tabu table randomly
            int p = rand() % periodNum;
            if (iter > tabuTable[p]) { continue; }
            tabuTable[p] = iter - 1;
            ++nonTabuCount;
        }
    }

    bool feasible = true;
    do {
        int tabuMoveCount = 0;
        int moveCount = 0;

        feasible = true;
        fill(isPeriodFixed.begin(), isPeriodFixed.end(), true);
        while (moveCount < totalMoveCount) {
            // select fixed periods randomly.
            int p = rand() % periodNum;
            if (!isPeriodFixed[p]) { continue; }
            isPeriodFixed[p] = false;
            ++moveCount;
            if (iter <= tabuTable[p]) { ++tabuMoveCount; }
            // the number of selected tabu periods should be no more than half of the total move count
            if (tabuMoveCount > totalMoveCount / 2) { feasible = false; break; }
        }
    } while (!feasible);

    for (int t = 0; t < periodNum; ++t) {
        if (isPeriodFixed[t]) { continue; }
        int tabuLen = 1 + rand() % (periodNum / 2);
        tabuTable[t] = iter + tabuLen;
    }
    return true;
}

bool Solver::solveVrp(IrpModelSolver::PresetX &presetX, double &obj, double &seconds, const List<IrpModelSolver::Node> &nodes) {
    obj = 0;
    for (int v = 0; v < input.vehicles_size(); ++v) {
        for (int t = 0; t < input.periodnum(); ++t) {
            for (int i = 0; i < input.nodes_size(); ++i) {
                for (int j = 0; j < input.nodes_size(); ++j) {
                    presetX.xEdge[v][t][i][j] = false;
                }
            }
            cout << "\nSolving period " << t;
            double singleObj, singleSeconds;
            if (!generateRoutingWithLkh(presetX.xEdge[v][t], singleObj, singleSeconds, presetX.xQuantity[v][t], nodes)) {
                cout << "Period " << t << " has not solved." << endl;
                return false;
            }
            obj += singleObj;
            seconds += singleSeconds;
        }
    }
    return true;
}

bool Solver::generateRouting(List<List<bool>>& edges, double & obj, double & seconds, const List<double>& quantity, const List<IrpModelSolver::Node> &nodes) {
    const int DefaultTimeLimit = 600;

    // if no delivery quantity in the period, return true.
    if (quantity.empty() || (quantity[0] < IrpModelSolver::DefaultDoubleGap)) {
        cout << " with 0 nodes.\n\n";
        return true;
    }

    List<ID> nodeIndices;
    IrpModelSolver::Input routeInput;
    routeInput.periodNum = 1;
    routeInput.vehicleNum = 1;
    routeInput.vehicleCapacity = 0;
    routeInput.nodes.push_back(nodes[0]);
    // add supplier.
    nodeIndices.push_back(0);
    edges.resize(input.nodes_size());
    edges[0].resize(nodes.size(), false);
    for (ID i = 1; i < nodes.size(); ++i) {
        edges[i].resize(nodes.size(), false);
        if (quantity[i] > IrpModelSolver::DefaultDoubleGap) {
            routeInput.nodes.push_back(nodes[i]);
            nodeIndices.push_back(i);
        }
    }
    routeInput.nodeNum = routeInput.nodes.size();
    cout << " with " << routeInput.nodeNum << " nodes.\n\n";

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
    obj = routeSolver.getMpSolverObjValue();
    seconds = routeSolver.getDurationInSecond();

    for (int i = 0; i < routeInput.nodeNum; ++i) {
        for (int j = 0; j < routeInput.nodeNum; ++j) {
            if (i == j) { continue; }
            edges[nodeIndices[i]][nodeIndices[j]] = routeSolver.presetX.xEdge[0][0][i][j];
        }
    }
    return true;
}

bool Solver::generateRoutingWithLkh(List<List<bool>>& edges, double & obj, double & seconds, const List<double>& quantity, const List<IrpModelSolver::Node> &nodes) {
    seconds = 0;
    // if no delivery quantity in the period, return true.
    if (quantity.empty() || (quantity[0] < IrpModelSolver::DefaultDoubleGap)) {
        cout << " with 0 nodes.\n\n";
        return true;
    }

    List<ID> visitedNodes;
    List<bool> containNode(input.nodes_size(), false);
    CachedTspSolver::CoordList2D coords;
    edges.resize(nodes.size());
    for (ID i = 0; i < nodes.size(); ++i) {
        edges[i].resize(nodes.size(), false);
        if (quantity[i] > IrpModelSolver::DefaultDoubleGap) {
            visitedNodes.push_back(i);
            containNode[i] = true;
            coords.push_back({ nodes[i].xCoord, nodes[i].yCoord });
        }
    }
    cout << " with " << visitedNodes.size() << " nodes.\n\n";
    if (visitedNodes.size() < 2) {
        return true;
    } else if (visitedNodes.size() == 2) {
        edges[visitedNodes[0]][visitedNodes[1]] = true;
        edges[visitedNodes[1]][visitedNodes[0]] = true;
        obj = (aux.routingCost[visitedNodes[0]][visitedNodes[1]] + aux.routingCost[visitedNodes[1]][visitedNodes[0]]);
    } else {
        CachedTspSolver::Tour tour;
        CachedTspSolver tspSolver(input.nodes_size(), "lkh/TspCache/" + env.friendlyInstName() + ".csv");
        if (!tspSolver.solve(tour, containNode, coords, [&](ID n) { return visitedNodes[n]; })) { return false; }
        obj = tour.distance;

        // convert solution of lkh to edges.
        ID preNode = 0;
        for (ID i = 1; i < tour.nodes.size(); ++i) {
            edges[preNode][tour.nodes[i]] = true;
            preNode = tour.nodes[i];
        }
        edges[preNode][0] = true;
    }

    return true;
}

void Solver::recordSolution(const IrpModelSolver::Input input, const IrpModelSolver::PresetX presetX) {
    //for (int v = 0; v < input.vehicleNum; ++v) {
    //    for (int t = 0; t < input.periodNum; ++t) {
    //        for (int i = 0; i < input.nodeNum; ++i) {
    //            if ((presetX.xQuantity[v][t][i] == 0 && presetX.xVisited[v][t][i]) 
    //                || (presetX.xQuantity[v][t][i] > 0 && !presetX.xVisited[v][t][i])) {
    //                cout << i << "\t" << presetX.xQuantity[v][t][i] << "\t" << presetX.xVisited[v][t][i] << endl;
    //            }
    //        }
    //    }
    //}
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
                q += lround(presetX.xQuantity[v][t][i]);
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
#pragma endregion Solver

}
