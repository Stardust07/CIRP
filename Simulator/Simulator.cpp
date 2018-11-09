#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>
#include <random>

#include <cstring>

#include "Simulator.h"
#include "ThreadPool.h"


using namespace std;


namespace szx {

void Simulator::initDefaultEnvironment() {
    Solver::Environment env;
    env.save(Env::DefaultEnvPath());

    Solver::Configuration cfg;
    cfg.save(Env::DefaultCfgPath());
}

void Simulator::run(const Task &task) {
    String instanceName(task.instSet + task.instId + ".json");
    String solutionName(task.instSet + task.instId + ".json");

    char argBuf[Cmd::MaxArgNum][Cmd::MaxArgLen];
    char *argv[Cmd::MaxArgNum];
    for (int i = 0; i < Cmd::MaxArgNum; ++i) { argv[i] = argBuf[i]; }
    strcpy(argv[ArgIndex::ExeName], ProgramName().c_str());

    int argc = ArgIndex::ArgStart;

    strcpy(argv[argc++], Cmd::InstancePathOption().c_str());
    strcpy(argv[argc++], (InstanceDir() + instanceName).c_str());

    System::makeSureDirExist(SolutionDir());
    strcpy(argv[argc++], Cmd::SolutionPathOption().c_str());
    strcpy(argv[argc++], (SolutionDir() + solutionName).c_str());

    if (!task.randSeed.empty()) {
        strcpy(argv[argc++], Cmd::RandSeedOption().c_str());
        strcpy(argv[argc++], task.randSeed.c_str());
    }

    if (!task.timeout.empty()) {
        strcpy(argv[argc++], Cmd::TimeoutOption().c_str());
        strcpy(argv[argc++], task.timeout.c_str());
    }

    if (!task.maxIter.empty()) {
        strcpy(argv[argc++], Cmd::MaxIterOption().c_str());
        strcpy(argv[argc++], task.maxIter.c_str());
    }

    if (!task.jobNum.empty()) {
        strcpy(argv[argc++], Cmd::JobNumOption().c_str());
        strcpy(argv[argc++], task.jobNum.c_str());
    }

    if (!task.runId.empty()) {
        strcpy(argv[argc++], Cmd::RunIdOption().c_str());
        strcpy(argv[argc++], task.runId.c_str());
    }

    if (!task.cfgPath.empty()) {
        strcpy(argv[argc++], Cmd::ConfigPathOption().c_str());
        strcpy(argv[argc++], task.cfgPath.c_str());
    }

    if (!task.logPath.empty()) {
        strcpy(argv[argc++], Cmd::LogPathOption().c_str());
        strcpy(argv[argc++], task.logPath.c_str());
    }

    Cmd::run(argc, argv);
}

void Simulator::run(const String &envPath) {
    char argBuf[Cmd::MaxArgNum][Cmd::MaxArgLen];
    char *argv[Cmd::MaxArgNum];
    for (int i = 0; i < Cmd::MaxArgNum; ++i) { argv[i] = argBuf[i]; }
    strcpy(argv[ArgIndex::ExeName], ProgramName().c_str());

    int argc = ArgIndex::ArgStart;

    strcpy(argv[argc++], Cmd::EnvironmentPathOption().c_str());
    strcpy(argv[argc++], envPath.c_str());

    Cmd::run(argc, argv);
}

void Simulator::debug() {
    Task task;
    task.instSet = "Instances_large_lowcost/";
    //task.instSet = "Instances_lowcost_H3/";
    task.instId = "abs1n50";
    task.randSeed = "1500972793";
    //task.randSeed = to_string(RandSeed::generate());
    task.timeout = "180";
    //task.maxIter = "1000000000";
    task.jobNum = "1";
    task.cfgPath = Env::DefaultCfgPath();
    task.logPath = Env::DefaultLogPath();
    task.runId = "0";

    run(task);
}

void Simulator::benchmark(int repeat) {
    Task task;
    task.instSet = "Instances_lowcost_H3/";
    //task.timeout = "180";
    //task.maxIter = "1000000000";
    task.timeout = "3600";
    //task.maxIter = "1000000000";
    task.jobNum = "1";
    task.cfgPath = Env::DefaultCfgPath();
    task.logPath = Env::DefaultLogPath();

    random_device rd;
    mt19937 rgen(rd());
    // EXTEND[szx][5]: read it from InstanceList.txt.
    vector<String> instList({ "abs1n5", "abs2n5" });
    for (int i = 0; i < repeat; ++i) {
        //shuffle(instList.begin(), instList.end(), rgen);
        for (auto inst = instList.begin(); inst != instList.end(); ++inst) {
            task.instId = *inst;
            task.randSeed = to_string(Random::generateSeed());
            task.runId = to_string(i);
            run(task);
        }
    }
}

void Simulator::parallelBenchmark(int repeat) {
    Task task;
    task.instSet = "Instances_lowcost_H3/";
    //task.timeout = "180";
    //task.maxIter = "1000000000";
    task.timeout = "3600";
    //task.maxIter = "1000000000";
    task.jobNum = "1";
    task.cfgPath = Env::DefaultCfgPath();
    task.logPath = Env::DefaultLogPath();

    ThreadPool<> tp(4);

    random_device rd;
    mt19937 rgen(rd());
    // EXTEND[szx][5]: read it from InstanceList.txt.
    vector<String> instList({ "abs1n5", "abs2n5" });
    for (int i = 0; i < repeat; ++i) {
        //shuffle(instList.begin(), instList.end(), rgen);
        for (auto inst = instList.begin(); inst != instList.end(); ++inst) {
            task.instId = *inst;
            task.randSeed = to_string(Random::generateSeed());
            task.runId = to_string(i);
            tp.push([=]() { run(task); });
            this_thread::sleep_for(1s);
        }
    }
}

void Simulator::generateInstance(const InstanceTrait &trait) {
    importBenchmarksFromCsv();
    convertAllInstancesToPb();
}

void Simulator::convertInstanceToPb(const String & fileName) {
    ifstream ifs(InstanceDir() + "Origin/" + fileName + ".dat");
    if (!ifs.is_open()) { return; }

    Problem::Input input;
    String instanceName(fileName);
    instanceName = instanceName.substr(instanceName.find_first_of('_') + 1);
    input.set_bestobj(benchmarks[instanceName][0]);
    input.set_referenceobj(benchmarks[instanceName][1]);
    input.set_referencetime(benchmarks[instanceName][2]);

    int nodeNum, periodNum, vehicleCapacity;
    ifs >> nodeNum >> periodNum >> vehicleCapacity;
    input.set_periodnum(periodNum);

    auto &vehicle(*input.add_vehicles());
    vehicle.set_id(0);
    vehicleCapacity /= 1; // all vehicles share the capacity.
    vehicle.set_capacity(vehicleCapacity);

    int id;
    double xPoint;
    double yPoint;
    int initialQuantity;
    int capacity;
    int minLevel;
    int unitDemand;
    double holdingCost;

    auto &supplier(*input.add_nodes());
    ifs >> id >> xPoint >> yPoint >> initialQuantity >> unitDemand >> holdingCost;
    setNodeInformation(supplier, id - 1, xPoint, yPoint, initialQuantity, initialQuantity + unitDemand * periodNum, 0, -unitDemand, holdingCost);
    for (int i = 1; i < nodeNum; ++i) {
        auto &node(*input.add_nodes());
        ifs >> id >> xPoint >> yPoint >> initialQuantity >> capacity >> minLevel >> unitDemand >> holdingCost;
        setNodeInformation(node, id - 1, xPoint, yPoint, initialQuantity, capacity, minLevel, unitDemand, holdingCost);
    }
    //input.nodeNum = input.nodes.size();
    ifs.close();

    ostringstream path;
    path << InstanceDir() << fileName << ".json";
    save(path.str(), input);
}

void Simulator::convertAllInstancesToPb() {
    int instanceCount[] = { 50, 50, 30, 30, 30, 30 };
    string instanceName[] = {
        "lowcost_H3/abs1n5","lowcost_H3/abs2n5","lowcost_H3/abs3n5","lowcost_H3/abs4n5","lowcost_H3/abs5n5",
        "lowcost_H3/abs1n10","lowcost_H3/abs2n10","lowcost_H3/abs3n10","lowcost_H3/abs4n10","lowcost_H3/abs5n10",
        "lowcost_H3/abs1n15","lowcost_H3/abs2n15","lowcost_H3/abs3n15","lowcost_H3/abs4n15","lowcost_H3/abs5n15",
        "lowcost_H3/abs1n20","lowcost_H3/abs2n20","lowcost_H3/abs3n20","lowcost_H3/abs4n20","lowcost_H3/abs5n20",
        "lowcost_H3/abs1n25","lowcost_H3/abs2n25","lowcost_H3/abs3n25","lowcost_H3/abs4n25","lowcost_H3/abs5n25",
        "lowcost_H3/abs1n30","lowcost_H3/abs2n30","lowcost_H3/abs3n30","lowcost_H3/abs4n30","lowcost_H3/abs5n30",
        "lowcost_H3/abs1n35","lowcost_H3/abs2n35","lowcost_H3/abs3n35","lowcost_H3/abs4n35","lowcost_H3/abs5n35",
        "lowcost_H3/abs1n40","lowcost_H3/abs2n40","lowcost_H3/abs3n40","lowcost_H3/abs4n40","lowcost_H3/abs5n40",
        "lowcost_H3/abs1n45","lowcost_H3/abs2n45","lowcost_H3/abs3n45","lowcost_H3/abs4n45","lowcost_H3/abs5n45",
        "lowcost_H3/abs1n50","lowcost_H3/abs2n50","lowcost_H3/abs3n50","lowcost_H3/abs4n50","lowcost_H3/abs5n50",

        "highcost_H3/abs1n5","highcost_H3/abs2n5","highcost_H3/abs3n5","highcost_H3/abs4n5","highcost_H3/abs5n5",
        "highcost_H3/abs1n10","highcost_H3/abs2n10","highcost_H3/abs3n10","highcost_H3/abs4n10","highcost_H3/abs5n10",
        "highcost_H3/abs1n15","highcost_H3/abs2n15","highcost_H3/abs3n15","highcost_H3/abs4n15","highcost_H3/abs5n15",
        "highcost_H3/abs1n20","highcost_H3/abs2n20","highcost_H3/abs3n20","highcost_H3/abs4n20","highcost_H3/abs5n20",
        "highcost_H3/abs1n25","highcost_H3/abs2n25","highcost_H3/abs3n25","highcost_H3/abs4n25","highcost_H3/abs5n25",
        "highcost_H3/abs1n30","highcost_H3/abs2n30","highcost_H3/abs3n30","highcost_H3/abs4n30","highcost_H3/abs5n30",
        "highcost_H3/abs1n35","highcost_H3/abs2n35","highcost_H3/abs3n35","highcost_H3/abs4n35","highcost_H3/abs5n35",
        "highcost_H3/abs1n40","highcost_H3/abs2n40","highcost_H3/abs3n40","highcost_H3/abs4n40","highcost_H3/abs5n40",
        "highcost_H3/abs1n45","highcost_H3/abs2n45","highcost_H3/abs3n45","highcost_H3/abs4n45","highcost_H3/abs5n45",
        "highcost_H3/abs1n50","highcost_H3/abs2n50","highcost_H3/abs3n50","highcost_H3/abs4n50","highcost_H3/abs5n50",

        "lowcost_H6/abs1n5","lowcost_H6/abs2n5","lowcost_H6/abs3n5","lowcost_H6/abs4n5","lowcost_H6/abs5n5",
        "lowcost_H6/abs1n10","lowcost_H6/abs2n10","lowcost_H6/abs3n10","lowcost_H6/abs4n10","lowcost_H6/abs5n10",
        "lowcost_H6/abs1n15","lowcost_H6/abs2n15","lowcost_H6/abs3n15","lowcost_H6/abs4n15","lowcost_H6/abs5n15",
        "lowcost_H6/abs1n20","lowcost_H6/abs2n20","lowcost_H6/abs3n20","lowcost_H6/abs4n20","lowcost_H6/abs5n20",
        "lowcost_H6/abs1n25","lowcost_H6/abs2n25","lowcost_H6/abs3n25","lowcost_H6/abs4n25","lowcost_H6/abs5n25",
        "lowcost_H6/abs1n30","lowcost_H6/abs2n30","lowcost_H6/abs3n30","lowcost_H6/abs4n30","lowcost_H6/abs5n30",

        "highcost_H6/abs1n5","highcost_H6/abs2n5","highcost_H6/abs3n5","highcost_H6/abs4n5","highcost_H6/abs5n5",
        "highcost_H6/abs1n10","highcost_H6/abs2n10","highcost_H6/abs3n10","highcost_H6/abs4n10","highcost_H6/abs5n10",
        "highcost_H6/abs1n15","highcost_H6/abs2n15","highcost_H6/abs3n15","highcost_H6/abs4n15","highcost_H6/abs5n15",
        "highcost_H6/abs1n20","highcost_H6/abs2n20","highcost_H6/abs3n20","highcost_H6/abs4n20","highcost_H6/abs5n20",
        "highcost_H6/abs1n25","highcost_H6/abs2n25","highcost_H6/abs3n25","highcost_H6/abs4n25","highcost_H6/abs5n25",
        "highcost_H6/abs1n30","highcost_H6/abs2n30","highcost_H6/abs3n30","highcost_H6/abs4n30","highcost_H6/abs5n30",

        "large_lowcost/abs1n50","large_lowcost/abs2n50","large_lowcost/abs3n50","large_lowcost/abs4n50","large_lowcost/abs5n50",
        "large_lowcost/abs6n50","large_lowcost/abs7n50","large_lowcost/abs8n50","large_lowcost/abs9n50","large_lowcost/abs10n50",
        "large_lowcost/abs1n100","large_lowcost/abs2n100","large_lowcost/abs3n100","large_lowcost/abs4n100","large_lowcost/abs5n100",
        "large_lowcost/abs6n100","large_lowcost/abs7n100","large_lowcost/abs8n100","large_lowcost/abs9n100","large_lowcost/abs10n100",
        "large_lowcost/abs1n200","large_lowcost/abs2n200","large_lowcost/abs3n200","large_lowcost/abs4n200","large_lowcost/abs5n200",
        "large_lowcost/abs6n200","large_lowcost/abs7n200","large_lowcost/abs8n200","large_lowcost/abs9n200","large_lowcost/abs10n200",

        "large_highcost/abs1n50","large_highcost/abs2n50","large_highcost/abs3n50","large_highcost/abs4n50","large_highcost/abs5n50",
        "large_highcost/abs6n50","large_highcost/abs7n50","large_highcost/abs8n50","large_highcost/abs9n50","large_highcost/abs10n50",
        "large_highcost/abs1n100","large_highcost/abs2n100","large_highcost/abs3n100","large_highcost/abs4n100","large_highcost/abs5n100",
        "large_highcost/abs6n100","large_highcost/abs7n100","large_highcost/abs8n100","large_highcost/abs9n100","large_highcost/abs10n100",
        "large_highcost/abs1n200","large_highcost/abs2n200","large_highcost/abs3n200","large_highcost/abs4n200","large_highcost/abs5n200",
        "large_highcost/abs6n200","large_highcost/abs7n200","large_highcost/abs8n200","large_highcost/abs9n200","large_highcost/abs10n200",
    };

    auto fileExists = [](const string &path) {
        ifstream fs(path);
        return static_cast<bool>(fs);
    };
    for (int i = 0; i < 220; ++i) {
        //if (fileExists(InstanceDir() + "Pb/Instances_" + instanceName[i] + ".json")) { continue; }
        convertInstanceToPb("Instances_" + instanceName[i]);
    }
}

bool Simulator::importBenchmarksFromCsv() {
    String instanceSets[] = { "lowcost_H3","lowcost_H6","highcost_H3","highcost_H6","large_lowcost","large_highcost" };

    for (int i = 0; i < 6; ++i) {
        ostringstream path;
        path << InstanceDir() << instanceSets[i] << ".csv";
        ifstream ifs(path.str());

        if (!ifs.is_open()) { return false; }

        String buf;
        ifs >> buf;
        while (!buf.empty()) {
            istringstream iss;
            int index = buf.find_first_of(',');
            String instanceName = buf.substr(0, index);
            double bestObj, referenceObj, referenceTime;
            iss.str(buf.substr(index + 1, buf.find_first_of(',', index + 1)));
            iss >> referenceObj;
            index = buf.find_first_of(',', index + 1);
            iss.str(buf.substr(index + 1, buf.find_first_of(',', index + 1)));
            iss >> bestObj;
            index = buf.find_first_of(',', index + 1);
            iss.str(buf.substr(index + 1));
            iss >> referenceTime;
            benchmarks[instanceSets[i] + "/" + instanceName] = { round(bestObj * 100) / 100, round(referenceObj * 100) / 100, referenceTime };
            buf.clear();
            ifs >> buf;
        }
    }
    
    return true;
}

}
