#include "lkhSolver.h"

#include <future>
#include <functional>
#include <fstream>
#include <sstream>
#include <iostream>

#include "../LKH3/LKH.h"
#include "../LKH3/Genetic.h"
#include "../LKH3/BIT.h"

#include "Input.h"


using namespace std;


namespace szx {

int lhkMain(lkh::Tour &sln, const lkh::Tour &hintSln) {
    if (!hintSln.nodes.empty()) {
        // EXTEND[szx][0]: utilize initial solution.
    }

    GainType Cost, OldOptimum;
    double Time, LastTime = GetTime();
    Node *N;
    int i;

    if (SubproblemSize > 0) {
        if (DelaunayPartitioning)
            SolveDelaunaySubproblems();
        else if (KarpPartitioning)
            SolveKarpSubproblems();
        else if (KCenterPartitioning)
            SolveKCenterSubproblems();
        else if (KMeansPartitioning)
            SolveKMeansSubproblems();
        else if (RohePartitioning)
            SolveRoheSubproblems();
        else if (MoorePartitioning || SierpinskiPartitioning)
            SolveSFCSubproblems();
        else
            SolveTourSegmentSubproblems();
        return EXIT_SUCCESS;
    }
    AllocateStructures();
    if (ProblemType == TSPTW)
        TSPTW_Reduce();
    if (ProblemType == VRPB || ProblemType == VRPBTW)
        VRPB_Reduce();
    if (ProblemType == PDPTW)
        PDPTW_Reduce();
    CreateCandidateSet();
    InitializeStatistics();

    if (Norm != 0 || Penalty) {
        Norm = 9999;
        BestCost = PLUS_INFINITY;
        BestPenalty = CurrentPenalty = PLUS_INFINITY;
    } else {
        /* The ascent has solved the problem! */
        Optimum = BestCost = (GainType)LowerBound;
        UpdateStatistics(Optimum, GetTime() - LastTime);
        RecordBetterTour();
        RecordBestTour();
        CurrentPenalty = PLUS_INFINITY;
        BestPenalty = CurrentPenalty = Penalty ? Penalty() : 0;
        WriteTour(OutputTourFileName, BestTour, BestCost);
        WriteTour(TourFileName, BestTour, BestCost);
        Runs = 0;
    }

    /* Find a specified number (Runs) of local optima */

    for (Run = 1; Run <= Runs; Run++) {
        LastTime = GetTime();
        Cost = FindTour();      /* using the Lin-Kernighan heuristic */
        if (MaxPopulationSize > 1 && !TSPTW_Makespan) {
            /* Genetic algorithm */
            int i;
            for (i = 0; i < PopulationSize; i++) {
                GainType OldPenalty = CurrentPenalty;
                GainType OldCost = Cost;
                Cost = MergeTourWithIndividual(i);
                if (TraceLevel >= 1 &&
                    (CurrentPenalty < OldPenalty ||
                    (CurrentPenalty == OldPenalty && Cost < OldCost))) {
                    if (CurrentPenalty)
                        printff("  Merged with %d: Cost = " GainFormat,
                            i + 1, Cost);
                    else
                        printff("  Merged with %d: Cost = " GainFormat "_"
                            GainFormat, i + 1, CurrentPenalty, Cost);
                    if (Optimum != MINUS_INFINITY && Optimum != 0) {
                        if (ProblemType != CCVRP && ProblemType != TRP &&
                            ProblemType != MLP &&
                            MTSPObjective != MINMAX &&
                            MTSPObjective != MINMAX_SIZE)
                            printff(", Gap = %0.4f%%",
                                100.0 * (Cost - Optimum) / Optimum);
                        else
                            printff(", Gap = %0.4f%%",
                                100.0 * (CurrentPenalty - Optimum) /
                                Optimum);
                    }
                    printff("\n");
                }
            }
            if (!HasFitness(CurrentPenalty, Cost)) {
                if (PopulationSize < MaxPopulationSize) {
                    AddToPopulation(CurrentPenalty, Cost);
                    if (TraceLevel >= 1)
                        PrintPopulation();
                } else if (SmallerFitness(CurrentPenalty, Cost,
                    PopulationSize - 1)) {
                    i = ReplacementIndividual(CurrentPenalty, Cost);
                    ReplaceIndividualWithTour(i, CurrentPenalty, Cost);
                    if (TraceLevel >= 1)
                        PrintPopulation();
                }
            }
        } else if (Run > 1 && !TSPTW_Makespan)
            Cost = MergeTourWithBestTour();
        if (CurrentPenalty < BestPenalty ||
            (CurrentPenalty == BestPenalty && Cost < BestCost)) {
            BestPenalty = CurrentPenalty;
            BestCost = Cost;
            RecordBetterTour();
            RecordBestTour();
            WriteTour(TourFileName, BestTour, BestCost);
        }
        OldOptimum = Optimum;
        if (!Penalty ||
            (MTSPObjective != MINMAX && MTSPObjective != MINMAX_SIZE)) {
            if (CurrentPenalty == 0 && Cost < Optimum)
                Optimum = Cost;
        } else if (CurrentPenalty < Optimum)
            Optimum = CurrentPenalty;
        if (Optimum < OldOptimum) {
            printff("*** New OPTIMUM = " GainFormat " ***\n\n", Optimum);
            if (FirstNode->InputSuc) {
                Node *N = FirstNode;
                while ((N = N->InputSuc = N->Suc) != FirstNode);
            }
        }
        Time = fabs(GetTime() - LastTime);
        UpdateStatistics(Cost, Time);
        if (TraceLevel >= 1 && Cost != PLUS_INFINITY) {
            printff("Run %d: ", Run);
            StatusReport(Cost, LastTime, "");
            printff("\n");
        }
        if (StopAtOptimum && MaxPopulationSize >= 1) {
            if (ProblemType != CCVRP && ProblemType != TRP &&
                ProblemType != MLP &&
                MTSPObjective != MINMAX &&
                MTSPObjective != MINMAX_SIZE ?
                CurrentPenalty == 0 && Cost == Optimum :
                CurrentPenalty == Optimum) {
                Runs = Run;
                break;
            }
        }
        if (PopulationSize >= 2 &&
            (PopulationSize == MaxPopulationSize ||
                Run >= 2 * MaxPopulationSize) && Run < Runs) {
            Node *N;
            int Parent1, Parent2;
            Parent1 = LinearSelection(PopulationSize, 1.25);
            do
                Parent2 = LinearSelection(PopulationSize, 1.25);
            while (Parent2 == Parent1);
            ApplyCrossover(Parent1, Parent2);
            N = FirstNode;
            do {
                if (ProblemType != HCP && ProblemType != HPP) {
                    int d = C(N, N->Suc);
                    AddCandidate(N, N->Suc, d, INT_MAX);
                    AddCandidate(N->Suc, N, d, INT_MAX);
                }
                N = N->InitialSuc = N->Suc;
            } while (N != FirstNode);
        }
        SRandom(++Seed);
    }
    if (TraceLevel > 0) { PrintStatistics(); }
    if (Salesmen > 1) {
        if (Dimension == DimensionSaved) {
            for (i = 1; i <= Dimension; i++) {
                N = &NodeSet[BestTour[i - 1]];
                (N->Suc = &NodeSet[BestTour[i]])->Pred = N;
            }
        } else {
            for (i = 1; i <= DimensionSaved; i++) {
                Node *N1 = &NodeSet[BestTour[i - 1]];
                Node *N2 = &NodeSet[BestTour[i]];
                Node *M1 = &NodeSet[N1->Id + DimensionSaved];
                Node *M2 = &NodeSet[N2->Id + DimensionSaved];
                (M1->Suc = N1)->Pred = M1;
                (N1->Suc = M2)->Pred = N1;
                (M2->Suc = N2)->Pred = M2;
            }
        }
        CurrentPenalty = BestPenalty;
        MTSP_Report(BestPenalty, BestCost);
        MTSP_WriteSolution(MTSPSolutionFileName, BestPenalty, BestCost);
        SINTEF_WriteSolution(SINTEFSolutionFileName, BestCost);
    }
    if (ProblemType == ACVRP ||
        ProblemType == BWTSP ||
        ProblemType == CCVRP ||
        ProblemType == CTSP ||
        ProblemType == CVRP ||
        ProblemType == CVRPTW ||
        ProblemType == MLP ||
        ProblemType == M_PDTSP ||
        ProblemType == M1_PDTSP ||
        MTSPObjective != -1 ||
        ProblemType == ONE_PDTSP ||
        ProblemType == OVRP ||
        ProblemType == PDTSP ||
        ProblemType == PDTSPL ||
        ProblemType == PDPTW ||
        ProblemType == RCTVRP ||
        ProblemType == RCTVRPTW ||
        ProblemType == SOP ||
        ProblemType == TRP ||
        ProblemType == TSPTW ||
        ProblemType == VRPB ||
        ProblemType == VRPBTW || ProblemType == VRPPD) {
        printff("Best %s solution:\n", Type);
        CurrentPenalty = BestPenalty;
        SOP_Report(BestCost);
    }

    // retrieve solution path.
    sln.distance = BestCost;
    if (Salesmen > 1) {
        // TODO[szx][0]: handle multi-vehicle instances.
        //Node *N = Depot, *NextN;
        //bool Forward = N->Suc->Id != N->Id + DimensionSaved;
        //do {
        //    GainType Sum = 0;
        //    do {
        //        sln.tours.back().nodes.push_back(N->Id <= Dim ? N->Id : Depot->Id);
        //        NextN = Forward ? N->Suc : N->Pred;
        //        Sum += C(N, NextN) - N->Pi - NextN->Pi;
        //        if (NextN->Id > DimensionSaved)
        //            NextN = Forward ? NextN->Suc : NextN->Pred;
        //        N = NextN;
        //    } while (N->DepotId == 0);
        //    sln.tours.back().distance = Sum / Precision;
        //} while (N != Depot);
    } else {
        int n = DimensionSaved;
        sln.nodes.resize(n);
        for (i = 1; (i < n) && (BestTour[i] != 1); i++) {}
        bool forward = Asymmetric ||
            (BestTour[i < n ? i + 1 : 1] < BestTour[i > 1 ? i - 1 : Dimension]);
        for (int j = 0; j < n; j++) {
            if (BestTour[i] <= n) {
                sln.nodes[j] = (BestTour[i] - lkh::LkhIdBase);
            }
            if (forward) {
                if (++i > n) { i = 1; }
            } else if (--i < 1) {
                i = n;
            }
        }
    }

    return EXIT_SUCCESS;
}


namespace lkh {

bool solveTsp(Tour &sln, const CoordList2D &coordList, const Tour &hintSln) {
    auto getLocalTime = [](const char *format = "%Y-%m-%d(%a)%H:%M:%S")->const char* {
        static constexpr int DateBufSize = 64;
        static char buf[DateBufSize];
        time_t t = time(NULL);
        tm *date = localtime(&t);
        strftime(buf, DateBufSize, format, date);
        return buf;
    };
    Environment::load();
    Configuration::load();
    Problem::load(coordList);
    system("mkdir \"lkh/ \" 2>  nul");
    system("mkdir \"lkh/tmp/ \" 2>  nul");
    system("mkdir \"lkh/sln/ \" 2>  nul");

    // parse input file for lkh
    string localTime = getLocalTime("%Y%m%d%H%M%S");
    ostringstream tspFilePath;
    tspFilePath << "lkh/tmp/" << localTime << ".tsp";///Users/douglaslee/Documents/codes/IRP2.0/pr
    fstream tspFile(tspFilePath.str(), ios::out);
    if (tspFile.bad()) {
        cout << "Failed to open file." << endl;
    }
    ostringstream oss;
    oss << "NAME : pr" << endl
        << "TYPE : TSP" << endl
        << " DIMENSION : " << coordList.size() << endl
        << "EDGE_WEIGHT_TYPE : EUC_2D" << endl
        << "NODE_COORD_SECTION" << endl;

    for (int i = 0; i < coordList.size(); ++i) {
        oss << (i + 1) << ' ' << coordList[i].x << ' ' << coordList[i].y << endl;
    }
    tspFile << oss.str() << "EOF";
    tspFile.close();

    // write to par file.
    ostringstream parFilePath;
    parFilePath << "lkh/tmp/" << localTime << ".par";
    fstream parFile(parFilePath.str(), ios::out);
    if (parFile.bad()) {
        cout << "Failed to open file." << endl;
    }
    ostringstream slnFilePath;
    slnFilePath << "lkh/sln/" << localTime << ".txt";
    parFile << "PROBLEM_FILE = " << tspFilePath.str() << "\n"
        << "MOVE_TYPE = " << 5 << "\n"
        << "PATCHING_C = " << 3 << "\n"
        << "PATCHING_A = " << 2 << "\n"
        << "POPULATION_SIZE=" << 2 << "\n"
        << "RUNS = " << 10 << "\n"
        << "POPULATION_SIZE=" << 2 << "\n"
        << "OUTPUT_TOUR_FILE = " << slnFilePath;
    parFile.close();

    ostringstream cmd;
    cmd << "LKH3.exe " << parFilePath.str();
    if (system(cmd.str().c_str()) != EXIT_SUCCESS) { return false; }

    // delete temp file
    remove(parFilePath.str().c_str());
    remove(tspFilePath.str().c_str());

    // read solution
    fstream fs(slnFilePath.str());
    string buf;
    while (fs >> buf) {
        if (buf == "Length") {
            fs >> buf >> buf;
            sln.distance = atoi(buf.c_str());
        }

        if (buf == "TOUR_SECTION") {
            for (int i = 0; i < coordList.size(); ++i) {
                fs >> buf;
                sln.nodes.push_back(atoi(buf.c_str()) - 1);
            }
            break;
        }
    }
    fs.close();
    remove(slnFilePath.str().c_str());
    return true;
    //return async(launch::async, [&]() {
    //    Environment::load();
    //    Configuration::load();
    //    Problem::load(coordList);
    //    return lhkMain(sln, hintSln);
    //}).get() == EXIT_SUCCESS;
}

bool solveTsp(Tour &sln, const CoordList3D &coordList, const Tour &hintSln) {
    return async(launch::async, [&]() {
        Environment::load();
        Configuration::load();
        Problem::load(coordList);
        return lhkMain(sln, hintSln);
    }).get() == EXIT_SUCCESS;
}

bool solveTsp(Tour &sln, const AdjMat &adjMat, const Tour &hintSln) {
    return async(launch::async, [&]() {
        Environment::load();
        Configuration::load();
        Problem::load(adjMat);
        return lhkMain(sln, hintSln);
    }).get() == EXIT_SUCCESS;
}

bool solveTsp(Tour &sln, const AdjList &adjList, const Tour &hintSln) {
    throw "not supported yet.";

    return async(launch::async, [&]() {
        Environment::load();
        Configuration::load();
        Problem::load(adjList);
        return lhkMain(sln, hintSln);
    }).get() == EXIT_SUCCESS;
}

bool solveTsp(Tour &sln, const EdgeList &edgeList, Graph::ID nodeNum, const Tour &hintSln) {
    throw "not supported yet.";

    return async(launch::async, [&]() {
        Environment::load();
        Configuration::load();
        Problem::load(edgeList, nodeNum);
        return lhkMain(sln, hintSln);
    }).get() == EXIT_SUCCESS;
}

}
}
