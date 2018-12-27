////////////////////////////////
/// usage : 1.	data interface for LKH.
/// 
/// note  : 1.	
////////////////////////////////

#ifndef SMART_SZX_GOAL_LKH3LIB_INPUT_H
#define SMART_SZX_GOAL_LKH3LIB_INPUT_H


#include <algorithm>
#include <functional>
#include <string>
#include <vector>

#include "Graph.h"
#include "TspSolver.h"

#include "../LKH3/LKH.h"
#include "../LKH3/Genetic.h"
#include "../LKH3/Heap.h"


namespace szx {
namespace lkh {

struct Environment { // controls the general behavior of the solver, e.g., I/O path, timeout or seed.
    static void load() {

    }
};


struct Configuration { // tunes the performance of the core algorithm.
    static void load() {
        //CandidateFiles = ;
        //CandidateFileName = ;

        //EdgeFiles = ;
        //EdgeFileName = ;

        //InitialTourFileName = ;

        //InputTourFileName = ;

        //MergeTourFiles = ;
        //MergeTourFileName = ;

        //MTSPSolutionFileName = ;

        //SINTEFSolutionFileName = ;

        //SubproblemTourFileName = ;

        //TourFileName = ;

        ProblemFileName = 0;
        PiFileName = 0;
        InputTourFileName = 0;
        OutputTourFileName = 0;
        TourFileName = 0;
        CandidateFiles = 0;
        MergeTourFiles = 0;

        AscentCandidates = 50;
        BackboneTrials = 0;
        Backtracking = 0;
        BWTSP_B = 0;
        BWTSP_Q = 0;
        BWTSP_L = INT_MAX;
        CandidateSetSymmetric = 0;
        CandidateSetType = ALPHA;
        Crossover = ERXT;
        DelaunayPartitioning = 0;
        DelaunayPure = 0;
        DemandDimension = 1;
        Excess = -1;
        ExternalSalesmen = 0;
        ExtraCandidates = 0;
        ExtraCandidateSetSymmetric = 0;
        ExtraCandidateSetType = QUADRANT;
        Gain23Used = 1;
        GainCriterionUsed = 1;
        GridSize = 1000000.0;
        InitialPeriod = -1;
        InitialStepSize = 0;
        InitialTourAlgorithm = WALK;
        InitialTourFraction = 1.0;
        KarpPartitioning = 0;
        KCenterPartitioning = 0;
        KMeansPartitioning = 0;
        Kicks = 1;
        KickType = 0;
        MaxBreadth = INT_MAX;
        MaxCandidates = 5;
        MaxPopulationSize = 0;
        MaxSwaps = -1;
        MaxTrials = -1;
        MoorePartitioning = 0;
        MoveType = 5;
        MoveTypeSpecial = 0;
        MTSPDepot = 1;
        MTSPMinSize = 1;
        MTSPMaxSize = -1;
        MTSPObjective = -1;
        NonsequentialMoveType = -1;
        Optimum = MINUS_INFINITY;
        PatchingA = 1;
        PatchingC = 0;
        PatchingAExtended = 0;
        PatchingARestricted = 0;
        PatchingCExtended = 0;
        PatchingCRestricted = 0;
        POPMUSIC_InitialTour = 0;
        POPMUSIC_MaxNeighbors = 5;
        POPMUSIC_SampleSize = 10;
        POPMUSIC_Solutions = 50;
        POPMUSIC_Trials = 1;
        Recombination = IPT;
        RestrictedSearch = 1;
        RohePartitioning = 0;
        Runs = 0;
        Salesmen = 1;
        Seed = 1;
        SierpinskiPartitioning = 0;
        StopAtOptimum = 1;
        Subgradient = 1;
        SubproblemBorders = 0;
        SubproblemsCompressed = 0;
        SubproblemSize = 0;
        SubsequentMoveType = 0;
        SubsequentMoveTypeSpecial = 0;
        SubsequentPatching = 1;
        TimeLimit = DBL_MAX;
        TraceLevel = 0;
        TSPTW_Makespan = 0;

        //if (AscentCandidates < 2) { eprintf("ASCENT_CANDIDATES: >= 2 expected"); }

        //if (BackboneTrials < 0) { eprintf("BACKBONE_TRIALS: non-negative integer expected"); }

        //if ((Backtracking < 0) || (Backtracking > 1)) { eprintf("BACKTRACKING: YES or NO expected"); }

        //if (BWTSP_B <= 0) { eprintf("BWTSP: positive integer expected"); }
        //if (BWTSP_Q <= 0) { eprintf("BWTSP: positive integer expected"); }
        //if (BWTSP_L <= 0) { eprintf("BWTSP: positive integer expected"); }

        //CandidateSetType = ALPHA;
        //CandidateSetType = DELAUNAY;
        //CandidateSetType = NN;
        //CandidateSetType = POPMUSIC;
        //CandidateSetType = QUADRANT;
        //if (CandidateSetType == DELAUNAY) {
        //    DelaunayPure = 1;
        //}

        //if (MTSPDepot <= 0) { eprintf("DEPOT: positive integer expected"); }

        //if (Excess < 0) { eprintf("EXCESS: non-negeative real expected"); }

        //if (ExternalSalesmen < 0) { eprintf("EXTERNAL_SALESMEN: non-negative integer expected"); }

        //if (ExtraCandidates < 0) { eprintf("EXTRA_CANDIDATES: non-negative integer expected"); }
        //ExtraCandidateSetSymmetric = 1;

        //ExtraCandidateSetType = NN;
        //ExtraCandidateSetType = QUADRANT;

        //if ((Gain23Used < 0) || (Gain23Used > 1)) { eprintf("GAIN23: YES or NO expected"); }

        //if ((GainCriterionUsed < 0) || (GainCriterionUsed > 1)) { eprintf("GAIN_CRITERION: YES or NO expected"); }

        //if (InitialPeriod < 0) { eprintf("INITIAL_PERIOD: non-negative integer expected"); }

        //if (InitialStepSize <= 0) { eprintf("INITIAL_STEP_SIZE: positive integer expected"); }

        //InitialTourAlgorithm = BORUVKA;
        //InitialTourAlgorithm = CTSP_ALG;
        //InitialTourAlgorithm = CVRP_ALG;
        //InitialTourAlgorithm = GREEDY;
        //InitialTourAlgorithm = MOORE;
        //InitialTourAlgorithm = MTSP_ALG;
        //InitialTourAlgorithm = NEAREST_NEIGHBOR;
        //InitialTourAlgorithm = QUICK_BORUVKA;
        //InitialTourAlgorithm = SIERPINSKI;
        //InitialTourAlgorithm = SOP_ALG;
        //InitialTourAlgorithm = TSPDL_ALG;
        //InitialTourAlgorithm = WALK;

        //if ((InitialTourFraction < 0) || (InitialTourFraction > 1)) { eprintf("INITIAL_TOUR_FRACTION: >= 0 or <= 1 expected"); }

        //if ((KickType != 0) && (KickType < 4)) { eprintf("KICK_TYPE: integer >= 4 expected"); }

        //if (Kicks < 0) { eprintf("KICKS: non-negative integer expected"); }

        //if ((TSPTW_Makespan < 0) || (TSPTW_Makespan > 1)) { eprintf("MAKESPAN: YES or NO expected"); }

        //if (MaxBreadth < 0) { eprintf("MAX_BREADTH: non-negative integer expected"); }

        //if (MaxCandidates < 0) { eprintf("MAX_CANDIDATES: non-negative integer expected"); }
        //CandidateSetSymmetric = 1;

        //if (MaxSwaps < 0) { eprintf("MAX_SWAPS: non-negative integer expected"); }

        //if (MaxTrials < 0) { eprintf("MAX_TRIALS: non-negative integer expected"); }

        //if (MoveType < 2) { eprintf("MOVE_TYPE: >= 2 expected"); }
        //MoveTypeSpecial = 0;
        //if (MoveTypeSpecial == 1) {
        //    if (MoveType != 3 && MoveType != 5) { eprintf("(MOVE_TYPE) SPECIAL, MOVE_TYPE must be 3 or 5"); }
        //}

        //if (MTSPMaxSize <= 0) { eprintf("MTSP_MAX_SIZE: positive integer expected"); }

        //MTSPObjective = MINMAX;
        //MTSPObjective = MINMAX_SIZE;
        //MTSPObjective = MINSUM;

        //if (NonsequentialMoveType < 4) { eprintf("NONSEQUENTIAL_MOVE_TYPE: >= 4 expected"); }

        //if (PatchingA < 0) { eprintf("PATCHING_A: non-negative integer expected"); }
        //PatchingARestricted = 1;
        //PatchingAExtended = 1;

        //if (PatchingC < 0) { eprintf("PATCHING_C: non-negative integer expected"); }
        //PatchingCRestricted = 1;
        //PatchingCExtended = 1;

        //if ((POPMUSIC_InitialTour < 0) || (POPMUSIC_InitialTour > 1)) { eprintf("POPMUSIC_INITIAL_TOUR: YES or NO expected"); }

        //if (POPMUSIC_MaxNeighbors <= 0) { eprintf("POPMUSIC_MAX_NEIGHBORS: positive integer expected"); }

        //if (POPMUSIC_SampleSize <= 0) { eprintf("POPMUSIC_SAMPLE_SIZE: positive integer expected"); }

        //if (POPMUSIC_Solutions <= 0) { eprintf("POPMUSIC_SOLUTIONS: positive integer expected"); }

        //if (POPMUSIC_Trials < 0) { eprintf("POPMUSIC_TRIALS: non-negative integer expected"); }

        //Recombination = IPT;
        //Recombination = GPX2;

        //if ((RestrictedSearch < 0) || (RestrictedSearch > 1)) { eprintf("RESTRICTED_SEARCH: YES or NO expected"); }

        //if (Runs <= 0) { eprintf("RUNS: positive integer expected"); }

        //if (Salesmen <= 0) { eprintf("VEHICLE: positive integer expected"); }

        //if (!strcmp(Keyword, "SPECIAL")) {
        //    Gain23Used = 0;
        //    KickType = 4;
        //    MaxSwaps = 0;
        //    MoveType = 5;
        //    MoveTypeSpecial = 1;
        //    MaxPopulationSize = 10;
        //}

        //if ((StopAtOptimum < 0) || (StopAtOptimum > 1)) { eprintf("STOP_AT_OPTIMUM: YES or NO expected"); }

        //if ((Subgradient < 0) || (Subgradient > 1)) { eprintf("SUBGRADIENT: YES or NO expected"); }

        //if (SubproblemSize < 3) { eprintf("SUBPROBLEM_SIZE: >= 3 expected"); }
        //DelaunayPartitioning = 1;
        //KarpPartitioning = 1;
        //KCenterPartitioning = 1;
        //KMeansPartitioning = 1;
        //MoorePartitioning = 1;
        //RohePartitioning = 1;
        //SierpinskiPartitioning = 1;
        //SubproblemBorders = 1;
        //SubproblemsCompressed = 1;

        //if ((SubsequentMoveType != 0) && (SubsequentMoveType < 2)) { eprintf("SUBSEQUENT_MOVE_TYPE: 0 or >= 2 expected"); }
        //SubsequentMoveTypeSpecial = 0;
        //if (SubsequentMoveTypeSpecial == 1) {
        //    if ((SubsequentMoveType != 3) && (SubsequentMoveType != 5)) {
        //        eprintf("(SUBSEQUENT_MOVE_TYPE) SPECIAL, SUBSEQUENT_MOVE_TYPE must be 3 or 5");
        //    }
        //}

        //if ((SubsequentPatching < 0) || (SubsequentPatching > 1)) { eprintf("SUBSEQUENT_PATCHING: YES or NO expected"); }

        //if (TimeLimit < 0) { eprintf("TIME_LIMIT: >= 0 expected"); }

        //if (!ProblemFileName) { eprintf("Problem file name is missing"); }
        //if (SubproblemSize == 0 && SubproblemTourFileName != 0) { eprintf("SUBPROBLEM_SIZE specification is missing"); }
        //if (SubproblemSize > 0 && SubproblemTourFileName == 0) { eprintf("SUBPROBLEM_TOUR_FILE specification is missing"); }

        postprocess();
    }

    static void postprocess() {
        MaxMatrixDimension = 20000;
        MergeWithTour = Recombination == IPT ? MergeWithTourIPT : MergeWithTourGPX2;
    }
};


struct Problem {
    #pragma region Setter
    #pragma region SetEdgeWeightType
    static void setEdgeWeightType_EXPLICIT() {
        WeightType = EXPLICIT;
        Distance = Distance_EXPLICIT;
    }

    #pragma region CoordBased
    static void setEdgeWeightType_ATT() {
        WeightType = ATT;
        Distance = Distance_ATT;
        c = c_ATT;
        CoordType = TWOD_COORDS;
    }
    static void setEdgeWeightType_CEIL_2D() {
        WeightType = CEIL_2D;
        Distance = Distance_CEIL_2D;
        c = c_CEIL_2D;
        CoordType = TWOD_COORDS;
    }
    static void setEdgeWeightType_CEIL_3D() {
        WeightType = CEIL_3D;
        Distance = Distance_CEIL_3D;
        c = c_CEIL_3D;
        CoordType = THREED_COORDS;
    }
    static void setEdgeWeightType_EUC_2D() {
        WeightType = EUC_2D;
        Distance = Distance_EUC_2D;
        c = c_EUC_2D;
        CoordType = TWOD_COORDS;
    }
    static void setEdgeWeightType_EUC_3D() {
        WeightType = EUC_3D;
        Distance = Distance_EUC_3D;
        c = c_EUC_3D;
        CoordType = THREED_COORDS;
    }
    static void setEdgeWeightType_FLOOR_2D() {
        WeightType = FLOOR_2D;
        Distance = Distance_FLOOR_2D;
        c = c_FLOOR_2D;
        CoordType = TWOD_COORDS;
    }
    static void setEdgeWeightType_FLOOR_3D() {
        WeightType = FLOOR_3D;
        Distance = Distance_FLOOR_3D;
        c = c_FLOOR_3D;
        CoordType = THREED_COORDS;
    }
    static void setEdgeWeightType_MAN_2D() {
        WeightType = MAN_2D;
        Distance = Distance_MAN_2D;
        CoordType = TWOD_COORDS;
    }
    static void setEdgeWeightType_MAN_3D() {
        WeightType = MAN_3D;
        Distance = Distance_MAN_3D;
        CoordType = THREED_COORDS;
    }
    static void setEdgeWeightType_MAX_2D() {
        WeightType = MAX_2D;
        Distance = Distance_MAX_2D;
        CoordType = TWOD_COORDS;
    }
    static void setEdgeWeightType_MAX_3D() {
        WeightType = MAX_3D;
        Distance = Distance_MAX_3D;
        CoordType = THREED_COORDS;
    }
    static void setEdgeWeightType_GEO() {
        WeightType = GEO;
        Distance = Distance_GEO;
        c = c_GEO;
        CoordType = TWOD_COORDS;
    }
    static void setEdgeWeightType_GEOM() {
        WeightType = GEOM;
        Distance = Distance_GEOM;
        c = c_GEOM;
        CoordType = TWOD_COORDS;
    }
    static void setEdgeWeightType_GEO_MEEUS() {
        WeightType = GEO_MEEUS;
        Distance = Distance_GEO_MEEUS;
        c = c_GEO_MEEUS;
        CoordType = TWOD_COORDS;
    }
    static void setEdgeWeightType_GEOM_MEEUS() {
        WeightType = GEOM_MEEUS;
        Distance = Distance_GEOM_MEEUS;
        c = c_GEOM_MEEUS;
        CoordType = TWOD_COORDS;
    }
    static void setEdgeWeightType_TOR_2D() {
        WeightType = TOR_2D;
        Distance = Distance_TOR_2D;
        CoordType = TWOD_COORDS;
    }
    static void setEdgeWeightType_TOR_3D() {
        WeightType = TOR_3D;
        Distance = Distance_TOR_3D;
        CoordType = THREED_COORDS;
    }
    static void setEdgeWeightType_XRAY1() {
        WeightType = XRAY1;
        Distance = Distance_XRAY1;
        CoordType = THREED_COORDS;
    }
    static void setEdgeWeightType_XRAY2() {
        WeightType = XRAY2;
        Distance = Distance_XRAY2;
        CoordType = THREED_COORDS;
    }
    #pragma endregion CoordBased

    static void setEdgeWeightType_SPECIAL() {
        WeightType = SPECIAL;
        Distance = Distance_SPECIAL;
    }
    #pragma endregion SetEdgeWeightType

    #pragma region CopyCoord
    static void copyCoord2D(Node &n, const Coord2D &c) {
        n.X = c.x;
        n.Y = c.y;
    }
    static void copyCoord3D(Node &n, const Coord3D &c) {
        copyCoord2D(n, c);
        n.Z = c.z;
    }
    #pragma endregion CopyCoord

    static void setNodeNum(ID nodeNum) {
        Dimension = nodeNum;
        if (Dimension < 0) { eprintf("DIMENSION: < 0"); }
        DimensionSaved = Dim = Dimension;
    }
    static void setNodeNum(size_t nodeNum) { setNodeNum(static_cast<ID>(nodeNum)); }
    #pragma endregion Setter


    static void load(const AdjMat &adjMat) {
        preprocess(ATSP);

        setNodeNum(adjMat.size1());

        EdgeWeightType = Copy("EXPLICIT");
        setEdgeWeightType_EXPLICIT();

        CoordType = NO_COORDS;

        //if (EdgeWeightFormat && SubproblemTourFileName) { eprintf("EDGE_DATA_FORMAT cannot be used together with SUBPROBLEM_TOUR_FILE"); }

        WeightFormat = FULL_MATRIX;

        CheckSpecificationPart();
        if (!FirstNode) { CreateNodes(); }
        ID n = Dimension / 2;
        assert(CostMatrix = (int *)calloc((size_t)n * n, sizeof(int)));
        for (Node *Ni = FirstNode; Ni->Id <= n; Ni = Ni->Suc) {
            Ni->C = &CostMatrix[(size_t)(Ni->Id - 1) * n] - 1;
        }

        for (ID i = LkhIdBase; i <= Dim; i++) {
            Node *Ni = &NodeSet[i];
            for (ID j = LkhIdBase; j <= Dim; j++) {
                Weight W = std::min<Weight>(adjMat[i - LkhIdBase][j - LkhIdBase], INT_MAX / 2 / Precision);
                if (Penalty && (W < 0)) { eprintf("EDGE_WEIGHT_SECTION: Negative weight"); }
                Ni->C[j] = W;
                if ((j != i) && (W > M)) { M = W; }
            }
        }

        for (ID i = LkhIdBase; i <= DimensionSaved; i++) {
            FixEdge(&NodeSet[i], &NodeSet[i + DimensionSaved]);
        }
        if (ProblemType == SOP || ProblemType == M1_PDTSP) {
            NodeSet[n].C[1] = 0;
        }
        Distance = Distance_ATSP;
        WeightType = -1; // TODO[szx][0]: find out why to clear the weight type.

        postprocess();
    }

    static void load(const AdjMat &adjMat, EdgeWeightFormats weightFormat) {
        preprocess();

        setNodeNum(adjMat.size());

        EdgeWeightType = Copy("EXPLICIT");
        setEdgeWeightType_EXPLICIT();

        CoordType = NO_COORDS;

        //if (EdgeWeightFormat && SubproblemTourFileName) { eprintf("EDGE_DATA_FORMAT cannot be used together with SUBPROBLEM_TOUR_FILE"); }

        WeightFormat = weightFormat;
        switch (WeightFormat) {
        case UPPER_ROW: case LOWER_ROW:
            break;
        case FULL_MATRIX:
            return load(adjMat);
        default:
            eprintf("EDGE_WEIGHT_SECTION: Unsupported weight format");
            break;
        }

        CheckSpecificationPart();
        if (!FirstNode) { CreateNodes(); }

        if (Asymmetric) {
            Asymmetric = 0; // TODO[szx][0]: check if overriding `Asymmetric` will lead to bugs.
            eprintf("EDGE_WEIGHT_SECTION: Symmetric graph for asymmetric problem");
        }
        assert(CostMatrix = (int *)calloc((size_t)Dimension * (Dimension - 1) / 2, sizeof(int)));
        Node *Ni = FirstNode->Suc;
        do {
            Ni->C = &CostMatrix[(size_t)(Ni->Id - 1) * (Ni->Id - 2) / 2] - 1;
        } while ((Ni = Ni->Suc) != FirstNode);

        switch (WeightFormat) {
        case UPPER_ROW:
            for (ID i = LkhIdBase; i < Dim; i++) {
                for (ID j = i + LkhIdBase; j <= Dim; j++) {
                    Weight W = adjMat[i - LkhIdBase][j - LkhIdBase];
                    if (W > INT_MAX / 2 / Precision) { W = INT_MAX / 2 / Precision; }
                    if (Penalty && (W < 0)) { eprintf("EDGE_WEIGHT_SECTION: Negative weight"); }
                    NodeSet[j].C[i] = W;
                }
            }
            break;
        case LOWER_ROW:
            for (ID i = LkhIdBase + 1; i <= Dim; i++) {
                for (ID j = LkhIdBase; j < i; j++) {
                    Weight W = adjMat[i - LkhIdBase][j - LkhIdBase];
                    if (W > INT_MAX / 2 / Precision) { W = INT_MAX / 2 / Precision; }
                    if (Penalty && (W < 0)) { eprintf("EDGE_WEIGHT_SECTION: Negative weight"); }
                    NodeSet[i].C[j] = W;
                }
            }
            break;
        }

        postprocess();
    }

    static void load(const CoordList2D &coordList) {
        preprocess();

        setNodeNum(coordList.size());

        EdgeWeightType = Copy("EUC_2D");
        setEdgeWeightType_EUC_2D();

        NodeCoordType = Copy("TWOD_COORDS");
        CoordType = TWOD_COORDS;
        CheckSpecificationPart();
        if (!FirstNode) { CreateNodes(); }
        for (ID i = LkhIdBase; i <= Dim; i++) {
            copyCoord2D(NodeSet[i], coordList[i - LkhIdBase]);
        }
        if (Asymmetric) { Convert2FullMatrix(); }

        postprocess();
    }

    static void load(const CoordList3D &coordList) {
        preprocess();

        setNodeNum(coordList.size());

        EdgeWeightType = Copy("EUC_3D");
        setEdgeWeightType_EUC_3D();

        NodeCoordType = Copy("THREED_COORDS");
        CoordType = THREED_COORDS;
        CheckSpecificationPart();
        if (!FirstNode) { CreateNodes(); }
        for (ID i = LkhIdBase; i <= Dim; i++) {
            copyCoord3D(NodeSet[i], coordList[i - LkhIdBase]);
        }
        if (Asymmetric) { Convert2FullMatrix(); }

        postprocess();
    }

    #pragma region UnsupportedInterface
    // [NotSupportedByLKH].
    static void load(const AdjList &adjList) {
        throw "not supported yet. (the weight in the edge list used by LKH seems different from the weight in objective)";

        preprocess();

        ID nodeNum = adjList.size();
        setNodeNum(nodeNum);

        EdgeWeightType = Copy("EXPLICIT");
        setEdgeWeightType_EXPLICIT();

        CoordType = NO_COORDS;

        CheckSpecificationPart();
        if (!FirstNode) { CreateNodes(); }

        EdgeDataFormat = Copy("ADJ_LIST");
        ID dstOffset = (Asymmetric ? (Dimension / 2) : 0);
        ID s = LkhIdBase;
        for (auto l = adjList.begin(); l != adjList.end(); ++l, ++s) {
            Node *src = &NodeSet[s];
            for (auto n = l->begin(); n != l->end(); ++n) {
                if ((n->dst <= 0) || (n->dst > nodeNum)) { eprintf("(EDGE_DATA_SECTION) Node number out of range: %d", n->dst); }
                if (s == n->dst) { eprintf("(EDGE_DATA_SECTION) Illegal edge: %d to %d", s, n->dst); }
                Node *dst = &NodeSet[n->dst + dstOffset + LkhIdBase];
                Weight w = n->weight * Precision;
                AddCandidate(src, dst, w, 1);
                AddCandidate(dst, src, w, 1);
            }
        }

        if (Asymmetric) {
            for (ID i = 1; i <= DimensionSaved; i++) {
                FixEdge(&NodeSet[i], &NodeSet[i + DimensionSaved]);
            }
        }
        WeightType = 1; // TODO[szx][0]: find out why it is EUC_2D instead of EXPLICIT.
        MaxCandidates = ExtraCandidates = 0;
        Distance = Distance_LARGE; // TODO[szx][0]: find out why to return constant distance.

        postprocess();
    }

    // [NotSupportedByLKH].
    static void load(const Graph::AdjList<Graph::AdjNode> &adjList) {
        throw "not supported yet. (the edge weight in objective must be provided using adjacency matrix or coords)";

        preprocess();

        ID nodeNum = adjList.size();
        setNodeNum(nodeNum);

        EdgeWeightType = Copy("EXPLICIT");
        setEdgeWeightType_EXPLICIT();

        CoordType = NO_COORDS;

        CheckSpecificationPart();
        if (!FirstNode) { CreateNodes(); }

        EdgeDataFormat = Copy("ADJ_LIST");
        ID dstOffset = (Asymmetric ? (Dimension / 2) : 0);
        ID s = LkhIdBase;
        for (auto l = adjList.begin(); l != adjList.end(); ++l, ++s) {
            Node *src = &NodeSet[s];
            for (auto n = l->begin(); n != l->end(); ++n) {
                if ((n->dst <= 0) || (n->dst > nodeNum)) { eprintf("(EDGE_DATA_SECTION) Node number out of range: %d", n->dst); }
                if (s == n->dst) { eprintf("(EDGE_DATA_SECTION) Illegal edge: %d to %d", s, n->dst); }
                Node *dst = &NodeSet[n->dst + dstOffset + LkhIdBase];
                AddCandidate(src, dst, 0, 1);
                AddCandidate(dst, src, 0, 1);
            }
        }

        if (Asymmetric) {
            for (ID i = 1; i <= DimensionSaved; i++) {
                FixEdge(&NodeSet[i], &NodeSet[i + DimensionSaved]);
            }
        }
        WeightType = 1; // TODO[szx][0]: find out why it is EUC_2D instead of EXPLICIT.
        MaxCandidates = ExtraCandidates = 0;
        Distance = Distance_1; // TODO[szx][0]: find out why to return constant distance.

        postprocess();
    }

    // [NotSupportedByLKH].
    static void load(const EdgeList &edgeList, ID nodeNum) {
        throw "not supported yet. (the weight in the edge list used by LKH seems different from the weight in objective)";

        preprocess();

        setNodeNum(nodeNum);

        EdgeWeightType = Copy("EXPLICIT");
        setEdgeWeightType_EXPLICIT();

        CoordType = NO_COORDS;

        CheckSpecificationPart();
        if (!FirstNode) { CreateNodes(); }

        EdgeDataFormat = Copy("EDGE_LIST");
        ID dstOffset = (Asymmetric ? (Dimension / 2) : 0);
        for (auto e = edgeList.begin(); e != edgeList.end(); ++e) {
            if ((e->src <= 0) || (e->src > nodeNum)) { eprintf("(EDGE_DATA_SECTION) Node number out of range: %d", e->src); }
            if ((e->dst <= 0) || (e->dst > nodeNum)) { eprintf("(EDGE_DATA_SECTION) Node number out of range: %d", e->dst); }
            if (e->src == e->dst) { eprintf("(EDGE_DATA_SECTION) Illegal edge: %d to %d", e->src, e->dst); }
            Node *src = &NodeSet[e->src + LkhIdBase];
            Node *dst = &NodeSet[e->dst + dstOffset + LkhIdBase];
            Weight w = e->weight * Precision;
            AddCandidate(src, dst, w, 1); // TODO[szx][0]: find out what is the weight exactly.
            AddCandidate(dst, src, w, 1);
        }

        if (Asymmetric) {
            for (ID i = 1; i <= DimensionSaved; i++) {
                FixEdge(&NodeSet[i], &NodeSet[i + DimensionSaved]);
            }
        }
        WeightType = 1; // TODO[szx][0]: find out why it is EUC_2D instead of EXPLICIT.
        MaxCandidates = ExtraCandidates = 0;
        Distance = Distance_LARGE; // TODO[szx][0]: find out why to return constant distance.

        postprocess();
    }

    // [NotSupportedByLKH].
    static void load(const Graph::EdgeList<Graph::Edge> &edgeList, ID nodeNum) {
        throw "not supported yet. (the edge weight in objective must be provided using adjacency matrix or coords)";

        preprocess();

        setNodeNum(nodeNum);

        EdgeWeightType = Copy("EXPLICIT");
        setEdgeWeightType_EXPLICIT();

        CoordType = NO_COORDS;

        CheckSpecificationPart();
        if (!FirstNode) { CreateNodes(); }

        EdgeDataFormat = Copy("EDGE_LIST");
        ID dstOffset = (Asymmetric ? (Dimension / 2) : 0);
        for (auto e = edgeList.begin(); e != edgeList.end(); ++e) {
            if ((e->src <= 0) || (e->src > nodeNum)) { eprintf("(EDGE_DATA_SECTION) Node number out of range: %d", e->src); }
            if ((e->dst <= 0) || (e->dst > nodeNum)) { eprintf("(EDGE_DATA_SECTION) Node number out of range: %d", e->dst); }
            if (e->src == e->dst) { eprintf("(EDGE_DATA_SECTION) Illegal edge: %d to %d", e->src, e->dst); }
            Node *src = &NodeSet[e->src + LkhIdBase];
            Node *dst = &NodeSet[e->dst + dstOffset + LkhIdBase];
            AddCandidate(src, dst, 0, 1);
            AddCandidate(dst, src, 0, 1);
        }

        if (Asymmetric) {
            for (ID i = 1; i <= DimensionSaved; i++) {
                FixEdge(&NodeSet[i], &NodeSet[i + DimensionSaved]);
            }
        }
        WeightType = 1; // TODO[szx][0]: find out why it is EUC_2D instead of EXPLICIT.
        MaxCandidates = ExtraCandidates = 0;
        Distance = Distance_1; // TODO[szx][0]: find out why to return constant distance.

        postprocess();
    }
    #pragma endregion UnsupportedInterface

    static void preprocess(int problemType = TSP) {
        FreeStructures();

        FirstNode = 0;
        WeightType = WeightFormat = ProblemType = -1;
        Name = Copy("Unnamed");
        Type = EdgeWeightType = EdgeWeightFormat = 0;
        EdgeDataFormat = NodeCoordType = DisplayDataType = 0;
        Distance = 0;
        C = 0;
        c = 0;
        DistanceLimit = DBL_MAX;

        //Read_BACKHAUL_SECTION();

        //Capacity = ;

        //Read_CTSP_SET_SECTION();

        //if (DemandDimension < 0) { eprintf("DIMENSION_DIMENSION: < 0"); }

        //Read_DEMAND_SECTION();

        //if (MTSPDepot <= 0) { eprintf("DEPOT_SECTION: Positive value expected"); }

        //for (ID i = 1; i <= Dim; i++) { NodeSet[i].DraftLimit = ; }

        //if (ExternalSalesmen < 0) { eprintf("EXTERNAL_SALESMEN: < 0"); }

        //Read_FIXED_EDGES_SECTION();

        //if (GridSize < 0) { eprintf("GRID_SIZE: non-negative real expected"); }

        //Read_PICKUP_AND_DELIVERY_SECTION();
        
        //RiskThreshold = ;

        Salesmen = 1;
        if (Salesmen <= 0) { eprintf("SALESMEN/VEHICLES: <= 0"); }

        //if (ServiceTime < 0) { eprintf("SERVICE_TIME: < 0"); }

        //for (ID i = 1; i <= Dim; i++) { NodeSet[i].ServiceTime = ; }

        //Read_TIME_WINDOW_SECTION();

        //Read_TOUR_SECTION(&ProblemFile);

        Type = Copy("TSP");
        ProblemType = problemType;
        //ProblemType = TSP;
        //ProblemType = ATSP;
        //ProblemType = SOP;
        //ProblemType = HCP; // Hamiltonian Circuit Problem.
        //ProblemType = HPP; // Hamiltonian Path Problem. // TODO[szx][5]: eliminate HPP problem type check.
        //ProblemType = BWTSP;
        //ProblemType = CCVRP;
        //ProblemType = CVRP; // CVRP || DCVRP
        //ProblemType = ACVRP;
        //ProblemType = CVRPTW;
        //ProblemType = MLP;
        //ProblemType = OVRP;
        //ProblemType = PDPTW;
        //ProblemType = PDTSP;
        //ProblemType = PDTSPF;
        //ProblemType = PDTSPL;
        //ProblemType = TRP; // TRP || MTRP || MTRPD
        //ProblemType = RCTVRP;
        //ProblemType = RCTVRPTW;
        //ProblemType = TSPTW;
        //ProblemType = VRPB;
        //ProblemType = VRPBTW;
        //ProblemType = VRPPD; // VRPSPD || VRPSPDTW || VRPMPD || VRPMPDTW || MVRPB
        //ProblemType = ONE_PDTSP; // 1-PDTSP
        //ProblemType = M_PDTSP; // M-PDTSP
        //ProblemType = M1_PDTSP; // M1-PDTSP
        //ProblemType = TSPDL;
        //ProblemType = CTSP;
        if (ProblemType == TOUR) { eprintf("TYPE: Type not implemented: TOUR", Type); }

        switch (ProblemType) {
        case ATSP: case CCVRP: case ACVRP: case CVRPTW: case MLP: case M_PDTSP: case M1_PDTSP:
        case ONE_PDTSP: case OVRP: case PDTSP: case PDTSPF: case PDTSPL: case PDPTW: case RCTVRP:
        case RCTVRPTW: case SOP: case TRP: case TSPDL: case TSPTW: case VRPB: case VRPBTW: case VRPPD:
            Asymmetric = 1;
            break;
        default:
            Asymmetric = 0;
            break;
        }

        Swaps = 0;
    }

    static void postprocess() {
        int i, j, K;

        /* Adjust parameters */
        if (Seed == 0)
            Seed = (unsigned)time(0);
        if (InitialStepSize == 0)
            InitialStepSize = 1;
        if (MaxSwaps < 0)
            MaxSwaps = Dimension;
        if (KickType > Dimension / 2)
            KickType = Dimension / 2;
        if (Runs == 0)
            Runs = 10;
        if (MaxCandidates > Dimension - 1)
            MaxCandidates = Dimension - 1;
        if (ExtraCandidates > Dimension - 1)
            ExtraCandidates = Dimension - 1;
        if (SubproblemSize >= Dimension)
            SubproblemSize = Dimension;
        else if (SubproblemSize == 0) {
            if (AscentCandidates > Dimension - 1)
                AscentCandidates = Dimension - 1;
            if (InitialPeriod < 0) {
                InitialPeriod = Dimension / 2;
                if (InitialPeriod < 100)
                    InitialPeriod = 100;
            }
            if (Excess < 0)
                Excess = 1.0 / DimensionSaved * Salesmen;
            if (MaxTrials == -1)
                MaxTrials = Dimension;
            HeapMake(Dimension);
        }
        if (POPMUSIC_MaxNeighbors > Dimension - 1)
            POPMUSIC_MaxNeighbors = Dimension - 1;
        if (POPMUSIC_SampleSize > Dimension)
            POPMUSIC_SampleSize = Dimension;
        Depot = &NodeSet[MTSPDepot];
        if (ProblemType == CVRP) {
            Node *N;
            int MinSalesmen;
            if (Capacity <= 0)
                eprintf("CAPACITY not specified");
            TotalDemand = 0;
            N = FirstNode;
            do
                TotalDemand += N->Demand;
            while ((N = N->Suc) != FirstNode);
            MinSalesmen =
                TotalDemand / Capacity + (TotalDemand % Capacity != 0);
            if (Salesmen == 1) {
                Salesmen = MinSalesmen;
                if (Salesmen > Dimension)
                    eprintf("CVRP: SALESMEN larger than DIMENSION");
            } else if (Salesmen < MinSalesmen)
                eprintf("CVRP: SALESMEN too small to meet demand");
            assert(Salesmen >= 1 && Salesmen <= Dimension);
            if (Salesmen == 1)
                ProblemType = TSP;
            Penalty = Penalty_CVRP;
        } else if (ProblemType == SOP || ProblemType == M1_PDTSP) {
            Constraint *Con;
            Node *Ni, *Nj;
            int n, k;
            OldDistance = Distance;
            Distance = Distance_SOP;
            if (ProblemType == M1_PDTSP) {
                for (i = 2; i < Dim; i++) {
                    Ni = &NodeSet[i];
                    for (k = n = 0; k < DemandDimension; k++) {
                        n = Ni->M_Demand[k];
                        if (n >= 0)
                            continue;
                        for (j = 2; j < Dim; j++) {
                            if (j == i)
                                continue;
                            Nj = &NodeSet[j];
                            if (Nj->M_Demand[k] == -n) {
                                Ni->C[j] = -1;
                                break;
                            }
                        }
                    }
                }
            }
            for (j = 2; j < Dim; j++) {
                Nj = &NodeSet[j];
                for (i = 2; i < Dim; i++) {
                    if (i != j && Nj->C[i] == -1) {
                        Ni = &NodeSet[i];
                        assert(Con =
                            (Constraint *)malloc(sizeof(Constraint)));
                        Con->t1 = Ni;
                        Con->t2 = Nj;
                        Con->Suc = FirstConstraint;
                        FirstConstraint = Con;
                        Con->Next = Ni->FirstConstraint;
                        Ni->FirstConstraint = Con;
                    }
                }
            }
            Salesmen = 1;
            Penalty = ProblemType == SOP ? Penalty_SOP : Penalty_M1_PDTSP;
        }
        if (ProblemType == TSPTW) {
            Salesmen = 1;
            Penalty = Penalty_TSPTW;
        } else
            TSPTW_Makespan = 0;
        if (Salesmen > 1) {
            if (Salesmen > Dim)
                eprintf("Too many salesmen/vehicles (> DIMENSION)");
            MTSP2TSP();
        }
        if (ExternalSalesmen > Salesmen)
            ExternalSalesmen = Salesmen;
        if (ProblemType == ACVRP)
            Penalty = Penalty_ACVRP;
        else if (ProblemType == CCVRP)
            Penalty = Penalty_CCVRP;
        else if (ProblemType == CTSP)
            Penalty = Penalty_CTSP;
        else if (ProblemType == CVRPTW)
            Penalty = Penalty_CVRPTW;
        else if (ProblemType == MLP)
            Penalty = Penalty_MLP;
        else if (ProblemType == OVRP)
            Penalty = Penalty_OVRP;
        else if (ProblemType == PDTSP)
            Penalty = Penalty_PDTSP;
        else if (ProblemType == PDTSPF)
            Penalty = Penalty_PDTSPF;
        else if (ProblemType == PDTSPL)
            Penalty = Penalty_PDTSPL;
        else if (ProblemType == PDPTW)
            Penalty = Penalty_PDPTW;
        else if (ProblemType == ONE_PDTSP)
            Penalty = Penalty_1_PDTSP;
        else if (ProblemType == M_PDTSP)
            Penalty = Penalty_M_PDTSP;
        else if (ProblemType == M1_PDTSP)
            Penalty = Penalty_M1_PDTSP;
        else if (ProblemType == RCTVRP || ProblemType == RCTVRPTW)
            Penalty = Penalty_RCTVRP;
        else if (ProblemType == TRP)
            Penalty = Penalty_TRP;
        else if (ProblemType == TSPDL)
            Penalty = Penalty_TSPDL;
        else if (ProblemType == TSPPD)
            Penalty = Penalty_TSPPD;
        if (ProblemType == VRPB)
            Penalty = Penalty_VRPB;
        else if (ProblemType == VRPBTW)
            Penalty = Penalty_VRPBTW;
        else if (ProblemType == VRPPD)
            Penalty = Penalty_VRPPD;
        if (BWTSP_B > 0) {
            if (Penalty)
                eprintf("BWTSP not compatible with problem type %s\n", Type);
            ProblemType = BWTSP;
            free(Type);
            Type = Copy("BWTSP");
            Penalty = Penalty_BWTSP;
            if (BWTSP_L != INT_MAX)
                BWTSP_L *= Scale;
        }
        if (Penalty && (SubproblemSize > 0 || SubproblemTourFile))
            eprintf("Partitioning not implemented for constrained problems");
        Depot->DepotId = 1;
        for (i = Dim + 1; i <= DimensionSaved; i++)
            NodeSet[i].DepotId = i - Dim + 1;
        if (Dimension != DimensionSaved) {
            NodeSet[Depot->Id + DimensionSaved].DepotId = 1;
            for (i = Dim + 1; i <= DimensionSaved; i++)
                NodeSet[i + DimensionSaved].DepotId = i - Dim + 1;
        }
        if (ServiceTime != 0) {
            for (i = 1; i <= Dim; i++)
                NodeSet[i].ServiceTime = ServiceTime;
            Depot->ServiceTime = 0;
        }
        if (CostMatrix == 0 && Dimension <= MaxMatrixDimension &&
            Distance != 0 && Distance != Distance_1 && Distance != Distance_LARGE
            && Distance != Distance_LARGE && Distance != Distance_ATSP
            && Distance != Distance_MTSP && Distance != Distance_SPECIAL) {
            Node *Ni, *Nj;
            assert(CostMatrix =
                (int *)calloc((size_t)Dim * (Dim - 1) / 2, sizeof(int)));
            Ni = FirstNode->Suc;
            do {
                Ni->C =
                    &CostMatrix[(size_t)(Ni->Id - 1) * (Ni->Id - 2) / 2] - 1;
                if (ProblemType != HPP || Ni->Id <= Dim)
                    for (Nj = FirstNode; Nj != Ni; Nj = Nj->Suc)
                        Ni->C[Nj->Id] = Fixed(Ni, Nj) ? 0 : Distance(Ni, Nj);
                else
                    for (Nj = FirstNode; Nj != Ni; Nj = Nj->Suc)
                        Ni->C[Nj->Id] = 0;
            } while ((Ni = Ni->Suc) != FirstNode);
            c = 0;
            WeightType = EXPLICIT;
        }
        if (ProblemType == TSPTW ||
            ProblemType == CVRPTW || ProblemType == VRPBTW ||
            ProblemType == PDPTW || ProblemType == RCTVRPTW) {
            M = INT_MAX / 2 / Precision;
            for (i = 1; i <= Dim; i++) {
                Node *Ni = &NodeSet[i];
                for (j = 1; j <= Dim; j++) {
                    Node *Nj = &NodeSet[j];
                    if (Ni != Nj &&
                        Ni->Earliest + Ni->ServiceTime + Ni->C[j] > Nj->Latest)
                        Ni->C[j] = M;
                }
            }
        }
        C = WeightType == EXPLICIT ? C_EXPLICIT : C_FUNCTION;
        D = WeightType == EXPLICIT ? D_EXPLICIT : D_FUNCTION;
        if (ProblemType != CVRP && ProblemType != CVRPTW &&
            ProblemType != CTSP &&
            ProblemType != TSP && ProblemType != ATSP) {
            M = INT_MAX / 2 / Precision;
            for (i = Dim + 1; i <= DimensionSaved; i++) {
                for (j = 1; j <= DimensionSaved; j++) {
                    if (j == i)
                        continue;
                    if (j == MTSPDepot || j > Dim)
                        NodeSet[i].C[j] = NodeSet[MTSPDepot].C[j] = M;
                    NodeSet[i].C[j] = NodeSet[MTSPDepot].C[j];
                    NodeSet[j].C[i] = NodeSet[j].C[MTSPDepot];
                }
            }
            if (ProblemType == CCVRP || ProblemType == OVRP)
                for (i = 1; i <= Dim; i++)
                    NodeSet[i].C[MTSPDepot] = 0;
        }
        if (Precision > 1 && CostMatrix) {
            for (i = 2; i <= Dim; i++) {
                Node *N = &NodeSet[i];
                for (j = 1; j < i; j++)
                    if (N->C[j] * Precision / Precision != N->C[j])
                        eprintf("PRECISION (= %d) is too large", Precision);
            }
        }
        if (SubsequentMoveType == 0) {
            SubsequentMoveType = MoveType;
            SubsequentMoveTypeSpecial = MoveTypeSpecial;
        }
        K = MoveType >= SubsequentMoveType || !SubsequentPatching ?
            MoveType : SubsequentMoveType;
        if (PatchingC > K)
            PatchingC = K;
        if (PatchingA > 1 && PatchingA >= PatchingC)
            PatchingA = PatchingC > 2 ? PatchingC - 1 : 1;
        if (NonsequentialMoveType == -1 ||
            NonsequentialMoveType > K + PatchingC + PatchingA - 1)
            NonsequentialMoveType = K + PatchingC + PatchingA - 1;
        if (PatchingC >= 1 && NonsequentialMoveType >= 4) {
            BestMove = BestSubsequentMove = BestKOptMove;
            if (!SubsequentPatching && SubsequentMoveType <= 5) {
                MoveFunction BestOptMove[] =
                { 0, 0, Best2OptMove, Best3OptMove,
                    Best4OptMove, Best5OptMove
                };
                BestSubsequentMove = BestOptMove[SubsequentMoveType];
            }
        } else {
            MoveFunction BestOptMove[] = { 0, 0, Best2OptMove, Best3OptMove,
                Best4OptMove, Best5OptMove
            };
            BestMove = MoveType <= 5 ? BestOptMove[MoveType] : BestKOptMove;
            BestSubsequentMove = SubsequentMoveType <= 5 ?
                BestOptMove[SubsequentMoveType] : BestKOptMove;
        }
        if (MoveTypeSpecial)
            BestMove = BestSpecialOptMove;
        if (SubsequentMoveTypeSpecial)
            BestSubsequentMove = BestSpecialOptMove;
        if (ProblemType == HCP || ProblemType == HPP)
            MaxCandidates = 0;
        if (TraceLevel >= 1) {
            printff("done\n");
            PrintParameters();
        }
        if (InitialTourFileName)
            ReadTour(InitialTourFileName, &InitialTourFile);
        if (InputTourFileName)
            ReadTour(InputTourFileName, &InputTourFile);
        if (SubproblemTourFileName && SubproblemSize > 0)
            ReadTour(SubproblemTourFileName, &SubproblemTourFile);
        if (MergeTourFiles >= 1) {
            free(MergeTourFile);
            assert(MergeTourFile =
                (FILE **)malloc(MergeTourFiles * sizeof(FILE *)));
            for (i = 0; i < MergeTourFiles; i++)
                ReadTour(MergeTourFileName[i], &MergeTourFile[i]);
        }
        free(LastLine);
        LastLine = 0;
    }

    static int TwoDWeightType() {
        if (Asymmetric) { return 0; }
        return WeightType == EUC_2D || WeightType == MAX_2D ||
            WeightType == MAN_2D || WeightType == CEIL_2D ||
            WeightType == FLOOR_2D ||
            WeightType == GEO || WeightType == GEOM ||
            WeightType == GEO_MEEUS || WeightType == GEOM_MEEUS ||
            WeightType == ATT || WeightType == TOR_2D ||
            (WeightType == SPECIAL && CoordType == TWOD_COORDS);
    }

    static int ThreeDWeightType() {
        if (Asymmetric) { return 0; }
        return WeightType == EUC_3D || WeightType == MAX_3D ||
            WeightType == MAN_3D || WeightType == CEIL_3D ||
            WeightType == FLOOR_3D || WeightType == TOR_3D ||
            WeightType == XRAY1 || WeightType == XRAY2 ||
            (WeightType == SPECIAL && CoordType == THREED_COORDS);
    }

    static void CheckSpecificationPart() {
        if (ProblemType == -1)
            eprintf("TYPE is missing");
        if (Dimension < 3)
            eprintf("DIMENSION < 3 or not specified");
        if (WeightType == -1 && !Asymmetric && ProblemType != HCP &&
            ProblemType != HPP && !EdgeWeightType)
            eprintf("EDGE_WEIGHT_TYPE is missing");
        if (WeightType == EXPLICIT && WeightFormat == -1 && !EdgeWeightFormat)
            eprintf("EDGE_WEIGHT_FORMAT is missing");
        if (WeightType == EXPLICIT && WeightFormat == FUNCTION)
            eprintf("Conflicting EDGE_WEIGHT_TYPE and EDGE_WEIGHT_FORMAT");
        if (WeightType != EXPLICIT &&
            (WeightType != SPECIAL || CoordType != NO_COORDS) &&
            WeightType != -1 && WeightFormat != -1 && WeightFormat != FUNCTION)
            eprintf("Conflicting EDGE_WEIGHT_TYPE and EDGE_WEIGHT_FORMAT");
        if ((ProblemType == ATSP || ProblemType == SOP) &&
            WeightType != EXPLICIT && WeightType != -1)
            eprintf("Conflicting TYPE and EDGE_WEIGHT_TYPE");
        if (CandidateSetType == DELAUNAY && !TwoDWeightType() &&
            MaxCandidates > 0)
            eprintf
            ("Illegal EDGE_WEIGHT_TYPE for CANDIDATE_SET_TYPE = DELAUNAY");
        if (CandidateSetType == QUADRANT && !TwoDWeightType() &&
            !ThreeDWeightType() && MaxCandidates + ExtraCandidates > 0)
            eprintf
            ("Illegal EDGE_WEIGHT_TYPE for CANDIDATE_SET_TYPE = QUADRANT");
        if (ExtraCandidateSetType == QUADRANT && !TwoDWeightType() &&
            !ThreeDWeightType() && ExtraCandidates > 0)
            eprintf
            ("Illegal EDGE_WEIGHT_TYPE for EXTRA_CANDIDATE_SET_TYPE = "
                "QUADRANT");
        if (InitialTourAlgorithm == QUICK_BORUVKA && !TwoDWeightType() &&
            !ThreeDWeightType())
            eprintf
            ("Illegal EDGE_WEIGHT_TYPE for INITIAL_TOUR_ALGORITHM = "
                "QUICK-BORUVKA");
        if (InitialTourAlgorithm == SIERPINSKI && !TwoDWeightType())
            eprintf
            ("Illegal EDGE_WEIGHT_TYPE for INITIAL_TOUR_ALGORITHM = "
                "SIERPINSKI");
        if (DelaunayPartitioning && !TwoDWeightType())
            eprintf("Illegal EDGE_WEIGHT_TYPE for DELAUNAY specification");
        if (KarpPartitioning && !TwoDWeightType() && !ThreeDWeightType())
            eprintf("Illegal EDGE_WEIGHT_TYPE for KARP specification");
        if (KCenterPartitioning && !TwoDWeightType() && !ThreeDWeightType())
            eprintf("Illegal EDGE_WEIGHT_TYPE for K-CENTER specification");
        if (KMeansPartitioning && !TwoDWeightType() && !ThreeDWeightType())
            eprintf("Illegal EDGE_WEIGHT_TYPE for K-MEANS specification");
        if (MoorePartitioning && !TwoDWeightType())
            eprintf("Illegal EDGE_WEIGHT_TYPE for MOORE specification");
        if (RohePartitioning && !TwoDWeightType() && !ThreeDWeightType())
            eprintf("Illegal EDGE_WEIGHT_TYPE for ROHE specification");
        if (SierpinskiPartitioning && !TwoDWeightType())
            eprintf("Illegal EDGE_WEIGHT_TYPE for SIERPINSKI specification");
        if (SubproblemBorders && !TwoDWeightType() && !ThreeDWeightType())
            eprintf("Illegal EDGE_WEIGHT_TYPE for BORDERS specification");
        if (InitialTourAlgorithm == MTSP_ALG && Asymmetric)
            eprintf("INTIAL_TOUR_ALGORITHM = MTSP is not applicable for "
                "asymetric problems");
    }

    static char *Copy(const char *S) {
        char *Buffer;

        if (!S || strlen(S) == 0)
            return 0;
        assert(Buffer = (char *)malloc(strlen(S) + 1));
        strcpy(Buffer, S);
        return Buffer;
    }

    static void CreateNodes() {
        Node *Prev = 0, *N = 0;
        int i;

        if (Dimension <= 0)
            eprintf("DIMENSION is not positive (or not specified)");
        if (Asymmetric) {
            Dim = DimensionSaved;
            DimensionSaved = Dimension + Salesmen - 1;
            Dimension = 2 * DimensionSaved;
        } else if (ProblemType == HPP) {
            Dimension++;
            if (Dimension > MaxMatrixDimension)
                eprintf("DIMENSION too large in HPP problem");
        }
        assert(NodeSet = (Node *)calloc(Dimension + 1, sizeof(Node)));
        for (i = 1; i <= Dimension; i++, Prev = N) {
            N = &NodeSet[i];
            if (i == 1)
                FirstNode = N;
            else
                Link(Prev, N);
            N->Id = i;
            if (MergeTourFiles >= 1)
                assert(N->MergeSuc =
                (Node **)calloc(MergeTourFiles, sizeof(Node *)));
            N->Earliest = 0;
            N->Latest = INT_MAX;
        }
        Link(N, FirstNode);
    }


    static int FixEdge(Node * Na, Node * Nb) {
        if (!Na->FixedTo1 || Na->FixedTo1 == Nb)
            Na->FixedTo1 = Nb;
        else if (!Na->FixedTo2 || Na->FixedTo2 == Nb)
            Na->FixedTo2 = Nb;
        else
            return 0;
        if (!Nb->FixedTo1 || Nb->FixedTo1 == Na)
            Nb->FixedTo1 = Na;
        else if (!Nb->FixedTo2 || Nb->FixedTo1 == Na)
            Nb->FixedTo2 = Na;
        else
            return 0;
        return 1;
    }

    static void Read_BACKHAUL_SECTION() {
        int Id;

        while (fscanint(ProblemFile, &Id) && Id != -1) {
            if (Id <= 0 || Id > Dim)
                eprintf("BACKHAUL_SECTION: Node number out of range: %d", Id);
            NodeSet[Id].Backhaul = 1;
            NodeSet[Id + DimensionSaved].Backhaul = 1;
        }
    }

    static void Read_CTSP_SET_SECTION() {
        Node *N;
        int Id, n, *ColorUsed;

        N = FirstNode;
        do {
            N->Color = 0;
        } while ((N = N->Suc) != FirstNode);
        assert(ColorUsed = (int *)calloc(Salesmen + 1, sizeof(int)));
        while (fscanf(ProblemFile, "%d", &Id) > 0) {
            if (Id < 1 || Id > Salesmen)
                eprintf("(CTSP_SET_SECTION) Color number %d outside range", Id);
            if (ColorUsed[Id])
                eprintf("(CTSP_SET_SECTION) Color number %d used twice", Id);
            ColorUsed[Id] = 1;
            for (;;) {
                if (fscanf(ProblemFile, "%d", &n) != 1)
                    eprintf("(CTSP_SET_SECTION) Missing -1");
                if (n == -1)
                    break;
                if (n < 1 || n > DimensionSaved)
                    eprintf("(CTSP_SET_SECTION) Node %d outside range", n);
                N = &NodeSet[n];
                if (N->Color != 0 && N->Color != Id)
                    eprintf("(CTSP_SET_SECTION) Node %d in two sets", n);
                if (N == Depot)
                    eprintf("(CTSP_SET_SECTION) Depot %d occurs in set %d", n, Id);
                N->Color = Id;
            }
        }
        free(ColorUsed);
    }

    static void Read_DEMAND_SECTION() {
        int Id, Demand, i, k;
        Node *N;

        for (i = 1; i <= Dim; i++) {
            fscanint(ProblemFile, &Id);
            if (Id <= 0 || Id > Dim)
                eprintf("DEMAND_SECTION: Node number out of range: %d", Id);
            N = &NodeSet[Id];
            if (DemandDimension > 1) {
                assert(N->M_Demand =
                    (int *)malloc(DemandDimension * sizeof(int)));
                for (k = 0; k < DemandDimension; k++) {
                    if (!fscanint(ProblemFile, &Demand))
                        eprintf("DEMAND_SECTION: Missing demand for node %d",
                            Id);
                    N->M_Demand[k] = Demand;
                }
            } else if (!fscanint(ProblemFile, &N->Demand))
                eprintf("DEMAND_SECTION: Missing demand for node %d", Id);
        }
    }

    static void Read_FIXED_EDGES_SECTION() {
        Node *Ni, *Nj, *N, *NPrev = 0, *NNext;
        int i, j, Count = 0;

        CheckSpecificationPart();
        if (!FirstNode)
            CreateNodes();
        if (ProblemType == HPP)
            Dimension--;
        if (!fscanint(ProblemFile, &i))
            i = -1;
        while (i != -1) {
            if (i <= 0 || i > (Asymmetric ? Dimension / 2 : Dimension))
                eprintf("FIXED_EDGES_SECTION: Node number out of range: %d",
                    i);
            fscanint(ProblemFile, &j);
            if (j <= 0 || j > (Asymmetric ? Dimension / 2 : Dimension))
                eprintf("FIXED_EDGES_SECTION: Node number out of range: %d",
                    j);
            if (i == j)
                eprintf("FIXED_EDGES_SECTION: Illegal edge: %d to %d", i, j);
            Ni = &NodeSet[i];
            Nj = &NodeSet[Asymmetric ? j + Dimension / 2 : j];
            if (!FixEdge(Ni, Nj))
                eprintf("FIXED_EDGES_SECTION: Illegal fix: %d to %d", i, j);
            /* Cycle check */
            N = Ni;
            Count = 0;
            do {
                NNext = N->FixedTo1 != NPrev ? N->FixedTo1 : N->FixedTo2;
                NPrev = N;
                Count++;
            } while ((N = NNext) && N != Ni);
            if (N == Ni && Count != Dimension)
                eprintf("FIXED_EDGES_SECTION: Illegal fix: %d to %d", i, j);
            if (!fscanint(ProblemFile, &i))
                i = -1;
        }
        if (ProblemType == HPP)
            Dimension++;
    }

    static void Read_PICKUP_AND_DELIVERY_SECTION() {
        int Id, i;
        Node *N = FirstNode;
        do
            N->V = 0;
        while ((N = N->Suc) != FirstNode);
        for (i = 1; i <= Dim; i++) {
            if (!fscanint(ProblemFile, &Id))
                eprintf("PICKUP_AND_DELIVERY_SECTION: Missing nodes");
            if (Id <= 0 || Id > Dim)
                eprintf
                ("PICKUP_AND_DELIVERY_SECTION: Node number out of range: %d",
                    Id);
            N = &NodeSet[Id];
            if (N->V == 1)
                eprintf("PICKUP_AND_DELIVERY_SECTION: "
                    "Node number occurs twice: %d", N->Id);
            N->V = 1;
            if (!fscanf(ProblemFile, "%d %lf %lf %lf %d %d",
                &N->Demand, &N->Earliest, &N->Latest, &N->ServiceTime,
                &N->Pickup, &N->Delivery))
                eprintf("PICKUP_AND_DELIVERY_SECTION: "
                    " Missing data for node %d", N->Id);
            if (N->ServiceTime < 0)
                eprintf("PICKUP_AND_DELIVERY_SECTION: "
                    "Negative Service Time for node %d", N->Id);
            if (N->Earliest > N->Latest)
                eprintf("PICKUP_AND_DELIVERY_SECTION: "
                    "Earliest > Latest for node %d", N->Id);
        }
        N = FirstNode;
        do
            if (!N->V && N->Id <= Dim)
                break;
        while ((N = N->Suc) != FirstNode);
        if (!N->V)
            eprintf("PICKUP_AND_DELIVERY_SECTION: No data given for node %d",
                N->Id);
        if (ProblemType != VRPPD) {
            do {
                if (N->Delivery) {
                    if (NodeSet[N->Delivery].Pickup != N->Id ||
                        N->Delivery == N->Id)
                        eprintf("PICKUP_AND_DELIVERY_SECTION: "
                            "Illegal pairing for node %d", N->Id);
                    if (N->Demand < 0)
                        eprintf("PICKUP_AND_DELIVERY_SECTION: "
                            "Negative demand for delivery node %d", N->Id);
                } else if (N->Pickup) {
                    if (NodeSet[N->Pickup].Delivery != N->Id
                        || N->Pickup == N->Id)
                        eprintf("PICKUP_AND_DELIVERY_SECTION: "
                            "Illegal pairing for node %d", N->Id);
                    if (N->Demand > 0)
                        eprintf("PICKUP_AND_DELIVERY_SECTION: "
                            "Positive demand for pickup node %d", N->Id);
                    if (N->Demand + NodeSet[N->Pickup].Demand)
                        eprintf("PICKUP_AND_DELIVERY_SECTION: "
                            "Demand for pickup node %d and demand for delivery "
                            "node %d does not sum to zero", N->Id,
                            N->Pickup);
                }
            } while ((N = N->Suc) != FirstNode);
        }
    }

    static void Read_TIME_WINDOW_SECTION() {
        int Id, i;
        Node *N = FirstNode;
        do
            N->V = 0;
        while ((N = N->Suc) != FirstNode);
        for (i = 1; i <= Dim; i++) {
            if (!fscanint(ProblemFile, &Id))
                eprintf("TIME_WINDOW_SECTION: Missing nodes");
            if (Id <= 0 || Id > Dim)
                eprintf("TIME_WINDOW_SECTION: Node number out of range: %d",
                    Id);
            N = &NodeSet[Id];
            if (N->V == 1)
                eprintf("TIME_WINDOW_SECTION: Node number occurs twice: %d",
                    N->Id);
            N->V = 1;
            if (!fscanf(ProblemFile, "%lf", &N->Earliest))
                eprintf("TIME_WINDOW_SECTION: Missing earliest time");
            if (!fscanf(ProblemFile, "%lf", &N->Latest))
                eprintf("TIME_WINDOW_SECTION: Missing latest time");
            if (N->Earliest > N->Latest)
                printff("%d: %f %f\n", N->Id, N->Earliest, N->Latest);
            if (N->Earliest > N->Latest)
                eprintf("TIME_WINDOW_SECTION: Earliest > Latest for node %d",
                    N->Id);
        }
        N = FirstNode;
        do
            if (!N->V && N->Id <= Dim)
                break;
        while ((N = N->Suc) != FirstNode);
        if (!N->V)
            eprintf("TIME_WINDOW_SECTION: No time window given for node %d",
                N->Id);
    }

    static void Read_TOUR_SECTION(FILE ** File) {
        Node *First = 0, *Last = 0, *N, *Na;
        int i, k;

        if (TraceLevel >= 1) {
            printff("Reading ");
            if (File == &InitialTourFile)
                printff("INITIAL_TOUR_FILE: \"%s\" ... ", InitialTourFileName);
            else if (File == &InputTourFile)
                printff("INPUT_TOUR_FILE: \"%s\" ... ", InputTourFileName);
            else if (File == &SubproblemTourFile)
                printff("SUBPROBLEM_TOUR_FILE: \"%s\" ... ",
                    SubproblemTourFileName);
            else
                for (i = 0; i < MergeTourFiles; i++)
                    if (File == &MergeTourFile[i])
                        printff("MERGE_TOUR_FILE: \"%s\" ... ",
                            MergeTourFileName[i]);
        }
        if (!FirstNode)
            CreateNodes();
        N = FirstNode;
        do
            N->V = 0;
        while ((N = N->Suc) != FirstNode);
        if (ProblemType == HPP)
            Dimension--;
        if (Asymmetric)
            Dimension = DimensionSaved;
        int b = 0;
        if (!fscanint(*File, &i))
            i = -1;
        else if (i == 0) {
            b = 1;
            i++;
        }
        for (k = 0; k <= Dimension && i != -1; k++) {
            if (i <= 0 || i > Dimension)
                eprintf("(TOUR_SECTION) Node number out of range: %d", i);
            N = &NodeSet[i];
            if (N->V == 1 && k != Dimension)
                eprintf("(TOUR_SECTION) Node number occurs twice: %d", N->Id);
            N->V = 1;
            if (k == 0)
                First = Last = N;
            else {
                if (Asymmetric) {
                    Na = N + Dimension;
                    Na->V = 1;
                } else
                    Na = 0;
                if (File == &InitialTourFile) {
                    if (!Na)
                        Last->InitialSuc = N;
                    else {
                        Last->InitialSuc = Na;
                        Na->InitialSuc = N;
                    }
                } else if (File == &InputTourFile) {
                    if (!Na)
                        Last->InputSuc = N;
                    else {
                        Last->InputSuc = Na;
                        Na->InputSuc = N;
                    }
                } else if (File == &SubproblemTourFile) {
                    if (!Na)
                        (Last->SubproblemSuc = N)->SubproblemPred = Last;
                    else {
                        (Last->SubproblemSuc = Na)->SubproblemPred = Last;
                        (Na->SubproblemSuc = N)->SubproblemPred = Na;
                    }
                } else {
                    for (i = 0; i < MergeTourFiles; i++) {
                        if (File == &MergeTourFile[i]) {
                            if (!Na)
                                Last->MergeSuc[i] = N;
                            else {
                                Last->MergeSuc[i] = Na;
                                Na->MergeSuc[i] = N;
                            }
                        }
                    }
                }
                Last = N;
            }
            if (k < Dimension) {
                fscanint(*File, &i);
                if (b)
                    if (i >= 0)
                        i++;
            }
            if (k == Dimension - 1)
                i = First->Id;
        }
        N = FirstNode;
        do {
            if (!N->V)
                eprintf("TOUR_SECTION: Node is missing: %d", N->Id);
        } while ((N = N->Suc) != FirstNode);
        if (File == &SubproblemTourFile) {
            do {
                if (N->FixedTo1 &&
                    N->SubproblemPred != N->FixedTo1
                    && N->SubproblemSuc != N->FixedTo1)
                    eprintf("Fixed edge (%d, %d) "
                        "does not belong to subproblem tour", N->Id,
                        N->FixedTo1->Id);
                if (N->FixedTo2 && N->SubproblemPred != N->FixedTo2
                    && N->SubproblemSuc != N->FixedTo2)
                    eprintf("Fixed edge (%d, %d) "
                        "does not belong to subproblem tour", N->Id,
                        N->FixedTo2->Id);
            } while ((N = N->Suc) != FirstNode);
        }
        if (ProblemType == HPP)
            Dimension++;
        if (Asymmetric)
            Dimension *= 2;
        if (TraceLevel >= 1)
            printff("done\n");
    }

    /*
    The ReadTour function reads a tour from a file.

    The format is as follows:

    OPTIMUM = <real>
    Known optimal tour length. A run will be terminated as soon as a tour
    length less than or equal to optimum is achieved.
    Default: MINUS_INFINITY.

    TOUR_SECTION :
    A tour is specified in this section. The tour is given by a list of integers
    giving the sequence in which the nodes are visited in the tour. The tour is
    terminated by a -1.

    EOF
    Terminates the input data. The entry is optional.

    Other keywords in TSPLIB format may be included in the file, but they are
    ignored.
    */

    static void ReadTour(char *FileName, FILE ** File) {
        static const char Delimiters[] = " :=\n\t\r\f\v\xef\xbb\xbf";
        char *Line, *Keyword, *Token;
        unsigned int i;
        int Done = 0;

        if (!(*File = fopen(FileName, "r")))
            eprintf("Cannot open tour file: \"%s\"", FileName);
        while ((Line = ReadLine(*File))) {
            if (!(Keyword = strtok(Line, Delimiters)))
                continue;
            for (i = 0; i < strlen(Keyword); i++)
                Keyword[i] = (char)toupper(Keyword[i]);
            if (strcmp(Keyword, "OPTIMUM") == 0) {
                if (!(Token = strtok(0, Delimiters)) ||
                    !sscanf(Token, GainInputFormat, &Optimum))
                    eprintf("[%s] (OPTIMUM): Integer expected", FileName);
            } else if (!strcmp(Keyword, "TOUR_SECTION")) {
                Read_TOUR_SECTION(File);
                Done = 1;
            } else if (!strcmp(Keyword, "EOF"))
                break;
            else
                eprintf("[%s] Unknown Keyword: %s", FileName, Keyword);
        }
        if (!Done)
            eprintf("Missing TOUR_SECTION in tour file: \"%s\"", FileName);
        fclose(*File);
    }

    static void Convert2FullMatrix() {
        int n = DimensionSaved, i, j;
        Node *Ni, *Nj;

        if (n > MaxMatrixDimension) {
            OldDistance = Distance;
            Distance = Distance_Asymmetric;
            for (i = 1; i <= n; i++) {
                Ni = &NodeSet[i];
                Nj = &NodeSet[i + n];
                Nj->X = Ni->X;
                Nj->Y = Ni->Y;
                Nj->Z = Ni->Z;
                FixEdge(Ni, Nj);
            }
            return;
        }
        assert(CostMatrix = (int *)calloc((size_t)n * n, sizeof(int)));
        for (i = 1; i <= n; i++) {
            Ni = &NodeSet[i];
            Ni->C = &CostMatrix[(size_t)(i - 1) * n] - 1;
        }
        for (i = 1; i <= Dim; i++) {
            Ni = &NodeSet[i];
            for (j = i + 1; j <= Dim; j++) {
                Nj = &NodeSet[j];
                Ni->C[j] = Nj->C[i] = Distance(Ni, Nj);
            }
        }
        for (i = 1; i <= n; i++)
            FixEdge(&NodeSet[i], &NodeSet[i + n]);
        c = 0;
        Distance = Distance_ATSP;
        WeightType = -1; // TODO[szx][0]: find out why to clear the weight type.
    }
};

}
}


#endif // SMART_SZX_GOAL_LKH__INPUT_H
