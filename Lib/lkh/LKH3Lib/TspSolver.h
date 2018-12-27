////////////////////////////////
/// usage : 1.	the interface for TSP solvers.
/// 
/// note  : 1.	
////////////////////////////////

#ifndef SMART_SZX_GOAL_LKH3LIB_TSP_SOLVER_H
#define SMART_SZX_GOAL_LKH3LIB_TSP_SOLVER_H


#include "Graph.h"


namespace szx {
namespace lkh {

using ID = Graph::ID; // node ID counts from 0 in the interface, but counts from 1 inside the LKH.
using Weight = int;
using Coord = double;

using Coord2D = Graph::Coord2D<Coord>;
using Coord3D = Graph::Coord3D<Coord>;
using AdjNode = Graph::WeightedAdjNode<Weight>;
using Edge = Graph::WeightedEdge<Weight>;

using CoordList2D = Graph::CoordList<Coord2D>;
using CoordList3D = Graph::CoordList<Coord3D>;
using AdjMat = Graph::AdjMat<Weight>;
using AdjList = Graph::AdjList<AdjNode>;
using EdgeList = Graph::EdgeList<Edge>;

using Tour = Graph::Tour<Weight>;


static constexpr ID LkhIdBase = 1;


bool solveTsp(Tour &sln, const CoordList2D &coordList, const Tour &hintSln = Tour());
bool solveTsp(Tour &sln, const CoordList3D &coordList, const Tour &hintSln = Tour());
bool solveTsp(Tour &sln, const AdjMat &adjMat, const Tour &hintSln = Tour());
bool solveTsp(Tour &sln, const AdjList &adjList, const Tour &hintSln = Tour());
bool solveTsp(Tour &sln, const EdgeList &edgeList, Graph::ID nodeNum, const Tour &hintSln = Tour());

}
}


#endif // SMART_SZX_GOAL_LKH3LIB_TSP_SOLVER_H
