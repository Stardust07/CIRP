////////////////////////////////
/// usage : 1.	TSP solver with optimal tour cache.
/// 
/// note  : 1.	
////////////////////////////////

#ifndef SMART_SZX_GOAL_LKH3LIB_CACHED_TSP_SOLVER_H
#define SMART_SZX_GOAL_LKH3LIB_CACHED_TSP_SOLVER_H


#include <string>

#include "TspCache.h"
#include "lkhSolver.h"


namespace szx {

struct CachedTspSolver {
    using ID = lkh::ID;
    using Weight = int;
    using Tour = lkh::Tour;
    using CoordList2D = lkh::CoordList2D;
    using CoordList3D = lkh::CoordList3D;
    using AdjMat = lkh::AdjMat;
    using AdjList = lkh::AdjList;
    using EdgeList = lkh::EdgeList;
    using TspCache = TspCache<Tour>;

    using MapNodeId = std::function<ID(ID)>;


    CachedTspSolver(lkh::ID nodeNum) : tspCache(nodeNum) {}
    CachedTspSolver(lkh::ID nodeNum, const std::string &cacheFilePath)
        : tspCache(nodeNum), cachePath(cacheFilePath) {
        tspCache.load(cachePath);
    }

    ~CachedTspSolver() { tspCache.save(cachePath); }


    template<typename InputData>
    bool solve(Tour &sln, const TspCache::NodeSet &containNode, const InputData &input, MapNodeId mapId, const Tour &hintSln = Tour()) {
        const Tour &cachedTour(tspCache.get(containNode));
        if (cachedTour.nodes.empty()) {
            if (!lkh::solveTsp(sln, input, hintSln)) { return false; }
            if (mapId) { // recover node ID.
                for (auto n = sln.nodes.begin(); n != sln.nodes.end(); ++n) { *n = mapId(*n); }
            }
            tspCache.set(sln, containNode); // TODO[szx][0]: is it thread safe?
        } else {
            sln = cachedTour;
        }

        return true;
    }
    template<typename InputData>
    bool solve(Tour &sln, const TspCache::NodeSet &containNode, const InputData &input, const Tour &hintSln = Tour()) {
        return solve(sln, containNode, input, MapNodeId(), hintSln);
    }
    //bool solve(Tour &sln, const EdgeList &edgeList, const Tour &hintSln = Tour()) {}


    TspCache tspCache;
    std::string cachePath;
};


}


#endif // SMART_SZX_GOAL_LKH3LIB_CACHED_TSP_SOLVER_H
