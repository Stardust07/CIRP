////////////////////////////////
/// usage : 1.	cache for precomputed TSP optima.
/// 
/// note  : 1.	
////////////////////////////////

#ifndef SMART_SZX_GOAL_LKH3LIB_CACHE_H
#define SMART_SZX_GOAL_LKH3LIB_CACHE_H


#include <iostream>
#include <fstream>
#include <functional>

#include "Graph.h"


namespace szx {

struct TspCacheBase {
    using ID = Graph::ID;
    using Weight = double;
    using NodeList = List<ID>; // the `nodes` should be ordered in increasing order during the traverse.
    using NodeSet = List<bool>; // the size of the set should be the same as the node number.
    using Tour = Graph::Tour<Weight>;
    using TraverseEvent = std::function<bool(const Tour&)>;


    TspCacheBase() {}
    TspCacheBase(ID nodeNumber) : nodeNum(nodeNumber) {}

    virtual bool save(const std::string &path) const {
        std::ofstream ofs(path);
        if (!ofs.is_open()) { return false; }
        forEach([&](const Tour &tour) {
            ofs << tour.distance << "," << tour.nodes.size();
            for (auto n = tour.nodes.begin(); n != tour.nodes.end(); ++n) {
                ofs << "," << *n;
            }
            return false;
        });
        return true;
    }

    virtual bool load(const std::string &path) {
        std::ifstream ifs(path);
        if (!ifs.is_open()) { return false; }
        for (;;) {
            char c;
            ID nodeNum;
            Tour tour;
            ifs >> tour.distance >> c >> nodeNum;
            if (!ifs) { return true; }
            tour.nodes.resize(nodeNum);
            for (auto n = tour.nodes.begin(); n != tour.nodes.end(); ++n) {
                ifs >> c >> *n;
            }
            set(tour);
        }
    }


    // return a non-empty tour if there is cached solution for such node set, otherwise cache miss happens.
    // if the returned tour is `tour`, call `tour.empty()` to check the status.
    virtual const Tour& get(const NodeSet &containNode) const = 0;
    virtual const Tour& get(const NodeList &nodes) const = 0;

    // return true is overwriting happens, otherwise a new entry is added.
    virtual bool set(const Tour &sln) = 0;
    // the node set in `sln` and `nodes` should be the same.
    // return true is overwriting happens, otherwise a new entry is added.
    virtual bool set(const Tour &sln, const NodeList &nodes) = 0;
    virtual bool set(const Tour &sln, const NodeSet &containNode) = 0;

    // `onTour` return true if the traverse should be breaked, otherwise the loop will continue.
    // return true if no break happens.
    virtual bool forEach(TraverseEvent onTour) const = 0;


protected:
    ID nodeNum;
};


struct TspCache_BinTreeImpl : public TspCacheBase {
    using TspCacheBase::TspCacheBase;


    virtual const Tour& get(const NodeSet &containNode) const override {
        return Tour();
    }

    virtual const Tour& get(const NodeList &nodes) const override {
        NodeSet containNode(nodeNum, false);
        for (auto n = nodes.begin(); n != nodes.end(); ++n) { containNode[*n] = true; }
        return get(containNode);
    }

    virtual bool set(const Tour &sln) override {
        return false;
    }
    virtual bool set(const Tour &sln, const NodeList &nodes) override {
        return false;
    }
    virtual bool set(const Tour &sln, const NodeSet &containNode) override {
        return false;
    }

    virtual bool forEach(std::function<bool(const Tour&)> onCacheEntry) const override {
        //for each cache entry {
        //    if (onCacheEntry(entry.tour)) { return false; }
        //}
    }
};


struct TspCache_TrieImpl : public TspCacheBase {
    
};


struct TspCache_HashImpl : public TspCacheBase {

};


using TspCache = TspCache_BinTreeImpl;

}


#endif // SMART_SZX_GOAL_LKH__CACHE_H
