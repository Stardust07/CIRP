////////////////////////////////
/// usage : 1.	cache for precomputed TSP optima.
/// 
/// note  : 1.	
////////////////////////////////

#ifndef SMART_SZX_GOAL_LKH3LIB_CACHE_H
#define SMART_SZX_GOAL_LKH3LIB_CACHE_H


#include <array>
#include <iostream>
#include <fstream>
#include <functional>
#include <unordered_map>
#include <mutex>

#include "Graph.h"


namespace szx {

template<typename Tour = Graph::Tour<Graph::DefaultWeightType>>
struct TspCacheBase {
    using ID = Graph::ID;
    using NodeList = Graph::List<ID>; // `nodeList` is a list of node IDs in increasing order.
    using NodeSet = Graph::List<bool>; // `nodeSet[n]` is true if node `n` is included in the set.
    using TraverseEvent = std::function<bool(const Tour&)>;


    static const Tour& emptyTour() {
        static const Tour et;
        return et;
    }


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
            ofs << std::endl;
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


    void toNodeSet(const NodeList &nodes, NodeSet &containNode) const {
        std::fill(containNode.begin(), containNode.end(), false);
        for (auto n = nodes.begin(); n != nodes.end(); ++n) { containNode[*n] = true; }
    }
    NodeSet toNodeSet(const NodeList &nodes) const {
        NodeSet containNode(nodeNum, false);
        for (auto n = nodes.begin(); n != nodes.end(); ++n) { containNode[*n] = true; }
        return containNode;
    }

    // return a non-empty tour if there is cached solution for such node set, otherwise cache miss happens.
    // if the returned tour is `tour`, call `tour.empty()` to check the status.
    virtual const Tour& get(const NodeSet &containNode) const = 0;
    virtual const Tour& get(const NodeList &orderedNodes) const = 0; // the orderedNodes is a list of node IDs in increasing order.

    // return true if overwriting happens or a new entry is added, otherwise better tour already exists.
    virtual bool set(const Tour &sln, const NodeSet &containNode) = 0;
    virtual bool set(const Tour &sln, const NodeList &orderedNodes) = 0; // the orderedNodes is a list of node IDs in increasing order.
    virtual bool set(const Tour &sln) = 0;

    // `onTour` return true if the traverse should be breaked, otherwise the loop will continue.
    // return true if no break happens.
    virtual bool forEach(TraverseEvent onTour) const = 0;

    ID getNodeNum() const { return nodeNum; }


protected:
    ID nodeNum;
};


template<typename Tour = Graph::Tour<Graph::DefaultWeightType>>
struct TspCache_BinTreeImpl : public TspCacheBase<Tour> {
    using ID = TspCacheBase<Tour>::ID;
    using NodeList = TspCacheBase<Tour>::NodeList; // `nodeList` is a list of node IDs in increasing order.
    using NodeSet = TspCacheBase<Tour>::NodeSet; // `nodeSet[n]` is true if node `n` is included in the set.
    using TraverseEvent = TspCacheBase<Tour>::TraverseEvent;
    using TspCacheBase<Tour>::emptyTour;
    using TspCacheBase<Tour>::toNodeSet;

protected:
    using TreeNodeID = int;
    // for non-leaf node, child[0]/child[1] leads to the sub-tree where the node is excluded/included.
    // for leaf node, child[0]/child[1] are the indices for the tour (where the last node is excluded/included) in the tourPool.
    // (child[0] < 0) means the sub-tree or tour is not cached.
    using BinTreeNode = std::array<TreeNodeID, 2>;


    static BinTreeNode newTreeNode() { return { -1, -1 }; }

public:
    static constexpr ID DefaultHintTourNum = (1 << 12);


    TspCache_BinTreeImpl(ID nodeNum, ID hintTourNum = DefaultHintTourNum) : TspCacheBase<Tour>(nodeNum) {
        nodePool.reserve(nodeNum * hintTourNum);
        tourPool.reserve(hintTourNum);
        nodePool.push_back(newTreeNode()); // the root.
    }


    virtual const Tour& get(const NodeSet &containNode) const override {
        TreeNodeID treeNode = 0;
        for (auto n = containNode.begin(); n != containNode.end(); ++n) {
            treeNode = nodePool[treeNode][*n];
            if (treeNode < 0) { return emptyTour(); } // cache miss.
        }
        return tourPool[treeNode]; // tour found.
    }

    virtual const Tour& get(const NodeList &orderedNodes) const override {
        return get(toNodeSet(orderedNodes));
    }

    virtual bool set(const Tour &sln, const NodeSet &containNode) override {
        std::lock_guard<std::mutex> writeLock(writeMutex);

        TreeNodeID treeNode = 0;
        auto lastNode = containNode.end() - 1;
        for (auto n = containNode.begin(); n != lastNode; ++n) {
            TreeNodeID &child = nodePool[treeNode][*n];
            if (child < 0) {
                treeNode = static_cast<TreeNodeID>(nodePool.size());
                child = treeNode;
                nodePool.push_back(newTreeNode());
            } else {
                treeNode = child;
            }
        }

        TreeNodeID &leaf = nodePool[treeNode][*lastNode];
        if (leaf < 0) {
            leaf = static_cast<TreeNodeID>(tourPool.size());
            tourPool.push_back(sln);
        } else if (sln.distance < tourPool[leaf].distance) {
            tourPool[leaf] = sln;
        } else {
            return false;
        }

        return true;
    }
    virtual bool set(const Tour &sln, const NodeList &orderedNodes) override {
        return set(sln, toNodeSet(orderedNodes));
    }
    virtual bool set(const Tour &sln) override {
        return set(sln, toNodeSet(sln.nodes));
    }

    virtual bool forEach(std::function<bool(const Tour&)> onCacheEntry) const override {
        for (auto t = tourPool.begin(); t != tourPool.end(); ++t) {
            if (onCacheEntry(*t)) { return false; }
        }
        return false;
    }

    virtual ID tourNum() const { return static_cast<ID>(tourPool.size()); }


    std::vector<BinTreeNode> nodePool;
    std::vector<Tour> tourPool;
    std::mutex writeMutex;
};


template<typename Tour = Graph::Tour<Graph::DefaultWeightType>>
struct TspCache_TrieImpl : public TspCacheBase<Tour> {
    
};


template<typename Tour = Graph::Tour<Graph::DefaultWeightType>>
struct TspCache_HashImpl : public TspCacheBase<Tour> {
    using ID = TspCacheBase<Tour>::ID;
    using NodeList = TspCacheBase<Tour>::NodeList; // `nodeList` is a list of node IDs in increasing order.
    using NodeSet = TspCacheBase<Tour>::NodeSet; // `nodeSet[n]` is true if node `n` is included in the set.
    using TraverseEvent = TspCacheBase<Tour>::TraverseEvent;
    using TspCacheBase<Tour>::emptyTour;
    using TspCacheBase<Tour>::toNodeSet;


    static constexpr ID DefaultHintTourNum = (1 << 12);


    TspCache_HashImpl(ID nodeNum, ID hintTourNum = DefaultHintTourNum) : TspCacheBase<Tour>(nodeNum) {}


    virtual const Tour& get(const NodeSet &containNode) const override {
        auto t = tourMap.find(containNode);
        if (t == tourMap.end()) { return emptyTour(); } // cache miss.
        return t->second; // tour found.
    }

    virtual const Tour& get(const NodeList &orderedNodes) const override {
        return get(toNodeSet(orderedNodes));
    }

    virtual bool set(const Tour &sln, const NodeSet &containNode) override {
        std::lock_guard<std::mutex> writeLock(writeMutex);

        auto t = tourMap.find(containNode);
        if (t == tourMap.end()) {
            tourMap[containNode] = sln;
        } else if (sln.distance < t->second.distance) {
            t->second = sln;
        } else {
            return false;
        }

        return true;
    }
    virtual bool set(const Tour &sln, const NodeList &orderedNodes) override {
        return set(sln, toNodeSet(orderedNodes));
    }
    virtual bool set(const Tour &sln) override {
        return set(sln, toNodeSet(sln.nodes));
    }

    virtual bool forEach(std::function<bool(const Tour&)> onCacheEntry) const override {
        for (auto t = tourMap.begin(); t != tourMap.end(); ++t) {
            if (onCacheEntry(t->second)) { return false; }
        }
        return false;
    }

    virtual ID tourNum() const { return static_cast<ID>(tourMap.size()); }


    std::unordered_map<NodeSet, Tour> tourMap;
    std::mutex writeMutex;
};


//template<typename Tour = Graph::Tour<Graph::DefaultWeightType>>
//using TspCache = TspCache_BinTreeImpl<Tour>;
template<typename Tour = Graph::Tour<Graph::DefaultWeightType>>
using TspCache = TspCache_HashImpl<Tour>;

}


#endif // SMART_SZX_GOAL_LKH__CACHE_H
