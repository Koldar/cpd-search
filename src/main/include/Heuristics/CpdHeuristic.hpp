#ifndef _CPD_SEARCH_CPD_HEURISTIC_HEADER__
#define _CPD_SEARCH_CPD_HEURISTIC_HEADER__

#include <tuple>
#include <cpp-utils/igraph.hpp>
#include <pathfinding-utils/IHeuristic.hpp>
#include <compressed-path-database/CpdManager.hpp>
#include "PerturbatedCost.hpp"

namespace pathfinding::search {

    using namespace cpp_utils::graphs;

    /**
     * @brief CPDHeuristic
     * 
     * this heuristic will yield, for every state, the cost of the CPD path from the state and the goal
     * 
     * @tparam STATE 
     * @tparam G 
     * @tparam V 
     */
    template <typename STATE, typename G, typename V>
    class CpdHeuristic: public IHeuristic<STATE> {
        typedef CpdHeuristic<STATE, G, V> CpdHeuristicInstance;
    public:
        using IHeuristic<STATE>::getHeuristic;
        using IMemorable::getByteMemoryOccupied;
    protected:
        /**
         * @brief manager for 
         * 
         */
        const compressed_path_database::CpdManager<G, V>& cpdManager;
        /**
         * @brief cache for H value already computed
         * 
         * @dot
         * digraph {
         *  A -> B -> C;
         *  D -> B;
         * }
         * @enddot
         * 
         * Suppose you compute the h value of A to C. To compute it, we will know that h_C = 0 and h_B = 1 (thus h_A=2).
         * When we compute the h value of D to C the h_B is already known, hen ce we can immediately retrieve H_D=2.
         * 
         * A vector where the index is a graph node id and the cell content is the h value. h value is set to cost::INFTY if we haven't compute the H value yet.
         * 
         */
        cpp_utils::vectorplus<cost_t> hOriginalCache;
        /**
         * @brief cache for H value already computed over the perturbated map
         * cost_t::INFTY if we haven't computed the H vlaue yet.
         * 
         */
        cpp_utils::vectorplus<cost_t> hPerturbatedCache;
        /**
         * @brief additional data from the last run of CpdHeuristic::getHeuristic
         * 
         * represents the cost the agent should pay supposing she would follow the path from the CPD without any waits.
         * This value is set after a call CpdHeuristic::getHeuristic.
         * 
         * UB not set before calling CpdHeuristic::getHeuristic
         */
        mutable cost_t lastPathActualCost;
        /**
         * @brief additional data from the last run of CpdHeuristic::getHeuristic
         * 
         * represents the cost the agent pay supposing she would follow the path from the CPD without any waits **on the original** graph.
         * This value is set after a call CpdHeuristic::getHeuristic
         * 
         * UB not set before calling CpdHeuristic::getHeuristic
         */
        mutable cost_t lastPathOriginalCost;
        /**
         * @brief the perturbated graph we're operating in
         * 
         */
        const IImmutableGraph<G, V, PerturbatedCost>& perturbatedGraph;
    public:
        /**
         * @brief Construct a new Cpd Heuristic object
         * 
         * @param cpdManager 
         * @param perturbatedGraph the graph we're going to use to fetch the costs fo the edges on the perturbated graph
         */
        CpdHeuristic(const compressed_path_database::CpdManager<G, V>& cpdManager, const IImmutableGraph<G, V, PerturbatedCost>& perturbatedGraph): 
            cpdManager{cpdManager}, 
            hOriginalCache{cpdManager.getReorderedGraph().numberOfVertices(), cost_t::INFTY}, 
            hPerturbatedCache{cpdManager.getReorderedGraph().numberOfVertices(), cost_t::INFTY},
            lastPathActualCost{cost_t::INFTY}, lastPathOriginalCost{cost_t::INFTY}, 
            perturbatedGraph{perturbatedGraph} {

            debug("CPDHeuristic constructor start");
            if (!cpdManager.isCpdLoaded()) {
                throw cpp_utils::exceptions::InvalidStateException<compressed_path_database::CpdManager<G, V>>{cpdManager};
            }
            debug("CPDHeuristic constructor end");
        }
        CpdHeuristic(const CpdHeuristicInstance& other) = delete;
        CpdHeuristic(CpdHeuristicInstance&& h): 
            cpdManager{h.cpdManager}, 
            hOriginalCache{std::move(h.hOriginalCache)}, 
            hPerturbatedCache{std::move(h.hPerturbatedCache)},
            lastPathOriginalCost{h.lastPathOriginalCost}, lastPathActualCost{h.lastPathActualCost}, 
            perturbatedGraph{h.perturbatedGraph} {
        }
        CpdHeuristicInstance& operator =(const CpdHeuristicInstance& h) = delete;
        CpdHeuristicInstance& operator =(CpdHeuristicInstance&& h) {
            this->cpdManager = h.cpdManager;
            this->hOriginalCache = std::move(h.hOriginalCache);
            this->hPerturbatedCache = ::std::move(h.hPerturbatedCache);
            this->lastPathActualCost = h.lastPathActualCost;
            this->lastPathOriginalCost = h.lastPathOriginalCost;
            this->perturbatedGraph = h.perturbatedGraph;
            return *this;
        }
        virtual ~CpdHeuristic() {
            debug("destroy cpd heuristic at", this);
        }
    public:
        virtual cost_t getHeuristic(const STATE& current, const STATE* agoal) {
            const STATE& goal = *agoal;

            moveid_t nextMove;
            nodeid_t nextNode;
            cost_t nextCost;

            /* 
            * first: node we in
            * second: next move the cpd told us to perform; 
            * third: cost of such move
            */
            cpp_utils::vectorplus<std::tuple<nodeid_t, moveid_t, cost_t, cost_t>> nodeVisited{};
            nodeid_t currentNode = current.getPosition();
            finer("fetching heuristic value of ", current, "towards", goal);
            while (true) {
                finer("we are in ", currentNode, "goal is", goal.getPosition());
                if (currentNode == goal.getPosition()) {
                    //current is goal
                    this->hOriginalCache[goal.getPosition()] = 0;
                    this->hPerturbatedCache[goal.getPosition()] = 0;
                    goto rollback;
                }
                if (hOriginalCache[currentNode].isNotInfinity()) {
                    //current h value is actually cached!
                    goto rollback;
                }
                //we need to compute the h value ourselves

                if (this->cpdManager.getFirstMove(currentNode, goal.getPosition(), nextMove, nextNode, nextCost)) {
                    //generated a move
                    auto edgeCost = this->perturbatedGraph.getOutEdge(currentNode, nextMove).getPayload();
                    cost_t actualCost = edgeCost.getCost();
                    nodeVisited.addTail(std::tuple<nodeid_t, moveid_t, cost_t, cost_t>{currentNode, nextMove, nextCost, actualCost});
                    currentNode = nextNode;
                } else {
                    //the cpd tells us that the goal is impossible to reach from start! we need to return +inf
                    //I expect that the CPD tells us immediately that the goal is unreachable, hence the assertion
                    assert(nodeVisited.isEmpty());
                    return cost_t::INFTY;
                }
            }

            /*
            * we need to update the caches of all the nodes we've visited for the first time during the loop
            * note that the last node (the goal or a node with hOriginalCache set) is not 
            * included in the sequence but will have have h_value set in hOriginalCache;
            */
            
            rollback:
            cost_t totalOriginalCost = hOriginalCache[currentNode];
            cost_t totalPerturbatedCost = hPerturbatedCache[currentNode];
            for (auto tuple: nodeVisited.reverse()) {
                totalOriginalCost += std::get<2>(tuple);
                totalPerturbatedCost += std::get<3>(tuple);

                this->hOriginalCache[std::get<0>(tuple)] = totalOriginalCost;
                this->hPerturbatedCache[std::get<0>(tuple)] = totalPerturbatedCost;
            }

            this->lastPathOriginalCost = totalOriginalCost;
            this->lastPathActualCost = totalPerturbatedCost;

            return totalOriginalCost;
        }
        
        virtual bool isAdmissible() const {
            return true;
        }
        
        virtual bool isConsistent() const {
            return true;
        }
    public:
        virtual MemoryConsumption getByteMemoryOccupied() const {
            MemoryConsumption result;
            result += sizeof(*this);
            result += cpdManager.getByteMemoryOccupied();
            return result;
        }
    public:
        virtual void cleanup() {
            this->hOriginalCache.fill(cost_t::INFTY);
            this->hPerturbatedCache.fill(cost_t::INFTY);
        }
    public:
        /**
         * @brief check if the heuristic values (h[n] and h'[n]) have been computed yet
         * 
         * @param node the node involved
         * @return true if the heuristic value from this node to the goal has been computed already
         * @return false otheriwse. if so, it not advisable to use the heuristic cost at all
         */
        bool isCostSet(nodeid_t node) const {
            return this->hOriginalCache[node] != cost_t::INFTY;
        }
        /**
         * @brief return the path from the given node to the goal using the original weights
         * 
         * @param node node involved
         * @return cost_t cpd-path from @c node to the goal using the original weights
         */
        cost_t getCPDPathOriginalWeights(nodeid_t node) const {
            DO_ON_DEBUG_IF(!this->isCostSet(node)) {
                throw cpp_utils::exceptions::InvalidArgumentException{"node", node, "heuristic costs have not been computed yet"};
            }
            return this->hOriginalCache[node];
        }
        /**
         * @brief return the path from the given node to the goal using the perturbated weights
         * 
         * @param node node involved
         * @return cost_t cpd-path from @c node to the goal using the perturbated weights
         */
        cost_t getCPDPathPerturbatedWeights(nodeid_t node) const {
            DO_ON_DEBUG_IF(!this->isCostSet(node)) {
                throw cpp_utils::exceptions::InvalidArgumentException{"node", node, "heuristic costs have not been computed yet"};
            }
            return this->hPerturbatedCache[node];
        }
    public:
        cost_t getLastOriginalCost() const {
            return this->lastPathOriginalCost;
        }
        cost_t getLastPerturbatedCost() const {
            return this->lastPathActualCost;
        }
        /**
         * @brief the number of nodes in the cache which H value has been set
         * 
         * @return size_t 
         */
        size_t getCachedElementsNumber() const {
            debug("the cache is ", this->hOriginalCache);
            return this->hOriginalCache.select([&](cost_t h) {return h.isNotInfinity();}).size();
        }
        const compressed_path_database::CpdManager<G, V>& getCpdManager() const {
            return this->cpdManager;
        }
    };

}

#endif