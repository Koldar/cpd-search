#ifndef _CPD_SEARCH_CPD_FOCAL_HEURISTIC_HEADER__
#define _CPD_SEARCH_CPD_FOCAL_HEURISTIC_HEADER__

#include <tuple>
#include <cpp-utils/igraph.hpp>
#include <pathfinding-utils/IHeuristic.hpp>
#include <compressed-path-database/CpdManager.hpp>
#include "PerturbatedCost.hpp"
#include "CpdHeuristic.hpp"

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
    class CpdFocalHeuristic: public CpdHeuristic<STATE, G, V> {
        typedef CpdFocalHeuristic<STATE, G, V> CpdFocalHeuristicInstance;
    private:
        /**
         * @brief for each vertex ni the graph, represents the cost we need to pay to go from the given node till the source of the first perturbation, if we were to follow the CPD path
         * 
         * cost_t::INFTY if the cost has not been initialized yet
         */
        vectorplus<cost_t> perturbatedPathCostCache;
        vectorplus<nodeid_t> perturbatedSourceIdCache;
        /**
         * @brief source of the first perturbated edge we encounter while following the CPD
         * 
         * UB not set before calling CpdHeuristic::getHeuristic
         * 
         */
        mutable nodeid_t lastEarliestPerturbationSourceId;
        /**
         * @brief cost we need to pay to reach lastEarliestPerturbationSourceId from the current node
         * 
         * UB not set before calling CpdHeuristic::getHeuristic
         */
        mutable cost_t lastEarliestPerturbationSourceIdCost;
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
        CpdFocalHeuristic(const cpd::CpdManager<G, V>& cpdManager, const IImmutableGraph<G, V, PerturbatedCost>& perturbatedGraph): 
            CpdHeuristic<STATE, G, V>{cpdManager, perturbatedGraph},
            perturbatedPathCostCache{cpdManager.getReorderedGraph().size(), cost_t::INFTY},
            perturbatedSourceIdCache{cpdManager.getReorderedGraph().size(), cost_t::INFTY},
            lastEarliestPerturbationSourceId{0}, lastEarliestPerturbationSourceIdCost{cost_t::INFTY}
            {
        }
        CpdFocalHeuristic(const CpdFocalHeuristicInstance& other) = delete;
        CpdFocalHeuristic(CpdFocalHeuristicInstance&& h): CpdHeuristic<STATE, G, V>{::std::move(h)},
            lastEarliestPerturbationSourceId{h.lastEarliestPerturbationSourceId}, 
            lastEarliestPerturbationSourceIdCost{h.lastEarliestPerturbationSourceIdCost},
            perturbatedPathCostCache{::std::move(h.perturbatedPathCostCache)}, 
            perturbatedSourceIdCache{::std::move(h.perturbatedSourceIdCache)} 
            {
            
        }
        CpdFocalHeuristicInstance& operator =(const CpdFocalHeuristicInstance& h) = delete;
        CpdFocalHeuristicInstance& operator =(CpdFocalHeuristicInstance&& h) {
            CpdHeuristic<STATE, G, V>::operator =(::std::move(h));
            this->lastEarliestPerturbationSourceId = h.lastEarliestPerturbationSourceId;
            this->lastEarliestPerturbationSourceIdCost = h.lastEarliestPerturbationSourceIdCost;
            this->perturbatedPathCostCache = std::move(h.perturbatedPathCostCache);
            this->perturbatedSourceIdCache = ::std::move(h.perturbatedSourceIdCache);
            return *this;
        }
        ~CpdFocalHeuristic() {
            debug("destroy cpd focal heuristic at", this);
        }
    public:
        virtual cost_t getHeuristic(const STATE& current, const STATE* agoal) {
            const STATE& goal = *agoal;

            moveid_t nextMove;
            nodeid_t nextNode;
            cost_t nextCost;
            bool perturbationDetected = false;

            /* 
            * first: node we in
            * second: next move the cpd told us to perform; 
            * third: cost of such move
            * fourth: cost of such move over the perturbated graph
            * fifth: true if going from node according to move goes along a perturbated edge
            */
            cpp_utils::vectorplus<std::tuple<nodeid_t, moveid_t, cost_t, cost_t, bool>> nodeVisited{};
            nodeid_t currentNode = current.getPosition();
            finer("fetching heuristic value of ", current, "towards", goal);
            while (true) {
                finer("we are in ", currentNode, "goal is", goal.getPosition());
                if (currentNode == goal.getPosition()) {
                    //current is goal
                    this->hOriginalCache[goal.getPosition()] = 0;
                    this->hPerturbatedCache[goal.getPosition()] = 0;
                    this->perturbatedPathCostCache[goal.getPosition()] = 0;
                    this->perturbatedSourceIdCache[goal.getPosition()] = goal.getPosition();
                    goto rollback;
                }
                if (this->hOriginalCache[currentNode].isNotInfinity()) {
                    //current h value is actually cached!
                    goto rollback;
                }
                //we need to compute the h value ourselves

                if (this->cpdManager.getFirstMove(currentNode, goal.getPosition(), nextMove, nextNode, nextCost)) {
                    //generated a move
                    auto edgeCost = this->perturbatedGraph.getOutEdge(currentNode, nextMove).getPayload();
                    cost_t actualCost = edgeCost.getCost();
                    nodeVisited.addTail(std::tuple<nodeid_t, moveid_t, cost_t, cost_t, bool>{currentNode, nextMove, nextCost, actualCost, edgeCost.isPerturbated()});
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
            * note that the last node (the goal or a node with hOriginalCache set) is NOT 
            * included in the sequence but will have have h_value set in hOriginalCache;
            */
            
            rollback:
            //they are NOT infinites!
            cost_t totalOriginalCost = this->hOriginalCache[currentNode];
            cost_t totalPerturbatedCost = this->hPerturbatedCache[currentNode];
            nodeid_t lastPerturbatedSourceId = this->perturbatedSourceIdCache[currentNode];
            cost_t lastUnperturbatedPathCost = this->perturbatedPathCostCache[currentNode];
            
            for (auto tuple: nodeVisited.reverse()) {
                auto node = std::get<0>(tuple);
                auto originalCostOfMoveFromNodeToNext = std::get<2>(tuple);
                auto perturbatedCostOfMoveFromNodeToNext = std::get<3>(tuple);
                auto perturbatedMove = std::get<4>(tuple);

                totalOriginalCost += originalCostOfMoveFromNodeToNext;
                totalPerturbatedCost += perturbatedCostOfMoveFromNodeToNext;

                this->hOriginalCache[node] = totalOriginalCost;
                this->hPerturbatedCache[node] = totalPerturbatedCost;

                if (perturbatedMove) {
                    //if this move is perturbated, the new source which was lastly perturbated needs to be updated
                    lastPerturbatedSourceId = node;
                    lastUnperturbatedPathCost = 0;
                } else {
                    lastUnperturbatedPathCost += originalCostOfMoveFromNodeToNext;
                }
                this->perturbatedSourceIdCache[node] = lastPerturbatedSourceId;
                this->perturbatedPathCostCache[node] = lastUnperturbatedPathCost;
            }

            this->lastPathOriginalCost = totalOriginalCost;
            this->lastPathActualCost = totalPerturbatedCost;
            this->lastEarliestPerturbationSourceId = lastPerturbatedSourceId;
            this->lastEarliestPerturbationSourceIdCost = lastUnperturbatedPathCost;

            return totalOriginalCost;
        }
    public:
        virtual MemoryConsumption getByteMemoryOccupied() const {
            throw cpp_utils::exceptions::NotYetImplementedException{};
        }
    public:
        virtual void cleanup() {
            CpdHeuristic<STATE, G, V>::cleanUp();
            this->earliestPerturbationSourceIdCostCache.fill(cost_t::INFTY);
        }
    public:
        /**
         * @brief after the last call of the heuristic, represent sthe earliest perturbations
         * 
         * outputs the vertex id of the vertex which is the source of the first perturbated edge encountered by following the cpd path
         * from the evaluated node till the goal
         * 
         * @pre
         *  @li ::getHeuristic called
         * 
         * @return nodeid_t source id of the first perturbated edge encountered up until this point.
         */
        nodeid_t getLastEarliestNodeBeforePerturbation() const {
            return this->lastEarliestPerturbationSourceId;
        }
        /**
         * @brief the cost fo reaching output of ::getLastEarliestNodeBeforePerturbation from the node involve din the last call of the heuristic
         * 
         * @pre
         *  @li ::getHeuristic called
         * 
         * @return cost_t 
         */
        cost_t getLastEarliestPerturbationSourceIdCost() const {
            return this->lastEarliestPerturbationSourceIdCost;
        }
    };

}

#endif