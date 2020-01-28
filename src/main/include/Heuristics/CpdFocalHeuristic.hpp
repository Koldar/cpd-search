#ifndef _CPD_SEARCH_CPD_FOCAL_HEURISTIC_HEADER__
#define _CPD_SEARCH_CPD_FOCAL_HEURISTIC_HEADER__

#include <tuple>
#include <cpp-utils/igraph.hpp>
#include <pathfinding-utils/IHeuristic.hpp>
#include <compressed-path-database/CpdManager.hpp>
#include <cpp-utils/exceptions.hpp>
#include "PerturbatedCost.hpp"
#include "CpdHeuristic.hpp"

namespace pathfinding::search {

    using namespace cpp_utils::graphs;

    /**
     * @brief CPDHeuristic
     * 
     * this heuristic will yield, for every state, the cost of the CPD path from the state and the goal
     * 
     * Furthermore, this heuristic will also tells you what si the next perturbation if you were to follow the cpd path from a given node, as well as its cost
     * 
     * 
     * @tparam STATE 
     * @tparam G 
     * @tparam V 
     */
    //TODO rename it insomething like CpdPerturbatedHeuristic
    template <typename STATE, typename G, typename V>
    class CpdFocalHeuristic: public CpdHeuristic<STATE, G, V> {
        using This = CpdFocalHeuristic<STATE, G, V>;
        using Super = CpdHeuristic<STATE, G, V>;
    private:
        /**
         * @brief for each vertex in the graph, represents the cost we need to pay to go from the given node till the source of the first perturbation, if we were to follow the CPD path
         * 
         * cost_t::INFTY if the cost has not been initialized yet
         */
        vectorplus<cost_t> perturbatedPathCostCache;
        vectorplus<nodeid_t> perturbatedSourceIdCache;
        /**
         * @brief a cache that tells us what is the node we need to go to from a given one using the cpd
         * 
         * In this way we avoid calling the cpd multiple times
         * 
         * The index of the vector is the id of the node we're interested in while the associated cell value is the id
         * of the move we need to perform.
         * 
         * If the move is greater than out degree of the vertex, it means it has not been set yet.
         * 
         * The next move of the goal is undefined
         */
        vectorplus<moveid_t> nextNodeCache;
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
    public:
        /**
         * @brief Construct a new Cpd Heuristic object
         * 
         * @param cpdManager 
         * @param perturbatedGraph the graph we're going to use to fetch the costs fo the edges on the perturbated graph
         */
        CpdFocalHeuristic(const compressed_path_database::CpdManager<G, V>& cpdManager, const IImmutableGraph<G, V, PerturbatedCost>& perturbatedGraph): 
            Super{cpdManager, perturbatedGraph},
            perturbatedPathCostCache{cpdManager.getReorderedGraph().size(), cost_t::INFTY},
            perturbatedSourceIdCache{cpdManager.getReorderedGraph().size(), 0},
            nextNodeCache{cpdManager.getReorderedGraph().size(), static_cast<moveid_t>(cpdManager.getReorderedGraph().getMaxOutDegree() + 1)},
            lastEarliestPerturbationSourceId{0}, lastEarliestPerturbationSourceIdCost{cost_t::INFTY}
            {
        }
        CpdFocalHeuristic(const This& other) = delete;
        CpdFocalHeuristic(This&& h): Super{::std::move(h)},
            lastEarliestPerturbationSourceId{h.lastEarliestPerturbationSourceId}, 
            lastEarliestPerturbationSourceIdCost{h.lastEarliestPerturbationSourceIdCost},
            perturbatedPathCostCache{::std::move(h.perturbatedPathCostCache)}, 
            perturbatedSourceIdCache{::std::move(h.perturbatedSourceIdCache)},
            nextNodeCache{::std::move(h.nextNodeCache)} 
            {
            
        }
        This& operator =(const This& h) = delete;
        This& operator =(This&& h) {
            Super::operator =(::std::move(h));
            this->lastEarliestPerturbationSourceId = h.lastEarliestPerturbationSourceId;
            this->lastEarliestPerturbationSourceIdCost = h.lastEarliestPerturbationSourceIdCost;
            this->perturbatedPathCostCache = std::move(h.perturbatedPathCostCache);
            this->perturbatedSourceIdCache = ::std::move(h.perturbatedSourceIdCache);
            this->nextNodeCache = ::std::move(h.nextNodeCache);
            return *this;
        }
        virtual ~CpdFocalHeuristic() {
            debug("destroy cpd focal heuristic at", this);
        }
    public:
        virtual cost_t getHeuristic(const STATE& current, const STATE* agoal) {
            const STATE& goal = *agoal;

            moveid_t nextMove;
            nodeid_t nextNodeCache;
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

                if (this->cpdManager.getFirstMove(currentNode, goal.getPosition(), nextMove, nextNodeCache, nextCost)) {
                    //generated a move
                    auto edgeCost = this->perturbatedGraph.getOutEdge(currentNode, nextMove).getPayload();
                    cost_t actualCost = edgeCost.getCost();
                    nodeVisited.addTail(std::tuple<nodeid_t, moveid_t, cost_t, cost_t, bool>{currentNode, nextMove, nextCost, actualCost, edgeCost.isPerturbated()});
                    currentNode = nextNodeCache;
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
                auto nextMove = std::get<1>(tuple);
                auto originalCostOfMoveFromNodeToNext = std::get<2>(tuple);
                auto perturbatedCostOfMoveFromNodeToNext = std::get<3>(tuple);
                auto perturbatedMove = std::get<4>(tuple);

                totalOriginalCost += originalCostOfMoveFromNodeToNext;
                totalPerturbatedCost += perturbatedCostOfMoveFromNodeToNext;

                this->hOriginalCache[node] = totalOriginalCost;
                this->hPerturbatedCache[node] = totalPerturbatedCost;
                this->nextNodeCache[node] = nextMove;

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
            throw cpp_utils::exceptions::NotYetImplementedException{"not mplemented"};
        }
    public:
        virtual void cleanup() {
            CpdHeuristic<STATE, G, V>::cleanup();
            this->perturbatedPathCostCache.fill(cost_t::INFTY);
            this->perturbatedSourceIdCache.fill(0);
            this->lastEarliestPerturbationSourceId = 0;
            this->lastEarliestPerturbationSourceIdCost = cost_t::INFTY;
        }
    public:
        /**
         * @brief last vertex on the cpd path, after which the cpd path becomes perturbated
         * 
         * @param node node where we start the cpd path towrds the goal
         * @return nodeid_t last vertex of the subpath @c node to @c goal which is unperturbated
         */
        nodeid_t getPerturbatedSourceId(nodeid_t node) const {
            return this->perturbatedSourceIdCache[node];
        }
        /**
         * @brief the cost we need to pay for reaching the first perturbation from the given node, using the cpd paths
         * 
         * @param node node involved
         * @return cost_t cost of the path
         */
        cost_t getPerturbatedPathCost(nodeid_t node) const {
            return this->perturbatedPathCostCache[node];
        }
        /**
         * @brief get the move to perform to follow the cpd path from node @c node
         * 
         * @param node the node where the cpd path starts
         * @return nodeid_t the next move to perform in order to follow the cpd path
         */
        moveid_t getNextMove(nodeid_t node) const {
            return this->nextNodeCache[node];
        }
        /**
         * @brief get the node on the cpd path starting from the node @c node
         * 
         * @param node the node where the cpd path starts
         * @return nodeid_t the next node along the cpd path
         */
        nodeid_t getNextNode(nodeid_t node) const {
            return this->originalGraph.getOutEdge(node, this->nextNodeCache[node]).sinkId;
        }
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