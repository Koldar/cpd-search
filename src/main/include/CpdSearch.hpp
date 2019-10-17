#ifndef _CPD_SEARCH_TIMED_CPD_SEARCH_HEADER__
#define _CPD_SEARCH_TIMED_CPD_SEARCH_HEADER__

#include <pathfinding-utils/GraphState.hpp>
#include "CpdHeuristic.hpp"
#include <compressed-path-database/CpdManager.hpp>
#include <pathfinding-utils/ISearchAlgorithm.hpp>
#include <pathfinding-utils/IStatePruner.hpp>
#include <pathfinding-utils/IStateExpander.hpp>
#include <pathfinding-utils/IStateSupplier.hpp>
#include <pathfinding-utils/IHeuristic.hpp>
#include <pathfinding-utils/StandardLocationGoalChecker.hpp>
#include <pathfinding-utils/StandardStateExpander.hpp>
#include <boost/smart_ptr/make_unique.hpp>
#include <cpp-utils/listGraph.hpp>

namespace pathfinding::search {

    using namespace cpp_utils;
    using namespace compressed_path_database;

    template <typename G, typename V>
    std::unique_ptr<IImmutableGraph<G, V, PerturbatedCost>> getPerturbatedMap(const IImmutableGraph<G,V, cost_t>& graph, const cpp_utils::vectorplus<Edge<cost_t>>& perturbations) {
        auto result = new cpp_utils::graphs::ListGraph<G,V,PerturbatedCost>{};

        if (!graph.areVertexIdsContiguous()) {
            throw cpp_utils::exceptions::ImpossibleException{};
        }
        //vertex
        for (nodeid_t vertexId=0; vertexId<graph.numberOfVertices(); ++vertexId) {
            result->addVertex(graph.getVertex(vertexId));
        }

        //edges
        for (nodeid_t sourceId=0; sourceId<graph.numberOfVertices(); ++sourceId) {
            for (auto outEdge: graph.getOutEdges(sourceId)) {
                result->addEdge(sourceId, outEdge.getPayload(), PerturbatedCost{outEdge.getSinkId(), false});
            }
        }

        //apply perturbation
        for (auto perturbation : perturbations) {
            if (result->getEdge(perturbation.getSourceId(), perturbation.getSinkId()).getCost() > perturbation.getPayload()) {
                throw cpp_utils::exceptions::ImpossibleException{"perturbations are required for problem assumption to be greater than the original weight!"};
            }
            result->changeWeightUndirectedEdge(perturbation.getSourceId(), perturbation.getSinkId(), PerturbatedCost{perturbation.getPayload(), true});
        }

        return result;
    }

    /**
     * @brief CPD search
     * 
     * Perturbations
     * =============
     * 
     * Each perturbation:
     * @li can only increase an edge-cost, not decrease it;
     * @li is uniquely identified by the edge it involves, the new edge cost associated;
     * @li detected at the beginning of the path planning episode and then assumed static;
     * 
     * Feature
     * =======
     * 
     * - as heuristic we still use CpdHeuristic;
     * - if the cpdpath(n, g) do not have perturbations, we early terminate;
     * - use the bounded algorithm from ijcai2019 (path planning with cpd heuristics);
     * - the upperbound is retrieved by simulating the path generatede by cpdpath(n,g);
     * 
     * State
     * =====
     * 
     * The state is GraphState. the states are over the perturbated graph. It's not a big deal since GraphState uses the underlying graph only for <<.
     * 
     * 
     * @tparam G type of the payload of the underlying map graph
     * @tparam V type of the payload of each vertex in map graph
     */
    template <typename G, typename V>
    class CpdSearch: public IMemorable, public ISearchAlgorithm<GraphState<G, V, PerturbatedCost>, const GraphState<G, V, PerturbatedCost>*, const GraphState<G, V, PerturbatedCost>&> {
        typedef GraphState<G, V, PerturbatedCost> GraphStateReal;
    public:
        /**
         * @brief Construct a new Time Cpd Search object
         * 
         * @param heuristic 
         * @param goalChecker 
         * @param supplier 
         * @param expander 
         * @param pruner 
         * @param cpdManager cpd manager that will be used to poll the cpd. It needs to be already loaded
         * @param epsilon suboptimality bound to test
         * @param openListCapacity 
         */
        CpdSearch(CpdHeuristic<GraphStateReal, G, V>& heuristic, IGoalChecker<GraphStateReal>& goalChecker, IStateSupplier<GraphStateReal, nodeid_t>& supplier, StandardStateExpander<GraphStateReal, G, V, PerturbatedCost, PerturbatedCost::getCost>& expander, IStatePruner<GraphStateReal>& pruner, const CpdManager<G,V>& cpdManager, cost_t epsilon, unsigned int openListCapacity = 1024) : 
            heuristic{heuristic}, goalChecker{goalChecker}, supplier{supplier}, expander{expander}, pruner{pruner},
            epsilon{epsilon}, cpdManager{cpdManager},
            openList{nullptr} {
                if (!heuristic.isConsistent()) {
                    throw cpp_utils::exceptions::InvalidArgumentException{"the heuristic is not consistent!"};
                }
                this->openList = new StaticPriorityQueue<GraphStateReal>{openListCapacity, true};
            }

        ~CpdSearch() {
            this->tearDownSearch();
            delete this->openList;
        }
        //the class cannot be copied whatsoever
        CpdSearch(const CpdSearch& other) = delete;
        CpdSearch& operator=(const CpdSearch& other) = delete;

        CpdSearch(CpdSearch&& other): heuristic{other.heuristic}, goalChecker{other.goalChecker}, supplier{other.supplier}, expander{other.expander}, pruner{other.pruner}, epsilon{other.epsilon}, cpdManager{other.cpdManager}, openList{other.openList} {
            other.openList = nullptr;
        }

        CpdSearch& operator=(CpdSearch&& other) {
            this->heuristic = other.heuristic;
            this->goalChecker = other.goalChecker;
            this->supplier = other.supplier;
            this->expander = other.expander;
            this->pruner = other.pruner;
            this->epsilon = other.epsilon;
            this->cpdManager = other.cpdManager;
            this->openList = other.openList;
            other.openList = nullptr;
            return *this;
        }

    public:
        virtual MemoryConsumption getByteMemoryOccupied() const {
            throw cpp_utils::exceptions::NotYetImplementedException{__func__};
        }
    private:
        /**
         * @brief parameter that allows us to tweak the suboptimality bound of the algorithm
         * 
         */
        cost_t epsilon;
        /**
         * @brief manager of cpd
         * 
         */
        const CpdManager<G, V>& cpdManager;
        CpdHeuristic<GraphStateReal, G, V>& heuristic;
        IGoalChecker<GraphStateReal>& goalChecker;
        StandardStateExpander<GraphStateReal, G, V, PerturbatedCost, PerturbatedCost::getCost>& expander;
        IStateSupplier<GraphStateReal, nodeid_t>& supplier;
        IStatePruner<GraphStateReal>& pruner;
        StaticPriorityQueue<GraphStateReal>* openList;
    protected:
        virtual cost_t computeF(cost_t g, cost_t h) const {
            return g + h;
        }
    protected:
        virtual void setupSearch(const GraphStateReal& start, const GraphStateReal* goal) {
            //cleanup before running since at the end we may want to poll information on the other structures
            this->heuristic.cleanup();
            this->expander.cleanup();
            this->supplier.cleanup();
            this->pruner.cleanup();
            this->openList->clear();
        }
        virtual void tearDownSearch() {
        }
        virtual std::unique_ptr<ISolutionPath<const GraphStateReal*, const GraphStateReal&>> buildSolutionFromGoalFetched(const GraphStateReal& start, const GraphStateReal& actualGoal, const GraphStateReal* goal) {
            auto result = new StateSolutionPath<GraphStateReal>{};
            const GraphStateReal* tmp = &actualGoal;
            while (tmp != nullptr) {
                result->addHead(tmp);
                tmp = tmp->getParent();
            }
            return std::unique_ptr<StateSolutionPath<GraphStateReal>>{result};
        }
        virtual cost_t getSolutionCostFromGoalFetched(const GraphStateReal& start, const GraphStateReal& actualGoal, const GraphStateReal* goal) const {
            return actualGoal.getCost();
        }
        virtual const GraphStateReal& performSearch(GraphStateReal& start, const GraphStateReal* expectedGoal) {
            if (expectedGoal != nullptr) {
                info("starting A*! start = ", start, "goal = ", *expectedGoal);
            } else {
                info("starting A*! start = ", start, "goal = ", "none");
            }
            

            const GraphStateReal* goal = nullptr;
            cost_t upperbound = cost_t::INFTY;
            const GraphStateReal* earlyTerminationState = nullptr;

            start.setG(0);
            start.setH(this->heuristic.getHeuristic(start, expectedGoal));
            start.setF(this->computeF(start.getG(), start.getH()));

            this->openList->push(start);
            while (!this->openList->isEmpty()) {
                GraphStateReal& current = this->openList->peek();
                info("state ", current, "popped from open list f=", current.getF(), "g=", current.getG(), "h=", current.getH());

                //check if the peeked state is actually a goal
                if (this->goalChecker.isGoal(current, expectedGoal)) {
                    info("state ", current, "is a goal!");
                    goal = &current;
                    goto goal_found;
                }

                this->openList->pop();
                //current.getF() is also a lowerbound since the heuristic is admissible
                cost_t lowerbound = current.getF();
                

                /*
                * upperbound/lowebound <= epsilon 
                * which we derive
                * eps * lowerbound >= upperbound
                */
                if ((this->epsilon * current.getF()) >= upperbound) {
                    //return the solution by concatenating the current path from start to current and the cpdpath from current to goal
                    //(considering the perturbations!)
                    info("epsilon * lowerbound >= upperbound! ", epsilon, "*", current.getF(), ">=", upperbound);
                    info("early terminating from ", *earlyTerminationState, "up until ", *expectedGoal);
                    goal = this->earlyTerminate(*earlyTerminationState, expectedGoal);
                    goto goal_found;
                }

                current.markAsExpanded();

                info("computing successors of state ", current, "...");
                for(auto pair: this->expander.getSuccessors(current, this->supplier)) {
                    GraphStateReal& successor = pair.first;
                    cost_t current_to_successor_cost = pair.second;

                    if (this->pruner.shouldPrune(successor)) {
                        info("child", successor, "of state ", current, "should be pruned!");
                        //skip neighbours already expanded
                        continue;
                    }

                    if (this->openList->contains(successor)) {
                        //state inside the open list. Check if we need to update the path
                        cost_t gval = current.getG() + current_to_successor_cost;
                        if (gval < successor.getG()) {
                            info("child", successor, "of state ", current, "present in open list and has a lower g. update its parent!");

                            //update successor information
                            successor.setG(gval);
                            successor.setF(this->computeF(gval, successor.getH()));
                            const GraphStateReal* oldParent = successor.getParent();
                            successor.setParent(&current);

                            this->openList->decrease_key(successor);
                        }
                    } else {
                        //state is not present in open list. Add to it
                        cost_t gval = current.getG() + current_to_successor_cost;
                        cost_t hval = this->heuristic.getHeuristic(successor, expectedGoal);
                        successor.setG(gval);
                        successor.setH(hval);
                        successor.setF(this->computeF(gval, hval));
                        successor.setParent(&current);

                        //we may have a new upperbound of the solution
                        if (upperbound > gval + this->heuristic.getLastPerturbatedCost()) {
                            info("updating upperbound. upperbound: from", upperbound, "to", gval + this->heuristic.getLastPerturbatedCost());
                            if (earlyTerminationState == nullptr) {
                                info("updating incumbent. incumbent: from null to", successor);
                            } else {
                                info("updating incumbent. incumbent: from", *earlyTerminationState, "to", successor);
                            }
                            
                            upperbound = gval + this->heuristic.getLastPerturbatedCost();
                            earlyTerminationState = &successor;
                        }

                        info("child", successor, "of state ", current, "not present in open list. Add it f=", successor.getF(), "g=", successor.getG(), "h=", successor.getH());
                        this->openList->push(successor);
                    }
                }
            }
            info("found no solutions!");
            throw SolutionNotFoundException{};

            goal_found:
            return *goal;

        }
    private:
        const GraphStateReal* earlyTerminate(const GraphStateReal& state, const GraphStateReal* expectedGoal) {
            moveid_t nextMove;
            nodeid_t nextVertex;
            cost_t originalMoveCost;
            const GraphStateReal* currentState = &state;
            nodeid_t goalVertex = expectedGoal->getPosition();
            
            while (true) {
                if (this->goalChecker.isGoal(*currentState, expectedGoal)) {
                    return currentState;
                }
                if (this->cpdManager.getFirstMove(currentState->getPosition(), goalVertex, nextMove, nextVertex, originalMoveCost)) {
                    std::pair<GraphStateReal&, cost_t> pair = this->expander.getSuccessor(*currentState, nextMove, this->supplier);
                    GraphStateReal& successor = pair.first;
                    cost_t actionCost = pair.second;
                    successor.setParent(const_cast<GraphStateReal*>(currentState));
                    successor.setG(currentState->getG() + actionCost);
                    successor.setH(cost_t::INFTY);
                    currentState = &successor;
                } else {
                    throw cpp_utils::exceptions::ImpossibleException{};
                }
            }
        }
    };

    /**
     * @brief simple class that create the cpd search more easily
     * 
     */
    class CpdSearchFactory {
    public:
        template <typename G, typename V>
        struct output_t {
            public:
                /**
                 * @brief place where the goal checker is located in memory
                 * 
                 */
                StandardLocationGoalChecker<GraphState<G, V, PerturbatedCost>> goalChecker;
                /**
                 * @brief place where the state supplier is located in memory
                 * 
                 */
                GraphStateSupplier<G, V, PerturbatedCost> stateSupplier;
                /**
                 * @brief place where the state expander is located in memory
                 * 
                 */
                StandardStateExpander<GraphState<G, V, PerturbatedCost>, G, V, PerturbatedCost, PerturbatedCost::getCost> stateExpander;
                /**
                 * @brief place where the state pruner is located in memory
                 * 
                 */
                PruneIfExpanded<GraphState<G, V, PerturbatedCost>> statePruner;
                /**
                 * @brief place where the heuristic is located
                 * 
                 * @note this needs to be the last field declared here, since its dependent on other fields.
                 * @see https://wiki.sei.cmu.edu/confluence/display/cplusplus/OOP53-CPP.+Write+constructor+member+initializers+in+the+canonical+order
                 */
                CpdHeuristic<GraphState<G, V, PerturbatedCost>, G, V> heuristic;
                /**
                 * @brief place where CPD timed search algorithm is located
                 * 
                 * @note this needs to be the last field declared here, since its dependent on other fields.
                 * @see https://wiki.sei.cmu.edu/confluence/display/cplusplus/OOP53-CPP.+Write+constructor+member+initializers+in+the+canonical+order
                 */
                CpdSearch<G, V> search;
            public:
                output_t(
                    const CpdManager<G, V>& cpdManager,
                    const IImmutableGraph<G, V, PerturbatedCost>& perturbatedGraph,
                    cost_t epsilon
                    ): 
                    heuristic{cpdManager, perturbatedGraph},
                    goalChecker{}, 
                    stateSupplier{perturbatedGraph}, 
                    stateExpander{perturbatedGraph}, 
                    statePruner{},
                    search{this->heuristic, this->goalChecker, this->stateSupplier, this->stateExpander, this->statePruner, this->heuristic.getCpdManager(), epsilon, 1024} {
                }

                output_t(const output_t& other) = delete;
                output_t(output_t&& other) noexcept : search{::std::move(other.search)}, heuristic(std::move(other.heuristic)), goalChecker(std::move(other.goalChecker)), stateSupplier(std::move(other.stateSupplier)), stateExpander(std::move(other.stateExpander)), statePruner(std::move(other.statePruner)) {
                }
                output_t& operator =(const output_t& other) = delete;
                output_t& operator =(output_t&& other) {
                    this->search = std::move(other.search);
                    this->heuristic = std::move(other.heuristic);
                    this->goalChecker = std::move(other.goalChecker);
                    this->stateSupplier = std::move(other.stateSupplier);
                    this->stateExpander = std::move(other.stateExpander);
                    this->statePruner = std::move(other.statePruner);
                    return *this;
                }
                virtual ~output_t() {
                }
        };
    public:
        /**
         * @brief cpd search on gridmaps
         * 
         * @tparam G payload of the whole graph involved
         * @tparam V payload of a single vertex in the graph
         * 
         * @param[in] cpdManager manager for the cpd. 
         * @param[in] perturbations the perturbation you want to apply to the graph
         * @param[in] epsilon bound
         * @return cpd search algorithm
         */
        template <typename G, typename V>
        output_t<G,V> get(const CpdManager<G,V>& cpdManager, const IImmutableGraph<G, V, PerturbatedCost>& perturbatedGraph, cost_t epsilon) {
            return output_t<G, V>{
                cpdManager,
                perturbatedGraph,
                epsilon
            };
        }
    };

}

#endif