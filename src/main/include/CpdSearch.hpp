#ifndef _CPD_SEARCH_TIMED_CPD_SEARCH_HEADER__
#define _CPD_SEARCH_TIMED_CPD_SEARCH_HEADER__

#include <boost/smart_ptr/make_unique.hpp>

#include <cpp-utils/listGraph.hpp>
#include <cpp-utils/listeners.hpp>
#include <cpp-utils/Timer.hpp>
#include <cpp-utils/NumTracker.hpp>

#include <pathfinding-utils/GraphState.hpp>
#include <pathfinding-utils/ISearchAlgorithm.hpp>
#include <pathfinding-utils/IStatePruner.hpp>
#include <pathfinding-utils/IStateExpander.hpp>
#include <pathfinding-utils/IStateSupplier.hpp>
#include <pathfinding-utils/IHeuristic.hpp>
#include <pathfinding-utils/StandardLocationGoalChecker.hpp>
#include <pathfinding-utils/StandardStateExpander.hpp>
#include <pathfinding-utils/AstarListener.hpp>

#include <compressed-path-database/CpdManager.hpp>

#include "CpdHeuristic.hpp"
#include "CpdSearchListener.hpp"
#include "cpd_search_generated_e.hpp"


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
    class CpdSearch: public IMemorable, public ISearchAlgorithm<GraphState<G, V, PerturbatedCost, cpd_search_generated_e>, const GraphState<G, V, PerturbatedCost, cpd_search_generated_e>*, const GraphState<G, V, PerturbatedCost, cpd_search_generated_e>&>, public ISingleListenable<listeners::CpdSearchListener<G, V, GraphState<G, V, PerturbatedCost, cpd_search_generated_e>>> {
        using State = GraphState<G, V, PerturbatedCost, cpd_search_generated_e>;
        using Supplier = IStateSupplier<State, nodeid_t, cpd_search_generated_e>;
        using Expander = StandardStateExpander<State, G, V, PerturbatedCost, cpd_search_generated_e>;
        using This =  CpdSearch<G, V>;
        using Listener = listeners::CpdSearchListener<G, V, State>;
        using Super2 = ISingleListenable<Listener>;
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
        CpdSearch(CpdHeuristic<State, G, V>& heuristic, IGoalChecker<State>& goalChecker, Supplier& supplier, Expander& expander, IStatePruner<State>& pruner, const CpdManager<G,V>& cpdManager, cost_t epsilon, unsigned int openListCapacity = 1024) : Super2{},
            heuristic{heuristic}, goalChecker{goalChecker}, supplier{supplier}, expander{expander}, pruner{pruner},
            epsilon{epsilon}, cpdManager{cpdManager},
            openList{nullptr} {
                debug("CpdSearch constructor started!");
                if (!heuristic.isConsistent()) {
                    throw cpp_utils::exceptions::InvalidArgumentException{"the heuristic is not consistent!"};
                }
                this->openList = new StaticPriorityQueue<State>{openListCapacity, true};
                debug("CpdSearhc constructor ended!");
            }

        virtual ~CpdSearch() {
            this->tearDownSearch();
            delete this->openList;
        }
        //the class cannot be copied whatsoever
        CpdSearch(const CpdSearch& other) = delete;
        CpdSearch& operator=(const CpdSearch& other) = delete;

        CpdSearch(CpdSearch&& other): Super2{std::move(other)}, heuristic{other.heuristic}, goalChecker{other.goalChecker}, supplier{other.supplier}, expander{other.expander}, pruner{other.pruner}, epsilon{other.epsilon}, cpdManager{other.cpdManager}, openList{other.openList} {
            other.openList = nullptr;
        }

        CpdSearch& operator=(CpdSearch&& other) {
            Super2::operator=(::std::move(other));

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
        CpdHeuristic<State, G, V>& heuristic;
        IGoalChecker<State>& goalChecker;
        Expander& expander;
        Supplier& supplier;
        IStatePruner<State>& pruner;
        StaticPriorityQueue<State>* openList;
    public:
        virtual std::string getName() const {
            return "CPD-Search";
        }
        virtual void setupSearch(const State* start, const State* goal) {
            //cleanup before running since at the end we may want to poll information on the other structures
            this->heuristic.cleanup();
            this->expander.cleanup();
            this->supplier.cleanup();
            this->pruner.cleanup();
            this->openList->clear();
            
            this->listener->cleanup();
        }
        virtual void tearDownSearch() {
        }
    protected:
        virtual cost_t computeF(cost_t g, cost_t h) const {
            return g + h;
        }
    protected:
        virtual std::unique_ptr<ISolutionPath<State>> buildSolutionFromGoalFetched(const State& start, const State& actualGoal, const State* goal) {
            auto result = new StateSolutionPath<State>{};
            const State* tmp = &actualGoal;
            while (tmp != nullptr) {
                debug("adding ", tmp, "to solution");
                result->addHead(*tmp);
                tmp = tmp->getParent();
            }
            return std::unique_ptr<StateSolutionPath<State>>{result};
        }
        virtual cost_t getSolutionCostFromGoalFetched(const State& start, const State& actualGoal, const State* goal) const {
            return actualGoal.getCost();
        }
        virtual const State& performSearch(State& start, const State* expectedGoal) {
            if (expectedGoal != nullptr) {
                info("starting A*! start = ", start, "goal = ", *expectedGoal);
            } else {
                info("starting A*! start = ", start, "goal = ", "none");
            }
            

            int aStarIteration = 0;
            const State* goal = nullptr;
            cost_t upperbound = cost_t::INFTY;
            const State* earlyTerminationState = nullptr;

            start.setG(0);
            this->fireEvent([&, start, aStarIteration](Listener& l) {l.onStartingComputingHeuristic(aStarIteration, start); });
            start.setH(this->heuristic.getHeuristic(start, expectedGoal));
            this->fireEvent([&, start, aStarIteration](Listener& l) {l.onEndingComputingHeuristic(aStarIteration, start); });
            start.setF(this->computeF(start.getG(), start.getH()));

            this->openList->push(start);
            while (!this->openList->isEmpty()) {
                State& current = this->openList->peek();
                info("state ", current, "popped from open list f=", current.getF(), "g=", current.getG(), "h=", current.getH(), " pointer=", &current, "parent pointer=", current.getParent());

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
                    this->fireEvent([&, earlyTerminationState, goal, aStarIteration](Listener& l) {l.onEarlyTerminationActivated(aStarIteration, *earlyTerminationState, *goal); });

                    goto goal_found;
                }

                current.markAsExpanded();
                this->fireEvent([&, current, aStarIteration](Listener& l) {l.onNodeExpanded(aStarIteration, current); });

                info("computing successors of state ", current, "...");
                for(auto pair: this->expander.getSuccessors(current, this->supplier)) {
                    State& successor = pair.first;
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
                            const State* oldParent = successor.getParent();
                            successor.setParent(&current);

                            this->openList->decrease_key(successor);
                        }
                    } else {
                        //state is not present in open list. Add to it
                        cost_t gval = current.getG() + current_to_successor_cost;
                        this->fireEvent([&, successor, aStarIteration](Listener& l) {l.onStartingComputingHeuristic(aStarIteration, successor); });
                        cost_t hval = this->heuristic.getHeuristic(successor, expectedGoal);
                        this->fireEvent([&, successor, aStarIteration](Listener& l) {l.onEndingComputingHeuristic(aStarIteration, successor); });
                        
                        successor.setG(gval);
                        successor.setH(hval);
                        successor.setF(this->computeF(gval, hval));
                        successor.setParent(&current);
                        this->fireEvent([&, successor, aStarIteration](Listener& l) {l.onNodeGenerated(aStarIteration, successor); });

                        //we may have a new upperbound of the solution
                        if (upperbound > gval + this->heuristic.getLastPerturbatedCost()) {
                            info("updating upperbound. upperbound: from", upperbound, "to", gval + this->heuristic.getLastPerturbatedCost());
                            if (earlyTerminationState == nullptr) {
                                info("updating incumbent. incumbent: from null to", successor);
                            } else {
                                info("updating incumbent. incumbent: from", *earlyTerminationState, "to", successor);
                            }
                            
                            auto newupperbound = gval + this->heuristic.getLastPerturbatedCost();
                            this->fireEvent([&, successor, aStarIteration](Listener& l) {l.onUpperboundRevised(aStarIteration, successor, upperbound, newupperbound); });
                            upperbound = newupperbound;

                            earlyTerminationState = &successor;
                        }

                        info("child", successor, "of state ", current, "not present in open list. Add it f=", successor.getF(), "g=", successor.getG(), "h=", successor.getH());
                        this->openList->push(successor);
                    }
                }

                aStarIteration += 1;
            }
            info("found no solutions!");
            throw SolutionNotFoundException{};

            goal_found:
            return *goal;

        }
    private:
        const State* earlyTerminate(const State& state, const State* expectedGoal) {
            moveid_t nextMove;
            nodeid_t nextVertex;
            cost_t originalMoveCost;
            const State* currentState = &state;
            nodeid_t goalVertex = expectedGoal->getPosition();
            
            while (true) {
                if (this->goalChecker.isGoal(*currentState, expectedGoal)) {
                    return currentState;
                }
                if (this->cpdManager.getFirstMove(currentState->getPosition(), goalVertex, nextMove, nextVertex, originalMoveCost)) {
                    std::pair<State&, cost_t> pair = this->expander.getSuccessor(*currentState, nextMove, this->supplier);
                    State& successor = pair.first;
                    cost_t actionCost = pair.second;
                    successor.setParent(const_cast<State*>(currentState));
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
                using State = GraphState<G, V, PerturbatedCost, cpd_search_generated_e>;
            public:
                /**
                 * @brief place where the goal checker is located in memory
                 * 
                 */
                StandardLocationGoalChecker<State> goalChecker;
                /**
                 * @brief place where the state supplier is located in memory
                 * 
                 */
                GraphStateSupplier<G, V, PerturbatedCost, cpd_search_generated_e> stateSupplier;
                /**
                 * @brief place where the state expander is located in memory
                 * 
                 */
                StandardStateExpander<State, G, V, PerturbatedCost, cpd_search_generated_e> stateExpander;
                /**
                 * @brief place where the state pruner is located in memory
                 * 
                 */
                PruneIfExpanded<State> statePruner;
                /**
                 * @brief place where the heuristic is located
                 * 
                 * @note this needs to be the last field declared here, since its dependent on other fields.
                 * @see https://wiki.sei.cmu.edu/confluence/display/cplusplus/OOP53-CPP.+Write+constructor+member+initializers+in+the+canonical+order
                 */
                CpdHeuristic<State, G, V> heuristic;
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
                    cost_t epsilon,
                    const cpp_utils::function_t<PerturbatedCost, cost_t>& costFunction
                    ): 
                    goalChecker{}, 
                    heuristic{cpdManager, perturbatedGraph},
                    stateSupplier{perturbatedGraph}, 
                    stateExpander{perturbatedGraph, costFunction}, 
                    statePruner{},
                    search{this->heuristic, this->goalChecker, this->stateSupplier, this->stateExpander, this->statePruner, this->heuristic.getCpdManager(), epsilon, 1024} {
                        debug("output factory constructor correctly called!");
                }

                output_t(const output_t& other) = delete;
                output_t(output_t&& other) noexcept : search{::std::move(other.search)}, heuristic(std::move(other.heuristic)), goalChecker(std::move(other.goalChecker)), stateSupplier(std::move(other.stateSupplier)), stateExpander(std::move(other.stateExpander)), statePruner(std::move(other.statePruner)) {
                    debug("output factory move constructor correctly called!");
                }
                output_t& operator =(const output_t& other) = delete;
                output_t& operator =(output_t&& other) {
                    debug("calling factory move =!");
                    this->search = std::move(other.search);
                    this->heuristic = std::move(other.heuristic);
                    this->goalChecker = std::move(other.goalChecker);
                    this->stateSupplier = std::move(other.stateSupplier);
                    this->stateExpander = std::move(other.stateExpander);
                    this->statePruner = std::move(other.statePruner);

                    debug("output factory move = correctly called!");
                    return *this;
                }
                virtual ~output_t() {
                    debug("output destructed!");
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
        output_t<G,V>* get(const CpdManager<G,V>& cpdManager, const IImmutableGraph<G, V, PerturbatedCost>& perturbatedGraph, cost_t epsilon) {
            debug("generating output...");
            cpp_utils::function_t<PerturbatedCost, cost_t> costFunction = [&](const PerturbatedCost& p) { return p.getCost(); };
            return new output_t<G, V>{
                cpdManager,
                perturbatedGraph,
                epsilon,
                costFunction
            };
        }
    };

}

#endif