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
#include <boost/smart_ptr/make_unique.hpp>

namespace pathfinding::search {

    using namespace cpp_utils;
    using namespace cpd;

    

    /**
     * @brief Time CPD search
     * 
     * Perturbations
     * =============
     * 
     * Each perturbation:
     * @li can only increase an edge-cost, not decrease it;
     * @li is uniquely identified by the timestamp it starts, the timestamp it ends, the edge it involves, the new edge cost associated;
     * 
     * Feature
     * =======
     * 
     * - as heuristic we still use CpdHeuristic;
     * - if the cpdpath(n, g) do not have timed perturbations, we early terminate;
     * - use the bounded algorithm from ijcai2019 (path planning with cpd heuristics);
     * - in the successor, we privilege actions which **do** move the agent. The waits are considered lastly:
     *  the underlying reason is that we want to wait as late as possible. Concretely, when considering the successors if
     *  no successors has a perturbated edge, then we do not generate the wait action;
     * - the upperbound is retrieved by simulating the path generatede by cpdpath(n,g);
     * - if the cpdpath(n, g) is +infty (the costs) we try to wait for k timestamps and check if it is available now (k represents a family of heuristics);
     * 
     * 
     * @tparam GraphState<G,V> 
     * @tparam G 
     * @tparam V 
     * @tparam OTHER 
     */
    template <typename G, typename V>
    class CpdSearch: public IMemorable, public ISearchAlgorithm<GraphState<G,V>, const GraphState<G,V>*, const GraphState<G,V>&> {
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
        CpdSearch(CpdHeuristic<GraphState<G,V>, G, V>& heuristic, IGoalChecker<GraphState<G,V>>& goalChecker, IStateSupplier<GraphState<G,V>, nodeid_t>& supplier, GraphStateExpander<G, V>& expander, IStatePruner<GraphState<G,V>>& pruner, const CpdManager<G,V>& cpdManager, cost_t epsilon, unsigned int openListCapacity = 1024) : 
            heuristic{heuristic}, goalChecker{goalChecker}, supplier{supplier}, expander{expander}, pruner{pruner},
            epsilon{epsilon}, cpdManager{cpdManager},
            openList{nullptr} {
                if (!heuristic.isConsistent()) {
                    throw cpp_utils::exceptions::InvalidArgumentException{"the heuristic is not consistent!"};
                }
                this->openList = new StaticPriorityQueue<GraphState<G,V>>{openListCapacity, true};
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
        CpdHeuristic<GraphState<G,V>, G, V>& heuristic;
        IGoalChecker<GraphState<G,V>>& goalChecker;
        GraphStateExpander<G, V>& expander;
        IStateSupplier<GraphState<G,V>, nodeid_t>& supplier;
        IStatePruner<GraphState<G,V>>& pruner;
        StaticPriorityQueue<GraphState<G,V>>* openList;
    protected:
        virtual cost_t computeF(cost_t g, cost_t h) const {
            return g + h;
        }
    protected:
        virtual void setupSearch(const GraphState<G,V>& start, const GraphState<G,V>* goal) {
            //cleanup before running since at the end we may want to poll information on the other structures
            this->heuristic.cleanup();
            this->expander.cleanup();
            this->supplier.cleanup();
            this->pruner.cleanup();
            this->openList->clear();
        }
        virtual void tearDownSearch() {
        }
        virtual std::unique_ptr<ISolutionPath<const GraphState<G,V>*, const GraphState<G,V>&>> buildSolutionFromGoalFetched(const GraphState<G,V>& start, const GraphState<G,V>& actualGoal, const GraphState<G,V>* goal) {
            auto result = new StateSolutionPath<GraphState<G,V>>{};
            const GraphState<G,V>* tmp = &actualGoal;
            while (tmp != nullptr) {
                result->addHead(tmp);
                tmp = tmp->getParent();
            }
            return std::unique_ptr<StateSolutionPath<GraphState<G,V>>>{result};
        }
        virtual cost_t getSolutionCostFromGoalFetched(const GraphState<G,V>& start, const GraphState<G,V>& actualGoal, const GraphState<G,V>* goal) const {
            return actualGoal.getCost();
        }
        virtual const GraphState<G,V>& performSearch(GraphState<G,V>& start, const GraphState<G,V>* expectedGoal) {
            if (expectedGoal != nullptr) {
                info("starting A*! start = ", start, "goal = ", *expectedGoal);
            } else {
                info("starting A*! start = ", start, "goal = ", "none");
            }
            

            const GraphState<G,V>* goal = nullptr;
            cost_t upperbound = cost_t::INFTY;
            const GraphState<G,V>* earlyTerminationState = nullptr;

            start.setG(0);
            start.setH(this->heuristic.getHeuristic(start, expectedGoal));
            start.setF(this->computeF(start.getG(), start.getH()));

            this->openList->push(start);
            while (!this->openList->isEmpty()) {
                GraphState<G,V>& current = this->openList->peek();
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
                    goal = this->earlyTerminate(*earlyTerminationState, expectedGoal);
                    goto goal_found;
                }

                current.markAsExpanded();

                info("computing successors of state ", current, "...");
                for(auto pair: this->expander.getSuccessors(current, this->supplier)) {
                    GraphState<G,V>& successor = pair.first;
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
                            const GraphState<G,V>* oldParent = successor.getParent();
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
        const GraphState<G,V>* earlyTerminate(const GraphState<G,V>& state, const GraphState<G,V>* expectedGoal) {
            moveid_t nextMove;
            nodeid_t nextVertex;
            cost_t originalMoveCost;
            const GraphState<G,V>* currentState = &state;
            nodeid_t goalVertex = expectedGoal->getPosition();
            
            while (true) {
                if (this->goalChecker.isGoal(*currentState, expectedGoal)) {
                    return currentState;
                }
                if (this->cpdManager.getFirstMove(currentState->getPosition(), goalVertex, nextMove, nextVertex, originalMoveCost)) {
                    std::pair<GraphState<G,V>&, cost_t> pair = this->expander.getSuccessor(*currentState, nextMove, this->supplier);
                    GraphState<G,V>& successor = pair.first;
                    cost_t actionCost = pair.second;
                    successor.setParent(const_cast<GraphState<G,V>*>(currentState));
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
                GraphStateGoalChecker<G, V> goalChecker;
                /**
                 * @brief place where the state supplier is located in memory
                 * 
                 */
                GraphStateSupplier<G, V> stateSupplier;
                /**
                 * @brief place where the state expander is located in memory
                 * 
                 */
                GraphStateExpander<G, V> stateExpander;
                /**
                 * @brief place where the state pruner is located in memory
                 * 
                 */
                PruneIfExpanded<GraphState<G, V>> statePruner;
                /**
                 * @brief place where the heuristic is located
                 * 
                 * @note this needs to be the last field declared here, since its dependent on other fields.
                 * @see https://wiki.sei.cmu.edu/confluence/display/cplusplus/OOP53-CPP.+Write+constructor+member+initializers+in+the+canonical+order
                 */
                CpdHeuristic<GraphState<G, V>, G, V> heuristic;
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
                    GraphStateGoalChecker<G, V>&& goalChecker, 
                    GraphStateSupplier<G, V>&& stateSupplier, 
                    GraphStateExpander<G, V>&& stateExpander, 
                    PruneIfExpanded<GraphState<G, V>>&& statePruner,
                    cost_t epsilon
                    ): 
                    goalChecker{::std::move(goalChecker)}, 
                    stateSupplier{::std::move(stateSupplier)}, 
                    stateExpander{::std::move(stateExpander)}, 
                    statePruner{::std::move(statePruner)},
                    heuristic{cpdManager},
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
        output_t<G,V> get(const CpdManager<G,V>& cpdManager, const IImmutableGraph<G, V, cost_t>& perturbatedGraph, cost_t epsilon) {
            auto stateSupplier = GraphStateSupplier<G, V>{perturbatedGraph};
            auto stateExpander = GraphStateExpander<G, V>{perturbatedGraph};
            auto goalChecker = GraphStateGoalChecker<G, V>{};
            auto statePruner = PruneIfExpanded<GraphState<G, V>>{};
            
            return output_t<G, V>{
                cpdManager,
                std::move(goalChecker),
                std::move(stateSupplier),
                std::move(stateExpander),
                std::move(statePruner),
                epsilon
            };
        }
    };

}

#endif