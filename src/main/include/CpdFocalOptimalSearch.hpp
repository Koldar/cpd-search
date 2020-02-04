#ifndef _CPD_SEARCH_CPD_FOCAL_SEARCH_OPTIMAL_HEADER__
#define _CPD_SEARCH_CPD_FOCAL_SEARCH_OPTIMAL_HEADER__

#include <cpp-utils/listeners.hpp>
#include <cpp-utils/MCValue.hpp>
#include <cpp-utils/MDValue.hpp>
#include <cpp-utils/LogNumberListener.hpp>

#include <pathfinding-utils/types.hpp>
#include <pathfinding-utils/FocalList.hpp>
#include <pathfinding-utils/ISearchAlgorithm.hpp>
#include <pathfinding-utils/IStatePruner.hpp>
#include <pathfinding-utils/StandardStateExpander.hpp>
#include <pathfinding-utils/StandardLocationGoalChecker.hpp>
#include <pathfinding-utils/NeverPrune.hpp>

#include "GraphFocalState.hpp"
#include "CpdFocalHeuristic.hpp"
#include "CpdFocalExpander.hpp"
#include "CpdFocalSupplier.hpp"
#include "CpdFocalSearchListener.hpp"

namespace pathfinding::search {

    using namespace compressed_path_database;
    using namespace pathfinding;
    using namespace pathfinding::data_structures;

    namespace internal {

        template <typename G, typename V>
        struct OpenListOptimalOrderer {
            bool operator ()(const GraphFocalState<G, V, PerturbatedCost>& a, const GraphFocalState<G, V, PerturbatedCost>& b) {
                if (a.getF() != b.getF()) {
                    //f are different
                    return a.getF() < b.getF();
                } else {
                    //f are the same. Tie breaking by sleectnig the state with bigger g
                    return a.getG() > b.getG();
                }
            }
        };

        template <typename G, typename V>
        struct FocalListOptimalOrderer {
            bool operator ()(const GraphFocalState<G, V, PerturbatedCost>& a, const GraphFocalState<G, V, PerturbatedCost>& b) {
                //f are the same. Tie breaking by sleectnig the state with bigger g
                return a.getG() > b.getG();
            }
        };

        template <typename G, typename V>
        struct FocalListOptimalSmallerFOrderer {
            bool operator ()(const GraphFocalState<G, V, PerturbatedCost>& a, const GraphFocalState<G, V, PerturbatedCost>& b) {
                //f are the same. Tie breaking by selecting the one with less f
                return a.getF() < b.getF();
            }
        };

        template <typename G, typename V>
        struct GraphFocalOptimalStateGetCost {
            cost_t operator()(const GraphFocalState<G, V, PerturbatedCost>& state) {
                return state.getF();
            }
        };

    }

    /**
     * @brief CPD Focal search Optimal
     * 
     * 
     * Like CPD Focal Search (::CPDFocalSearch) but with he gimmicky that when we find a solution we keep going up until 
     * we reach the optimal solution. This algorithm is optimal. To make the bounded suboptimal algorithm CpdFocalSearch optimal
     * we need to keep going until the upperbound equals to lowerbound.
     * 
     * As soon as a solution is found, the upperbound is set to its cost.
     * 
     * 
     * IMportant notes
     * ===============
     * 
     * @li when we revise the upperbound in the successors of a state we don't save the successors that has revised the upperbound last, since such successor it not necessary the next element popped from focal list: in this step we don't compute a new goal. Hence for this very reason, goal->getG() != upperbound (albeit at the ne dof the algorithm we have that goal->getG() == upperbound)
     * @li when we early terminate or wyhen we get a new goal, we check first if the new path is actually shorter than the previous one. this because this search is NOT a bestFirst search, so it is possible to obtain worse solution temporary;
     * @li the heuristic is consisent, but the search is not a bestFirst search, so items in the closed list can be put in open again if new_g < old_g;
     * @li if the upperbound is less than the lowerbound and no solutions have been found, the search has no solutions.
     * 
     * @tparam GraphState<G, V, cost_t> 
     * @tparam G 
     * @tparam V 
     * @tparam OTHER 
     */
    template <typename G, typename V>
    class CpdFocalOptimalSearch: public IMemorable, public ISearchAlgorithm<GraphFocalState<G, V, PerturbatedCost>, const GraphFocalState<G, V, PerturbatedCost>*, const GraphFocalState<G, V, PerturbatedCost>&>, public ISingleListenable<listeners::CpdFocalSearchListener<G, V, GraphFocalState<G, V, PerturbatedCost>>> {
        using State = GraphFocalState<G, V, PerturbatedCost>;
        using This = CpdFocalOptimalSearch<G, V>;
        using Listener = listeners::CpdFocalSearchListener<G, V, State>;
        using Super2 = ISingleListenable<Listener>;
    private:
        /**
         * @brief parameter that allows us to tweak the suboptimality bound of the algorithm
         * 
         */
        fractional_number<cost_t> epsilon;
        /**
         * @brief manager of cpd
         * 
         */
        const CpdManager<G, V>& cpdManager;

        CpdFocalHeuristic<State, G, V>& heuristic;
        IGoalChecker<State>& goalChecker;
        CpdFocalExpander<G, V>& expander;
        IStateSupplier<State, nodeid_t, cpd_search_generated_e>& supplier;
        IStatePruner<State>& pruner;

        FocalList<State, internal::OpenListOptimalOrderer<G,V>, internal::FocalListOptimalOrderer<G,V>, internal::GraphFocalOptimalStateGetCost<G,V>, cost_t>* focalList;
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
         */
        CpdFocalOptimalSearch(CpdFocalHeuristic<State, G, V>& heuristic, IGoalChecker<State>& goalChecker, IStateSupplier<State, nodeid_t, cpd_search_generated_e>& supplier, CpdFocalExpander<G, V>& expander, IStatePruner<State>& pruner, const CpdManager<G,V>& cpdManager, const fractional_number<cost_t>& focalListWeight, const fractional_number<cost_t>& epsilon) : Super2{},
            heuristic{heuristic}, goalChecker{goalChecker}, supplier{supplier}, expander{expander}, pruner{pruner},
            epsilon{epsilon}, cpdManager{cpdManager}
            {

            this->focalList = new FocalList<State, internal::OpenListOptimalOrderer<G,V>, internal::FocalListOptimalOrderer<G,V>, internal::GraphFocalOptimalStateGetCost<G,V>, cost_t>{focalListWeight};
            if (!heuristic.isConsistent()) {
                throw cpp_utils::exceptions::InvalidArgumentException{"the heuristic is not consistent!"};
            }
        }

        virtual ~CpdFocalOptimalSearch() {
            this->tearDownSearch();
            delete this->focalList;
        }
        //the class cannot be copied whatsoever
        CpdFocalOptimalSearch(const This& other) = delete;
        This& operator=(const CpdFocalOptimalSearch& other) = delete;

        CpdFocalOptimalSearch(This&& other): Super2{other}, heuristic{other.heuristic}, goalChecker{other.goalChecker}, supplier{other.supplier}, expander{other.expander}, pruner{other.pruner}, epsilon{other.epsilon}, cpdManager{other.cpdManager}, focalList{::std::move(other.focalList)} {
        }

        This& operator=(This&& other) {
            Super2::operator=(other);

            this->heuristic = other.heuristic;
            this->goalChecker = other.goalChecker;
            this->supplier = other.supplier;
            this->expander = other.expander;
            this->pruner = other.pruner;
            this->epsilon = other.epsilon;
            this->cpdManager = other.cpdManager;
            this->focalList = ::std::move(other.focalList);

            return *this;
        }

    public:
        virtual MemoryConsumption getByteMemoryOccupied() const {
            throw cpp_utils::exceptions::NotYetImplementedException{__func__};
        }
    protected:
        virtual cost_t computeF(cost_t g, cost_t h) const {
            return g + h;
        }
    public:
        virtual std::string getName() const {
            return "CPD-Focal-Search";
        }
        virtual void setupSearch(const State* start, const State* goal) {
            //cleanup before running since at the end we may want to poll information on the other structures
            this->heuristic.cleanup();
            this->expander.cleanup();
            this->supplier.cleanup();
            this->pruner.cleanup();

            this->focalList->cleanup();

            this->listener->cleanup();
        }
        virtual void tearDownSearch() {
        }
    protected:

        /**
         * @brief generate the states between from and to by following the cpd path
         * 
         * In the return value neither @c from and @¢ to are present, only the interstates.
         * 
         * This means that if `from` is adjacent to `to` we will return.
         * 
         * @pre
         *  @li from != to
         * 
         * The vector return starts from the state near `from` and go up until it reaches the previous state of `to`
         * 
         * @param from the search node where we start
         * @param to the search node where we need to go
         * @return vectorplus<const State*> list of possibly new state generated going from @c from to @c to
         */
        virtual vectorplus<const State*> connectStates(const State* from, const State* to) const {
            vectorplus<const State*> result{};
            const State* tmp = from;
            //result contains only the nodes between from and to, from and to excluded
            if (from->getPosition() == to->getPosition()) {
                return result;
            }
            while(true) {
                moveid_t nextMove;
                nodeid_t nextNode;
                cost_t moveCost;
                if (!this->cpdManager.getFirstMove(tmp->getPosition(), to->getPosition(), nextMove, nextNode, moveCost)) {
                    throw cpp_utils::exceptions::ImpossibleException{};
                }
                State* tmp2 = &this->supplier.getState(nextNode, cpd_search_generated_e::FROM_SOLUTION);
                tmp2->setParent(const_cast<State*>(tmp));
                if (tmp2->getPosition() == to->getPosition()) {
                    goto exit;
                }
                
                result.add(tmp2);
                tmp = tmp2;
            }

            exit:;
            return result;
        }
        virtual std::unique_ptr<ISolutionPath<State>> buildSolutionFromGoalFetched(const State& start, const State& actualGoal, const State* goal) {
            auto result = new StateSolutionPath<State>{};

            if (actualGoal == start) {
                result->add(start);
            } else {
                const State* cpdPathEnd = &actualGoal;
                const State* cpdPathStart = actualGoal.getParent();
                
                while (cpdPathStart != nullptr) {
                    info("need to connect path from", *cpdPathStart, " to ", *cpdPathEnd);
                    //we need to rebuild the path, since the search has jumped from a loation to another one via the CPD.
                    //specifically, we need to connect "tmp" with the head of the building solution, with the CPD until the CPD finds such head
                    debug("adding", *cpdPathEnd, "in solution");
                    result->addHead(*cpdPathEnd);
                    vectorplus<const State*> connect = this->connectStates(cpdPathStart, cpdPathEnd);
                    for (auto s : connect.reverse()) {
                        debug("adding ", s, "in solution");
                        result->addHead(*s);
                    }

                    cpdPathEnd = cpdPathStart;
                    cpdPathStart = cpdPathStart->getParent();
                }
                //add the start
                result->addHead(*cpdPathEnd);
            }

            info(*result);
            
            return std::unique_ptr<StateSolutionPath<State>>{result};
        }
        virtual cost_t getSolutionCostFromGoalFetched(const State& start, const State& actualGoal, const State* goal) const {
            return actualGoal.getCost();
        }
        virtual const State& performSearch(State& start, const State* expectedGoal) {
            info("************************* NEW SEARCH *****************************");
            if (expectedGoal != nullptr) {
                info("starting A*! start = ", start, "goal = ", *expectedGoal);
            } else {
                info("starting A*! start = ", start, "goal = ", "none");
            }
            
            int aStarIteration = 0;
            const State* goal = nullptr;

            MDValue<cost_t> upperbound{cost_t::INFTY};
            //TODO if we add the number listener, for some weird reason there is an invalid free as soon we  call the onNumberDecreased callback method... -.-""
            // auto upperboundListener = LogNumberListener<cost_t>{"upperbound", 8};
            // upperbound.setListener(upperboundListener);
            /*
             * the smallest f-value in the open list. this value tends to get bigger and bigger
             */
            MCValue<cost_t> lowerbound{cost_t{0}};
            //TODO if we add the number listener, for some weird reason there is an invalid free as soon we  call the onNumberDecreased callback method... -.-""
            // auto lowerboundListener = LogNumberListener<cost_t>{"lowerbound", 8};
            // lowerbound.setListener(lowerboundListener);

            State* startPtr = &start;
            start.setG(0);
            this->fireEvent([startPtr, aStarIteration](Listener& l) {l.onStartingComputingHeuristic(aStarIteration, *startPtr); });
            start.setH(this->heuristic.getHeuristic(start, expectedGoal));
            this->fireEvent([startPtr, aStarIteration](Listener& l) {l.onEndingComputingHeuristic(aStarIteration, *startPtr); });
            

            //update lastEarliestPerturbationSourceId
            start.updateEarlyPerturbationInfo(
                this->heuristic.getLastEarliestNodeBeforePerturbation(),
                this->heuristic.getLastEarliestPerturbationSourceIdCost()
            );

            start.setF(this->computeF(start.getG(), start.getH()));
            this->focalList->pushInOpenAndInFocal(start);
            lowerbound = start.getF();
            upperbound = start.getG() + this->heuristic.getLastPerturbatedCost();

            while (!this->focalList->isOpenEmpty()) {

                //UPDATE FOCAL LIST
                //f_min represents the value with the smaller f value. It is basically the lowerbound
                lowerbound = this->focalList->updateFocalWithOpenChanges(static_cast<cost_t>(lowerbound));

                DO_ON_DEBUG {
                    this->focalList->checkFocalInvariant();
                }

                State& current = this->focalList->peekFromFocal();
                State* currentPtr = &current;
                info("************************* NEW A* STEP #", aStarIteration, "*****************************");
                critical("PEEK FROM FOCAL: ", current, " f=", current.getF(), "g=", current.getG(), "h=", current.getH(), "lowerbound=", lowerbound, "upperbound=", upperbound, "goal cost=", goal != nullptr ? goal->getG() : 0, "open size", this->focalList->getOpenListSize(), "focal size", this->focalList->getFocalListSize());

                DO_ON_DEBUG_IF(goal != nullptr && goal->getG() < upperbound) {
                    throw cpp_utils::exceptions::makeImpossibleException("goal", *goal, "is supposed to always be >= to the upperbound", upperbound);
                }

                //check if g is greater than the upperbound: if we pop a state with g > upperbound, then we can safely put it in closed list, since it cannot be a goal. Since we are in non best-first search, there may happen that in the next iteration we are able to find a better path : in that case we will open the node again
                if (current.getG() > upperbound) {
                    info("put state in closed list since g>upperbound");
                    this->focalList->popFromFocal();
                    current.markAsExpanded();
                    continue;
                }

                //check if the peeked state is actually a goal
                if (this->goalChecker.isGoal(current, expectedGoal)) {
                    //we need to ensure that the goal reached has a shorter path w.r.t the previous goal
                    //otherwise we ignore this goal. We can obtain a goal via a longer path because this is NOT a best first search!
                    
                    //pop from focal
                    this->focalList->popFromFocal();
                    current.markAsExpanded();

                    //we update the goal only if this one improves the cost of the last one
                    if (this->isGoalFoundBetter(static_cast<cost_t>(upperbound), goal, current)) {
                        info("GOAL STATE REACHED ", current);
                        goal = &current;
                        //we revise the upperbound
                        upperbound = goal->getG();
                        //the goal we have found might not be the optimal. we need to keep going
                        this->fireEvent([&current, aStarIteration](Listener& l) { l.onSolutionFound(aStarIteration, current); });
                    }

                    // we have reached the goal. It's impossible to go to the goal using the goal with an alternate route.
                    //keep going
                    continue;
                }

                this->focalList->popFromFocal();
                current.markAsExpanded();

                //ok, this node wasn't a goal. Check the bound. Sometimes the bounds tell us if we need to stop searching
                critical("checking bound: lowerbound >= upperbound ? (", lowerbound, ">=", upperbound, ")");
                if (goal != nullptr) {
                    //we have a solution. since the lb >= ub, we can only fetch symmetric optimal solutions, hence we stop
                    if (static_cast<cost_t>(lowerbound) >= static_cast<cost_t>(upperbound)) {
                        info("we have the optimal solution. ", upperbound, "<=", lowerbound, "! Yeiling optimal solution");
                        goto optimal_solution_found;
                    }
                } else {
                    //the lowerbound is strictly greater than the upperbound, so no solution can possibly be found.
                    if (static_cast<cost_t>(lowerbound) > static_cast<cost_t>(upperbound)) {
                        //we need to prune the state since it's impossible that the state will be able to reduce the upperbound
                        continue;
                    }
                }

                // ok, no optimal solution was found, but at least we might have a lower solution. Check if we have at least a bounded solution
                //current.get() is not the same of lowerbound, since the usage of focal may have chosen something that has not the minimum f!
                if (this->shouldWeEarlyTerminate(current.getF(), static_cast<cost_t>(upperbound), current, goal)) {
                    //return the solution by concatenating the current path from start to current and the cpdpath from current to goal
                    //(considering the perturbations!)
                    info("early terminating from ", current, "up until ", *expectedGoal, ". New solution updated");
                    goal = this->earlyTerminate(current, expectedGoal);

                    //revise upperbound
                    info("revising upperbound to ", goal->getG());
                    upperbound = goal->getG();
                    //we don't skip the successors, since by following the successor of this node we may improve the solution.
                    /* for example in C we may early terminate, but we still keep going on expanding states
                     *
                     * A -> B -> C -> BIG PERTURBATION -> GOAL
                     *           |                          ^
                     *           V                          |
                     *           F  ----------------------> G
                     */
                }

                //current.markAsExpanded();
                this->fireEvent([currentPtr, aStarIteration](Listener& l) {l.onNodeExpanded(aStarIteration, *currentPtr); });

                info("computing successors of state ", current, "...");
                for(auto pair: this->expander.getSuccessors(current, this->supplier)) {
                    State& successor = pair.first;
                    cost_t current_to_successor_cost = pair.second;
                    info("handling successor", successor, "( cost from parent ", current_to_successor_cost, ")");

                    //remember: this is NOT a best first search, so we may need to reopen states (even if the heuristic is consistent)
                    //however, since the heuristic is assumed to be consistent, the lowerbound never decreases
                    if (this->pruner.shouldPrune(successor)) {
                        info("child", successor, "of state ", current, "should be pruned!");
                        //skip neighbours already expanded
                        continue;
                    }

                    if (this->focalList->containsInOpen(successor)) {
                        //state inside the open list. Check if we need to update the path
                        cost_t gval = current.getG() + current_to_successor_cost;
                        if (gval >= successor.getG()) {
                            info("child", successor, "already present in open list but its g was not improved");
                            continue;
                        } 
                        info("child", successor, "of state ", current, "present in open list and its g has been improved from", successor.getG(), "to", gval);

                        cost_t oldSuccessorF = successor.getF();
                        //update successor information
                        successor.setG(gval);
                        successor.setF(this->computeF(gval, successor.getH()));
                        const State* oldParent = successor.getParent();
                        successor.setParent(&current);

                        this->focalList->decreaseOpenListKey(successor);


                        // the previous code was, but it didn't handle well fractional w. if ((oldSuccessorF > this->focalList->getW() * lowerbound) && (successor.getF() <= (this->focalList->getW() * lowerbound)))
                        //should be in focal perform this step: this->w.getDenominator() * n <= this->w.getNumerator() * lowerbound;
                        if (!this->focalList->shouldBeInFocal(oldSuccessorF, static_cast<cost_t>(lowerbound)) && this->focalList->shouldBeInFocal(successor.getF(), static_cast<cost_t>(lowerbound))) {
                            //the successor was not inside the focal list but now with this update it is
                            this->focalList->promoteToFocal(successor);
                        }
                    } else if (successor.isExpanded()) {
                        //state is in closed list
                        cost_t gval = current.getG() + current_to_successor_cost;

                        //check if there is scenario where we can ignore the state
                        // no solution has been found yet
                        if (gval >= successor.getG()) {
                            info("child", successor, "was closed but its g was still better than via ", current);
                            continue;
                        }
                        info("child", successor, "was closed but its g has been improved by going via ", current, "from", successor.getG(), "to", gval);

                        //otherwise reput in openlist
                        //reput in open list
                        successor.setG(gval);
                        successor.setF(this->computeF(gval, successor.getH()));
                        const State* oldParent = successor.getParent();
                        successor.setParent(&current);
                        //we need to add in openlist for sure. But we need to check if we should add it in focal as well
                        // previous check was (successor.getF() <= this->focalList->getW() * bestF)
                        if (this->focalList->shouldBeInFocal(successor.getF(), static_cast<cost_t>(lowerbound))) {
                            //the state should be put in both open and focal
                            this->focalList->pushInOpenAndInFocal(successor);
                        } else {
                            this->focalList->pushInOpen(successor);
                        }
                    } else {
                        State* successorPtr = &successor;
                        //state is not present in open list. Add to it
                        cost_t gval = current.getG() + current_to_successor_cost;
                        this->fireEvent([successorPtr, aStarIteration](Listener& l) {l.onStartingComputingHeuristic(aStarIteration, *successorPtr); });
                        cost_t hval = this->heuristic.getHeuristic(successor, expectedGoal);
                        this->fireEvent([successorPtr, aStarIteration](Listener& l) {l.onEndingComputingHeuristic(aStarIteration, *successorPtr); });

                        //update lastEarliestPerturbationSourceId
                        successor.updateEarlyPerturbationInfo(
                            this->heuristic.getLastEarliestNodeBeforePerturbation(),
                            this->heuristic.getLastEarliestPerturbationSourceIdCost()
                        );

                        successor.setG(gval);
                        successor.setH(hval);
                        successor.setF(this->computeF(gval, hval));
                        successor.setParent(&current);
                        this->fireEvent([successorPtr, aStarIteration](Listener& l) {l.onNodeGenerated(aStarIteration, *successorPtr); });

                        //we may have a new upperbound of the solution
                        if (upperbound > gval + this->heuristic.getLastPerturbatedCost()) {
                            info("updating upperbound. upperbound: from", upperbound, "to", gval + this->heuristic.getLastPerturbatedCost());
                            upperbound = gval + this->heuristic.getLastPerturbatedCost();
                            info("early terminating from ", successor, "up until ", *expectedGoal, ". New solution updated");
                            goal = this->earlyTerminate(current, expectedGoal);
                        }

                        info("child", successor, "of state ", current, "not present in open list. Add it f=", successor.getF(), "g=", successor.getG(), "h=", successor.getH(), "(lowerbound of focal is", lowerbound, ")");
                        // previous check was (successor.getF() <= this->focalList->getW() * bestF)
                        if (this->focalList->shouldBeInFocal(successor.getF(), static_cast<cost_t>(lowerbound))) {
                            //the state should be put in both open and focal
                            this->focalList->pushInOpenAndInFocal(successor);
                        } else {
                            this->focalList->pushInOpen(successor);
                        }
                        
                    }
                }

                aStarIteration += 1;
            }
            //the open list is empty, but we may have found a solution. If we have found one, we simple return
            if (goal == nullptr) {
                info("found no solutions!");
                throw SolutionNotFoundException{};
            }
            

            optimal_solution_found:
            return *goal;

        }
    private:
        /**
         * @brief check if the goal we have just found improve the last goal found or not
         * 
         * @param previousGoal the previous goal found. if nullptr no goal waspreviously found
         * @param currentGoal the goal we have just reached
         * @return true if @c currentGoal is better than @c previousGoal
         * @return false otherwise
         */
        bool isGoalFoundBetter(cost_t upperbound, const State* previousGoal, const State& currentGoal) const {
            if (previousGoal != nullptr) {
                return currentGoal.getG() < upperbound;
            } else {
                return true;
            }
        }
        bool shouldWeEarlyTerminate(cost_t lowerbound, cost_t upperbound, const State& current, const State* goal) const {
            /*
                * in focal:
                * f_1(n) <= w * f_1min (w > 1)
                * 
                * so here the current may have f(n) >= f_1min && f <= w f_1min
                * so the maximum lowerbound is w f_1min
                * 
                * upperbound/(w f_1min) <= epsilon
                * 
                * to preserve the ratio we need to multiply upperbound with w as well
                * 
                * w * upperbound/lowerbound <= epsilon
                * 
                * nw/dw * upperbound/lowerbound <= nepsilon/depsilon
                * nw * upperbound * depsilon <=  dw * nepsilon * lowerbound
                * 
                * dw * nepsilon * lowerbound >= nw upperbound * depsilon
                * 
                * We need to compute several solutions. In the first one the goal has not been found yet, so to ensure that we compute a bounded suboptimal solution
                * we should have >=. Afterwards we don't care about solutions with the same bound, but we want solutions that improve the cost. So we add such check.
                */
            if (goal != nullptr) {
                //we need to ensure that following this route will actually improve the cost.
                auto newSolutionCost = current.getG() + this->heuristic.getCPDPathPerturbatedWeights(current.getId());
                info("checking if current solution with cost ", upperbound, "is worse than the new solution of cost", newSolutionCost, "(current.g = ", current.getG(), ", perturbated path cost", this->heuristic.getCPDPathPerturbatedWeights(current.getId()), ")");
                if (newSolutionCost < upperbound ) {
                    auto result = ((this->focalList->getW().getNumerator() * this->epsilon.getNumerator() * lowerbound) >= (this->focalList->getW().getDenominator() * this->epsilon.getDenominator() * upperbound));
                    if (result) {
                        info("it is true that", this->focalList->getW().getNumerator(), "*", this->epsilon.getNumerator(), "*", lowerbound, ">=", this->focalList->getW().getDenominator(), "*", this->epsilon.getDenominator(), "*", upperbound, "(w_n * epsilon_n * lowerbound >= w_d * epsilon_d * upperbound)");
                    }
                    return result;
                }
                return false;
            } else {
                auto result = ((this->focalList->getW().getNumerator() * this->epsilon.getNumerator() * lowerbound) >= (this->focalList->getW().getDenominator() * this->epsilon.getDenominator() * upperbound));
                if (result) {
                    info("it is true that", this->focalList->getW().getNumerator(), "*", this->epsilon.getNumerator(), "*", lowerbound, ">=", this->focalList->getW().getDenominator(), "*", this->epsilon.getDenominator(), "*", upperbound, "(w_n * epsilon_n * lowerbound >= w_d * epsilon_d * upperbound)");
                }
                return result;
            }
        }
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
                    successor.setReason(cpd_search_generated_e::CPDPATH);
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
    class CpdFocalOptimalSearchFactory {
    public:
        template <typename G, typename V>
        struct output_t {
            public:
                /**
                 * @brief place where the goal checker is located in memory
                 * 
                 */
                StandardLocationGoalChecker<GraphFocalState<G, V, PerturbatedCost>> goalChecker;
                /**
                 * @brief place where the state supplier is located in memory
                 * 
                 */
                CpdFocalSupplier<G, V, PerturbatedCost> stateSupplier;
                /**
                 * @brief place where the state expander is located in memory
                 * 
                 */
                CpdFocalExpander<G, V> stateExpander;
                /**
                 * @brief place where the state pruner is located in memory
                 * 
                 */
                NeverPrune<GraphFocalState<G, V, PerturbatedCost>> statePruner;
                /**
                 * @brief place where the heuristic is located
                 * 
                 * @note this needs to be the last field declared here, since its dependent on other fields.
                 * @see https://wiki.sei.cmu.edu/confluence/display/cplusplus/OOP53-CPP.+Write+constructor+member+initializers+in+the+canonical+order
                 */
                CpdFocalHeuristic<GraphFocalState<G, V, PerturbatedCost>, G, V> heuristic;
                /**
                 * @brief place where CPD timed search algorithm is located
                 * 
                 * @note this needs to be the last field declared here, since its dependent on other fields.
                 * @see https://wiki.sei.cmu.edu/confluence/display/cplusplus/OOP53-CPP.+Write+constructor+member+initializers+in+the+canonical+order
                 */
                CpdFocalOptimalSearch<G, V> search;
            public:
                output_t(
                    const CpdManager<G, V>& cpdManager,
                    const IImmutableGraph<G, V, PerturbatedCost>& perturbatedGraph,
                    const fractional_number<cost_t>& focalListW,
                    const fractional_number<cost_t>& epsilon
                    ): 
                    heuristic{cpdManager, perturbatedGraph},
                    goalChecker{}, 
                    stateSupplier{perturbatedGraph}, 
                    stateExpander{perturbatedGraph}, 
                    statePruner{},
                    search{this->heuristic, this->goalChecker, this->stateSupplier, this->stateExpander, this->statePruner, this->heuristic.getCpdManager(), focalListW, epsilon}
                     {
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
        output_t<G,V>* get(const CpdManager<G,V>& cpdManager, const IImmutableGraph<G, V, PerturbatedCost>& perturbatedGraph, const fractional_number<cost_t>& focalListW, const fractional_number<cost_t>& epsilon) {
            return new output_t<G, V>{
                cpdManager,
                perturbatedGraph,
                focalListW,
                epsilon
            };
        }
    };

}

#endif
