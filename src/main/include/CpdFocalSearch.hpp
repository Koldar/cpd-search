#ifndef _CPD_SEARCH_CPD_FOCAL_SEARCH_HEADER__
#define _CPD_SEARCH_CPD_FOCAL_SEARCH_HEADER__

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

    using namespace cpp_utils;
    using namespace pathfinding;
    using namespace compressed_path_database;
    using namespace pathfinding::data_structures;

    namespace internal {

        template <typename G, typename V>
        struct OpenListOrderer {
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
        struct FocalListOrderer {
            bool operator ()(const GraphFocalState<G, V, PerturbatedCost>& a, const GraphFocalState<G, V, PerturbatedCost>& b) {
                //f are the same. Tie breaking by sleectnig the state with bigger g
                return a.getG() > b.getG();
            }
        };

        template <typename G, typename V>
        struct GraphFocalStateGetCost {
            cost_t operator()(const GraphFocalState<G, V, PerturbatedCost>& state) {
                return state.getF();
            }
        };

    }

    /**
     * @brief CPD Focal search
     * 
     * 
     * Like CPD Search but with some optimizations.
     * 
     * Successors:
     * As node successors we put, aside the usual successors, 
     * the nodes along the cpd path towards the goal just before the earliest perturbation (or the target itself if no
     * perturbation are present).
     * 
     * Focal list:
     * We have an open list, a focal list and a suboptimality bound w (like WA*)
     * the 2 lists have 2 f functions used to sort the states within them.
     * f_1: sorts states in open
     * f_2: sorts states in focal
     * 
     * f_1min: the state in open with the least f value (ie., peek);
     * Focal contains all the states in open which satisfies the condition:
     * f_1(n) <= w * f_1min (w > 1)
     * 
     * criterion in open list: as in CPD search: h is the cpd path of n to t target using the original weights
     * criterion in focal list: choose the state with highest g value
     * 
     * 
     * @tparam GraphState<G, V, cost_t> 
     * @tparam G 
     * @tparam V 
     * @tparam OTHER 
     */
    template <typename G, typename V>
    class CpdFocalSearch: public IMemorable, public ISearchAlgorithm<GraphFocalState<G, V, PerturbatedCost>, const GraphFocalState<G, V, PerturbatedCost>*, const GraphFocalState<G, V, PerturbatedCost>&>, public ISingleListenable<listeners::CpdFocalSearchListener<G, V, GraphFocalState<G, V, PerturbatedCost>>> {
        using GraphStateReal = GraphFocalState<G, V, PerturbatedCost>;
        using This = CpdFocalSearch<G, V>;
        using Listener = listeners::CpdFocalSearchListener<G, V, GraphStateReal>;
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

        CpdFocalHeuristic<GraphStateReal, G, V>& heuristic;
        IGoalChecker<GraphStateReal>& goalChecker;
        CpdFocalExpander<G, V>& expander;
        IStateSupplier<GraphStateReal, nodeid_t, generation_enum_t>& supplier;
        IStatePruner<GraphStateReal>& pruner;

        FocalList<GraphStateReal, internal::OpenListOrderer<G,V>, internal::FocalListOrderer<G,V>, internal::GraphFocalStateGetCost<G,V>, cost_t>* focalList;
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
        CpdFocalSearch(CpdFocalHeuristic<GraphStateReal, G, V>& heuristic, IGoalChecker<GraphStateReal>& goalChecker, IStateSupplier<GraphStateReal, nodeid_t, generation_enum_t>& supplier, CpdFocalExpander<G, V>& expander, IStatePruner<GraphStateReal>& pruner, const CpdManager<G,V>& cpdManager, const fractional_number<cost_t>& focalListWeight, const fractional_number<cost_t>& epsilon) : Super2{},
            heuristic{heuristic}, goalChecker{goalChecker}, supplier{supplier}, expander{expander}, pruner{pruner},
            epsilon{epsilon}, cpdManager{cpdManager}
            {

            this->focalList = new FocalList<GraphStateReal, internal::OpenListOrderer<G,V>, internal::FocalListOrderer<G,V>, internal::GraphFocalStateGetCost<G,V>, cost_t>{focalListWeight};
            if (!heuristic.isConsistent()) {
                throw cpp_utils::exceptions::InvalidArgumentException{"the heuristic is not consistent!"};
            }
        }

        virtual ~CpdFocalSearch() {
            this->tearDownSearch();
            delete this->focalList;
        }
        //the class cannot be copied whatsoever
        CpdFocalSearch(const This& other) = delete;
        This& operator=(const CpdFocalSearch& other) = delete;

        CpdFocalSearch(This&& other): Super2{other}, heuristic{other.heuristic}, goalChecker{other.goalChecker}, supplier{other.supplier}, expander{other.expander}, pruner{other.pruner}, epsilon{other.epsilon}, cpdManager{other.cpdManager}, focalList{::std::move(other.focalList)} {
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
        virtual void setupSearch(const GraphStateReal* start, const GraphStateReal* goal) {
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
         * @return vectorplus<const GraphStateReal*> list of possibly new state generated going from @c from to @c to
         */
        virtual vectorplus<const GraphStateReal*> connectStates(const GraphStateReal* from, const GraphStateReal* to) const {
            vectorplus<const GraphStateReal*> result{};
            const GraphStateReal* tmp = from;
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
                GraphStateReal* tmp2 = &this->supplier.getState(nextNode, generation_enum_t::FROM_SOLUTION);
                tmp2->setParent(const_cast<GraphStateReal*>(tmp));
                if (tmp2->getPosition() == to->getPosition()) {
                    goto exit;
                }
                
                result.add(tmp2);
                tmp = tmp2;
            }

            exit:;
            return result;
        }
        virtual std::unique_ptr<ISolutionPath<const GraphStateReal*, const GraphStateReal&>> buildSolutionFromGoalFetched(const GraphStateReal& start, const GraphStateReal& actualGoal, const GraphStateReal* goal) {
            auto result = new StateSolutionPath<GraphStateReal>{};

            if (actualGoal == start) {
                result->add(&start);
            } else {
                const GraphStateReal* cpdPathEnd = &actualGoal;
                const GraphStateReal* cpdPathStart = actualGoal.getParent();
                
                while (cpdPathStart != nullptr) {
                    info("need to connect path from", *cpdPathStart, " to ", *cpdPathEnd);
                    //we need to rebuild the path, since the search has jumped from a loation to another one via the CPD.
                    //specifically, we need to connect "tmp" with the head of the building solution, with the CPD until the CPD finds such head
                    debug("adding", *cpdPathEnd, "in solution");
                    result->addHead(cpdPathEnd);
                    vectorplus<const GraphStateReal*> connect = this->connectStates(cpdPathStart, cpdPathEnd);
                    for (auto s : connect.reverse()) {
                        debug("adding ", s, "in solution");
                        result->addHead(s);
                    }

                    cpdPathEnd = cpdPathStart;
                    cpdPathStart = cpdPathStart->getParent();
                }
                //add the start
                result->addHead(cpdPathEnd);
            }

            for (auto s: *result) {
                info("s is", *s, "and g is", s->getG());
            }
            
            return std::unique_ptr<StateSolutionPath<GraphStateReal>>{result};
        }
        virtual cost_t getSolutionCostFromGoalFetched(const GraphStateReal& start, const GraphStateReal& actualGoal, const GraphStateReal* goal) const {
            return actualGoal.getCost();
        }
        virtual const GraphStateReal& performSearch(GraphStateReal& start, const GraphStateReal* expectedGoal) {
            info("************************* NEW SEARCH *****************************");
            if (expectedGoal != nullptr) {
                info("starting A*! start = ", start, "goal = ", *expectedGoal);
            } else {
                info("starting A*! start = ", start, "goal = ", "none");
            }
            
            int aStarIteration = 0;
            const GraphStateReal* goal = nullptr;
            MDValue<cost_t> upperbound = cost_t::INFTY;
            upperbound.setListener(LogNumberListener<cost_t>{"upperbound", 8});
            /*
             * the smallest f-value in the open list. this value tends to get bigger and bigger
             */
            MCValue<cost_t> lowerbound = cost_t{0};
            lowerbound.setListener(LogNumberListener<cost_t>{"lowerbound", 8});
            const GraphStateReal* earlyTerminationState = nullptr;

            GraphStateReal* startPtr = &start;
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

                GraphStateReal& current = this->focalList->peekFromFocal();
                GraphStateReal* currentPtr = &current;
                info("************************* NEW A* STEP #", aStarIteration, "*****************************");
                info("PEEK FROM OPEN: ", current, " f=", current.getF(), "g=", current.getG(), "h=", current.getH(), "bestF=", lowerbound);

                //check if the peeked state is actually a goal
                if (this->goalChecker.isGoal(current, expectedGoal)) {
                    info("GOAL STATE REACHED ", current);
                    goal = &current;
                    //the goal we have found might not be the optimal. we need to keep going
                    this->fireEvent([&current, aStarIteration](Listener& l) { l.onSolutionFound(aStarIteration, current); });

                    goto goal_found;
                }

                this->focalList->popFromFocal();

                //the lowerbound is strictly greater than the upperbound, so no solution can possibly be found.
                if (static_cast<cost_t>(lowerbound) > static_cast<cost_t>(upperbound)) {
                    //we need to prune the state since it's impossible that the state will be able to reduce the upperbound
                    current.markAsExpanded();
                    continue;
                }

                //ok, we're still in the allowed bound, so a solution can still be found
                if (this->shouldWeEarlyTerminate(current.getF(), static_cast<cost_t>(upperbound), current, goal)) {
                    //return the solution by concatenating the current path from start to current and the cpdpath from current to goal
                    //(considering the perturbations!)
                    info("early terminating from ", current, "up until ", *expectedGoal, ". New solution updated");
                    goal = this->earlyTerminate(current, expectedGoal);
                    goto goal_found;
                }

                current.markAsExpanded();
                this->fireEvent([currentPtr, aStarIteration](Listener& l) {l.onNodeExpanded(aStarIteration, *currentPtr); });

                info("computing successors of state ", current, "...");
                for(auto pair: this->expander.getSuccessors(current, this->supplier)) {
                    GraphStateReal& successor = pair.first;
                    cost_t current_to_successor_cost = pair.second;
                    info("handling successor", successor, "( cost from parent ", current_to_successor_cost, ")");

                    if (this->pruner.shouldPrune(successor)) {
                        info("child", successor, "of state ", current, "should be pruned!");
                        //skip neighbours already expanded
                        continue;
                    }

                    if (this->focalList->containsInOpen(successor)) {
                        //state inside the open list. Check if we need to update the path
                        cost_t gval = current.getG() + current_to_successor_cost;
                        info("child", successor, "already present in open list. Check if its g is improved...");
                        if (gval < successor.getG()) {
                            info("child", successor, "of state ", current, "present in open list and has a lower g. update its parent!");

                            cost_t oldSuccessorF = successor.getF();
                            //update successor information
                            successor.setG(gval);
                            successor.setF(this->computeF(gval, successor.getH()));
                            const GraphStateReal* oldParent = successor.getParent();
                            successor.setParent(&current);

                            this->focalList->decreaseOpenListKey(successor);


                            // the previous code was, but it didn't handle well fractional w. if ((oldSuccessorF > this->focalList->getW() * lowerbound) && (successor.getF() <= (this->focalList->getW() * lowerbound)))
                            //should be in focal perform this step: this->w.getDenominator() * n <= this->w.getNumerator() * lowerbound;
                            if (!this->focalList->shouldBeInFocal(oldSuccessorF, static_cast<cost_t>(lowerbound)) && this->focalList->shouldBeInFocal(successor.getF(), static_cast<cost_t>(lowerbound))) {
                                //the successor was not inside the focal list but now with this update it is
                                this->focalList->promoteToFocal(successor);
                            }
                        }
                    } else if (successor.isExpanded()) {
                        //state is in closed list
                        cost_t gval = current.getG() + current_to_successor_cost;

                        //check if there is scenario where we can ignore the state
                        // no solution has been found yet
                        if (gval >= successor.getG()) {
                            continue;
                        }

                        //otherwise reput in openlist
                        //reput in open list
                        successor.setG(gval);
                        successor.setF(this->computeF(gval, successor.getH()));
                        const GraphStateReal* oldParent = successor.getParent();
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
                        GraphStateReal* successorPtr = &successor;
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
                            if (earlyTerminationState == nullptr) {
                                info("updating incumbent. incumbent: from null to", successor);
                            } else {
                                info("updating incumbent. incumbent: from", *earlyTerminationState, "to", successor);
                            }
                            
                            upperbound = gval + this->heuristic.getLastPerturbatedCost();
                            //TODO i think the early terminate state shouldn't be saved at all: in focal list this state is not  necessary the one which is pop from focal! (in cpd search it was)
                            //what we need is not the actual state, but the upperbound revision
                            earlyTerminationState = &successor;
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
            info("found no solutions!");
            throw SolutionNotFoundException{};

            goal_found:
            return *goal;

        }
    private:
        bool shouldWeEarlyTerminate(cost_t lowerbound, cost_t upperbound, const GraphStateReal& current, const GraphStateReal* goal) const {
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
                * We need to compute one bounded solution. To ensure that we compute a bounded suboptimal solution
                * we should have >=
                */
                auto result = ((this->focalList->getW().getNumerator() * this->epsilon.getNumerator() * lowerbound) >= (this->focalList->getW().getDenominator() * this->epsilon.getDenominator() * upperbound));
                if (result) {
                    info("it is true that", this->focalList->getW().getNumerator(), "*", this->epsilon.getNumerator(), "*", lowerbound, ">=", this->focalList->getW().getDenominator(), "*", this->epsilon.getDenominator(), "*", upperbound, "(w_n * epsilon_n * lowerbound >= w_d * epsilon_d * upperbound)");
                }
                return result;
        }
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
                    successor.setSource(generation_enum_t::FROM_EARLY_TERMINATION);
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
    class CpdFocalSearchFactory {
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
                CpdFocalSearch<G, V> search;
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
        output_t<G,V>* get(const CpdManager<G,V>& cpdManager, const IImmutableGraph<G, V, PerturbatedCost>& perturbatedGraph, const fractional_cost& focalListW, const fractional_cost& epsilon) {
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
