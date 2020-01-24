#ifndef _CPD_SEARCH_COUNTCPDSEARCHLISTENER_HEADER__
#define _CPD_SEARCH_COUNTCPDSEARCHLISTENER_HEADER__

#include <cpp-utils/NumTracker.hpp>
#include <cpp-utils/Timer.hpp>

#include <pathfinding-utils/StateCounter.hpp>

#include "CpdSearch.hpp"


namespace pathfinding::search::listeners {

    template <typename G, typename V, typename STATE>
    class CountCpdSearchListener: public CpdSearchListener<G,V, STATE>, public StateCounter {
        using This = CountCpdSearchListener<G, V, STATE>;
        using Super1 = CpdSearchListener<G, V, STATE>;
        using Super2 = StateCounter;
    public:
        CountCpdSearchListener() = default;
        virtual ~CountCpdSearchListener() = default;
        CountCpdSearchListener(const This& o) = default;
        CountCpdSearchListener(This&& o) = default;
        This& operator=(const This& o) = default;
        This& operator=(This&& o) = default;
    public:
        void onNodeExpanded(int iteration, const GraphState<G, V, PerturbatedCost>& s) {
            this->updateNodeExpanded();
        }
        void onNodeGenerated(int iteration, const GraphState<G, V, PerturbatedCost>& s) {
            this->updateNodeGenerated();
        }
        void onStartingComputingHeuristic(int iteration, const GraphState<G, V, PerturbatedCost>& s) {
            this->startHeuristicTimer();
        }
        void onEndingComputingHeuristic(int iteration, const GraphState<G, V, PerturbatedCost>& s) {
            this->stopHeuristicTimer();
        }
        void onSolutionFound(int iteration, const STATE& goal) {

        }
        virtual void onEarlyTerminationActivated(int iteration, const STATE& from, const STATE& goal) {

        }
        virtual void onUpperboundRevised(int iteration, const STATE& from, cost_t oldUpperbound, cost_t newUpperbound) {

        }
    public:
        void cleanup() {
            Super2::cleanup();
        }
    };
}

#endif 