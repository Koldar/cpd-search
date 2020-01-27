#ifndef _CPDSEARCH_COUNTCPDJUMPSEARCHLISTENER_HEADER__
#define _CPDSEARCH_COUNTCPDJUMPSEARCHLISTENER_HEADER__

#include <cpp-utils/NumTracker.hpp>
#include <cpp-utils/Timer.hpp>

#include <pathfinding-utils/StateCounter.hpp>

#include "CpdJumpSearch.hpp"


namespace pathfinding::search::listeners {

    template <typename G, typename V, typename STATE>
    class CountCpdJumpSearchListener: public CpdJumpSearchListener<G,V, STATE>, public StateCounter {
        using This = CountCpdJumpSearchListener<G, V, STATE>;
        using Super1 = CpdJumpSearchListener<G, V, STATE>;
        using Super2 = StateCounter;
    public:
        CountCpdJumpSearchListener() = default;
        virtual ~CountCpdJumpSearchListener() = default;
        CountCpdJumpSearchListener(const This& o) = default;
        CountCpdJumpSearchListener(This&& o) = default;
        This& operator=(const This& o) = default;
        This& operator=(This&& o) = default;
    public:
        void onNodeExpanded(int iteration, const STATE& s) {
            this->updateNodeExpanded();
        }
        void onNodeGenerated(int iteration, const STATE& s) {
            this->updateNodeGenerated();
        }
        void onStartingComputingHeuristic(int iteration, const STATE& s) {
            this->startHeuristicTimer();
        }
        void onEndingComputingHeuristic(int iteration, const STATE& s) {
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