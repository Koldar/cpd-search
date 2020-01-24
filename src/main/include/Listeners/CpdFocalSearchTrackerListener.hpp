#ifndef _PATHFINDINGUTILS_CPDFOCALSEARCHTRACKERLISTENER_HEADER__
#define _PATHFINDINGUTILS_CPDFOCALSEARCHTRACKERLISTENER_HEADER__

#include <cpp-utils/AbstractEnum.hpp>

#include <pathfinding-utils/StateTracker.hpp>
#include <pathfinding-utils/StateCounter.hpp>

#include "CpdFocalSearchListener.hpp"

namespace pathfinding::search::listeners {

    using namespace cpp_utils;

    /**
     * @brief a listener that keeps track of the state we have expanded, and the optimal path generated
     * 
     * the listener allows you to keep track of heuristic average timings, set of node
     * expanded and the first solution an algorithm has found
     * 
     * @tparam STATE type of the A* state
     * @see AstarTrackerListener
     */
    template <typename G, typename V, typename E, typename STATE>
    class CpdFocalSearchTrackerListener: public CpdFocalSearchListener<G, V, STATE>, public StateTracker<G, V, E>, public StateCounter {
    public:
        using This = CpdFocalSearchTrackerListener<G, V, E, STATE>;
        using Super1 = CpdFocalSearchListener<G, V, STATE>;
        using Super2 = StateTracker<G, V, E>;
        using Super3 = StateCounter;

    public:
        CpdFocalSearchTrackerListener(const IImmutableGraph<G,V,E>& perturbatedGraph): Super1{}, Super2{perturbatedGraph}, Super3{} {
        }
        virtual ~CpdFocalSearchTrackerListener() {
            
        }
        CpdFocalSearchTrackerListener(const This& o) = default;
        CpdFocalSearchTrackerListener(This&& o) = default;
        This& operator =(const This& o) = default;
        This& operator = (This&& o) = default;
    public:
        virtual void cleanup() {
            Super2::cleanup();
            Super3::cleanup();
        }
        virtual void onNodeExpanded(int iteration, const STATE& node) {
            Super2::updateNodeExpanded(node);
            Super3::updateNodeExpanded();
        }
        virtual void onNodeGenerated(int iteration, const STATE& node) {
            Super2::updateNodeGenerated(node);
            Super3::updateNodeGenerated();
        }
        virtual void onStartingComputingHeuristic(int iteration, const STATE& s) {
            Super3::startHeuristicTimer();
        }
        virtual void onEndingComputingHeuristic(int iteration, const STATE& s) {
            Super3::stopHeuristicTimer();
        }
        virtual void onSolutionFound(int iteration, const STATE& goal) {
            static function_t<STATE, nodeid_t> mapper = [&](const STATE& s) { return s.getPosition();};
            Super2::updateSolution(goal, mapper);
        }

        virtual void onEarlyTerminationActivated(int iteration, const STATE& from, const STATE& goal) {
            
        }

        virtual void onUpperboundRevised(int iteration, const STATE& from, cost_t oldUpperbound, cost_t newUpperbound) {
            critical("upperbound revised from", oldUpperbound, "to", newUpperbound);
        }
    };

}

#endif