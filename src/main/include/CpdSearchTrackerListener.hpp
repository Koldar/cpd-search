#ifndef _PATHFINDINGUTILS_CPDSEARCHTRACKERLISTENER_HEADER__
#define _PATHFINDINGUTILS_CPDSEARCHTRACKERLISTENER_HEADER__

#include <cpp-utils/AbstractEnum.hpp>

#include <pathfinding-utils/AStarTrackerListener.hpp>

namespace pathfinding::search {

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
    class CpdSearchTrackerListener: public CpdSearchListener<G, V, STATE>, public AstarTrackerListener<G, V, E, STATE> {
    public:
        typedef CpdSearchTrackerListener<G, V, E, STATE> This;
        typedef CpdSearchListener<G, V, STATE> Super1;
        typedef AstarTrackerListener<G, V, E, STATE> Super2;
    public:
        CpdSearchTrackerListener(const IImmutableGraph<G,V,E>& originalGraph): Super1{}, Super2{originalGraph} {
        }
        virtual ~CpdSearchTrackerListener() = default;
        CpdSearchTrackerListener(const This& o) = default;
        CpdSearchTrackerListener(This&& o) = default;
        This& operator =(const This& o) = default;
        This& operator = (This&& o) = default;
    public:
        virtual void cleanup() {
            Super2::cleanup();
        }
        virtual void onNodeExpanded(const STATE& node) {
            Super2::onNodeExpanded(node);
        }
        virtual void onNodeGenerated(const STATE& node) {
            Super2::onNodeGenerated(node);
        }
        virtual void onStartingComputingHeuristic(const STATE& s) {
            Super2::onStartingComputingHeuristic(s);
        }
        virtual void onEndingComputingHeuristic(const STATE& s) {
            Super2::onEndingComputingHeuristic(s);
        }
        virtual void onSolutionFound(const STATE& goal) {
            Super2::onSolutionFound(goal);
        }

        virtual void onEarlyTerminationActivated(const STATE& from, const STATE& goal) {
            
        }
        virtual void onUpperboundRevised(const STATE& from, cost_t oldUpperbound, cost_t newUpperbound) {

        }
    };

}

#endif