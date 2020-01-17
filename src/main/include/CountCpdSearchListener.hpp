#ifndef _CPD_SEARCH_COUNTCPDSEARCHLISTENER_HEADER__
#define _CPD_SEARCH_COUNTCPDSEARCHLISTENER_HEADER__

#include <cpp-utils/NumTracker.hpp>
#include <cpp-utils/Timer.hpp>

#include "CpdSearch.hpp"


namespace pathfinding::search {

    template <typename G, typename V, typename STATE>
    class CountCpdSearchListener: public CpdSearchListener<G,V, STATE> {
        typedef CountCpdSearchListener<G, V, STATE> This;
        typedef CpdSearchListener<G, V, STATE> Super;
    private:
        int nodeExpanded;
        int nodeGenerated;
        /**
         * @brief times used by the algorithm to compute the heuristic
         * 
         */
        NumTracker<long> heuristicTime;
        Timer heuristicTimer;
    public:
        CountCpdSearchListener(): Super{}, nodeExpanded{0}, nodeGenerated{0}, heuristicTime{}, heuristicTimer{false} {

        }
        CountCpdSearchListener(const Super& o): Super{o}, nodeExpanded{0}, nodeGenerated{0}, heuristicTime{}, heuristicTimer{false} {
        }
        virtual ~CountCpdSearchListener() {

        }
        CountCpdSearchListener(const This& o): nodeExpanded{o.nodeExpanded}, nodeGenerated{o.nodeGenerated}, heuristicTime{o.heuristicTime}, heuristicTimer{o.heuristicTimer} {

        }
        CountCpdSearchListener(This&& o): nodeExpanded{o.nodeExpanded}, nodeGenerated{o.nodeGenerated}, heuristicTime{o.heuristicTime}, heuristicTimer{o.heuristicTimer} {

        }
        This& operator=(const This& o) {
            this->nodeExpanded = o.nodeExpanded;
            this->nodeGenerated = o.nodeGenerated;
            this->heuristicTime = o.heuristicTime;
            this->heuristicTimer = o.heuristicTimer;

            return *this;
        }
        This& operator=(This&& o) {
            this->nodeExpanded = o.nodeExpanded;
            this->nodeGenerated = o.nodeGenerated;
            this->heuristicTime = o.heuristicTime;
            this->heuristicTimer = o.heuristicTimer;
            
            return *this;
        }
    public:
        int getNodeExpanded() const {
            return this->nodeExpanded;
        }
        int getNodeGenerated() const {
            return this->nodeGenerated;
        }
        NumTracker<long> getHeuristicTracking() const {
            return this->heuristicTime;
        }
    public:
        void onNodeExpanded(const GraphState<G, V, PerturbatedCost>& s) {
            this->nodeExpanded += 1;
        }
        void onNodeGenerated(const GraphState<G, V, PerturbatedCost>& s) {
            this->nodeGenerated += 1;
        }
        void onStartingComputingHeuristic(const GraphState<G, V, PerturbatedCost>& s) {
            this->heuristicTimer.cleanup();
            this->heuristicTimer.start();
        }
        void onEndingComputingHeuristic(const GraphState<G, V, PerturbatedCost>& s) {
            this->heuristicTimer.stop();
            this->heuristicTime.update(this->heuristicTimer.getElapsedMicroSeconds().toLong());
        }
        void onSolutionFound(const STATE& goal) {

        }
        virtual void onEarlyTerminationActivated(const STATE& from, const STATE& goal) {

        }
        virtual void onUpperboundRevised(const STATE& from, cost_t oldUpperbound, cost_t newUpperbound) {

        }
    public:
        void cleanup() {
            this->nodeExpanded = 0;
            this->nodeGenerated = 0;
            this->heuristicTime.cleanup();
            this->heuristicTimer.cleanup();
        }
    };
}

#endif 