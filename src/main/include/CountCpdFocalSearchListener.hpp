#ifndef _CPD_FOCAL_SEARCH_COUNTCPDFOCALSEARCHLISTENER_HEADER__
#define _CPD_FOCAL_SEARCH_COUNTCPDFOCALSEARCHLISTENER_HEADER__

#include <cpp-utils/NumTracker.hpp>
#include <cpp-utils/Timer.hpp>

#include <cpd-search/CpdFocalSearch.hpp>


namespace pathfinding::search {

    template <typename G, typename V>
    class CountCpdFocalSearchListener: public CpdFocalSearchListener<G,V> {
        typedef CountCpdFocalSearchListener<G,V> This;
        typedef CpdFocalSearchListener<G,V> Super;
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
        CountCpdFocalSearchListener(): nodeExpanded{0}, nodeGenerated{0}, heuristicTime{}, heuristicTimer{false} {

        }
        virtual ~CountCpdFocalSearchListener() {

        }
        CountCpdFocalSearchListener(const This& o): nodeExpanded{o.nodeExpanded}, nodeGenerated{o.nodeGenerated}, heuristicTime{o.heuristicTime}, heuristicTimer{o.heuristicTimer} {

        }
        CountCpdFocalSearchListener(This&& o): nodeExpanded{o.nodeExpanded}, nodeGenerated{o.nodeGenerated}, heuristicTime{o.heuristicTime}, heuristicTimer{o.heuristicTimer} {

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
        void onNodeExpanded(const GraphFocalState<G, V, PerturbatedCost>& s) {
            this->nodeExpanded += 1;
        }
        void onNodeGenerated(const GraphFocalState<G, V, PerturbatedCost>& s) {
            this->nodeGenerated += 1;
        }
        void onStartingComputingHeuristic(const GraphFocalState<G, V, PerturbatedCost>& s) {
            this->heuristicTimer.cleanup();
            this->heuristicTimer.start();
        }
        void onEndingComputingHeuristic(const GraphFocalState<G, V, PerturbatedCost>& s) {
            this->heuristicTimer.stop();
            this->heuristicTime.update(this->heuristicTimer.getElapsedMicroSeconds().toLong());
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