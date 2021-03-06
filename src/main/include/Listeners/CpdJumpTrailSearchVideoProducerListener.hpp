#ifndef _CPDSEARCH_CPDJUMPTRAILSEARCHVIDEOPRODUCERLISTENER_HEADER__
#define _CPDSEARCH_CPDJUMPTRAILSEARCHVIDEOPRODUCERLISTENER_HEADER__

#include <pathfinding-utils/StateVideoProducer.hpp>

#include "CpdJumpSearchImageProducerListener.hpp"
#include "CpdJumpTrailSearchListener.hpp"


namespace pathfinding::search::listeners {

    using namespace cpp_utils;

    /**
     * @brief a listener that produces a video as soon as a solution is found by Cpd Jump Search Algorithm
     * 
     * @tparam G 
     * @tparam V 
     * @tparam EORIGINAL 
     * @tparam EPERTURBATED 
     * @tparam STATE 
     * @tparam MAPTYPE 
     * @tparam IMAGETYPE 
     */
    template <typename G, typename V, typename EORIGINAL, typename EPERTURBATED, typename STATE, typename MAPTYPE, typename IMAGETYPE>
    class CpdJumpTrailSearchVideoProducerListener: public CpdJumpTrailSearchListener<G, V, STATE>, public StateVideoProducer<G, V, EORIGINAL, EPERTURBATED, MAPTYPE, IMAGETYPE> {
    public:
        using This = CpdJumpTrailSearchVideoProducerListener<G, V, EORIGINAL, EPERTURBATED, STATE, MAPTYPE, IMAGETYPE>;
        using Super1 = CpdJumpTrailSearchListener<G, V, STATE>;
        using Super2 = StateVideoProducer<G, V, EORIGINAL, EPERTURBATED, MAPTYPE, IMAGETYPE>;
    public:
        CpdJumpTrailSearchVideoProducerListener(const IImmutableGraph<G,V,EORIGINAL>& originalGraph, const IImmutableGraph<G, V, EPERTURBATED>& perturbatedGraph, const MAPTYPE& map, const function_t<EPERTURBATED,cost_t>& costFunction): Super2{originalGraph, perturbatedGraph, map, costFunction} {
        }
        virtual ~CpdJumpTrailSearchVideoProducerListener() {

        }
        CpdJumpTrailSearchVideoProducerListener(const This& o) = default;
        CpdJumpTrailSearchVideoProducerListener(This&& o) = default;
        This& operator = (const This& o) = default;
        This& operator = (This&& o) = default;
    public:
        virtual void onEarlyTerminationActivated(int iteration, const STATE& from, const STATE& goal) {

        }

        virtual void onUpperboundRevised(int iteration, const STATE& from, cost_t oldUpperbound, cost_t newUpperbound) {

        }

        virtual void onNodeExpanded(int iteration, const STATE& node) {
            this->updateNodeExpanded(node);
            boost::filesystem::path p{scout("./cpdjumptrailsearch_videoframe_iteration_", iteration, ".jpeg")};
            this->addImage(p);
        }
        
        virtual void onNodeGenerated(int iteration, const STATE& node) {
            this->updateNodeGenerated(node);
        }

        virtual void onStartingComputingHeuristic(int iteration, const STATE& s) {
        }

        virtual void onEndingComputingHeuristic(int iteration, const STATE& s) {

        }
        
        virtual void onSolutionFound(int iteration, const STATE& goal) {
            static function_t<STATE, nodeid_t> mapper = [&](const STATE& s) { return s.getPosition();};
            critical("creating video!");
            this->updateSolution(goal, mapper);
            this->buildVideo(scout("./cpdjumptrailsearch_behaviour_", iteration, ".mp4"), "./test.wav");
        }
    public:
        virtual void cleanup() {
            Super2::cleanup();
        }
    };

}

#endif