#ifndef _CPDSEARCH_CPDJUMPSEARCHVIDEOPRODUCERLISTENER_HEADER__
#define _CPDSEARCH_CPDJUMPSEARCHVIDEOPRODUCERLISTENER_HEADER__

#include <pathfinding-utils/StateVideoProducer.hpp>

#include "CpdJumpSearchImageProducerListener.hpp"


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
    class CpdJumpSearchVideoProducerListener: public CpdJumpSearchListener<G, V, STATE>, public StateVideoProducer<G, V, EORIGINAL, EPERTURBATED, MAPTYPE, IMAGETYPE> {
    public:
        using This = CpdJumpSearchVideoProducerListener<G, V, EORIGINAL, EPERTURBATED, STATE, MAPTYPE, IMAGETYPE>;
        using Super1 = CpdJumpSearchListener<G, V, STATE>;
        using Super2 = StateVideoProducer<G, V, EORIGINAL, EPERTURBATED, MAPTYPE, IMAGETYPE>;
    public:
        CpdJumpSearchVideoProducerListener(const IImmutableGraph<G,V,EORIGINAL>& originalGraph, const IImmutableGraph<G, V, EPERTURBATED>& perturbatedGraph, const MAPTYPE& map, const function_t<EPERTURBATED,cost_t>& costFunction): Super2{originalGraph, perturbatedGraph, map, costFunction} {
        }
        virtual ~CpdJumpSearchVideoProducerListener() {

        }
        CpdJumpSearchVideoProducerListener(const This& o) = default;
        CpdJumpSearchVideoProducerListener(This&& o) = default;
        This& operator = (const This& o) = default;
        This& operator = (This&& o) = default;
    public:
        virtual void onEarlyTerminationActivated(int iteration, const STATE& from, const STATE& goal) {

        }

        virtual void onUpperboundRevised(int iteration, const STATE& from, cost_t oldUpperbound, cost_t newUpperbound) {

        }

        virtual void onNodeExpanded(int iteration, const STATE& node) {
            this->updateNodeExpanded(node);
            IMAGETYPE* image = this->drawMap();
            boost::filesystem::path p{scout("./cpdjumpsearch_videoframe_iteration_", iteration, ".jpeg")};
            image->saveJPEG(p.native());
            this->addImage(p);
            delete image;
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
            this->updateSolution(goal, mapper);
            this->buildVideo(scout("./cpdjumpsearch_behaviour_", iteration, ".mp4"), "./test.wav");
        }
    public:
        virtual void cleanup() {
            Super2::cleanup();
        }
    };

}

#endif