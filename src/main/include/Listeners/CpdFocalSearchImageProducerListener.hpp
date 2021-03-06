#ifndef _CPDSEARCH_CPDFOCALSEARCHIMAGEPRODUCER_HEADER__
#define _CPDSEARCH_CPDFOCALSEARCHIMAGEPRODUCER_HEADER__

#include <pathfinding-utils/StateImageProducer.hpp>
#include <pathfinding-utils/StateTracker.hpp>

#include "CpdFocalSearchListener.hpp"

namespace pathfinding::search::listeners {

    template <typename G, typename V, typename EORIGINAL, typename EPERTURBATED, typename STATE, typename MAPTYPE, typename IMAGETYPE>
    class CpdFocalSearchImageProducerListener: public CpdFocalSearchListener<G, V, STATE>, public StateImageProducer<G, V, EORIGINAL, EPERTURBATED, MAPTYPE, IMAGETYPE> {
    public:
        using This = CpdFocalSearchImageProducerListener<G, V, EORIGINAL, EPERTURBATED, STATE, MAPTYPE, IMAGETYPE>;
        using Super1 = CpdFocalSearchListener<G, V, STATE>;
        using Super2 = StateImageProducer<G, V, EORIGINAL, EPERTURBATED, MAPTYPE, IMAGETYPE>;
    public:
        CpdFocalSearchImageProducerListener(const IImmutableGraph<G,V,EORIGINAL>& originalGraph, const IImmutableGraph<G, V, EPERTURBATED>& perturbatedGraph, const MAPTYPE& map, const function_t<EPERTURBATED,cost_t>& costFunction): Super2{originalGraph, perturbatedGraph, map, costFunction} {

        }
        virtual ~CpdFocalSearchImageProducerListener() {

        }
        CpdFocalSearchImageProducerListener(const This& o) = default;
        CpdFocalSearchImageProducerListener(This&& o) = default;
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
            boost::filesystem::path p{scout("./iteration_", iteration, ".bmp")};
            image->saveBMP(p.native());
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
        }
    public:
        virtual void cleanup() {
            Super2::cleanup();
        }
    };

}

#endif