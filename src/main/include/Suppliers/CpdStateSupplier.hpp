#ifndef _CPDSEARCH_CPDSTATESUPPLIER_HEADER__
#define _CPDSEARCH_CPDSTATESUPPLIER_HEADER__

#include <pathfinding-utils/AbstractSimpleWeightedDirectedGraphStateSupplier.hpp>
#include "CpdState.hpp"
#include "PerturbatedCost.hpp"

namespace pathfinding::search {

    /**
     * @brief A supplier which generate as many search nodes as there are vertices in the map in the form of CpdState
     * 
     * @tparam G payload type pf the map
     * @tparam V payload type of each vertex in the graph
     */
    template <typename G, typename V, typename E, typename REASON>
    class CpdStateSupplier: public AbstractSimpleWeightedDirectedGraphStateSupplier<CpdState<G, V, E, REASON>, G, V, E, REASON> {
        using State = CpdState<G, V, E, REASON>;
        using This = CpdStateSupplier<G, V, E, REASON>;
        using Super = AbstractSimpleWeightedDirectedGraphStateSupplier<State, G, V, E, REASON>;
    public:
        /**
         * @brief Construct a new Graph State Supplier object
         * 
         * @param graph the graph that will be blindly for each search node we're going to create
         */
        CpdStateSupplier(const IImmutableGraph<G, V, E>& graph): Super{graph} {
        }
        CpdStateSupplier(const This& other) = delete;
        CpdStateSupplier(This&& other): Super{::std::move(other)} {
        }
        virtual ~CpdStateSupplier() {
        }
        This& operator =(This&& other) {
            Super::operator=(::std::move(other));
            return *this;
        }
        This& operator =(const This& other) = delete;
    protected:
        virtual stateid_t generateStateId(nodeid_t location, const REASON& reason) {
            return location;
        }
        virtual State generateNewInstance(stateid_t id, nodeid_t location, const REASON& reason) {
            return State{id, this->graph, location, reason};
        }
    };

}

#endif