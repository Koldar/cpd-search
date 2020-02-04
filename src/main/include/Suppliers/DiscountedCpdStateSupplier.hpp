#ifndef _CPDSEARCH_DISCOUNTEDCPDSTATESUPPLIER_HEADER__
#define _CPDSEARCH_DISCOUNTEDCPDSTATESUPPLIER_HEADER__

#include <pathfinding-utils/AbstractSimpleWeightedDirectedGraphStateSupplier.hpp>
#include "DiscountedCpdState.hpp"
#include "PerturbatedCost.hpp"
#include "cpd_search_generated_e.hpp"

namespace pathfinding::search {

    /**
     * @brief A supplier which generate as many search nodes as there are vertices in the map in the form of CpdState
     * 
     * @tparam G payload type pf the map
     * @tparam V payload type of each vertex in the graph
     */
    template <typename G, typename V, typename E>
    class DiscountedCpdStateSupplier: public AbstractSimpleWeightedDirectedGraphStateSupplier<DiscountedCpdState<G, V, E, cpd_search_generated_e>, G, V, E, cpd_search_generated_e> {
        using This = DiscountedCpdStateSupplier<G, V, E>;
        using State = DiscountedCpdState<G, V, E, cpd_search_generated_e>;
        using Super = AbstractSimpleWeightedDirectedGraphStateSupplier<State, G, V, E, cpd_search_generated_e>;
    public:
        /**
         * @brief Construct a new Graph State Supplier object
         * 
         * @param graph the graph that will be blindly for each search node we're going to create
         */
        DiscountedCpdStateSupplier(const IImmutableGraph<G, V, E>& graph): Super{graph} {
        }
        DiscountedCpdStateSupplier(const This& other) = delete;
        DiscountedCpdStateSupplier(This&& other): Super{::std::move(other)} {
        }
        virtual ~DiscountedCpdStateSupplier() {
        }
        This& operator =(This&& other) {
            Super::operator=(::std::move(other));
            return *this;
        }
        This& operator =(const This& other) = delete;
    protected:
        virtual stateid_t generateStateId(nodeid_t location, const cpd_search_generated_e& reason) {
            return location;
        }
        virtual State generateNewInstance(stateid_t id, nodeid_t location, const cpd_search_generated_e& reason) {
            return State{id, this->graph, location, reason};
        }
    };

}

#endif