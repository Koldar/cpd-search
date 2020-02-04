#ifndef _CPD_SEARCH_CPD_FOCAL_SUPPLIER_HEADER__
#define _CPD_SEARCH_CPD_FOCAL_SUPPLIER_HEADER__

namespace pathfinding::search {

    /**
     * @brief A supplier which generate as many search nodes as there are vertices in the map
     * 
     * @tparam G payload type pf the map
     * @tparam V payload type of each vertex in the graph
     */
    template <typename G, typename V, typename E>
    class CpdFocalSupplier: public AbstractSimpleWeightedDirectedGraphStateSupplier<GraphFocalState<G, V, E>, G, V, E, cpd_search_generated_e> {
        using State = GraphFocalState<G, V, E>;
        using This = CpdFocalSupplier<G, V, E>;
        using Super = AbstractSimpleWeightedDirectedGraphStateSupplier<State, G, V, E, cpd_search_generated_e>;
    public:
        /**
         * @brief Construct a new Graph State Supplier object
         * 
         * @param graph the graph that will be blindly for each search node we're going to create
         */
        CpdFocalSupplier(const IImmutableGraph<G, V, E>& graph) : Super{graph} {

        }
        virtual ~CpdFocalSupplier() {

        }
        CpdFocalSupplier(const This& other) = delete;
        CpdFocalSupplier(This&& other) : Super{::std::move(other)} {
        }
        This& operator =(const This& other) = delete;
        This& operator =(This&& other) {
            Super::operator =(std::move(other));
            return *this;
        }
    protected:
        virtual stateid_t generateStateId(nodeid_t location, const cpd_search_generated_e& source) {
            return location;
        }

        virtual State generateNewInstance(stateid_t id, nodeid_t location, const cpd_search_generated_e& source) {
            return State{id, this->graph, location, source};
        }
    public:
        
    };

}

#endif