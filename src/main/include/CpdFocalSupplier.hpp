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
    class CpdFocalSupplier: public AbstractSimpleWeightedDirectedGraphStateSupplier<GraphFocalState<G, V, E>, G, V, E> {
        typedef CpdFocalSupplier<G, V, E> CpdFocalSupplierInstance;
    protected:
        virtual stateid_t generateStateId(nodeid_t location) {
            return location;
        }

        virtual GraphFocalState<G, V, E> generateNewInstance(stateid_t id, nodeid_t location) {
            return GraphFocalState<G, V, E>{id, this->graph, location};
        }
    public:
        /**
         * @brief Construct a new Graph State Supplier object
         * 
         * @param graph the graph that will be blindly for each search node we're going to create
         */
        CpdFocalSupplier(const IImmutableGraph<G, V, E>& graph): AbstractSimpleWeightedDirectedGraphStateSupplier<GraphFocalState<G, V, E>, G, V, E>{graph} {

        }
        CpdFocalSupplier(CpdFocalSupplierInstance&& other): AbstractSimpleWeightedDirectedGraphStateSupplier<GraphFocalState<G, V, E>, G, V, E>{::std::move(other)} {
            
        }
        ~CpdFocalSupplier() {

        }
        CpdFocalSupplierInstance& operator =(CpdFocalSupplierInstance&& other) {
            AbstractSimpleWeightedDirectedGraphStateSupplier<GraphFocalState<G, V, E>, G, V, E>::operator=(::std::move(other));
            return *this;
        }
    };

}

#endif