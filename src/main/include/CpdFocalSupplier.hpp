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
    class CpdFocalSupplier: public AbstractSimpleWeightedDirectedGraphStateSupplier<GraphFocalState<G, V, E>, G, V, E, generation_enum_t> {
        typedef CpdFocalSupplier<G, V, E> CpdFocalSupplierInstance;
        typedef AbstractSimpleWeightedDirectedGraphStateSupplier<GraphFocalState<G, V, E>, G, V, E, generation_enum_t> Super;
    protected:
        const std::shared_ptr<StaticPriorityQueue<GraphFocalState<G, V, PerturbatedCost>>>& openList;
        const std::shared_ptr<StaticPriorityQueue<GraphFocalState<G, V, PerturbatedCost>>>& focalList;
    public:
        /**
         * @brief Construct a new Graph State Supplier object
         * 
         * @param graph the graph that will be blindly for each search node we're going to create
         * @param openList openList CpdFocalSearch will use
         * @param focalList focalList CpdFocalSearch will use
         */
        CpdFocalSupplier(const IImmutableGraph<G, V, E>& graph, const std::shared_ptr<StaticPriorityQueue<GraphFocalState<G, V, PerturbatedCost>>>& openList, const std::shared_ptr<StaticPriorityQueue<GraphFocalState<G, V, PerturbatedCost>>>& focalList) : Super{graph}, openList{openList}, focalList{focalList} {

        }
        ~CpdFocalSupplier() {

        }
        CpdFocalSupplier(const CpdFocalSupplierInstance& other) = delete;
        CpdFocalSupplier(CpdFocalSupplierInstance&& other) : Super{::std::move(other)}, openList{other.openList}, focalList{other.focalList} {
        }
        CpdFocalSupplierInstance& operator =(const CpdFocalSupplierInstance& other) = delete;
        CpdFocalSupplierInstance& operator =(CpdFocalSupplierInstance&& other) {
            Super::operator =(std::move(other));
            this->openList = other.openList;
            this->focalList = other.focalList;
            return *this;
        }
    protected:
        virtual stateid_t generateStateId(nodeid_t location, generation_enum_t source) {
            return location;
        }

        virtual GraphFocalState<G, V, E> generateNewInstance(stateid_t id, nodeid_t location, generation_enum_t source) {
            info("open list is ", this->openList->size(),  "and focal list is ", this->focalList->size());
            return GraphFocalState<G, V, E>{id, this->graph, location, this->openList, this->focalList, source};
        }
    public:
        
    };

}

#endif